"""Message-type -> HDF5 column extractors.

Each extractor turns one deserialised ROS message into a `Batch` of equal-length
columns that get appended to a group. Adding support for a new message type
means adding one class here and listing its `MSGTYPES` -- no changes anywhere
else in the pipeline, and no topic names baked into the code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from .evt3 import make_decoder

US = 1_000_000


@dataclass
class VLen:
    """A ragged column: one variable-length array per row."""

    rows: list[np.ndarray]
    dtype: np.dtype


@dataclass
class Batch:
    columns: dict[str, Any] = field(default_factory=dict)
    attrs: dict[str, Any] = field(default_factory=dict)

    def __len__(self) -> int:
        for value in self.columns.values():
            return len(value.rows) if isinstance(value, VLen) else len(value)
        return 0


def stamp_us(header) -> int:
    return int(header.stamp.sec) * US + int(header.stamp.nanosec) // 1000


class Extractor:
    """Base class. `MSGTYPES` binds it to ROS message types."""

    NAME: str = ''
    MSGTYPES: tuple[str, ...] = ()
    #: suggested prefix when auto-naming a group for this kind of data
    GROUP_PREFIX: str = ''
    #: carries state between messages, so it must see one topic in order and
    #: cannot be farmed out to a pool of workers
    STATEFUL: bool = False

    def __call__(self, msg, bag_time_us: int, use_bag_time: bool) -> Batch | None:
        raise NotImplementedError

    @staticmethod
    def _time(msg, bag_time_us: int, use_bag_time: bool) -> int:
        header = getattr(msg, 'header', None)
        if use_bag_time or header is None:
            return bag_time_us
        return stamp_us(header)


class EventArrayExtractor(Extractor):
    """dv_ros2 / dvs_msgs / prophesee style `Event[] events` arrays."""

    NAME = 'events'
    MSGTYPES = (
        'dv_ros2_msgs/msg/EventArray',
        'dvs_msgs/msg/EventArray',
        'prophesee_event_msgs/msg/EventArray',
    )

    def __call__(self, msg, bag_time_us, use_bag_time):
        events = msg.events
        if not len(events):
            return None
        # One pass, building python lists, then a single numpy conversion.
        xs = np.fromiter((e.x for e in events), dtype=np.uint16, count=len(events))
        ys = np.fromiter((e.y for e in events), dtype=np.uint16, count=len(events))
        ts = np.fromiter(
            (e.ts.sec * US + e.ts.nanosec // 1000 for e in events),
            dtype=np.int64, count=len(events),
        )
        ps = np.fromiter((bool(e.polarity) for e in events), dtype=np.uint8, count=len(events))
        return Batch(
            columns={'x': xs, 'y': ys, 't': ts, 'p': ps},
            attrs={'height': int(msg.height), 'width': int(msg.width)},
        )


class EventPacketExtractor(Extractor):
    """event_camera_msgs `EventPacket`, whose payload is an encoded byte stream.

    EVT3 carries a free-running sensor clock, not wall time, so the packet's
    `time_base` has to be brought in. Three modes:

    ``packet`` (default)
        Decode each packet independently and anchor its first event to that
        packet's `time_base`, exactly the contract the message documents
        ("event ros time = time_base + decoded event_time"). Intervals within a
        packet come from the sensor; across packets they come from `time_base`.
        This is the only mode that survives streams whose packets are not one
        contiguous sensor-clock sequence -- e.g. bags whose packets each open
        with a time preamble that steps the TIME_HIGH counter backwards, which
        a continuous decoder misreads as a 2**24 us rollover.

    ``absolute``
        One decoder across the whole stream, anchored once to the first
        packet's `time_base`. Use when the sensor clock is known to be
        contiguous and you want its timing end to end.

    ``sensor``
        The raw sensor clock, no anchoring.
    """

    NAME = 'event_packet'
    MSGTYPES = ('event_camera_msgs/msg/EventPacket',)
    STATEFUL = True

    MODES = ('packet', 'absolute', 'sensor')

    def __init__(self, event_time: str = 'packet'):
        if event_time not in self.MODES:
            raise SystemExit(f'--event-time must be one of {", ".join(self.MODES)}')
        self.mode = event_time
        self._decoder = None
        self._encoding = None
        self._offset = None
        #: events whose timestamp was pulled forward to keep the stream ordered
        self.adjusted = 0
        self._last_t = None

    def _get_decoder(self, encoding: str):
        if self.mode == 'packet' or self._decoder is None or encoding != self._encoding:
            packet_mode = self.mode == 'packet'
            self._decoder = make_decoder(
                encoding,
                detect_rollover=not packet_mode,
                time_source='low' if packet_mode else 'full',
            )
            self._encoding = encoding
        return self._decoder

    def __call__(self, msg, bag_time_us, use_bag_time):
        encoding = str(msg.encoding)
        decoded = self._get_decoder(encoding).decode(msg.events)
        if decoded['t'].size == 0:
            return None

        if self.mode != 'sensor':
            anchor = bag_time_us if use_bag_time else int(msg.time_base) // 1000
            if self.mode == 'packet':
                offset = anchor - int(decoded['t'][0])
            else:
                if self._offset is None:
                    self._offset = anchor - int(decoded['t'][0])
                offset = self._offset
            decoded['t'] = decoded['t'] + offset

        if self.mode == 'packet':
            # Each packet is anchored to its own time_base, which has ~1ms
            # resolution, so a long packet can overrun the next packet's anchor.
            # Events arrive in stream order, so pulling stragglers forward to
            # the running maximum keeps the dataset sorted.
            raw_t = decoded['t']
            monotonic = np.maximum.accumulate(raw_t)
            if self._last_t is not None:
                monotonic = np.maximum(monotonic, self._last_t)
            self.adjusted += int((monotonic != raw_t).sum())
            decoded['t'] = monotonic
            self._last_t = int(monotonic[-1])

        return Batch(
            columns=decoded,
            attrs={'height': int(msg.height), 'width': int(msg.width),
                   'encoding': encoding},
        )


class ImuExtractor(Extractor):
    NAME = 'imu'
    MSGTYPES = ('sensor_msgs/msg/Imu',)

    def __call__(self, msg, bag_time_us, use_bag_time):
        acc, gyro, ori = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        cols = {
            't': np.int64(self._time(msg, bag_time_us, use_bag_time)),
            'ax': acc.x, 'ay': acc.y, 'az': acc.z,
            'wx': gyro.x, 'wy': gyro.y, 'wz': gyro.z,
            'qx': ori.x, 'qy': ori.y, 'qz': ori.z, 'qw': ori.w,
        }
        return Batch(columns={k: np.atleast_1d(v) for k, v in cols.items()})


class OdometryExtractor(Extractor):
    NAME = 'odom'
    MSGTYPES = ('nav_msgs/msg/Odometry',)

    def __call__(self, msg, bag_time_us, use_bag_time):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        twist = msg.twist.twist
        cols = {
            't': np.int64(self._time(msg, bag_time_us, use_bag_time)),
            'px': pos.x, 'py': pos.y, 'pz': pos.z,
            'qx': ori.x, 'qy': ori.y, 'qz': ori.z, 'qw': ori.w,
            'vx': twist.linear.x, 'vy': twist.linear.y, 'vz': twist.linear.z,
            'avx': twist.angular.x, 'avy': twist.angular.y, 'avz': twist.angular.z,
        }
        return Batch(columns={k: np.atleast_1d(v) for k, v in cols.items()})


class PoseExtractor(Extractor):
    NAME = 'pose'
    MSGTYPES = (
        'geometry_msgs/msg/PoseStamped',
        'geometry_msgs/msg/PoseWithCovarianceStamped',
        'geometry_msgs/msg/TransformStamped',
    )

    def __call__(self, msg, bag_time_us, use_bag_time):
        if hasattr(msg, 'transform'):
            pos, ori = msg.transform.translation, msg.transform.rotation
        elif hasattr(msg.pose, 'pose'):
            pos, ori = msg.pose.pose.position, msg.pose.pose.orientation
        else:
            pos, ori = msg.pose.position, msg.pose.orientation
        cols = {
            't': np.int64(self._time(msg, bag_time_us, use_bag_time)),
            'px': pos.x, 'py': pos.y, 'pz': pos.z,
            'qx': ori.x, 'qy': ori.y, 'qz': ori.z, 'qw': ori.w,
        }
        return Batch(columns={k: np.atleast_1d(v) for k, v in cols.items()})


class RangeExtractor(Extractor):
    """sensor_msgs/Range -- rangefinders, sonar, mavros distance sensors."""

    NAME = 'range'
    MSGTYPES = ('sensor_msgs/msg/Range',)

    def __call__(self, msg, bag_time_us, use_bag_time):
        cols = {
            't': np.int64(self._time(msg, bag_time_us, use_bag_time)),
            'range': msg.range,
            'min_range': msg.min_range,
            'max_range': msg.max_range,
            'field_of_view': msg.field_of_view,
            'radiation_type': np.uint8(msg.radiation_type),
        }
        return Batch(columns={k: np.atleast_1d(v) for k, v in cols.items()})


class NavSatFixExtractor(Extractor):
    NAME = 'navsatfix'
    MSGTYPES = ('sensor_msgs/msg/NavSatFix',)

    def __call__(self, msg, bag_time_us, use_bag_time):
        cols = {
            't': np.int64(self._time(msg, bag_time_us, use_bag_time)),
            'latitude': msg.latitude,
            'longitude': msg.longitude,
            'altitude': msg.altitude,
            'status': np.int8(msg.status.status),
            'service': np.uint16(msg.status.service),
        }
        return Batch(columns={k: np.atleast_1d(v) for k, v in cols.items()})


class MagneticFieldExtractor(Extractor):
    NAME = 'magnetic_field'
    MSGTYPES = ('sensor_msgs/msg/MagneticField',)

    def __call__(self, msg, bag_time_us, use_bag_time):
        field = msg.magnetic_field
        cols = {
            't': np.int64(self._time(msg, bag_time_us, use_bag_time)),
            'mx': field.x, 'my': field.y, 'mz': field.z,
        }
        return Batch(columns={k: np.atleast_1d(v) for k, v in cols.items()})


#: single-value sensor_msgs types, mapped to the field holding the value
_SENSOR_SCALARS = {
    'sensor_msgs/msg/Temperature': 'temperature',
    'sensor_msgs/msg/FluidPressure': 'fluid_pressure',
    'sensor_msgs/msg/RelativeHumidity': 'relative_humidity',
    'sensor_msgs/msg/Illuminance': 'illuminance',
}


class SensorScalarExtractor(Extractor):
    """Stamped single-value sensor_msgs: temperature, pressure, humidity, ..."""

    NAME = 'sensor_scalar'
    MSGTYPES = tuple(_SENSOR_SCALARS)

    def __call__(self, msg, bag_time_us, use_bag_time):
        field = _SENSOR_SCALARS[msg.__msgtype__]
        return Batch(columns={
            't': np.atleast_1d(np.int64(self._time(msg, bag_time_us, use_bag_time))),
            'value': np.atleast_1d(np.float64(getattr(msg, field))),
            'variance': np.atleast_1d(np.float64(getattr(msg, 'variance', 0.0))),
        })


# ROS image encoding -> (numpy dtype, channel count).
_ENCODINGS = {
    'mono8': (np.uint8, 1), '8UC1': (np.uint8, 1),
    'mono16': (np.uint16, 1), '16UC1': (np.uint16, 1),
    'rgb8': (np.uint8, 3), 'bgr8': (np.uint8, 3), '8UC3': (np.uint8, 3),
    'rgba8': (np.uint8, 4), 'bgra8': (np.uint8, 4), '8UC4': (np.uint8, 4),
    'rgb16': (np.uint16, 3), 'bgr16': (np.uint16, 3),
    'rgba16': (np.uint16, 4), 'bgra16': (np.uint16, 4),
    '32FC1': (np.float32, 1), '32FC3': (np.float32, 3),
    '64FC1': (np.float64, 1),
}


def decode_image(msg, to_rgb: bool = True) -> np.ndarray:
    """sensor_msgs/Image -> (H, W, C) array, optionally BGR(A)-swapped to RGB(A)."""
    encoding = str(msg.encoding)
    dtype, channels = _ENCODINGS.get(encoding, (None, None))
    buf = np.frombuffer(np.asarray(msg.data, dtype=np.uint8).tobytes(), dtype=np.uint8)

    if dtype is None:
        # Unknown encoding: infer channels from the buffer size, assume 8-bit.
        dtype = np.uint8
        pixels = int(msg.height) * int(msg.width)
        channels = max(1, buf.size // pixels) if pixels else 1

    arr = buf.view(dtype)
    height, width = int(msg.height), int(msg.width)
    # `step` may pad rows; trim to the declared width.
    step = int(getattr(msg, 'step', 0)) or width * channels * np.dtype(dtype).itemsize
    row_items = step // np.dtype(dtype).itemsize
    if row_items * height == arr.size and row_items != width * channels:
        arr = arr.reshape(height, row_items)[:, :width * channels]
    arr = arr.reshape(height, width, channels)

    if to_rgb and channels >= 3:
        lowered = encoding.lower()
        if lowered.startswith('bgra'):
            arr = arr[..., [2, 1, 0, 3]]
        elif lowered.startswith('bgr'):
            arr = arr[..., ::-1]
    return np.ascontiguousarray(arr)


class ImageExtractor(Extractor):
    NAME = 'image'
    MSGTYPES = ('sensor_msgs/msg/Image',)
    GROUP_PREFIX = 'images'

    def __init__(self, to_rgb: bool = True):
        self.to_rgb = to_rgb

    def __call__(self, msg, bag_time_us, use_bag_time):
        img = decode_image(msg, self.to_rgb)
        return Batch(
            columns={
                't': np.atleast_1d(np.int64(self._time(msg, bag_time_us, use_bag_time))),
                'images': img[None, ...],
            },
            attrs={'height': int(msg.height), 'width': int(msg.width),
                   'encoding': str(msg.encoding)},
        )


class CompressedImageExtractor(Extractor):
    """Stores the compressed blob as-is unless OpenCV is available to decode it."""

    NAME = 'compressed_image'
    MSGTYPES = ('sensor_msgs/msg/CompressedImage',)
    GROUP_PREFIX = 'images'

    def __init__(self, decode: bool = True):
        self.decode = decode
        self._cv2 = None
        if decode:
            try:
                import cv2
                self._cv2 = cv2
            except ImportError:
                self._cv2 = None

    def __call__(self, msg, bag_time_us, use_bag_time):
        blob = np.frombuffer(np.asarray(msg.data, dtype=np.uint8).tobytes(), dtype=np.uint8)
        t = np.atleast_1d(np.int64(self._time(msg, bag_time_us, use_bag_time)))
        if self._cv2 is not None:
            img = self._cv2.imdecode(blob, self._cv2.IMREAD_UNCHANGED)
            if img is not None:
                if img.ndim == 2:
                    img = img[..., None]
                elif img.shape[2] >= 3:  # OpenCV decodes to BGR(A)
                    img = img[..., [2, 1, 0, 3]] if img.shape[2] == 4 else img[..., ::-1]
                return Batch(columns={'t': t, 'images': np.ascontiguousarray(img)[None, ...]})
        return Batch(columns={'t': t, 'blob': VLen([blob], np.dtype(np.uint8)),
                              'format': np.array([str(msg.format)], dtype=object)})


_MULTIARRAY_DTYPES = {
    'Float32MultiArray': np.float32, 'Float64MultiArray': np.float64,
    'Int8MultiArray': np.int8, 'Int16MultiArray': np.int16,
    'Int32MultiArray': np.int32, 'Int64MultiArray': np.int64,
    'UInt8MultiArray': np.uint8, 'UInt16MultiArray': np.uint16,
    'UInt32MultiArray': np.uint32, 'UInt64MultiArray': np.uint64,
}


class MultiArrayExtractor(Extractor):
    """std_msgs `*MultiArray` -> ragged `data` + per-row `shape`."""

    NAME = 'multiarray'
    MSGTYPES = tuple(f'std_msgs/msg/{n}' for n in _MULTIARRAY_DTYPES)

    def __call__(self, msg, bag_time_us, use_bag_time):
        dtype = _MULTIARRAY_DTYPES[msg.__msgtype__.rsplit('/', 1)[-1]]
        data = np.asarray(msg.data, dtype=dtype).ravel()
        shape = np.array([int(d.size) for d in msg.layout.dim], dtype=np.int32)
        if shape.size == 0:
            shape = np.array([data.size], dtype=np.int32)
        return Batch(columns={
            # MultiArray has no header, so the bag receive time is the only clock.
            't': np.atleast_1d(np.int64(bag_time_us)),
            'data': VLen([data], np.dtype(dtype)),
            'shape': VLen([shape], np.dtype(np.int32)),
        })


_SCALAR_DTYPES = {
    'Bool': np.uint8, 'Float32': np.float32, 'Float64': np.float64,
    'Int8': np.int8, 'Int16': np.int16, 'Int32': np.int32, 'Int64': np.int64,
    'UInt8': np.uint8, 'UInt16': np.uint16, 'UInt32': np.uint32, 'UInt64': np.uint64,
}


class ScalarExtractor(Extractor):
    NAME = 'scalar'
    MSGTYPES = tuple(f'std_msgs/msg/{n}' for n in _SCALAR_DTYPES)

    def __call__(self, msg, bag_time_us, use_bag_time):
        dtype = _SCALAR_DTYPES[msg.__msgtype__.rsplit('/', 1)[-1]]
        return Batch(columns={
            't': np.atleast_1d(np.int64(bag_time_us)),
            'value': np.atleast_1d(np.asarray(msg.data, dtype=dtype)),
        })


_CLASSES: tuple[type[Extractor], ...] = (
    EventArrayExtractor, EventPacketExtractor, ImuExtractor, OdometryExtractor,
    PoseExtractor, RangeExtractor, NavSatFixExtractor, MagneticFieldExtractor,
    SensorScalarExtractor, ImageExtractor, CompressedImageExtractor,
    MultiArrayExtractor, ScalarExtractor,
)

#: extractor name -> class
BY_NAME: dict[str, type[Extractor]] = {c.NAME: c for c in _CLASSES}
#: ROS message type -> extractor name, used for automatic topic discovery
BY_MSGTYPE: dict[str, str] = {t: c.NAME for c in _CLASSES for t in c.MSGTYPES}


def make(name: str, **kwargs) -> Extractor:
    try:
        cls = BY_NAME[name]
    except KeyError:
        raise SystemExit(
            f"unknown extractor {name!r}; available: {', '.join(sorted(BY_NAME))}"
        ) from None
    accepted = getattr(cls.__init__, '__code__', None)
    if accepted is None or accepted.co_argcount <= 1:
        return cls()
    names = accepted.co_varnames[1:accepted.co_argcount]
    return cls(**{k: v for k, v in kwargs.items() if k in names})
