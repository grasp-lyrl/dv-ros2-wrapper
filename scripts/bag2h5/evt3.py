"""Vectorised decoders for `event_camera_msgs/msg/EventPacket` payloads.

`event_camera_codecs` ships C++ only (no Python bindings), and `event_camera_py`
is not generally installed, so the EVT3 stream is decoded here with numpy.

Reference: Prophesee EVT 3.0 -- a stream of little-endian uint16 words whose top
4 bits select the word type and whose low 12 bits carry the payload.
"""

from __future__ import annotations

import numpy as np

# Word types (word >> 12).
ADDR_Y = 0x0
ADDR_X = 0x2
VECT_BASE_X = 0x3
VECT_12 = 0x4
VECT_8 = 0x5
TIME_LOW = 0x6
TIME_HIGH = 0x8

_TIME_LOW_BITS = 12
_TIME_HIGH_WRAP = 1 << 12


def _ffill_source(flag: np.ndarray) -> np.ndarray:
    """Index of the most recent True in `flag` at or before each position."""
    idx = np.where(flag, np.arange(flag.size, dtype=np.int64), 0)
    return np.maximum.accumulate(idx)


class Evt3Decoder:
    """Stateful EVT3 decoder.

    The 12-bit TIME_HIGH counter wraps every 2**24 us and packets routinely
    straddle a wrap, so the rollover count has to persist across packets --
    decoding each packet independently produces ~16.7s backwards jumps.
    This mirrors `event_camera_codecs`' Evt3Decoder, including its
    `MIN_DISTANCE` hysteresis for high times that step back slightly without a
    real rollover, and like it ignores `time_base` (EVT3 carries sensor time).
    """

    #: a TIME_HIGH this much below the previous one is jitter, not a rollover
    MIN_DISTANCE = 10

    def __init__(self, detect_rollover: bool = True, time_source: str = 'full'):
        # `time_source='low'` builds time from TIME_LOW alone, unwrapping it
        # every 4096 us. Within a packet that is sufficient (packets are
        # milliseconds long and carry TIME_LOW every few events) and it is
        # immune to the TIME_HIGH preambles that converted bags prepend to each
        # packet, which otherwise stretch a packet across seconds.
        self.time_source = time_source
        self.detect_rollover = detect_rollover
        self.rollover = 0          # accumulated 2**24 us periods
        self.last_high: int | None = None
        self.time_low = 0
        self.last_y: int | None = None
        self._had_y = False

    def decode(self, raw) -> dict[str, np.ndarray]:
        words = np.frombuffer(np.asarray(raw, dtype=np.uint8).tobytes(), dtype='<u2')
        if words.size == 0:
            return _empty()

        kind = (words >> 12).astype(np.uint8)
        payload = (words & 0x0FFF).astype(np.int64)

        if self.time_source == 'low':
            t_words = self._time_from_low(kind, payload, words.size)
            return self._emit(kind, payload, t_words, words.size)

        # --- timestamps ---------------------------------------------------
        is_high = kind == TIME_HIGH
        time_high = np.empty(words.size, dtype=np.int64)
        if is_high.any():
            highs = payload[is_high]
            prev = np.empty(highs.size, dtype=np.int64)
            prev[0] = self.last_high if self.last_high is not None else highs[0]
            prev[1:] = highs[:-1]
            if self.detect_rollover:
                steps = ((highs < prev) & ((prev - highs) > self.MIN_DISTANCE)).astype(np.int64)
            else:
                steps = np.zeros(highs.size, dtype=np.int64)
            rollovers = self.rollover + np.cumsum(steps)
            unwrapped = (rollovers << 24) + (highs << _TIME_LOW_BITS)

            time_high[:] = 0
            time_high[is_high] = unwrapped
            src = _ffill_source(is_high)
            time_high = time_high[src]
            # words before the first TIME_HIGH keep the carried-over value
            first = int(np.argmax(is_high))
            time_high[:first] = (self.rollover << 24) + \
                ((self.last_high or 0) << _TIME_LOW_BITS)

            self.rollover = int(rollovers[-1])
            self.last_high = int(highs[-1])
        else:
            carried = (self.rollover << 24) + ((self.last_high or 0) << _TIME_LOW_BITS)
            time_high[:] = carried

        is_low = kind == TIME_LOW
        time_low = np.empty(words.size, dtype=np.int64)
        time_low[:] = 0
        time_low[is_low] = payload[is_low]
        if is_low.any():
            src = _ffill_source(is_low)
            time_low = time_low[src]
            first = int(np.argmax(is_low))
            time_low[:first] = self.time_low
            self.time_low = int(payload[is_low][-1])
        else:
            time_low[:] = self.time_low

        # TIME_HIGH already carries `high << 12`; OR is what the C++ does.
        t_words = time_high | time_low


        return self._emit(kind, payload, t_words, words.size)

    def _time_from_low(self, kind, payload, size) -> np.ndarray:
        """Microsecond time built from TIME_LOW alone, unwrapped every 4096 us."""
        is_low = kind == TIME_LOW
        time_low = np.zeros(size, dtype=np.int64)
        if is_low.any():
            lows = payload[is_low]
            wraps = np.zeros(lows.size, dtype=np.int64)
            wraps[1:] = (np.diff(lows) < 0).cumsum()
            time_low[is_low] = lows + (wraps << _TIME_LOW_BITS)
            time_low = time_low[_ffill_source(is_low)]
            time_low[:int(np.argmax(is_low))] = int(lows[0])
        return time_low

    def _emit(self, kind, payload, t_words, size) -> dict[str, np.ndarray]:

        is_y = kind == ADDR_Y
        y_words = np.empty(size, dtype=np.int64)
        y_words[:] = 0
        y_words[is_y] = payload[is_y] & 0x07FF
        if is_y.any():
            y_words = y_words[_ffill_source(is_y)]
            first = int(np.argmax(is_y))
            y_words[:first] = self.last_y if self.last_y is not None else 0
            self.last_y = int((payload[is_y] & 0x07FF)[-1])
        else:
            y_words[:] = self.last_y if self.last_y is not None else 0

        # An event is only meaningful once a y coordinate has been seen; before
        # the first ADDR_Y in a stream there is no row to attach it to.
        valid = np.ones(size, dtype=bool)
        if not self._had_y:
            if is_y.any():
                valid[:int(np.argmax(is_y))] = False
                self._had_y = True
            else:
                valid[:] = False

        # --- single-pixel events (ADDR_X) -----------------------------------
        single = (kind == ADDR_X) & valid
        single_idx = np.flatnonzero(single)
        sx = payload[single_idx] & 0x07FF
        sp = ((payload[single_idx] >> 11) & 1).astype(np.uint8)

        # --- vector events (VECT_BASE_X + VECT_12 / VECT_8) -----------------
        is_base = kind == VECT_BASE_X
        width = np.zeros(size, dtype=np.int64)
        width[kind == VECT_12] = 12
        width[kind == VECT_8] = 8
        vect_idx = np.flatnonzero((width > 0) & valid)

        if vect_idx.size and is_base.any():
            # Exclusive prefix sum of vector widths gives the offset from the
            # governing VECT_BASE_X to each following vector word.
            cw_excl = np.cumsum(width) - width
            base_src = _ffill_source(is_base)[vect_idx]

            base_x = np.zeros(size, dtype=np.int64)
            base_x[is_base] = payload[is_base] & 0x07FF
            base_p = np.zeros(size, dtype=np.uint8)
            base_p[is_base] = ((payload[is_base] >> 11) & 1).astype(np.uint8)

            start_x = base_x[base_src] + (cw_excl[vect_idx] - cw_excl[base_src])
            bits = (payload[vect_idx, None] >> np.arange(12, dtype=np.int64)) & 1
            rows, cols = np.nonzero(bits)

            vx = start_x[rows] + cols
            vp = base_p[base_src][rows]
            v_word = vect_idx[rows]
        else:
            vx = np.zeros(0, dtype=np.int64)
            vp = np.zeros(0, dtype=np.uint8)
            v_word = np.zeros(0, dtype=np.int64)
            cols = np.zeros(0, dtype=np.int64)

        # --- merge, restoring stream order ----------------------------------
        word_idx = np.concatenate([single_idx, v_word])
        sub_idx = np.concatenate([np.zeros(single_idx.size, dtype=np.int64), cols])
        order = np.lexsort((sub_idx, word_idx))

        x = np.concatenate([sx, vx])[order]
        p = np.concatenate([sp, vp])[order]
        idx = word_idx[order]

        return {
            'x': x.astype(np.uint16),
            'y': y_words[idx].astype(np.uint16),
            't': t_words[idx],
            'p': p.astype(np.uint8),
        }


def _empty() -> dict[str, np.ndarray]:
    return {
        'x': np.zeros(0, dtype=np.uint16),
        'y': np.zeros(0, dtype=np.uint16),
        't': np.zeros(0, dtype=np.int64),
        'p': np.zeros(0, dtype=np.uint8),
    }


DECODERS = {'evt3': Evt3Decoder}


def make_decoder(encoding: str, **kwargs):
    try:
        return DECODERS[encoding](**kwargs)
    except KeyError:
        raise NotImplementedError(
            f"EventPacket encoding {encoding!r} is not supported; only "
            f"{sorted(DECODERS)} are decoded natively. Install `event_camera_py` "
            f"and use its decoder for other encodings."
        ) from None
