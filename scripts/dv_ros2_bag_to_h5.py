#!/usr/bin/env python3
"""Convert a Neurofly ROS bag to HDF5.

Topics are matched to extractors by *message type*, so any bag works without
editing this script. Inspect a bag first:

    ./dv_ros2_bag_to_h5.py --input my_bag --list

Convert everything it understands, one group per topic:

    ./dv_ros2_bag_to_h5.py --input my_bag --output out.h5

Rename groups, or force an extractor, without touching the code:

    ./dv_ros2_bag_to_h5.py --input my_bag --output out.h5 \
        --map /neurofly1/control_odom=vicon_odom

or put the same thing in a YAML file and pass --config.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    import rosbags  # noqa: F401
except ImportError:
    raise SystemExit(
        'this tool needs the `rosbags` package:\n\n'
        '    pip install rosbags\n\n'
        'It replaces rosbag2_py/rclpy, so no ROS environment has to be sourced.'
    ) from None

from bag2h5 import extractors
from bag2h5.config import Selection, load_config, parse_map
from bag2h5.convert import convert, describe
from bag2h5.typestore import DEFAULT_STORE, available_stores


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--input', '-i', required=True,
                        help='bag directory or file (rosbag2 mcap/sqlite3, or ROS1 .bag)')
    parser.add_argument('--output', '-o', default='output.h5', help='output HDF5 file')
    parser.add_argument('--list', '-l', action='store_true',
                        help='show topics, types and the resolved plan, then exit')

    group = parser.add_argument_group('topic selection')
    group.add_argument('--map', '-m', action='append', default=[], metavar='TOPIC=GROUP[:EXTRACTOR]',
                       help='send a topic to a specific HDF5 group and/or extractor; repeatable')
    group.add_argument('--config', '-c', help='YAML file with the same mapping, plus defaults')
    group.add_argument('--include', action='append', default=[], metavar='REGEX',
                       help='only convert topics matching a regex; repeatable')
    group.add_argument('--exclude', action='append', default=[], metavar='REGEX',
                       help='never convert topics matching a regex; repeatable')
    group.add_argument('--no-auto', action='store_true',
                       help='convert only topics named by --map/--config')

    group = parser.add_argument_group('message definitions')
    group.add_argument('--typestore', default=DEFAULT_STORE, choices=available_stores(),
                       help=f'built-in message set to start from (default: {DEFAULT_STORE})')
    group.add_argument('--msg-path', action='append', default=[], metavar='DIR',
                       help='extra directory to scan for custom .msg files; repeatable. '
                            'AMENT_PREFIX_PATH is always scanned.')

    group = parser.add_argument_group('conversion')
    group.add_argument('--jobs', '-j', type=int, default=max(1, (os.cpu_count() or 2) - 2),
                       help='worker processes for deserialisation (1 = in-process)')
    group.add_argument('--chunk-size', type=int, default=512,
                       help='messages per work unit when --jobs > 1')
    group.add_argument('--time-source', choices=('header', 'bag'), default='header',
                       help='timestamp to record; falls back to bag time when a '
                            'message has no header (default: header)')
    group.add_argument('--compression', default='gzip', choices=('gzip', 'lzf', 'none'),
                       help='HDF5 compression (default: gzip)')
    group.add_argument('--compression-level', type=int, default=1,
                       help='gzip level 0-9 (default: 1)')
    group.add_argument('--event-time', choices=('packet', 'absolute', 'sensor'), default='packet',
                       help="for event_camera EventPacket, whose EVT3 stream carries a "
                            "free-running sensor clock: anchor every packet to its own "
                            "time_base (packet, default), anchor the whole stream once "
                            "(absolute), or keep the raw sensor clock (sensor)")
    group.add_argument('--keep-bgr', action='store_true',
                       help='store images in their original channel order instead of RGB')
    group.add_argument('--no-decode-compressed', action='store_true',
                       help='store CompressedImage blobs verbatim instead of decoding')
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    overrides: dict[str, dict] = {}
    defaults: dict = {}
    if args.config:
        overrides, defaults = load_config(args.config)
    for spec in args.map:
        topic, entry = parse_map(spec)
        overrides.setdefault(topic, {}).update(entry)

    time_source = defaults.get('time_source', args.time_source)
    selection = Selection(
        overrides=overrides,
        include=args.include or defaults.get('include') or [],
        exclude=args.exclude or defaults.get('exclude') or [],
        auto=not args.no_auto,
    )

    bag = Path(args.input).expanduser()
    if not bag.exists():
        raise SystemExit(f'no such bag: {bag}')

    if args.list:
        describe(bag, args.typestore, args.msg_path, selection)
        return 0

    convert(
        bag=bag,
        output=Path(args.output),
        store=args.typestore,
        msg_paths=args.msg_path,
        selection=selection,
        jobs=max(1, args.jobs),
        chunk_size=max(1, args.chunk_size),
        use_bag_time=(time_source == 'bag'),
        compression=None if args.compression == 'none' else args.compression,
        compression_level=args.compression_level,
        image_opts={
            'to_rgb': not args.keep_bgr,
            'decode': not args.no_decode_compressed,
            'event_time': args.event_time,
        },
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
