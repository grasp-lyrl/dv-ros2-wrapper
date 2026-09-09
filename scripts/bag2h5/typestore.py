"""Typestore construction for reading bags without a sourced ROS environment.

`rosbags` needs message definitions to deserialise CDR. Three sources are used,
in increasing priority:

1. a built-in distro typestore (all the common `sensor_msgs`/`nav_msgs`/... types),
2. `.msg` files discovered in ROS install/share trees, which is how custom
   packages like `dv_ros2_msgs` and `event_camera_msgs` are picked up,
3. definitions embedded in the bag itself (mcap always embeds them; rosbag2
   sqlite3 only from format version 8), which `AnyReader` registers on open.
"""

from __future__ import annotations

import os
from pathlib import Path

from rosbags.typesys import Stores, get_typestore
from rosbags.typesys.msg import get_types_from_msg

# Some packages install ROS2 definitions under `msg_ros2/` to sit alongside a
# ROS1 `msg/` directory (event_camera_msgs, dvs_msgs, prophesee_event_msgs).
_MSG_DIRS = ('msg', 'msg_ros2')

DEFAULT_STORE = 'ROS2_HUMBLE'


def available_stores() -> list[str]:
    return [s.name for s in Stores]


def _iter_msg_files(root: Path):
    """Yield (typename, path) for every .msg under a share/ tree or a msg dir."""
    for sub in _MSG_DIRS:
        # <prefix>/<pkg>/share/<pkg>/<sub>/*.msg  (an install space)
        yield from ((f.parent.parent.name, f) for f in root.glob(f'*/share/*/{sub}/*.msg'))
        # <prefix>/<pkg>/<sub>/*.msg  (a single package prefix, or a source tree)
        yield from ((f.parent.parent.name, f) for f in root.glob(f'*/{sub}/*.msg'))
        # <pkg>/<sub>/*.msg  (pointed directly at a package)
        yield from ((f.parent.parent.name, f) for f in root.glob(f'{sub}/*.msg'))


def _workspace_roots(hints: list[Path]) -> list[Path]:
    """Find colcon install/ spaces by walking up from the given paths.

    This is what makes the tool work when the workspace holding the custom
    messages has not been sourced -- the common case when converting a bag
    from a checkout rather than from a running robot.
    """
    found: list[Path] = []
    for hint in hints:
        current = hint.resolve() if hint.exists() else hint
        for parent in [current, *current.parents][:8]:
            for name in ('install', 'devel'):
                cand = parent / name
                if cand.is_dir() and cand not in found:
                    found.append(cand)
    return found


def discover_msg_roots(extra: list[str] | None = None,
                       hints: list[Path] | None = None) -> list[Path]:
    """Roots to scan for .msg files, most specific first.

    Explicit --msg-path entries, then colcon workspaces inferred from the
    script and bag locations, then the sourced environment.
    """
    roots: list[Path] = []

    def add(path: Path) -> None:
        if path.is_dir() and path not in roots:
            roots.append(path)

    for item in extra or []:
        add(Path(item).expanduser())

    for root in _workspace_roots([Path(__file__).parent, *(hints or [])]):
        add(root)

    for var in ('AMENT_PREFIX_PATH', 'COLCON_PREFIX_PATH'):
        for item in os.environ.get(var, '').split(os.pathsep):
            if not item:
                continue
            p = Path(item)
            # AMENT_PREFIX_PATH entries are per-package prefixes; scanning the
            # parent picks up every package in the same install space at once.
            add(p)
            add(p.parent)
    return roots


def build_typestore(store: str = DEFAULT_STORE, msg_paths: list[str] | None = None,
                    verbose: bool = False, hints: list[Path] | None = None):
    """Return a typestore seeded with a distro store plus discovered custom types."""
    try:
        typestore = get_typestore(Stores[store])
    except KeyError:
        raise SystemExit(
            f"unknown typestore {store!r}; choose one of {', '.join(available_stores())}"
        ) from None

    discovered: dict[str, object] = {}
    seen: set[str] = set()
    for root in discover_msg_roots(msg_paths, hints):
        for pkg, path in _iter_msg_files(root):
            name = f'{pkg}/msg/{path.stem}'
            if name in typestore.types or name in seen:
                continue
            seen.add(name)
            try:
                text = path.read_text()
            except OSError:
                continue  # dangling symlink in a stale install space
            try:
                discovered.update(get_types_from_msg(text, name))
            except Exception as exc:  # a malformed or ROS1-only definition
                if verbose:
                    print(f'  skipped {name}: {exc}')

    if discovered:
        # Types already present in the distro store must not be re-registered.
        fresh = {k: v for k, v in discovered.items() if k not in typestore.types}
        if fresh:
            typestore.register(fresh)
        if verbose:
            print(f'registered {len(fresh)} custom message types')
    return typestore
