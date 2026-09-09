#!/usr/bin/env python3
"""Play back events (and IMU, if present) from an HDF5 file written by
dv_ros2_bag_to_h5.py.

Groups are discovered by their contents rather than by name, so a file whose
groups are named after topics (`neurofly1/events`) works the same as one using
explicit names from a --map/--config run (`events`).

    ./visualize_events_h5.py out.h5 --list
    ./visualize_events_h5.py out.h5 --dt-ms 20
    ./visualize_events_h5.py out.h5 --events-group neurofly1/events --start 12.5

Events are streamed in blocks, so a file with hundreds of millions of events
plays without loading them all: the previous version read every column up
front, which is ~6 GB for a 480M-event recording.

Keys: space pauses, left/right seek by 1s, q or Esc quits.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
import h5py

US = 1_000_000.0

EVENT_COLUMNS = ('x', 'y', 't', 'p')
IMU_COLUMNS = ('t', 'ax', 'ay', 'az', 'wx', 'wy', 'wz')


# --- file layout discovery -------------------------------------------------

def _iter_groups(f: h5py.File):
    out = []
    f.visititems(lambda name, obj: out.append((name, obj))
                 if isinstance(obj, h5py.Group) else None)
    return out


def _has_columns(grp, columns) -> bool:
    return all(c in grp and isinstance(grp[c], h5py.Dataset) for c in columns)


def find_groups(f: h5py.File, columns) -> list[str]:
    return [name for name, grp in _iter_groups(f) if _has_columns(grp, columns)]


def pick_group(f: h5py.File, columns, requested: str | None, kind: str) -> str | None:
    """Resolve a group by explicit name, else by unique content match."""
    if requested:
        if requested not in f:
            raise SystemExit(f'no group {requested!r} in file')
        if not _has_columns(f[requested], columns):
            raise SystemExit(
                f'group {requested!r} lacks {kind} columns {", ".join(columns)}'
            )
        return requested

    found = find_groups(f, columns)
    if not found:
        return None
    if len(found) > 1:
        # Prefer the largest, but say so -- silently picking would be worse.
        found.sort(key=lambda n: -f[n][columns[0]].shape[0])
        print(f'note: several {kind} groups ({", ".join(found)}); '
              f'using {found[0]}. Override with --{kind}-group.')
    return found[0]


def describe(path: str) -> None:
    with h5py.File(path, 'r') as f:
        print(f'{path}')
        if f.attrs:
            print('  attrs:', dict(f.attrs))
        events = set(find_groups(f, EVENT_COLUMNS))
        imus = set(find_groups(f, IMU_COLUMNS))
        for name, grp in _iter_groups(f):
            cols = sorted(k for k in grp if isinstance(grp[k], h5py.Dataset))
            if not cols:
                continue
            rows = grp[cols[0]].shape[0]
            kind = 'events' if name in events else 'imu' if name in imus else ''
            tag = f'  [{kind}]' if kind else ''
            print(f'  {name}{tag}')
            print(f'      {rows:,} rows  columns: {", ".join(cols)}')
            if grp.attrs:
                print(f'      attrs: {dict(grp.attrs)}')


# --- streaming event access ------------------------------------------------

def searchsorted_dataset(dset, value) -> int:
    """Index of the first element >= value, without reading the whole dataset."""
    lo, hi = 0, dset.shape[0]
    while lo < hi:
        mid = (lo + hi) // 2
        if dset[mid] < value:
            lo = mid + 1
        else:
            hi = mid
    return lo


class EventStream:
    """Sequential reader over an events group, buffered in fixed blocks."""

    def __init__(self, grp, block: int = 1 << 22):
        self.x, self.y, self.t, self.p = (grp[c] for c in EVENT_COLUMNS)
        self.n = self.t.shape[0]
        self.block = block
        self.pos = 0          # next unread global index
        self._buf = None      # (x, y, t, p) for [buf_lo, buf_hi)
        self._buf_lo = 0
        self._buf_hi = 0

    def seek(self, t_us: float) -> None:
        self.pos = searchsorted_dataset(self.t, t_us)
        self._buf = None
        self._buf_lo = self._buf_hi = 0

    def _ensure(self, index: int) -> None:
        if self._buf is not None and self._buf_lo <= index < self._buf_hi:
            return
        lo = index
        hi = min(lo + self.block, self.n)
        self._buf = (self.x[lo:hi], self.y[lo:hi], self.t[lo:hi], self.p[lo:hi])
        self._buf_lo, self._buf_hi = lo, hi

    def until(self, t_end_us: float):
        """Consume and return every event with t < t_end_us as (x, y, p)."""
        xs, ys, ps = [], [], []
        while self.pos < self.n:
            self._ensure(self.pos)
            bx, by, bt, bp = self._buf
            off = self.pos - self._buf_lo
            # Search the whole (sorted) buffer rather than a slice of it;
            # bt[off:] would copy up to the block size on every frame.
            take = max(0, int(np.searchsorted(bt, t_end_us, side='left')) - off)
            if take:
                end = off + take
                xs.append(bx[off:end])
                ys.append(by[off:end])
                ps.append(bp[off:end])
                self.pos += take
            if take == 0 or self.pos < self._buf_hi:
                break  # reached t_end inside this block
        if not xs:
            empty = np.empty(0, dtype=np.int32)
            return empty, empty, empty
        return np.concatenate(xs), np.concatenate(ys), np.concatenate(ps)


# --- rendering -------------------------------------------------------------

def draw_events_frame(xs, ys, ps, height, width):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    if len(xs) == 0:
        return img
    xs = np.clip(np.asarray(xs, dtype=np.int32), 0, width - 1)
    ys = np.clip(np.asarray(ys, dtype=np.int32), 0, height - 1)
    ps = np.asarray(ps)
    pos = ps > 0
    # BGR: positive red, negative green
    img[ys[pos], xs[pos]] = (0, 0, 255)
    img[ys[~pos], xs[~pos]] = (0, 255, 0)
    return img


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('h5file')
    parser.add_argument('--list', '-l', action='store_true',
                        help='show the groups in the file and exit')
    parser.add_argument('--dt-ms', type=float, default=100.0,
                        help='accumulation window per frame in ms (default: 100)')
    parser.add_argument('--events-group', help='group holding x/y/t/p')
    parser.add_argument('--imu-group', help='group holding t/ax/wx/...')
    parser.add_argument('--no-imu', action='store_true', help='skip the IMU plot')
    parser.add_argument('--start', type=float, default=0.0,
                        help='start this many seconds into the recording')
    parser.add_argument('--speed', type=float, default=1.0,
                        help='playback rate; 0 plays as fast as possible (default: 1)')
    parser.add_argument('--duration', type=float,
                        help='stop after this many seconds of recording time')
    args = parser.parse_args()

    if args.list:
        describe(args.h5file)
        return 0

    import cv2  # imported late so --list works without a display stack

    with h5py.File(args.h5file, 'r') as f:
        ev_name = pick_group(f, EVENT_COLUMNS, args.events_group, 'events')
        if ev_name is None:
            raise SystemExit(
                'no group with x/y/t/p found. Run with --list to see the layout.')
        grp = f[ev_name]
        height = int(grp.attrs.get('height', 480))
        width = int(grp.attrs.get('width', 640))

        stream = EventStream(grp)
        if stream.n == 0:
            raise SystemExit(f'{ev_name} contains no events')
        t0 = float(grp['t'][0])
        t_file_end = float(grp['t'][stream.n - 1])
        t_end = t_file_end
        if args.duration:
            t_end = min(t_end, t0 + max(args.start, 0.0) * US + args.duration * US)

        imu = None
        imu_disabled = args.no_imu
        if not args.no_imu:
            imu_name = pick_group(f, IMU_COLUMNS, args.imu_group, 'imu')
            if imu_name is not None:
                g = f[imu_name]
                # IMU is small enough to hold; subsample if it ever is not.
                step = max(1, g['t'].shape[0] // 200_000)
                imu = {c: g[c][::step] for c in IMU_COLUMNS}
                imu['name'] = imu_name

        print(f'{ev_name}: {stream.n:,} events, {width}x{height}')
        print(f'  recording spans {(t_file_end - t0) / US:.3f}s '
              f'(t0 = {t0 / US:.6f})')
        play_from = max(args.start, 0.0)
        if play_from or t_end < t_file_end:
            print(f'  playing {play_from:.3f}s .. {(t_end - t0) / US:.3f}s')
        if imu is not None:
            print(f'{imu["name"]}: {imu["t"].size:,} samples')
        elif imu_disabled:
            print('IMU plot disabled (--no-imu)')
        else:
            print('no IMU group found')

        fig = vlines = None
        if imu is not None:
            fig, vlines = _setup_imu_plot(imu, t0)

        dt = args.dt_ms * 1000.0
        cur_t = t0 + max(args.start, 0.0) * US
        if cur_t > t0:
            stream.seek(cur_t)
        start_t = cur_t
        paused = False
        frame = np.zeros((height, width, 3), dtype=np.uint8)
        xs = np.empty(0, dtype=np.int32)
        shown = 0

        import time
        cv2.namedWindow('events', cv2.WINDOW_NORMAL)
        wall0 = time.time()
        while cur_t < t_end:
            if not paused:
                xs, ys, ps = stream.until(cur_t + dt)
                frame = draw_events_frame(xs, ys, ps, height, width)
                shown += len(xs)
                cur_t += dt
                if vlines is not None:
                    _move_marker(fig, vlines, (cur_t - t0) / US)
                if args.speed > 0:
                    target = wall0 + (cur_t - start_t) / US / args.speed
                    lag = target - time.time()
                    if lag > 0:
                        time.sleep(lag)

            label = f'{(cur_t - t0) / US:8.3f}s  {len(xs) if not paused else 0:>7} ev'
            view = frame.copy()
            cv2.putText(view, label + ('  [paused]' if paused else ''), (6, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            cv2.imshow('events', view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                paused = not paused
                wall0 = time.time() - (cur_t - start_t) / US / max(args.speed, 1e-9)
            elif key in (ord('q'), 27):
                break
            elif key in (81, ord(',')):      # left / seek back
                cur_t = max(t0, cur_t - US)
                stream.seek(cur_t)
                wall0 = time.time() - (cur_t - start_t) / US / max(args.speed, 1e-9)
            elif key in (83, ord('.')):      # right / seek forward
                cur_t = min(t_end, cur_t + US)
                stream.seek(cur_t)
                wall0 = time.time() - (cur_t - start_t) / US / max(args.speed, 1e-9)

        cv2.destroyAllWindows()

    print(f'played {(cur_t - start_t) / US:.3f}s, {shown:,} events drawn')
    return 0


def _setup_imu_plot(imu, t0):
    from matplotlib import pyplot as plt
    plt.ion()
    fig, axs = plt.subplots(6, 1, sharex=True, figsize=(8, 9))
    # Seconds from the start of the recording; the raw column is epoch
    # microseconds, which makes for a useless axis.
    ts = (imu['t'] - t0) / US
    for ax, key, color in zip(axs, ('ax', 'ay', 'az', 'wx', 'wy', 'wz'),
                              'rgbrgb'):
        ax.plot(ts, imu[key], color=color, lw=0.8)
        ax.set_ylabel(key)
    axs[-1].set_xlabel('time since start (s)')
    vlines = [ax.axvline(0.0, color='k', lw=1) for ax in axs]
    fig.tight_layout()
    fig.canvas.draw()
    plt.show(block=False)
    return fig, vlines


def _move_marker(fig, vlines, t_sec):
    try:
        for vl in vlines:
            vl.set_xdata([t_sec, t_sec])
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
    except Exception:
        pass  # non-interactive backend


if __name__ == '__main__':
    sys.exit(main())
