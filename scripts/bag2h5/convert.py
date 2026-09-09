"""Bag -> HDF5 driver."""

from __future__ import annotations

import concurrent.futures as cf
import shutil
import sys
import time
from collections import deque
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

from . import extractors
from .config import Selection, TopicPlan, plan_topics
from .typestore import build_typestore
from .writer import H5Writer

# Per-worker state, initialised once per process.
_WORKER: dict = {}


def _init_worker(fielddefs, is_ros2, plans, use_bag_time, image_opts) -> None:
    # The definitions the reader actually resolved are shipped in, rather than
    # rebuilt here: for mcap bags AnyReader uses the definitions embedded in the
    # bag and ignores the default typestore, so rebuilding could disagree.
    typestore = get_typestore(Stores.EMPTY)
    typestore.register(fielddefs)
    _WORKER['typestore'] = typestore
    _WORKER['is_ros2'] = is_ros2
    _WORKER['use_bag_time'] = use_bag_time
    _WORKER['by_topic'] = {
        p.topic: (p.group, extractors.make(p.extractor, **image_opts)) for p in plans
    }


def _deserialize_chunk(chunk):
    """Run in a worker: raw CDR -> list of (group, Batch)."""
    typestore = _WORKER['typestore']
    by_topic = _WORKER['by_topic']
    use_bag_time = _WORKER['use_bag_time']
    deserialize = (typestore.deserialize_cdr if _WORKER['is_ros2']
                   else typestore.deserialize_ros1)
    out = []
    for topic, msgtype, rawdata, bag_time_ns in chunk:
        group, extractor = by_topic[topic]
        msg = deserialize(rawdata, msgtype)
        batch = extractor(msg, bag_time_ns // 1000, use_bag_time)
        if batch is not None:
            out.append((group, batch))
    return out


def _split_stateful(plans):
    """Stateful extractors must see their topic in order, in one process."""
    stateful, stateless = [], []
    for plan in plans:
        target = stateful if extractors.BY_NAME[plan.extractor].STATEFUL else stateless
        target.append(plan)
    return stateful, stateless


def describe(bag: Path, store: str, msg_paths, selection: Selection) -> None:
    """Print what the bag holds and what would be exported."""
    typestore = build_typestore(store, msg_paths, verbose=True, hints=[bag])
    with AnyReader([bag], default_typestore=typestore) as reader:
        counts: dict[tuple[str, str], int] = {}
        for conn in reader.connections:
            counts[(conn.topic, conn.msgtype)] = counts.get((conn.topic, conn.msgtype), 0) + conn.msgcount
        plans, skipped = plan_topics(reader.connections, selection)
        known = reader.typestore.types
        unknown = sorted({p.msgtype for p in plans if p.msgtype not in known})

    width = max((len(t) for t, _ in counts), default=10)
    print(f'\n{len(counts)} topic(s) in {bag}\n')
    print(f'{"TOPIC".ljust(width)}  {"MESSAGES":>10}  {"EXTRACTOR":<16}  DESTINATION')
    by_topic = {p.topic: p for p in plans}
    reasons = {t: r for t, _, r in skipped}
    for (topic, msgtype), count in sorted(counts.items()):
        plan = by_topic.get(topic)
        if plan:
            print(f'{topic.ljust(width)}  {count:>10}  {plan.extractor:<16}  {plan.group}')
        else:
            print(f'{topic.ljust(width)}  {count:>10}  {"-":<16}  skipped: {reasons.get(topic, "?")}')
        print(f'{"".ljust(width)}  {"":>10}  {msgtype}')

    if unknown:
        print('\nno message definition available for:')
        for name in unknown:
            print(f'  {name}')
        print('Build or source the workspace providing them, or pass --msg-path.')


def convert(bag: Path, output: Path, store: str, msg_paths, selection: Selection,
            jobs: int = 1, chunk_size: int = 512, use_bag_time: bool = False,
            compression: str | None = 'gzip', compression_level: int = 1,
            image_opts: dict | None = None, progress_interval: float = 5.0) -> dict[str, int]:
    image_opts = image_opts or {}
    typestore = build_typestore(store, msg_paths, verbose=True, hints=[bag])

    with AnyReader([bag], default_typestore=typestore) as reader:
        plans, skipped = plan_topics(reader.connections, selection)
        _check_known(plans, reader.typestore)
        if not plans:
            raise SystemExit(
                'no topics selected. Run with --list to see what the bag contains.'
            )

        wanted = {p.topic for p in plans}
        connections = [c for c in reader.connections if c.topic in wanted]
        total = sum(c.msgcount for c in connections)

        print(f'\nconverting {total} message(s) from {len(plans)} topic(s) -> {output}')
        for plan in plans:
            print(f'  {plan.topic}  ->  {plan.group}  [{plan.extractor}]')
        for topic, msgtype, reason in skipped:
            print(f'  skipping {topic} ({msgtype}): {reason}')

        writer = H5Writer(output, compression=compression, compression_opts=compression_level)
        writer.set_root_attrs({
            'source_bag': str(bag),
            'time_unit': 'microseconds',
            'time_source': 'bag' if use_bag_time else 'header',
        })

        progress = _Progress(total, progress_interval)
        done = 0
        try:
            if jobs > 1:
                done = _run_parallel(reader, connections, plans, writer, jobs, chunk_size,
                                     use_bag_time, image_opts, progress)
            else:
                done = _run_serial(reader, connections, plans, writer,
                                   use_bag_time, image_opts, progress)
        finally:
            progress.close()
            counts = writer.counts()
            notes = writer.notes()
            writer.close()

    print(f'\nread {done} message(s) in {progress.elapsed():.1f}s')
    for group, rows in sorted(counts.items()):
        print(f'  {group}: {rows} row(s)')
    for note in notes:
        print(f'  note: {note}')
    print(f'wrote {output}')
    return counts


def _check_known(plans, typestore) -> None:
    missing = sorted({p.msgtype for p in plans if p.msgtype not in typestore.types})
    if missing:
        raise SystemExit(
            'no message definition available for:\n  ' + '\n  '.join(missing) +
            '\nBuild or source the workspace providing them, or point --msg-path at '
            'a directory containing their .msg files.'
        )


def _fmt_duration(seconds: float) -> str:
    seconds = int(max(seconds, 0))
    if seconds < 60:
        return f'{seconds}s'
    if seconds < 3600:
        return f'{seconds // 60}m{seconds % 60:02d}s'
    return f'{seconds // 3600}h{(seconds % 3600) // 60:02d}m'


class _Progress:
    """A live progress bar on a terminal, periodic lines when redirected.

    Nothing is imported for this: the tool stays installable with just
    rosbags/h5py/numpy.
    """

    FILL = '\u2588'   # full block
    EMPTY = '\u2591'  # light shade

    def __init__(self, total: int, interval: float = 5.0, stream=None):
        self.total = max(int(total), 0)
        self.stream = stream if stream is not None else sys.stdout
        self.is_tty = bool(getattr(self.stream, 'isatty', lambda: False)())
        # Redraw often on a terminal; when piped, fall back to occasional lines
        # so a log file does not fill with thousands of repaints.
        self.interval = 0.1 if self.is_tty else interval
        self.started = time.time()
        self.last_draw = 0.0
        self.done = 0
        self.width = 0

    # -- public API --------------------------------------------------------

    def update(self, done: int) -> None:
        self.done = done
        now = time.time()
        if now - self.last_draw < self.interval:
            return
        self.last_draw = now
        self._draw(now)

    def close(self) -> None:
        """Leave the cursor on a clean line before anything else prints."""
        if self.is_tty and self.width:
            self.stream.write('\r' + ' ' * self.width + '\r')
            self.stream.flush()
            self.width = 0

    def elapsed(self) -> float:
        return time.time() - self.started

    # -- internals ---------------------------------------------------------

    def _stats(self, now: float) -> tuple[str, float]:
        elapsed = now - self.started
        rate = self.done / elapsed if elapsed > 0 else 0.0
        frac = min(self.done / self.total, 1.0) if self.total else 0.0
        parts = [f'{100 * frac:5.1f}%', f'{self.done:,}/{self.total:,}', f'{rate:,.0f} msg/s']
        if rate > 0 and self.total:
            parts.append(f'eta {_fmt_duration((self.total - self.done) / rate)}')
        return '  '.join(parts), frac

    def _draw(self, now: float) -> None:
        text, frac = self._stats(now)
        if not self.is_tty:
            self.stream.write(f'  {text}\n')
            self.stream.flush()
            return

        # get_terminal_size only falls back when the query *raises*; a pty with
        # no winsize (script, tmux detach, some CI) reports 0 columns, which
        # would truncate the bar to nothing.
        columns = shutil.get_terminal_size((80, 24)).columns
        if columns < 30:
            columns = 80
        # Width depends only on the terminal, never on the stats text, so the
        # bar does not wobble as the numbers and the ETA change length.
        bar_width = max(10, min(32, columns - 60))
        filled = int(round(bar_width * frac))
        bar = self.FILL * filled + self.EMPTY * (bar_width - filled)
        line = f'  [{bar}]  {text}'[:columns]
        # Pad to erase the previous, possibly longer, line.
        self.width = max(self.width, len(line))
        self.stream.write('\r' + line.ljust(self.width))
        self.stream.flush()


def _run_serial(reader, connections, plans, writer, use_bag_time,
                image_opts, progress) -> int:
    by_topic = {p.topic: (p.group, extractors.make(p.extractor, **image_opts)) for p in plans}
    writer.note_extractors(x for _, x in by_topic.values())
    done = 0
    for conn, bag_time_ns, rawdata in reader.messages(connections=connections):
        group, extractor = by_topic[conn.topic]
        msg = reader.deserialize(rawdata, conn.msgtype)
        batch = extractor(msg, bag_time_ns // 1000, use_bag_time)
        if batch is not None:
            writer.append(group, batch)
        done += 1
        progress.update(done)
    return done


def _run_parallel(reader, connections, plans, writer, jobs, chunk_size,
                  use_bag_time, image_opts, progress) -> int:
    stateful, stateless = _split_stateful(plans)
    inline = {p.topic: (p.group, extractors.make(p.extractor, **image_opts)) for p in stateful}
    writer.note_extractors(x for _, x in inline.values())

    if not stateless:  # nothing worth a worker pool
        return _run_serial(reader, connections, plans, writer, use_bag_time,
                           image_opts, progress)

    done = 0
    # Bound the number of in-flight chunks so the reader cannot outrun the
    # writer and pull the whole bag into memory.
    window = max(2 * jobs, 4)
    pending: deque = deque()

    with cf.ProcessPoolExecutor(
        max_workers=jobs, initializer=_init_worker,
        initargs=(reader.typestore.fielddefs, reader.is2, stateless, use_bag_time,
                  image_opts),
    ) as pool:
        def drain(limit: int) -> None:
            nonlocal done
            while len(pending) > limit:
                future, size = pending.popleft()
                for group, batch in future.result():
                    writer.append(group, batch)
                done += size

        chunk = []
        read = 0
        for conn, bag_time_ns, rawdata in reader.messages(connections=connections):
            # The bar follows messages read, which advances smoothly; `done`
            # only moves when a batch of chunks is drained, which is bursty.
            read += 1
            progress.update(read)
            if conn.topic in inline:
                # Decoded here, in bag order, so the decoder keeps its state.
                # Each topic owns its own group, so interleaving with pooled
                # results never reorders anything within a group.
                group, extractor = inline[conn.topic]
                msg = reader.deserialize(rawdata, conn.msgtype)
                batch = extractor(msg, bag_time_ns // 1000, use_bag_time)
                if batch is not None:
                    writer.append(group, batch)
                done += 1
                continue

            chunk.append((conn.topic, conn.msgtype, rawdata, bag_time_ns))
            if len(chunk) >= chunk_size:
                pending.append((pool.submit(_deserialize_chunk, chunk), len(chunk)))
                chunk = []
                drain(window)
        if chunk:
            pending.append((pool.submit(_deserialize_chunk, chunk), len(chunk)))
        drain(0)
    return done
