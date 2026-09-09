"""Buffered, schema-inferring HDF5 writer.

Datasets are created from the first batch appended to a column, so extractors
never have to declare a schema up front. Rows are buffered per group and
flushed in large blocks -- appending message-by-message would otherwise resize
the dataset once per ROS message.
"""

from __future__ import annotations

import numpy as np
import h5py

from .extractors import Batch, VLen

_STR_DTYPE = h5py.string_dtype(encoding='utf-8')


class H5Writer:
    def __init__(self, path, compression: str | None = 'gzip', compression_opts: int | None = 1,
                 buffer_rows: int = 1 << 16, buffer_bytes: int = 64 << 20):
        self.file = h5py.File(path, 'w', libver='latest')
        self.compression = compression or None
        self.compression_opts = compression_opts if self.compression == 'gzip' else None
        self.buffer_rows = buffer_rows
        self.buffer_bytes = buffer_bytes
        self._buffers: dict[str, dict[str, list]] = {}
        self._buffered_bytes: dict[str, int] = {}
        self._pending: dict[str, int] = {}
        self._counts: dict[str, int] = {}
        self._warned_shape: set[str] = set()
        self._extractors: list = []

    # -- public API --------------------------------------------------------

    def append(self, group: str, batch: Batch) -> None:
        rows = len(batch)
        if rows == 0:
            return
        buf = self._buffers.setdefault(group, {})
        for name, value in batch.columns.items():
            buf.setdefault(name, []).append(value)
        self._buffered_bytes[group] = self._buffered_bytes.get(group, 0) + _nbytes(batch)
        self._counts[group] = self._counts.get(group, 0) + rows
        # Tracked incrementally: recomputing it from the buffer on every append
        # makes a group that rarely fills its buffer quadratic in message count.
        pending = self._pending.get(group, 0) + rows
        self._pending[group] = pending
        if batch.attrs:
            self._set_attrs(group, batch.attrs)
        if pending >= self.buffer_rows or self._buffered_bytes[group] >= self.buffer_bytes:
            self.flush_group(group)

    def flush_group(self, group: str) -> None:
        buf = self._buffers.get(group)
        if not buf:
            return
        grp = self.file.require_group(group)
        for name, parts in buf.items():
            if not parts:
                continue
            if isinstance(parts[0], VLen):
                rows = [row for part in parts for row in part.rows]
                self._append_vlen(grp, name, rows, parts[0].dtype)
            else:
                self._append_array(grp, name, parts)
        self._buffers[group] = {}
        self._buffered_bytes[group] = 0
        self._pending[group] = 0

    def close(self) -> None:
        for group in list(self._buffers):
            self.flush_group(group)
        self.file.close()

    def counts(self) -> dict[str, int]:
        return dict(self._counts)

    def note_extractors(self, extractors) -> None:
        """Track extractors that may want to report something after the run."""
        self._extractors.extend(extractors)

    def notes(self) -> list[str]:
        out = []
        for extractor in self._extractors:
            adjusted = getattr(extractor, 'adjusted', 0)
            if adjusted:
                out.append(f'{extractor.NAME}: nudged {adjusted} event timestamp(s) '
                           f'forward to keep the stream sorted')
        return out

    def set_root_attrs(self, attrs: dict) -> None:
        for key, value in attrs.items():
            self.file.attrs[key] = value

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # -- internals ---------------------------------------------------------

    def _set_attrs(self, group: str, attrs: dict) -> None:
        grp = self.file.require_group(group)
        for key, value in attrs.items():
            if key not in grp.attrs:
                grp.attrs[key] = value

    def _append_array(self, grp, name: str, parts: list[np.ndarray]) -> None:
        parts = [np.asarray(p) for p in parts]
        if parts[0].dtype == object:  # variable-length strings
            data = np.concatenate(parts)
            dset = self._require(grp, name, (), _STR_DTYPE, chunk_rows=1024)
            self._extend(dset, [s.encode() if isinstance(s, str) else s for s in data])
            return

        tail = parts[0].shape[1:]
        mismatched = [i for i, p in enumerate(parts) if p.shape[1:] != tail]
        if mismatched:
            parts = [p if p.shape[1:] == tail else _resize_nearest(p, tail) for p in parts]
            key = f'{grp.name}/{name}'
            if key not in self._warned_shape:
                self._warned_shape.add(key)
                print(f'  warning: {key} has frames of varying shape; '
                      f'resampling to {tail} (nearest neighbour)')

        data = np.concatenate(parts) if len(parts) > 1 else parts[0]
        dset = self._require(grp, name, tail, data.dtype, chunk_rows=self._chunk_rows(tail, data.dtype))
        self._extend(dset, data)

    def _append_vlen(self, grp, name: str, rows: list[np.ndarray], dtype: np.dtype) -> None:
        dset = self._require(grp, name, (), h5py.vlen_dtype(dtype), chunk_rows=1024)
        self._extend(dset, rows)

    def _require(self, grp, name: str, tail: tuple, dtype, chunk_rows: int):
        if name in grp:
            return grp[name]
        kwargs = {}
        # h5py rejects compression on variable-length string/array datasets in
        # some builds only for the filters we do not use; gzip is fine here.
        if self.compression:
            kwargs['compression'] = self.compression
            if self.compression_opts is not None:
                kwargs['compression_opts'] = self.compression_opts
        return grp.create_dataset(
            name, shape=(0, *tail), maxshape=(None, *tail), dtype=dtype,
            chunks=(chunk_rows, *tail), **kwargs,
        )

    @staticmethod
    def _chunk_rows(tail: tuple, dtype) -> int:
        if not tail:
            return 1 << 16
        # Aim for roughly 1 MiB chunks, at least one full frame.
        frame_bytes = int(np.prod(tail)) * np.dtype(dtype).itemsize
        return max(1, min(64, (1 << 20) // max(frame_bytes, 1)))

    @staticmethod
    def _extend(dset, data) -> None:
        start = dset.shape[0]
        end = start + len(data)
        dset.resize((end, *dset.shape[1:]))
        dset[start:end] = data


def _nbytes(batch: Batch) -> int:
    total = 0
    for value in batch.columns.values():
        if isinstance(value, VLen):
            total += sum(r.nbytes for r in value.rows)
        elif isinstance(value, np.ndarray) and value.dtype != object:
            total += value.nbytes
    return total


def _resize_nearest(arr: np.ndarray, tail: tuple) -> np.ndarray:
    """Nearest-neighbour resample of an (N, H, W, ...) block onto `tail`."""
    if arr.ndim < 3 or len(tail) < 2:
        raise ValueError(f'cannot reconcile shape {arr.shape[1:]} with {tail}')
    rows = np.linspace(0, arr.shape[1] - 1, tail[0]).astype(np.intp)
    cols = np.linspace(0, arr.shape[2] - 1, tail[1]).astype(np.intp)
    out = arr[:, rows][:, :, cols]
    if out.shape[1:] != tail:  # channel count differs; pad or trim
        target_c = tail[2] if len(tail) > 2 else 1
        have_c = out.shape[3] if out.ndim > 3 else 1
        out = out.reshape(*out.shape[:3], have_c)
        if have_c > target_c:
            out = out[..., :target_c]
        else:
            out = np.repeat(out[..., :1], target_c, axis=3)
    return np.ascontiguousarray(out)
