"""Topic -> (extractor, HDF5 group) resolution.

Nothing about a particular robot is hard-coded. A topic is exported when its
message type has an extractor; the destination group defaults to the topic name
and can be overridden per topic from the CLI or a YAML file.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from . import extractors


@dataclass(frozen=True)
class TopicPlan:
    topic: str
    msgtype: str
    extractor: str
    group: str


@dataclass
class Selection:
    """User-supplied overrides, independent of any particular bag."""

    overrides: dict[str, dict] = None      # topic -> {group?, extractor?}
    include: list[str] = None              # regexes; empty means "everything"
    exclude: list[str] = None
    auto: bool = True                      # export unmapped-but-supported topics

    def __post_init__(self):
        self.overrides = self.overrides or {}
        self.include = self.include or []
        self.exclude = self.exclude or []


def sanitize_group(topic: str) -> str:
    """`/zed_node/left/image_rect` -> `zed_node/left/image_rect`."""
    cleaned = re.sub(r'[^0-9A-Za-z_/]+', '_', topic.strip('/'))
    cleaned = re.sub(r'/{2,}', '/', cleaned).strip('/')
    return cleaned or 'topic'


def parse_map(spec: str) -> tuple[str, dict]:
    """Parse `TOPIC=GROUP`, `TOPIC=GROUP:EXTRACTOR` or `TOPIC=:EXTRACTOR`.

    Group names may not contain ':' -- the last ':' separates the extractor.
    """
    topic, sep, rhs = spec.partition('=')
    if not sep:
        raise SystemExit(f'--map needs TOPIC=GROUP[:EXTRACTOR], got {spec!r}')
    entry: dict = {}
    group, sep, extractor = rhs.rpartition(':')
    if sep:
        if extractor:
            entry['extractor'] = extractor
    else:
        group = rhs
    if group:
        entry['group'] = group.strip('/')
    if not entry:
        raise SystemExit(f'--map entry {spec!r} sets neither a group nor an extractor')
    return topic, entry


def load_config(path: str) -> tuple[dict[str, dict], dict]:
    """Read a YAML mapping file; returns (topic overrides, defaults)."""
    try:
        import yaml
    except ImportError:
        raise SystemExit('--config needs PyYAML installed (pip install pyyaml)') from None
    data = yaml.safe_load(Path(path).read_text()) or {}
    raw_topics = data.get('topics') or {}
    overrides: dict[str, dict] = {}
    for topic, value in raw_topics.items():
        if value is None:
            overrides[topic] = {}
        elif isinstance(value, str):
            overrides[topic] = {'group': value.strip('/')}
        elif isinstance(value, dict):
            entry = {k: v for k, v in value.items() if k in ('group', 'extractor')}
            if 'group' in entry:
                entry['group'] = str(entry['group']).strip('/')
            overrides[topic] = entry
        else:
            raise SystemExit(f'config: bad entry for topic {topic!r}: {value!r}')
    return overrides, data.get('defaults') or {}


def _matches(patterns: list[str], topic: str) -> bool:
    return any(re.search(p, topic) for p in patterns)


def plan_topics(connections, selection: Selection) -> tuple[list[TopicPlan], list[tuple[str, str, str]]]:
    """Resolve bag connections into a conversion plan.

    Returns (plans, skipped) where `skipped` is (topic, msgtype, reason).
    """
    plans: list[TopicPlan] = []
    skipped: list[tuple[str, str, str]] = []
    used_groups: dict[str, str] = {}

    for conn in sorted({(c.topic, c.msgtype) for c in connections}):
        topic, msgtype = conn
        override = selection.overrides.get(topic, {})
        explicit = topic in selection.overrides

        if selection.exclude and _matches(selection.exclude, topic):
            skipped.append((topic, msgtype, 'excluded'))
            continue
        if selection.include and not _matches(selection.include, topic) and not explicit:
            skipped.append((topic, msgtype, 'not included'))
            continue
        if not explicit and not selection.auto:
            skipped.append((topic, msgtype, 'not mapped (--no-auto)'))
            continue

        name = override.get('extractor') or extractors.BY_MSGTYPE.get(msgtype)
        if name is None:
            skipped.append((topic, msgtype, 'no extractor for this message type'))
            continue
        if name not in extractors.BY_NAME:
            raise SystemExit(f'topic {topic}: unknown extractor {name!r}; '
                             f'available: {", ".join(sorted(extractors.BY_NAME))}')

        group = override.get('group')
        if not group:
            prefix = extractors.BY_NAME[name].GROUP_PREFIX
            group = sanitize_group(topic)
            if prefix:
                group = f'{prefix}/{group}'

        if group in used_groups and used_groups[group] != topic:
            raise SystemExit(
                f'group {group!r} is claimed by both {used_groups[group]!r} and '
                f'{topic!r}; disambiguate with --map'
            )
        used_groups[group] = topic
        plans.append(TopicPlan(topic=topic, msgtype=msgtype, extractor=name, group=group))

    return plans, skipped
