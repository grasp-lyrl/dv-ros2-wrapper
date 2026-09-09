"""Generic ROS 1/2 bag -> HDF5 conversion.

Reading is done with `rosbags`, so no ROS environment needs to be sourced and
both mcap and sqlite3 bags work. Topics are bound to extractors by message
type, not by name; see `extractors.py` to add a new message type.
"""

from .config import Selection, TopicPlan, plan_topics, sanitize_group
from .convert import convert, describe
from .typestore import available_stores, build_typestore
from .writer import H5Writer

__all__ = [
    'H5Writer', 'Selection', 'TopicPlan', 'available_stores', 'build_typestore',
    'convert', 'describe', 'plan_topics', 'sanitize_group',
]
