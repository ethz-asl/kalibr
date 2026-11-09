"""Minimal ROS 2 port of the ROS 1 `rosbag.Bag` API.

Only the members used by kalibr's dataset readers and writers are provided:

    Bag(bagfile)                        # read mode
    Bag(bagfile, 'w')                   # write mode
    bag._get_connections(topics=...)
    bag._get_indexes(connections)
    bag._read_message(position)
    bag.write(topic, message, stamp)
    bag.close()

`_read_message` returns a ``(topic, message, stamp)`` tuple where ``stamp``
mimics the old ``genpy.Time`` interface (``secs``, ``nsecs``, ``to_sec()``) and
``message`` has a ``_type`` attribute holding the ROS 1 style type name (e.g.
``sensor_msgs/Image``) in addition to the deserialized ROS 2 message fields.

All messages are read once into memory, which preserves the random-access
behaviour of the original implementation.
"""

import os


class _RosTime(object):
    """Minimal stand-in for ``genpy.Time``."""

    def __init__(self, secs=0, nsecs=0):
        self.secs = int(secs)
        self.nsecs = int(nsecs)

    def to_sec(self):
        return self.secs + self.nsecs * 1e-9

    def __repr__(self):
        return '_RosTime(secs=%d, nsecs=%d)' % (self.secs, self.nsecs)


def _patch_builtin_interfaces_time():
    """Add the legacy ``genpy.Time`` interface to ``builtin_interfaces.msg.Time``.

    kalibr's dataset readers access ``header.stamp.secs``, ``header.stamp.nsecs``
    and ``header.stamp.to_sec()``.  The ROS 2 message only provides ``sec``,
    ``nanosec`` and no conversion method, so the missing members are added as
    class-level descriptors once.
    """
    from builtin_interfaces.msg import Time

    if not hasattr(Time, 'secs'):
        Time.secs = property(lambda self: self.sec)
    if not hasattr(Time, 'nsecs'):
        Time.nsecs = property(lambda self: self.nanosec)
    if not hasattr(Time, 'to_sec'):
        Time.to_sec = lambda self: self.sec + self.nanosec * 1e-9


class _Index(object):
    """Minimal stand-in for a rosbag message index entry."""

    def __init__(self, position):
        self.position = position


def _type_from_message(msg):
    """Return the ROS 2 message type string of a deserialized message."""
    module = msg.__class__.__module__  # e.g. 'sensor_msgs.msg._image'
    name = msg.__class__.__name__
    pkg = module.split('.msg.')[0]
    return '%s/msg/%s' % (pkg, name)


class Bag(object):
    def __init__(self, bagfile, mode='r'):
        self.bagfile = bagfile
        if mode == 'w':
            self._init_writer(bagfile)
        else:
            self._messages = []  # list of (topic, type, serialized, timestamp_ns)
            self._load(bagfile)

    def _load(self, bagfile):
        from rosbag2_py import SequentialReader
        from rosbag2_py import StorageOptions
        from rosbag2_py import ConverterOptions

        reader = SequentialReader()
        storage_options = StorageOptions(uri=bagfile, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        reader.open(storage_options, converter_options)

        type_map = {}
        for topic_meta in reader.get_all_topics_and_types():
            type_map[topic_meta.name] = topic_meta.type

        while reader.has_next():
            topic, data, t = reader.read_next()
            self._messages.append((topic, type_map.get(topic, ''), data, t))

    def _init_writer(self, bagfile):
        from rosbag2_py import SequentialWriter
        from rosbag2_py import StorageOptions
        from rosbag2_py import ConverterOptions

        self._writer = SequentialWriter()
        storage_options = StorageOptions(uri=bagfile, storage_id='sqlite3')
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        self._writer.open(storage_options, converter_options)
        self._written_topics = set()

    def write(self, topic, msg, t):
        from rclpy.serialization import serialize_message
        from rosbag2_py import TopicMetadata

        type_name = _type_from_message(msg)
        if topic not in self._written_topics:
            self._writer.create_topic(TopicMetadata(
                name=topic, type=type_name, serialization_format='cdr'))
            self._written_topics.add(topic)

        secs = getattr(t, 'secs', getattr(t, 'sec', 0))
        nsecs = getattr(t, 'nsecs', getattr(t, 'nanosec', 0))
        self._writer.write(topic, serialize_message(msg),
                           int(secs) * 1000000000 + int(nsecs))

    def close(self):
        if hasattr(self, '_writer'):
            del self._writer
            self._writer = None

    def _get_connections(self, topics=None):
        if topics is None:
            wanted = set(t for (t, _, _, _) in self._messages)
        elif isinstance(topics, str):
            wanted = set([topics])
        else:
            wanted = set(topics)
        # Return one pseudo-connection per matching topic.
        return [t for t in wanted if t in set(m[0] for m in self._messages)]

    def _get_indexes(self, connections):
        for conn in connections:
            positions = [i for i, (t, _, _, _) in enumerate(self._messages)
                         if t == conn]
            if positions:
                yield [_Index(p) for p in positions]

    def _read_message(self, position):
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        _patch_builtin_interfaces_time()

        topic, type_name, data, t = self._messages[position]
        msg_cls = get_message(type_name)
        msg = deserialize_message(data, msg_cls)

        # Expose the ROS 1 style type name used by kalibr.  ROS 2 messages use
        # __slots__, so the attribute is attached to the class (constant per
        # message type) instead of the instance.
        ros1_type = type_name.replace('/msg/', '/').replace('/srv/', '/')
        if not hasattr(msg_cls, '_type'):
            msg_cls._type = ros1_type

        stamp = _RosTime(t // 1000000000, t % 1000000000)
        return (topic, msg, stamp)
