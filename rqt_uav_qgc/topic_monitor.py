import importlib
import time

from rclpy.node import Node


def resolve_msg_type(node: Node, topic_name: str):
    """
    Query the ROS 2 graph for the message type of *topic_name* and return the
    corresponding Python class.

    Returns ``(msg_class, type_str)`` on success, or ``(None, error_str)`` on
    failure (topic not found / package not installed).

    Example
    -------
    ``px4_msgs/msg/VehicleStatus``  →  ``px4_msgs.msg.VehicleStatus``
    """
    # 1. Look up the type string from the live graph
    names_and_types = dict(node.get_topic_names_and_types())
    if topic_name not in names_and_types:
        return None, f"Topic '{topic_name}' not found in ROS 2 graph"

    type_strs = names_and_types[topic_name]  # list, usually one entry
    type_str = type_strs[0]  # e.g. "px4_msgs/msg/VehicleStatus"

    # 2. Convert "pkg/msg/Cls" → import pkg.msg and get attribute Cls
    try:
        # type_str format: "package_name/msg/ClassName"  (ROS 2 IDL style)
        parts = type_str.split('/')
        if len(parts) != 3:
            return None, f"Unexpected type format: '{type_str}'"
        pkg, _subdir, cls_name = parts
        module = importlib.import_module(f"{pkg}.msg")
        msg_class = getattr(module, cls_name)
        return msg_class, type_str
    except Exception as exc:
        return None, f"Cannot import '{type_str}': {exc}"


class TopicMonitor:
    """
    A simplified helper to monitor ROS 2 topic frequency and data fields.
    """
    def __init__(self, node: Node, topic_name: str, msg_type, callback=None):
        self.node = node
        self.topic_name = topic_name
        self.msg_type = msg_type
        self.callback = callback

        self.hz = 0.0
        self.msg_count = 0
        self.last_msg = None
        self._last_time = None
        
        # Subscribe to the topic
        self.sub = self.node.create_subscription(
            msg_type,
            topic_name,
            self._msg_callback,
            10
        )

    def _msg_callback(self, msg):
        current_time = time.time()
        
        # Calculate Frequency (Hz)
        if self._last_time is not None:
            dt = current_time - self._last_time
            if dt > 0:
                current_hz = 1.0 / dt
                # Simple exponential moving average to smooth the Hz display
                self.hz = 0.9 * self.hz + 0.1 * current_hz
        
        self._last_time = current_time
        self.msg_count += 1
        self.last_msg = msg
        
        # Trigger UI update callback if provided
        if self.callback:
            self.callback(self)

    def stop(self):
        if hasattr(self, 'sub'):
            self.node.destroy_subscription(self.sub)

    @staticmethod
    def get_field_value(msg, field_path: str):
        """
        Dynamically extract a field value from a message using a dot-path.
        Example: get_field_value(msg, "pose.position.x")
        """
        try:
            val = msg
            for part in field_path.split('.'):
                val = getattr(val, part)
            return val
        except Exception:
            return None
