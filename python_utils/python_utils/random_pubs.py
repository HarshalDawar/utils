#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import importlib
import random
import string


class RandomPublishers(Node):
  def __init__(self, msg_types, n_publishers, freq_min, freq_max):
    super().__init__("random_publishers")

    self.get_logger().info(
      f"Starting {n_publishers} random publishers "
      f"with frequencies [{freq_min}, {freq_max}] Hz"
    )

    for i in range(n_publishers):
      # Pick random message type
      type_str = random.choice(msg_types)
      msg_type = self._import_type(type_str)

      # Random frequency
      freq = random.uniform(freq_min, freq_max)

      # Topic name
      topic = f"/pub_{i}"

      pub = self.create_publisher(msg_type, topic, 10)

      # Create callback with message type
      timer = self.create_timer(1.0 / freq, self._make_callback(pub, msg_type))

      self.get_logger().info(f"[{topic}] {type_str} @ {freq:.2f} Hz")

  def _import_type(self, type_str):
    module, cls = type_str.rsplit(".", 1)
    return getattr(importlib.import_module(module), cls)

  def _make_callback(self, publisher, msg_type):
    def callback():
      msg = msg_type()

      if hasattr(msg, "data"):
        # std_msgs/String
        if isinstance(msg.data, str):
          msg.data = ''.join(random.choices(string.ascii_letters, k=8))

        # std_msgs/Float32
        elif isinstance(msg.data, float):
          msg.data = float(random.uniform(0, 100))

        # std_msgs/Int32
        elif isinstance(msg.data, int):
          msg.data = int(random.randint(0, 100))

      publisher.publish(msg)

    return callback


def main(args=None):
  rclpy.init(args=args)

  # Example set of message types
  msg_types = [
    "std_msgs.msg.String",
    "std_msgs.msg.Int32",
    "std_msgs.msg.Float32",
  ]

  node = RandomPublishers(
    msg_types=msg_types,
    n_publishers=13,
    freq_min=0.5,
    freq_max=1000.0,
  )

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()


if __name__ == "__main__":
  main()
