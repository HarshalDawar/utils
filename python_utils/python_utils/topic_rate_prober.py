#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import time
import os
from functools import partial

class TopicSniffer(Node):
  def __init__(self):
    super().__init__('topic_sniffer')
    self.topic_subs = {}   # {topic_name: subscription}
    self.topic_stats = {}  # {topic_name: [timestamps]}
    self.first_display = True
    self.refresh_timer = self.create_timer(2.0, self.update_subscriptions)
    self.rate_timer = self.create_timer(1.0, self.display_rates)
    print("\n:sleuth_or_spy: Starting dynamic topic sniffer with live rate display...\n")

  def update_subscriptions(self):
    """Discover all active topics and subscribe dynamically."""
    topic_names_and_types = self.get_topic_names_and_types()
    for name, types in topic_names_and_types:
      if not types:
        continue

      topic_type = types[0]

      # Updated: simpler membership check
      if name in self.topic_subs:
        continue

      # Resolve message type
      try:
        MsgType = get_message(topic_type)
      except Exception as e:
        self.get_logger().warn(f":warning: Could not import '{topic_type}' for topic '{name}': {e}")
        continue

      # Create subscription
      subscription = self.create_subscription(
        MsgType,
        name,
        partial(self.listener_callback, topic_name=name),
        10
      )

      # Updated: store directly in dict
      self.topic_subs[name] = subscription
      self.topic_stats[name] = []

      print(f":white_check_mark: Subscribed to: {name:<30} [{topic_type}]")

  def listener_callback(self, msg, topic_name):
    """Record timestamps for each received message."""
    now = time.time()
    timestamps = self.topic_stats.get(topic_name, [])
    timestamps.append(now)

    # Updated: faster sliding window cleanup
    cutoff = now - 2.0
    while timestamps and timestamps[0] < cutoff:
      timestamps.pop(0)

  def display_rates(self):
    """Compute and display formatted message rates with color highlighting."""
    if not self.topic_stats:
      return

    target_rate = 100.0

    output = []
    # Header for the display
    topic_col_width = max((len(name) for name in self.topic_stats.keys()), default=10)
    topic_col_width = min(topic_col_width, 100)  # cap width
    header = f"{'Topic Name':<{topic_col_width}} {'Rate (Hz)':>10} {'Msgs':>8} {'Window (s)':>10}"
    output.append(header)
    output.append("-" * (topic_col_width + 40))  # line separator

    # Body of the display (for each topic)
    for topic_name, timestamps in sorted(self.topic_stats.items()):
      n = len(timestamps)

      if n > 1:
        duration = timestamps[-1] - timestamps[0]
        rate = (n - 1) / duration if duration > 0 else 0.0
        rate_str = f"{rate:>10.2f}"
        dur_str = f"{duration:>10.2f}"

        diff = rate - target_rate
        if abs(diff) < target_rate * 0.2:
            color_code = 32
        elif abs(diff) < target_rate * 0.5:
            color_code = 33
        else:
            color_code = 31

        rate_str = f"\033[{color_code}m{rate_str}\033[0m"

      elif n == 1:
        rate_str = f"{'~':>10}"
        dur_str = f"{'-':>10}"
      else:
        rate_str = f"{'—':>10}"
        dur_str = f"{'-':>10}"

      # Truncate very long names neatly
      display_name = topic_name if len(topic_name) <= topic_col_width else topic_name[:topic_col_width - 3] + '...'

      # Append the formatted line to the output
      output.append(f"{display_name:<{topic_col_width}} {rate_str} {n:>8} {dur_str}")

    output.append("=" * (topic_col_width + 40))  # final separator line

    # Print the output in a way that overwrites the old data
    print("\033[H\033[J", end="")
    print("\n".join(output))
    print("\nPress Ctrl+C to exit.\n")


def main(args=None):
  DISTRO = os.getenv("ROS_DISTRO")
  rclpy.init(args=args)

  # Updated: fix typo
  if DISTRO == 'jazzy':
    # Jazzy-specific placeholder
    pass

  node = TopicSniffer()

  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    # Updated: simpler cleanup (safe on Humble and Jazzy)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()