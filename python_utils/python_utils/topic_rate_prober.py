#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
import time
import os
from functools import partial
import sys
import math

import termios
import tty
import select
import atexit


class TopicSniffer(Node):
  def __init__(self):
    super().__init__('topic_sniffer')
    self.topic_subs = {}   # {topic_name: subscription}
    self.topic_stats = {}  # {topic_name: [timestamps]}
    self.topic_sizes = {}  # {topic_name: [size_bytes]}
    self.first_display = True
    self.refresh_timer = self.create_timer(2.0, self.update_subscriptions)
    self.rate_timer = self.create_timer(1.0, self.display_rates)

    self.setup_terminal()
    self.key_timer = self.create_timer(0.1, self.handle_keys)
    self.exit_timer = self.create_timer(0.1, self.check_exit)
    self._exit_requested = False

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
      self.topic_sizes[name] = []

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

    # Convert msg to serialized form (ROS2 standard)
    try:
      size_bytes = sys.getsizeof(msg)
    except Exception:
      size_bytes = 0

    sizes = self.topic_sizes.get(topic_name, [])
    sizes.append(size_bytes)

    while sizes and len(sizes) > len(timestamps):
      sizes.pop(0)

  def display_rates(self):
    """Compute and display formatted message rates with color highlighting."""
    if not self.topic_stats:
      return

    target_rate = 100.0

    output = []
    # Header for the display
    topic_col_width = max((len(name) for name in self.topic_stats.keys()), default=10)
    topic_col_width = min(topic_col_width, 100)  # cap width
    header = f"{'Topic Name':<{topic_col_width}} {'Rate (Hz)':>10} {'Msgs':>8}  {'MB/s':>10} {'Window (s)':>10} {'Histogram <--':>12}"
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

      total_bytes = sum(self.topic_sizes.get(topic_name, []))
      mb_per_sec = (total_bytes / 1e6) / duration if n > 1 else 0.0
      mb_str = f"{mb_per_sec:>10.3f}"

      hist = ""
      if n > 1:
        # Convert timestamps to per-message intervals
        intervals = [timestamps[i] - timestamps[i - 1] for i in range(1, n)]
        if intervals:
          # Normalize intervals to 8 sparkline levels
          spark = "▁▂▃▄▅▆▇█"
          imin, imax = min(intervals), max(intervals)

          # Avoid division by zero when all intervals identical
          if imax == imin:
            hist = spark[-1] * min(len(intervals), 10)
          else:
            bins = intervals[-10:]  # last 10 intervals (recent activity)
            hist = ""
            for iv in bins:
              norm = (iv - imin) / (imax - imin)
              idx = int(norm * (len(spark) - 1))
              hist += spark[idx]

          # Pad to length 10 if fewer bins
          hist = hist.rjust(10, spark[0])
        else:
          hist = "." * 10
      else:
        hist = "." * 10

      # Append the formatted line to the output
      output.append(f"{display_name:<{topic_col_width}} {rate_str} {n:>8} {mb_str} {dur_str} {hist}")

    output.append("=" * (topic_col_width + 40))  # final separator line

    # Print the output in a way that overwrites the old data
    print("\033[H\033[J", end="")
    print("\n".join(output))
    footer = (
      "\n"
      " Keys: "
      "\033[1mESC\033[0m Quit   "
      "\033[1mCtrl+C\033[0m Force Quit   "
      "\033[1m↻\033[0m Auto-refresh\n"
    )
    print(footer)
    print("\nPress Ctrl+C to exit.\n")


  def setup_terminal(self):
    """Put terminal into raw mode for key reading."""
    self.stdin_fd = sys.stdin.fileno()
    self.old_term_settings = termios.tcgetattr(self.stdin_fd)
    tty.setcbreak(self.stdin_fd)

    atexit.register(self.restore_terminal)

  def restore_terminal(self):
    """Restore terminal settings on exit."""
    if hasattr(self, "old_term_settings"):
        termios.tcsetattr(
            self.stdin_fd,
            termios.TCSADRAIN,
            self.old_term_settings
        )
        del self.old_term_settings

  def poll_keyboard(self):
    """Check for keypresses without blocking."""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
      ch = sys.stdin.read(1)
      return ch
    return None
  
  def handle_keys(self):
    key = self.poll_keyboard()
    if not key or self._exit_requested:
      return

    # ESC key
    if key == '\x1b':
      print("\nESC pressed — exiting.")
      self._exit_requested = True

  def check_exit(self):
    if self._exit_requested:
      rclpy.shutdown()

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
    if rclpy.ok():
      rclpy.shutdown()

if __name__ == '__main__':
  main()