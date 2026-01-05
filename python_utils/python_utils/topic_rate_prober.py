#!/usr/bin/env python3
import atexit
from functools import partial
import os
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


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

        # Filter state
        self.filter_text = None  # Will hold the search term (e.g., 'abc')
        self.filter_invert = False  # Whether to invert the search (e.g., '!abc')
        self.filter_refresh_needed = False  # Flag to indicate if refresh is needed
        self.filter_editing = False

        print('\n:sleuth_or_spy: Starting dynamic topic sniffer with live rate display...\n')

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
                self.get_logger().warn(
                    f':warning: Could not import "{topic_type}" for topic "{name}": {e}')
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

            print(f':white_check_mark: Subscribed to: {name:<30} [{topic_type}]')

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
        header = f"{'Topic Name':<{topic_col_width}} {
            'Rate (Hz)':>10} {'Msgs':>8}  {'MB/s':>10} {'Window (s)':>10} {'Histogram <--':>12}"
        output.append(header)
        output.append('-' * (topic_col_width + 40))  # line separator

        # Body of the display (for each topic)
        for topic_name, timestamps in sorted(self.topic_stats.items()):
            # Apply filter to topics
            if self.filter_text:
                if self.filter_invert and self.filter_text in topic_name:
                    continue  # Skip topic if it matches the filter and is inverted
                elif not self.filter_invert and self.filter_text not in topic_name:
                    continue  # Skip topic if it doesn't match the filter

            n = len(timestamps)

            if n > 1:
                duration = timestamps[-1] - timestamps[0]
                rate = (n - 1) / duration if duration > 0 else 0.0
                rate_str = f'{rate:>10.2f}'
                dur_str = f'{duration:>10.2f}'

                diff = rate - target_rate
                if abs(diff) < target_rate * 0.2:
                    color_code = 32
                elif abs(diff) < target_rate * 0.5:
                    color_code = 33
                else:
                    color_code = 31

                rate_str = f'\033[{color_code}m{rate_str}\033[0m'

            elif n == 1:
                rate_str = f"{'~':>10}"
                dur_str = f"{'-':>10}"
            else:
                rate_str = f"{'—':>10}"
                dur_str = f"{'-':>10}"

            # Truncate very long names neatly
            display_name = topic_name if len(
                topic_name) <= topic_col_width else topic_name[:topic_col_width - 3] + '...'

            total_bytes = sum(self.topic_sizes.get(topic_name, []))
            mb_per_sec = (total_bytes / 1e6) / duration if n > 1 else 0.0
            mb_str = f'{mb_per_sec:>10.3f}'

            hist = ''
            if n > 1:
                # Convert timestamps to per-message intervals
                intervals = [timestamps[i] - timestamps[i - 1] for i in range(1, n)]
                if intervals:
                    # Normalize intervals to 8 sparkline levels
                    spark = '▁▂▃▄▅▆▇█'
                    imin, imax = min(intervals), max(intervals)

                    # Avoid division by zero when all intervals identical
                    if imax == imin:
                        hist = spark[-1] * min(len(intervals), 10)
                    else:
                        bins = intervals[-10:]  # last 10 intervals (recent activity)
                        hist = ''
                        for iv in bins:
                            norm = (iv - imin) / (imax - imin)
                            idx = int(norm * (len(spark) - 1))
                            hist += spark[idx]

                    # Pad to length 10 if fewer bins
                    hist = hist.rjust(10, spark[0])
                else:
                    hist = '.' * 10
            else:
                hist = '.' * 10

            # Append the formatted line to the output
            output.append(
                f'{display_name:<{topic_col_width}} {rate_str} {n:>8} {mb_str} {dur_str} {hist}')

        output.append('=' * (topic_col_width + 40))  # final separator line

        # Print the output in a way that overwrites the old data
        print('\033[H\033[J', end='')
        print('\n'.join(output))
        footer = (
            '\n'
            ' Keys: '
            '\033[1mESC\033[0m Quit   '
            '\033[1mCtrl+C\033[0m Force Quit   '
            '\033[1m↻\033[0m Auto-refresh   '
            '\033[1mf\033[0m Filter Topics\n'
        )
        print(footer)
        # If user is editing filter, reprint the input prompt line without clearing screen
        if self.filter_editing:
            self.print_input_prompt(self.filter_text or '')

    def setup_terminal(self):
        """Put terminal into raw mode for key reading."""
        self.stdin_fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)

        atexit.register(self.restore_terminal)

    def restore_terminal(self):
        """Restore terminal settings on exit."""
        if hasattr(self, 'old_term_settings'):
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
        """Check for keypresses and handle filter input non-blocking."""
        key = self.poll_keyboard()
        if not key or self._exit_requested:
            return

        # ESC key: quit
        if key == '\x1b':
            print('\nESC pressed — exiting.')
            self._exit_requested = True
            return

        # Start filter mode
        elif key == 'f' and not self.filter_editing:
            self.filter_editing = True
            self.filter_input_buffer = ''
            self.get_logger().info('Enter filter text (prefix with "!" to exclude): ')
            return

        # If we're editing filter, process keys
        if self.filter_editing:
            # Enter key finishes editing
            if key in ('\r', '\n'):
                self.filter_editing = False
                self.apply_filter(self.filter_input_buffer)
                print()  # Move to next line after input
            # Backspace
            elif key == '\x7f':
                self.filter_input_buffer = self.filter_input_buffer[:-1]
            # Regular character
            elif len(key) == 1 and key.isprintable():
                self.filter_input_buffer += key

            # Apply filter live as user types
            self.apply_filter(self.filter_input_buffer, live=True)

    def check_exit(self):
        if self._exit_requested:
            rclpy.shutdown()

    def get_filter_input(self):
        """Prompt user for a filter string, updating dynamically."""
        filter_text = self.filter_text or ''
        self.filter_editing = True
        self.display_rates()  # Initial display with prompt

        try:
            while True:
                ch = self.poll_keyboard()
                if ch is None:
                    time.sleep(0.05)
                    continue

                if ch == '\x1b':  # ESC cancels
                    self.filter_editing = False
                    return None
                elif ch == '\x7f':  # Backspace
                    filter_text = filter_text[:-1]
                elif ch in ('\r', '\n'):  # Enter finishes input
                    break
                else:
                    filter_text += ch

                # Update dynamically without clearing the prompt
                self.filter_text = filter_text
                self.display_rates()

            # Finalize filter after Enter
            if filter_text.startswith('!'):
                self.filter_invert = True
                self.filter_text = filter_text[1:]
            else:
                self.filter_invert = False
                self.filter_text = filter_text

            self.filter_editing = False
            self.display_rates()
            print()  # move to new line after input
            return filter_text

        except KeyboardInterrupt:
            self.filter_editing = False
            return None

    def print_input_prompt(self, filter_text):
        """Print the current filter input in the terminal."""
        print(f'\rFilter: {filter_text}', end='', flush=True)

    def apply_filter(self, filter_text, live=False):
        """Apply the filter based on user input. If live=True, suppress extra printing."""
        if filter_text.startswith('!'):
            self.filter_invert = True
            self.filter_text = filter_text[1:]
        else:
            self.filter_invert = False
            self.filter_text = filter_text

        if not live:
            print(f'Filtering topics: {
                "NOT " if self.filter_invert else ""}containing "{self.filter_text}"')

        # Display topics, but if live, keep the filter prompt visible
        self.display_rates()
        if self.filter_editing:
            print(f'\rFilter: {self.filter_input_buffer}', end='', flush=True)


def main(args=None):
    DISTRO = os.getenv('ROS_DISTRO')
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
