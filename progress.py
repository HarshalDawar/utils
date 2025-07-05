#!/usr/bin/env python

import sys
import time

class bcolors:
    LOWEST_0 = '\033[91m'
    LOWER_1 = '\033[93m'
    LOW_2 = '\033[95m'
    HIGH_3 = '\033[94m'
    HIGHER_4 = '\033[96m'
    HIGHEST_5 = '\033[92m'
    ENDC__1 = '\033[0m'

def progress_bar(progress, total, length=100, fill_char='█',
                empty_char='-', add_rotator=True, color_enabled=True, color_dynamic=True,
                ):
    # print(bcolors.WARNING +  "tester" + bcolors.ENDC)
    percent = 100 * (progress / float(total))
    filled_length = int(length * progress // total)
    bar = fill_char * filled_length + empty_char * (length - filled_length)
    content = ''
    if add_rotator:
        rotator = ['|', '/', '-', '\\']
    else:
        rotator = ['']
    if color_enabled and color_dynamic:
        if percent < 20:
            color = bcolors.LOWEST_0
        elif percent < 40:
            color = bcolors.LOWER_1
        elif percent < 60:
            color = bcolors.LOW_2
        elif percent < 80:
            color = bcolors.HIGH_3
        elif percent < 95:
            color = bcolors.HIGHER_4
        else:
            color = bcolors.HIGHEST_5
    content = color + f'\r|{bar}| {rotator[int(progress / total * len(rotator) * 10) % len(rotator)]} {percent:06.2f}%' + bcolors.ENDC__1
    # content = f'\r|{bar}| {rotator[int(progress / total * len(rotator) * 10) % len(rotator)]} {percent:06.2f}%'
    print(content, end='', flush=True)

if __name__ == "__main__":
    total = 100
    for i in range(total + 1):
        progress_bar(i, total)
        time.sleep(0.05)
    print()  # Move to next line after completion