#!/usr/bin/env python3
"""
Log Analysis Script
Analyzes timing data from logs.txt and calculates averages and standard deviations
for neural network inference, sensor reads, and ESP-NOW transmit times.

How to use:
1. Connect the MCU (in this case the rc car controller) to the computer.
2. Capture the log output using `pio device monitor --filter log2file` or copy-paste from the terminal.
3. Save the logs to a file named `logs.txt` in the project root directory.
4. Run this script: `python scripts/analyze_logs.py`
"""

import re
import numpy as np
from pathlib import Path

def analyze_logs(log_file_path):
    """Parse logs and extract timing data"""

    # Initialize data collections
    nn_times = []
    left_sensor_times = []
    center_sensor_times = []
    right_sensor_times = []
    espnow_times = []

    # Regular expressions for each timing type
    nn_pattern = r'\[TIMING\] Neural network inference \+ action: (\d+) us'
    left_sensor_pattern = r'\[SENSOR\] Left VL6180X read: (\d+) us'
    center_sensor_pattern = r'\[SENSOR\] Center VL6180X read: (\d+) us'
    right_sensor_pattern = r'\[SENSOR\] Right VL6180X read: (\d+) us'
    espnow_pattern = r'\[TX TIME\] ESP-NOW transmit: (\d+) us'

    try:
        with open(log_file_path, 'r') as f:
            for line in f:
                line = line.strip()

                # Extract neural network times
                nn_match = re.search(nn_pattern, line)
                if nn_match:
                    nn_times.append(int(nn_match.group(1)))

                # Extract sensor times
                left_match = re.search(left_sensor_pattern, line)
                if left_match:
                    left_sensor_times.append(int(left_match.group(1)))

                center_match = re.search(center_sensor_pattern, line)
                if center_match:
                    center_sensor_times.append(int(center_match.group(1)))

                right_match = re.search(right_sensor_pattern, line)
                if right_match:
                    right_sensor_times.append(int(right_match.group(1)))

                # Extract ESP-NOW times
                espnow_match = re.search(espnow_pattern, line)
                if espnow_match:
                    espnow_times.append(int(espnow_match.group(1)))

    except FileNotFoundError:
        print(f"Error: Log file '{log_file_path}' not found")
        return

    # Calculate statistics for each category
    categories = [
        ("Neural Network Inference", nn_times),
        ("Left VL6180X Sensor", left_sensor_times),
        ("Center VL6180X Sensor", center_sensor_times),
        ("Right VL6180X Sensor", right_sensor_times),
        ("ESP-NOW Transmit", espnow_times)
    ]

    print("=" * 60)
    print("LOG ANALYSIS RESULTS")
    print("=" * 60)

    for name, times in categories:
        if times:
            avg = np.mean(times)
            std = np.std(times)
            count = len(times)
            min_val = np.min(times)
            max_val = np.max(times)

            print(f"\n{name}:")
            print(f"  Count: {count}")
            print(f"  Average: {avg:.1f} us")
            print(f"  Std Dev: {std:.1f} us")
            print(f"  Min: {min_val} us")
            print(f"  Max: {max_val} us")
        else:
            print(f"\n{name}: No data found")

    print("\n" + "=" * 60)

if __name__ == "__main__":
    # Look for logs.txt in the project root directory
    log_file = Path(__file__).parent.parent / "logs.txt"
    if not log_file.exists():
        print(f"logs.txt not found at {log_file}")
    else:
        analyze_logs(log_file)