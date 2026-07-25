#!/usr/bin/env python3
"""
fix_gesture_timing.py

A recorded gesture samples /joint_states continuously from the moment
/start_recording is called until /stop_save_recording is called -
including any real-world time spent between deliberate moves (reading,
thinking, typing). JointTrajectoryController paces playback against
each point's recorded timestamp, so a recording session that took
several minutes of wall-clock time will replay just as slowly.

This script removes ONLY the genuinely idle stretches - consecutive
samples where the joint positions barely changed - and caps any
remaining pause to a short fixed duration. Every real motion segment
keeps its exact original timing and speed, so playback still
reproduces the same contact dynamics as the live demonstration.

Do NOT uniformly rescale every timestamp by one factor instead - that
speeds up the real motion segments too, and can change how the robot
physically interacts with objects during playback.

Usage:
    python3 fix_gesture_timing.py <input.yaml> <output.yaml> \
        [--position-threshold 0.001] [--max-hold 0.3]

Example:
    python3 fix_gesture_timing.py align_t_task_raw.yaml align_t_task.yaml
"""

import argparse
import yaml


def fix_gesture_timing(input_path, output_path,
                        position_threshold=0.001, max_hold=0.3):
    with open(input_path, 'r') as f:
        data = yaml.safe_load(f)

    points = data.get('points', [])
    if not points:
        raise ValueError(f"No points found in {input_path}")

    original_duration = points[-1]['time_from_start'] - points[0]['time_from_start']

    filtered = [points[0]]
    for i in range(1, len(points)):
        prev_pos = filtered[-1]['positions']
        curr_pos = points[i]['positions']
        delta = max(abs(a - b) for a, b in zip(curr_pos, prev_pos))
        if delta > position_threshold:
            filtered.append(points[i])

    new_times = [0.0]
    for i in range(1, len(filtered)):
        original_gap = filtered[i]['time_from_start'] - filtered[i - 1]['time_from_start']
        capped_gap = min(original_gap, max_hold)
        new_times.append(new_times[-1] + capped_gap)

    for i, p in enumerate(filtered):
        p['time_from_start'] = round(new_times[i], 3)

    data['points'] = filtered

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    print(f"Input:  {len(points)} points, {original_duration:.2f}s")
    print(f"Output: {len(filtered)} points, {new_times[-1]:.2f}s")
    print(f"Saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Strip idle/thinking time out of a recorded gesture, "
                    "preserving the exact speed of every real motion segment."
    )
    parser.add_argument("input", help="Path to the raw recorded gesture YAML")
    parser.add_argument("output", help="Path to write the cleaned gesture YAML")
    parser.add_argument(
        "--position-threshold", type=float, default=0.001,
        help="Radians of joint movement below which two consecutive "
             "samples are treated as 'the same position' (default: 0.001)"
    )
    parser.add_argument(
        "--max-hold", type=float, default=0.3,
        help="Longest pause in seconds kept between waypoints after "
             "cleanup (default: 0.3)"
    )
    args = parser.parse_args()
    fix_gesture_timing(
        args.input, args.output, args.position_threshold, args.max_hold
    )


if __name__ == "__main__":
    main()
