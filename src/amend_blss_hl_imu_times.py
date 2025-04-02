# This program fixes the xsen 630 IMU data host timestamps assuming the regular interval
# as we don't record the sensor timestamps or its mapping.

# In BLSS HL, the IMU sensor times and host times suffer from obvious jittering,
# and the sensor times at header.stamp are slightly earlier than host times at ros msg recording time.
# The good news is that the time gaps between arbitrary long IMU msg seqs are very 
# close to the expected values computed from the index gaps,
# meaning that the IMU msg sensor times are indeeded affected by ONLY white noise.

# The VLP16 host times also suffer from obvious jittering.
# The VLP16 sensor times show very small jittering, so I assume it is stamped according to the sensor clock.
# The VLP16 sensor times are mostly 0.1 seconds earlier than VLP16 host times.

import numpy as np
import os
import rospy
from sensor_msgs.msg import Imu
import rosbag

def smooth_imu_timestamps_bottom(timestamps, nominal_interval=0.0025):
    # This function assumes the entry at the bottom of the cliff have good timestamps.
    # We assume the times at jump indices correspond to the true times,
    # as data are buffered and then relayed.
    # For each index before jump index, back propagate the times until the previous jump index by
    # assigning the previous entries with time at jump index - (jump index - entry index) * interval
    timestamps = np.array(timestamps)
    diffs = np.diff(timestamps)
    jump_indices = list(np.where(diffs > 4 * nominal_interval)[0])
    jump_indices.append(len(timestamps) - 1)

    corrected_timestamps = timestamps.copy()
    total_bads = 0

    for j, jump_idx in enumerate(jump_indices):
        # Note values at jump_idx are at the bottom of the cliff.
        if j == 0:
            for i in range(jump_idx - 1, -1, -1):
                predicted_time = timestamps[jump_idx] - (jump_idx - i) * nominal_interval
                corrected_timestamps[i] = predicted_time
            continue
        prev_jump_idx = jump_indices[j - 1]
        start_idx = prev_jump_idx + 1
        predicted_start_time = timestamps[jump_idx] - (jump_idx - start_idx) * nominal_interval
        interval = nominal_interval
        if predicted_start_time <= corrected_timestamps[prev_jump_idx]:
            interval = (timestamps[jump_idx] - timestamps[prev_jump_idx]) / (jump_idx - prev_jump_idx)

        for i in range(jump_idx - 1, start_idx - 1, -1):
            predicted_time = timestamps[jump_idx] - (jump_idx - i) * interval
            corrected_timestamps[i] = predicted_time

        # sanity check
        predicted_time = timestamps[jump_idx] - (jump_idx - prev_jump_idx) * nominal_interval
        dt = predicted_time - corrected_timestamps[prev_jump_idx]
        if abs(dt) > nominal_interval:
            print(f"Warn: Inconsistent time diff {prev_jump_idx}: original {corrected_timestamps[prev_jump_idx]:.6f}, "
                    f"back predicted {predicted_time:.6f}, diff {dt:.6f}")
            total_bads += 1

        prev_jump_idx = jump_idx
    if total_bads:
        print(f'Warn: Cliff bottom total inconsistency warnings {total_bads} out of {len(timestamps)} with ratio {total_bads / len(timestamps):.4f}')
    diffs2 = np.diff(corrected_timestamps)
    neg_indices = np.where(diffs2 <= 0)[0]
    if len(neg_indices):
        print(f'Error: Negative time gaps at {neg_indices}')
    if len(corrected_timestamps) != len(timestamps):
        print(f'Error: Missing time entries {len(corrected_timestamps)} and {len(timestamps)}')
    return corrected_timestamps


def smooth_imu_timestamps_top(timestamps, nominal_interval=0.0025):
    # This function assumes the entry at the top of the cliff have good timestamps.
    # We assume the times at jump indices correspond to the true times,
    # as data are buffered and then relayed.
    # For each index after the jump index, forward propagate the times until the next jump index by
    # assigning the subsequent entries with time at jump index + (entry index - jump index) * interval.
    timestamps = np.array(timestamps)
    diffs = np.diff(timestamps)
    jump_indices = list(np.where(diffs > 4 * nominal_interval)[0])
    top_indices = [0]
    for jump in jump_indices:
        top_indices.append(jump + 1)

    corrected_timestamps = timestamps.copy()
    total_bads = 0
    for j, jump_idx in enumerate(top_indices):
        # Note values at jump_idx are on the top of the cliff.
        if j + 1 == len(top_indices):
            # For the end, increase values following the jump index
            for id in range(jump_idx + 1, len(timestamps), 1):
                corrected_timestamps[id] = timestamps[jump_idx] + (id - jump_idx) * nominal_interval
            break
        end_idx = top_indices[j + 1] # exclusive
        predicted_time = timestamps[jump_idx] + (end_idx - 1 - jump_idx) * nominal_interval
        interval = nominal_interval
        if predicted_time >= timestamps[end_idx]:
            adjusted_interval = (timestamps[end_idx] - timestamps[jump_idx]) / (end_idx - jump_idx)
            interval = adjusted_interval

        for id in range(jump_idx + 1, end_idx, 1):
            corrected_timestamps[id] = timestamps[jump_idx] + (id - jump_idx) * interval

        # sanity check
        predicted_time = timestamps[jump_idx] + (end_idx - jump_idx) * nominal_interval
        dt = timestamps[end_idx] - predicted_time
        if abs(dt) > nominal_interval:
            print(f"Warn: Inconsistent time diff {end_idx}: original {timestamps[end_idx]:.6f}, "
                    f"back predicted {predicted_time:.6f}, diff {dt:.6f}")
            total_bads += 1
    if total_bads:
        print(f'Warn: Cliff top total inconsistency warnings {total_bads} out of {len(timestamps)} with ratio {total_bads / len(timestamps):.4f}')
    diffs2 = np.diff(corrected_timestamps)
    neg_indices = np.where(diffs2 <= 0)[0]
    if len(neg_indices):
        print(f'Error: Negative time gaps at {neg_indices}')
    if len(corrected_timestamps) != len(timestamps):
        print(f'Error: Missing time entries {len(corrected_timestamps)} and {len(timestamps)}')
    return corrected_timestamps


def load_imu_data(bagfile, topic):
    bag = rosbag.Bag(bagfile, 'r')
    timestamps = []
    messages = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        timestamps.append(msg.header.stamp.to_sec())
        messages.append(msg)
    bag.close()
    return timestamps, messages


def save_txt(file_path, original_timestamps, smoothed_timestamps_top, smoothed_timestamps_bottom):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'w') as f:
        f.write('#original smoothed_top smoothed_bottom\n')
        for orig, top, bottom in zip(original_timestamps, smoothed_timestamps_top, smoothed_timestamps_bottom):
            f.write(f"{orig:.9f} {top:.9f} {bottom:.9f}\n")


def save_imu_to_bag(bagfile, new_topic, messages, smoothed_timestamps):
    with rosbag.Bag(bagfile, 'a') as outbag:
        topics_info = outbag.get_type_and_topic_info().topics
        existing_topics = topics_info.keys()
        if new_topic in existing_topics:
            print(f"Warning: Topic {new_topic} already exists in {bagfile}. Skipping modification.")
            return
        for msg, new_time in zip(messages, smoothed_timestamps):
            nt = rospy.Time.from_sec(new_time)
            msg.header.stamp = nt
            outbag.write(new_topic, msg, t=nt)


if __name__ == "__main__":
    import sys

    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} bagfile topic")
        sys.exit(1)

    bagfile = sys.argv[1]
    topic = sys.argv[2]
    # With preliminary tests, it seems smooth_imu_timestamps_top has much fewer inconsistency than
    # smooth_imu_timestamps_bottom, 0.0017 vs 0.035.
    timestamps, messages = load_imu_data(bagfile, topic)
    smoothed_timestamps_bottom = smooth_imu_timestamps_bottom(timestamps, 0.0025)
    smoothed_timestamps_top = smooth_imu_timestamps_top(timestamps, 0.0025)

    dirname = os.path.dirname(bagfile)
    basename = os.path.splitext(os.path.basename(bagfile))[0]
    imu_file = os.path.join(dirname, topic.strip('/').replace('/', '_') + "_smoothed.txt")
    save_txt(imu_file, timestamps, smoothed_timestamps_top, smoothed_timestamps_bottom)

    new_topic = topic + '_dejit_top'
    save_imu_to_bag(bagfile, new_topic, messages, smoothed_timestamps_top)
    print(f'Appended {new_topic} to {bagfile}')

    new_topic = topic + '_dejit_bot'
    save_imu_to_bag(bagfile, new_topic, messages, smoothed_timestamps_bottom)
    print(f'Appended {new_topic} to {bagfile}')
