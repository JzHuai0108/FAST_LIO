# This program fixes the BLSS HL IMU times by using a custom algorithm.

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

def smooth_imu_timestamps(timestamps, nominal_interval=0.0025):
    timestamps = np.array(timestamps)
    diffs = np.diff(timestamps)
    jump_indices = list(np.where(diffs > 4 * nominal_interval)[0])
    jump_indices.append(len(timestamps) - 1)

    # We assume the times at jump indices correspond to the true times, as data are buffered and then relayed.
    # For each jump index, back propagate the times until the previous jump index by
    # assigning the previous entries with time at jump index - (jump index - entry index) * interval
    corrected_timestamps = timestamps.copy()
    total_bads = 0
    prev_jump_idx = -1
    for jump_idx in jump_indices:
        start_idx = prev_jump_idx + 1
        for i in range(jump_idx - 1, start_idx - 1, -1):
            predicted_time = timestamps[jump_idx] - (jump_idx - i) * nominal_interval
            corrected_timestamps[i] = predicted_time
        # check the value at prev jump idx
        if prev_jump_idx >= 0:
            while corrected_timestamps[start_idx] - corrected_timestamps[prev_jump_idx] <= 0:
                for k in range(start_idx, jump_idx + 1, 1):
                    corrected_timestamps[k] = corrected_timestamps[k] + nominal_interval

            # sanity check
            predicted_time = timestamps[jump_idx] - (jump_idx - prev_jump_idx) * nominal_interval
            dt = corrected_timestamps[prev_jump_idx] - predicted_time
            if abs(dt) > nominal_interval:
                print(f"Warn: Inconsistent time diff {prev_jump_idx}: original {corrected_timestamps[prev_jump_idx]:.6f}, "
                      f"back predicted {predicted_time:.6f}, diff {dt:.6f}")
                total_bads += 1

        prev_jump_idx = jump_idx
    if total_bads:
        print(f'Warn: Total inconsistency warnings {total_bads} out of {len(timestamps)} with ratio {total_bads / len(timestamps):.4f}')
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


def save_txt(file_path, original_timestamps, smoothed_timestamps):
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, 'w') as f:
        for orig, smooth in zip(original_timestamps, smoothed_timestamps):
            f.write(f"{orig:.9f} {smooth:.9f}\n")


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
    timestamps, messages = load_imu_data(bagfile, topic)
    smoothed_timestamps = smooth_imu_timestamps(timestamps, 0.0025)
    
    dirname = os.path.dirname(bagfile)
    basename = os.path.splitext(os.path.basename(bagfile))[0]
    imu_file = os.path.join(dirname, topic.strip('/').replace('/', '_') + "_smoothed.txt")
    save_txt(imu_file, timestamps, smoothed_timestamps)

    new_topic = topic + '_dejitter'
    save_imu_to_bag(bagfile, new_topic, messages, smoothed_timestamps)
    print(f'Appended {new_topic} to {bagfile}')
