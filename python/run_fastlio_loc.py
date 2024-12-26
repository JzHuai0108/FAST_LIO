import os
import numpy as np
import rospy
import rosbag
import scipy.spatial.transform
import subprocess
import time

def favor_mti3dk(bagfile):
    start = time.time()
    bag = rosbag.Bag(bagfile) # Opening a rosbag often takes very long.
    end = time.time()

    start = time.time()
    topics = bag.get_type_and_topic_info().topics
    end = time.time()

    bag.close()
    if '/mti3dk/imu' in topics:
        return '/mti3dk/imu'
    if '/x36d/imu_raw' in topics:
        return '/x36d/imu_raw'
    else:
        print('Error: No mti3dk or x36d IMU topic found in the bag')
        return ''

def parse_time(timestr):
    parts = timestr.split('.')
    secs = int(parts[0])
    l = len(parts[1])
    nsecs = int(parts[1]) * 10**(9 - l)
    return rospy.Time(secs, nsecs)

def fastlioloc(bagfile, imu_topic, tls_dir, tls_dist_thresh, state_filename, save_dir, init_pose_file, start_time=None):
    """
    Run the fastlio localization for a rosbag file.

    Args:
    bagfile: str, the path to the rosbag file
    imu_topic: str, the IMU topic name
    tls_dir: str, the path to the directory containing the TLS data
    state_filename: str, the basename of the file to save the state of the robot
    save_dir: str, the path to the directory to save the output files for this bag
    init_pose_file: str, the path to the file containing the initial pose and velocity for this bag
    start_time: rospy.Time, the start time of the bag file. If None, the start time is set to 0.
    """
    if start_time is not None:
        bag_start_time = '{}.{:09d}'.format(start_time.secs, start_time.nsecs)
    else:
        bag_start_time = "0"

    if 'mti3dk' in imu_topic:
        configyamlname = "hesai32_mti3dk_handheld.yaml"
    elif 'x36d' in imu_topic:
        configyamlname = "hesai32_x36d_handheld.yaml"
    elif 'zed2i' in imu_topic:
        configyamlname = "hesai32_zed2i_handheld.yaml"
    else:
        print('Error: Unknown IMU topic {}'.format(imu_topic))
        return
    # get the abs path of this script
    this_script_path = os.path.abspath(__file__)
    fastlio_dir = os.path.dirname(os.path.dirname(this_script_path))
    script_path = os.path.join(fastlio_dir, 'shell/loc_launch.sh')

    result = subprocess.run([script_path, configyamlname, bagfile, fastlio_dir, tls_dir, 
                             init_pose_file, bag_start_time, str(tls_dist_thresh), state_filename, save_dir],
                            capture_output=True, text=True)
    logfilename = state_filename.split('.')[0] + '.log'
    logfile = os.path.join(save_dir, logfilename)
    with open(logfile, 'a') as f:
        f.write("Python subprocess stdout:\n{}\n".format(result.stdout))
        f.write("Python subprocess stderr:\n{}\n".format(result.stderr))
        f.write("Python subprocess return code: {}\n".format(result.returncode))

def save_init_state(pose, vel, t, init_pose_file):
    """
    Save the initial state of the robot to a file.
    
    Args:
    pose: list of 7 floats, the position and quaternion of the robot
    vel: list of 3 floats, the velocity of the robot
    t: rospy.Time, the time of the initial state
    init_pose_file: str, the path to the file to save the initial state
    """
    p = pose[:3]
    q = pose[3:]
    # check norm of q close to 1
    if abs(np.linalg.norm(q) - 1) > 1e-3:
        print('Error: The norm of the quaternion is not close to 1')
        return

    T = np.eye(4)
    T[:3, :3] = scipy.spatial.transform.Rotation.from_quat(q).as_matrix()
    T[:3, 3] = p
    with open(init_pose_file, 'w') as f:
        for i in range(3):
            f.write('{} {} {} {}\n'.format(T[i, 0], T[i, 1], T[i, 2], T[i, 3]))
        f.write('{} {} {}\n'.format(vel[0], vel[1], vel[2]))
        f.write('{}.{:09d}\n'.format(t.secs, t.nsecs))

def read_forward_result(forwardtxt, queryt, delim=','):
    """
    Read the forward result file and find the closest time to the query time.
    
    Args:
    forwardtxt: str, the path to the forward result file
    queryt: rospy.Time, the time to query the closest time in the file
    """
    lines = []
    times = []
    with open(forwardtxt, 'r') as f:
        for l in f:
            if l.startswith('#') or l.strip() == '':
                continue
            parts = l.strip().split(delim)
            t = parse_time(parts[0])
            times.append(t)
            lines.append(parts[1:])
    # find the closest time
    time_diffs = [abs((t - queryt).to_sec()) for t in times]
    idx = np.argmin(time_diffs)
    if time_diffs[idx] > 0.1:
        print(f'Warn: Could not find the closest time to {queryt.secs}.{queryt.nsecs:09d} in {forwardtxt}.')
        print(f'The closest time is {times[idx].secs}.{times[idx].nsecs:09d} with gap {time_diffs[idx]} secs.')
    return lines[idx][:7], lines[idx][7:10]

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser("Run fastlio localization for front and back segment of rosbags in forward and backward mode",
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("reftraj_dir", type=str, help="Directory containing reference trajectory files")
    parser.add_argument("bagdir", type=str, help="Directory containing rosbags of corrected timestamps")
    parser.add_argument("tls_dir", type=str, help="Directory containing tls data")
    parser.add_argument("outputdir", type=str, help="Output directory")
    parser.add_argument("--tls_dist_thresh", type=float, default=8.0, help="Threshold on distance to TLS trajectory to abort LIO localization")
    args = parser.parse_args()

    baglist = []
    if args.bagdir.endswith('.bag'):
        baglist = [args.bagdir]
    else:
        for root, dirs, files in os.walk(args.bagdir):
            for file in files:
                if file.endswith('.bag') and file.startswith('data') and file[4].isdigit():
                    baglist.append(os.path.join(root, file))

    # Map bag files to their corresponding reference trajectories
    bag2traj = {}
    for bagname in baglist:
        basename = os.path.basename(bagname)
        dirname = os.path.dirname(bagname)
        run_name, _ = os.path.splitext(basename)
        date = os.path.basename(dirname)
        reffile = os.path.join(args.reftraj_dir, date, run_name, "tls_T_xt32.txt")
        if not os.path.isfile(reffile):
            print(f"Warning: Reference trajectory {reffile} for {bagname} not found")
            continue
        bag2traj[bagname] = reffile
    print(f"Found {len(bag2traj)} bag files with reference trajectories")

    # Process the front and back segment of every rosbag
    bagfiles = list(bag2traj.keys())
    bagfiles = sorted(bagfiles)
    for i, bagfile in enumerate(bagfiles):
        print(f'Processing bag {i+1}/{len(bagfiles)}: {bagfile}')
        reftraj_file = bag2traj[bagfile]
        # load the reftraj file in TUM format
        times = []
        poses = []
        with open(reftraj_file, 'r') as f:
            for l in f:
                if l.startswith('#') or l.strip() == '':
                    continue
                parts = l.strip().split()
                mt = parse_time(parts[0])
                times.append(mt)
                poses.append(parts[1:])
        # check if there exist a large gap in times
        time_diffs = np.diff([t.to_sec() for t in times])
        max_diff_idx = np.argmax(time_diffs)
        max_diff = time_diffs[max_diff_idx]
        # find the second max
        time_diffs[max_diff_idx] = 0
        second_max_diff = np.max(time_diffs)

        front_endidx = max_diff_idx
        single = True
        if second_max_diff > 0.5:
            print(f'Error: Found two large gaps in the reference trajectory of {bagfile}')
            continue
        if max_diff < 0.5:
            print(f'Info: No large gap <={max_diff:.5f} in the reference trajectory of {bagfile}')
            front_endidx = len(times) - 1
            single = True
        else:
            print(f'Info: Found a large gap {max_diff:.5f} in the reference trajectory of {bagfile}')
            front_endidx = max_diff_idx
            print('time at maxdiff = {}.{:09d} and next time = {}.{:09d}'.format(
                  times[max_diff_idx].secs, times[max_diff_idx].nsecs, times[max_diff_idx+1].secs, times[max_diff_idx+1].nsecs))
            single = False

        d = os.path.dirname(reftraj_file)
        run = os.path.basename(d)
        date = os.path.basename(os.path.dirname(d))

        # get mirror time
        bag = rosbag.Bag(bagfile)
        tmirror = rospy.Time(bag.get_end_time())
        bag.close()
        t2 = rospy.Duration(tmirror.secs, tmirror.nsecs) * 2
        tmirror2 = rospy.Time(t2.secs, t2.nsecs)

        # get imu topic
        lidar_topic = '/hesai/pandar'
        imu_topic = favor_mti3dk(bagfile)
        whole_seq_tls_dist_thresh = 100

        # forward processing of the front segment
        save_dir = os.path.join(args.outputdir, date, run, 'front')
        init_pose_file = os.path.join(save_dir, "forward_init_pose.txt")
        save_init_state(poses[0], [0, 0, 0], times[0], init_pose_file)
        state_filename = "forward_states.txt"
        print('Processing the front segment of {} with IMU topic {}'.format(bagfile, imu_topic))

        # when the TLS fully covers the seq, we set the tls dist threshold large.
        front_seq_tls_dist_thresh = whole_seq_tls_dist_thresh if single else args.tls_dist_thresh
        fastlioloc(bagfile, imu_topic, args.tls_dir, front_seq_tls_dist_thresh, state_filename, save_dir, init_pose_file)
        forwardfronttxt = os.path.join(save_dir, state_filename)
        frontendtime = times[front_endidx]
        frontendpose, frontendv = read_forward_result(forwardfronttxt, frontendtime)

        # backward processing of the front segment
        init_pose_file = os.path.join(save_dir, "backward_init_pose.txt")
        state_filename = "backward_states.txt"
        mirrortime = tmirror2 - frontendtime
        negativev = [-float(v) for v in frontendv]
        save_init_state(frontendpose, negativev, mirrortime, init_pose_file)
        rev_front_bag = os.path.join(save_dir, 'reversed.bag')
        print('Processing the reversed front segment of {} with IMU topic {}'.format(rev_front_bag, imu_topic))
        fastlioloc(rev_front_bag, imu_topic, args.tls_dir, front_seq_tls_dist_thresh, state_filename, save_dir, init_pose_file)
        # The below two lines are used in generating the bag of the reversed front segment.
        # endpose = poses[front_endidx] # the pose for the first scan of the reversed bag.
        # reverse_bag2(bagfile, rev_front_bag, lidar_topic, imu_topic, times[0] - buffer, times[front_endidx], endpose)

        if not single:
            # backward processing of the back segment
            save_dir = os.path.join(args.outputdir, date, run, 'back')
            rev_back_bag = os.path.join(save_dir, 'reversed.bag')
            init_pose_file = os.path.join(save_dir, "backward_init_pose.txt")
            state_filename = "backward_states.txt"
            mirrortime = tmirror2 - times[-1]
            save_init_state(poses[-1], [0, 0, 0], mirrortime, init_pose_file)
            print('Processing the reversed back segment of {} with IMU topic {}'.format(rev_back_bag, imu_topic))
            fastlioloc(rev_back_bag, imu_topic, args.tls_dir, args.tls_dist_thresh, state_filename, save_dir, init_pose_file)

            mirrorbackendtime = tmirror2 - times[front_endidx+1]
            mirrorbackendtime = rospy.Time(mirrorbackendtime.secs, mirrorbackendtime.nsecs)
            backwardbacktxt = os.path.join(save_dir, state_filename)
            backendpose, backendv = read_forward_result(backwardbacktxt, mirrorbackendtime)
            # forward processing of the back segment
            init_pose_file = os.path.join(save_dir, "forward_init_pose.txt")
            state_filename = "forward_states.txt"
            negativev = [-float(v) for v in backendv]
            save_init_state(backendpose, negativev, times[front_endidx+1], init_pose_file)
            print('Processing the back segment of {} with IMU topic {}'.format(bagfile, imu_topic))
            fastlioloc(bagfile, imu_topic, args.tls_dir, args.tls_dist_thresh, state_filename, save_dir, init_pose_file, times[front_endidx+1])
            # The below two lines are used in generating the bag of the reversed front segment.
            # endpose = poses[-1]
            # reverse_bag2(bagfile, rev_back_bag, lidar_topic, imu_topic, times[front_endidx+1] - buffer, times[-1] + buffer, endpose)
    print('Done')
