# Estimate the small lidar IMU time offset by repeating lidar-inertial odometry and td rot calib

# input rosbag
# 1 extract imu from rosbag into txt dejit_bot
# 2 run fastlio
# 3 run td rot calib matlab scripts
# 4 repeat 1 and 2 until td inc is small

import argparse
import io
import os
import sys
import subprocess
import matlab.engine


def get_imu_posesensor_offset(script_path, imufile, posefile, topic_output_dir, i_R_p_init, logfile):
    eng = matlab.engine.start_matlab()
    eng.cd(script_path, nargout=0)
    out = io.StringIO()
    err = io.StringIO()
    iwinsize = 10
    pwinsize = 1
    [i_R_p, td, td_list] = eng.estimateRotAndTdByPose(imufile, posefile, topic_output_dir, i_R_p_init, iwinsize, pwinsize, nargout=3, stdout=out, stderr=err)
    eng.quit()
    with open(logfile, 'a') as f:
        f.write(out.getvalue())
        f.write(err.getvalue())
    return i_R_p, td, td_list


def isFileOk(datafile):
    if not os.path.isfile(datafile):
        print('Warning: {} does not exist'.format(datafile))
        return False
    with open(datafile, 'r') as f:
        lines = f.readlines()
    if len(lines) < 5:
        print('Warning: {} has only {} lines'.format(datafile, len(lines)))
        return False
    return True


def fastlio(bagfile, imu_topic, state_filename, save_dir,
            start_time=None, end_time=None, td_lidar_to_imu=0.0):
    """
    Run the fastlio for a rosbag file.

    Args:
    bagfile: str, the path to the rosbag file
    imu_topic: str, the IMU topic name
    state_filename: str, the basename of the file to save the state of the robot
    save_dir: str, the path to the directory to save the output files for this bag
    start_time: float, the start time of the msgs to be used. If None, the start time is set to 0.
    end_time: float, If None, the end time is set to 0.
    """
    if start_time is not None:
        msg_start_time = '{:.09f}'.format(start_time)
    else:
        msg_start_time = "0"
    
    if end_time is not None:
        msg_end_time = '{:.09f}'.format(end_time)
    else:
        msg_end_time = "0"

    if '/imu/data' in imu_topic:
        configyamlname = "vlp16_kuangye.yaml"
    else:
        print('Error: Unknown IMU topic {}'.format(imu_topic))
        return
    # get the abs path of this script
    this_script_path = os.path.abspath(__file__)
    fastlio_dir = os.path.dirname(os.path.dirname(this_script_path))
    script_path = os.path.join(fastlio_dir, 'shell/odom_launch.sh')
    result = subprocess.run([script_path, configyamlname, bagfile, fastlio_dir, 
                             msg_start_time, msg_end_time, 
                             state_filename, save_dir, str(td_lidar_to_imu)],
                             capture_output=True, text=True)

    logfilename = state_filename.split('.')[0] + '.log'
    logfile = os.path.join(save_dir, logfilename)
    with open(logfile, 'a') as f:
        f.write("Python subprocess stdout:\n{}\n".format(result.stdout))
        f.write("Python subprocess stderr:\n{}\n".format(result.stderr))
        f.write("Python subprocess return code: {}\n".format(result.returncode))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run lidar-IMU time offset calibration")
    parser.add_argument("bagfile", type=str, help="Path to the ROS bag file")
    parser.add_argument("imufile", type=str, help="Path to IMU data file")
    parser.add_argument("save_dir", type=str, help="Directory to save results")
    parser.add_argument("--start_time", default=0.0, type=float, help="Start time in seconds")
    parser.add_argument("--end_time", default=0.0, type=float, help="End time in seconds")

    parser.add_argument(
        "--script_path",
        type=str,
        default="/home/jhuai/Documents/lidar/td_rot_calib",
        help="Path to time calibration MATLAB scripts",
    )

    args = parser.parse_args()

    td_lidar_to_imu = 0.0
    for i in range(6):
        state_filename = f"scan_states_{i}.txt"
        save_dir_iter = f'{args.save_dir}/{i}'
        fastlio(
            args.bagfile,
            "/imu/data_dejit_bot",
            state_filename,
            save_dir_iter,
            start_time=args.start_time,
            end_time=args.end_time,
            td_lidar_to_imu=td_lidar_to_imu,
        )

        train = "imu"
        query = "vlp16"
        t_R_q_init = matlab.double([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])

        lofile = os.path.join(save_dir_iter, f"scan_states_odom.txt")
        print(f"Align target {args.imufile} source {lofile}")
        if not isFileOk(lofile):
            continue

        logfile = os.path.join(args.save_dir, f"td_rot_calib.txt")
        i_R_p, td, td_list = get_imu_posesensor_offset(
            args.script_path,
            args.imufile,
            lofile,
            args.save_dir,
            t_R_q_init,
            logfile,
        )
        td_lidar_to_imu -= td # jhuai: we use minus as it converges, but don't know why.
        print(f"Iter {i}: td {td} accum td {td_lidar_to_imu} i_R_p {i_R_p}")

