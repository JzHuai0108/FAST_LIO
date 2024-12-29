#!/bin/bash
# generate ref trajs in the UTM50N frame for some seqs not in the dataset.
# warn: depends on ROS1.
import os
import shutil

from run_lioloc_w_ref import fastlioloc, get_mirror_time


def get_lidar_td(mti3dk_to_ins, xt32_to_ins):
    # because KISS-ICP results timestamp is at t, whereas the motion is at t + dt/2, dt = 0.1 for XT32;
    # and we used KISS-ICP results to align lidar and GNSS_INS data.
    # lidar clock + xt32_to_mti3dk = mti3dk imu clock
    # for backward sequences, time offsets should be negated.
    correct_xt32_to_ins = [x - 0.05 for x in xt32_to_ins]
    xt32_to_mti3dk = [correct - mti for correct, mti in zip(correct_xt32_to_ins, mti3dk_to_ins)]
    xt32_to_mti3dk_bwd = [-offset for offset in xt32_to_mti3dk]

    return xt32_to_mti3dk


def lioloc_seqs(seqdir, bagnames, xt32_to_mti3dk, tls_dir, outlocdir):
    single_seq_tls_dist_thresh = 100.0
    tls_dist_thresh = 8.0
    for bi, bagname in enumerate(bagnames):
        td_lidar_to_imu = xt32_to_mti3dk[bi]
        date, run = bagname.split('/')
        run = run.split('_')[0]
        # verify that the initial poses are ready
        front_init_pose_file = os.path.join(outlocdir, date, run, 'front/forward_init_pose.txt')
        back_init_pose_file = os.path.join(outlocdir, date, run, "back/backward_init_pose.txt")
        single = True
        assert os.path.isfile(front_init_pose_file)
        if os.path.isfile(back_init_pose_file):
            single = False

        # run fastlioloc for the front segment
        bagfile = os.path.join(seqdir, bagname + '.bag')
        state_filename = "forward_states.txt"
        imu_topic = '/mti3dk/imu'
        print('Processing the front segment of {} with IMU topic {}'.format(bagfile, imu_topic))

        # when the TLS fully covers the seq, we set the tls dist threshold large.
        front_seq_tls_dist_thresh = single_seq_tls_dist_thresh if single else tls_dist_thresh
        save_dir = os.path.join(outlocdir, date, run, 'front')

        fastlioloc(bagfile, imu_topic, tls_dir, front_seq_tls_dist_thresh, 
                   state_filename, save_dir, front_init_pose_file, td_lidar_to_imu=td_lidar_to_imu)
        if not single:
            # run fastlioloc for the reversed back segment
            save_dir = os.path.join(outlocdir, date, run, 'back')
             # jhuai: To save time, we used an early version of the reversed bag 
             # which was derived from the fairly early stage datax_aligned.bag.
            rev_back_bag = os.path.join(save_dir, run + '_aligned.bag')
            init_pose_file = os.path.join(save_dir, "backward_init_pose.txt")
            state_filename = "backward_states.txt"
            print('Processing the reversed back segment of {} with IMU topic {}'.format(rev_back_bag, imu_topic))
            fastlioloc(rev_back_bag, imu_topic, tls_dir, tls_dist_thresh, 
                       state_filename, save_dir, init_pose_file, td_lidar_to_imu=-td_lidar_to_imu)

    # Finally, run cascaded_pgo for each seq, will be done in another program.


if __name__ == "__main__":
    seqdir = '/media/pi/BackupPlus/jhuai/data/homebrew/ebike'
    outlocdir = '/media/pi/BackupPlus/jhuai/results/front_back_snapshots'
    bagnames = ['20231105/data1_aligned', '20231105_aft/data3_aligned', '20231105_aft/data6_aligned', '20231109/data2_aligned']
    tls_dir = "/media/pi/BackupPlus/jhuai/data/homebrew/whu_tls_1030"
    # The time offsets are read from the time corrections.csv of the dataset
    # for the seq closest and earlier in time to the chosen seqs.
    mti3dk_to_ins = [0.073072527, 0.064948506, 0.067292968, 0.071477062]
    xt32_to_ins = [0.027011781, 0.022238177, 0.024842864, 0.031391372]
    xt32_to_mti3dk = get_lidar_td(mti3dk_to_ins, xt32_to_ins)
    lioloc_seqs(seqdir, bagnames, xt32_to_mti3dk, tls_dir, outlocdir)

    seqdir = '/media/pi/My_Book/jhuai/data/zongmu'
    bagnames = ['20240113/data4_aligned']
    mti3dk_to_ins = [0.066870657]
    xt32_to_ins = [0.02096006]
    xt32_to_mti3dk = get_lidar_td(mti3dk_to_ins, xt32_to_ins)
    lioloc_seqs(seqdir, bagnames, xt32_to_mti3dk, tls_dir, outlocdir)
