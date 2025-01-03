#!/bin/bash
# This script deals with the localization problematic sequences case by case.
# In the below, the init time adjustment is sometimes needed because
# the original rosbag has changed and hence its end time which was used as mirror time.
# the default acc_cov: 0.01, gyr_cov: 0.002 for both mti3dk and x36d imu.

# Problematic sequences
# date, bad case, solution, status
# 0920/2, front backward states drift in the middle, set lidar offset time to -0.05 for backward=-(0.0793 - 0.03) 0.05 for forward, OK
# 1019/2, forward backward states diff large at some point, set lidar offset time to 0.05 for backward -0.05 for forward also adjust backward init pose per reversed bag start time, OK
# 1105/2, front backward states drift from the start, set the correct init state at the reversed bag start time(key) also set lidar offset time to 0.05 for backward and trim the forward end 2 sec, OK
# 1105/4, front backward states drift from the start and front forward states have jumps, set lidar offset time to 0.05 for backward -0.05 for forward, OK jump mitigated
# 1105_aft/4, front forward states has a large jump in the middle, decrease acc_cov: 5.0e-4 and gyr_cov:1.0e-4, OK jump mitigated
# 0123/data3, back forward states drift, adjust the initial pose per the start time and set lidar offset time to -0.05 for forward, OK
# 1213/data5, back forward states drift, adjust the initial pose per the start time and set lidar offset time to -0.05 for forward, OK

source $HOME/Documents/lidar/fastlioslam_ws/devel/setup.bash
script=$HOME/Documents/lidar/fastlioslam_ws/src/FAST_LIO/python/run_lioloc_w_ref.py

tls_dir=/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/BackupPlus/jhuai/results/ref_trajs_all
outputdir=/media/$USER/BackupPlus/jhuai/results/front_back_snapshots
bagdir=/media/$USER/My_Book/jhuai/data/zip
configyamlname="hesai32_mti3dk_handheld.yaml"

bagnames=(
    20231105/data2.bag
    20231105/data4.bag
    20231105_aft/data4.bag)

# tls_dir=/media/$USER/MyBookDuo/jhuai/data/homebrew/whu_tls_1030
# reftraj_dir=/media/$USER/MyBookDuo/jhuai/results/ref_trajs_all
# outputdir=/media/$USER/MyBookDuo/jhuai/results/front_back_snapshots
# bagdir=/media/$USER/MyBookDuo/jhuai/data/zip
# configyamlname="hesai32_x36d_handheld.yaml"

# bagnames=(
#     20230920/data2.bag
#     20231019/data2.bag)

fastlio_dir=$HOME/Documents/lidar/fastlioslam_ws/src/FAST_LIO

configyaml="$fastlio_dir/config/$configyamlname"

for bn in "${bagnames[@]}"; do
    echo "Processing $bn"
    bagpath="$bagdir/$bn"
    # Check if the bag file exists
    if [[ -f "$bagpath" ]]; then
        echo "$bagpath found"
        cmd="python3 $script $reftraj_dir $bagpath $tls_dir $outputdir --tls_dist_thresh=100"
        echo $cmd
        $cmd
    else
        echo "Warning: Bag file $bagpath does not exist."
    fi
done
