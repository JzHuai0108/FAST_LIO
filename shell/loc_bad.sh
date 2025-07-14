#!/bin/bash
# This script deals with the localization problematic sequences case by case.
# In the below, the init time adjustment is sometimes needed because
# the original rosbag has changed and hence its end time which was used as mirror time.
# the default acc_cov: 0.01, gyr_cov: 0.002 for both mti3dk and x36d imu.

# Problematic sequences
# 2025 Jul 14 edited
# case, symptom, solution, status
# 1105/2/front/bwd, traj at the end diff much from fwd traj, bg=(0,0,0) at start, OK
# 1105/3/front/fwd, a small jump at bump, initialize localizer at each step with odometry increment through traj, OK
# 1105/4/front/bwd, some jumps, initialize localizer at each step with odometry increment through traj, OK
# 1105_aft/4/front/fwd, a big jump, initialize localizer at each step with odometry increment through traj, OK
# 1213/2/back/bwd, slip at the start, fix the initial pose for back segment forward mode, OK
# 1213/4/front/fwd, slip at the start, fix the initial pose for front segment forward mode, OK
# 1213/4/back/fwd + bwd, large velocity diff, initialize localizer at each step with odometry increment through traj, OK
# 0116/2/front/fwd, slip at the start, fix the initial pose for front segment forward mode, OK
# 0116/2/front/fwd and bwd, large velocity diff, initialize localizer at each step with odometry increment through traj, OK

# case, symptom, solution, status
# 1208/4/back/fwd, huge drift, initialize localizer at each step with odometry increment for 0.5 second, OK
# 0123/3/back/fwd, huge drift, initialize localizer at each step with odometry increment for 0.5 second, OK

# 2025 Jan 3 edited
# date, bad case, solution, status
# 0920/2, front backward states drift in the middle, set lidar offset time to -0.05 for backward=-(0.0793 - 0.03) 0.05 for forward, OK
# 1019/2, forward backward states diff large at some point, set lidar offset time to 0.05 for backward -0.05 for forward also adjust backward init pose per reversed bag start time, OK
# 1105/2, front backward states drift from the start, set the correct init state at the reversed bag start time(key) also set lidar offset time to 0.05 for backward and trim the forward end 2 sec, OK
# 1105/4, front backward states drift from the start and front forward states have jumps, set lidar offset time to 0.05 for backward -0.05 for forward, OK jump mitigated
# 1105_aft/4, front forward states has a large jump in the middle, decrease acc_cov: 5.0e-4 and gyr_cov:1.0e-4, OK jump mitigated
# 0123/data3, back forward states drift, adjust the initial pose per the start time and set lidar offset time to -0.05 for forward, OK
# 1213/data5, back forward states drift, adjust the initial pose per the start time and set lidar offset time to -0.05 for forward, OK

FASTLIO_WS=$HOME/Documents/lidar/fastlioslam_ws
source $FASTLIO_WS/devel/setup.bash

fastlio_dir=$FASTLIO_WS/src/FAST_LIO
script=$fastlio_dir/python/run_lioloc_w_ref.py

run_loc() {
for i in "${!bagnames[@]}"; do
    bn=${bagnames[$i]}
    loctype=${loctypes[$i]}
    echo "Processing $bn with loctype $loctype"
    bagpath="$bagdir/$bn"
    # Check if the bag file exists
    if [[ -f "$bagpath" ]]; then
        echo "$bagpath found"
        loc_flag="--loc_follow_odom=${follow_odom[$i]}"
        cmd="python3 $script $reftraj_dir $bagpath $tls_dir $outputdir $loc_flag --tls_dist_thresh=8 --loc_type=$loctype"
        echo "$cmd"
        $cmd
    else
        echo "Warning: Bag file $bagpath does not exist."
    fi
done
}


tls_dir=/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/BackupPlus/jhuai/results/ref_trajs_all
outputdir=/media/$USER/BackupPlus/jhuai/results/front_back_snapshots
bagdir=/media/$USER/My_Book/jhuai/data/zip

bagnames=(
    20231105/data2.bag # front_bwd
    20231105/data3.bag # front_fwd 100
    20231105/data4.bag # front_fwd 1e6
    20231105/data4.bag # front_bwd 1e6
    20231105_aft/data4.bag # front_fwd 1e6
    20231213/data2.bag # back_bwd 1e6
    20231213/data4.bag # back_fwd 1e6
    20231213/data4.bag # back bwd 1e6
    20240116/data2.bag # back fwd 1e6
    20240116/data2.bag # back bwd 1e6
)

loctypes=(
front_bwd
front_fwd
front_fwd
front_bwd
front_fwd
back_bwd
back_fwd
back_bwd
back_fwd
back_bwd
)

follow_odom=(
0.5
100 1e6 1e6
1e6 1e6 1e6 1e6 1e6 1e6)

run_loc
