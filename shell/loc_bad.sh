#!/bin/bash
# This script deals with the localization problematic sequences case by case.
# In the below, the init time adjustment is sometimes needed because
# the original rosbag has changed and hence its end time which was used as mirror time.
# the default acc_cov: 0.01, gyr_cov: 0.002 for both mti3dk and x36d imu.

# Problematic sequences
# 2025 May 6 edited
# case, symptom, solution, status
# 1105/4/front/bwd, some jumps,
# 1105_aft/4/front/fwd, a big jump,
# 1208/4/back/fwd,
# 1213/4/back/fwd,
# 1213/5/back/fwd
# 0113/2/back/fwd
# 0115/3/back/fwd
# 0116_eve/5/back/fwd
# 0123/3/back/fwd


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

tls_dir=/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/BackupPlus/jhuai/results/ref_trajs_all
outputdir=/media/$USER/BackupPlus/jhuai/results/front_back_snapshots
bagdir=/media/$USER/My_Book/jhuai/data/zip

bagnames=(
    20231105/data4.bag
    20231105_aft/data4.bag
    20231208/data4.bag
    20231213/data4.bag
    20231213/data5.bag
    20240113/data2.bag
    20240115/data3.bag
    20240116_eve/data5.bag
)

loctypes=(front_bwd
front_fwd
back_fwd
back_fwd
back_fwd
back_fwd
back_fwd
back_fwd
)


tls_dir=/media/$USER/MyBookDuo/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/MyBookDuo/jhuai/results/ref_trajs_all
outputdir=/media/$USER/MyBookDuo/jhuai/results/front_back_snapshots
bagdir=/media/$USER/MyBookDuo/jhuai/data/zip

bagnames=(
    20240123/data3.bag
)

loctypes=(back_fwd
)

follow_odom=(1)

for i in "${!bagnames[@]}"; do
    bn=${bagnames[$i]}
    loctype=${loctypes[$i]}
    echo "Processing $bn with loctype $loctype"
    bagpath="$bagdir/$bn"
    # Check if the bag file exists
    if [[ -f "$bagpath" ]]; then
        echo "$bagpath found"
        # Construct optional flag based on follow_odom[i]
        if [[ ${follow_odom[$i]} -eq 1 ]]; then
            loc_flag="--loc_follow_odom"
        else
            loc_flag=""
        fi

        cmd="python3 $script $reftraj_dir $bagpath $tls_dir $outputdir $loc_flag --tls_dist_thresh=100 --loc_type=$loctype"
        echo "$cmd"
        $cmd
    else
        echo "Warning: Bag file $bagpath does not exist."
    fi
done
