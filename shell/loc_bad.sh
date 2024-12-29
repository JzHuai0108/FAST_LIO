#!/bin/bash
# This script deals with the localization problematic sequences case by case.
# In the below, the init time adjustment is sometimes needed because
# the original rosbag has changed and hence its end time which was used as mirror time.

# Problematic sequences
# date, run, bad cases, solution, status
# 0920, 1, f+b forward + backward, enlarge the tls_dist_thresh and decrease the IMU noise level and adjust the backward_init_pose early in time(key), OK
# 0921, 3, f+b, enlarge tls_dist_thresh and decrease the IMU noise level and adjust the backward_init_pose time to the bag start time(key), OK
# 0921, 5, f + b, enlarge tls_dist_thresh and decrease IMU noise level, OK
# 1007, 4, f + b, enlarge tls_dist_thresh and decrease IMU noise level, OK

# 1105, 2, f+b, enlarge tls_dist_thresh, OK
# 1105, 3, f+b forward + backward, disable IMU state propagation at start, OK
# 1105, 4, f+b, enlarge tls_dist_thresh, OK with small jumps
# 1105, 6, f+b, enlarge tls_dist_thresh, OK
# 1105_aft, 2, f+b, enlarge tls_dist_thresh, OK
# 1105_aft, 4, f+b both has a jump, decrease the IMU noise level, OK
# 1208, 1, front begin has a jump, trim by hand, OK
# 0113, 1, front backward both has a jump, decrease the IMU noise level, OK


source $HOME/Documents/lidar/fastlioslam_ws/devel/setup.bash
script=$HOME/Documents/lidar/fastlioslam_ws/src/FAST_LIO/python/run_lioloc_w_ref.py

tls_dir=/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/BackupPlus/jhuai/results/ref_trajs_all
outputdir=/media/$USER/BackupPlus/jhuai/results/front_back_snapshots
bagdir=/media/$USER/My_Book/jhuai/data/zip
configyamlname="hesai32_mti3dk_handheld.yaml"

tls_dir=/media/$USER/MyBookDuo/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/MyBookDuo/jhuai/results/ref_trajs_all
outputdir=/media/$USER/MyBookDuo/jhuai/results/front_back_snapshots
bagdir=/media/$USER/MyBookDuo/jhuai/data/zip
configyamlname="hesai32_x36d_handheld.yaml"

bagnames=(
    20230920/data1.bag
    20230921/data3.bag
    20230921/data5.bag
    20231007/data4.bag)

fastlio_dir=$HOME/Documents/lidar/fastlioslam_ws/src/FAST_LIO

configyaml="$fastlio_dir/config/$configyamlname"
if [[ -f "$configyaml" ]]; then
    echo "$configyaml found"
    sed -i "/    acc_cov:/c\    acc_cov: 0.01" $configyaml
    sed -i "/    gyr_cov:/c\    gyr_cov: 0.002" $configyaml
else
    echo "Warning: config yaml $configyaml does not exist."
fi

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

sed -i "/    acc_cov:/c\    acc_cov: 0.04" $configyaml # restore
sed -i "/    gyr_cov:/c\    gyr_cov: 0.01" $configyaml # restore
