#!/bin/bash

configyamlname=$1
bagfile=$2
fastlio_dir=$3
tls_dir=$4
init_pose_file=$5
bag_start_time=$6
tls_dist_thresh=$7
state_filename=$8
save_dir=$9

echo "configyamlname: $configyamlname"
echo "bagfile: $bagfile"
echo "fastlio_dir: $fastlio_dir"
echo "tls_dir: $tls_dir"
echo "init_pose_file: $init_pose_file"
echo "bag_start_time: $bag_start_time"
echo "tls_dist_thresh: $tls_dist_thresh"
echo "state_filename: $state_filename"
echo "save_dir: $save_dir"

# The following 3 trajs are covered by and cover the TLS map.
ref_traj_file1="$fastlio_dir/data/20231105_aft/data2/scan_states.txt" # the basketball court loop
ref_traj_file2="$fastlio_dir/data/20231109/data1/scan_states.txt" # the starlake loop
ref_traj_file3="$fastlio_dir/data/20231109/data2/scan_states.txt" # the software school loop
ref_traj_file4="$fastlio_dir/data/20231213/data1/scan_states.txt" # the xinghu bldg loop

setupfile="$fastlio_dir/../../devel/setup.bash"
echo "source $setupfile"
source $setupfile

configyaml="$fastlio_dir/config/$configyamlname"
sed -i "/tls_dist_thresh/c\    tls_dist_thresh: $tls_dist_thresh" $configyaml
# get state_filename without extension
logname="${state_filename%%.*}.log"

cmd="roslaunch fast_lio loc_hesai32_handheld.launch \
    configyaml:=$configyamlname \
    bagfile:=$bagfile tls_ref_traj_files:=\"$ref_traj_file1;$ref_traj_file2;$ref_traj_file3;$ref_traj_file4\" \
    tls_dir:=$tls_dir odom_mode:=2 \
    init_lidar_pose_file:=$init_pose_file \
    bag_start_time:=$bag_start_time \
    state_filename:=$state_filename \
    save_dir:=$save_dir"

echo "$cmd"
$cmd 2>&1 | tee $save_dir/$logname
sed -i "/tls_dist_thresh/c\    tls_dist_thresh: 8.0" $configyaml
