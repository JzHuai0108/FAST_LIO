#!/bin/bash

ws_dir="/home/jhuai/Documents/lidar/fastlioslam_ws"
fastlio_dir="$ws_dir/src/FAST_LIO"
tls_dir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/whu_tls_1030"
configyaml="$fastlio_dir/config/mid360.yaml"

lidarslam() {
cd $ws_dir
source devel/setup.bash
whichend=$1
echo "whichend: $whichend"
bagnames=("${@:2}")
for i in "${!bagnames[@]}"; do 
  bag=${bagnames[$i]}
  timeoffset=${timeoffsets[$i]}
  echo "Processing bag: $bag"
  bagfile=$datadir/"$bag"/movie.bag

  save_dir=$result_dir/$bag/$whichend
  init_pose_file=${init_pose_files[$i]}
  echo "bagfile: $bagfile, time offset to be set: $timeoffset"
  echo "init_lidar_pose_file: $init_pose_file"
  echo "save_dir: $save_dir"
  mkdir -p $save_dir
  configyamlname=$(basename $configyaml)
  sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: $timeoffset" $configyaml
  sed -i "/tls_dist_thresh/c\    tls_dist_thresh: $tls_dist_thresh" $configyaml
  roslaunch fast_lio loc_mid360_phone.launch \
      configyaml:=$configyamlname \
      bagfile:=$bagfile tls_ref_traj_files:="$ref_traj_file1;$ref_traj_file2;$ref_traj_file3;$ref_traj_file4" \
      tls_dir:=$tls_dir odom_mode:=1 \
      init_lidar_pose_file:=$init_pose_file \
      save_dir:=$save_dir 2>&1 | tee $save_dir/fastlio.log
done
sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: 0.0" $configyaml; # reset
sed -i "/tls_dist_thresh/c\    tls_dist_thresh: 8.0" $configyaml
}

# The following 3 trajs are covered by and cover the TLS map.
ref_traj_file1="$fastlio_dir/data/20231105_aft/data2/scan_states.txt" # the basketball court loop
ref_traj_file2="$fastlio_dir/data/20231109/data1/scan_states.txt" # the starlake loop
ref_traj_file3="$fastlio_dir/data/20231109/data2/scan_states.txt" # the software school loop
ref_traj_file4="$fastlio_dir/data/20231213/data1/scan_states.txt" # the xinghu bldg loop

tls_dist_thresh=200

init_pose_dir=/media/jhuai/ExtremeSSD/jhuai/livox_phone/config

result_dir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/k60pro_livox_loc"
datadir=/media/jhuai/ExtremeSSD/jhuai/livox_phone/k60pro_livox/20250207/places
bagnames=(2025_02_07_10_16_26 
2025_02_07_10_19_59 
2025_02_07_10_25_05 
2025_02_07_10_28_32)

init_pose_files=(
$init_pose_dir/xinghubldg/20250206_tls_T_xt32.txt
$init_pose_dir/xinghubldg/20250206_tls_T_xt32.txt
$init_pose_dir/basement/20250206_tls_T_xt32.txt
$init_pose_dir/basement/20250206_tls_T_xt32x.txt
)

timeoffsets=(-0.005 -0.005 -0.005 -0.005 -0.005)
lidarslam front "${bagnames[@]}"

result_dir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox_loc"
datadir=/media/jhuai/ExtremeSSD/jhuai/livox_phone/s22plus_livox/20250206
bagnames=(2025_02_06_17_25_42
2025_02_06_17_29_26
2025_02_06_17_35_12
2025_02_06_17_41_47
)

timeoffsets=(0.00 -0.005 -0.005 -0.005 -0.005)
init_pose_files=(
$init_pose_dir/xinghubldg/20250206_tls_T_xt32.txt
$init_pose_dir/xinghubldg/20250206_tls_T_xt32.txt
$init_pose_dir/basement/20250206_tls_T_xt32.txt
$init_pose_dir/basement/20250206_tls_T_xt32.txt
)
lidarslam front "${bagnames[@]}"
