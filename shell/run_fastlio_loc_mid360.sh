#!/bin/bash

ws_dir="/home/jhuai/Documents/lidar/fastlioslam_ws"
fastlio_dir="$ws_dir/src/blss_lidar_slam/FAST_LIO"
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

  init_pose_file=$init_pose_dir/$bag/$whichend/tls_T_xt32.txt
  save_dir=$result_dir/$bag/$whichend
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
      tls_dir:=$tls_dir locmode:=true \
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
datadir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/k40pro_livox/basement"
result_dir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/snapshots/k40pro_livox/basement"
bagnames=(2024_10_30_16_10_25 2024_10_30_16_34_18)
init_pose_dir=$result_dir
timeoffsets=(0.0125 0.0125)
lidarslam front "${bagnames[@]}"

datadir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/k40pro_livox/xinghubldg"
result_dir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/snapshots/k40pro_livox/xinghubldg"
bagnames=(2024_10_30_16_23_17 2024_10_30_16_29_09)
# bagnames=(2024_10_30_16_23_17) # 23_17 loc mode always fail, so we dismiss it for now.

init_pose_dir=$result_dir
timeoffsets=(0.0125 0.0125)
lidarslam front "${bagnames[@]}"
