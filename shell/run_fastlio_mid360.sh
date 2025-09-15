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
  echo "bagfile: $bagfile, time offset to be set: $timeoffset"
  echo "save_dir: $save_dir"
  mkdir -p $save_dir/pcd

  configyamlname=$(basename $configyaml)
  sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: $timeoffset" $configyaml
  sed -i "/tls_dist_thresh/c\    tls_dist_thresh: $tls_dist_thresh" $configyaml
  roslaunch fast_lio loc_mid360_phone.launch \
      configyaml:=$configyamlname \
      bagfile:=$bagfile tls_ref_traj_files:="$ref_traj_file1;$ref_traj_file2;$ref_traj_file3;$ref_traj_file4" \
      tls_dir:=$tls_dir odom_mode:=0 \
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
datadir=/media/jhuai/ExtremeSSD/jhuai/livox_phone/k60pro_livox/20250207/places
result_dir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/k60pro_livox"
bagnames=(2025_02_07_10_16_26
 2025_02_07_10_19_59
 2025_02_07_10_25_05
 2025_02_07_10_28_32
 2025_02_07_10_48_23
 2025_02_07_10_51_02)

timeoffsets=(-0.005 -0.005 -0.005 -0.005 -0.005 -0.005)
lidarslam front "${bagnames[@]}"

datadir=/media/jhuai/ExtremeSSD/jhuai/livox_phone/s22plus_livox
result_dir="/media/jhuai/ExtremeSSD/jhuai/livox_phone/results/s22plus_livox"
bagnames=(20250207/places/2025_02_07_09_40_51
    20250207/places/2025_02_07_09_43_25
    20250206/2025_02_06_17_18_51
    20250206/2025_02_06_17_25_42
    20250206/2025_02_06_17_29_26
    20250206/2025_02_06_17_35_12
    20250206/2025_02_06_17_41_47
    20250124/2025_01_24_16_58_12
    20250124/2025_01_24_17_00_46
    20250124/2025_01_24_17_16_53
    20250124/2025_01_24_17_25_46
    20250101/2025_01_01_10_41_48
    20250101/2025_01_01_10_44_13
    20250101/2025_01_01_10_51_06
    20241205/2024_12_05_16_11_23
    20241205/2024_12_05_16_32_45
    20241205/2024_12_05_16_33_55
)

timeoffsets=(-0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005 -0.005)
lidarslam front "${bagnames[@]}"

