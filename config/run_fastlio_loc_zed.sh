#!/bin/bash

ws_dir="/media/$USER/docker/lidarslam/fastlio_slam_ws_rel"
fastlio_dir="$ws_dir/src/FAST_LIO_SLAM/FAST_LIO"
result_dir="/media/jhuai/MyBookDuo/jhuai/results/front_back_snapshots"
tls_dir="/media/jhuai/T7/temp/whu_tls_1030"
init_pose_dir="/media/jhuai/MyBookDuo/jhuai/results/front_back_snapshots"

configyaml="$fastlio_dir/config/hesai32_zed2i_handheld.yaml"

lidarslam() {
cd $ws_dir
source devel/setup.bash
whichend=$1
echo "whichend: $whichend"
bagnames=("${@:2}")
for i in "${!bagnames[@]}"; do 
  bag=${bagnames[$i]}
  timeoffset=${timeoffsets[$i]}
  echo "Processing bag: "$bag"_aligned.bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=""
  if [[ $whichend == "front" ]]; then
    bagfile=$datadir/"$bag"_aligned.bag
  elif [[ $whichend == "back" ]]; then
    bagfile=$init_pose_dir/$date/$run/$whichend/"$run"_aligned.bag
  else
    echo "Invalid whichend: $whichend"
    exit 1
  fi

  init_pose_file=$init_pose_dir/$date/$run/$whichend/tls_T_xt32.txt
  save_dir=$init_pose_dir/$date/$run/$whichend
  echo "bagfile: $bagfile, time offset to be set: $timeoffset"
  echo "init_lidar_pose_file: $init_pose_file"
  echo "save_dir: $save_dir"
  mkdir -p $save_dir

  configyamlname=$(basename $configyaml)
  sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: $timeoffset" $configyaml
  roslaunch fast_lio loc_hesai32_handheld.launch \
      configyaml:=$configyamlname \
      bagfile:=$bagfile tls_ref_traj_files:="$ref_traj_file1;$ref_traj_file2;$ref_traj_file3;$ref_traj_file4" \
      tls_dir:=$tls_dir \
      init_lidar_pose_file:=$init_pose_file \
      save_dir:=$save_dir 2>&1 | tee $save_dir/fastlio.log
done
sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: 0.0" $configyaml; # reset
}

# The following 3 trajs are covered by and cover the TLS map.
ref_traj_file1="$fastlio_dir/data/20231105_aft/data2/scan_states.txt" # the basketball court loop
ref_traj_file2="$fastlio_dir/data/20231109/data1/scan_states.txt" # the starlake loop
ref_traj_file3="$fastlio_dir/data/20231109/data2/scan_states.txt" # the software school loop
ref_traj_file4="$fastlio_dir/data/20231213/data1/scan_states.txt" # the xinghu bldg loop

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/handheld"
bagnames=(
"20230920/data1"
"20230920/data2"
"20230921/data2"
"20230921/data3"
"20230921/data4"
"20230921/data5")

timeoffsets=(
-0.063126	# 20230920/data1
-0.188544	# 20230920/data2
-0.060487	# 20230921/data2
-0.054847	# 20230921/data3
-0.060223	# 20230921/data4
-0.065487	# 20230921/data5
)

lidarslam front "${bagnames[@]}"

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/ebike"
bagnames=("20231007/data1"
"20231007/data2"
"20231007/data3"
"20231007/data4"
"20231007/data5"
"20231019/data1"
"20231019/data2"
"20231025/data1"
"20231025/data2")

timeoffsets=(
-0.078789 # 20231007/data1
-0.085042 # 20231007/data2
-0.063951	# 20231007/data3
-0.063899	# 20231007/data4
-0.083408	# 20231007/data5
-0.059665	# 20231019/data1
-0.071511	# 20231019/data2
-0.024084	# 20231025/data1
0.0	# 20231025/data2
)

lidarslam front "${bagnames[@]}"
