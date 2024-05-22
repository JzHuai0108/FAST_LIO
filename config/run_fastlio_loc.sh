#!/bin/bash

datadir="/media/jhuai/BackupPlus/jhuai/data/homebrew/zongmu"
# ws_dir="/media/jhuai/docker/lidarslam/catkin_fastlio_slam"
ws_dir="/media/jhuai/docker/lidarslam/fastlio_slam_ws_rel"
result_dir="/media/jhuai/BackupPlus/jhuai/results/fastlio_loc"
tls_dir="/media/jhuai/BackupPlus/jhuai/data/homebrew/whu_tls_1030"
init_pose_dir="/media/jhuai/BackupPlus/jhuai/results/front_back_snapshots"

bags202312=(20231201/data2
      20231201/data3
      20231208/data1
      20231208/data2
      20231208/data3
      20231208/data4
      20231208/data5
      20231213/data1
      20231213/data2
      20231213/data3
      20231213/data4
      20231213/data5)

bag20231105=(
20231105/data1
20231105/data2
20231105/data3
20231105/data4
20231105/data5
20231105/data6
20231105/data7)

bag20231105_aft=(20231105_aft/data1
20231105_aft/data2
20231105_aft/data3
20231105_aft/data4
20231105_aft/data5
20231105_aft/data6)

bag20231109=(20231109/data1
20231109/data2
20231109/data3
20231109/data4)

# The following 3 trajs are covered by and cover the TLS map.
ref_traj_file1="$ws_dir/src/FAST_LIO_SLAM/FAST_LIO/data/20231105_aft/data2/scan_states.txt" # the basketball court loop
ref_traj_file2="$ws_dir/src/FAST_LIO_SLAM/FAST_LIO/data/20231109/data1/scan_states.txt" # the starlake loop
ref_traj_file3="$ws_dir/src/FAST_LIO_SLAM/FAST_LIO/data/20231109/data2/scan_states.txt" # the software school loop
ref_traj_file4="$ws_dir/src/FAST_LIO_SLAM/FAST_LIO/data/20231213/data1/scan_states.txt" # the xinghu bldg loop

lidarslam() {
cd $ws_dir
source devel/setup.bash
whichend=$1
echo "whichend: $whichend"
bagnames=("${@:2}")
for bag in "${bagnames[@]}"; do
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
  echo "bagfile: $bagfile"
  echo "init_lidar_pose_file: $init_pose_file"
  echo "save_dir: $save_dir"
  mkdir -p $save_dir
  roslaunch fast_lio loc_hesai32_handheld.launch \
      configyaml:=hesai32_mti3dk_handheld.yaml \
      bagfile:=$bagfile tls_ref_traj_files:="$ref_traj_file1;$ref_traj_file2;$ref_traj_file3;$ref_traj_file4" \
      tls_dir:=$tls_dir \
      init_lidar_pose_file:=$init_pose_file \
      save_dir:=$save_dir
done
}


lidarslam front "${bags202312[@]}"
lidarslam back "${bags202312[@]}"
