#!/bin/bash


ws_dir="/media/$USER/docker/lidarslam/fastlio_slam_ws_rel"
ws_dir="/home/$USER/Documents/lidar/fastlioslam_ws"
result_dir="/media/$USER/BackupPlus/jhuai/results/front_back_snapshots"
tls_dir="/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030"
init_pose_dir="/media/$USER/BackupPlus/jhuai/results/front_back_snapshots"

# The following trajs are covered by and cover the TLS map.
ref_traj_file1="$ws_dir/src/FAST_LIO/data/20231105_aft/data2/scan_states.txt" # the basketball court loop
ref_traj_file2="$ws_dir/src/FAST_LIO/data/20231109/data1/scan_states.txt" # the starlake loop
ref_traj_file3="$ws_dir/src/FAST_LIO/data/20231109/data2/scan_states.txt" # the software school loop
ref_traj_file4="$ws_dir/src/FAST_LIO/data/20231213/data1/scan_states.txt" # the xinghu bldg loop

lidarslam() {
cd $ws_dir
source devel/setup.bash
whichend=$1
echo "whichend: $whichend"
bagnames=("${@:2}")
count=0
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
      save_dir:=$save_dir 2>&1 | tee $save_dir/fastlio.log
  count=$((count+1))
  # if [ $count -eq 1 ]; then
  #   break
  # fi
done
}

datadir="/media/pi/My_Book/jhuai/data/zongmu"
bags202401=(
  20240113/data1
  20240113/data2
  20240113/data3
  20240113/data4
  20240113/data5
  20240115/data1
  20240115/data2
  20240115/data3
  20240115/data4
  20240116/data2
  20240116/data3
  20240116/data4
  20240116/data5
  20240116_eve/data1
  20240116_eve/data2
  20240116_eve/data3
  20240116_eve/data4
  20240116_eve/data5)
lidarslam front "${bags202401[@]}"
lidarslam back "${bags202401[@]}"

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/zongmu"
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
lidarslam front "${bags202312[@]}"
lidarslam back "${bags202312[@]}"

datadir="/media/pi/BackupPlus/jhuai/data/homebrew/ebike"
bags202311=(
20231105/data1
20231105/data2
20231105/data3
20231105/data4
20231105/data5
20231105/data6
20231105/data7
20231105_aft/data1
20231105_aft/data2
20231105_aft/data3
20231105_aft/data4
20231105_aft/data5
20231105_aft/data6
20231109/data1
20231109/data2
20231109/data3
20231109/data4)

lidarslam front "${bags202311[@]}"
