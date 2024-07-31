#!/bin/bash


ws_dir="/media/$USER/docker/lidarslam/fastlio_slam_ws_rel"
ws_dir="/home/$USER/Documents/lidar/fastlioslam_ws"
fastlio_dir="$ws_dir/src/FAST_LIO"

tls_dir="/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030"
init_pose_dir="/media/$USER/BackupPlus/jhuai/results/front_back_snapshots"
result_dir="/media/$USER/BackupPlus/jhuai/results/front_back_snapshots"
result_dir="/media/$USER/BackupPlus/jhuai/results/front_back_loccheck"

configyaml="$fastlio_dir/config/hesai32_mti3dk_handheld.yaml"

# The following trajs are covered by and cover the TLS map.
ref_traj_file1="$fastlio_dir/data/20231105_aft/data2/scan_states.txt" # the basketball court loop
ref_traj_file2="$fastlio_dir/data/20231109/data1/scan_states.txt" # the starlake loop
ref_traj_file3="$fastlio_dir/data/20231109/data2/scan_states.txt" # the software school loop
ref_traj_file4="$fastlio_dir/data/20231213/data1/scan_states.txt" # the xinghu bldg loop

lidarslam() {
cd $ws_dir
source devel/setup.bash
whichend=$1
echo "whichend: $whichend"
bagnames=("${@:2}")
count=0
for i in "${!bagnames[@]}"; do 
  bag=${bagnames[$i]}
  timeoffset=${timeoffsets[$i]}
  echo "Processing bag: $bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=""
  if [[ $whichend == "front" ]]; then
    bagfile=$datadir/"$bag".bag
  elif [[ $whichend == "back" ]]; then
    bagfile=$init_pose_dir/$date/$run/$whichend/"$run"_aligned.bag
  else
    echo "Invalid whichend: $whichend"
    exit 1
  fi

  init_pose_file=$init_pose_dir/$date/$run/$whichend/tls_T_xt32.txt
  save_dir=$result_dir/$date/$run/$whichend
  echo "bagfile: $bagfile, time offset to be set: $timeoffset"
  echo "init_lidar_pose_file: $init_pose_file"
  echo "save_dir: $save_dir"
  mkdir -p $save_dir

  configyamlname=$(basename $configyaml)
  sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: $timeoffset" $configyaml
  sed -i "/tls_dist_thresh/c\    tls_dist_thresh: $tls_dist_thresh" $configyaml

  roslaunch fast_lio loc_hesai32_handheld.launch \
      configyaml:=$configyamlname \
      bagfile:=$bagfile tls_ref_traj_files:="$ref_traj_file1;$ref_traj_file2;$ref_traj_file3;$ref_traj_file4" \
      tls_dir:=$tls_dir \
      init_lidar_pose_file:=$init_pose_file \
      save_dir:=$save_dir 2>&1 | tee $save_dir/fastlio.log
  count=$((count+1))
  # if [ $count -eq 1 ]; then
  #   break
  # fi
done
sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: 0.0" $configyaml; # reset
sed -i "/tls_dist_thresh/c\    tls_dist_thresh: 8.0" $configyaml; # reset
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
# lidarslam front "${bags202401[@]}"
# lidarslam back "${bags202401[@]}"

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
# lidarslam front "${bags202312[@]}"
# lidarslam back "${bags202312[@]}"

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

# lidarslam front "${bags202311[@]}"

tls_dist_thresh=1000
datadir="/media/pi/My_Book/jhuai/data/zip"
bagnames=(
  "20231105/data3"
  "20231105_aft/data4")
timeoffsets=(0 0)
# lidarslam front "${bagnames[@]}"

tls_dist_thresh=8
datadir="/media/pi/My_Book/jhuai/data/zip"
bagnames=(
  "20240116/data5"
)
timeoffsets=(0)
# lidarslam front "${bagnames[@]}"

datadir="/media/pi/My_Book/jhuai/data/zip"
tls_dist_thresh=100
loc_check_bags=(
  20231105/data6
  20231109/data3
  20231109/data4
)
timeoffsets=(0 0 0)
lidarslam back "${loc_check_bags[@]}"
lidarslam front "${loc_check_bags[@]}"


datadir="/media/pi/My_Book/jhuai/data/zip"
bags202401=(
  20240113/data1)
timeoffsets=(0)
lidarslam front "${bags202401[@]}"
