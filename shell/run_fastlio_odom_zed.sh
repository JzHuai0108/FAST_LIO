#!/bin/bash

ws_dir="/media/$USER/docker/lidarslam/catkin_fastlio_slam"
ws_dir="$HOME/Documents/lidar/fastlioslam_ws"
fastlio_dir="$ws_dir/src/FAST_LIO"
configyaml="$fastlio_dir/config/hesai32_zed2i_handheld.yaml"

result_dir="/media/$USER/MyBookDuo/jhuai/results/fastlio"

lidarslam() {
cd $ws_dir
source devel/setup.bash
bagnames=("$@")
for i in "${!bagnames[@]}"; do 
  bag=${bagnames[$i]}
  timeoffset=${timeoffsets[$i]}
  echo "Processing bag: "$bag"_aligned.bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=$datadir/"$bag"_aligned.bag
  save_dir=$result_dir/$bag/
  echo "bagfile: $bagfile save_dir: $save_dir"
  mkdir -p $save_dir

  configyamlname=$(basename $configyaml)
  echo "configyamlname: $configyamlname"
  sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: $timeoffset" $configyaml
  cmd="roslaunch fast_lio mapping_hesai32_handheld.launch configyaml:=$configyamlname bagfile:=$bagfile save_dir:=$save_dir"
  echo $cmd
  $cmd 2>&1 | tee $save_dir/fastlio.log
done
sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: 0.0" $configyaml; # reset
}

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/handheld"
bagnames=(
# "20230920/data1"
# "20230920/data2"
"20230921/data2"
# "20230921/data3"
# "20230921/data4"
# "20230921/data5"
)

timeoffsets=(
# -0.063126	# 20230920/data1
# -0.188544	# 20230920/data2
-0.060487	# 20230921/data2
# -0.054847	# 20230921/data3
# -0.060223	# 20230921/data4
# -0.065487	# 20230921/data5
)

lidarslam "${bagnames[@]}"

datadir="/media/jhuai/MyBookDuo/jhuai/data/homebrew/ebike"
bagnames=(
# "20231007/data1"
"20231007/data2"
# "20231007/data3"
"20231007/data4"
# "20231007/data5"
"20231019/data1"
# "20231019/data2"
# "20231025/data1"
# "20231025/data2"
)

timeoffsets=(
# -0.078789 # 20231007/data1
-0.085042 # 20231007/data2
# -0.063951	# 20231007/data3
-0.063899	# 20231007/data4
# -0.083408	# 20231007/data5
-0.059665	# 20231019/data1
# -0.071511	# 20231019/data2
# -0.024084	# 20231025/data1
# 0.0	# 20231025/data2
)

lidarslam "${bagnames[@]}"
