#!/bin/bash
datadir="/media/jhuai/BackupPlus/jhuai/data/homebrew/zongmu"
ws_dir="/media/jhuai/docker/lidarslam/catkin_fastlio_slam"
result_dir="/media/jhuai/BackupPlus/jhuai/results/fastlio"

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

# rerun for failed seqs
failed=(
20231105_aft/data3
)

lidarslam() {
cd $ws_dir
source devel/setup.bash
bagnames=("$@")
for bag in "${bagnames[@]}"; do
  echo "Processing bag: "$bag"_aligned.bag"
  date=${bag%%/*} # the first part of $bag
  run=${bag#*/} # the second part of $bag
  bagfile=$datadir/"$bag"_aligned.bag
  save_dir=$result_dir/$bag/
  echo "bagfile: $bagfile save_dir: $save_dir"
  mkdir -p $save_dir
  roslaunch fast_lio mapping_hesai32_handheld.launch \
      configyaml:=hesai32_mti3dk_handheld.yaml \
      bagfile:=$bagfile save_dir:=$save_dir 2>&1 | tee $save_dir/fastlio.log
done
}

lidarslam "${bags202312[@]}"
