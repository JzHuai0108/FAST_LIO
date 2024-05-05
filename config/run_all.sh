# batch process all homebrew handheld seqs
# recommended settings
# point_filter_num 4
# filter_size_map 0.25

#!/bin/bash
# example usage: ./SC-PGO/launch/process_coloradar.sh /media/pi/BackupPlus/jhuai/data/coloradar/rosbags /home/pi/Desktop/temp/coloradar

bagpath="$1"
outputpath="$2"
mappath="$3"

bag20231105=(
20231105/data1_aligned
20231105/data2_aligned
20231105/data3_aligned
20231105/data4_aligned
20231105/data5_aligned
20231105/data6_aligned
20231105/data7_aligned
)


bag20231105_aft=(20231105_aft/data1_aligned
20231105_aft/data2_aligned
20231105_aft/data3_aligned
20231105_aft/data4_aligned
20231105_aft/data5_aligned
20231105_aft/data6_aligned
)

bag20231109=(20231109/data1_aligned
20231109/data2_aligned
20231109/data3_aligned
20231109/data4_aligned)


lidarslam() {
cd /home/cyw/CYW/TPAMI/FAST_LIO_LOC_TLS_map_ws/
source devel/setup.bash
bagnames=("$@")
for bag in "${bagnames[@]}"; do
  echo "Processing bag: $bag"
  pathtmp=${bag#*/}
  echo "init_lidar_pose_file: $bagpath/${bag%%/*}/trans/${pathtmp%_*}_lidar_to_TLS.txt"
  mkdir -p $outputpath/$bag
  roslaunch fast_lio loc_hesai32_handheld.launch \
      configyaml:=hesai32_mti3dk_handheld.yaml \
      bagfile:=$bagpath/$bag.bag \
      tls_dir:=$mappath \
      init_lidar_pose_file:=$bagpath/${bag%%/*}/trans/${pathtmp%_*}_lidar_to_TLS.txt \
      save_dir:=$outputpath/$bag/
done
}

# rerun for failed seqs
failed=(
20231105_aft/data3_aligned
)

# lidarslam "${bag20231105[@]}"
# lidarslam "${bag20231105_aft[@]}"
# lidarslam "${bag20231109[@]}"
lidarslam "${failed[@]}"
