#!/bin/bash

configyamlname=$1
bagfile=$2
fastlio_dir=$3
msg_start_time=$4
msg_end_time=$5
state_filename=$6
save_dir=$7
if [ "$#" -gt 7 ]; then
    time_offset_lidar_to_imu=${8}
else
    time_offset_lidar_to_imu=0.0
fi

echo "configyamlname: $configyamlname"
echo "bagfile: $bagfile"
echo "fastlio_dir: $fastlio_dir"
echo "msg_start_time: $msg_start_time"
echo "msg_end_time: $msg_end_time"
echo "state_filename: $state_filename"
echo "save_dir: $save_dir"
echo "time_offset_lidar_to_imu: $time_offset_lidar_to_imu"

setupfile="$fastlio_dir/../../devel/setup.bash"
echo "source $setupfile"
source $setupfile

configyaml="$fastlio_dir/config/$configyamlname"

sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: $time_offset_lidar_to_imu" $configyaml

# get state_filename without extension
logname="${state_filename%%.*}.log"

cmd="roslaunch fast_lio mapping_vlp16_kuangye.launch \
    configyaml:=$configyaml \
    bagfile:=$bagfile \
    msg_start_time:=$msg_start_time \
    msg_end_time:=$msg_end_time \
    save_dir:=$save_dir"

echo "$cmd"
$cmd 2>&1 | tee $save_dir/$logname
sed -i "/time_offset_lidar_to_imu/c\    time_offset_lidar_to_imu: 0.0" $configyaml
