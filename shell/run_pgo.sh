# deprecated, use the entry point.sh in cpgo instead.

ws_dir=/home/jhuai/Documents/swift_vio_ws
ws_dir=/home/pi/Documents/lie_splines_ws
executable=$ws_dir/devel/lib/cascaded_pgo/pgo


bagnames=(
# "20231007/data2"
# "20231007/data4"
# "20231019/data1"
"20240113/data1" # --cull_begin_secs 0.5 and --cull_end_secs 1.5
)

liolocdir="/media/pi/BackupPlus/jhuai/results/front_back_snapshots"
kissicpdir="/media/pi/BackupPlus/jhuai/results/kissicp"
fastliodir="/media/pi/BackupPlus/jhuai/results/fastlio"

run_pgo() {
cd $ws_dir
source devel/setup.bash
for bagname in "${bagnames[@]}"; do
  date=${bagname:0:8}
  run=${bagname:9}
  echo "date: $date, run: $run"
  front_loc_file=$liolocdir/$bagname/front/scan_states.txt
  back_loc_file=$liolocdir/$bagname/back/scan_states.txt
  odom_file=$kissicpdir/$bagname/"$run"_aligned_poses_tum.txt
  odom_file=$fastliodir/$bagname/scan_states.txt
  output_path=$liolocdir/$bagname/pgo
  mkdir -p $output_path
  $executable $odom_file $front_loc_file $back_loc_file $output_path --cull_begin_secs 0.5 \
        --cull_end_secs 1.5 --opt_rotation_only=true --opt_translation_only=true --opt_poses=true
done
}

run_pgo