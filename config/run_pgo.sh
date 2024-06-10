
ws_dir=/home/jhuai/Documents/swift_vio_ws
executable=$ws_dir/devel/lib/cascaded_pgo/pgo


bagnames=(
"20231007/data2"
"20231007/data4"
# "20231019/data1"
)

liolocdir="/media/jhuai/MyBookDuo/jhuai/results/front_back_snapshots"
kissicpdir="/media/jhuai/MyBookDuo/jhuai/results/kissicp"
fastliodir="/media/jhuai/MyBookDuo/jhuai/results/fastlio"

run_pgo() {
cd $ws_dir
source devel/setup.bash
for bagname in "${bagnames[@]}"; do
  date=${bagname:0:8}
  run=${bagname:9}
  echo "date: $date, run: $run"
  front_loc_file=$liolocdir/$bagname/front/scan_states_keep.txt
  back_loc_file=$liolocdir/$bagname/back/scan_states_keep.txt
  odom_file=$kissicpdir/$bagname/"$run"_aligned_poses_tum_keep.txt
  odom_file=$fastliodir/$bagname/scan_states.txt
  output_path=$liolocdir/$bagname/pgo
  mkdir -p $output_path
  $executable $odom_file $front_loc_file $back_loc_file $output_path --cull_begin_secs 1.5 \
        --cull_end_secs 8.0 --opt_rotation_only=true --opt_translation_only=true --opt_poses=true
done
}

run_pgo