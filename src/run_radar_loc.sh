
datadir=/media/jhuai/BackupPlus/jhuai/data/homebrew/handheld
tlsdatadir=/media/jhuai/BackupPlus/jhuai/data/whu_tls_1030
outputdir=/media/jhuai/BackupPlus/jhuai/results/radar_tls_loc

radar_loc_proj2() {
bag=$1
savedir=$outputdir/${bag%%.*}
mkdir -p $savedir
cmd="roslaunch fast_lio loc_ars548_whu_handheld.launch bagfile:=$datadir/$bag use_doppler:=true \
  max_doppler_residual_ratio:=4 max_dist_ratio:=0.8 init_lidar_pose_file:=$datadir/20231109/ars548_radar4_raw/data1_radar_to_TLS.txt \
  mapdir:=$tlsdatadir/project2/regis save_directory:=$savedir"
echo $cmd
$cmd
}

radar_loc_proj1() {
bag=$1
savedir=$outputdir/${bag%%.*}
mkdir -p $savedir
cmd="roslaunch fast_lio loc_ars548_whu_handheld.launch bagfile:=$datadir/$bag use_doppler:=true \
  max_doppler_residual_ratio:=4 max_dist_ratio:=0.8 init_lidar_pose_file:=$datadir/20231109/ars548_radar4_raw/data1_radar_to_TLS.txt \
  mapdir:=$tlsdatadir/project1/regis save_directory:=$savedir"
echo $cmd
$cmd
}

proj1_bags=(
20231109/data1_aligned.bag
20231109/data3_aligned.bag
20231105_aft/data3_aligned.bag
20231105_aft/data4_aligned.bag
20231105/data2_aligned.bag
20231105/data3_aligned.bag
)

proj2_bags=(
20231109/data2_aligned.bag
20231109/data4_aligned.bag
20231105_aft/data1_aligned.bag
20231105_aft/data2_aligned.bag
20231105_aft/data5_aligned.bag
20231105_aft/data6_aligned.bag
20231105/data4_aligned.bag
20231105/data5_aligned.bag
20231105/data6_aligned.bag
)

for bag in ${proj1_bags[@]}; do
  radar_loc_proj1 $bag
done

for bag in ${proj2_bags[@]}; do
  radar_loc_proj2 $bag
done