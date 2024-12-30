source /home/$USER/Documents/lidar/fastlioslam_ws/devel/setup.bash
script=/home/$USER/Documents/lidar/fastlioslam_ws/src/FAST_LIO/python/run_lioloc_w_ref.py


tls_dir=/media/$USER/MyBookDuo/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/MyBookDuo/jhuai/results/ref_trajs_all
bagdir=/media/$USER/MyBookDuo/jhuai/data/zip
outputdir=/media/$USER/MyBookDuo/jhuai/results/front_back_snapshots
python3 $script $reftraj_dir $bagdir $tls_dir $outputdir


tls_dir=/media/$USER/BackupPlus/jhuai/data/homebrew/whu_tls_1030
reftraj_dir=/media/$USER/BackupPlus/jhuai/results/ref_trajs_all
bagdir=/media/$USER/My_Book/jhuai/data/zip
outputdir=/media/$USER/BackupPlus/jhuai/results/front_back_snapshots
python3 $script $reftraj_dir $bagdir $tls_dir $outputdir

