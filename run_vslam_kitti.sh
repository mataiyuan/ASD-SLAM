BAG_NAME=/media/mataiyuan/yooongchun1/数据集/dataset/sequences/05
OUT_ADDR=/home/mataiyuan/ASD-SLAM/DataSave
EXE_ROOT=/home/mataiyuan/ASD-SLAM

ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/kitti

${ORB_SLAM_ADDR} --bag_addr=${BAG_NAME} --output_addr=${OUT_ADDR}/ --voc_addr=/home/mataiyuan/visual_map/dbowmodel/hardnet/small_voc.yml.gz --camera_config=/home/mataiyuan/ASD-SLAM/cameraconfig/KITTI/kitti04-12.txt --image_topic=/camera/image_raw --min_frame=0 --max_frame=50000 --step_frame=1 --use_orb=false --feature_count=2000 --feature_scale_factor=1.2 --feature_level=8 --min_match_count=100 --max_step_KF=15 --v=0 --logtostderr=true --colorlogtostderr=true 




