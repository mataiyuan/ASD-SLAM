BAG_NAME=/media/mataiyuan/Elements/kitti_00_0.bag
OUT_ADDR=/home/mataiyuan/visual_map/datasave
EXE_ROOT=/home/mataiyuan/visual_map/visual_map

ORB_SLAM_ADDR=${EXE_ROOT}/devel/lib/vslam/kitti

${ORB_SLAM_ADDR} --bag_addr=${BAG_NAME} --output_addr=${OUT_ADDR}/ --voc_addr=/home/mataiyuan/visual_map/dbowmodel/hardnet/small_voc.yml.gz --camera_config=/home/mataiyuan/visual_map/cameraconfig/MH_EUROC/EuRoC_config.txt --image_topic=/cam0/image_raw --min_frame=0 --max_frame=8000 --step_frame=1 --use_orb=false --feature_count=2000 --feature_scale_factor=1.2 --feature_level=8 --min_match_count=100 --max_step_KF=15 --v=0 --logtostderr=true --colorlogtostderr=true 
