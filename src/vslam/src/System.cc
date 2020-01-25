#include "System.h"
#include "Converter.h"
#include <thread>
#include <iomanip>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "Tracking.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrameDatabase.h"
#include <gflags/gflags.h>
#include <glog/logging.h>
#include "visual_map/visual_map.h"
#include "visual_map/visual_map_seri.h"

DEFINE_string(voc_addr, "", "Vocabulary file address.");
DEFINE_string(map_addr, "", "map file address.");

namespace ORB_SLAM2
{
    std::vector<std::string> split(const std::string& str, const std::string& delim)
    {
        std::vector<std::string> tokens;
        size_t prev = 0, pos = 0;
        do
        {
            pos = str.find(delim, prev);
            if (pos == std::string::npos) pos = str.length();
            std::string token = str.substr(prev, pos-prev);
            if (!token.empty()) tokens.push_back(token);
            prev = pos + delim.length();
        }
        while (pos < str.length() && prev < str.length());
        return tokens;
    }
    void System::LoadORBMap(std::string map_name, 
                                 ORB_SLAM2::ORBVocabulary*& mpVocabulary, 
                                 ORB_SLAM2::KeyFrameDatabase*& mpKeyFrameDatabase, 
                                 ORB_SLAM2::Map*& mpMap
                   ){
        
        vm::VisualMap map;
        
        vm::loader_visual_map(map, map_name);
        map.ComputeUniqueId();
        
        mpMap = new ORB_SLAM2::Map();
        CHECK_GT(map.frames.size(),0);
        std::vector<float> cam_info;
        cam_info.push_back(map.frames[0]->fx);
        cam_info.push_back(map.frames[0]->fy);
        cam_info.push_back(map.frames[0]->cx);
        cam_info.push_back(map.frames[0]->cy);
        cam_info.push_back(map.frames[0]->width);
        cam_info.push_back(map.frames[0]->height);
        std::vector<ORB_SLAM2::KeyFrame*> kfs;
        for(int i=0; i<map.frames.size(); i++){
            ORB_SLAM2::KeyFrame* pKF = new ORB_SLAM2::KeyFrame();
            cv::Mat pose_c_w=ORB_SLAM2::Converter::toCvMat(map.frames[i]->getPose()).inv();
            int desc_width=map.frames[i]->descriptors.rows();
            int desc_count=map.frames[i]->descriptors.cols();
            cv::Mat desc_mat = cv::Mat(desc_count, desc_width, CV_8UC1);
            for(int j=0; j<desc_width; j++){
                for(int k=0; k<desc_count; k++){
                    desc_mat.at<unsigned char>(k, j) = map.frames[i]->descriptors(j, k);
                } 
            }
            pKF->setData(i, map.frames[i]->time_stamp, map.frames[i]->kps, cam_info, map.frames[i]->frame_file_name, 8, 1.2, pose_c_w, 
                        desc_mat, mpMap, mpKeyFrameDatabase, mpVocabulary);
            pKF->SetGlobalMapFlag(true);
            mpMap->AddKeyFrame(pKF);
            kfs.push_back(pKF);
        }

        for(int i=0; i<map.mappoints.size(); i++){
            int mp_id=i;
            ORB_SLAM2::MapPoint* pMP=NULL;
            for (int j=0; j<map.mappoints[i]->track.size(); j++){
                int kp_id=map.mappoints[i]->track[j].kp_ind;
                int kpframe_id=map.mappoints[i]->track[j].frame->id;                
                if(pMP==NULL){
                    pMP = new ORB_SLAM2::MapPoint(ORB_SLAM2::Converter::toCvMat(map.mappoints[i]->position),kfs[kpframe_id],mpMap);
                }

                kfs[kpframe_id]->AddMapPoint(pMP,kp_id);
                pMP->AddObservation(kfs[kpframe_id],kp_id);     
                pMP->SetGlobalMapFlag(true);
            }
            mpMap->AddMapPoint(pMP);
        } 
        
        std::vector<ORB_SLAM2::MapPoint*> mps_all=mpMap->GetAllMapPoints();
        for(int i=0; i<mps_all.size(); i++){
            mps_all[i]->ComputeDistinctiveDescriptors();
            mps_all[i]->UpdateNormalAndDepth();
        }
        
        vector<ORB_SLAM2::KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it){
            (*it)->finishDataSetting();
            mpKeyFrameDatabase->add((*it));
        }
        for (vector<ORB_SLAM2::KeyFrame*>::iterator it = vpKFs.begin(); it != vpKFs.end(); ++it){
            (*it)->UpdateConnections();
        }
        first_loc_frameid=vpKFs.size();
        std::cout<<"map loaded!"<<std::endl;
     }
    
    System::System(bool do_loop_detect_flag, bool loop_for_loc)
    {
        mpVocabulary = new ORBVocabulary();//读取ORB词袋
        LOG(INFO) <<"FLAGS_voc_addr: "<<FLAGS_voc_addr;
        //bool bVocLoad= mpVocabulary->loadFromBinaryFile(FLAGS_voc_addr);
	bool bVocLoad  =false;
        if(bVocLoad==false){
            std::cout<<"try binary voc failed, use txt format to load."<<std::endl;
            mpVocabulary->load(FLAGS_voc_addr);
        }
	std::cout<<"done mpVocabulary"<<endl;
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);//创建关键帧数据库
	std::cout<<"done mpKeyFrameDatabase"<<endl;
        if(FLAGS_map_addr!=""){
            LoadORBMap(FLAGS_map_addr, mpVocabulary,  mpKeyFrameDatabase, mpMap);
        }else{
            mpMap = new Map();//创建地图对象
        }
        mpTracker = new Tracking(mpVocabulary, mpMap, mpKeyFrameDatabase,0 ,false, loop_for_loc);/////默认loop_for_loc为false
	std::cout<<"done Tracking"<<endl;
        mpLocalMapper = new LocalMapping(mpMap, true);
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, false);
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
        mpLocalMapper->SetdoLoop(do_loop_detect_flag);
        mpLocalMapper->SetLocMode(loop_for_loc);
        mpLoopCloser->SetLoopLoc(loop_for_loc);
        last_kfcount=0;
    }
    
    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp, std::string file_name)
    {
        cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp, file_name);
        return Tcw;
    }
    
    cv::Mat System::TrackLocalization(const cv::Mat &im, const double &timestamp, std::string file_name)
    {
        cv::Mat Tcw = mpTracker->Loc(im,timestamp, file_name);
        return Tcw;
    }
    
    Frame System::getCurrentFrame()
    {
        return mpTracker->mCurrentFrame;
    }
    
    Map*  System::getMapPointer()
    {
        return mpMap;
    }

    Tracking* System::getTrackPointer()
    {
        return mpTracker;
    }
    void System::getTraj(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& quas){
        std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            while(pKF->isBad())
            {
                continue;
            }
            // cv::Mat Two = vpKFs[0]->GetPoseInverse();
            // cv::Mat Trw = pKF->GetPose()*Two;
            cv::Mat Trw = pKF->GetPose();
            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            //Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(2),-twc.at<float>(1));
            Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
            posis.push_back(posi);
            Eigen::Quaterniond rot(ORB_SLAM2::Converter::toMatrix3d(Rwc));
            quas.push_back(rot);
        }
    }
    
    void System::getPC(std::vector<Eigen::Vector3d>& pcs, bool b_global_mp){
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* pMP=vpMPs[i];
            if(pMP->isBad()){
                continue;
            }
            if(b_global_mp) {
                if(!pMP->GetGlobalMapFlag())
                    continue;
            } else {
                if(pMP->GetGlobalMapFlag())
                    continue;
            }

            Eigen::Vector3d posi = Converter::toVector3d(pMP->GetWorldPos());
            pcs.push_back(posi);
        }
    }
    
    void System::getDebugImg(cv::Mat& img, float& err, int& count, int & mp_count_, int& kf_count_){
        err=0;
        count=0;
        img=cv::Mat();
        std::vector<MapPoint*> vpMPs= mpMap->GetAllMapPoints();
        std::vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        mp_count_=vpMPs.size();
        kf_count_=vpKFs.size();
        if(mpTracker->created_new_kf){
            KeyFrame* lastKF = mpTracker->last_kf;
            img=mpTracker->mKfImage;
            cv::Mat pose = lastKF->GetPose();
            Eigen::Matrix4d pose_eig= Converter::toMatrix4d(pose);
            cv::Mat img_rgb;
            cv::cvtColor(img, img_rgb, cv::COLOR_GRAY2BGRA);
            std::vector<MapPoint*> mps= lastKF->GetMapPointMatches();
            double total_error=0;
            for(int i=0; i<mps.size(); i++){
                if(mps[i]!=NULL){
                    if(!mps[i]->isBad()){
                        Eigen::Vector4d posi_mp;
                        cv::Mat posi_mp_cv = mps[i]->GetWorldPos();
                        posi_mp[0]=posi_mp_cv.at<float>(0);
                        posi_mp[1]=posi_mp_cv.at<float>(1);
                        posi_mp[2]=posi_mp_cv.at<float>(2);
                        posi_mp[3]=1;
                        Eigen::Matrix<double, 3, 4> K_eig=Eigen::Matrix<double, 3, 4>::Zero();
                        K_eig(0,0)=lastKF->fx;
                        K_eig(1,1)=lastKF->fy;
                        K_eig(0,2)=lastKF->cx;
                        K_eig(1,2)=lastKF->cy;
                        K_eig(2,2)=1;
                        Eigen::Vector3d projected_pt = K_eig*pose_eig*posi_mp;
                        cv::Point2f obs_pt(projected_pt(0)/projected_pt(2), projected_pt(1)/projected_pt(2));
                        int kp_index = mps[i]->GetIndexInKeyFrame(lastKF);
                        cv::Point2f pt= lastKF->mvKeysUn[kp_index].pt;
                        cv::Point2f d_pt=obs_pt-pt;
                        //std::cout<<pt.x<<","<<pt.y<<" "<<obs_pt.x<<","<<obs_pt.y<<std::endl;
                        total_error=total_error+sqrt(d_pt.x*d_pt.x + d_pt.y*d_pt.y);
                        cv::circle(img_rgb, obs_pt ,3 ,cv::Scalar(0, 255, 255), 1, 1 ,0);
                        count++;
                    }
                }
            }
            err=total_error/count;
            img=img_rgb;
        }
    }
    
    size_t findDesc(std::vector<std::pair<KeyFrame*, size_t>>& target_list, std::pair<KeyFrame*, size_t> query){
        size_t re=-1;
        for(int i=0; i<target_list.size(); i++){
            if(query.first==target_list[i].first && query.second==target_list[i].second){
                if(!target_list[i].first->isBad()){
                    re=i;
                }else{
                    re=0;
                }
                //std::cout<<(int)re<<std::endl;
                break;
            }
        }
        return re;
    }

	double System::distance(const cv::Mat &a, const cv::Mat &b)
	{

	

	  double sqd = 0.;
	  for(int i = 0; i < 128; i += 4)
	  {
		sqd += (a.at<float>( 0 ,i) - b.at<float>( 0 , i))*(a.at<float>( 0 ,i) -b.at<float>( 0 , i));
		sqd += (a.at<float>( 0 ,i+1) -b.at<float>( 0 , i+1))*(a.at<float>( 0 ,i+1) - b.at<float>( 0 , i+1));
		sqd += (a.at<float>( 0 ,i+2) -b.at<float>( 0 , i+2))*(a.at<float>( 0 ,i+2) - b.at<float>( 0 , i+2));
		sqd += (a.at<float>( 0 ,i+3) - b.at<float>( 0 , i+3))*(a.at<float>( 0 ,i+3) - b.at<float>( 0 , i+3));
	  }

	  return sqd;
	}
    
    void System::saveToVisualMap(string map_filename){
        vm::VisualMap map;
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
        cv::Mat Two = vpKFs[0]->GetPoseInverse();
        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
            {
                continue;
            }

            cv::Mat Trw = pKF->GetPose()*Two;
            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            Eigen::Vector3d posi(twc.at<float>(0),twc.at<float>(1),twc.at<float>(2));
            Eigen::Quaterniond rot(ORB_SLAM2::Converter::toMatrix3d(Rwc));
            //LOG(INFO)<<pKF->file_name_;
            std::vector<std::string> splited = split(pKF->file_name_, "/");
            std::string filename= splited.back();
            std::shared_ptr<vm::Frame> frame_p;
            frame_p.reset(new vm::Frame);
            frame_p->time_stamp=pKF->mTimeStamp;
            frame_p->fx=pKF->fx;
            frame_p->fy=pKF->fy;
            frame_p->cx=pKF->cx;
            frame_p->cy=pKF->cy;
            frame_p->k1=0;
            frame_p->k2=0;
            frame_p->p1=0;
            frame_p->p2=0;
            frame_p->frame_file_name=filename;
            frame_p->position=posi;
            frame_p->direction=rot;
            frame_p->kps= pKF->mvKeysUn;
            int desc_width=pKF->mDescriptors.cols;
            int desc_count=pKF->mDescriptors.rows;
            //LOG(INFO)<<pKF->mDescriptors.cols;
            //LOG(INFO)<<pKF->mDescriptors.rows;
            frame_p->descriptors.resize(desc_width, desc_count);
            for(int j=0; j<desc_width; j++){
                for(int k=0; k<desc_count; k++){
		    //std::cout<<pKF->mDescriptors.at<float>(k, j)<<",";
                    frame_p->descriptors(j, k)=pKF->mDescriptors.at<float>(k, j);
                } 
            }
            for(int j=0; j<pKF->mvKeysUn.size(); j++){
                frame_p->obss.push_back(nullptr);
            }
            frame_p->id=pKF->mnId;
            map.frames.push_back(frame_p);
        }
        
        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* mp = vpMPs[i];
            if (mp->isBad()){
                continue;
            }
			/*
			该行是为了得到所有地图点的描述子
			*/
			
				

            std::shared_ptr<vm::MapPoint> mappoint_p;
            mappoint_p.reset(new vm::MapPoint);
            cv::Mat mp_posi_cv=vpMPs[i]->GetWorldPos();
            Eigen::Vector3d posi(mp_posi_cv.at<float>(0),mp_posi_cv.at<float>(1),mp_posi_cv.at<float>(2));
            mappoint_p->position=posi;
            mappoint_p->id=vpMPs[i]->mnId;
            map.mappoints.push_back(mappoint_p);
        }
        for(int i=0; i<vpKFs.size(); i++){
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];
            if(pKF->isBad()){
                continue;
            }
            if(pKF->mnId!=map.frames[i]->id){
                std::cout<<"[error]pKF->mnId!=frame_p->id"<<std::endl;
                exit(0);
            }
            for(int j=0; j<pKF->mvpMapPoints.size(); j++){
                if(pKF->mvpMapPoints[j]!=NULL){
                    for(int k=0; k<map.mappoints.size(); k++){
                        if(map.mappoints[k]->id==pKF->mvpMapPoints[j]->mnId){
                            map.frames[i]->obss[j]=map.mappoints[k];
                            break;
                        }
                    }
                }
            }
        }
        for(size_t i=0; i<vpKFs.size(); i++){
            KeyFrame* pKF = vpKFs[i];
            if(pKF->isBad()){
                continue;
            }
            std::shared_ptr<vm::Frame> p_frame=nullptr;
            for(int k=0; k<map.frames.size(); k++){
                if(map.frames[k]->id==pKF->mnId){
                    p_frame= map.frames[k];
                    break;
                }
            }
            CHECK_NOTNULL(p_frame);
            pKF->UpdateConnections();
            pKF->UpdateBestCovisibles();
            //set<KeyFrame*> vpConnectedKFs= pKF->GetConnectedKeyFrames();
            vector<KeyFrame*> vpConnectedKFs = pKF->GetCovisiblesByWeight(30);
//             if(vpConnectedKFs.size()==0){
//                 vpConnectedKFs=pKF->GetCovisiblesByWeight(0);
//             }
            //LOG(INFO)<<vpConnectedKFs.size();
            for(vector<KeyFrame*>::const_iterator vit=vpConnectedKFs.begin(); vit!=vpConnectedKFs.end(); vit++){
                KeyFrame* pKFn = *vit;
                if(pKFn->isBad()){
                    continue;
                }
                std::shared_ptr<vm::Frame> p2_frame;
                for(int n=0; n<map.frames.size(); n++){
                    if(map.frames[n]->id==pKFn->mnId){
                        p2_frame= map.frames[n];
                        break;
                    }
                }
                CHECK_NOTNULL(p2_frame);
                map.pose_graph_v1.push_back(p_frame);
                map.pose_graph_v2.push_back(p2_frame);
                int w = pKF->GetWeight(pKFn);
                map.pose_graph_weight.push_back(w);
                map.pose_graph_e_scale.push_back(1);
                Eigen::Matrix4d r_pose = p2_frame->getPose().inverse()*p_frame->getPose();
                map.pose_graph_e_posi.push_back(r_pose.block(0,3,3,1));
                map.pose_graph_e_rot.push_back(r_pose.block(0,0,3,3));
            }
        }
        map.CalPoseEdgeVal();
        map.AssignKpToMp();
        LOG(INFO)<<map_filename+"/chamo.map";
        vm::save_visual_map(map, map_filename+"/chamo.map");
    }
    
/*
保存轨迹

*/

void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(10) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
 /*   if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }
*/

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}






    void System::saveResult(string map_filename){
        string track_file=map_filename+"/track.txt";
        string desc_file=map_filename+"/desc.txt";
        string kp_file=map_filename+"/kps.txt";
        string posi_file=map_filename+"/posi.txt";
        string traj_file=map_filename+"/traj.txt";
        
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
        cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(traj_file.c_str());
        
        int count=0;

        for(int i=0; i<vpKFs.size(); i++)
        {
            ORB_SLAM2::KeyFrame* pKF = vpKFs[i];

            if(pKF->isBad())
            {
                continue;
            }

            cv::Mat Trw = pKF->GetPose()*Two;

            cv::Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Trw.rowRange(0,3).col(3);
            
            std::vector<std::string> splited = split(pKF->file_name_, "/");
            std::string filename= splited.back();
            std::stringstream ss;
            f<<filename<<",";

            f << pKF->mnId<<","<< Rwc.at<float>(0,0) << "," << Rwc.at<float>(0,1)  << "," << Rwc.at<float>(0,2) << ","  << twc.at<float>(0) << "," <<
                Rwc.at<float>(1,0) << "," << Rwc.at<float>(1,1)  << "," << Rwc.at<float>(1,2) << ","  << twc.at<float>(1) << "," <<
                Rwc.at<float>(2,0) << "," << Rwc.at<float>(2,1)  << "," << Rwc.at<float>(2,2) << ","  << twc.at<float>(2) << endl;
            
            count++;
        }
        f.close();

        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        std::vector<cv::Mat> posis;
        
        int desc_count=0;
        int track_count=0;
        std::vector<std::pair<KeyFrame*, size_t>> desc_list;
        std::vector<std::vector<size_t>> track_list;
        std::vector<cv::Mat> posi_list;
        for(int i=0; i<vpMPs.size(); i++){
            MapPoint* mp = vpMPs[i];
            if (mp->isBad()){
                continue;
            }
            map<KeyFrame*, size_t> tracks= mp->GetObservations();
            std::vector<size_t> track_out;
            for(auto item: tracks){
                size_t desc_id = findDesc(desc_list, item);
                if(desc_id!=0){
                    if(desc_id==-1){
                        desc_list.push_back(item);
                        track_out.push_back(desc_list.size()-1);
                    }else{
                        track_out.push_back(desc_id);
                    }
                }
                
            }
            if(track_out.size()>=3){
                track_list.push_back(track_out);
                posi_list.push_back(vpMPs[i]->GetWorldPos());
            }
        }
        std::cout<<"desc_list: "<<desc_list.size()<<std::endl;
        std::cout<<"track_list: "<<track_list.size()<<std::endl;
        f.open(track_file.c_str());
        //f << fixed;
        for(int i=0; i<track_list.size(); i++){
            for (auto track: track_list[i]){
                f<<track<<",";
            }
            f<<std::endl;
        }
        f.close();
        f.open(posi_file.c_str());
        for(int i=0; i<posi_list.size(); i++){
            for(int j=0; j<3; j++){
                f<<posi_list[i].at<float>(j)<<",";
            }
            f<<std::endl;
        }
        f.close();
        f.open(kp_file.c_str());
        for(auto desc: desc_list){
            
            cv::KeyPoint kp = desc.first->mvKeysUn[desc.second];
            f<<kp.pt.x<<","<<kp.pt.y<<","<<kp.octave<<","<<desc.first->file_name_;
            f<<std::endl;
        }
        f.close();
        f.open(desc_file.c_str());
        for(auto desc: desc_list){
            
            cv::Mat desc_mat = desc.first->mDescriptors.row(desc.second);
            //std::cout<<desc_mat<<std::endl;
            for(int i=0; i<desc_mat.cols; i++){
                f<<(int)desc_mat.at<unsigned char>(0,i)<<",";
            }
            f<<std::endl;
        }
        f.close();
    }
} //namespace ORB_SLAM
