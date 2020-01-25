/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"
#include "read_write_data_lib/read_write.h"
#include<iostream>
#include <gflags/gflags.h>
#include<mutex>

DEFINE_bool(use_orb, false, "Choose use orb or freak descriptor. Set to true if use orb.");
DEFINE_string(camera_config, "", "Config file of camera calibiration.");
DEFINE_int32(max_step_KF, 15, "The max number of frame between two KeyFrames.");
DEFINE_int32(feature_count, 2000, "Number of feature to extract.");
DEFINE_double(feature_scale_factor, 1.2, "Scale factor between levels.");
DEFINE_int32(feature_level, 8, "Pyramid levels");
DEFINE_int32(min_match_count, 200, "Min limit of matched features to add a KF.");

using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(ORBVocabulary* pVoc, Map *pMap, KeyFrameDatabase* pKFDB, const int sensor, bool bReuse, bool bloc_mode):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpMap(pMap), mnLastRelocFrameId(0),
    b_localization_mode(bloc_mode)
{
    Eigen::Matrix3d cam_inter;
    Eigen::Vector4d cam_distort;
    Eigen::Matrix4d Tbc;
    CHAMO::read_cam_info(FLAGS_camera_config, cam_inter, cam_distort, Tbc);   
    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = cam_inter(0,0);
    K.at<float>(1,1) = cam_inter(1,1);
    K.at<float>(0,2) = cam_inter(0,2);
    K.at<float>(1,2) = cam_inter(1,2);
    K.copyTo(mK);//////mk mdistcoef //Calibration matrix
    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = cam_distort(0);
    DistCoef.at<float>(1) = cam_distort(1);
    DistCoef.at<float>(2) = cam_distort(2);
    DistCoef.at<float>(3) = cam_distort(3);
    DistCoef.copyTo(mDistCoef);
    mbf = 0;
    mMinFrames = FLAGS_max_step_KF;
    mMaxFrames = FLAGS_max_step_KF;
    mbRGB = 1;
    int nFeatures = FLAGS_feature_count;
    float fScaleFactor = FLAGS_feature_scale_factor;
    int nLevels = FLAGS_feature_level;
    int fIniThFAST = 20;
    int fMinThFAST = 7;

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    reloc_fail_count=0;
    created_new_kf=false;

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp, std::string file_name)
{
    cv::undistort(im, mImGray, mK, mDistCoef);
    cv::Mat distCoefZero=cv::Mat::zeros(mDistCoef.rows, mDistCoef.cols, mDistCoef.type());
	 // 构造Frame
    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)// 没有成功初始化的前一个状态就是NO_IMAGES_YET
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,distCoefZero,mbf,mThDepth,file_name, FLAGS_use_orb);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,distCoefZero,mbf,mThDepth,file_name, FLAGS_use_orb);
    Track();

    return mCurrentFrame.mTcw.clone();
}

cv::Mat Tracking::GrabImageMonocular(Frame mframe)
{
    mCurrentFrame = mframe;
    Track();

    return mCurrentFrame.mTcw.clone();
}
cv::Mat Tracking::Loc(const cv::Mat &im, const double &timestamp, std::string file_name)
{
    cv::undistort(im, mImGray, mK, mDistCoef);
    cv::Mat distCoefZero=cv::Mat::zeros(mDistCoef.rows, mDistCoef.cols, mDistCoef.type());
    mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,distCoefZero,mbf,mThDepth,file_name, FLAGS_use_orb);
    bool bOK=true;
    if(mState!=OK){
        bOK = Relocalization();
        UpdateLocalMap();
        mLastFrame = Frame(mCurrentFrame);
    }
    if(bOK){
        CheckReplacedInLastFrame();
        if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2){
            bOK = TrackReferenceKeyFrame();
            // std::cout<<"OK4: "<<bOK<<std::endl;
        }else{
            bOK = TrackWithMotionModel();
            // std::cout<<"OK2: "<<bOK<<std::endl;
            if(!bOK){
                bOK = TrackReferenceKeyFrame();
                // std::cout<<"OK5: "<<bOK<<std::endl;
            } 
        }
        if(bOK){
            bOK = TrackLocalMap();
            // std::cout<<"OK1: "<<bOK<<std::endl;
        }
        
        if(bOK)
            mState = OK;
        else
            mState=LOST;
        
        if(bOK){
            // Update motion model
            if(!mLastFrame.mTcw.empty()){
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++){
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP){
                    if(pMP->Observations()<1){
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
                }
            }

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame()){
                CreateNewKeyFrame();
                mKfImage=mImGray;
                last_kf=mCurrentFrame.mpReferenceKF;
                created_new_kf=true;
            } else {
                created_new_kf = false;
            }

            for(int i=0; i<mCurrentFrame.N;i++){
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }
    
    if(!mCurrentFrame.mTcw.empty()){
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
    }else{
        std::cout<<"mCurrentFrame.mTcw.empty()"<<std::endl;
        // This can happen if tracking is lost
//         mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    }
    
    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{

    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;
    

    if(mState==NOT_INITIALIZED)
    {
        MonocularInitialization();

        if(mState!=OK)
            return;
    }
    else// 步骤2：跟踪
    {
        // System is initialized. Track Frame.
        bool bOK;

        if(!mbOnlyTracking)
        {
            if(mState==OK)
            {
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();//跟踪参考帧
                     std::cout<<"OK4: "<<bOK<<std::endl;
                }
                else
                {

                    bOK = TrackWithMotionModel();
                     std::cout<<"OK2: "<<bOK<<std::endl;
                    if(!bOK){
                        bOK = TrackReferenceKeyFrame();
                         std::cout<<"OK5: "<<bOK<<std::endl;
                    }
                        
                }
            }
            else
            {
                
                bOK = Relocalization();
                // std::cout<<"OK Reloc: "<<bOK<<std::endl;
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            
            if(bOK){
                bOK = TrackLocalMap();
                // std::cout<<"OK1: "<<bOK<<std::endl;
            }
            
                
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame()){
                CreateNewKeyFrame();
                mKfImage=mImGray;
                last_kf=mCurrentFrame.mpReferenceKF;
                created_new_kf=true;
                //TrackReferenceKeyFrame();
            }else{
                created_new_kf=false;
                
            }
                

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                return;
            }
            cout << "Track failded, and return!!!!!!!!!!!!!!!!!" << endl;
            return;
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
    }
    else
    {
        std::cout<<"mCurrentFrame.mTcw.empty()"<<std::endl;
        // This can happen if tracking is lost
//         mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
    }

}

void Tracking::MonocularInitialization()
{


	
    if(!mpInitializer)
    {
        // Set Reference Frame
		//step 1：第一次进入该方法,如果当前帧关键点数>100,将当前帧保存为初始帧和最后一帧，并创建一个初始化器
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
			//mvbPrevMatched的大小设置为已经提取的关键点的个数，mvbPrevMatched最大的情况就是所有特征点都被跟踪上-------//mCurrentFrame.mvKeysUn   经过矫正模型矫正的关键点坐标
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
				//mvbPrevMatched中存储关键点的坐标，用于新的一帧到来时进行匹配
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;
			//由当前帧构造初始器 sigma:1.0 iterations:200----其中iterations主要是为了ransac迭代
            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);
			//mvIniMatches中所有的元素值设置为-1
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else//如果是第二次进入，这时候已经创建了初始器
    {
       if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)
        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            std::cout<<Rcw<<std::endl;
            std::cout<<tcw<<std::endl;
			//step 5：删除那些无法进行三角化的匹配点
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
					//表示两帧对应的关键点不再匹配
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);
            CreateInitialMapMonocular();        
        }
	else
	{
		std::cout<<"单目初始化失败："<<endl;
	}
    }
}

/**
 * 功能：为关键帧初始化生成对应的MapPoints
*/

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    pKFini->mbFirstConnection=false;
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    //1：计算初始化关键帧和当前关键帧的BOW
    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    //2.将基础关键帧和当前关键帧插入地图中，地图中就会显示
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    //3.遍历所有匹配的关键点创建对应的mapPoint、
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
	//用已经初始化好的3D点来创建world坐标
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);//////这里也证明了mvIniMatches的值是当前帧特征点的索引

        /**
         * 3.2：为该MapPoint添加相关属性
        */
        //1）.哪些关键帧可以观测到该MapPoint

        pMP->AddObservation(pKFini,i);
        //mvIniMatches[i]为pMP这个MapPoint在pKFcur这个关键帧中对应的关键点的index值
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
        //2).从众多观测到该MapPoint的特征点中挑选区分度最高的描述子
        pMP->ComputeDistinctiveDescriptors();
        //3).更新该MapPoint平均观测方向以及观测距离的范围
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        //将mappoint插入到地图中
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    //4：更新关键帧间的连接关系
    //在3D点和关键帧之间建立边，每个边有一个权重，边的权重是该关键帧与当前帧公共3D点的个数
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    //6：将MapPoints的中值深度归一化到1，并归一化两帧之间变换,评估关键帧场景深度，q=2表示中值
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;////////////////////////////这里用中位数的倒数来进行类似归一化的操作，因为本来就是尺度无关的，但是呢，为了显示，万一刚开始尺度太小可能就无法显示，所以做了这一步类似归一化的操作。

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<50)
    {
        cout << "Wrong initialization, reseting..." << endl;
        //todo this reset make tool not friendly
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    //将Tc2w中的平移向量t进行了修改 ？？？？？？？？？？？？？？
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }
    //往LocalMapper中插入关键帧
    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    //将当前关键帧设置为参考关键帧
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
    mpLocalMapper->DoMapping();
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;
    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
    //changsq 15->5
    if(nmatches<5)
    {
        return false;

    }
    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    //changsq 10->5
    return nmatchesMap>=5;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.8,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    th=15;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,true);
    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,true);
    }

    if(nmatches<20)
    {
        
        return false;
    }
    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map pTrackLocalMapoints tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }

        }
    }
    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    // if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
    //     return false;

    //changsq 30->5 
    if(mnMatchesInliers<5)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mnMatchesInliers<FLAGS_min_match_count){
        return true;
    }
    if(mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames){
        return true;
    }
    return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mCurrentFrame.mpReferenceKF = pKF;
    
    mpLocalMapper->InsertKeyFrame(pKF);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
//     clock_t start,finish;
//     double totaltime;
//     start=clock();
    mpLocalMapper->DoMapping();
//     finish=clock();
//     totaltime=(double)(finish-start)/CLOCKS_PER_SEC;
//     std::cout<<"LocalMapping Time: "<<totaltime<<std::endl;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)

        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
    //ToDo: search for more global connection, need to rewrite SearchGlobalConnection 
    int count = 0;
    nToMatch = 0;
    for(int i = 0; i < mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            nToMatch++;
            if(mCurrentFrame.mvpMapPoints[i]->GetGlobalMapFlag())
                count++;
        }
        
    }
    if(count < 20 && b_localization_mode)
        std::cout << "Global mp percentage in LocalMap: " <<float(count) / nToMatch<<"  "<<count<< std::endl;
    // if(count < 20 && b_localization_mode)
    //     Relocalization(false);
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::SearchGlobalConnection()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);
    if(vpCandidateKFs.empty()){
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<5)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                std::cout << "nmatches: " <<nmatches<< std::endl;
                nCandidates++;
            }
        }
    }
    std::cout << "Global Connections nCandidates: " <<nCandidates<< std::endl;

    ORBmatcher matcher2(0.9,true);

    for (int i = 0; i < nKFs; i++) {
        if (vbDiscarded[i])
            continue;

        set<MapPoint*> sFound;

        for (int j = 0; j < vvpMapPointMatches[i].size(); j++) {
            if(vvpMapPointMatches[i][j]) {
                mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
                sFound.insert(vvpMapPointMatches[i][j]);
            }
        }
        int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

        std::cout << "nGood00: " << nGood << std::endl;
        if (nGood < 10)
            continue;

        for (int io = 0; io < mCurrentFrame.N; io++)
            if (mCurrentFrame.mvbOutlier[io])
                mCurrentFrame.mvpMapPoints[io] = static_cast<MapPoint*>(NULL);

        int nadditional =
                matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 0.7);/////之前是100 现在修改为0.5
    }

    return true;

}

bool Tracking::Relocalization(bool b_update_pose)
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame,b_localization_mode);
    if(vpCandidateKFs.empty()){
        reloc_fail_count++;
        return false;
    }

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<5)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }
    // std::cout << "Reloc nCandidates: " <<nCandidates<< std::endl;

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,0.7);//之前是100 现在修改为0.5

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,0.5);//之前是64 现在修改为0.4

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    mnLastKeyFrameId=vpCandidateKFs[i]->mnId;
                    mpLastKeyFrame = vpCandidateKFs[i];

                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        reloc_fail_count++;
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        reloc_fail_count=0;
        if(b_update_pose)
            mCurrentFrame.SetPose(mCurrentFrame.mTcw);
        std::cout << "Relocalization!" << std::endl;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}

std::vector<MapPoint*> Tracking::GetmvpLocalMapPoints()
{
    return mvpLocalMapPoints;
}
std::vector<KeyFrame*> Tracking::GetmvpLocalKeyFrames()
{
    return mvpLocalKeyFrames;
}

} //namespace ORB_SLAM
