#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include"System.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include "visualization/color-palette.h"
#include "visualization/color.h"
#include "visualization/common-rviz-visualization.h"
#include <glog/logging.h>
#include <gflags/gflags.h>

using namespace std;

DEFINE_string(bag_addr, "", "Bag contain the image data.");
DEFINE_string(output_addr, "", "Place to save the output file.");
DEFINE_string(image_topic, "img", "Topic of image in bag.");
DEFINE_int32(min_frame, 100000, "First frame to be processed.");
DEFINE_int32(max_frame, 0, "Last frame to be processed.");
DEFINE_int32(step_frame, 1, "The number of frames to be skiped.");

void show_pose_as_marker(std::vector<Eigen::Quaterniond>& rots, std::vector<Eigen::Vector3d>& posis, std::string topic){
    visualization::PoseVector poses_vis;
    for(int i=0; i<rots.size(); i=i+1){
        visualization::Pose pose;
        pose.G_p_B = posis[i];
        pose.G_q_B = rots[i];

        pose.id =poses_vis.size();
        pose.scale = 0.2;
        pose.line_width = 0.02;
        pose.alpha = 1;
        poses_vis.push_back(pose);
    }
    visualization::publishVerticesFromPoseVector(poses_vis, visualization::kDefaultMapFrame, "vertices", topic);
}

void show_mp_as_cloud(std::vector<Eigen::Vector3d>& mp_posis, std::string topic){
    Eigen::Matrix3Xd points;
    points.resize(3,mp_posis.size());
    for(int i=0; i<mp_posis.size(); i++){
        points.block<3,1>(0,i)=mp_posis[i];
    }    
    publish3DPointsAsPointCloud(points, visualization::kCommonRed, 1.0, visualization::kDefaultMapFrame,topic);
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();
    google::ParseCommandLineFlags(&argc, &argv, true);
    visualization::RVizVisualizationSink::init();
    ORB_SLAM2::System sys;
    std::string bag_str=FLAGS_bag_addr;
    std::string out_str=FLAGS_output_addr;
    std::string img_topic=FLAGS_image_topic;
    int min_frame=FLAGS_min_frame;
    int max_frame=FLAGS_max_frame;
    int step=FLAGS_step_frame;


    //topics.push_back("img");

    int img_count=-1;



    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(bag_str, vstrImageFilenames, vTimestamps);
    int nImages = vstrImageFilenames.size();
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
	cv::Mat im;
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }
            if(ni%step!=0){
                continue;
            }
            if(ni <min_frame){
                continue;
            }
            if(ni >max_frame){
                break;
            }
                sys.TrackMonocular(im, tframe, vstrImageFilenames[ni]);                
                std::vector<Eigen::Vector3d> pcs;
                sys.getPC(pcs);
                std::vector<Eigen::Vector3d> posis;
                std::vector<Eigen::Quaterniond> quas;
                sys.getTraj(posis, quas);
                float reproject_err_t;
                int match_count_t;
                int mp_count_t;
                int kf_count_t;
                cv::Mat img_display;
                sys.getDebugImg(img_display, reproject_err_t, match_count_t, mp_count_t, kf_count_t);
                if(!img_display.empty()){
                    cv::imshow("chamo", img_display);
                    show_mp_as_cloud(posis, "vslam_output_posi");
                    cv::waitKey(1);
                }                
            
        
    }
    sys.saveToVisualMap(out_str);
    sys.SaveKeyFrameTrajectoryTUM("/home/mataiyuan/visual_map/trajectory_path/KittiTrajectory.txt");
    return 0;
}
