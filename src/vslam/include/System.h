#ifndef SYSTEM_H
#define SYSTEM_H
#include<opencv2/core/core.hpp>
#include<string>
#include<thread>
#include "Eigen/Dense"
#include "ORBVocabulary.h"
#include "Frame.h"
namespace ORB_SLAM2
{
class Map;
class Tracking;
class LocalMapping;
class LoopClosing;
class KeyFrameDatabase;

class System
{
public:
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };
    System(bool do_loop_detect_flag=true, bool loop_for_loc = false);
    void saveResult(string map_filename);
    cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp, std::string file_name="");
    cv::Mat TrackLocalization(const cv::Mat &im, const double &timestamp, std::string file_name="");
    void getPC(std::vector<Eigen::Vector3d>& pcs, bool b_global_mp = false);
    void getTraj(std::vector<Eigen::Vector3d>& posis, std::vector<Eigen::Quaterniond>& quas);
    void getDebugImg(cv::Mat& img, float& err, int& count, int & mp_count_, int& kf_count_);
    void saveToVisualMap(string map_filename);
    Frame getCurrentFrame(); 
    Map*  getMapPointer();
    Tracking* getTrackPointer();
	void SaveTrajectoryTUM(const string &filename);
    void SaveKeyFrameTrajectoryTUM(const string &filename);
    void LoadORBMap(std::string map_name, 
                                 ORB_SLAM2::ORBVocabulary*& mpVocabulary, 
                                 ORB_SLAM2::KeyFrameDatabase*& mpKeyFrameDatabase, 
                                 ORB_SLAM2::Map*& mpMap
                   );
	double distance(const cv::Mat &a, const cv::Mat &b);///这是为了计算方差才写的

	void MappointStatical(vector<MapPoint*> vpMPs , string path);
private:
    ORBVocabulary* mpVocabulary;
    KeyFrameDatabase* mpKeyFrameDatabase;
    Map* mpMap;
    Tracking* mpTracker;
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopCloser;
    int last_kfcount;
    int first_loc_frameid;
};

}// namespace ORB_SLAM

#endif // SYSTEM_H
