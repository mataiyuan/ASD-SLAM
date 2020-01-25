#pragma once
#include "visual_map/visual_map_common.h"

namespace vm {
class Frame;
class TrackItem {
public:
    std::shared_ptr<Frame> frame;
    int kp_ind;
    void getUV(float& x, float& y, int& octave);
};

class MapPoint {
public:
    int id = -1;
    int match_count=0;
    Eigen::Vector3d position;
    std::vector<TrackItem> track;
    float calDescDiff(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& query_desc);
    void getALLDesc(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& descs);
};

}  // namespace vm
