#include "visual_map/map_point.h"
#include <glog/logging.h>
#include "visual_map/frame.h"
namespace vm {

float DescriptorDistance(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& a, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& b){


        float dist=0;
        int count=a.rows();
        for(int i=0; i<count; i++)
        {
		dist = dist + ( a(i,0) - b(i, 0) )*( a(i,0) - b(i, 0) );
        }
	
        return dist;
    }

void TrackItem::getUV(float& x, float& y, int& octave)
{
    if ((size_t)kp_ind >= frame->kps.size()) {
        std::cout << "[TrackItem::getUV][error]kp_ind>=frame->kps.size()" << std::endl;
        exit(0);
    }
    x = frame->kps[kp_ind].pt.x;
    y = frame->kps[kp_ind].pt.y;
    octave = frame->kps[kp_ind].octave;
}
    
    void MapPoint::getALLDesc(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& descs){
        if(track.size()==0){
            return;
        }
        int desc_width=track[0].frame->descriptors.rows();
        descs.resize(desc_width, track.size());
        for(int i=0; i<track.size(); i++){
            std::shared_ptr<vm::Frame> frame=track[i].frame;
            int kp_ind=track[i].kp_ind;
            descs.block(0,i, desc_width,1)=frame->descriptors.block(0,kp_ind, desc_width,1);
        }
    }
    
    float MapPoint::calDescDiff(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& query_desc){
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> target_descs;
        float min_diff=100000;
        int desc_width=query_desc.rows();
        getALLDesc(target_descs);
        for(int i=0; i<target_descs.cols(); i++){
            Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> tar_desc = target_descs.block(0,i,desc_width, 1);
            float diff = DescriptorDistance(tar_desc, query_desc);
            if(diff<min_diff){
                min_diff=diff;
            }
        }
        return min_diff;
    }

}  // namespace vm
