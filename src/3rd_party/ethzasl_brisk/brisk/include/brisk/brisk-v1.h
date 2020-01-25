/*
    BRISK - Binary Robust Invariant Scalable Keypoints
    Reference implementation of
    [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
      Binary Robust Invariant Scalable Keypoints, in Proceedings of
      the IEEE International Conference on Computer Vision (ICCV2011).

    Copyright (C) 2011  The Autonomous Systems Lab (ASL), ETH Zurich,
    Stefan Leutenegger, Simon Lynen and Margarita Chli.

    This file is part of BRISK.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
       * Neither the name of the ASL nor the names of its contributors may be
         used to endorse or promote products derived from this software without
         specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _BRISK_H_
#define _BRISK_H_

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <agast/oast9-16.h>
#include <agast/agast7-12s.h>
#include <agast/agast5-8.h>
#include <emmintrin.h>

#ifndef M_PI
  #define M_PI 3.141592653589793
#endif

namespace brisk_v1{
using namespace cv; // BAD!
// some helper structures for the Brisk pattern representation
struct BriskPatternPoint{
  float x;         // x coordinate relative to center
  float y;         // x coordinate relative to center
  float sigma;     // Gaussian smoothing sigma
};
struct BriskShortPair{
  unsigned int i;  // index of the first pattern point
  unsigned int j;  // index of other pattern point
};
struct BriskLongPair{
  unsigned int i;  // index of the first pattern point
  unsigned int j;  // index of other pattern point
  int weighted_dx; // 1024.0/dx
  int weighted_dy; // 1024.0/dy
};

  // this is needed to avoid aliasing issues with the __m128i data type:
#ifdef __GNUC__
  typedef unsigned char __attribute__ ((__may_alias__)) UCHAR_ALIAS;
  typedef unsigned short __attribute__ ((__may_alias__)) UINT16_ALIAS;
  typedef unsigned int __attribute__ ((__may_alias__)) UINT32_ALIAS;
  typedef unsigned long int __attribute__ ((__may_alias__)) UINT64_ALIAS;
  typedef int __attribute__ ((__may_alias__)) INT32_ALIAS;
  typedef uint8_t __attribute__ ((__may_alias__)) U_INT8T_ALIAS;
#endif
#ifdef _MSC_VER
  // Todo: find the equivalent to may_alias
  #define UCHAR_ALIAS unsigned char //__declspec(noalias)
  #define UINT32_ALIAS unsigned int //__declspec(noalias)
  #define __inline__ __forceinline
#endif

  class CV_EXPORTS BriskDescriptorExtractor : public cv::DescriptorExtractor{
  public:
    // create a descriptor with standard pattern
    BriskDescriptorExtractor(bool rotationInvariant=true, bool scaleInvariant=true, float patternScale=1.0f);
    // custom setup
    BriskDescriptorExtractor(std::vector<float> &radiusList, std::vector<int> &numberList,
      bool rotationInvariant=true, bool scaleInvariant=true,
      float dMax=5.85f, float dMin=8.2f, std::vector<int> indexChange=std::vector<int>());
    virtual ~BriskDescriptorExtractor();

    // call this to generate the kernel:
    // circle of radius r (pixels), with n points;
    // short pairings with dMax, long pairings with dMin
    void generateKernel(std::vector<float> &radiusList,
      std::vector<int> &numberList, float dMax=5.85f, float dMin=8.2f,
      std::vector<int> indexChange=std::vector<int>());

    // TODO: implement read and write functions
    //virtual void read( const cv::FileNode& );
    //virtual void write( cv::FileStorage& ) const;

    int descriptorSize() const;
    int descriptorType() const;

    bool rotationInvariance;
    bool scaleInvariance;

    // this is the subclass keypoint computation implementation: (not meant to be public - hacked)
    virtual void computeImpl(const Mat& image, std::vector<KeyPoint>& keypoints,
        Mat& descriptors) const;

    //opencv 2.1{
    virtual void compute(const Mat& image, std::vector<KeyPoint>& keypoints, Mat& descriptors) const{
      computeImpl(image,keypoints,descriptors);
    }
    //}

  protected:
    __inline__ int smoothedIntensity(const cv::Mat& image,
        const cv::Mat& integral,const float key_x,
          const float key_y, const unsigned int scale,
          const unsigned int rot, const unsigned int point) const;
    // pattern properties
    BriskPatternPoint* patternPoints_;  //[i][rotation][scale]
    unsigned int points_;         // total number of collocation points
    float* scaleList_;          // lists the scaling per scale index [scale]
    unsigned int* sizeList_;      // lists the total pattern size per scale index [scale]
    static const unsigned int scales_;  // scales discretization
    static const float scalerange_;   // span of sizes 40->4 Octaves - else, this needs to be adjusted...
    static const unsigned int n_rot_; // discretization of the rotation look-up

    // pairs
    int strings_;           // number of uchars the descriptor consists of
    float dMax_;            // short pair maximum distance
    float dMin_;            // long pair maximum distance
    BriskShortPair* shortPairs_;    // d<_dMax
    BriskLongPair* longPairs_;      // d>_dMin
    unsigned int noShortPairs_;     // number of shortParis
    unsigned int noLongPairs_;      // number of longParis

    // general
    static const float basicSize_;
  };

  // a layer in the Brisk detector pyramid
  class CV_EXPORTS BriskLayer
  {
  public:
    // constructor arguments
    struct CV_EXPORTS CommonParams
    {
      static const int HALFSAMPLE = 0;
      static const int TWOTHIRDSAMPLE = 1;
    };
    // construct a base layer
    BriskLayer(const cv::Mat& img, float scale=1.0f, float offset=0.0f);
    // derive a layer
    BriskLayer(const BriskLayer& layer, int mode);

    // Fast/Agast without non-max suppression
    void getAgastPoints(uint8_t threshold, std::vector<agast::KeyPoint>& keypoints);

    // get scores - attention, this is in layer coordinates, not scale=1 coordinates!
    inline uint8_t getAgastScore(int x, int y, uint8_t threshold);
    inline uint8_t getAgastScore_5_8(int x, int y, uint8_t threshold);
    inline uint8_t getAgastScore(float xf, float yf, uint8_t threshold, float scale=1.0f);

    // accessors
    inline const cv::Mat& img() const {return img_;}
    inline const cv::Mat& scores() const {return scores_;}
    inline float scale() const {return scale_;}
    inline float offset() const {return offset_;}

    // half sampling
    static inline void halfsample(const cv::Mat& srcimg, cv::Mat& dstimg);
    // two third sampling
    static inline void twothirdsample(const cv::Mat& srcimg, cv::Mat& dstimg);

  private:
    // access gray values (smoothed/interpolated)
    __inline__ uint8_t value(const cv::Mat& mat, float xf, float yf, float scale);
    // the image
    cv::Mat img_;
    // its Fast scores
    cv::Mat scores_;
    // coordinate transformation
    float scale_;
    float offset_;
    // agast
    cv::Ptr<agast::OastDetector9_16> oastDetector_;
    cv::Ptr<agast::AgastDetector5_8> agastDetector_5_8_;
  };

  class CV_EXPORTS BriskScaleSpace
  {
  public:
    // construct telling the octaves number:
    BriskScaleSpace(uint8_t _octaves=3);
    ~BriskScaleSpace();

    // construct the image pyramids
    void constructPyramid(const cv::Mat& image);

    // get Keypoints
    void getKeypoints(const uint8_t _threshold, std::vector<cv::KeyPoint>& keypoints);

  protected:
    // nonmax suppression:
    __inline__ bool isMax2D(const uint8_t layer,
        const int x_layer, const int y_layer);
    // 1D (scale axis) refinement:
    __inline__ float refine1D(const float s_05,
        const float s0, const float s05, float& max); // around octave
    __inline__ float refine1D_1(const float s_05,
        const float s0, const float s05, float& max); // around intra
    __inline__ float refine1D_2(const float s_05,
        const float s0, const float s05, float& max); // around octave 0 only
    // 2D maximum refinement:
    __inline__ float subpixel2D(const int s_0_0, const int s_0_1, const int s_0_2,
        const int s_1_0, const int s_1_1, const int s_1_2,
        const int s_2_0, const int s_2_1, const int s_2_2,
        float& delta_x, float& delta_y);
    // 3D maximum refinement centered around (x_layer,y_layer)
    __inline__ float refine3D(const uint8_t layer,
        const int x_layer, const int y_layer,
        float& x, float& y, float& scale, bool& ismax);

    // interpolated score access with recalculation when needed:
    __inline__ int getScoreAbove(const uint8_t layer,
        const int x_layer, const int y_layer);
    __inline__ int getScoreBelow(const uint8_t layer,
        const int x_layer, const int y_layer);

    // return the maximum of score patches above or below
    __inline__ float getScoreMaxAbove(const uint8_t layer,
        const int x_layer, const int y_layer,
        const int threshold, bool& ismax,
        float& dx, float& dy);
    __inline__ float getScoreMaxBelow(const uint8_t layer,
        const int x_layer, const int y_layer,
        const int threshold, bool& ismax,
        float& dx, float& dy);

    // the image pyramids:
    uint8_t layers_;
    std::vector<brisk_v1::BriskLayer> pyramid_;

    // Agast:
    uint8_t threshold_;
    uint8_t safeThreshold_;

    // some constant parameters:
    static const float safetyFactor_;
    static const float basicSize_;
  };

  // wrapping class for the common interface
  class CV_EXPORTS BriskFeatureDetector : public FeatureDetector
  {
  public:
    BriskFeatureDetector(int thresh, int octaves=3);
    //~FastSseFeatureDetector();
    int threshold;
    int octaves;
  protected:
    // also this should in fact be protected...:
    virtual void detectImpl( const cv::Mat& image,
        std::vector<cv::KeyPoint>& keypoints,
        const cv::Mat& mask=cv::Mat() ) const;
  };
}

#endif //_BRISK_H_
