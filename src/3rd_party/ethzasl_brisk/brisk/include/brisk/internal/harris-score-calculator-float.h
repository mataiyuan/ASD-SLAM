/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INTERNAL_HARRIS_SCORE_CALCULATOR_FLOAT_H_
#define INTERNAL_HARRIS_SCORE_CALCULATOR_FLOAT_H_

#ifdef __ARM_NEON__
// Not implemented.
#else
#include <emmintrin.h>
#include <tmmintrin.h>

#include <vector>

#include <agast/wrap-opencv.h>
#include <brisk/internal/macros.h>
#include <brisk/internal/score-calculator.h>

namespace brisk {
class HarrisScoreCalculatorFloat : public ScoreCalculator<float> {
 public:
  typedef ScoreCalculator<float> Base_t;

  // Provide accessor implementations here in order to enable inlining.
  inline double Score(double u, double v) {
    // Simple bilinear interpolation - no checking (for speed).
    const int u_int = static_cast<int>(u);
    const int v_int = static_cast<int>(v);
    if (u_int + 1 >= _scores.cols || v_int + 1 >= _scores.rows || u_int < 0
        || v_int < 0)
      return 0.0;
    const double ru = u - static_cast<double>(u_int);
    const double rv = v - static_cast<double>(v_int);
    const double oneMinus_ru = 1.0 - ru;
    const double oneMinus_rv = 1.0 - rv;
    return oneMinus_rv
        * (oneMinus_ru * _scores.at<float>(v_int, u_int)
            + ru * _scores.at<float>(v_int, u_int + 1))
        + rv
            * (oneMinus_ru * _scores.at<float>(v_int + 1, u_int)
                + ru * _scores.at<float>(v_int + 1, u_int + 1));
  }
  inline Base_t::Score_t Score(int u, int v) {
    return _scores.at<float>(v, u);
  }
  virtual void Get2dMaxima(float absoluteThreshold,
                           std::vector<PointWithScore>* points);

 protected:
  // Calculates the Harris scores.
  virtual void InitializeScores();

  // Harris specific.
  static void GetCovarEntries(const agast::Mat& src, agast::Mat& dxdx, agast::Mat& dydy,
                              agast::Mat& dxdy);
  static void CornerHarris(const agast::Mat& dxdxSmooth, const agast::Mat& dydySmooth,
                           const agast::Mat& dxdySmooth, agast::Mat& score);
};
}  // namespace brisk
#endif  // __ARM_NEON__
#endif  // INTERNAL_HARRIS_SCORE_CALCULATOR_FLOAT_H_
