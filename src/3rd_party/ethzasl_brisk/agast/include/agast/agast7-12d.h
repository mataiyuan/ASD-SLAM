//
//    AGAST, an adaptive and generic corner detector based on the
//              accelerated segment test for a 8 pixel mask
//
//    Copyright (C) 2010  Elmar Mair
//    All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without
//    modification, are permitted provided that the following conditions are met:
//        * Redistributions of source code must retain the above copyright
//          notice, this list of conditions and the following disclaimer.
//        * Redistributions in binary form must reproduce the above copyright
//          notice, this list of conditions and the following disclaimer in the
//          documentation and/or other materials provided with the distribution.
//        * Neither the name of the <organization> nor the
//          names of its contributors may be used to endorse or promote products
//          derived from this software without specific prior written permission.
//
//    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
//    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
//    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//    DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
//    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
//    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
//    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef AGAST7_12D_H
#define AGAST7_12D_H

#include <stdint.h>
#include <agast/wrap-opencv.h>
#include <agast/ast-detector.h>

namespace agast {
class AgastDetector7_12d : public AstDetector {
 public:
  AgastDetector7_12d() : AstDetector() { }
  AgastDetector7_12d(int width, int height, int thr)
      : AstDetector(width, height, thr) {
    init_pattern();
  }
  ~AgastDetector7_12d() { }
  void detect(const unsigned char* im, std::vector<agast::KeyPoint>& keypoints,
              const agast::Mat* thrmap);
  void nms(const unsigned char* im, const std::vector<agast::KeyPoint>& keypoints,
           std::vector<agast::KeyPoint>& keypoints_nms);
  int get_borderWidth() {
    return borderWidth;
  }
  int cornerScore(const unsigned char* p);

 private:
  static const int borderWidth = 3;
  int_fast16_t s_offset0;
  int_fast16_t s_offset1;
  int_fast16_t s_offset2;
  int_fast16_t s_offset3;
  int_fast16_t s_offset4;
  int_fast16_t s_offset5;
  int_fast16_t s_offset6;
  int_fast16_t s_offset7;
  int_fast16_t s_offset8;
  int_fast16_t s_offset9;
  int_fast16_t s_offset10;
  int_fast16_t s_offset11;

  void init_pattern() {
    s_offset0 = (-3) + (0) * xsize;
    s_offset1 = (-2) + (-1) * xsize;
    s_offset2 = (-1) + (-2) * xsize;
    s_offset3 = (0) + (-3) * xsize;
    s_offset4 = (1) + (-2) * xsize;
    s_offset5 = (2) + (-1) * xsize;
    s_offset6 = (3) + (0) * xsize;
    s_offset7 = (2) + (1) * xsize;
    s_offset8 = (1) + (2) * xsize;
    s_offset9 = (0) + (3) * xsize;
    s_offset10 = (-1) + (2) * xsize;
    s_offset11 = (-2) + (1) * xsize;
  }
};
}  // namespace agast

#endif  // AGAST7_12D_H
