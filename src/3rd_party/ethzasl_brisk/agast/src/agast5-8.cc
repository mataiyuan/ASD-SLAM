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

// Machine generated code.
// Probability of an equal pixel on the Bresenham's circle: 0.33 and 0.1.
// Number of equal pixels to switch: 1.
// Number of unequal pixels to switch: 6.
// Memory costs: cache = 0.2.
//               same line = 1.
//               memory = 4.

#include <stdint.h>																			
#include <stdlib.h>
#include <agast/agast5-8.h>

namespace agast {
void AgastDetector5_8::detect(const unsigned char* im,
                              std::vector<agast::KeyPoint>& corners_all,
                              const agast::Mat* /*thrmap*/) {
  int total = 0;
  int nExpectedCorners = corners_all.capacity();
  agast::KeyPoint h;
  register int x, y;
  register int xsizeB = xsize - 2;
  register int ysizeB = ysize - 1;
  register int_fast16_t offset0, offset1, offset2, offset3, offset4, offset5,
      offset6, offset7;
  register int width;

  corners_all.resize(0);

  offset0 = s_offset0;
  offset1 = s_offset1;
  offset2 = s_offset2;
  offset3 = s_offset3;
  offset4 = s_offset4;
  offset5 = s_offset5;
  offset6 = s_offset6;
  offset7 = s_offset7;
  width = xsize;

  for (y = 1; y < ysizeB; y++) {
    x = 0;
    while (1) {
      homogeneous: {
        x++;
        if (x > xsizeB)
          break;
        else {
          register const unsigned char* const p = im + y * width + x;
          register const int cb = *p + b;
          register const int c_b = *p - b;
          if (p[offset0] > cb)
            if (p[offset2] > cb)
              if (p[offset3] > cb)
                if (p[offset5] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else if (p[offset7] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset1] > cb)
                  if (p[offset4] > cb)
                    goto success_homogeneous;
                  else if (p[offset7] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset7] > cb)
                if (p[offset6] > cb)
                  if (p[offset5] > cb)
                    if (p[offset1] > cb)
                      goto success_structured;
                    else if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else if (p[offset1] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset5] < c_b)
                if (p[offset3] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else if (p[offset5] > cb)
              if (p[offset7] > cb)
                if (p[offset6] > cb)
                  if (p[offset1] > cb)
                    goto success_homogeneous;
                  else if (p[offset4] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else if (p[offset5] < c_b)
              if (p[offset3] < c_b)
                if (p[offset2] < c_b)
                  if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else if (p[offset4] < c_b)
                    if (p[offset6] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset7] < c_b)
                  if (p[offset4] < c_b)
                    if (p[offset6] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else
              goto homogeneous;
          else if (p[offset0] < c_b)
            if (p[offset2] < c_b)
              if (p[offset7] > cb)
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto homogeneous;
                  else if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset5] > cb)
                  if (p[offset3] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset7] < c_b)
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto homogeneous;
                  else if (p[offset1] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset6] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else if (p[offset1] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset3] < c_b)
                if (p[offset5] < c_b)
                  if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else if (p[offset4] < c_b)
                    if (p[offset6] < c_b)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset1] < c_b)
                  if (p[offset4] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else if (p[offset5] > cb)
              if (p[offset3] > cb)
                if (p[offset2] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset7] > cb)
                  if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_structured;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else if (p[offset5] < c_b)
              if (p[offset7] < c_b)
                if (p[offset6] < c_b)
                  if (p[offset1] < c_b)
                    goto success_homogeneous;
                  else if (p[offset4] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else
              goto homogeneous;
          else if (p[offset3] > cb)
            if (p[offset5] > cb)
              if (p[offset2] > cb)
                if (p[offset1] > cb)
                  if (p[offset4] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset4] > cb)
                  if (p[offset6] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset7] > cb)
                if (p[offset4] > cb)
                  if (p[offset6] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else
              goto homogeneous;
          else if (p[offset3] < c_b)
            if (p[offset5] < c_b)
              if (p[offset2] < c_b)
                if (p[offset1] < c_b)
                  if (p[offset4] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset4] < c_b)
                  if (p[offset6] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset7] < c_b)
                if (p[offset4] < c_b)
                  if (p[offset6] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else
              goto homogeneous;
          else
            goto homogeneous;
        }
      }
      structured: {
        x++;
        if (x > xsizeB)
          break;
        else {
          register const unsigned char* const p = im + y * width + x;
          register const int cb = *p + b;
          register const int c_b = *p - b;
          if (p[offset0] > cb)
            if (p[offset2] > cb)
              if (p[offset3] > cb)
                if (p[offset5] > cb)
                  if (p[offset7] > cb)
                    if (p[offset1] > cb)
                      goto success_structured;
                    else if (p[offset4] > cb)
                      if (p[offset6] > cb)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                  else if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    goto structured;
                else if (p[offset7] > cb)
                  if (p[offset1] > cb)
                    goto success_structured;
                  else
                    goto structured;
                else if (p[offset1] > cb)
                  if (p[offset4] > cb)
                    goto success_structured;
                  else
                    goto structured;
                else
                  goto structured;
              else if (p[offset7] > cb)
                if (p[offset6] > cb)
                  if (p[offset5] > cb)
                    if (p[offset1] > cb)
                      goto success_structured;
                    else if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else if (p[offset1] > cb)
                    goto success_structured;
                  else
                    goto structured;
                else
                  goto structured;
              else if (p[offset5] < c_b)
                if (p[offset3] < c_b)
                  if (p[offset7] < c_b)
                    if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto structured;
            else if (p[offset5] > cb)
              if (p[offset7] > cb)
                if (p[offset6] > cb)
                  if (p[offset1] > cb)
                    goto success_structured;
                  else if (p[offset4] > cb)
                    goto success_structured;
                  else
                    goto structured;
                else
                  goto structured;
              else
                goto structured;
            else if (p[offset5] < c_b)
              if (p[offset3] < c_b)
                if (p[offset2] < c_b)
                  if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                  else if (p[offset4] < c_b)
                    if (p[offset6] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    goto structured;
                else if (p[offset7] < c_b)
                  if (p[offset4] < c_b)
                    if (p[offset6] < c_b)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto structured;
            else
              goto homogeneous;
          else if (p[offset0] < c_b)
            if (p[offset2] < c_b)
              if (p[offset7] > cb)
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      if (p[offset4] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                  else if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    goto structured;
                else if (p[offset5] > cb)
                  if (p[offset3] > cb)
                    if (p[offset4] > cb)
                      if (p[offset6] > cb)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                  else
                    goto homogeneous;
                else
                  goto structured;
              else if (p[offset7] < c_b)
                if (p[offset3] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else if (p[offset4] < c_b)
                      if (p[offset6] < c_b)
                        goto success_structured;
                      else
                        goto structured;
                    else
                      goto structured;
                  else if (p[offset1] < c_b)
                    goto success_structured;
                  else
                    goto structured;
                else if (p[offset6] < c_b)
                  if (p[offset5] < c_b)
                    if (p[offset1] < c_b)
                      goto success_structured;
                    else if (p[offset4] < c_b)
                      goto success_structured;
                    else
                      goto structured;
                  else if (p[offset1] < c_b)
                    goto success_structured;
                  else
                    goto structured;
                else
                  goto structured;
              else if (p[offset3] < c_b)
                if (p[offset5] < c_b)
                  if (p[offset1] < c_b)
                    if (p[offset4] < c_b)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else if (p[offset4] < c_b)
                    if (p[offset6] < c_b)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset1] < c_b)
                  if (p[offset4] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else if (p[offset5] > cb)
              if (p[offset3] > cb)
                if (p[offset2] > cb)
                  if (p[offset1] > cb)
                    if (p[offset4] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_structured;
                    else
                      goto structured;
                  else
                    goto structured;
                else if (p[offset7] > cb)
                  if (p[offset4] > cb)
                    if (p[offset6] > cb)
                      goto success_homogeneous;
                    else
                      goto homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto structured;
            else if (p[offset5] < c_b)
              if (p[offset7] < c_b)
                if (p[offset6] < c_b)
                  if (p[offset1] < c_b)
                    goto success_structured;
                  else if (p[offset4] < c_b)
                    goto success_structured;
                  else
                    goto structured;
                else
                  goto structured;
              else
                goto structured;
            else
              goto homogeneous;
          else if (p[offset3] > cb)
            if (p[offset5] > cb)
              if (p[offset2] > cb)
                if (p[offset1] > cb)
                  if (p[offset4] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset4] > cb)
                  if (p[offset6] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset7] > cb)
                if (p[offset4] > cb)
                  if (p[offset6] > cb)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else
              goto homogeneous;
          else if (p[offset3] < c_b)
            if (p[offset5] < c_b)
              if (p[offset2] < c_b)
                if (p[offset1] < c_b)
                  if (p[offset4] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else if (p[offset4] < c_b)
                  if (p[offset6] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else if (p[offset7] < c_b)
                if (p[offset4] < c_b)
                  if (p[offset6] < c_b)
                    goto success_homogeneous;
                  else
                    goto homogeneous;
                else
                  goto homogeneous;
              else
                goto homogeneous;
            else
              goto homogeneous;
          else
            goto homogeneous;
        }
      }
      success_homogeneous: if (total == nExpectedCorners) {
        if (nExpectedCorners == 0) {
          nExpectedCorners = 512;
          corners_all.reserve(nExpectedCorners);
        } else {
          nExpectedCorners *= 2;
          corners_all.reserve(nExpectedCorners);
        }
      }
      agast::KeyPointX(h) = x;
      agast::KeyPointY(h) = y;
      corners_all.push_back(h);
      total++;
      goto homogeneous;
      success_structured: if (total == nExpectedCorners) {
        if (nExpectedCorners == 0) {
          nExpectedCorners = 512;
          corners_all.reserve(nExpectedCorners);
        } else {
          nExpectedCorners *= 2;
          corners_all.reserve(nExpectedCorners);
        }
      }
      agast::KeyPointX(h) = x;
      agast::KeyPointY(h) = y;
      corners_all.push_back(h);
      total++;
      goto structured;
    }
  }
}
} // namespace agast
