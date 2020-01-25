/**
 * File: FSift.cpp
 * Date: April 2015
 * Author: Thierry Malon
 * Description: functions for Sift descriptors
 * License: see the LICENSE.txt file
 *
 */

#include <vector>
#include <string>
#include <sstream>
#include<iostream>
#include "FClass.h"
#include "FSift.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void FSift::meanValue(const std::vector<FSift::pDescriptor> &descriptors,
  FSift::TDescriptor &mean)
{
/*
  vector<float> mean_tmp = vector<float>( (mean.reshape(1, 1)));
  mean_tmp.resize(0);
  mean_tmp.resize(FSift::L, 0);

  float s = descriptors.size();

  vector<FSift::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
     FSift::pDescriptor it_tmp = *it;
    vector<float> desc_tmp = vector<float>( (it_tmp->reshape(1, 1)));
   // const FSift::TDescriptor &desc = **it;
    for(int i = 0; i < FSift::L; i += 4)
    {
      mean_tmp[i  ] += desc_tmp[i  ] / s;
      mean_tmp[i+1] += desc_tmp[i+1] / s;
      mean_tmp[i+2] += desc_tmp[i+2] / s;
      mean_tmp[i+3] += desc_tmp[i+3] / s;
    }
  }

  cv::Mat mat = cv::Mat(mean_tmp);//将vector变成单列的mat
  mean = mat.reshape(1,1).clone();//PS：必须clone()一份，否则返回出错
*/


  mean = FSift::TDescriptor::zeros(1, FSift::L , CV_32FC1);///////这句话很重要，一定要重新给ｍｅａｎ分配，不然出奇怪的出错

  float s = descriptors.size();

  vector<FSift::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
     FSift::pDescriptor it_tmp = *it;
    //vector<float> desc_tmp = vector<float>( (it_tmp->reshape(1, 1)));

    for(int i = 0; i < FSift::L; i += 4)
    {
      
      mean.at<float>(0 , i) += it_tmp->at<float>(0 , i) / s;
      mean.at<float>(0 , i+1) += it_tmp->at<float>(0 , i+1)  / s;
      mean.at<float>(0 , i+2) += it_tmp->at<float>(0 , i+2)  / s;
      mean.at<float>(0 , i+3) += it_tmp->at<float>(0 , i+3)  / s;

    }
  }









}

// --------------------------------------------------------------------------

double FSift::distance(const FSift::TDescriptor &a, const FSift::TDescriptor &b)
{

	

  double sqd = 0.;
  for(int i = 0; i < FSift::L; i += 4)
  {
    sqd += (a.at<float>( 0 ,i) - b.at<float>( 0 , i))*(a.at<float>( 0 ,i) -b.at<float>( 0 , i));
    sqd += (a.at<float>( 0 ,i+1) -b.at<float>( 0 , i+1))*(a.at<float>( 0 ,i+1) - b.at<float>( 0 , i+1));
    sqd += (a.at<float>( 0 ,i+2) -b.at<float>( 0 , i+2))*(a.at<float>( 0 ,i+2) - b.at<float>( 0 , i+2));
    sqd += (a.at<float>( 0 ,i+3) - b.at<float>( 0 , i+3))*(a.at<float>( 0 ,i+3) - b.at<float>( 0 , i+3));
  }

  return sqd;
}

// --------------------------------------------------------------------------

std::string FSift::toString(const FSift::TDescriptor &a)
{
	stringstream ss;
	  for(int i = 0; i < FSift::L; ++i)
	  {
	    ss <<a.at<float>( 0 , i) << " ";
	  }

	  return ss.str();
}

	// --------------------------------------------------------------------------

void FSift::fromString(FSift::TDescriptor &a, const std::string &s)
{
	
	  a = FSift::TDescriptor(1 , FSift::L , CV_32FC1);

	  stringstream ss(s);

	  for(int i = 0; i < FSift::L; ++i)
	  {

	    ss >> a.at<float>( 0 , i);

	  }



}


// --------------------------------------------------------------------------

/*
void FSift::toMat32F(const std::vector<TDescriptor> &descriptors,
    cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }

  const int N = descriptors.size();
  const int L = FSift::L;

  mat.create(N, L, CV_32F);

  for(int i = 0; i < N; ++i)
  {
    const TDescriptor& desc = descriptors[i];
    float *p = mat.ptr<float>(i);
    for(int j = 0; j < L; ++j, ++p)
    {
      *p = desc[j];
    }
  }
}
*/

// --------------------------------------------------------------------------

} // namespace DBoW2


