/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * $Id$
 * Johann Prankl, 2010-12-01
 * prankl@acin.tuwien.ac.at
 */

#ifndef PCLA_CC_LABELING_HH
#define PCLA_CC_LABELING_HH 

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdexcept>
#include <float.h>
#include <iostream>
#include <set>
#include <map>
#include <time.h>

namespace pclA
{

class Label
{
public:
  ushort id;
  Label *parent;
  ushort rank;
  Label(ushort _id) : id(_id), parent(this), rank(0) {}
};


class CCLabeling
{
public:
  class Parameter
  {
  public:
    float thr;                       // min euc. distance to connect points (0.02)
    unsigned minClusterSize;         // min cluster size
    Parameter(float th=.02, unsigned minsize=100) : thr(th), minClusterSize(minsize) {}
  };

private:

  Label* Find(Label *x);
  ushort Union(Label *x, Label* y);

  inline float SqrDistance(const cv::Vec4f &pt1, const cv::Vec4f &pt2);
  inline float SqrDistanceZ(const cv::Vec4f &pt1, const cv::Vec4f &pt2);
  inline float Sqr(const float x);


public:
  Parameter param;

  CCLabeling(Parameter _param=Parameter());
  ~CCLabeling();

  void Operate(const cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, std::vector<unsigned> &cluster_size);
  void FilterClusterSize(cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, std::vector<unsigned> &cluster_size);
  void FilterLargestCluster(cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, std::vector<unsigned> &cluster_size);
  void CreateMask(const cv::Mat_<ushort> &labels, const std::vector<unsigned> &cluster_size, cv::Mat_<uchar> &mask);

};




/*********************** INLINE METHODES **************************/

inline float CCLabeling::Sqr(const float x)
{
  return x*x;
}

inline float CCLabeling::SqrDistance(const cv::Vec4f &pt1, const cv::Vec4f &pt2)
{
  if (pt2[0]==pt2[0] && pt2[1]==pt2[1] && pt2[2]==pt2[2])
    return Sqr(pt1[0]-pt2[0])+Sqr(pt1[1]-pt2[1])+Sqr(pt1[2]-pt2[2]);

  return FLT_MAX;
}

inline float CCLabeling::SqrDistanceZ(const cv::Vec4f &pt1, const cv::Vec4f &pt2)
{
  if (pt2[0]==pt2[0] && pt2[1]==pt2[1] && pt2[2]==pt2[2])
    return Sqr(pt1[2]-pt2[2]);

  return FLT_MAX;
}



}

#endif

