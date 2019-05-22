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
 */

#ifndef PCLA_BILATERAL_FILTER_CLOUD_HH
#define PCLA_BILATERAL_FILTER_CLOUD_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>

namespace pclA 
{

/**
 * some utils for surface modeling
 */
class BilateralFilter
{
public:
  class Parameter
  {
    public:
      float radius;                     // half kernel size
      float dSigma;
      float rSigma;

      Parameter(float r=9, float _dSigma=5, float _rSigma=.02)
       : radius(r), dSigma(_dSigma), rSigma(_rSigma) {}
  };

private:
  Parameter param;

  int width, height;
  static float NaN;

  float invSqrSigmaD, invSqrSigmaR;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud;

  void FilterCloud(const pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out);
  void FilterCloud(const pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out, const std::vector<int> &indices);


  inline float DistC(const int dx, const int dy);
  inline float DistS(const float df[3]);
  inline float DistS(const Eigen::Vector3f &df);

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BilateralFilter(Parameter p=Parameter());
  ~BilateralFilter();

  /** Set parameter for bilateral filtering **/
  void setParameter(Parameter p);

  /** Set input point cloud **/
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  /** Compute results **/
  void compute();

  /** Compute results for a set of points **/
  void compute(const std::vector<int> &indices);

  /** Get bilateral-filtered point cloud **/
  void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
};




/*********************** INLINE METHODES **************************/
inline int BilateralFilter::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short BilateralFilter::X(int idx)
{
  return idx%width;
}

inline short BilateralFilter::Y(int idx)
{
  return idx/width;
}

inline float BilateralFilter::DistC(const int dx, const int dy)
{
  return exp(-0.5 * (float)(dx*dx + dy*dy) * invSqrSigmaD);
}

inline float BilateralFilter::DistS(const float df[3])
{
  return exp(-0.5 * (float)(df[0]*df[0] + df[1]*df[1] + df[2]*df[2]) * invSqrSigmaR);
}

inline float BilateralFilter::DistS(const Eigen::Vector3f &df)
{
  return exp(-0.5 * (float)(df.squaredNorm()) * invSqrSigmaR);
}


}

#endif

