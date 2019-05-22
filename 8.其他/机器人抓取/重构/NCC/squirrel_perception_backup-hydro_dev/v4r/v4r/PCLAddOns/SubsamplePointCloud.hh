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

#ifndef PCLA_SUBSAMPLE_POINT_CLOUD_HH
#define PCLA_SUBSAMPLE_POINT_CLOUD_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include "pcl/filters/extract_indices.h"
#include "pcl/sample_consensus/sac_model_plane.h"
// #include "v4r/PCore/PMath.hh"
// #include "v4r/PCore/PVector.hh"
// #include "v4r/PCore/PPlane.hh"


namespace pclA 
{

/**
 * some utils for surface modeling
 */
class SubsamplePointCloud
{
public:
  class Parameter
  {
    public:
      int subsamplingFactor;     // factor for subsampling (e.g. 2 -> half size)
      int radius;
      float dist;
      bool useMean;
      bool useBilateralFilter;

      Parameter(int factor=2, int _radius=1, float _dist=0.02, bool _useMean=true, 
         bool _useBilateralFilter=false)
       : subsamplingFactor(factor), radius(_radius), dist(_dist), useMean(_useMean), 
         useBilateralFilter(_useBilateralFilter) {}
  };

private:
  Parameter param;
  float sqrDist;

  int factor;
  int width, height;
  static float NaN;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr resCloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;


  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr lsPlane;

  void Subsample(pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out);
  void SubsampleMean(pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out);
  void SubsamplePoints(pcl::PointCloud<pcl::PointXYZRGB> &cloud, unsigned u, unsigned v, pcl::PointXYZRGB &ptOut);


  inline int GetIdx(short x, short y);
  inline bool IsNaN(const pcl::PointXYZRGB &pt);
  inline float Sqr(const float x);
  inline float SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2);



public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SubsamplePointCloud(Parameter p=Parameter());
  ~SubsamplePointCloud();

  void setParameter(Parameter p);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  void compute();
  void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  void getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
};




/*********************** INLINE METHODES **************************/
inline int SubsamplePointCloud::GetIdx(short x, short y)
{
  return y*width+x;
}

inline bool SubsamplePointCloud::IsNaN(const pcl::PointXYZRGB &pt)
{
  if (pt.x!=pt.x)
    return true;
  return false;
}

inline float SubsamplePointCloud::Sqr(const float x)
{       
  return x*x;
}       
        
inline float SubsamplePointCloud::SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2)
{       
  if (pt1.x==pt1.x && pt2.x==pt2.x)
    return Sqr(pt1.x-pt2.x)+Sqr(pt1.y-pt2.y)+Sqr(pt1.z-pt2.z);
        
  return FLT_MAX;
}       

}

#endif

