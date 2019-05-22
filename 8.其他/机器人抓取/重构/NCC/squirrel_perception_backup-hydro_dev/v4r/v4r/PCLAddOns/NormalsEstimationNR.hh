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
 * Johann Prankl, 6.12.2011
 */

#ifndef PCLA_NORMALS_ESTIMATION_NR_HH
#define PCLA_NORMALS_ESTIMATION_NR_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/feature.h>
#include <pcl/octree/octree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/shared_ptr.hpp>

#include "CCEuclideanClustering.hh"


namespace pclA 
{
template<typename T1,typename T2, typename T3>
extern void Div3(const T1 v[3], T2 s, T3 r[3]);

template<typename T1,typename T2, typename T3>
extern void Add3(const T1 v1[3], const T2 v2[3], T3 r[3]);

template<typename T1,typename T2>
extern T1 Dot3(const T1 v1[3], const T2 v2[3]);
  
template<typename T1>
extern T1 Norm3(const T1 v[3]);

template<typename T1, typename T2>
extern void Normalise3(const T1 v[3], T2 r[3]);

template<typename T1,typename T2, typename T3>
extern void Mul3(const T1 v[3], T2 s, T3 r[3]);

template<typename T1,typename T2>
extern T1 DistanceSqr3(const T1 d1[3], const T2 d2[3]);

/**
 * Surface normals estimation with neighborhood reorganization
 */
class NormalsEstimationNR
{
public:
  class Parameter
  {
    public:
      int radius;   
      float maxDist;
      float alpha;       //1000
      float beta;        //0.001
      int maxIter;       //5
      float epsConverge; //0.001
      float maxPointDist;
      float treeResolution;   // for octree (used for unorganized point clouds)
      bool filter;            // connected component filter of neighbours (maxPointDist)
      bool useOctree;
      Parameter(int _radius=5, float _maxDist=0.02, float _alpha=1000, float _beta=0.001, 
         int _maxIter=5, float eps=0.001, float _maxPointDist=0.02, float _treeResolution=0.03, 
         bool _filter=false, bool _useOctree=false)
       : radius(_radius), maxDist(_maxDist), alpha(_alpha), beta(_beta), 
         maxIter(_maxIter), epsConverge(eps), maxPointDist(_maxPointDist), treeResolution(_treeResolution),
         filter(_filter), useOctree(_useOctree) {}
  };

private:
  Parameter param;

  int width, height;
  float sqrMaxDist;
  float sqrMaxPointDist;
  float sqrEpsConverge;
  static float NaN;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr m0,m1, normals;
  std::vector<bool> converged;

  std::vector<std::vector<int> > neighbours;  //the first index of each neighbours vector is the point itself

  void Init();
  void InitNeighbours();  
  void InitNeighbours(const std::vector<int> &mask);
  void CCFilterNeighbours();
  void ComputeNormalsLS(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &normals);
  void ComputeNormalsWeightedLS(pcl::PointCloud<pcl::PointXYZRGB> &loud, 
         pcl::PointCloud<pcl::Normal> &normals_m, pcl::PointCloud<pcl::Normal> &normals);
  void ComputeCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
        const std::vector<int> &indices, const Eigen::Vector4f &mean, Eigen::Matrix3f &cov);
  void ComputeWeightedCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
        const std::vector<int> &indices, const std::vector<float> &weights, 
        const Eigen::Vector4f &weightedMean, Eigen::Matrix3f &cov);
  void ComputeWeightedMean (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
        const std::vector<int> &indices, const std::vector<float> &weights, Eigen::Vector4f &weightedMean);
  bool ReorganizeNeighborhood(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &normals_n, 
        pcl::PointCloud<pcl::Normal> &normals_m0, pcl::PointCloud<pcl::Normal> &normals_m1);


  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);
  inline bool IsNaN(const pcl::PointXYZRGB &pt);
  inline bool IsNaN(const pcl::Normal &n);
  inline float Sqr(const float x);
  inline float SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2);



public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NormalsEstimationNR(Parameter p=Parameter());
  ~NormalsEstimationNR();

  void setParameter(Parameter p);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  void compute();
  void compute(const std::vector<int> &mask);

  void getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
};




/*********************** INLINE METHODES **************************/
inline int NormalsEstimationNR::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short NormalsEstimationNR::X(int idx)
{
  return idx%width;
}

inline short NormalsEstimationNR::Y(int idx)
{
  return idx/width;
}

inline bool NormalsEstimationNR::IsNaN(const pcl::PointXYZRGB &pt)
{
  if (pt.x!=pt.x)
    return true;
  return false;
}

inline bool NormalsEstimationNR::IsNaN(const pcl::Normal &n)
{
  if (n.normal[0]!=n.normal[0])
    return true;
  return false;
}

inline float NormalsEstimationNR::Sqr(const float x)
{
  return x*x;
}

inline float NormalsEstimationNR::SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2)
{
  if (pt1.x==pt1.x && pt2.x==pt2.x)
    return Sqr(pt1.x-pt2.x)+Sqr(pt1.y-pt2.y)+Sqr(pt1.z-pt2.z);

  return FLT_MAX;
}


}

#endif

