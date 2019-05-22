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

#ifndef PCLA_HOLE_FILLING_HH
#define PCLA_HOLE_FILLING_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pclA 
{

/**
 * some utils for surface modeling
 */
class HoleFillingBF
{
public:
  class Parameter
  {
    public:
      float radius;                     // half kernel size
      float dSigma;                     // for gaussian weighted kernel
      float rSigma;
      float colSigma;
      float fx, fy;                     // camera parameter
      float cx, cy;

      Parameter(float r=5, float _dSigma=3, float _rSigma=.02, float _colSigma=7,
         float _fx=525, float _fy=525, float _cx=320, float _cy=240)
       : radius(r), dSigma(_dSigma), rSigma(_rSigma),colSigma(_colSigma),
         fx(_fx), fy(_fy), cx(_cx), cy(_cy) {}
  };

private:
  Parameter param;

  int width, height;
  static float NaN;

  float negInvSqrSigmaCol, negInvSqrSigmaD, negInvSqrSigmaR;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud, tmp_cloud;

  int fillHoles(const pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out);


  inline float DistS(const Eigen::Vector3f &df);
  inline float DistC(const int dx, const int dy);
  inline float DistCol(const Eigen::Vector3i &dcol);
  

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  HoleFillingBF(const Parameter &p=Parameter());
  ~HoleFillingBF();

  /** Set parameter for bilateral filtering **/
  void setParameter(const Parameter &p);

  /** Set input point cloud **/
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  /** Compute results **/
  void compute();

  /** Get bilateral-filtered point cloud **/
  void getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
};




/*********************** INLINE METHODES **************************/
inline int HoleFillingBF::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short HoleFillingBF::X(int idx)
{
  return idx%width;
}

inline short HoleFillingBF::Y(int idx)
{
  return idx/width;
}

inline float HoleFillingBF::DistC(const int dx, const int dy)
{
  return exp((float)(dx*dx + dy*dy) * negInvSqrSigmaD);
}

inline float HoleFillingBF::DistS(const Eigen::Vector3f &df)
{
  return exp(df.squaredNorm() * negInvSqrSigmaR);
}

inline float HoleFillingBF::DistCol(const Eigen::Vector3i &dcol)
{
  return exp(dcol.squaredNorm() * negInvSqrSigmaCol);
}


}

#endif

