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
 * Johann Prankl, 2012-02-18
 * prankl@acin.tuwien.ac.at
 */

#ifndef PCLA_CC_EUCLIDEAN_CLUSTERING_HH
#define PCLA_CC_EUCLIDEAN_CLUSTERING_HH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/feature.h>
#include <stdexcept>
#include <float.h>
#include <iostream>
#include <set>
#include <map>
#include <time.h>

namespace pclA
{


class CCEuclideanClustering
{
public:
  class Label
  {
  public:
    unsigned short id;
    Label *parent;
    unsigned short rank;
    Label(unsigned short _id) : id(_id), parent(this), rank(0) {}
  };
  class Parameter
  {
  public:
    float thr;                       // min euc. distance to connect points (0.02)
    unsigned minClusterSize;         // min cluster size
    Parameter(float th=.02, unsigned minsize=100) : thr(th), minClusterSize(minsize) {}
  };

private:
  int width, height;
  float sqrThr;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<unsigned short>::Ptr labels;
  std::vector<unsigned> sizeClusters;

  Label* Find(Label *x);
  unsigned short Union(Label *x, Label* y);

  void Clustering(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<unsigned short> &labels);


  inline float SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2);
  inline float Sqr(const float x);
  inline bool IsNaN(const pcl::PointXYZRGB &pt);
  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);


public:
  Parameter param;

  CCEuclideanClustering(Parameter _param=Parameter());
  ~CCEuclideanClustering();

  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  void compute();
  void getLabels(pcl::PointCloud<unsigned short>::Ptr &_labels);
  void getSizeClusters(std::vector<unsigned> &_sizeClusters);
};




/*********************** INLINE METHODES **************************/
inline int CCEuclideanClustering::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short CCEuclideanClustering::X(int idx)
{
  return idx%width;
}

inline short CCEuclideanClustering::Y(int idx)
{
  return idx/width;
}

inline float CCEuclideanClustering::Sqr(const float x)
{
  return x*x;
}

inline float CCEuclideanClustering::SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2)
{
  if (!IsNaN(pt1) && !IsNaN(pt2))
    return Sqr(pt1.x-pt2.x)+Sqr(pt1.y-pt2.y)+Sqr(pt1.z-pt2.z);

  return FLT_MAX;
}

inline bool CCEuclideanClustering::IsNaN(const pcl::PointXYZRGB &pt)
{
  if (pt.x!=pt.x)
    return true;
  return false;
}



}

#endif

