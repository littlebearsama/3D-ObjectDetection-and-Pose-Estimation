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

#ifndef PCLA_NORMALSCLUSTERING_HH
#define PCLA_NORMALSCLUSTERING_HH

#include <iostream>
#include <stdexcept>
//#include <omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <boost/shared_ptr.hpp>




namespace pclA 
{

/**
 * some utils for surface modeling
 */
class NormalsClustering
{
public:
  class Label
  {
  public:
    unsigned id;
    Label *parent;
    unsigned rank;
    Label(unsigned _id) : id(_id), parent(this), rank(0) {}
  };
  class Parameter
  {
    public:
      float thrDist;
      float thrAngle;

      Parameter(float _thrDist=0.02, float _thrAngle=0.2)
       : thrDist(_thrDist), thrAngle(_thrAngle) {}
  };

private:
  Parameter param;

  int width, height;
  static float NaN;

  float sqrThrDist;
  float cosThrAngle;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  boost::shared_ptr<std::vector<unsigned> > labels;
  boost::shared_ptr<std::vector<pcl::PointIndices::Ptr> > clusters;
  boost::shared_ptr<std::vector<unsigned> > sizeClusters;

  Label* Find(Label *x);
  unsigned Union(Label *x, Label* y);

  void Operate(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
        pcl::PointCloud<pcl::Normal> &normals, std::vector<unsigned> &labels, 
        std::vector<pcl::PointIndices::Ptr> &clusters, std::vector<unsigned> &sizeClusters);

  inline float Dot(const pcl::Normal &n1, const pcl::Normal &n2);
  inline float SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2);
  inline float Sqr(const float x);
  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  NormalsClustering(Parameter p=Parameter());
  ~NormalsClustering();

  void setParameter(Parameter p);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  void setInputNormals(const pcl::PointCloud<pcl::Normal>::Ptr &_normals);

  void compute();

  void getClusters(std::vector<pcl::PointIndices::Ptr> &_clusters);
  void getSizeClusters(std::vector<unsigned> &_sizeClusters);

  void getClusters(boost::shared_ptr<std::vector<pcl::PointIndices::Ptr> > &_clusters);
  void getSizeClusters(boost::shared_ptr<std::vector<unsigned> > &_sizeClusters);
  void getLabels(boost::shared_ptr<std::vector<unsigned> > &_labels);
};




/*********************** INLINE METHODES **************************/
inline int NormalsClustering::GetIdx(short x, short y)
{
  return y*width+x;
}

inline short NormalsClustering::X(int idx)
{
  return idx%width;
}

inline short NormalsClustering::Y(int idx)
{
  return idx/width;
}

inline float NormalsClustering::Sqr(const float x)
{
  return x*x;
}

inline float NormalsClustering::SqrDistance(const pcl::PointXYZRGB &pt1, const pcl::PointXYZRGB &pt2)
{
  if (pt1.x==pt1.x && pt2.x==pt2.x)
    return Sqr(pt1.x-pt2.x)+Sqr(pt1.y-pt2.y)+Sqr(pt1.z-pt2.z);

  return FLT_MAX;
}

inline float NormalsClustering::Dot(const pcl::Normal &n1, const pcl::Normal &n2)
{
  return n1.normal[0]*n2.normal[0] + n1.normal[1]*n2.normal[1] + n1.normal[2]*n2.normal[2];
}


}

#endif

