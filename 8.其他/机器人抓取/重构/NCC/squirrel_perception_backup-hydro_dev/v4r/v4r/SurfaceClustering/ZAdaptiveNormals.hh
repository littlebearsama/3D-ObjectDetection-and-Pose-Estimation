/**
 * $Id$
 *
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

#ifndef SURFACE_ZADAPTIVE_NORMALS_HH
#define SURFACE_ZADAPTIVE_NORMALS_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/eigen.h>
#include <pcl/common/eigen.h>
#include <boost/shared_ptr.hpp>

#include "v4r/EPUtils/EPUtils.hpp"


namespace surface 
{
/**
 * Surface normals estimation
 */
template <typename T>
class ZAdaptiveNormals
{
public:
  class Parameter
  {
    public:
      double radius;            // euclidean inlier radius
      int kernel;               // kernel radius [px]
      bool adaptive;            // Activate z-adaptive normals calcualation
      float kappa;              // gradient
      float d;                  // constant
      float kernel_radius[8];   // Kernel radius for each 0.5 meter intervall (0-4m)
      Parameter(double _radius=0.02, int _kernel=5, bool _adaptive=false, float _kappa=0.005125, float _d = 0.0)
       : radius(_radius), kernel(_kernel), adaptive(_adaptive), kappa(_kappa), d(_d) {}
  };

private:
  Parameter param;

  float NaN;
  int width, height;

  float sqr_radius;
  
  cv::Mat mask;

  typename pcl::PointCloud<T>::Ptr cloud;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  void estimateNormals();
  void getIndices(int u, int v, int kernel, std::vector<int> &indices) const;
  float computeNormal(const std::vector<int> &indices, Eigen::Matrix3f &eigen_vectors) const;


  inline int getIdx(short x, short y) const;
  inline short X(int idx) const;
  inline short Y(int idx) const;
  inline bool checkNotNaN(const T &p) const;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ZAdaptiveNormals(Parameter p=Parameter());
  ~ZAdaptiveNormals();

  void setParameter(Parameter p);
  void setInputCloud(const typename pcl::PointCloud<T>::Ptr &_cloud);

  void compute();
  void compute(const std::vector<int> &_mask);

  void getNormals(pcl::PointCloud<pcl::Normal> &_normals);
  // for compatibility
  void getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
  // print normals
  void print(std::string file_name);
};




/*********************** INLINE METHODES **************************/

template <typename T>
inline int ZAdaptiveNormals<T>::getIdx(short x, short y) const
{
  return y*width+x;
}

template <typename T>
inline short ZAdaptiveNormals<T>::X(int idx) const
{
  return idx%width;
}

template <typename T>
inline short ZAdaptiveNormals<T>::Y(int idx) const
{
  return idx/width;
}

// return true of point is not NaN
template <typename T>
inline bool ZAdaptiveNormals<T>::checkNotNaN(const T &p) const
{
  if(std::isnan(p.x) ||
     std::isnan(p.y) ||
     std::isnan(p.z))
  {
    return(false);
  }
  return(true);
}

/********************** ZAdaptiveNormals ************************
 * Constructor/Destructor
 */

template <typename T>
ZAdaptiveNormals<T>::ZAdaptiveNormals(Parameter p)
{
  NaN = std::numeric_limits<float>::quiet_NaN();
  
  setParameter(p);
  param.kernel_radius[0] = 3;
  param.kernel_radius[1] = 3;
  param.kernel_radius[2] = 3;
  param.kernel_radius[3] = 3;
  param.kernel_radius[4] = 4;
  param.kernel_radius[5] = 5;
  param.kernel_radius[6] = 6;
  param.kernel_radius[7] = 7;
}

template <typename T>
ZAdaptiveNormals<T>::~ZAdaptiveNormals()
{
}

/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
template <typename T>
void ZAdaptiveNormals<T>::setInputCloud(const typename pcl::PointCloud<T>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error ("[ZAdaptiveNormals::compute] Need an organized point cloud!");
  
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
  
  normals.reset(new pcl::PointCloud<pcl::Normal>);
  normals->points.resize(cloud->points.size());
  normals->width = cloud->width;
  normals->height = cloud->height;
  normals->is_dense = cloud->is_dense;
}

/**
 * compute the normals
 */
template <typename T>
void ZAdaptiveNormals<T>::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[ZAdaptiveNormals::compute] No point cloud available!");
  
  mask = cv::Mat_<int>::ones(height,width);
  
  estimateNormals();
}

/**
 * compute the normals using a mask
 */
template <typename T>
void ZAdaptiveNormals<T>::compute(const std::vector<int> &_mask)
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[ZAdaptiveNormals::compute] No point cloud available!");
  
  mask = cv::Mat_<int>::zeros(height,width);
  for(int i = 0; i < _mask.size(); ++i)
  {
    int idx = _mask.at(i);
    mask.at<int>(Y(idx),X(idx)) = 1;
  }
  
  estimateNormals();
  
}


/**
 * getNormals
 */
template <typename T>
void ZAdaptiveNormals<T>::getNormals(pcl::PointCloud<pcl::Normal> &_normals)
{
  _normals = *normals;
}

template <typename T>
void ZAdaptiveNormals<T>::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  _normals = normals;
}

/**
 * setParameter
 */
template <typename T>
void ZAdaptiveNormals<T>::setParameter(Parameter p)
{
  param = p;
  sqr_radius = p.radius*p.radius;
}

/************************** PRIVATE ************************/

/**
 * GetIndices of the neigbourhood of the point depending on the kernel size
 */
template <typename T>
void ZAdaptiveNormals<T>::getIndices(int u, int v, int kernel, std::vector<int> &indices) const
{
  indices.clear();
  
  const T &pt = cloud->points.at(getIdx(u,v));
  
  for (int vkernel = -kernel; vkernel <= kernel; ++vkernel)
  {
    for (int ukernel = -kernel; ukernel <= kernel; ++ukernel)
    {
      int y = v + vkernel;
      int x = u + ukernel;
      
      float center_dist = sqrt(vkernel*vkernel + ukernel*ukernel);
      
      if ( (x > 0) && (y > 0) && (x < width) && (y < height))
      {
        int idx = getIdx(x,y);
        const T &pt1 = cloud->points.at(idx);
        
        if(checkNotNaN(pt1))
        {
          float new_sqr_radius = sqr_radius;
          
          if(param.adaptive)
          {
            float val = param.kappa * center_dist * pt1.z + param.d;
            new_sqr_radius = val*val;
          }
          
          if ((pt.getVector3fMap()-pt1.getVector3fMap()).squaredNorm() < new_sqr_radius)
          {
            indices.push_back(idx);
          }
        }
      }
    }
  }
}

/**
 * ComputeNormal
 */
template <typename T>
float ZAdaptiveNormals<T>::computeNormal(const std::vector<int> &indices, Eigen::Matrix3f &eigen_vectors) const
{
  if (indices.size()<4)
    return NaN;
  
  Eigen::Vector3f mean;
  EPUtils::computeMean<T>(*cloud,mean,indices);
  
  Eigen::Matrix3f cov;
  EPUtils::computeCovarianceMatrix<T>(*cloud,mean,cov,indices);
  
  Eigen::Vector3f eigen_values;
  pcl::eigen33 (cov, eigen_vectors, eigen_values);
  float eigsum = eigen_values.sum();
  if (eigsum != 0)
    return fabs (eigen_values[0] / eigsum );
  
  return NaN;
}


/**
 * EstimateNormals
 */
template <typename T>
void ZAdaptiveNormals<T>::estimateNormals()
{
  bool havenan = false;
  
  #pragma omp parallel for shared(havenan)
  for (int v=0; v<height; v++)
  {
    for (int u=0; u<width; u++)
    {
    if(mask.at<int>(v,u) > 0)
    {
      std::vector<int> indices;
      int idx = getIdx(u,v);
      T &pt = cloud->points.at(idx);
      pcl::Normal &n = normals->points.at(idx);
      if(checkNotNaN(pt))
      {
        if(param.adaptive)
        {
          int dist = (int) (pt.z*2); // *2 => every 0.5 meter another kernel radius
	  if(dist > 7)
	    dist = 7;
          getIndices(u,v, param.kernel_radius[dist], indices);
        }
        else
          getIndices(u,v, param.kernel, indices);
      }
        
      if (indices.size()<4)
      {
        #pragma omp critical
        {
          havenan = true;
        }
        n.normal[0] = NaN;
        n.normal[1] = NaN;
        n.normal[2] = NaN;
        pt.x = NaN;
        pt.y = NaN;
        pt.z = NaN;
        continue;
      }
        
      EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
      n.curvature = computeNormal(indices, eigen_vectors);
        
      n.normal[0] = eigen_vectors (0,0);
      n.normal[1] = eigen_vectors (1,0);
      n.normal[2] = eigen_vectors (2,0);
        
      // orient normal to us
      if (n.getNormalVector3fMap().dot(pt.getVector3fMap()) > 0)
      {
        n.getNormalVector4fMap() *= -1;
      }
      // the fourth parameter is to complete hessian form --> d coefficient in the plane
      n.getNormalVector4fMap()[3] = 0;
      n.getNormalVector4fMap()[3] = -1 * n.getNormalVector3fMap().dot(pt.getVector3fMap());
    }
      
    }
  }
  
  if (havenan)
  {
    cloud->is_dense=false;
    normals->is_dense=false;
  }
}

/**
 * Print normals into file
 */
template <typename T>
void ZAdaptiveNormals<T>::print(std::string file_name)
{
  FILE *f = std::fopen(file_name.c_str(), "w");
  for(int i = 0; i < normals->size(); ++i)
  {
    pcl::Normal n = normals->points.at(i);
    fprintf(f,"%d %f %f %f %f \n",i,n.normal[0],n.normal[1],n.normal[2],n.curvature);
  }
  std::fclose(f);
}

}//namespace surface 

#endif

