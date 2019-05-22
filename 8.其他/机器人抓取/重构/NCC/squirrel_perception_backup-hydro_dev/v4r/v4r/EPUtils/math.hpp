/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
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


#ifndef EPMATH_H
#define EPMATH_H

#include "headers.hpp"

namespace EPUtils
{

float dotProduct(Eigen::Vector3f v1, Eigen::Vector3f v2);
float dotProduct(cv::Point3f v1, cv::Point3f v2);

float vectorLength(Eigen::Vector3f v);
float vectorLength(cv::Point3d v);

float calculateCosine(Eigen::Vector3f v1, Eigen::Vector3f v2);
float calculateCosine(cv::Point3d v1, cv::Point3d v2);

Eigen::Vector3f normalize(Eigen::Vector3f v);
cv::Point3d normalize(cv::Point3d v);

Eigen::Vector3f crossProduct(Eigen::Vector3f v1, Eigen::Vector3f v2);
cv::Point3d crossProduct(cv::Point3d v1, cv::Point3d v2);

Eigen::Vector3f crossProduct(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);
cv::Point3d crossProduct(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3);

Eigen::Vector3f calculatePlaneNormal(Eigen::Vector3f v1, Eigen::Vector3f v2);
Eigen::Vector3f calculatePlaneNormal(Eigen::Vector3f p1, Eigen::Vector3f p2, Eigen::Vector3f p3);

cv::Point3d calculatePlaneNormal(cv::Point3d v1, cv::Point3d v2);
cv::Point3d calculatePlaneNormal(cv::Point3d p1, cv::Point3d p2, cv::Point3d p3);

#ifndef NOT_USE_PCL

float dotProduct(pcl::Normal v1, pcl::Normal v2);
float dotProduct(pcl::PointXYZ v1, pcl::PointXYZ v2);

float vectorLength(pcl::Normal v);
float vectorLength(pcl::PointXYZ v);

float calculateCosine(pcl::Normal v1, pcl::Normal v2);
float calculateCosine(pcl::PointXYZ v1, pcl::PointXYZ v2);

pcl::Normal normalize(pcl::Normal v);
pcl::PointXYZ normalize(pcl::PointXYZ v);

pcl::Normal crossProduct(pcl::Normal v1, pcl::Normal v2);
pcl::PointXYZ crossProduct(pcl::PointXYZ v1, pcl::PointXYZ v2);

pcl::Normal crossProduct(pcl::Normal p1, pcl::Normal p2, pcl::Normal p3);
pcl::PointXYZ crossProduct(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);

pcl::Normal calculatePlaneNormal(pcl::Normal v1, pcl::Normal v2);
pcl::Normal calculatePlaneNormal(pcl::Normal p1, pcl::Normal p2, pcl::Normal p3);
pcl::PointXYZ calculatePlaneNormal(pcl::PointXYZ v1, pcl::PointXYZ v2);
pcl::PointXYZ calculatePlaneNormal(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);

void ProjectPointsOnThePlane(pcl::ModelCoefficients::ConstPtr coefficients,
                             pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                             pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_projected,
                             std::vector<float> &distances, 
                             pcl::PointIndices::Ptr indices = pcl::PointIndices::Ptr(new pcl::PointIndices()),
                             bool normalize = true);

template <class T>
bool computeMean(const typename pcl::PointCloud<T> &cloud, Eigen::Vector3f &mean,
                 std::vector<int> indices = std::vector<int>())
{
  if(indices.size() == 0)
  {
    indices.reserve(cloud.size());
    for(unsigned int i = 0; i < cloud.size(); ++i)
    {
      if(std::isnan(cloud.points.at(i).x) ||
         std::isnan(cloud.points.at(i).y) ||
         std::isnan(cloud.points.at(i).z))
      {
        continue;
      }
      else
      {
        indices.push_back(i);
      }
    }
  }

  if(indices.size() == 0)
    return(false);
  
  mean.setZero();
  for (unsigned i=0; i < indices.size(); ++i)
  {
    int idx = indices.at(i);
    mean[0] += cloud.points.at(idx).x;
    mean[1] += cloud.points.at(idx).y;
    mean[2] += cloud.points.at(idx).z;
  }

  mean /= (float)indices.size();

  return(true);
}

template <class T>
bool computeCovarianceMatrix(const typename pcl::PointCloud<T> &cloud, const Eigen::Vector3f &mean,
                             Eigen::Matrix3f &cov, std::vector<int> indices = std::vector<int>())
{
  if(indices.size() == 0)
  {
    indices.reserve(cloud.size());
    for(unsigned int i = 0; i < cloud.size(); ++i)
    {
      if(std::isnan(cloud.points.at(i).x) ||
         std::isnan(cloud.points.at(i).y) ||
         std::isnan(cloud.points.at(i).z))
      {
        continue;
      }
      else
      {
        indices.push_back(i);
      }
    }
  }
  
  bool done = false;
  cov.setZero ();
  
  for (unsigned pi = 0; pi < indices.size (); ++pi)
  {
    float x = cloud.points.at(indices.at(pi)).x - mean[0];
    float y = cloud.points.at(indices.at(pi)).y - mean[1];
    float z = cloud.points.at(indices.at(pi)).z - mean[2];
    
    cov(0,0) += x*x;
    cov(0,1) += x*y;
    cov(0,2) += x*z;
    
    cov(1,0) += y*x;
    cov(1,1) += y*y;
    cov(1,2) += y*z;
    
    cov(2,0) += z*x;
    cov(2,1) += z*y;
    cov(2,2) += z*z;
    
    done = true;
  }
  
  return(done);
}

#endif

} //namespace EPUtils

#endif
