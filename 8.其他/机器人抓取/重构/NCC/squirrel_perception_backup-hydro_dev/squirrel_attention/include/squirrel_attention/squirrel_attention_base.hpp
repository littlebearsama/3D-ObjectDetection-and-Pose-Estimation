/*
 * squirrel_attention_base.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_BASE_HP_
#define SQUIRREL_ATTENTION_BASE_HP_

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <squirrel_attention/pcl_conversions.h>
#include "v4r/AttentionModule/AttentionModule.hpp"
#include "v4r/EPUtils/EPUtils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class AttentionBaseService
{
protected:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer attention_;
  ros::NodeHandle *n_;
  
  void getImageFromPointCloud(pcl::PointCloud<PointT>::Ptr scene, cv::Mat &RGB);
  int 
  preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, 
                    pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr objects_indices);
  
public:
  AttentionBaseService () {};
  virtual ~AttentionBaseService () { 
    if(n_)
      delete n_; 
  };
};

void AttentionBaseService::getImageFromPointCloud(pcl::PointCloud<PointT>::Ptr scene, cv::Mat &RGB)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  EPUtils::pointCloudXYZRGB_2_cloudXYZimageRGB(scene,scene_xyz,RGB,scene->width,scene->height);
}
int 
AttentionBaseService::preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, 
		                              pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr objects_indices)
{
  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
  {
    return(pclAddOns::FILTER);
  }
  
  // segment plane
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
  if(!pclAddOns::SegmentPlane<pcl::PointXYZRGB>(cloud,indices,plane_indices,objects_indices,coefficients))
  {
    return(pclAddOns::SEGMENT);
  }
  
  //calculate point cloud normals
  if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,objects_indices,normals))
  {
    return(pclAddOns::NORMALS);
  }
  
  return(0);
}

#endif //SQUIRREL_ATTENTION_BASE_HP_