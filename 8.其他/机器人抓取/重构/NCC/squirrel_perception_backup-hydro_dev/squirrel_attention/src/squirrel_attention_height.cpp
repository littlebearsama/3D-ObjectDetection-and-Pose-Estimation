/*
 * squirrel_attention_height.cpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_attention/squirrel_attention_height.hpp>

bool 
AttentionHeightService::calculate (squirrel_object_perception_msgs::GetSaliencyHeight::Request & req, squirrel_object_perception_msgs::GetSaliencyHeight::Response & response)
{
  pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);
  int type_ = req.height_type.data;
  
  //prepare point cloud
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::PointIndices::Ptr scene_indices(new pcl::PointIndices());
  preparePointCloud(scene,coefficients,normals,scene_indices);
  
  // start creating parameters
  saliencyMap_.reset(new AttentionModule::SurfaceHeightSaliencyMap);
  saliencyMap_->setWidth(scene->width);
  saliencyMap_->setHeight(scene->height);
  saliencyMap_->setCloud(scene);
  saliencyMap_->setIndices(scene_indices);
  saliencyMap_->setNormals(normals);
  saliencyMap_->setModelCoefficients(coefficients);
  saliencyMap_->setHeightType(type_);
  saliencyMap_->setCombinationType(AttentionModule::AM_COMB_SUM);
  saliencyMap_->setNormalizationType(EPUtils::NT_NONE);
  saliencyMap_->calculate();
  
  cv::Mat map;
  if(!(saliencyMap_->getMap(map)))
  {
    return(false);
  }

//   cv::imshow("map",map);
//   cv::waitKey(-1);
  
  resize(map,map,cv::Size(scene->width,scene->height));
  map.convertTo(map,CV_8U,255);

  cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
  ros::Time time = ros::Time::now();
  // convert OpenCV image to ROS message
  cv_ptr->header.stamp = time;
  cv_ptr->header.frame_id = "image";
  cv_ptr->encoding = "mono8";
  cv_ptr->image = map;
    
  //sensor_msgs::Image im;
  cv_ptr->toImageMsg(response.saliency_map);
  return(true);
}

void
AttentionHeightService::initialize (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_attention_height_server");
  n_ = new ros::NodeHandle ();
    
  attention_ = n_->advertiseService ("/squirrel_attention_height", &AttentionHeightService::calculate, this);
  ROS_INFO("Ready to get service calls...");
  ros::spin ();
}

int
main (int argc, char ** argv)
{
  AttentionHeightService m;
  m.initialize (argc, argv);

  return 0;
}