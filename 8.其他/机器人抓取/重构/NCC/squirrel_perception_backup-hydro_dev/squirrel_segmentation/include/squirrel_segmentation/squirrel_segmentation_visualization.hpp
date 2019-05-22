/*
 * squirrel_segmentation_visualization.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <squirrel_segmentation/pcl_conversions.h>

#include "squirrel_object_perception_msgs/SegmentVisualizationInit.h"
#include "squirrel_object_perception_msgs/SegmentVisualizationOnce.h"

#include "v4r/EPUtils/EPUtils.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifndef SQUIRREL_SEGMENTATION_VISUALIZATION_HPP_
#define SQUIRREL_SEGMENTATION_VISUALIZATION_HPP_

class SegmenterVisualization
{
private:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer SegmentVisualizationInit_;
  ros::ServiceServer SegmentVisualizationOnce_;
  ros::Publisher SaliencyPub_;
  ros::Publisher SegmentationPub_;
  ros::NodeHandle *n_;
  cv::Mat RGB;
  cv::Mat salMap;

  bool
  segmentVisualizationInit (squirrel_object_perception_msgs::SegmentVisualizationInit::Request & req, squirrel_object_perception_msgs::SegmentVisualizationInit::Response & response);
  
  bool
  segmentVisualizationOnce (squirrel_object_perception_msgs::SegmentVisualizationOnce::Request & req, squirrel_object_perception_msgs::SegmentVisualizationOnce::Response & response);
public:
  SegmenterVisualization ();
  ~SegmenterVisualization ();

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_SEGMENTATION_VISUALIZATION_HPP_
