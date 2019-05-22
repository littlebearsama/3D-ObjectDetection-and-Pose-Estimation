/*
 * squirrel_segmentation_incremental.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <squirrel_segmentation/pcl_conversions.h>

#include "squirrel_object_perception_msgs/SegmentInit.h"
#include "squirrel_object_perception_msgs/SegmentOnce.h"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceSegmenter/segmentation.hpp"
#include "v4r/EPUtils/EPUtils.hpp"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifndef SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_
#define SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_

class SegmenterIncremental
{
private:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer SegmentInit_;
  ros::ServiceServer SegmentOnce_;
  ros::NodeHandle *n_;
  boost::shared_ptr<segmentation::Segmenter> segmenter_;
  std::string model_filename_, scaling_filename_;

  bool
  segmentInit (squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response);
  
  bool
  segmentOnce (squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response);
public:
  SegmenterIncremental ();
  ~SegmenterIncremental ();

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_SEGMENTATION_INCREMENTAL_HPP_
