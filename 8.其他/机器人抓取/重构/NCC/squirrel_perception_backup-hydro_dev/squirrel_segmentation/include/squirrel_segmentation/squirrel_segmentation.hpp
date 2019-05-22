/*
 * squirrel_segmentation.hpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
 
#include <sstream>

#include "sensor_msgs/PointCloud2.h"
#include <squirrel_segmentation/pcl_conversions.h>

#include "squirrel_object_perception_msgs/Segment.h"

#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/SurfaceSegmenter/segmentation.hpp"
#include "v4r/EPUtils/EPUtils.hpp"
#include "v4r/AttentionModule/AttentionModule.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#ifndef SQUIRREL_SEGMENTATION_HPP_
#define SQUIRREL_SEGMENTATION_HPP_

class SegmenterComplete
{
private:
  typedef pcl::PointXYZRGB PointT;
  ros::ServiceServer Segment_;
  ros::NodeHandle *n_;
  boost::shared_ptr<segmentation::Segmenter> segmenter_;
  std::string model_filename_, scaling_filename_;

  bool
  segment (squirrel_object_perception_msgs::Segment::Request & req, squirrel_object_perception_msgs::Segment::Response & response);
  
public:
  SegmenterComplete ();
  ~SegmenterComplete ();

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_SEGMENTATION_HPP_
