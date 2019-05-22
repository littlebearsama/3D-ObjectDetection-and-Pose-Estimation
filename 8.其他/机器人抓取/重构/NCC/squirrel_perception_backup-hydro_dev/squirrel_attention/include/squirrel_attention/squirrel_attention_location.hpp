/*
 * squirrel_attention_location.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_LOCATION_HPP_
#define SQUIRREL_ATTENTION_LOCATION_HPP_

#include "squirrel_object_perception_msgs/GetSaliencyLocation.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class AttentionLocationService: public AttentionBaseService
{
protected:
  int location_;
  cv::Point center_point_;
  boost::shared_ptr<AttentionModule::LocationSaliencyMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliencyLocation::Request & req, squirrel_object_perception_msgs::GetSaliencyLocation::Response & response);
  
public:
  AttentionLocationService(): AttentionBaseService(),location_(AttentionModule::AM_CENTER),center_point_(cv::Point(0,0)) {};
  virtual ~AttentionLocationService() {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_LOCATION_HPP_