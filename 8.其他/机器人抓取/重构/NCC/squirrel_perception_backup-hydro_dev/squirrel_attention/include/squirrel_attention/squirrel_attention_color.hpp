/*
 * squirrel_attention_color.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_COLOR_HPP_
#define SQUIRREL_ATTENTION_COLOR_HPP_

#include "squirrel_object_perception_msgs/GetSaliencyColor.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class AttentionColorService : public AttentionBaseService
{
protected:
  cv::Scalar color_;
  boost::shared_ptr<AttentionModule::ColorSaliencyMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliencyColor::Request & req, squirrel_object_perception_msgs::GetSaliencyColor::Response & response);
  
public:
  AttentionColorService (): AttentionBaseService(), color_(cv::Scalar(0,255,0)) {};
  virtual ~AttentionColorService () {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_COLOR_HPP_