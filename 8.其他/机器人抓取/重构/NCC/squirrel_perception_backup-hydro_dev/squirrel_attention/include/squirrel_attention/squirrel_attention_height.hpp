/*
 * squirrel_attention_height.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_HEIGHT_HPP_
#define SQUIRREL_ATTENTION_HEIGHT_HPP_

#include "squirrel_object_perception_msgs/GetSaliencyHeight.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class AttentionHeightService : public AttentionBaseService
{
private:
  int type_;
  boost::shared_ptr<AttentionModule::SurfaceHeightSaliencyMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliencyHeight::Request & req, squirrel_object_perception_msgs::GetSaliencyHeight::Response & response);
  
public:
  AttentionHeightService () : AttentionBaseService(), type_(0) {};
  virtual ~AttentionHeightService () {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_HEIGHT_HPP_