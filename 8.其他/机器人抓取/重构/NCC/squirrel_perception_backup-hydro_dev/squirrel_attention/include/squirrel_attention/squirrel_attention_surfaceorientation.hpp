/*
 * squirrel_attention_surfaceorientation.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_SURFACEORIENTATION_HPP_
#define SQUIRREL_ATTENTION_SURFACEORIENTATION_HPP_

#include "squirrel_object_perception_msgs/GetSaliencySurfaceOrientation.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class AttentionSurfaceOrientationService : public AttentionBaseService
{
private:
  int type_;
  boost::shared_ptr<AttentionModule::RelativeSurfaceOrientationMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliencySurfaceOrientation::Request & req, squirrel_object_perception_msgs::GetSaliencySurfaceOrientation::Response & response);
  
public:
  AttentionSurfaceOrientationService () : AttentionBaseService(), type_(1) {};
  virtual ~AttentionSurfaceOrientationService () {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_SURFACEORIENTATION_HPP_