/*
 * squirrel_attention_symmetry.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_SYMMETRY_HPP_
#define SQUIRREL_ATTENTION_SYMMETRY_HPP_

#include "squirrel_object_perception_msgs/GetSaliencySymmetry.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class AttentionSymmetryService : public AttentionBaseService
{
private:
  boost::shared_ptr<AttentionModule::SymmetryMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliencySymmetry::Request & req, squirrel_object_perception_msgs::GetSaliencySymmetry::Response & response);
  
public:
  AttentionSymmetryService (): AttentionBaseService() {};
  virtual ~AttentionSymmetryService () {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_SYMMETRY_HPP_