/*
 * squirrel_attention_3Dsymmetry.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_3DSYMMETRY_HPP_
#define SQUIRREL_ATTENTION_3DSYMMETRY_HPP_

#include "squirrel_object_perception_msgs/GetSaliency3DSymmetry.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class Attention3DSymmetryService : public AttentionBaseService
{
private:
  boost::shared_ptr<AttentionModule::Symmetry3DMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliency3DSymmetry::Request & req, squirrel_object_perception_msgs::GetSaliency3DSymmetry::Response & response);
  
public:
  Attention3DSymmetryService () : AttentionBaseService() {};
  virtual ~Attention3DSymmetryService () {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_3DSYMMETRY_HPP_