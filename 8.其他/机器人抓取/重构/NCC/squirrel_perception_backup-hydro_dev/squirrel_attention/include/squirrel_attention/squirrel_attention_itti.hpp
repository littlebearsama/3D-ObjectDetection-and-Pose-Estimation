/*
 * squirrel_attention_symmetry.hpp
 *
 *  Created on: Nov 6, 2014
 *      Author: Ekaterina Potapova
 */

#ifndef SQUIRREL_ATTENTION_ITTI_HPP_
#define SQUIRREL_ATTENTION_ITTI_HPP_

#include "squirrel_object_perception_msgs/GetSaliencyItti.h"
#include <squirrel_attention/squirrel_attention_base.hpp>

class AttentionIttiService : public AttentionBaseService
{
private:
  boost::shared_ptr<AttentionModule::IKNSaliencyMap> saliencyMap_;
  
  bool
  calculate (squirrel_object_perception_msgs::GetSaliencyItti::Request & req, squirrel_object_perception_msgs::GetSaliencyItti::Response & response);
  
public:
  AttentionIttiService () : AttentionBaseService() {};
  virtual ~AttentionIttiService () {};

  void
  initialize (int argc, char ** argv);
};

#endif //SQUIRREL_ATTENTION_ITTI_HPP_