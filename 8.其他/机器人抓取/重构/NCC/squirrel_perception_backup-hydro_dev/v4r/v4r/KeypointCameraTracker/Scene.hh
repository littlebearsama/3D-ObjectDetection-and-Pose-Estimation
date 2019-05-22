/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KT_SCENE_HH
#define KT_SCENE_HH


#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <Eigen/Dense>
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "v4r/KeypointTools/DataMatrix2D.hpp"
#include "v4r/KeypointTools/PointTypes.hpp"
#include "View.hh"

namespace kp
{


/**
 * Scene
 */
class Scene
{
public:
  std::vector<View::Ptr> views;
  std::vector<Eigen::Vector3d> points;

  int idx_keyframe;

  Scene() : idx_keyframe(-1) {}
  ~Scene() {}

  bool setKeyframeLast();
  bool setKeyframeLast(const cv::Mat_<unsigned char> &image);
  bool setKeyframeLast(const DataMatrix2D<PointXYZRGB> &cloud);
  bool setKeyframe(const cv::Mat_<unsigned char> &image, int idx);

  inline int getKeyframe() { return idx_keyframe; }
  inline int getFrameLast() { return int(views.size())-1; }

  typedef SmartPtr< ::kp::Scene > Ptr;
  typedef SmartPtr< ::kp::Scene const > ConstPtr;
};

/*************************** INLINE METHODES **************************/


} //--END--

#endif

