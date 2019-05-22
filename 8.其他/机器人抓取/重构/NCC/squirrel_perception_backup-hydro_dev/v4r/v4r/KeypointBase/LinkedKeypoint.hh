/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KT_LINKED_KEYPOINT_HH
#define KT_LINKED_KEYPOINT_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Dense>


namespace kp
{

class LinkedKeypoint : public cv::KeyPoint
{
public:
  static const float NaN;

  int tile_label;
  std::vector<std::pair<int,int> > links; // links to <frame, keypoint>
  float dist_tmp;           // descriptor distances (temporary valid)

  int nb;
  static int nb_cnt;

  cv::Point2f pt_pred;      // predicted point location in current tracking frame
  bool have_pred;

  Eigen::Vector3f pt3;      // corresponding 3d point (local coordinates)
  Eigen::Vector3f normal;
  int pt3_glob;             // index to global 3d point

  LinkedKeypoint();
  LinkedKeypoint(const cv::KeyPoint &_key, int _tile_label);
  ~LinkedKeypoint();

};


/*************************** INLINE METHODES **************************/



} //--END--

#endif

