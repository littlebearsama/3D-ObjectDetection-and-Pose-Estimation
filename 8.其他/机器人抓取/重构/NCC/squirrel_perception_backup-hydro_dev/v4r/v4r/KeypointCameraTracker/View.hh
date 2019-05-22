/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_VIEW_HH
#define KP_VIEW_HH


#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <Eigen/Dense>
#include <stdexcept>
#include "v4r/KeypointBase/LinkedKeypoint.hh"
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "v4r/KeypointTools/DataMatrix2D.hpp"

namespace kp
{

/**
 * AR Marker
 */
class Marker
{
public:
  int id;
  std::vector< cv::Point2f > pts;       // image points
  std::vector< int> glob_idx;           // idx to global 3d points

  Marker(int _id) : id(_id) {
    pts.resize(4);
    glob_idx.resize(4,-1);
  }
};

/**
 * View
 */
class View
{
public:
  int idx;
  bool is_keyframe;
  int num_tiles;
  int local_idx;

  std::vector<LinkedKeypoint> keys;   // keypoints
  cv::Mat descs;

  std::vector< Marker > marker;

  Eigen::Vector3f center;      // in global coordinates
  Eigen::Matrix4f pose;

  bool tracked;

  std::vector<int> links;

  cv::Mat image;
  DataContainer::Ptr cloud;

  static const float NaN;

  View(int _idx=-1, int _num_tiles=1, bool _is_keyframe=false);
  ~View();

  static void insert(View &view, const std::vector<cv::KeyPoint> &_keys, const cv::Mat &_descs, int tile_label);

  typedef SmartPtr< ::kp::View > Ptr;
  typedef SmartPtr< ::kp::View const > ConstPtr;
};

typedef SmartPtr< std::vector<View::Ptr> > ViewsPtr;


/*************************** INLINE METHODES **************************/



} //--END--

#endif

