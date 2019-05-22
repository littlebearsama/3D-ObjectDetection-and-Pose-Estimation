/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "View.hh"




namespace kp 
{


using namespace std;

const float View::NaN = std::numeric_limits<float>::quiet_NaN();


/************************************************************************************
 * Constructor/Destructor
 */
View::View(int _idx, int _num_tiles, bool _is_keyframe) 
  : idx(_idx), num_tiles(_num_tiles), is_keyframe(_is_keyframe), center(Eigen::Vector3f(NaN,NaN,NaN)),
    pose(Eigen::Matrix4f::Identity()), tracked(false)
{
}

View::~View()
{
}

/***************************************************************************************/


/**
 * insert
 */
void View::insert(View &view, const std::vector<cv::KeyPoint> &_keys, const cv::Mat &_descs, int tile_label)
{
  if (view.descs.cols!=0 && view.descs.cols!=_descs.cols)
    throw std::runtime_error("[View::insert] Invalid descriptor size!");

  view.keys.reserve(view.keys.size()+_keys.size());

  for (unsigned i=0; i<_keys.size(); i++)
    view.keys.push_back(LinkedKeypoint(_keys[i], tile_label));

  view.descs.push_back(_descs);
}


}












