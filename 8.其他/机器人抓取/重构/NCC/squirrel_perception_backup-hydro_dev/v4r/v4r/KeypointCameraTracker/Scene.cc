/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "Scene.hh"

namespace kp
{

/**
 * setKeyframeLast
 */
bool Scene::setKeyframeLast()
{
  if (views.size()==0)
    return false;

  idx_keyframe = views.size()-1;
  views.back()->is_keyframe=true;

  return true;
}


/**
 * setKeyframeLast
 */
bool Scene::setKeyframeLast(const cv::Mat_<unsigned char> &image)
{
  if (views.size()==0)
    return false;

  idx_keyframe = views.size()-1;
  views.back()->is_keyframe=true;
  image.copyTo(views.back()->image);

  return true;
}

/**
 * setKeyframe
 */
bool Scene::setKeyframe(const cv::Mat_<unsigned char> &image, int idx)
{
  if (idx < (int)views.size())
    return false;

  idx_keyframe = idx;
  views[idx]->is_keyframe=true;
  image.copyTo(views[idx]->image);

  return true;
}

/**
 * setKeyframeLast
 */
bool Scene::setKeyframeLast(const DataMatrix2D<PointXYZRGB> &cloud)
{
  if (views.size()==0)
    return false;

  idx_keyframe = views.size()-1;
  views.back()->is_keyframe=true;

  DataMatrix2D<PointXYZRGB>::Ptr tmp_cloud(new DataMatrix2D<PointXYZRGB>);
  *tmp_cloud = cloud;
  views.back()->cloud = tmp_cloud;  

  return true;
}


} //--END--


