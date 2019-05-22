/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_KEYPOINT_TRACKER_HH
#define KP_KEYPOINT_TRACKER_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include <Eigen/Dense>

#include "v4r/KeypointBase/FeatureDetector.hh"
#include "v4r/KeypointBase/FeatureDetector_KD_FAST_IMGD.hh"
#include "Scene.hh"
#include "View.hh"
//#include "v4r/KeypointTools/DataMatrix2D.hpp"
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "v4r/KeypointTools/ImageTransformRANSAC.hh"

namespace kp
{

class KeypointTracker 
{
public:
  class Parameter
  {
  public:
    int tiles;
    int max_features_tile;
    float inl_dist;
    float pecnt_prefilter;
    bool refineLK;          // refine keypoint location (LK)
    bool refineMappedLK;    // refine keypoint location (map image, then LK)
    int min_total_matches;        // minimum total number of mathches
    int min_tiles_used;          // minimum number of tiles with matches
    bool affine_outl_rejection;
    int fast_pyr_levels;
    Parameter(int _tiles=3, int _max_features_tile=100, float _inl_dist=7., float _pecnt_prefilter=0.02,
        bool _refineLK=true, bool _refineMappedLK=false, 
        int _min_total_matches=10, int _min_tiles_used=3, 
        bool _affine_outl_rejection=true, int _fast_pyr_levels=2)
      : tiles(_tiles), max_features_tile(_max_features_tile), inl_dist(_inl_dist), pecnt_prefilter(_pecnt_prefilter),
        refineLK(_refineLK), refineMappedLK(_refineMappedLK),
        min_total_matches(_min_total_matches), min_tiles_used(_min_tiles_used), 
        affine_outl_rejection(_affine_outl_rejection), fast_pyr_levels(_fast_pyr_levels)
    {
    }
  };

private:
  Parameter param;

  int tile_size_w, tile_size_h;
  const static int PATCH_SIZE = 15;
  int status; // 0 no keypoints, 1 keypoints detected, 2 keypoints tracked

  std::vector<cv::KeyPoint> keys;
  cv::Mat descs;

   cv::Mat_<unsigned char> im_tmp;

  Scene::Ptr scene;
  int idx_keyframe;
  std::vector<cv::DMatch> matches;

  FeatureDetector::Ptr detector;
  cv::Ptr<cv::BFMatcher> matcher;

  void updateKeypointCoordinates(const cv::Rect &rect,std::vector<cv::KeyPoint> &_keys);
  void setKeypointLinks(View &view, View &keyframe, std::vector<cv::DMatch> &matches);
  int filterKeypointLinks(View &view, View &keyframe);
  int filterKeypointLinks2(View &view, View &keyframe);
  void predictKeypoints(View &view, View &keyframe);
  void preFilterMatches(View &view, View &keyframe, std::vector<cv::DMatch> &matches);
  void refineKeypointLocationLK(const cv::Mat_<unsigned char> &image, View &view, View &keyframe);
  void refineKeypointLocationMappedLK(const cv::Mat_<unsigned char> &image, View &view, View &keyframe);
  bool isTrackOK(const View &view);
  void selectKeypoints(std::vector<cv::KeyPoint> &keys, cv::Mat &descs, const cv::Mat_<unsigned char> &mask);


  inline void transformPoint(const cv::Point2f &pt1, cv::Point2f &pt2, const cv::Mat_<float> &transform);
  inline  void getExpandedRect(int u, int v, int rows, int cols, cv::Rect &rect);


public:
  cv::Mat dbg;

  KeypointTracker(const Parameter &p=Parameter());
  ~KeypointTracker();

  void track(const cv::Mat_<unsigned char> &image, const cv::Mat_<unsigned char> &mask=cv::Mat());
  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene) {scene = _scene;}

  typedef SmartPtr< ::kp::KeypointTracker> Ptr;
  typedef SmartPtr< ::kp::KeypointTracker const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline void KeypointTracker::transformPoint(const cv::Point2f &pt1, cv::Point2f &pt2, const cv::Mat_<float> &transform)
{
  pt2.x = transform(0,0)*pt1.x + transform(0,1)*pt1.y + transform(0,2);
  pt2.y = transform(1,0)*pt1.x + transform(1,1)*pt1.y + transform(1,2);
  float t = transform(2,0)*pt1.x + transform(2,1)*pt1.y + transform(2,2);
  pt2.x /= t;
  pt2.y /= t;
}

/**
 * getExpandedRect
 */
inline void KeypointTracker::getExpandedRect(int u, int v, int rows, int cols, cv::Rect &rect)
{
  int border = PATCH_SIZE;

  int x1 = u*tile_size_w;
  int y1 = v*tile_size_h;
  int x2 = x1 + tile_size_w;
  int y2 = y1 + tile_size_h;

  x1 -= border;
  y1 -= border;
  x2 += border;
  y2 += border;

  if (x1<0) x1 = 0;
  if (y1<0) y1 = 0;
  if (x2>=cols) x2 = cols-1;
  if (y2>=rows) y2 = rows-1;

  rect = cv::Rect(x1, y1, x2-x1, y2-y1);  
}


} //--END--

#endif

