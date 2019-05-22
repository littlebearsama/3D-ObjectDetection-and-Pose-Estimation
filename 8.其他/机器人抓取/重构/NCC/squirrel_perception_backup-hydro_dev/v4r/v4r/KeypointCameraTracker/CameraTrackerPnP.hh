/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CAMERA_TRACKER_HH
#define KP_CAMERA_TRACKER_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Dense>
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "View.hh"
#include "Scene.hh"

namespace kp
{

class CameraTrackerPnP 
{
public:
  class Parameter
  {
  public:
    int iterationsCount;       // 1000
    float reprojectionError;   // 3
    int minInliersCount;       // 100
    double min_total_score;         // minimum score (matches weighted with inv. inl. dist)
    int min_tiles_used;          // minimum number of tiles with matches
    Parameter(int _iterationsCount=5000, float _reprojectionError=3,
      int _minInliersCount=50, double _min_total_score=10, int _min_tiles_used=3) 
    : iterationsCount(_iterationsCount), reprojectionError(_reprojectionError),
      minInliersCount(_minInliersCount), min_total_score(_min_total_score), min_tiles_used(_min_tiles_used){}
  };

private:
  Parameter param;
  
  Scene::Ptr scene;

  cv::Mat_<cv::Vec3b> image;
  cv::Mat_<unsigned char> im_gray;

  cv::Mat_<double> dist_coeffs;
  cv::Mat_<double> intrinsic;

  std::vector<cv::Point3f> model_pts;
  std::vector<cv::Point2f> query_pts;

  void trackPose(View &keyframe, View &view);
  void getPoints(View &keyframe, View &view, std::vector<cv::Point3f> &model_pts, std::vector<cv::Point2f> &query_pts);
  void removeOutlierLinks(View &keyframe, View &view, double &score, 
        std::vector<double> &score_per_tile);
  bool isTrackOK(int num_tiles, const double &total_score, const std::vector<double> &score_per_tile);




public:
  cv::Mat dbg;

  CameraTrackerPnP(const Parameter &p=Parameter());
  ~CameraTrackerPnP();

  bool track();

  Scene::Ptr &getSharedData() {return scene;}
  void setSharedData(Scene::Ptr &_scene) {scene = _scene;}

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr< ::kp::CameraTrackerPnP> Ptr;
  typedef SmartPtr< ::kp::CameraTrackerPnP const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

