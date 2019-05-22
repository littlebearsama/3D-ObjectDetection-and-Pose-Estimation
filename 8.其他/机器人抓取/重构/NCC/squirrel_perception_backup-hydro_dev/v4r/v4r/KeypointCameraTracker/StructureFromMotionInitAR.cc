/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "StructureFromMotionInitAR.hh"
#include "v4r/KeypointTools/invPose.hpp"


namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
StructureFromMotionInitAR::StructureFromMotionInitAR(const Parameter &p)
 : param(p)
{ 
  keytracker.reset( new KeypointTracker(param.kt_param) );
  camtracker.reset( new CameraTrackerPnP(param.ct_param) );
  #ifndef KP_NO_ARMARKER
  ardetector.reset( new ARMarkerDetector(param.ar_pattern, param.ar_param) );
  #else
  throw std::runtime_error("[StructureFromMotionInitAR] ARMarker detection is not available!");
  #endif
  tri.reset( new Triangulation(param.tr_param) );
  #ifndef KP_NO_CERES_AVAILABLE
  bundler.reset( new ProjBundleAdjuster(param.ba_param) );
  full_bundler.reset( new ProjBundleAdjuster(
      ProjBundleAdjuster::Parameter(INT_MAX, param.full_ba_optimize_cam) ) );
  #else
  cout<<"[StructureFromMotionInitAR] ATTENTION: No bundle adjustment! Ceres is not installed!"<<endl;
  #endif

  scene = keytracker->getSharedData();

  camtracker->setSharedData(scene);
  #ifndef KP_NO_ARMARKER
  ardetector->setSharedData(scene);
  #endif
  tri->setSharedData(scene);
  #ifndef KP_NO_CERES_AVAILABLE
  bundler->setSharedData(scene);
  full_bundler->setSharedData(scene);
  #endif
}

StructureFromMotionInitAR::~StructureFromMotionInitAR()
{
}


/**
 * setKeyframe
 */
void StructureFromMotionInitAR::setKeyframe(const cv::Mat_<unsigned char> &im, Scene &scene)
{
  if (scene.views.size() > 0 && scene.views.back()->pose != Eigen::Matrix4f::Identity())
  {
    scene.setKeyframeLast(im);
  }
}

/**
 * getDeltaViewingAngle
 */
double StructureFromMotionInitAR::getDeltaViewingAngle(Scene &scene, View &keyframe, Eigen::Matrix4f &pose1, Eigen::Matrix4f &pose2)
{
  unsigned cnt=0;
  Eigen::Vector3f v1, v2;
  Eigen::Vector3d center(0.,0.,0.);
  std::vector<Eigen::Vector3d> &pts = scene.points;
  Eigen::Matrix4f inv_pose1, inv_pose2;
  invPose(pose1,inv_pose1);
  invPose(pose2,inv_pose2);

  for (unsigned i=0; i<keyframe.keys.size(); i++)
  {
    LinkedKeypoint &key = keyframe.keys[i];
    if ( key.pt3_glob >=0 )
    {
      center += pts[key.pt3_glob];
      cnt++;
    }
  }
  for (unsigned i=0; i<keyframe.marker.size(); i++)
  {
    Marker &m = keyframe.marker[i];
    for (unsigned j=0; j<m.glob_idx.size(); j++)
    {
      if (m.glob_idx[j]>=0)
      {
        center += pts[m.glob_idx[j]];
        cnt++;
      }
    }
  }

  if (cnt<=0) return 0.;

  center /= double(cnt);

  Eigen::Vector3f cenf(center[0],center[1],center[2]);

  v1 = (inv_pose1.block<3,1>(0,3)-cenf).normalized();
  v2 = (inv_pose2.block<3,1>(0,3)-cenf).normalized();

  return acos(v1.dot(v2));
}

/**
 * cleanUpDate
 */
void StructureFromMotionInitAR::cleanUpDate(Scene &scene)
{
  int cnt_kf=0;

  for (int i=scene.views.size()-1; i>=0; i--)
  {
    View &view = *scene.views[i];

    if (view.is_keyframe)
    {
      cnt_kf++;
    }
    else
    {
      if (cnt_kf>=3)
      {
        if (view.keys.size()>0)
        {
          view.keys = std::vector<LinkedKeypoint>();
          view.descs = cv::Mat();
          view.marker = std::vector< Marker >();
          view.links = std::vector<int>();
        }
        else break;
      }
    }
  }
}

/**
 * cleanUpImages
 */
void StructureFromMotionInitAR::cleanUpImages(Scene &scene)
{
  int cnt_kf=0;

  for (int i=scene.views.size()-1; i>=0; i--)
  {
    View &view = *scene.views[i];

    if (view.is_keyframe)
    {
      if (cnt_kf>=3)
      {
        if (!view.image.empty())
        {
          view.image.release();
        }
        else break;
      }

      cnt_kf++;
    }
  }
}





/***************************************************************************************/


/**
 * track
 */
bool StructureFromMotionInitAR::track(const cv::Mat &_image, Eigen::Matrix4f &pose, const cv::Mat &mask)
{
  if (intrinsic.empty()) 
    throw std::runtime_error("[StructureFromMotionInitAR::track] Camera parameter not set!");

  bool ok = false;
  int idx_keyframe = -1;

  if( _image.type() == CV_8U ) {
    cv::merge(std::vector<cv::Mat>(3,_image), image);
    im_gray = _image;
  } else {
    image = _image;
    cv::cvtColor( _image, im_gray, CV_RGB2GRAY );
  }

  // tracking of the current keyframe
  keytracker->track(im_gray, mask);
  #ifndef KP_NO_ARMARKER
  ardetector->detect(image);
  #endif
  ok = camtracker->track();
  #ifndef KP_NO_ARMARKER
  ardetector->reconstruct();
  #endif

  // create a new keyframe?
  idx_keyframe = scene->getKeyframe();

  if (idx_keyframe < 0)
  {
    setKeyframe(im_gray, *scene);
  }
  else
  {
    int idx_frame = scene->getFrameLast();
    double angle = getDeltaViewingAngle( *scene, *scene->views[idx_keyframe],
                      scene->views[idx_keyframe]->pose, scene->views[idx_frame]->pose );

    if ( ok && idx_frame != idx_keyframe && angle > param.delta_angle_keyframe/180.*M_PI)
    {
      #ifndef KP_NO_CERES_AVAILABLE
      bundler->optimize();
      #endif
      int num_pts = tri->triangulate();

      if (num_pts > param.min_points_keyframe)
      {
        setKeyframe(im_gray, *scene);
      }
    }
  }

  // clean up and return pose
  if (param.clean_up_data) {
    cleanUpDate(*scene);
  }
  if (param.clean_up_images) {
    cleanUpImages(*scene);
  }
  if (scene->views.size()>0) {
    pose = scene->views.back()->pose;
  }

  return ok;
}

/**
 * doFullBundleAdjustment
 */
void StructureFromMotionInitAR::doFullBundleAdjustment()
{
  #ifndef KP_NO_CERES_AVAILABLE
  full_bundler->optimize(); 
  #else
  throw std::runtime_error("[StructureFromMotionInitAR::doFullBundleAdjustment] Ceres is not installed");
  #endif
}

/**
 * setSharedData
 */
void StructureFromMotionInitAR::setSharedData(Scene::Ptr &_scene) 
{
  scene = _scene;

  keytracker->setSharedData(scene);
  camtracker->setSharedData(scene);
  #ifndef KP_NO_ARMARKER
  ardetector->setSharedData(scene);
  #endif
  tri->setSharedData(scene);
  #ifndef KP_NO_CERES_AVAILABLE
  bundler->setSharedData(scene);
  full_bundler->setSharedData(scene);
  #endif
}

/**
 * setCameraParameter
 */
void StructureFromMotionInitAR::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows; i++)
      dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }

  camtracker->setCameraParameter(intrinsic, dist_coeffs);
  #ifndef KP_NO_ARMARKER
  ardetector->setCameraParameter(intrinsic, dist_coeffs);
  #endif
  tri->setCameraParameter(intrinsic, dist_coeffs);
  #ifndef KP_NO_CERES_AVAILABLE
  bundler->setCameraParameter(intrinsic, dist_coeffs);
  full_bundler->setCameraParameter(intrinsic, dist_coeffs);
  #endif
}

/**
 * setDebugImage
 */
void StructureFromMotionInitAR::setDebugImage(const cv::Mat _image)
{
  im_dbg = _image;

  keytracker->dbg = im_dbg;
  camtracker->dbg = im_dbg;
  #ifndef KP_NO_ARMARKER
  ardetector->dbg = im_dbg;
  #endif
  #ifndef KP_NO_CERES_AVAILABLE
  bundler->dbg = im_dbg;
  #endif
}

} // -- the end --












