/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "CameraTrackerRGBD.hh"
#include "v4r/KeypointTools/convertImage.hpp"
#include "v4r/KeypointTools/invPose.hpp"




namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
CameraTrackerRGBD::CameraTrackerRGBD(const Parameter &p)
 : param(p)
{ 
  thr_image_motion_px = -1; 

  keytracker.reset(new KeypointTracker(param.kt_param));
  rt.reset(new RigidTransformationRANSAC(param.rt_param));

  scene = keytracker->getSharedData();

  if (param.detect_loops) {
    loopclosing.reset(new LoopClosingRT(param.lc_param) );
    loopclosing->setSharedData(scene);
  }


  if (param.kt_param.tiles*param.kt_param.tiles < param.min_tiles_used)
    param.min_tiles_used = 1;
}

CameraTrackerRGBD::~CameraTrackerRGBD()
{
}

/**
 * getPoints
 */
void CameraTrackerRGBD::getPoints(View &keyframe, View &view, std::vector<Eigen::Vector3f> &src_pts, std::vector<Eigen::Vector3f> &tgt_pts)
{
  src_pts.clear();
  tgt_pts.clear();
  src_pts.reserve(view.keys.size());
  tgt_pts.reserve(view.keys.size());

  Eigen::Matrix4f inv_pose;
  invPose(keyframe.pose, inv_pose);

  Eigen::Matrix3f R = inv_pose.topLeftCorner<3,3>();
  Eigen::Vector3f t = inv_pose.block<3,1>(0,3);

  for (unsigned i=0; i<view.keys.size(); i++) 
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      LinkedKeypoint &kf_key = keyframe.keys[key.links[0].second];
      if (!isnan(kf_key.pt3[0])) 
      {
        tgt_pts.push_back(key.pt3);
        src_pts.push_back( R*kf_key.pt3+t );
      }
    }
  }
}

/**
 * trackPose
 */
void CameraTrackerRGBD::trackPose(View &keyframe, View &view)
{
  vector<int> inliers;

  getPoints(keyframe, view, src_pts, tgt_pts);  

  if (src_pts.size() > 3)
  {
    rt->compute(src_pts, tgt_pts, view.pose, inliers);
  }
}

/**
 * removeOutlierLinks
 */
void CameraTrackerRGBD::removeOutlierLinks(View &keyframe, View &view, double &score, std::vector<double> &score_per_tile, double &dist_px)
{
  int cnt=0;
  double dist, tmp;
  float sqr_inl_dist = param.rt_param.inl_dist*param.rt_param.inl_dist;
  Eigen::Matrix4f inv_pose, delta_pose;
  Eigen::Vector3f pt3;
  
  invPose(keyframe.pose, inv_pose);
  delta_pose = view.pose*inv_pose;

  Eigen::Matrix3f R = delta_pose.topLeftCorner<3,3>();
  Eigen::Vector3f t = delta_pose.block<3,1>(0,3);

  score = 0.; 
  score_per_tile.clear();
  score_per_tile.resize(view.num_tiles*view.num_tiles,0.); 
  dist_px = 0.;

  for (unsigned i=0; i<view.keys.size(); i++) 
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      LinkedKeypoint &kf_key = keyframe.keys[key.links[0].second];
      if (!isnan(kf_key.pt3[0])) 
      {      
        pt3 = R*kf_key.pt3 + t;
        
        dist = (pt3-key.pt3).squaredNorm();

        if (dist < sqr_inl_dist)
        {
          cnt++;
          tmp = sqr_inl_dist - dist;
          score += tmp;
          score_per_tile[key.tile_label] += tmp;
          dist_px += (Eigen::Map<Eigen::Vector2f>(&key.pt.x)-Eigen::Map<Eigen::Vector2f>(&kf_key.pt.x)).norm();
        }
        else key.links.clear();
      }
      else key.links.clear();
    }
  }

  score /= sqr_inl_dist;
  for (unsigned i=0; i<score_per_tile.size(); i++)
    score_per_tile[i] /= sqr_inl_dist;
  
  if (cnt>0) dist_px/=double(cnt);
}

/**
 * getDeltaViewingAngle
 */
double CameraTrackerRGBD::getDeltaViewingAngle(Eigen::Matrix4f &pose1, Eigen::Matrix4f &pose2)
{
  Eigen::Vector3f v0(0,0,1), v1, v2;

  v1 = pose1.topLeftCorner<3,3>()*v0;
  v2 = pose2.topLeftCorner<3,3>()*v0;

  double cosa = v1.dot(v2);
  
  if (cosa > 0.9999999)
    return 0.;
  return acos(cosa);
}

/**
 * isTrackOK
 */
bool CameraTrackerRGBD::isTrackOK(int num_tiles, const double &total_score, const std::vector<double> &score_per_tile)
{
  int sum=0, z=0;
  std::vector<int> score_rows(num_tiles,0);
  std::vector<int> score_cols(num_tiles,0);

  for (unsigned i=0; i<score_per_tile.size(); i++)
  {
    if (score_per_tile[i]>=1.)
    {
      z++;
      sum += score_per_tile[i];
      score_rows[i/num_tiles]++;
      score_cols[i%num_tiles]++;
    }
  }

  unsigned zv=0, zu=0;
  for (unsigned i=0; i<score_rows.size(); i++)
    if (score_rows[i] > 0) zv++;
  for (unsigned i=0; i<score_cols.size(); i++)
    if (score_cols[i] > 0) zu++;

  if (z>=param.min_tiles_used && sum > param.min_total_score)
    if (param.min_tiles_used < 3 || (zv>=2 && zu>=2))
      return true;
  return false;
}

/**
 * lookupPoints3D
 */
void CameraTrackerRGBD::lookupPoints3D(const DataMatrix2D<PointXYZRGB> &cloud, View &view)
{
  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    int x = int(key.pt.x+.5);
    int y = int(key.pt.y+.5);

    if (x>=0 && y>=0 && x<(int)cloud.cols && y<(int)cloud.rows)
    {
      const PointXYZRGB &pt = cloud(y,x);

      if (!isnan(pt[0]))
      {
        key.pt3 = pt.pt.segment<3>(0);
      }
    }
  }
}

/**
 * setKeyframe
 */
void CameraTrackerRGBD::setKeyframe(const DataMatrix2D<PointXYZRGB> &cloud, Scene &scene)
{
  if (scene.views.size() > 0)// && scene.views.back()->pose != Eigen::Matrix4f::Identity())
  {
    scene.setKeyframeLast(im_gray);
    if (param.log_clouds)
      scene.setKeyframeLast(cloud);
  }
}



/***************************************************************************************/

bool CameraTrackerRGBD::track(const DataMatrix2D<PointXYZRGB> &cloud, Eigen::Matrix4f &pose, const cv::Mat_<unsigned char> &mask)
{
  if (cloud.rows<=1)
    throw std::runtime_error("[CameraTrackerRGBD::track] Need an organized point cloud!");

  if (!dbg.empty()){ 
    keytracker->dbg = dbg;
    if (loopclosing.get()!=0) loopclosing->dbg = dbg;
  }


  bool ok = false;
  int idx_keyframe = -1;
  double score, dist_px;
  std::vector<double> score_per_tile; 

  convertImage(cloud, image); 
  cv::cvtColor( image, im_gray, CV_RGB2GRAY );

  if (thr_image_motion_px < 0) thr_image_motion_px = param.thr_image_motion*double(image.cols);

  // tracking of the current keyframe
  keytracker->track(im_gray, mask);
  lookupPoints3D(cloud, *scene->views.back());

  idx_keyframe = scene->getKeyframe();

  if (idx_keyframe < 0)
  {
    setKeyframe(cloud, *scene);
    ok=true;
  }
  else
  {
    // track the camera pose
    trackPose(*scene->views[idx_keyframe], *scene->views.back());  
    removeOutlierLinks(*scene->views[idx_keyframe], *scene->views.back(), score, score_per_tile, dist_px);

    ok = isTrackOK(scene->views.back()->num_tiles, score, score_per_tile);

    if (ok)
    {
      // set keyframe
      double angle = getDeltaViewingAngle(scene->views[idx_keyframe]->pose, scene->views.back()->pose );

      if (scene->getFrameLast() != idx_keyframe 
          && (angle > param.angle_init_keyframe/180.*M_PI || dist_px>thr_image_motion_px) )
      {
        setKeyframe(cloud, *scene);
      }

      pose =  scene->views.back()->pose;

      // detect loops
      if (param.detect_loops) loopclosing->detectLoops(*scene->views.back());
    }

    if (!dbg.empty())       //<< debug draw
    {
      View &view = *scene->views.back();
      View &kf_view = *scene->views[idx_keyframe];

      if (ok)
        cv::circle(dbg, cv::Point(70,50), 10, CV_RGB(0,255,0), -1);
      else cv::circle(dbg, cv::Point(70,50), 10, CV_RGB(255,0,0), -1);

      for (unsigned i=0; i<view.keys.size(); i++) {
        if (view.keys[i].links.size()>0) 
          cv::line(dbg,view.keys[i].pt,kf_view.keys[view.keys[i].links[0].second].pt,CV_RGB(0,255,0));
      }
    }                      //>> end
  }

  return ok;
}

/**
 * doFullBundleAdjustment
 */
void CameraTrackerRGBD::doFullBundleAdjustment()
{
  #ifndef KP_NO_CERES_AVAILABLE
  bundler.reset( new BundleAdjusterRT());
  bundler->dbg = dbg;
  bundler->setSharedData(scene);
  bundler->optimize();
  #else
  throw std::runtime_error("[CameraTrackerRGBD::doFullBundleAdjustment] Ceres not found!");
  #endif
}


/**
 * setSharedData
 */
void CameraTrackerRGBD::setSharedData(Scene::Ptr &_scene)
{
  scene = _scene;

  keytracker->setSharedData(scene);
  if (param.detect_loops) loopclosing->setSharedData(scene);
}


}












