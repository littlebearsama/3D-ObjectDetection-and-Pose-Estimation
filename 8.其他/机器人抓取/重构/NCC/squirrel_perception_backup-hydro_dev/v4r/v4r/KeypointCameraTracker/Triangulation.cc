/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

/**
 * $Id$
 */

#include "Triangulation.hh"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/projectPointToImage.hpp"

//#define KT_DEBUG


namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
Triangulation::Triangulation(const Parameter &p)
 : param(p)
{ 
  min_angle_rad = param.min_angle/180.*M_PI;
}

Triangulation::~Triangulation()
{
}

/**
 * getPoints
 */
void Triangulation::getPoints(View &keyframe, View &view, std::vector<cv::Point2f> &pts_kf, std::vector<cv::Point2f> &pts_view, std::vector< std::pair<int, int> > &kf_view_idx)
{
  int kf_idx;
  pts_kf.clear();
  pts_view.clear();
  kf_view_idx.clear();

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.pt3_glob<0 && key.links.size()>0)
    {
      kf_idx = key.links[0].second;
      pts_kf.push_back(keyframe.keys[kf_idx].pt);
      pts_view.push_back(key.pt);
      kf_view_idx.push_back(make_pair(kf_idx,i));
    }
  }

  if (pts_view.size()<3) return;

  if (!dist_coeffs.empty())
  {
    cv::Mat_<double> P(3,4);
    P << intrinsic(0,0),0,intrinsic(0,2),0, 0,intrinsic(1,1),intrinsic(1,2),0, 0,0,1,0;

    std::vector<cv::Point2f> pts_tmp(pts_kf.size());
    cv::undistortPoints(cv::Mat(pts_kf), cv::Mat(pts_tmp), intrinsic, dist_coeffs, cv::Mat(), P);

    pts_kf = pts_tmp;

    pts_tmp.resize(pts_view.size());
    cv::undistortPoints(cv::Mat(pts_view), cv::Mat(pts_tmp), intrinsic, dist_coeffs, cv::Mat(), P);

    pts_view = pts_tmp;
  }
}

/**
 * triangulatePoints
 */
void Triangulation::triangulatePoints(std::vector<cv::Point2f> &pts_kf, std::vector<cv::Point2f> &pts_view, std::vector< std::pair<int, int> > &kf_view_idx, View &keyframe, View &view)
{
  float dist_kf, dist_view;
  cv::Mat pts4d;
  cv::Mat_<float> p_kf(3,4), p_view(3,4);
  Eigen::Matrix<float, 3, 4> proj_kf, proj_view, C(Eigen::Matrix<float, 3, 4>::Zero());
  std::vector<Eigen::Vector3d> &pts = scene->points;

  C(0,0) = intrinsic(0,0); C(0,1) = intrinsic(0,1); C(0,2) = intrinsic(0,2);
  C(1,0) = intrinsic(1,0); C(1,1) = intrinsic(1,1); C(1,2) = intrinsic(1,2);
  C(2,0) = intrinsic(2,0); C(2,1) = intrinsic(2,1); C(2,2) = intrinsic(2,2);

  proj_kf = C*keyframe.pose;
  proj_view = C*view.pose;

  eigenToCv(proj_kf, p_kf);
  eigenToCv(proj_view, p_view);

  if (pts_kf.size()<3 || pts_view.size()<3) return;
 
  cv::triangulatePoints(p_kf, p_view, cv::Mat(pts_kf), cv::Mat(pts_view), pts4d);

  float w;
  Eigen::Vector3f v1, v2, pt3, pt3_cam;
  Eigen::Vector2f pt;
  Eigen::Matrix3f R_view = view.pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t_view = view.pose.block<3,1>(0, 3);
  Eigen::Matrix3f R_kf = keyframe.pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t_kf = keyframe.pose.block<3,1>(0, 3);
  float sqr_thr = param.reprojection_error*param.reprojection_error; 
  Eigen::Matrix4f inv_pose_kf, inv_pose_view;
  invPose(keyframe.pose, inv_pose_kf);
  invPose(view.pose, inv_pose_view);

  for (int i=0; i<pts4d.cols; i++)
  {
    w = pts4d.at<float>(3,i);
    pt3 = Eigen::Vector3f(pts4d.at<float>(0,i)/w, pts4d.at<float>(1,i)/w, pts4d.at<float>(2,i)/w);

    pt3_cam = R_kf*pt3 + t_kf;
    projectPointToImage(&pt3_cam[0], intrinsic.ptr<double>(), &pt[0]);
    dist_kf = (pt-Eigen::Map<Eigen::Vector2f>(&pts_kf[i].x)).squaredNorm();

    pt3_cam = R_view*pt3 + t_view;
    projectPointToImage(&pt3_cam[0], intrinsic.ptr<double>(), &pt[0]);
    dist_view = (pt-Eigen::Map<Eigen::Vector2f>(&pts_view[i].x)).squaredNorm();

    v1 = (inv_pose_kf.block<3,1>(0,3)-pt3).normalized();
    v2 = (inv_pose_view.block<3,1>(0,3)-pt3).normalized();

    #ifdef KT_DEBUG
    cout<<pt3.transpose()<<": "<<sqrt(dist_kf)<<", "<<sqrt(dist_view)<<", "<<acos(v1.dot(v2))*180./M_PI;
    #endif

    if (dist_kf<sqr_thr && dist_view<sqr_thr && acos(v1.dot(v2)) > min_angle_rad)
    {
      std::pair<int, int> &idx = kf_view_idx[i];
      keyframe.keys[idx.first].pt3_glob = pts.size();
      view.keys[idx.second].pt3_glob = pts.size();
      pts.push_back(Eigen::Vector3d(pt3[0],pt3[1],pt3[2]));
      #ifdef KT_DEBUG
      cout<<"ok"<<endl;
      #endif
    }
    #ifdef KT_DEBUG
    else cout<<"failed"<<endl;
   #endif
  }
  
}

/**
 * filter
 */
void Triangulation::filter(Scene &scene, View &view)
{
  Eigen::Vector2f pt_im;
  Eigen::Matrix3f R = view.pose.topLeftCorner<3,3>();
  Eigen::Vector3f pt, t = view.pose.block<3,1>(0,3);
  float dist;
  float sqr_inl_dist = param.reprojection_error2*param.reprojection_error2;
  bool have_dist = !dist_coeffs.empty();


  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.pt3_glob>=0)
    {
      Eigen::Vector3d &pt_glob = scene.points[key.pt3_glob];
      pt = Eigen::Vector3f(pt_glob[0], pt_glob[1], pt_glob[2]);

      pt = R * pt + t;

      if (have_dist)
        projectPointToImage(&pt[0],intrinsic.ptr<double>(),dist_coeffs.ptr<double>(),&pt_im[0]);
      else projectPointToImage(&pt[0], intrinsic.ptr<double>(), &pt_im[0]);

      dist = (pt_im-Eigen::Map<Eigen::Vector2f>(&key.pt.x)).squaredNorm();

      if (dist>sqr_inl_dist) key.pt3_glob=-1;
      if (pt[2] < 0.01 ) key.pt3_glob=-1;
    }
  }

  for (unsigned i=0; i<view.marker.size(); i++)
  {
    Marker &m = view.marker[i];
    for (unsigned j=0; j<m.pts.size(); j++)
    {
      Eigen::Vector3d &pt_glob = scene.points[m.glob_idx[j]];
      pt = Eigen::Vector3f(pt_glob[0], pt_glob[1], pt_glob[2]);

      pt = R * pt + t;

      if (have_dist)
        projectPointToImage(&pt[0],intrinsic.ptr<double>(),dist_coeffs.ptr<double>(),&pt_im[0]);
      else projectPointToImage(&pt[0], intrinsic.ptr<double>(), &pt_im[0]);

      dist = (pt_im-Eigen::Map<Eigen::Vector2f>(&m.pts[j].x)).squaredNorm();

      if (dist>sqr_inl_dist) m.glob_idx[j] = -1;
      if (pt[2] < 0.01 ) m.glob_idx[j] = -1;

    }
  }
}









/***************************************************************************************/

/**
 * triangulate
 */
int Triangulation::triangulate()
{
  int cnt=0;
  int idx_keyframe = scene->getKeyframe();
  int idx_view = scene->getFrameLast();

  if (idx_keyframe<0 || idx_keyframe==idx_view)
    return cnt;

  std::vector< std::pair<int, int> > kf_view_idx;
  View &view = *scene->views.back();
  View &keyframe = *scene->views[idx_keyframe];

  filter(*scene, view);
  //filter(*scene, keyframe);

  getPoints(keyframe, view, pts_kf, pts_view, kf_view_idx);

//cv::correctMatches(InputArray F, InputArray points1, InputArray points2, OutputArray newPoints1, OutputArray newPoints2)
  triangulatePoints( pts_kf, pts_view, kf_view_idx, keyframe, view);

  for (unsigned i=0; i<view.keys.size(); i++)
    if (view.keys[i].pt3_glob >= 0) cnt++;

  return cnt;
}


/**
 * setCameraParameter
 */
void Triangulation::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
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
}


}












