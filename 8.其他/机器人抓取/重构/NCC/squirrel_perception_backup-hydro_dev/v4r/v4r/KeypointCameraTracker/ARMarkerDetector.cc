/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "ARMarkerDetector.hh"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/projectPointToImage.hpp"

namespace kp
{

using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
ARMarkerDetector::ARMarkerDetector(const string &config_file, const Parameter &p)
 : param(p)
{ 
  setup(config_file);
}

ARMarkerDetector::~ARMarkerDetector()
{
}


/**
 * setup
 * file format:
 *   number_of_marker
 *   marker_file_name
 *   x y z   # 3d coordinates of the upper right point [1 1]
 *   x y z   # 3d coordinates of the lower right point [1 0]
 *   x y z   # 3d coordinates of the lower left point [0 0]
 *   x y z   # 3d coordinates of the upper left point [0 1]
 *   ...
 */
void ARMarkerDetector::setup(const string &config_file)
{
  int num, id, z=0;;
  string file;
  cv::Point3f pt;

  ifstream in(config_file.c_str());

  if (in.is_open())
  {
    in>>num;

    while(!in.eof() && z<num)
    {
      in>>file;

      id = ardetector.loadMarker(file);
  
      if (id<0)
        throw runtime_error("[ARMarkerDetector::Setup] Marker file not found!");

      for (unsigned i=0; i<4; i++)
      {
        in>> pt.x >> pt.y >> pt.z;
        pts_marker[id].push_back(pt);
      }

      z++;
    }
    
  }
  else throw runtime_error("[ARMarkerDetector::Setup] Config file not found!");
}

/**
 * addToView
 */
void ARMarkerDetector::addToView(int id, const std::vector<cv::Point2f> &pts_image)
{
  std::map<int, std::vector<int> >::iterator itg;
  View &view = *scene->views.back();

  view.marker.push_back(Marker(id));
  Marker &m = view.marker.back();
  m.pts = pts_image;

  itg =  pts_global.find(id);

  if (itg != pts_global.end())
    m.glob_idx = itg->second;
}

/**
 * filter
 */
void ARMarkerDetector::filter(Scene &scene, View &view)
{
  Eigen::Vector2f pt_im;
  Eigen::Matrix3f R = view.pose.topLeftCorner<3,3>();
  Eigen::Vector3f pt, t = view.pose.block<3,1>(0,3);
  float dist;
  float sqr_inl_dist = param.reprojection_error*param.reprojection_error;
  bool have_dist = !dist_coeffs.empty();

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

      if (dist > sqr_inl_dist) m.glob_idx[j] = -1;
      if (pt[2] < 0.01 ) m.glob_idx[j] = -1;
    }
  }
}




/******************************* PUBLIC ***************************************/

/**
 * detect
 */
void ARMarkerDetector::detect(const cv::Mat_<cv::Vec3b> &image)
{
  if (scene.get()==0)
    throw std::runtime_error("[ARMarkerDetector::detect] No scene container available. Call \"setSharedData\"! ");
  if (scene->views.size()==0)
    throw std::runtime_error("[ARMarkerDetector::detect] No scene view available. Use e.g. \"KeypointTracker\"! ");

  std::vector<V4R::Marker> marker = ardetector.detectMarker(image,150);
  std::vector<cv::Point2f> pts_image;

  for (unsigned i=0; i<marker.size(); i++)
  {
    V4R::Marker &m = marker[i];

    if (m.id != -1)
    {
      pts_image.clear();
 
      for (unsigned j=0; j<4; j++)
      {
        pts_image.push_back(cv::Point2f(m.vertex[j][0], m.vertex[j][1]));
      }

      addToView(m.id, pts_image);

      if (!dbg.empty())
      {
        cv::line(dbg, cv::Point2f(m.vertex[2][0], m.vertex[2][1]), cv::Point2f(m.vertex[1][0], m.vertex[1][1]), CV_RGB(255,0,0), 2);
        cv::line(dbg, cv::Point2f(m.vertex[2][0], m.vertex[2][1]), cv::Point2f(m.vertex[3][0], m.vertex[3][1]), CV_RGB(0,255,0), 2);
      }
    }
  }
}

/**
 * reconstruct
 */
void ARMarkerDetector::reconstruct()
{
  cv::Mat_<double> rod, R(3,3), t;
  std::map<int, std::vector<cv::Point3f> >::iterator itm;
  View &view = *scene->views.back();
  Eigen::Matrix4f pose, inv_pose;
  Eigen::Matrix3f eR;
  Eigen::Vector3f et, pt;
  Scene &ref = *scene;

  for (unsigned i=0; i<view.marker.size(); i++)
  {
    Marker &m = view.marker[i];
    if (m.glob_idx[0] == -1)
    {
      itm = pts_marker.find(m.id);
      if (itm != pts_marker.end())
      {
        std::vector<cv::Point3f> &pts = itm->second;
        cv::solvePnP(cv::Mat(pts), cv::Mat(m.pts), intrinsic, dist_coeffs, rod, t, false);
        cv::Rodrigues(rod,R);
        cvToEigen(R, t, pose); 

        if (view.pose == Eigen::Matrix4f::Identity())
        {  
          view.pose = pose;
        }

        invPose(view.pose,inv_pose);
        pose = inv_pose*pose;

        eR = pose.topLeftCorner<3,3>();
        et = pose.block<3,1>(0, 3);

        std::vector<int> &pg = pts_global[m.id];
       
        for (unsigned j=0; j<pts.size(); j++)
        {
          m.glob_idx[j] = ref.points.size();
          pg.push_back(ref.points.size());
          pt = eR*Eigen::Map<Eigen::Vector3f>(&pts[j].x) + et;
          ref.points.push_back(Eigen::Vector3d(pt[0],pt[1],pt[2]));
        }
      }
    }
  }

  filter(*scene, view);
}


/**
 * setCameraParameter
 */
void ARMarkerDetector::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
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












