/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "CameraTrackerPnP.hh"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/projectPointToImage.hpp"



namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
CameraTrackerPnP::CameraTrackerPnP(const Parameter &p)
 : param(p)
{ 
}

CameraTrackerPnP::~CameraTrackerPnP()
{
}


/**
 * getPoints
 */
void CameraTrackerPnP::getPoints(View &keyframe, View &view, std::vector<cv::Point3f> &model_pts, std::vector<cv::Point2f> &query_pts)
{
  int idx;
  query_pts.clear();
  model_pts.clear();
  query_pts.reserve(view.keys.size());
  model_pts.reserve(view.keys.size());
  std::vector<Eigen::Vector3d> &gp = scene->points;

  for (unsigned i=0; i<view.keys.size(); i++) 
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      idx = keyframe.keys[key.links[0].second].pt3_glob;
      if (idx>=0)
      {
        Eigen::Vector3d &pt3 = gp[ idx ];
        query_pts.push_back(key.pt);
        model_pts.push_back( cv::Point3f(pt3[0],pt3[1],pt3[2]) );
      }
    }
  }

  for (unsigned i=0; i<view.marker.size(); i++)
  {
    Marker &m = view.marker[i];

    for (unsigned j=0; j<m.pts.size(); j++)
    {
      if (m.glob_idx[j]>=0)
      {
        Eigen::Vector3d &pt3 = gp[ m.glob_idx[j] ];
        query_pts.push_back( m.pts[j] );
        model_pts.push_back( cv::Point3f(pt3[0],pt3[1],pt3[2]) );
      }
    }
  }
}

/**
 * trackPose
 */
void CameraTrackerPnP::trackPose(View &keyframe, View &view)
{
  getPoints(keyframe, view, model_pts, query_pts);

  if (query_pts.size() > 4)
  {
    //ransac pose
    std::vector<int> inliers;
    cv::Mat_<double> R(3,3), rvec, tvec;
    std::vector<cv::Point3f> model_pts2;
    std::vector<cv::Point2f> query_pts2;

    cv::solvePnPRansac(cv::Mat(model_pts), cv::Mat(query_pts), intrinsic, dist_coeffs, rvec, tvec, false, param.iterationsCount, param.reprojectionError, param.minInliersCount, inliers, cv::P3P );

    if (inliers.size()<5)
      return;

    // refine pose
    model_pts2.resize(inliers.size());
    query_pts2.resize(inliers.size());

    for (unsigned i=0; i<inliers.size(); i++)
    {
      model_pts2[i] = model_pts[inliers[i]];
      query_pts2[i] = query_pts[inliers[i]];
    }

    cv::solvePnP(cv::Mat(model_pts2), cv::Mat(query_pts2), intrinsic, dist_coeffs, rvec, tvec, true, cv::ITERATIVE );

    // return pose
    cv::Rodrigues(rvec, R);
    view.pose.setIdentity();
    for (unsigned i=0; i<3; i++)
      for (unsigned j=0; j<3; j++)
        view.pose(i,j) = R(i,j);
    for (unsigned i=0; i<3; i++)
      view.pose(i,3) = tvec(i);
  }
}


/**
 * removeOutlierLinks
 */
//int total_inliers=0;
//double total_score=0.;
void CameraTrackerPnP::removeOutlierLinks(View &keyframe, View &view, double &score, std::vector<double> &score_per_tile)
{
  int idx, cnt=0;
  double dist, tmp;
  bool have_dist = !dist_coeffs.empty();
  float sqr_inl_dist = param.reprojectionError*param.reprojectionError;
  Eigen::Matrix3f R = view.pose.topLeftCorner<3, 3>();
  Eigen::Vector3f t = view.pose.block<3,1>(0, 3);
  Eigen::Vector3f pt3;
  Eigen::Vector2f pt;

  score = 0.;
  score_per_tile.clear();
  score_per_tile.resize(view.num_tiles*view.num_tiles,0.);
  std::vector<Eigen::Vector3d> &gp = scene->points;
//std::vector<int> h(6,0);

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      idx = keyframe.keys[key.links[0].second].pt3_glob;
      if (idx>=0)
      {
        Eigen::Vector3d &pt3_glob = gp[ idx ];
        pt3 = R * Eigen::Vector3f(pt3_glob[0],pt3_glob[1],pt3_glob[2]) + t;

        if (have_dist)
          projectPointToImage(&pt3[0], intrinsic.ptr<double>(), dist_coeffs.ptr<double>(), &pt[0]);
        else projectPointToImage(&pt3[0], intrinsic.ptr<double>(), &pt[0]);

        dist = (pt- Eigen::Map<Eigen::Vector2f>(&key.pt.x)).squaredNorm();

        if (dist < sqr_inl_dist)
        {
          cnt++;
          tmp = sqr_inl_dist - dist;
          score += tmp;
          score_per_tile[key.tile_label] += tmp;
          key.pt3_glob = idx;                    // set 3d point index
        }
        else key.links.clear();
/*double d = sqrt(dist);
if (d<0.2) h[0]++;
else if (d<0.5) h[1]++;
else if (d<1.) h[2]++;
else if (d<2.) h[3]++;
else if (d<3.) h[4]++;
else h[5]++;*/
      }
    }
  }
/*for (unsigned i=0; i<h.size(); i++)
cout << h[i] << " ";
cout<<endl;*/

  score /= sqr_inl_dist;
  for (unsigned i=0; i<score_per_tile.size(); i++)
    score_per_tile[i] /= sqr_inl_dist;

  //cout<<"[CameraTrackerPnP] Number of inliers: "<<cnt<<endl;
  //total_inliers+=cnt;
  //total_score+=score;
  //cout<<"total_inliers="<<total_inliers<<endl;
//cout<<"total_score="<<total_score<<endl;
}

/**
 * isTrackOK
 */
bool CameraTrackerPnP::isTrackOK(int num_tiles, const double &total_score, const std::vector<double> &score_per_tile)
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
    if (score_rows[i] > 0) zu++;

  if (z>=param.min_tiles_used && sum > param.min_total_score)
    if (param.min_tiles_used < 3 || (zv>=2 && zu>=2))
      return true;
  return false;
}





/***************************************************************************************/


/**
 * track
 */
bool CameraTrackerPnP::track()
{
  // track camera
  double score;
  std::vector<double> score_per_tile; 
  int idx_keyframe = scene->getKeyframe();
  View &view = *scene->views.back();

  if ( idx_keyframe != -1)
  {
    View &keyframe = *scene->views[idx_keyframe];
    trackPose(keyframe, view);
    removeOutlierLinks(keyframe, view, score, score_per_tile);

    if (!dbg.empty()) {
      for (unsigned i=0; i<view.keys.size(); i++) {
        if (view.keys[i].links.size()>0) {
          cv::line(dbg,view.keys[i].pt,keyframe.keys[view.keys[i].links[0].second].pt,CV_RGB(0,255,0)); }
      }
    }
    //cout<<"score="<<score<<endl;
    //for (unsigned i=0; i<score_per_tile.size(); i++)
    //  cout<<score_per_tile[i]<<" ";
    //cout<<endl;
    //cout<<"--"<<endl;

    if (isTrackOK(view.num_tiles, score, score_per_tile) || view.marker.size()>0)
    {
      if (!dbg.empty()) cv::circle(dbg, cv::Point(70,50), 10, CV_RGB(0,255,0), -1);

      view.tracked=true;
      return true;
    }

    if (!dbg.empty()) cv::circle(dbg, cv::Point(70,50), 10, CV_RGB(255,0,0), -1);
  }

  return false;
}

/**
 * setCameraParameter
 */
void CameraTrackerPnP::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
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












