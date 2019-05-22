/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "LoopClosingRT.hh"
#include "v4r/KeypointTools/invPose.hpp"



namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
LoopClosingRT::LoopClosingRT(const Parameter &p)
 : param(p)
{ 
  rt.reset(new RigidTransformationRANSAC(param.rt_param));
  matcher = new cv::BFMatcher(cv::NORM_L2, true);

  rad_max_angle = param.max_angle*M_PI/180.;
  sqr_max_distance = param.max_distance*param.max_distance;
}

LoopClosingRT::~LoopClosingRT()
{
}

/**
 * computeCenter
 */
Eigen::Vector3f LoopClosingRT::computeCenter(const View &frame)
{
  unsigned cnt=0;
  Eigen::Vector3f center(0.,0.,0.);
  Eigen::Matrix4f inv_pose;

  for (unsigned i=0; i<frame.keys.size(); i++)
  {
    const LinkedKeypoint &key = frame.keys[i];
    if (!isnan(key.pt3[0]))
    {
      center += key.pt3;
      cnt++;
    }
  }

  center/=float(cnt);
  
  invPose(frame.pose, inv_pose);

  return inv_pose.topLeftCorner<3,3>()*center + inv_pose.block<3,1>(0,3);
}


/**
 * getDeltaViewingAngle
 */
double LoopClosingRT::getDeltaViewingAngle(View &frame1, const View &frame2)
{
  Eigen::Vector3f v0(0,0,1), v1, v2;

  if (isnan(frame1.center[0]))
    frame1.center = computeCenter(frame1);

  v1 = frame1.pose.topLeftCorner<3,3>()*v0;
  v2 = frame2.pose.topLeftCorner<3,3>()*v0;

  double cosa = v1.dot(v2);

  if (cosa > 0.9999999)
    return 0.;
  return acos(cosa);
}

/**
 * getDeltaViewingAngle
 */
double LoopClosingRT::getDeltaSqrPoseDistance(View &frame1, const View &frame2)
{
  Eigen::Matrix4f inv_pose1, inv_pose2;

  invPose(frame1.pose, inv_pose1);
  invPose(frame2.pose, inv_pose2);

  return (inv_pose1.block<3,1>(0,3)-inv_pose2.block<3,1>(0,3)).squaredNorm();
}

/**
 * isTrackOK
 */
bool LoopClosingRT::isTrackOK(int num_tiles, const double &total_score, const std::vector<double> &score_per_tile)
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
 * detectHypotheses
 */
void LoopClosingRT::detectHypotheses(std::vector<View::Ptr> &views, View &view, std::vector<int> &indices)
{ 
  if (!view.is_keyframe)
    return;

  bool have_delta=false;
  double v_pi = M_PI/4.;
  double angle, sqr_dist;
  vector<bool> valid_loops(views.size(), true);     // avoid multple similar loops

  indices.clear();

  // search backwards
  for (int i=view.idx-1; i>=0; i--)
  {
    if (views[i]->is_keyframe)
    {
      angle = getDeltaViewingAngle(view, *views[i]);
      sqr_dist = getDeltaSqrPoseDistance(view, *views[i]);

      if (!have_delta)
      {
        valid_loops[i] = false;
        if (angle > v_pi) have_delta=true;

        View &ref = *views[i];
        for (unsigned j=0; j<ref.links.size(); j++)
          valid_loops[ref.links[j]] = false;
      }

      if (have_delta && angle < rad_max_angle && sqr_dist < sqr_max_distance)
        indices.push_back(i);
    }
  }

  // search forwards (if loop closing is done in a batch process)
  for (int i=view.idx+1; i<(int)views.size(); i++)
  {
    if (views[i]->is_keyframe)
    {
      angle = getDeltaViewingAngle(view, *views[i]);

      if (!have_delta)
      {
        valid_loops[i] = false;
        if (angle > v_pi) have_delta=true;

        View &ref = *views[i];
        for (unsigned j=0; j<ref.links.size(); j++)
          valid_loops[ref.links[j]] = false;
      }

      if (have_delta && angle < rad_max_angle)
        indices.push_back(i);
    }
  }

  // check loops
  valid_loops[view.idx] = false;
  unsigned z;
  for (int i=0; i<(int)indices.size(); i++)
  {
    View &ref = *views[indices[i]];
    for (z=0; z<ref.links.size(); z++)
      if (!valid_loops[ref.links[z]])
        break;
    if (z!=ref.links.size())
    {
      indices.erase(indices.begin()+i);
      i--;
    }
  }
}


/**
 * matchHypotheses
 */
void LoopClosingRT::matchHypotheses(std::vector<View::Ptr> &views, View &view, std::vector<int> &indices, std::vector<std::vector<cv::DMatch> > &matches)
{
  matches.clear();
  matches.resize(indices.size());

  for (unsigned i=0; i<indices.size(); i++)
  {
    View &kf = *views[indices[i]];

    matcher->match(view.descs, kf.descs, matches[i]);
  }
}

/**
 * verifyMatchesRT
 */
int LoopClosingRT::verifyMatchesRT(std::vector<View::Ptr> &views, View &view, std::vector<int> &indices, std::vector<std::vector<cv::DMatch> > &matches, std::vector<Eigen::Matrix4f> &poses)
{
  int num_inl=0, idx=-1;
  std::vector<int> inliers;
  std::vector<Eigen::Vector3f> src_pts;
  std::vector<Eigen::Vector3f> tgt_pts;
  poses.resize(matches.size());

  for (unsigned i=0; i<matches.size(); i++)
  {
    View &kf = *views[indices[i]];
    std::vector<cv::DMatch> &ms = matches[i];
    src_pts.resize(ms.size());
    tgt_pts.resize(ms.size());

    if (ms.size()<param.min_total_score)
    {
      poses[i].setIdentity();
      continue;
    }

    for (unsigned j=0; j<ms.size(); j++)
    {
      cv::DMatch &ma = ms[j];
      src_pts[j] = kf.keys[ma.trainIdx].pt3;
      tgt_pts[j] = view.keys[ma.queryIdx].pt3;
      ma.imgIdx = -1;
    }

    rt->compute(src_pts, tgt_pts, poses[i], inliers);

    //mark inliers
    for (unsigned j=0; j<inliers.size(); j++)
      ms[inliers[j]].imgIdx = 1;
    
    if (int(inliers.size())>num_inl)
    {
      num_inl = inliers.size();
      idx =  i;
    }
  }

  return idx;
}

/**
 * insertKeypointLinks
 */
bool LoopClosingRT::insertKeypointLinks(View &kf, View &view, std::vector<cv::DMatch> &matches)
{
  double total_score=0;
  std::vector<double> score_per_tile(view.num_tiles*view.num_tiles,0.);

  if (view.num_tiles*view.num_tiles < param.min_tiles_used)
    param.min_tiles_used = 1;

  // check quality
  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    if (ma.imgIdx==1)
    {
      total_score+=1;
      score_per_tile[view.keys[ma.queryIdx].tile_label]+=1;
    }
  }

  // insert links
  if (isTrackOK(view.num_tiles, total_score, score_per_tile))
  {
    for (unsigned i=0; i<matches.size(); i++)
    {
      cv::DMatch &ma = matches[i];
      if (ma.imgIdx==1)
      {
        view.keys[ma.queryIdx].links.push_back(make_pair(kf.idx, ma.trainIdx));
      }
    }

    if (!dbg.empty()) cout<<"Have loop: "<<kf.idx<<" ("<<total_score<<") <<<<<----------------------"<<endl;

    return true;
  }
  return false;
}




/***************************************************************************************/

/**
 * detectLoops
 */
void LoopClosingRT::detectLoops(View &view)
{ 
  vector<int> loops;
  vector<vector<cv::DMatch> > matches;
  vector<Eigen::Matrix4f> poses;

  detectHypotheses(scene->views, view, loops);

  matchHypotheses(scene->views, view, loops, matches);

  int idx_best = verifyMatchesRT(scene->views, view, loops, matches, poses);

  if (idx_best!=-1)
  {
    if (insertKeypointLinks(*scene->views[loops[idx_best]], view, matches[idx_best]))
    {
      view.links.push_back(loops[idx_best]);
    }
  }
}



}












