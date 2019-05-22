/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "KeypointTracker.hh"
#include <opencv2/highgui/highgui.hpp>
//#include <pcl/common/time.h>



namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
KeypointTracker::KeypointTracker(const Parameter &p)
  : param(p), status(0), idx_keyframe(-1)
{ 
  detector.reset(new FeatureDetector_KD_FAST_IMGD(FeatureDetector_KD_FAST_IMGD::Parameter(param.max_features_tile, 1.2, param.fast_pyr_levels, PATCH_SIZE)) );
  matcher = new cv::BFMatcher(cv::NORM_L2);

  scene.reset(new Scene);
  
  if (param.min_tiles_used > param.tiles*param.tiles)
    param.min_tiles_used = param.tiles*param.tiles;
}

KeypointTracker::~KeypointTracker()
{
}

/**
 * @brief updateKeypointCoordinates
 */
void KeypointTracker::updateKeypointCoordinates(const cv::Rect &rect, std::vector<cv::KeyPoint> &keys)
{
   cv::Point2f pt_offs(rect.x, rect.y);

   for (unsigned i=0; i<keys.size(); i++)
     keys[i].pt += pt_offs;
}

/**
 * @brief KeypointTracker::setKeypointLinks
 * @param view
 * @param keyframe
 * @param matches
 */
void KeypointTracker::setKeypointLinks(View &view, View &keyframe, std::vector<cv::DMatch> &matches)
{
  if (matches.size()<5)
    return;

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    LinkedKeypoint &view_key = view.keys[ma.queryIdx];
    if (ma.distance<view_key.dist_tmp)
    {
      view_key.dist_tmp = ma.distance;
      view_key.links.push_back(make_pair(keyframe.idx,ma.trainIdx));
    }
  }
}

/**
 * @brief KeypointTracker::filterKeypointLinks2
 * @param view
 * @param keyframe
 */
int KeypointTracker::filterKeypointLinks2(View &view, View &keyframe)
{
  Eigen::Matrix3f transform;
  std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > src;
  std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > tgt;
  unsigned cnt=0;
  std::vector<int> inliers;
  std::vector<bool> inl_map;
  ImageTransformRANSAC imt(ImageTransformRANSAC::Parameter(param.inl_dist,0.01,10000));

  src.reserve(view.keys.size());
  tgt.reserve(view.keys.size());
  inl_map.reserve(view.keys.size());

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      if (status&2)
        src.push_back(Eigen::Map<Eigen::Vector2f>(&keyframe.keys[key.links[0].second].pt_pred.x));
      else src.push_back(Eigen::Map<Eigen::Vector2f>(&keyframe.keys[key.links[0].second].pt.x));
      tgt.push_back(Eigen::Map<Eigen::Vector2f>(&key.pt.x));
      inl_map.push_back(false);
    }
  }

  if (src.size()<4)
    return src.size();

  if (param.affine_outl_rejection)
    imt.ransacAffine(src,tgt,transform,inliers);
  else imt.ransacSimilarity(src,tgt,transform,inliers);
  //imt.ransacHomography(src,tgt,transform,inliers);

  for (unsigned i=0; i<keyframe.keys.size(); i++)
    keyframe.keys[i].have_pred = false;
  for (unsigned i=0; i<inliers.size(); i++)
    inl_map[inliers[i]]=true;

  unsigned z=0;
  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      keyframe.keys[key.links[0].second].have_pred = false;

      if (inl_map[z])
      {
        keyframe.keys[key.links[0].second].pt_pred = key.pt;
        keyframe.keys[key.links[0].second].have_pred = true;
        cnt++;
      }
      else key.links.clear();

      z++;
    }
  }

  return cnt;
}

/**
 * @brief KeypointTracker::filterKeypointLinks
 * @param view
 * @param keyframe
 */
int KeypointTracker::filterKeypointLinks(View &view, View &keyframe)
{
  cv::Mat_<float> transform;
  std::vector<cv::Point2f> src, tgt;
  unsigned cnt=0;

  src.reserve(view.keys.size());
  tgt.reserve(view.keys.size());

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {

      src.push_back(keyframe.keys[key.links[0].second].pt_pred);
      tgt.push_back(key.pt);
    }
  }

  if (src.size()<5)
    return src.size();

  transform = cv::findHomography(src, tgt, CV_RANSAC, param.inl_dist);
  cv::perspectiveTransform(src,src,transform);

  for (unsigned i=0; i<keyframe.keys.size(); i++)
    keyframe.keys[i].have_pred = false;

  unsigned z=0;
  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];

    if (key.links.size()>0)
    {
      keyframe.keys[key.links[0].second].have_pred = false;
      if (cv::norm(src[z]-tgt[z]) < param.inl_dist)
      {
        keyframe.keys[key.links[0].second].pt_pred = key.pt;
        keyframe.keys[key.links[0].second].have_pred = true;
        cnt++;
      }
      else key.links.clear();

      z++;
    }
  }

  return cnt;
}

/**
 * @brief KeypointTracker::predictKeypoints
 * @param view
 * @param keyframe
 */
void KeypointTracker::predictKeypoints(View &view, View &keyframe)
{
  cv::Mat_<float> transform;
  std::vector<cv::Point2f> src, tgt;

  src.reserve(view.keys.size());
  tgt.reserve(view.keys.size());

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      src.push_back(keyframe.keys[key.links[0].second].pt);
      tgt.push_back(key.pt);
    }
  }

  if (src.size()<5)
    return;

  transform = cv::findHomography(src, tgt);

  for (unsigned i=0; i<keyframe.keys.size(); i++)
  {
    LinkedKeypoint &kf_key = keyframe.keys[i];
    if (!kf_key.have_pred)
    {
      transformPoint(kf_key.pt, kf_key.pt_pred, transform);
    }
  }
}

/**
 * preFilterMatches
 */
void KeypointTracker::preFilterMatches(View &view, View &keyframe, std::vector<cv::DMatch> &matches)
{
  std::vector<Eigen::Vector2f> mots(matches.size());

  for (unsigned i=0; i<matches.size(); i++)
  {
    cv::DMatch &ma = matches[i];
    mots[i] = Eigen::Map<Eigen::Vector2f>(&view.keys[ma.queryIdx].pt.x) - 
              Eigen::Map<Eigen::Vector2f>(&keyframe.keys[ma.trainIdx].pt.x);//_pred.x);
  }

  float thr;
  unsigned j, z=0;

  for (unsigned i=0; i<mots.size(); i++)
  {
    Eigen::Vector2f &mot = mots[i];

    thr = param.pecnt_prefilter*mot.norm();
    if (thr<param.inl_dist) thr = param.inl_dist;

    thr = thr*thr;

    for (j=0; j<mots.size(); j++)
    {
      if (i!=j && (mots[j]-mot).squaredNorm()<thr)
        break;
    }

    if (j!=mots.size())
    {
      matches[z] = matches[i];
      z++;
    }
  }

  matches.resize(z);
}

/**
 * @brief KeypointTracker::refineKeypointLocationMappedLK
 * @param image
 * @param view
 * @param keyframe
 */
void KeypointTracker::refineKeypointLocationMappedLK(const cv::Mat_<unsigned char> &image, View &view, View &keyframe)
{
  std::vector<unsigned char> status;
  std::vector<float> error;
  std::vector<cv::Point2f> src, tgt;
  src.reserve(keyframe.keys.size());
  tgt.reserve(view.keys.size());
  cv::Point2f pt;

  cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
  cv::Size win_size(21,21);
  int max_level = 0;

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      src.push_back(keyframe.keys[key.links[0].second].pt);
      tgt.push_back(key.pt);
    }
  }

  if (src.size()<5)
    return;

  cv::Mat transform = cv::findHomography(src, tgt, CV_RANSAC, param.inl_dist/2.);
  cv::warpPerspective(keyframe.image, im_tmp, transform, image.size());

  for (unsigned i=0; i<src.size(); i++)
  {
    pt = src[i];
    transformPoint(pt, src[i], transform);
  }

  cv::calcOpticalFlowPyrLK(im_tmp, image, src, tgt, status, error, win_size, max_level, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, 0.001 );

  unsigned z=0, cnt=0;
  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      if (status[z]!=0)
      {
        key.pt = tgt[z];
        cnt++;
      }
      else key.links.clear();
      z++;
    }
  }

}

/**
 * @brief KeypointTracker::refineKeypointLocation
 * @param image
 * @param view
 * @param keyframe
 */
void KeypointTracker::refineKeypointLocationLK(const cv::Mat_<unsigned char> &image, View &view, View &keyframe)
{
  std::vector<unsigned char> status;
  std::vector<float> error;
  std::vector<cv::Point2f> src, tgt;
  src.reserve(keyframe.keys.size());
  tgt.reserve(view.keys.size());

  cv::TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
  cv::Size win_size(21,21);
  int max_level = 0;

  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      src.push_back(keyframe.keys[key.links[0].second].pt);
      tgt.push_back(key.pt);
    }
  }

  if (src.size()<5)
    return;

  cv::calcOpticalFlowPyrLK(keyframe.image, image, src, tgt, status, error, win_size, max_level, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, 0.001 );

  unsigned z=0, cnt=0;
  for (unsigned i=0; i<view.keys.size(); i++)
  {
    LinkedKeypoint &key = view.keys[i];
    if (key.links.size()>0)
    {
      if (status[z]!=0)
      {
        key.pt = tgt[z];
        cnt++;
      }
      else key.links.clear();
      z++;
    }
  }
  //cout<<"lttracker cnt="<<cnt<<"/"<<src.size()<<endl;
}

/**
 * isTrackOK
 */
bool KeypointTracker::isTrackOK(const View &view)
{
  std::vector<int> inl_per_tile(param.tiles*param.tiles,0);

  for (unsigned i=0; i<view.keys.size(); i++)
    if (view.keys[i].links.size()>0)
      inl_per_tile[view.keys[i].tile_label]++;

  int sum=0, z=0; 
  for (unsigned i=0; i<inl_per_tile.size(); i++)
  {
    if (inl_per_tile[i]>0)
    {
      z++;
      sum += inl_per_tile[i];
    }
  }

  if (z>=param.min_tiles_used && sum > param.min_total_matches)
    return true;
  return false;
}

/**
 * selectKeypoints
 */
void KeypointTracker::selectKeypoints(std::vector<cv::KeyPoint> &keys, cv::Mat &descs, const cv::Mat_<unsigned char> &mask)
{
  if (descs.type()!=CV_32F)
    throw std::runtime_error("[KeypointTracker::selectKeypoints] Descriptor type is not supported!");

  if (mask.empty())
    return;

  unsigned z=0;
  int cols = descs.cols;
  int x, y;

  for (unsigned i=0; i<keys.size(); i++)
  {
    cv::KeyPoint &key = keys[i];
    x = int(key.pt.x+.5);
    y = int(key.pt.y+.5);

    if (x>=0 && y>=0 && x<mask.cols && y<<mask.rows && mask(y,x)>0)
    {
      memcpy ( &descs.at<float>(z,0), &descs.at<float>(i,0), sizeof(float)*cols );
      keys[z] = keys[i];
      z++;
    }
  }

  keys.resize(z);
  descs.resize(z);
}






/***************************************************************************************/

/**
 * track
 */
//unsigned cnt=0;
void KeypointTracker::track(const cv::Mat_<unsigned char> &image, const cv::Mat_<unsigned char> &mask)
{
  //pcl::ScopeTime t("keypoint tracking");
  if (!dbg.empty()) cout<<"-- [KeypointTracker::track] debug out --"<<endl;

  tile_size_w = image.cols/param.tiles;
  tile_size_h = image.rows/param.tiles;

  int inl=0;
  scene->views.push_back(View::Ptr(new View(scene->views.size(),param.tiles)));
  View &view = *scene->views.back();
  cv::Rect rect;

  // detct keypoints
  for (int v=0; v<param.tiles; v++)
  {
    for (int u=0; u<param.tiles; u++)
    {
      getExpandedRect(u,v, image.rows, image.cols, rect);
      detector->detect(image(rect), keys, descs);

      if (!dbg.empty()) {
        cv::rectangle(dbg, cv::Rect(u*tile_size_w, v*tile_size_h, tile_size_w, tile_size_h), CV_RGB(255,255,0));
        cv::rectangle(dbg, rect, CV_RGB(0,255,0));
      }

      updateKeypointCoordinates(rect,keys);
      selectKeypoints(keys, descs, mask);
      View::insert(view, keys, descs, v*param.tiles+u);
      status |= 1;

      //cout<<"tile "<<v*param.tiles+u<<": "<<keys.size()<<" features"<<endl;  //DEBUG!!!!
    }
  }

  // track keypoints
  idx_keyframe = scene->getKeyframe();

  if (idx_keyframe!=-1)
  {
    View &keyframe = *scene->views[idx_keyframe];

    matcher->match(view.descs, keyframe.descs, matches);

    preFilterMatches(view, keyframe, matches);

    if (!dbg.empty()) {
      for (unsigned i=0; i<matches.size(); i++) {
        cv::DMatch &ma = matches[i];
        cv::line(dbg,keyframe.keys[ma.trainIdx].pt, view.keys[ma.queryIdx].pt,CV_RGB(0,0,0));
      }
    }

    setKeypointLinks(view, keyframe, matches);

    //{ pcl::ScopeTime t("refine keypoint location");
    if (param.refineLK) refineKeypointLocationLK(image, view, keyframe);
    else if (param.refineMappedLK) refineKeypointLocationMappedLK(image, view, keyframe);
    //}

    //{ pcl::ScopeTime t("filter keypoints");
    //inl=filterKeypointLinks(view, keyframe);     // cv homography version
    inl=filterKeypointLinks2(view, keyframe);  // aff version

    if (isTrackOK(view))
    {
      if (!dbg.empty()) cv::circle(dbg, cv::Point(50,50), 8, CV_RGB(0,255,0), -1);

      predictKeypoints(view, keyframe);
      status |= 2;
    } 
    else 
    {
      if (!dbg.empty()) cv::circle(dbg, cv::Point(50,50), 8, CV_RGB(255,0,0), -1);
      status &= ~2;
    }
    //}
  }

  if (!dbg.empty()) {
    cout<<"total number of features: "<<view.keys.size()<<endl;
    for (unsigned i = 0; i<view.keys.size(); i++)
      cv::circle(dbg, view.keys[i].pt, 2, CV_RGB(0,0,255));
    if (idx_keyframe!=-1)
    {
      View &keyframe = *scene->views[idx_keyframe];
      for (unsigned i=0; i<keyframe.keys.size(); i++)
          cv::line(dbg,keyframe.keys[i].pt,keyframe.keys[i].pt_pred,CV_RGB(255,0,0));
      for (unsigned i=0; i<view.keys.size(); i++) {
        if (view.keys[i].links.size()>0) { //cnt++;
          cv::line(dbg,view.keys[i].pt,keyframe.keys[view.keys[i].links[0].second].pt,CV_RGB(255,255,255)); }
      }
      for (unsigned i=0; i<view.keys.size(); i++) {
        if (view.keys[i].links.size()>0)
          cv::line(dbg,view.keys[i].pt,keyframe.keys[view.keys[i].links[0].second].pt_pred,CV_RGB(255,255,0));
      }
    }
    cout<<"number of prefiltered matches: "<<matches.size()<<endl;
    cout<<"number of inlier: "<<inl<<endl;
  }
}




}












