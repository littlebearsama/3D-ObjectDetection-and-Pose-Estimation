/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "OptimizePointsMultiViewGD.hh"
#include <opencv2/highgui/highgui.hpp>
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/ScopeTime.hpp"
#include "v4r/KeypointTools/rotation.h"


namespace kp 
{

using namespace std;


/********************** OptimizePointsMultiViewGD ************************
 * Constructor/Destructor
 */
OptimizePointsMultiViewGD::OptimizePointsMultiViewGD(const Parameter &p)
 : have_im_pts(false), have_depth(false), have_points(false), 
   have_normals(false), have_im_indices(false), param(p)
{
  cos_view_ray_normal_offset = cos(param.view_ray_normal_offset*M_PI/180);
  mv_views.resize(1);
  mv_views[0].idx = 0;
  poses.resize(1);
}

OptimizePointsMultiViewGD::~OptimizePointsMultiViewGD()
{
}




/************************** PRIVATE ************************/


/**
 * optimizeSequentialDim
 */
bool OptimizePointsMultiViewGD::optimizeSequentialDim(MultiViewPatchError &mpe, Eigen::Vector3d &rot, double &ddepth)
{
  int z;
  double delta_a = param.init_delta_rot;
  double delta_s = param.init_delta_trans; 
  double tmp_ddepth, sv_ddepth, sv_err, err, sv_err1, start_err;
  Eigen::Vector3d tmp_rot, sv_rot;

  sv_ddepth = ddepth = 0.;
  sv_rot = rot = Eigen::Vector3d::Zero();

  mpe(rot, ddepth, start_err);
  sv_err1 = start_err;

  for (z=0; z<param.max_iter; z++)
  {
    sv_err = sv_err1;

    for (int k=-1; k<=1; k+=2)
    {
      // test rotations
      for (int j=0; j<3; j++)
      {
        tmp_ddepth = ddepth;
        tmp_rot = rot;
        tmp_rot[j] += k*delta_a;

        mpe(tmp_rot, tmp_ddepth, err);

        if (err < sv_err)
        {
          sv_ddepth = tmp_ddepth;
          sv_rot = tmp_rot;
          sv_err = err;
        }
      }    
      // test depth
      tmp_ddepth = ddepth + k*delta_s;
      tmp_rot = rot;

      mpe(tmp_rot, tmp_ddepth, err);

      if (err < sv_err)
      {
        sv_ddepth = tmp_ddepth;
        sv_rot = tmp_rot;
        sv_err = err;
      }
    }

    if (sv_err < sv_err1)
    {
      rot = sv_rot;
      ddepth = sv_ddepth;
      sv_err1 = sv_err;
    }
    else
    {
      delta_a *= 0.5;
      delta_s *= 0.5;

      if (delta_a < param.convergence_rot && delta_s < param.convergence_trans)
      {
        break;
      }
    }
  }

  if ( sv_err1 < start_err && z < param.max_iter)
  {
    return true;
  }
  return false;
}

/**
 * initData
 */
void OptimizePointsMultiViewGD::initData(const Eigen::Matrix4f &pose) 
{
  if (!(have_im_pts && have_depth) && !have_points)
    throw std::runtime_error("[OptimizePointsMultiViewGD::InitData] Invalid data! You need ether 3D points or image points and the depth!"); 

  view_rays.clear();
  
  Eigen::Vector3f pt3, vr, t, inv_t;
  Eigen::Matrix3f R, inv_R;
  Eigen::Matrix4f inv_pose;

  invPose(pose, inv_pose);
  R = pose.topLeftCorner<3,3>();
  t = pose.block<3,1>(0,3);
  inv_R = inv_pose.topLeftCorner<3,3>();
  inv_t = inv_pose.block<3,1>(0,3);

  if (have_im_pts && have_depth)
  {
    if (im_pts.size() != depth.size())
      throw std::runtime_error("[OptimizePointsMultiViewGD::InitData] Number of depth points != number of image points!");

    // init 3d points
    if (!have_points || points.size() != depth.size())
    {
      view_rays.resize(im_pts.size());
      points.resize(im_pts.size());
      for (unsigned i=0; i<im_pts.size(); i++)
      {
        cv::Point2f &pt = im_pts[i];
        vr = -Eigen::Vector3f((pt.x-C(0,2))/C(0,0),(pt.y-C(1,2))/C(1,1),1.).normalized();
        view_rays[i] = inv_R * vr;
        points[i] = inv_R*(-vr*depth[i]) + inv_t;
      }
    }
  }

  if (have_points)
  {
    // init image points
    if (!have_im_pts || im_pts.size()!=points.size())
    {
      im_pts.resize(points.size());
      for (unsigned i=0; i<points.size(); i++)
      {
        pt3 = R*points[i] + t;
        im_pts[i] = cv::Point2f(pt3[0]/pt3[2]*C(0,0)+C(0,2), pt3[1]/pt3[2]*C(1,1)+C(1,2));
      }
    }
    // init depth
    if (!have_depth || depth.size()!=points.size())
    {
      depth.resize(points.size());
      for (unsigned i=0; i<points.size(); i++)
      {
        depth[i] = (R*points[i]+t).norm();
      }
    }
  }

  // init view rays
  if (view_rays.size()!=points.size())
  {
    view_rays.resize(points.size());
    for (unsigned i=0; i<points.size(); i++)
    {
      view_rays[i] = -inv_R * ( (R*points[i]+t).normalized() );
    }
  }

  // init normals
  if (!have_normals || normals.size()!=view_rays.size())
  {
    normals.resize(view_rays.size());
    for (unsigned i=0; i<view_rays.size(); i++)
    {
      normals[i] = view_rays[i];
    }
  }

  // clar visibility
  if (!have_im_indices || im_indices.size()!=points.size())
  {
    im_indices.clear();
    im_indices.resize(points.size());
  }
}


/************************** PUBLIC *************************/

/**
 * clearFrames
 */
void OptimizePointsMultiViewGD::clearFrames()
{
  mv_views.resize(1);
  poses.resize(1);
}

/**
 * addFrame
 */
int OptimizePointsMultiViewGD::addFrame(const cv::Mat_<unsigned char> &image, const Eigen::Matrix4f &pose)
{
  mv_views.push_back(MVView());
  
  mv_views.back().idx = mv_views.size()-1;
  mv_views.back().image = image;
  poses.push_back(pose);

  return mv_views.size()-2;
}

/**
 * setImagePoints
 */
void OptimizePointsMultiViewGD::setImagePoints(const std::vector<cv::Point2f> &_im_pts) 
{
  im_pts=_im_pts;
  have_im_pts=true;
}

/**
 * setDepth
 */
void OptimizePointsMultiViewGD::setDepth(const std::vector<double> &_depth) 
{
  depth=_depth;
  have_depth=true;
}

/**
 * setPoints
 */
void OptimizePointsMultiViewGD::setPoints(const std::vector<Eigen::Vector3f> &_points) 
{ 
  points=_points;
  have_points=true;
}

/**
 * setNormals
 */
void OptimizePointsMultiViewGD::setNormals(const std::vector<Eigen::Vector3f> &_normals) 
{
  normals=_normals;
  have_normals=true;
}

/**
 * setImageIndices
 */
void OptimizePointsMultiViewGD::setImageIndices(const std::vector< std::vector<int> > &_idx) 
{
  im_indices=_idx;
  have_im_indices=true;
}

/**
 * initPoses
 */
void OptimizePointsMultiViewGD::initPoses(std::vector<MVView> &mv_views, std::vector<Eigen::Matrix4f> &poses)
{
  
  Eigen::Matrix4f delta_pose, inv_pose;

  invPose(poses[0], inv_pose);

  mv_views[0].R.setIdentity();
  mv_views[0].t.setZero();

  for (unsigned i=1; i<mv_views.size(); i++)
  {
    delta_pose =  poses[i]*inv_pose;
    getR(delta_pose,  mv_views[i].R);
    getT(delta_pose,  mv_views[i].t);        
  }
}

/**
 * optimize
 */
void OptimizePointsMultiViewGD::optimize(const cv::Mat_<unsigned char> &image, const Eigen::Matrix4f &pose)
{
  if (mv_views.size()<2)
    return;

  // init data
  poses[0] = pose;
  mv_views[0].image = image;

  initData(pose);
  initPoses(mv_views, poses);

  MVPoint mv_point;
  Eigen::Vector3d rot;
  double ddepth;

  Eigen::Vector3d vrx, n_new;
  Eigen::Vector3f t, inv_t;
  Eigen::Matrix3f R, inv_R;
  Eigen::Matrix4f inv_pose;

  invPose(pose, inv_pose);

  R = pose.topLeftCorner<3,3>();
  t = pose.block<3,1>(0,3);
  inv_R = inv_pose.topLeftCorner<3,3>();
  inv_t = inv_pose.block<3,1>(0,3);

  std::vector<unsigned char> patch0(param.patch_size.width*param.patch_size.height);
  std::vector<unsigned char> patch1(param.patch_size.width*param.patch_size.height);
  std::vector<MVView*> ptr_mv_views;
  std::vector<double> residuals;

  converged.clear();
  converged.resize(depth.size(), false);

  // optimize points
  for (unsigned j=0; j<im_pts.size(); j++)
  {
    const cv::Point2f &pt = im_pts[j];

    if (pt.x<param.patch_size.width || pt.y<param.patch_size.height || 
        pt.x>=image.cols-param.patch_size.width || 
        pt.y>=image.rows-param.patch_size.height)
      continue;

    mv_point = MVPoint(pt, R*points[j]+t, R*normals[j], R*view_rays[j]);

    // select views (test visibility, test error)
    ptr_mv_views.clear();
    ptr_mv_views.push_back(&mv_views[0]);
    if (have_im_indices)
    {
      for (unsigned i=0; i<im_indices[j].size(); i++)
      {
        MVView *ptr_mvx = &mv_views[im_indices[j][i]+1];
        vrx = ptr_mvx->R * mv_point.vr;
        if (mv_point.n.dot(vrx) > cos_view_ray_normal_offset)
          ptr_mv_views.push_back(ptr_mvx);
      }
    }
    else
    {
      for (unsigned i=1; i<mv_views.size(); i++)
      {
        MVView *ptr_mvx = &mv_views[i];
        vrx = ptr_mvx->R * mv_point.vr;
        if (mv_point.n.dot(vrx) > cos_view_ray_normal_offset)
          ptr_mv_views.push_back(ptr_mvx);
      }
    }

    MultiViewPatchError mpe_select = MultiViewPatchError( ptr_mv_views, mv_point, C, invC,
            param.patch_size.height, param.patch_size.width, &patch0[0], &patch1[0] );
    
    mpe_select(rot, ddepth, residuals);

    unsigned z=0;
    for (unsigned i=0; i<ptr_mv_views.size(); i++) {
      if (residuals[i] < param.max_residual) {
        ptr_mv_views[z] = ptr_mv_views[i];
        z++;
      }
    }

    if (z<2) continue;
    ptr_mv_views.resize(z);

    // create cost function
    MultiViewPatchError mpe = MultiViewPatchError( ptr_mv_views, mv_point, C, invC, 
            param.patch_size.height, param.patch_size.width, &patch0[0], &patch1[0] );

    // simple optimizer
    if(optimizeSequentialDim(mpe, rot, ddepth))
    {
      kp::AngleAxisRotatePoint(&rot[0], &mv_point.n[0], &n_new[0]);
      depth[j] -= ddepth;
      normals[j] = inv_R * n_new.cast<float>();
      points[j] = inv_R * (-mv_point.vr.cast<float>()*depth[j]) + inv_t;
      
      converged[j] = true;

      mpe(rot, ddepth, residuals);

      im_indices[j].clear();
      for (unsigned i=0; i<residuals.size(); i++) 
      {
        if (residuals[i] > param.max_residual_optimized) {
          converged[j] = false;
          break;
        } else im_indices[j].push_back(ptr_mv_views[i]->idx-1);
      }
    }
  }

  have_im_pts= have_depth= have_points= have_normals= have_im_indices = false;
}

/**
 * setCameraParameter
 */
void OptimizePointsMultiViewGD::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  cv::Mat_<double> intrinsic;

  if (!_dist_coeffs.empty())
    throw std::runtime_error("[OptimizePointsMultiViewGD::setCameraParameter] Image distortions are not supported!");
  if (_intrinsic.rows!=3 || _intrinsic.cols!=3)
    throw std::runtime_error("[OptimizePointsMultiViewGD::setCameraParameter] Invalid intrinsic! Need a 3x3 matrix");

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else intrinsic = _intrinsic;

  for (int v=0; v<3; v++)
    for (int u=0; u<3; u++)
      C(v,u) = intrinsic(v,u);

  invC = C.inverse();
}



} //-- THE END --






