/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "RefineProjectedPointLocationLK.hh"
#include "v4r/KeypointTools/invPose.hpp"
#include "v4r/KeypointTools/projectPointToImage.hpp"
#include "v4r/KeypointTools/warpPatchHomography.hpp"
#include "v4r/KeypointTools/Vector.hpp"
#include <opencv2/highgui/highgui.hpp>

#include <omp.h>


namespace kp 
{

using namespace std;


/********************** RefineProjectedPointLocationLK ************************
 * Constructor/Destructor
 */
RefineProjectedPointLocationLK::RefineProjectedPointLocationLK(const Parameter &p)
 : param(p)
{
}

RefineProjectedPointLocationLK::~RefineProjectedPointLocationLK()
{
}




/************************** PRIVATE ************************/

/**
 * getIntensityDifference
 */
void RefineProjectedPointLocationLK::getIntensityDifference(const cv::Mat_<unsigned char> &im1, cv::Mat_<unsigned char> &im2, const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &diff)
{
  diff = cv::Mat_<float>(height, width);
  int hw = width/2, hh = height/2;

  for (int v = -hh ; v <= hh ; v++)
  {
    for (int u = -hw ; u <= hw ; u++)  
    {
      diff(v+hh,u+hw) = getInterpolated(im1, pt1.x+u, pt1.y+v) - getInterpolated(im2, pt2.x+u, pt2.y+v);
    }
  }
}

/**
 * getPatchInterpolated
 */
void RefineProjectedPointLocationLK::getPatchInterpolated(const cv::Mat_<unsigned char> &image, const cv::Point2f &pt, cv::Mat_<unsigned char> &patch, int width, int height)
{
  patch = cv::Mat_<unsigned char>(height, width);
  int hw = width/2, hh = height/2;

  for (int v = -hh ; v <= hh ; v++)
  {
    for (int u = -hw ; u <= hw ; u++)
    {
      patch(v+hh,u+hw) = (unsigned char)getInterpolated(image, pt.x+u, pt.y+v);
    }
  }
}


/**
 * getGradientSum
 */
void RefineProjectedPointLocationLK::getGradientSum(const cv::Mat_<float> &dx1, const cv::Mat_<float> &dy1, const cv::Mat_<float> &dx2, const cv::Mat_<float> &dy2, const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &dx, cv::Mat_<float> &dy)
{
  dx = cv::Mat_<float>(height,width);
  dy = cv::Mat_<float>(height,width);

  int hw = width/2, hh = height/2;

  for (int v = -hh ; v <= hh ; v++)
  {
    for (int u = -hw ; u <= hw ; u++)  
    {
      dx(v+hh,u+hw) = getInterpolated(dx1,pt1.x+u,pt1.y+v) + getInterpolated(dx2,pt2.x+u,pt2.y+v);
      dy(v+hh,u+hw) = getInterpolated(dy1,pt1.x+u,pt1.y+v) + getInterpolated(dy2,pt2.x+u,pt2.y+v);
    }
  }
}

/**
 * getGradientMatrix22
 */
void RefineProjectedPointLocationLK::getGradientMatrix22(const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, float &gxx, float &gxy, float &gyy)
{
  float gx, gy;

  gxx = gxy = gyy = 0.;

  for (int v=0; v<dx.rows; v++)
  {
    for (int u=0; u<dx.cols; u++)
    {
      gx = dx(v,u);
      gy = dy(v,u);
      gxx += gx*gx;
      gxy += gx*gy;
      gyy += gy*gy;
    }
  }
}

/**
 * getErrorVector2
 */
void RefineProjectedPointLocationLK::getErrorVector2(const cv::Mat_<float> &diff, const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, cv::Point2f &err)
{
  float d;

  err = cv::Point2f(0.,0.);

  for (int v=0; v<diff.rows; v++)
  {
    for (int u=0; u<diff.cols; u++)
    {
      d = diff(v,u);
      err.x += d * dx(v,u);
      err.y += d * dy(v,u);
    }
  }

  err *= param.step_factor;
}

/** 
 * solve
 * [gxx gxy] [delta.x] = [err.x]
 * [gxy gyy] [delta.y] = [err.y]
 */
bool RefineProjectedPointLocationLK::solve(const cv::Point2f &err, float gxx, float gxy, float gyy, cv::Point2f &delta)
{
  float det = gxx*gyy - gxy*gxy;

  if (det < param.min_determinant)  return false;

  delta.x = (gyy*err.x - gxy*err.y)/det;
  delta.y = (gxx*err.y - gxy*err.x)/det;

  return true;
}


/************************** PUBLIC *************************/

/**
 * trackImagePoints
 * @param converged 1..converged, -1..out_of_bound, -2..small_determinant, -3..large_error
 */
void RefineProjectedPointLocationLK::refineImagePoints(const std::vector<Eigen::Vector3f> &pts, const std::vector<Eigen::Vector3f> &normals, std::vector<cv::Point2f> &im_pts_tgt, std::vector<int> &converged)
{
  if (im_tgt.rows==0 || im_tgt.cols==0 || im_src.rows==0 || im_src.cols==0)
    throw std::runtime_error("[RefineProjectedPointLocationLK::optimize] No data available!");

  cv::Point2f delta, err;
  cv::Mat_<unsigned char> patch;
  cv::Mat_<float> patch_dx, patch_dy, diff, sum_dx, sum_dy;
  cv::Mat_<unsigned char> roi_patch, patch1, patch2;
  cv::Mat_<float> roi_dx, roi_dy;
  float gxx, gxy, gyy;

  Eigen::Matrix<float,3,3,Eigen::RowMajor> H, T;
  Eigen::Vector3f n, pt3;
  double d;

  int hw = (param.patch_size.width-2)/2;
  int hh = (param.patch_size.height-2)/2;
  cv::Point2f pt_patch(hw,hh);

  delta_pose =  pose_src*inv_pose_tgt;
  delta_R = delta_pose.topLeftCorner<3,3>();
  delta_t = delta_pose.block<3,1>(0,3);

  bool have_dist = !tgt_dist_coeffs.empty();
  im_pts_tgt.resize(pts.size());
  converged.resize(pts.size());
  residuals.resize(pts.size());

  #pragma omp parallel for private(d, pt3, n, T, H, gxx, gxy, gyy, roi_dx, roi_dy, roi_patch, patch1, patch2, patch_dx, patch_dy, diff, sum_dx, sum_dy, patch, delta, err)
  for (unsigned i=0; i<pts.size(); i++)
  {
    T.setIdentity();
    patch = cv::Mat_<unsigned char>(param.patch_size);

    converged[i] = 1;

    pt3 = R_tgt*pts[i] + t_tgt;
    n = R_tgt*normals[i];

    cv::Point2f &pt_im = im_pts_tgt[i];

    if (have_dist)
      kp::projectPointToImage(&pt3[0], tgt_intrinsic.ptr<double>(), tgt_dist_coeffs.ptr<double>(), &pt_im.x);
    else kp::projectPointToImage(&pt3[0], tgt_intrinsic.ptr<double>(), &pt_im.x);

    T(0,2) = pt_im.x - (int)patch.cols/2;
    T(1,2) = pt_im.y - (int)patch.rows/2;
    d = n.transpose()*pt3;
    H = delta_R + 1./d*delta_t*n.transpose();
    H = src_C * H * tgt_C.inverse() * T;

    warpPatchHomography( (const unsigned char*)im_src.ptr(), im_src.rows, im_src.cols,
                         (float*)H.data(), (unsigned char*)patch.ptr(), patch.rows, patch.cols);

    cv::Sobel( patch, patch_dx, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
    cv::Sobel( patch, patch_dy, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

    roi_patch = patch(cv::Rect(1,1,patch.cols-2,patch.rows-2));
    roi_dx = patch_dx(cv::Rect(1,1,patch.cols-2,patch.rows-2));
    roi_dy = patch_dy(cv::Rect(1,1,patch.cols-2,patch.rows-2));

    int z=0;
    do  {
      if (  pt_im.x-hw < 0.0f || im_tgt.cols-(pt_im.x+hw) < 1.001 ||
            pt_im.y-hh < 0.0f || im_tgt.rows-(pt_im.y+hh) < 1.001 ) {
        converged[i] = -1;
        break;
      }

      getIntensityDifference(im_tgt, roi_patch, pt_im, pt_patch, roi_patch.cols, roi_patch.rows, diff);
      getGradientSum(im_tgt_dx, im_tgt_dy, roi_dx, roi_dy, pt_im, pt_patch, roi_patch.cols, roi_patch.rows, sum_dx, sum_dy);

      getGradientMatrix22(sum_dx, sum_dy, gxx, gxy, gyy);
      getErrorVector2(diff, sum_dx, sum_dy, err);
      err = -1*err;
      
      if (!solve(err, gxx, gxy, gyy, delta))
      {
        converged[i] = -2;
        break;
      }

      pt_im += delta;
      z++;
    }  while( (fabs(delta.x)>=param.min_displacement || fabs(delta.y)>=param.min_displacement) && 
               z < param.max_iterations);

    if (converged[i])
    {
      if (  pt_im.x-hw < 0.0f || im_tgt.cols-(pt_im.x+hw) < 1.001 ||
            pt_im.y-hh < 0.0f || im_tgt.rows-(pt_im.y+hh) < 1.001 ) 
      {
        converged[i] = -1;
      }
      else
      {
        if (!param.use_ncc)
        {
          getIntensityDifference(im_tgt, roi_patch, pt_im, pt_patch, roi_patch.cols, roi_patch.rows, diff);

          cv::Scalar sum = cv::sum(abs(diff));
          float residual = sum[0]/(roi_patch.rows*roi_patch.cols);
          residuals[i] = residual;

          if (residual > param.max_residual)
            converged[i] = -3;
        }
        else
        {
          getPatchInterpolated(im_tgt, pt_im, patch1, roi_patch.cols, roi_patch.rows);
          getPatchInterpolated(roi_patch, pt_patch, patch2, roi_patch.cols, roi_patch.rows);
          
          float ncc = distanceNCCb(patch1.ptr(), patch2.ptr(), roi_patch.rows*roi_patch.cols);
          residuals[i] = ncc;
          
          if (1.-ncc > param.ncc_residual)
            converged[i] = -3;
        }
      }
    }
  }
}

/**
 * setSourceImage
 */
void RefineProjectedPointLocationLK::setSourceImage(const cv::Mat_<unsigned char> &_im_src, const Eigen::Matrix4f &_pose_src)
{
  im_src = _im_src;
  pose_src = _pose_src;
}

/**
 * setTargetImage
 */
void RefineProjectedPointLocationLK::setTargetImage(const cv::Mat_<unsigned char> &_im_tgt, const Eigen::Matrix4f &_pose_tgt)
{
  im_tgt = _im_tgt;
  cv::Sobel( im_tgt, im_tgt_dx, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  cv::Sobel( im_tgt, im_tgt_dy, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

  pose_tgt = _pose_tgt;
  kp::invPose(pose_tgt, inv_pose_tgt);

  R_tgt = pose_tgt.topLeftCorner<3,3>();
  t_tgt = pose_tgt.block<3,1>(0,3);
}


/**
 * setSourceCameraParameter
 */
void RefineProjectedPointLocationLK::setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  src_dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(src_intrinsic, CV_64F);
  else src_intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    src_dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows && i<8; i++)
      src_dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }

  src_C = Eigen::Matrix3f::Identity();
  src_C(0,0) = src_intrinsic(0,0);
  src_C(1,1) = src_intrinsic(1,1);
  src_C(0,2) = src_intrinsic(0,2);
  src_C(1,2) = src_intrinsic(1,2);
}

/**
 * setTargetCameraParameter
 */
void RefineProjectedPointLocationLK::setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs)
{
  tgt_dist_coeffs = cv::Mat_<double>();

  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(tgt_intrinsic, CV_64F);
  else tgt_intrinsic = _intrinsic;

  if (!_dist_coeffs.empty())
  {
    tgt_dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows && i<8; i++)
      tgt_dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }

  tgt_C = Eigen::Matrix3f::Identity();
  tgt_C(0,0) = tgt_intrinsic(0,0);
  tgt_C(1,1) = tgt_intrinsic(1,1);
  tgt_C(0,2) = tgt_intrinsic(0,2);
  tgt_C(1,2) = tgt_intrinsic(1,2);
}


} //-- THE END --






