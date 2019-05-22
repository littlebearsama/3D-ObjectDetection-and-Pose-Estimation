/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_MULTIVIEW_PATCH_ERROR_HPP
#define KP_MULTIVIEW_PATCH_ERROR_HPP

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/KeypointTools/warpPatchHomography.hpp"
#include "v4r/KeypointTools/Vector.hpp"
#include "v4r/KeypointTools/rotation.h"


namespace kp
{



class MVView
{
public:
  int idx;
  Eigen::Matrix3d R;   //delta R with respect to view 0
  Eigen::Vector3d t;   //delta t with respect ot view 0
  cv::Mat_<unsigned char> image;
};

class MVPoint
{     
public:
  cv::Point2f im_pt; 
  Eigen::Vector3d pt;      // 3d point in view 0 coordinate system (target)
  Eigen::Vector3d n;       // normal
  Eigen::Vector3d vr;      // view ray
  MVPoint(){}
  MVPoint(const cv::Point2f _im_pt, const Eigen::Vector3f &_pt, 
    const Eigen::Vector3f &_n, const Eigen::Vector3f &_vr)
  : im_pt(_im_pt), pt(Eigen::Vector3d(_pt[0],_pt[1],_pt[2])), 
    n(Eigen::Vector3d(_n[0],_n[1],_n[2])), vr(Eigen::Vector3d(_vr[0],_vr[1],_vr[2])) {}
};  



/**
 * MultiViewPatchError
 */
class MultiViewPatchError
{
public:
//  bool dbg;

  const std::vector<MVView*> &mv_views;
  const MVPoint &mv_point;
  const Eigen::Matrix3d &C;
  const Eigen::Matrix3d &invC;
  
  int rows, cols;          // patch size

  unsigned char *patch0;   // temp memory to avoid reallocation (rows x cols)
  unsigned char *patch1;

  bool own_patch0, own_patch1;

  const static double VIEW_RAY_NORMAL_OFFSET = 0.087; //=cos(85*M_PI/180);

  /**
   * Constructor/ destructor
   */
  MultiViewPatchError(const std::vector<MVView*> &_mv_views, const MVPoint &_mv_point, 
    const Eigen::Matrix3d &_C, const Eigen::Matrix3d &_invC,
    int _patch_rows=21, int _patch_cols=21, 
    unsigned char *_patch0=0, unsigned char *_patch1=0)
  : /*dbg(false),*/ mv_views(_mv_views), mv_point(_mv_point), 
    C(_C), invC(_invC),
    rows(_patch_rows), cols(_patch_cols),
    patch0(_patch0), patch1(_patch1), own_patch0(false), own_patch1(false)
  {
    if (patch0==0)
    {
      patch0 = new unsigned char[rows*cols];
      own_patch0 = true;
    }
    if (patch1==0)
    {
      patch1 = new unsigned char[rows*cols];
      own_patch1 = true;
    }
  }

  ~MultiViewPatchError()
  {
    if (own_patch0) delete[] patch0;
    if (own_patch1) delete[] patch1;
  }

  /**
   * residual operator
   */
  bool operator()(const Eigen::Vector3d &rot,  // delta rotation
                  const double &depth,         // delta depth
                  double &residual) 
  {
    double d, err;
    Eigen::Matrix<double,3,3,Eigen::RowMajor> H, T(Eigen::Matrix<double,3,3,Eigen::RowMajor>::Identity());
    Eigen::Vector3d pt, n;

    T(0,2) = mv_point.im_pt.x - (int)cols/2;
    T(1,2) = mv_point.im_pt.y - (int)rows/2;

    kp::AngleAxisRotatePoint(&rot[0], &mv_point.n[0], &n[0]);

    if (n.dot(mv_point.vr) < VIEW_RAY_NORMAL_OFFSET)
    {
      residual = DBL_MAX;
      return false;
    }

    pt = mv_point.pt + depth*mv_point.vr;

    if (!warpPatchHomography( mv_views[0]->image.ptr(),
                              mv_views[0]->image.rows, mv_views[0]->image.cols, 
                              T.data(), patch0, rows, cols ) )
    {
      residual = DBL_MAX;
      return false;
    }
//cv::Mat_<unsigned char> im_patch0(rows, cols,patch0);
//cv::Mat_<unsigned char> im_patch1(rows, cols,patch1);
//cv::imshow("patch0",im_patch0);
//cv::imshow("image", mv_views[0]->image);
//cv::waitKey(0);

    int cnt=0;
    residual = 0;

    for (unsigned i=1; i<mv_views.size(); i++)
    {
      MVView *vd = mv_views[i];
  
      d = n.transpose()*pt;
      H = vd->R + 1./d*vd->t*n.transpose();
      H = C * H * invC * T;
    
      if (warpPatchHomography( vd->image.ptr(), vd->image.rows, vd->image.cols, 
                                H.data(), patch1, rows, cols) )
      {
//cv::imshow("patch1",im_patch1);
//cv::waitKey(0);
        //err = (double)distanceL1b(patch0, patch1, rows*cols);
        err = 1.-distanceNCCb(patch0, patch1, rows*cols);

//if (dbg) std::cout<<err<<" ";
        residual += ( err );
        cnt++;
      }
    }

    if (cnt>0) {
      residual /= (double)cnt;
      return true;
    }

    residual = DBL_MAX;
    return false;
  }

  /**********************************************
   * residual operator
   * return error for each view
   */
  bool operator()(const Eigen::Vector3d &rot,  // delta rotation
                  const double &depth,         // delta depth
                  std::vector<double> &residuals) 
  {
    double d;
    Eigen::Matrix<double,3,3,Eigen::RowMajor> H, T(Eigen::Matrix<double,3,3,Eigen::RowMajor>::Identity());
    Eigen::Vector3d pt, n;
    
    residuals.clear();

    T(0,2) = mv_point.im_pt.x - (int)cols/2;
    T(1,2) = mv_point.im_pt.y - (int)rows/2;

    kp::AngleAxisRotatePoint(&rot[0], &mv_point.n[0], &n[0]);

    if (n.dot(mv_point.vr) < VIEW_RAY_NORMAL_OFFSET)
      return false;

    pt = mv_point.pt + depth*mv_point.vr;

    if (!warpPatchHomography( mv_views[0]->image.ptr(),
                              mv_views[0]->image.rows, mv_views[0]->image.cols, 
                              T.data(), patch0, rows, cols ) )
      return false;

    int cnt=0;
    residuals.resize(mv_views.size(), DBL_MAX);
    residuals[0] = 0;

    for (unsigned i=1; i<mv_views.size(); i++)
    {
      MVView *vd = mv_views[i];
  
      d = n.transpose()*pt;
      H = vd->R + 1./d*vd->t*n.transpose();
      H = C * H * invC * T;
    
      if (warpPatchHomography( vd->image.ptr(), vd->image.rows, vd->image.cols, 
                                H.data(), patch1, rows, cols) )
      {
        residuals[i] = 1.-distanceNCCb(patch0, patch1, rows*cols);
        cnt++;
      }
    }

    if (cnt>0) 
      return true;

    return false;
  }

};


}



#endif

