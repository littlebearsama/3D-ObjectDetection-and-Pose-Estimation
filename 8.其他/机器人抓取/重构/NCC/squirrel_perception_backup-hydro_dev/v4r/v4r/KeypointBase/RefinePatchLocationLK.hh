/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_REFINE_PATCH_LOCATION_LK_HH
#define KP_REFINE_PATCH_LOCATION_LK_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "v4r/KeypointTools/SmartPtr.hpp" 





namespace kp
{

/**
 * RefinePatchLocationLK
 */
class RefinePatchLocationLK
{
public:
  class Parameter
  {
  public:
    float step_factor;        // 2.???
    float min_determinant;
    float min_displacement;   //0.1
    int max_iterations;       //10
    int max_residual;
    Parameter(float _step_factor=10., float _min_determinant=0.01, float _min_displacement=0.1, 
      int _max_iterations=10, int _max_residual=15.)
    : step_factor(_step_factor), min_determinant(_min_determinant), min_displacement(_min_displacement),
      max_iterations(_max_iterations), max_residual(_max_residual) {}
  };

private:
  Parameter param;

  cv::Mat_<unsigned char> im_gray;
  cv::Mat_<float> im_dx, im_dy;

  cv::Mat_<float> patch_dx, patch_dy, diff, sum_dx, sum_dy;
  cv::Mat_<unsigned char> roi_patch;
  cv::Mat_<float> roi_dx, roi_dy;

  cv::Mat_<unsigned char> im_src, im_tgt;
  Eigen::Matrix4f pose_src, pose_tgt;
  
  void getIntensityDifference(const cv::Mat_<unsigned char> &im1, cv::Mat_<unsigned char> &im2, 
        const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &diff);
  void getGradientSum(const cv::Mat_<float> &dx1, const cv::Mat_<float> &dy1, const cv::Mat_<float> &dx2, 
        const cv::Mat_<float> &dy2, const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height, 
        cv::Mat_<float> &dx, cv::Mat_<float> &dy);
  void getGradientMatrix22(const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, float &gxx, float &gxy, float &gyy);
  void getErrorVector2(const cv::Mat_<float> &diff, const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, 
        cv::Point2f &err);
  bool solve(const cv::Point2f &err, float gxx, float gxy, float gyy, cv::Point2f &delta);


  inline float getInterpolated(const cv::Mat_<unsigned char> &im, const float &x, const float &y);
  inline float getInterpolated(const cv::Mat_<float> &im, const float &x, const float &y);


public:

  RefinePatchLocationLK(const Parameter &p=Parameter());
  ~RefinePatchLocationLK();

  bool optimize(const cv::Mat_<unsigned char> &patch, cv::Point2f &pt);
  void setImage(const cv::Mat_<unsigned char> &im);

  typedef SmartPtr< ::kp::RefinePatchLocationLK> Ptr;
  typedef SmartPtr< ::kp::RefinePatchLocationLK const> ConstPtr;
};




/*********************** INLINE METHODES **************************/

/** 
 * getInterpolated
 * get a bilinear interpolated pixel
 */
inline float RefinePatchLocationLK::getInterpolated(const cv::Mat_<unsigned char> &im, const float &x, const float &y)
{
  int xt = (int) x;
  int yt = (int) y;
  float ax = x - xt;
  float ay = y - yt;

  return ( (1.-ax) * (1.-ay) * im(yt,xt) + 
            ax     * (1.-ay) * im(yt,xt+1) +
           (1.-ax) *  ay     * im(yt+1,xt) +
            ax     *  ay     * im(yt+1,xt+1) );
}

/** 
 * getInterpolated
 * get a bilinear interpolated pixel
 */
inline float RefinePatchLocationLK::getInterpolated(const cv::Mat_<float> &im, const float &x, const float &y)
{
  int xt = (int) x;
  int yt = (int) y;
  float ax = x - xt;
  float ay = y - yt;

  return ( (1.-ax) * (1.-ay) * im(yt,xt) + 
            ax     * (1.-ay) * im(yt,xt+1) +
           (1.-ax) *  ay     * im(yt+1,xt) +
            ax     *  ay     * im(yt+1,xt+1) );
}



}

#endif

