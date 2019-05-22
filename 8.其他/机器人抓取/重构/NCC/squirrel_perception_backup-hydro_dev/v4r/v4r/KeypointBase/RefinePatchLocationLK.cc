/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "RefinePatchLocationLK.hh"
//#include <opencv2/highgui/highgui.hpp>
//#include "v4r/CameraTrackerPnP/ScopeTime.hpp"


namespace kp 
{

using namespace std;


/********************** RefinePatchLocationLK ************************
 * Constructor/Destructor
 */
RefinePatchLocationLK::RefinePatchLocationLK(const Parameter &p)
 : param(p)
{
}

RefinePatchLocationLK::~RefinePatchLocationLK()
{
}




/************************** PRIVATE ************************/

/**
 * getIntensityDifference
 */
void RefinePatchLocationLK::getIntensityDifference(const cv::Mat_<unsigned char> &im1, cv::Mat_<unsigned char> &im2, const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &diff)
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
 * getGradientSum
 */
void RefinePatchLocationLK::getGradientSum(const cv::Mat_<float> &dx1, const cv::Mat_<float> &dy1, const cv::Mat_<float> &dx2, const cv::Mat_<float> &dy2, const cv::Point2f &pt1, const cv::Point2f &pt2, int width, int height, cv::Mat_<float> &dx, cv::Mat_<float> &dy)
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
void RefinePatchLocationLK::getGradientMatrix22(const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, float &gxx, float &gxy, float &gyy)
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
void RefinePatchLocationLK::getErrorVector2(const cv::Mat_<float> &diff, const cv::Mat_<float> &dx, const cv::Mat_<float> &dy, cv::Point2f &err)
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
bool RefinePatchLocationLK::solve(const cv::Point2f &err, float gxx, float gxy, float gyy, cv::Point2f &delta)
{
  float det = gxx*gyy - gxy*gxy;

  if (det < param.min_determinant)  return false;

  delta.x = (gyy*err.x - gxy*err.y)/det;
  delta.y = (gxx*err.y - gxy*err.x)/det;

  return true;
}


/************************** PUBLIC *************************/

/**
 * optimize
 */
bool RefinePatchLocationLK::optimize(const cv::Mat_<unsigned char> &patch, cv::Point2f &pt)
{
  if (im_gray.rows<=patch.rows || im_gray.cols<=patch.cols)
    throw std::runtime_error("[RefinePatchLocationLK::optimize] No data available!");

  int z=0;
  cv::Point2f delta, err;
  
  cv::Sobel( patch, patch_dx, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  cv::Sobel( patch, patch_dy, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );

  roi_patch = patch(cv::Rect(1,1,patch.cols-2,patch.rows-2));
  roi_dx = patch_dx(cv::Rect(1,1,patch.cols-2,patch.rows-2));
  roi_dy = patch_dy(cv::Rect(1,1,patch.cols-2,patch.rows-2));

  int hw = roi_dx.cols/2;
  int hh = roi_dx.rows/2;
  cv::Point2f pt_patch(hw,hh);
  float gxx, gxy, gyy;

//getIntensityDifference(im_gray, roi_patch, pt, pt_patch, roi_patch.cols, roi_patch.rows, diff);
//cv::Scalar sum1 = cv::sum(abs(diff));
//cout<<"residual="<<sum1[0]/(roi_patch.rows*roi_patch.cols)<<endl;

  do  {
    if (  pt.x-hw < 0.0f || im_gray.cols-(pt.x+hw) < 1.001 ||
          pt.y-hh < 0.0f || im_gray.rows-(pt.y+hh) < 1.001 ) {
      return false;
    }
//cout<<"iter="<<z<<", ";

    getIntensityDifference(im_gray, roi_patch, pt, pt_patch, roi_patch.cols, roi_patch.rows, diff);
    getGradientSum(im_dx, im_dy, roi_dx, roi_dy, pt, pt_patch, roi_patch.cols, roi_patch.rows, sum_dx, sum_dy);

    getGradientMatrix22(sum_dx, sum_dy, gxx, gxy, gyy);
    getErrorVector2(diff, sum_dx, sum_dy, err);
    err = -1*err;
    
    if (!solve(err, gxx, gxy, gyy, delta))
    {
//cout<<"delta="<<delta<<"f"<<endl;
      return false;
    }
//cout<<"delta="<<delta<<"t"<<endl;

    pt += delta;
    z++;
  }  while( (fabs(delta.x)>=param.min_displacement || fabs(delta.y)>=param.min_displacement) && 
             z < param.max_iterations);

  if (  pt.x-hw < 0.0f || im_gray.cols-(pt.x+hw) < 1.001 ||
        pt.y-hh < 0.0f || im_gray.rows-(pt.y+hh) < 1.001 ) {
    return false;
  }

  getIntensityDifference(im_gray, roi_patch, pt, pt_patch, roi_patch.cols, roi_patch.rows, diff);

  cv::Scalar sum = cv::sum(abs(diff));
//cout<<"new-residual="<<sum[0]/(roi_patch.rows*roi_patch.cols)<<endl;


  if (sum[0]/(roi_patch.rows*roi_patch.cols) > param.max_residual)
    return false;


  return true;
}

/**
 * setImage
 * set the target image
 */
void RefinePatchLocationLK::setImage(const cv::Mat_<unsigned char> &im)
{
  im_gray = im;
  cv::Sobel( im_gray, im_dx, CV_32F, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  cv::Sobel( im_gray, im_dy, CV_32F, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
}

} //-- THE END --






