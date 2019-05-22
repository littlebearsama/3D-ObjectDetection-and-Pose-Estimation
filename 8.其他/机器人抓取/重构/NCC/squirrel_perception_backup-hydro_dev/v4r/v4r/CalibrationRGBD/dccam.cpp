/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer, Thomas Mörwald
//
// All rights reserved.
//
// This file is part of the R4R library.
//
// The R4R library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The R4R library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the R4R library. If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////*/

#include "dccam.h"

using namespace cv;

CDepthColorCam::CDepthColorCam() :
  m_rgb_cam(),
  m_depth_cam()
{

}

CDepthColorCam::CDepthColorCam(CCam rgb, CDepthCam depth):
  m_rgb_cam(rgb),
  m_depth_cam(depth)
{

}

void CDepthColorCam::ConfigureRGB(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F) {

  m_rgb_cam.m_size[0] = size[0];
  m_rgb_cam.m_size[1] = size[1];
  m_rgb_cam.m_f[0] = f[0];
  m_rgb_cam.m_f[1] = f[1];
  m_rgb_cam.m_c[0] = c[0];
  m_rgb_cam.m_c[1] = c[1];
  m_rgb_cam.m_alpha = alpha;

  for(size_t i=0;i<5;i++)
    m_rgb_cam.m_k[i]=k[i];

  m_rgb_cam.m_F = F;
  m_rgb_cam.m_Finv = F.inv();

}

void CDepthColorCam::ConfigureDepth(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& range,const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a) {

  m_depth_cam.m_size[0] = size[0];
  m_depth_cam.m_size[1] = size[1];
  m_depth_cam.m_f[0] = f[0];
  m_depth_cam.m_f[1] = f[1];
  m_depth_cam.m_c[0] = c[0];
  m_depth_cam.m_c[1] = c[1];
  m_depth_cam.m_alpha = alpha;

  for(size_t i=0;i<5;i++)
    m_depth_cam.m_k[i]=k[i];

  m_depth_cam.m_F = F;
  m_depth_cam.m_Finv = F.inv();
  m_depth_cam.m_range[0] = range[0];
  m_depth_cam.m_range[1] = range[1];
  m_depth_cam.m_d[0] = d[0];
  m_depth_cam.m_d[1] = d[1];
  m_depth_cam.m_D = D;
  m_depth_cam.m_a[0] = a[0];
  m_depth_cam.m_a[1] = a[1];

}

Point3f CDepthColorCam::GetPoint(const size_t& i, const size_t& j, const float &d) const {

  //  float d = (float)disp.at<unsigned short>(i,j);

//  float z = 1.0f/(m_depth_cam.m_d[0]+d*m_depth_cam.m_d[1]);
  float z = m_depth_cam.DisparityToDepth(i,j,d);

  Point2i u(j,i);
  Vec3f xc = m_depth_cam.UnProjectLocal(u);

//  Point3f xd;
//  xd.x = (1.0f/m_depth_cam.m_f[0])*((float)j-m_depth_cam.m_c[0]);
//  xd.y = (1.0f/m_depth_cam.m_f[1])*((float)i-m_depth_cam.m_c[1]);
//  xd.z = 1.0;

  return xc * z;
}

Vec3f CDepthColorCam::GetNormal(size_t i, size_t j, const Mat& disp) const {

  // FIXME: this is slow (convert disparity to depth only once per pixel)
  Vec3f result;
  result *= 0;

  if(i==0 || j==0 || i==disp.rows-1 || j==disp.cols-1)
    return result;

  float z[5];
  z[0] = m_depth_cam.DisparityToDepth(i-1,j,(float)disp.at<unsigned short>(i-1,j));
  z[1] = m_depth_cam.DisparityToDepth(i+1,j,(float)disp.at<unsigned short>(i+1,j));
  z[2] = m_depth_cam.DisparityToDepth(i,j-1,(float)disp.at<unsigned short>(i,j-1));
  z[3] = m_depth_cam.DisparityToDepth(i,j+1,(float)disp.at<unsigned short>(i,j+1));
  z[4] = m_depth_cam.DisparityToDepth(i,j+1,(float)disp.at<unsigned short>(i,j));

  if(z[0]>0 && z[1]>0 && z[2]>0 && z[3]>0 && z[4]>0) {

    result[0] = 0.5*(z[3]-z[2])*(m_depth_cam.m_f[0]/z[4]);   // -dzdx
    result[1] = 0.5*(z[1]-z[0])*(m_depth_cam.m_f[1]/z[4]);   // -dzdy
    result[2] = -1;

  }

  float nn = cv::norm(result);

  if(nn)
    result /= nn;

  return result;

}



Vec3b CDepthColorCam::GetColor(cv::Point3f x, const cv::Mat& rgb) const {

  // project it to rgb image plane
  Vec2f uc = m_rgb_cam.Project(x);

  // round
  int irgb = (int)floor(uc[1]+0.5);
  int jrgb = (int)floor(uc[0]+0.5);

  Vec3b result;
  if(irgb<0 || irgb>=rgb.cols || jrgb<0 || jrgb>=rgb.cols) {

    result *= 0;
    return result;

  }

  // interpolate
  //    float vd = uc[1] - irgb;
  //    float ud = uc[0] - jrgb;

  //    Vec3f I00, I01, I10, I11, I0, I1;
  //    I00 = (Vec3f)rgb.at<Vec3b>(irgb,jrgb);
  //    I01 = (Vec3f)rgb.at<Vec3b>(irgb,jrgb+1);
  //    I10 = (Vec3f)rgb.at<Vec3b>(irgb+1,jrgb);
  //    I11 = (Vec3f)rgb.at<Vec3b>(irgb+1,jrgb+1);
  //    I0 = I00*(1-ud) + I01*ud;
  //    I1 = I10*(1-ud) + I11*ud;

  result = rgb.at<Vec3b>(irgb,jrgb);
  //result = (Vec3b)(I0*(1-vd) + I1*vd);

  return result;

}



Mat CDepthColorCam::WarpRGBToDepth(const cv::Mat& disp, const cv::Mat& rgb) {

  Mat result = Mat(disp.rows,disp.cols,CV_8UC3);

  for(size_t i=0; i<disp.rows; i++) {

    for(size_t j=0; j<disp.cols; j++) {

      float d = disp.at<unsigned short>(i,j);
      float z = m_depth_cam.DisparityToDepth(i,j,d);

      Point2i u(j,i);

      Vec3f xc = m_depth_cam.UnProjectLocal(u);

      Point3f x = xc*z;

      result.at<Vec3b>(i,j) = GetColor(x,rgb);

    }

  }

  return result;

}

cv::Mat CDepthColorCam::WarpDepthToRGB(const cv::Mat& disp, const cv::Mat& rgb) {

  Mat result = Mat(rgb.rows,rgb.cols,CV_32FC1);
  result *= 0;

  for(size_t i=0; i<disp.rows; i++) {

    for(size_t j=0; j<disp.cols; j++) {

      float d = disp.at<unsigned short>(i,j);
      float z = m_depth_cam.DisparityToDepth(i,j,d);

      Point2i u(j,i);

      Vec3f xc = m_depth_cam.UnProjectLocal(u);
      xc = xc*z;

      // transform to rgb cam coordinates
      Vec3f xcr = m_rgb_cam.TransformTo(xc);

      // project and round to nearest point
      Vec2f uc = m_rgb_cam.ProjectLocal(xcr);
      int irgb = (int)floor(uc[1]+0.5);
      int jrgb = (int)floor(uc[0]+0.5);

      if(irgb>=0 && irgb<m_rgb_cam.m_size[1] && jrgb>=0 && jrgb<m_rgb_cam.m_size[0] && z>m_depth_cam.m_range[0] && z<m_depth_cam.m_range[1])
        result.at<float>(irgb,jrgb) = xcr[2];

    }

  }

  return result;
}

cv::Mat CDepthColorCam::WarpDisparityToRGBPointsUndistorted(const cv::Mat& disp, const cv::Mat& rgb) const {

  Mat result = Mat(rgb.rows,rgb.cols,CV_32FC3);
  result *= 0;

  for(size_t i=0; i<disp.rows; i++) {

    for(size_t j=0; j<disp.cols; j++) {

      float d = disp.at<unsigned short>(i,j);
      float z = m_depth_cam.DisparityToDepth(i,j,d);

      Point2i u(j,i);
      Vec3f xc = m_depth_cam.UnProjectLocal(u);
      xc = xc*z;

      // transform to rgb cam coordinates
      Vec3f xcr = m_rgb_cam.TransformTo(xc);

      // project and round to nearest point
      Vec2f uc = m_rgb_cam.ProjectPinhole(xcr);
      int irgb = (int)floor(uc[1]+0.5);
      int jrgb = (int)floor(uc[0]+0.5);

      if(irgb>=0 && irgb<m_rgb_cam.m_size[1] && jrgb>=0 && jrgb<m_rgb_cam.m_size[0] && z>m_depth_cam.m_range[0] && z<m_depth_cam.m_range[1])
        result.at<Point3f>(irgb,jrgb) = Point3f(xcr);

    }

  }

  return result;
}

void CDepthColorCam::DisparityToPoints(const Mat& disp, Mat &result)
{
  result = Mat(disp.rows,disp.cols,CV_32FC3);

  for(size_t i=0; i<disp.rows; i++)
    for(size_t j=0; j<disp.cols; j++)
      result.at<Point3f>(i,j) = GetPoint(i,j,(float)disp.at<unsigned char>(i,j));
}

float CDepthColorCam::DisparityToDepth(int d) {

  return 1.0/(m_depth_cam.m_d[0]+d*m_depth_cam.m_d[1]);

}

void CDepthColorCam::GetDisparityRange(size_t& min, size_t& max) {

  min = (size_t)((1/m_depth_cam.m_range[1]-m_depth_cam.m_d[0])/m_depth_cam.m_d[1]);
  max = (size_t)((1/m_depth_cam.m_range[0]-m_depth_cam.m_d[0])/m_depth_cam.m_d[1]);

}

std::ostream& operator << (std::ostream& os, const CDepthColorCam& x) {

  os << x.m_rgb_cam << std::endl;
  os << x.m_depth_cam;

  return os;

}

std::istream& operator >> (std::istream& is, CDepthColorCam& x) {

  is >> x.m_rgb_cam;
  is.get();
  is >> x.m_depth_cam;

  return is;

}

void CDepthColorCam::set_default_params_asus_pro_live()
{
  // get params
  vector<size_t> srgb, sd;
  vector<float> frgb, fd, crgb, cd, krgb, kd, range, dd, da;
  srgb.push_back(640);
  srgb.push_back(480);
  frgb.push_back(536.96);
  frgb.push_back(536.06);
  crgb.push_back(312.29);
  crgb.push_back(242.41);
  krgb.push_back(0.0554);
  krgb.push_back(-0.2035);
  krgb.push_back(-0.0006);
  krgb.push_back(-0.0004);
  krgb.push_back(0.1593);

  Mat F = Mat::eye(4,4,CV_32FC1);
  F.at<float>(0,0) = 0.99996;
  F.at<float>(0,1) = 0.00813;
  F.at<float>(0,2) = 0.00443;
  F.at<float>(0,3) = -0.02680;
  F.at<float>(1,0) = -0.00808;
  F.at<float>(1,1) = 0.99991;
  F.at<float>(1,2) = -0.01111;
  F.at<float>(1,3) = -0.00095;
  F.at<float>(2,0) = -0.00452;
  F.at<float>(2,1) = 0.01107;
  F.at<float>(2,2) = 0.99993;
  F.at<float>(2,3) = -0.00350;

  m_rgb_cam = CCam(srgb,frgb,crgb,0,krgb,F);

  sd.push_back(640);
  sd.push_back(480);
  fd.push_back(579.28);
  fd.push_back(576.95);
  cd.push_back(318.24);
  cd.push_back(234.93);
  kd.push_back(0);
  kd.push_back(0);
  kd.push_back(0);
  kd.push_back(0);
  kd.push_back(0);
  range.push_back(0.4);
  range.push_back(3.8);
  dd.push_back(3.14);
  dd.push_back(-0.002890);
  da.push_back(2.1830);
  da.push_back(0.0043);

  cv::Mat D(Mat::zeros(480,640,CV_32FC1));
  m_depth_cam = CDepthCam(sd,fd,cd,0,vector<float>(5,0),Mat::eye(4,4,CV_32FC1),range,dd,D,da);
}

void CDepthColorCam::set_default_params_primesense()
{
  // get params
  vector<size_t> srgb, sd;
  vector<float> frgb, fd, crgb, cd, krgb, kd, range, dd, da;
  srgb.push_back(640);
  srgb.push_back(480);
  frgb.push_back(540.85);
  frgb.push_back(540.72);
  crgb.push_back(320.66);
  crgb.push_back(239.82);
  krgb.push_back(0.0749);
  krgb.push_back(-0.3184);
  krgb.push_back(0.0003);
  krgb.push_back(0.0001);
  krgb.push_back(0.3475);

  Mat F = Mat::eye(4,4,CV_32FC1);
  F.at<float>(0,0) = 0.99996;
  F.at<float>(0,1) = -0.00886;
  F.at<float>(0,2) = -0.00334;
  F.at<float>(0,3) = -0.02543;

  F.at<float>(1,0) = 0.00881;
  F.at<float>(1,1) = 0.99989;
  F.at<float>(1,2) = -0.01228;
  F.at<float>(1,3) = -0.00682;

  F.at<float>(2,0) = 0.00345;
  F.at<float>(2,1) = 0.01225;
  F.at<float>(2,2) = 0.99992;
  F.at<float>(2,3) = 0.01095;

  m_rgb_cam = CCam(srgb,frgb,crgb,0,krgb,F);

  sd.push_back(640);
  sd.push_back(480);
  fd.push_back(563.38);
  fd.push_back(564.55);
  cd.push_back(320.71);
  cd.push_back(228.53);
  kd.push_back(0);
  kd.push_back(0);
  kd.push_back(0);
  kd.push_back(0);
  kd.push_back(0);
  range.push_back(0.4);
  range.push_back(3.8);
  dd.push_back(4.11);
  dd.push_back(-0.003018);
  da.push_back(-3.5645);
  da.push_back(-0.0035);

  cv::Mat D(Mat::zeros(480,640,CV_32FC1));
  m_depth_cam = CDepthCam(sd,fd,cd,0,vector<float>(5,0),Mat::eye(4,4,CV_32FC1),range,dd,D,da);
}
