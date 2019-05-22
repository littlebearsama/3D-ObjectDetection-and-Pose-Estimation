/*
 * cam.cpp
 *
 *  Created on: May 30, 2012
 *      Author: jbalzer
 */


#include <math.h>
#include <iostream>
#include <fstream>
#include <string>

#include "cam.h"

using namespace std;
using namespace cv;

CCam::CCam() {

  m_size[0] = 640;
  m_size[1] = 480;
  m_f[0] = 500;
  m_f[1] = 500;
  m_c[0] = 319.5;
  m_c[1] = 239.5;

  for(size_t i=0;i<5;i++)
    m_k[i]=0;

  m_alpha = 0;

  m_F = Mat::eye(4,4,CV_32FC1);

  m_Finv = m_F.clone();

}

CCam::CCam(const CCam& cam) {

  m_size[0] = cam.m_size[0];
  m_size[1] = cam.m_size[1];
  m_f[0] = cam.m_f[0];
  m_f[1] = cam.m_f[1];
  m_c[0] = cam.m_c[0];
  m_c[1] = cam.m_c[1];

  for(size_t i=0;i<5;i++)
    m_k[i] = cam.m_k[i];

  m_alpha = cam.m_alpha;

  m_F = cam.m_F.clone();

  m_Finv = cam.m_Finv.clone();

}


CCam::CCam(const vector<size_t>& size, const vector<float>& f, const vector<float>& c, const float& alpha, const vector<float>& k, const Mat& F):
  m_alpha(alpha),
  m_F(F) {

  m_size[0] = size[0];
  m_size[1] = size[1];
  m_f[0] = f[0];
  m_f[1] =f[1];
  m_c[0] = c[0];
  m_c[1] = c[1];

  for(size_t i=0;i<5;i++)
    m_k[i]=k[i];

  m_Finv = m_F.clone().inv();

}



ostream& operator<< (ostream& os, const CCam& x) {

  os << "# dims" << endl;
  os << x.m_size[0] << " " << x.m_size[1] << endl;
  os << "# focal lengths" << endl;
  os << x.m_f[0] << " " << x.m_f[1] << endl;
  os << "# principle point" << endl;
  os << x.m_c[0] << " " << x.m_c[1] << endl;
  os << "# radial distortion coefficients" << endl;
  os << x.m_k[0] << " " << x.m_k[1] << " " << x.m_k[2] << " " << x.m_k[3] << " " << x.m_k[4] << endl;
  os << "# skew coefficient" << endl;
  os << x.m_alpha << endl;
  os << "# frame world -> cam" << endl;

  for(size_t i=0; i<4; i++) {

    for(size_t j=0; j<4; j++) {

      os << x.m_F.at<float>(i,j);

      if(j<3)
        os << " ";

    }

    if(i<3)
      os << endl;

  }

  return os;

}

istream& operator >> (istream& is, CCam& x) {

  string linebuffer;

  getline(is,linebuffer);

  is >> x.m_size[0];
  is >> x.m_size[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_f[0];
  is >> x.m_f[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_c[0];
  is >> x.m_c[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_k[0];
  is >> x.m_k[1];
  is >> x.m_k[2];
  is >> x.m_k[3];
  is >> x.m_k[4];
  is.get();

  getline(is,linebuffer);

  is >> x.m_alpha;
  is.get();

  getline(is,linebuffer);

  for(size_t i=0; i<4; i++) {

    for(size_t j=0; j<4; j++) {

      is >> x.m_F.at<float>(i,j);

    }

    if(i<3)
      is.get();

  }

  x.m_Finv = x.m_F.clone().inv();

  return is;

}



Vec2f CCam::Project(const Vec3f& x) const {

  // transform into camera coordinate system
  Vec3f xc = TransformTo(x);

  return ProjectLocal(xc);

}

Vec3f CCam::TransformTo(const cv::Vec3f& x) const {

  Vec3f xc;
  xc *= 0;

  for(size_t i=0; i<3; i++) {

    for(size_t j=0; j<3; j++) {

      xc[i] += m_F.at<float>(i,j)*x[j];

    }

  }

  xc[0] = xc[0] + m_F.at<float>(0,3);
  xc[1] = xc[1] + m_F.at<float>(1,3);
  xc[2] = xc[2] + m_F.at<float>(2,3);

  return xc;

}

Vec2f CCam::ProjectPinhole(const Vec3f& xc) const {

  // result
  Vec2f xp;

  if(xc[2]<0) {

    xp[0] = -1000;
    xp[1] = -1000;
    return xp;

  }

  // pinhole projection
  Vec2f xn;
  xn[0] = xc[0]/xc[2];
  xn[1] = xc[1]/xc[2];

  // transform to pixel coordinates
  xp[0] = m_f[0]*xn[0] + m_c[0];
  xp[1] = m_f[1]*xn[1] + m_c[1];

  return xp;
}


Vec2f CCam::ProjectLocal(const Vec3f& xc) const {

  // result
  Vec2f xp;

  if(xc[2]<0) {

    xp[0] = -1000;
    xp[1] = -1000;
    return xp;

  }

  // pinhole projection
  Vec2f xn;
  xn[0] = xc[0]/xc[2];
  xn[1] = xc[1]/xc[2];

  // radial distortion
  float r = cv::norm(xn);

  Vec2f dx;
  dx[0] = 2*m_k[2]*xn[0]*xn[1] + m_k[3]*(r*r + 2*xn[0]*xn[0]);
  dx[1] = m_k[2]*(r*r + 2*xn[1]*xn[1]) + 2*m_k[3]*xn[0]*xn[1];

  float fac = 1 + m_k[0]*r*r + m_k[1]*r*r*r*r + m_k[4]*r*r*r*r*r*r;
  Vec2f xd = xn*fac + dx;

  // transform to pixel coordinates
  xp[0] = m_f[0]*(xd[0] + m_alpha*xd[1]) + m_c[0];
  xp[1] = m_f[1]*xd[1] + m_c[1];

  return xp;

}

Vec3f CCam::UnProject(const Vec2i& u) const {

  Vec3f x, xc;

  xc = UnProjectLocal(u);

  x *= 0;

  for(size_t i=0; i<3; i++) {

    for(size_t j=0; j<3; j++) {

      x[i] += m_Finv.at<float>(i,j)*xc[j];

    }

  }

  x[0] = x[0] + m_Finv.at<float>(0,3);
  x[1] = x[1] + m_Finv.at<float>(1,3);
  x[2] = x[2] + m_Finv.at<float>(2,3);


  return x;

}

Vec3f CCam::UnProjectLocal(const Vec2i& u) const {

  /* CAVEAT: Distortion model is usually not invertible. Here we use
     * the forward model but have to make sure that the distortion
     * coefficients are estimated to fit it.
     */
  Vec2f xd;
  xd[0] = (1/m_f[0])*((float)u[0]-m_c[0]);
  xd[1] = (1/m_f[1])*((float)u[1]-m_c[1]);

  float r = cv::norm(xd);

  Vec2f dx;
  dx[0] = 2*m_k[2]*xd[0]*xd[1] + m_k[3]*(r*r + 2*xd[0]*xd[0]);
  dx[1] = m_k[2]*(r*r + 2*xd[1]*xd[1]) + 2*m_k[3]*xd[0]*xd[1];

  float fac = 1 + m_k[0]*r*r + m_k[1]*r*r*r*r + m_k[4]*r*r*r*r*r*r;
  Vec2f xn = xd*fac + dx;

  Vec3f xc;
  xc[0] = xn[0];
  xc[1] = xn[1];
  xc[2] = 1.0;

  return xc;

}

cv::Mat CCam::GetUndistorted(const cv::Mat& src) const
{
  cv::Mat F = cv::Mat::eye(3,3,CV_32FC1);;
  F.at<float>(0,0) = m_f[0];
  F.at<float>(1,1) = m_f[1];
  F.at<float>(0,2) = m_c[0];
  F.at<float>(1,2) = m_c[1];

  cv::Mat dist = cv::Mat::zeros(1,5,CV_32FC1);
  dist.at<float>(0,0) = m_k[0];
  dist.at<float>(0,1) = m_k[1];
  dist.at<float>(0,2) = m_k[2];
  dist.at<float>(0,3) = m_k[3];
  dist.at<float>(0,4) = m_k[4];

  cv::Mat undist;
  cv::undistort(src, undist, F, dist);
  return undist;
}

void CCam::GetIntrinsics(size_t size[2], float f[2], float c[2], float& alpha, float k[5]) const
{
  size[0] = m_size[0];
  size[1] = m_size[1];
  f[0] = m_f[0];
  f[1] = m_f[1];
  c[0] = m_c[0];
  c[1] = m_c[1];
  alpha = m_alpha;
  k[0] = m_k[0];
  k[1] = m_k[1];
  k[2] = m_k[2];
  k[3] = m_k[3];
  k[4] = m_k[4];
}

CDepthCam::CDepthCam():
  CCam(),
  m_d(),
  m_D(Mat::zeros(480,640,CV_32FC1)),
  m_a () {

  m_range[0] = 0.3;
  m_range[1] = 1.3;
  m_d[0] = 4;
  m_d[1] = -0.003;
  m_a[0] = 0;
  m_a[1] = 0;

}

CDepthCam::CDepthCam(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& range, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a):
  CCam(size,f,c,alpha,k,F),
  m_D(D) {

  m_range[0] = range[0];
  m_range[1] = range[1];
  m_d[0] = d[0];
  m_d[1] = d[1];
  m_a[0] = a[0];
  m_a[1] = a[1];

}

float CDepthCam::DisparityToDepth(size_t i, size_t j, float d) const
{

  float D = m_D.at<float>(i,j);
  float dc = d + D*exp(m_a[0]-m_a[1]*d);
  return 1.0/(m_d[0]+dc*m_d[1]);

//  return 1.0/(m_d[0]+d*m_d[1]);
}


ostream& operator << (ostream& os, const CDepthCam& x) {

  os << "# dims" << endl;
  os << x.m_size[0] << " " << x.m_size[1] << endl;
  os << "# focal lengths" << endl;
  os << x.m_f[0] << " " << x.m_f[1] << endl;
  os << "# principle point" << endl;
  os << x.m_c[0] << " " << x.m_c[1] << endl;
  os << "# radial distortion coefficients" << endl;
  os << x.m_k[0] << " " << x.m_k[1] << " " << x.m_k[2] << " " << x.m_k[3] << " " << x.m_k[4] << endl;
  os << "# skew coefficient" << endl;
  os << x.m_alpha << endl;
  os << "# range" << endl;
  os << x.m_range[0] << " " << x.m_range[1] << endl;
  os << "# disparity-to-depth conversion coefficients" << endl;
  os << x.m_d[0] << " " << x.m_d[1] << endl;
  os << "# radial depth error correction coefficients" << endl;
  os << x.m_a[0] << " " << x.m_a[1] << endl;
  os << "# frame world -> cam" << endl;

  for(size_t i=0; i<4; i++) {

    for(size_t j=0; j<4; j++) {

      os << x.m_F.at<float>(i,j);

      if(j<3)
        os << " ";

    }

    if(i<3)
      os << endl;

  }

  return os;

}

istream& operator >> (istream& is, CDepthCam& x) {

  string linebuffer;

  getline(is,linebuffer);

  is >> x.m_size[0];
  is >> x.m_size[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_f[0];
  is >> x.m_f[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_c[0];
  is >> x.m_c[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_k[0];
  is >> x.m_k[1];
  is >> x.m_k[2];
  is >> x.m_k[3];
  is >> x.m_k[4];
  is.get();

  getline(is,linebuffer);

  is >> x.m_alpha;
  is.get();


  getline(is,linebuffer);

  is >> x.m_range[0];
  is >> x.m_range[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_d[0];
  is >> x.m_d[1];
  is.get();

  getline(is,linebuffer);

  is >> x.m_a[0];
  is >> x.m_a[1];
  is.get();

  getline(is,linebuffer);

  for(size_t i=0; i<4; i++) {

    for(size_t j=0; j<4; j++) {

      is >> x.m_F.at<float>(i,j);

    }

    if(i<3)
      is.get();

  }

  x.m_Finv = x.m_F.clone().inv();

  return is;

}

void CDepthCam::GetRange(float range[2]) const
{
  range[0] = m_range[0];
  range[1] = m_range[2];
}
