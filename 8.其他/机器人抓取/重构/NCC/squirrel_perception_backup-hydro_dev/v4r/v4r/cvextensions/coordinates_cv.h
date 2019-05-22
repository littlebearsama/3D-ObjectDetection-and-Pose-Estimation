/**
 * @file coordinates_cv.h
 * @author Markus Bader
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef COORDINATES_CV_H
#define COORDINATES_CV_H

#include <opencv/cv.h>
namespace cv
{

template<typename _T>
static inline const void  cartToSphere(const _T &x, const _T &y, const _T &z, _T &r, _T &inclination, _T &azimuth){
  r = sqrt(x*x + y*y + z*z);
  if(r == 0){
    inclination = azimuth = 0;
  } else {
    inclination = acos(z / r);
    azimuth = atan2(y, x);
  }
};

template<typename _T>
static inline const void cartToSphere(const Point3_<_T> &src, Vec<_T, 3> &des){
  cartToSphere(src.x, src.y, src.z, des[0], des[1], des[2]);
};

template<typename _T>
static inline const cv::Vec<_T,3> cartToSphere(const Point3_<_T> &src){
  Vec<_T,3> des;
  cartToSphere(src.x, src.y, src.z, des[0], des[1], des[2]);
  return des;
};

template<typename _T>
static inline const void  sphereToCart( const _T &r, const _T &inclination, const _T &azimuth, _T &x, _T &y, _T &z){
            x = r * (sin(inclination) * cos (azimuth));
            y = r * (sin(inclination) * sin (azimuth));
            z = r * cos(inclination);
};

template<typename _T>
static inline const void  sphereToCart(const Vec<_T,3> &src, Point3_<_T> &des){
  sphereToCart(src[0], src[1], src[2], des.x, des.y, des.z);
};

template<typename _T>
static inline const cv::Point3_<_T>  sphereToCart(const Vec<_T,3> &src){
  Point3_<_T> des;
  sphereToCart(src[0], src[1], src[2], des.x, des.y, des.z);
  return des;
};
};
#endif //COORDINATES_CV_H
