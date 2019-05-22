/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_NORMALIZED_CROSS_CORRELATION_HPP
#define KP_NORMALIZED_CROSS_CORRELATION_HPP 

#include <stdexcept>
#include <opencv2/core/core.hpp>

namespace kp 
{

/** 
 * normalizedCrossCorrelation
 * of two patches with the same size
 */
inline float normalizedCrossCorrelation(const cv::Mat_<unsigned char> &im1, const cv::Mat_<unsigned char> &im2)
{
  if (im1.size()!=im2.size())
    throw std::runtime_error("[normalizedCrossCorrelation] Image size must be equal!");

  float dist=0, m1=0, m2=0, s1=0, s2=0;
  float size = im1.rows*im1.cols;

  if (size<2) 
    throw std::runtime_error("[normalizedCrossCorrelation] Image is empty!");

  for (int v=0; v<im1.rows; v++)
  for (int u=0; u<im1.cols; u++)
  {
    m1 += float(im1(v,u));
    m2 += float(im2(v,u));
  }
  m1 /= size;
  m2 /= size;

  for (int v=0; v<im1.rows; v++)
  for (int u=0; u<im1.cols; u++)
  {
    s1 += sqr(float(im1(v,u))-m1);
    s2 += sqr(float(im2(v,u))-m2);
  }
  s1 /= (size-1.);
  s2 /= (size-1.);

  for (int v=0; v<im1.rows; v++)
  for (int u=0; u<im1.cols; u++)
    dist += (float(im1(v,u))-m1)*(float(im2(v,u))-m2);

  dist /= size*sqrt(s1)*sqrt(s2);

  return dist;
}


} //--END--

#endif




