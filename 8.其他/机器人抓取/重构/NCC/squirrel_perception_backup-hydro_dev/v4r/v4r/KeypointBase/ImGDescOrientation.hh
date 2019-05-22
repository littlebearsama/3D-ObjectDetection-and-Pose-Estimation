/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_IM_GDESC_ORIENTATION_HH
#define KP_IM_GDESC_ORIENTATION_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <stdexcept>
#include "v4r/KeypointTools/SmartPtr.hpp"

namespace kp 
{

class ImGDescOrientation
{
public:
  class Parameter
  {
  public:
    bool smooth;
    float sigma; //1.6
    Parameter(bool _smooth=true, float _sigma=1.6)
    : smooth(_smooth), sigma(_sigma) {}
  };

private:
  Parameter param;

  cv::Mat_<unsigned char> im_smooth;
  cv::Mat_<short> im_dx, im_dy;
  cv::Mat_<float> lt_gauss;

  std::vector<float> hist;

  void ComputeGradients(const cv::Mat_<unsigned char> &im);
  void ComputeAngle(const cv::Mat_<float> &weight, float &angle);
  void ComputeLTGaussCirc(const cv::Mat_<unsigned char> &im);


public:

  ImGDescOrientation(const Parameter &p=Parameter());
  ~ImGDescOrientation();

  void compute(const cv::Mat_<unsigned char> &im, float &angle);
  void compute(const cv::Mat_<unsigned char> &im, const cv::Mat_<float> &weight, float &angle);

  typedef SmartPtr< ::kp::ImGDescOrientation> Ptr;
  typedef SmartPtr< ::kp::ImGDescOrientation const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

