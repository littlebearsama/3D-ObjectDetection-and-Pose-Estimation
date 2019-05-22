/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_IM_GRADIENT_DESCRIPTOR_HH
#define KP_IM_GRADIENT_DESCRIPTOR_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Dense>
#include <stdexcept>
#include "v4r/KeypointTools/SmartPtr.hpp"

namespace kp 
{

class ImGradientDescriptor
{
public:
  class Parameter
  {
  public:
    bool smooth;
    float sigma; //1.6
    float thrCutDesc;
    bool gauss_lin;
    bool computeRootGD;  // L1 norm and square root => euc dist = hellinger dist
    bool normalize;
    Parameter(bool _smooth=true, float _sigma=1.6, float _thrCutDesc=.2, bool _gauss_lin=false,
      bool _computeRootGD=true, bool _normalize=true)
    : smooth(_smooth), sigma(_sigma), thrCutDesc(_thrCutDesc), gauss_lin(_gauss_lin), 
      computeRootGD(_computeRootGD), normalize(_normalize) {}
  };

private:
  Parameter param;

  cv::Mat_<unsigned char> im_smooth;
  cv::Mat_<short> im_dx, im_dy;
  cv::Mat_<float> lt_gauss;

  void ComputeGradients(const cv::Mat_<unsigned char> &im);
  void ComputeDescriptor(std::vector<float> &desc, const cv::Mat_<float> &weight);
  void ComputeDescriptorInterpolate(std::vector<float> &desc, const cv::Mat_<float> &weight);
  void ComputeLTGauss(const cv::Mat_<unsigned char> &im);
  void ComputeLTGaussCirc(const cv::Mat_<unsigned char> &im);
  void ComputeLTGaussLin(const cv::Mat_<unsigned char> &im);
  void Normalize(std::vector<float> &desc);
  void Cut(std::vector<float> &desc);

  inline int sign(const float &v);


public:

  ImGradientDescriptor(const Parameter &p=Parameter());
  ~ImGradientDescriptor();

  void compute(const cv::Mat_<unsigned char> &im, std::vector<float> &desc);
  void compute(const cv::Mat_<unsigned char> &im,const cv::Mat_<float> &weight, std::vector<float> &desc);

  typedef SmartPtr< ::kp::ImGradientDescriptor> Ptr;
  typedef SmartPtr< ::kp::ImGradientDescriptor const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

inline int ImGradientDescriptor::sign(const float &v)
{
  if (v<0.) return -1;
  return +1;
}


} //--END--

#endif

