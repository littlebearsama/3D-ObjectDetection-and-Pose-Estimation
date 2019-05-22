/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_COMPUTE_GRADIENT_DESCRIPTORS_HH
#define KP_COMPUTE_GRADIENT_DESCRIPTORS_HH

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ImGradientDescriptor.hh"


namespace kp 
{

class ComputeImGradientDescriptors
{
public:
  class Parameter
  {
  public:
    int win_size;
    ImGradientDescriptor::Parameter ghParam;
    Parameter(int _win_size=34, 
      const ImGradientDescriptor::Parameter &_ghParam=ImGradientDescriptor::Parameter())
    : win_size(_win_size), ghParam(_ghParam) {}
  };

private:
  Parameter param;

  int h_win;

public:
 

  ComputeImGradientDescriptors(const Parameter &p=Parameter());
  ~ComputeImGradientDescriptors();

  void compute(const cv::Mat_<unsigned char> &image, const std::vector<cv::Point2f> &pts, 
        cv::Mat &descriptors);
  void compute(const cv::Mat_<unsigned char> &image, const std::vector<cv::KeyPoint> &keys, 
        cv::Mat &descriptors);
  //void compute(const cv::Mat_<unsigned char> &image, const std::vector<AffKeypoint> &keys, 
  //      cv::Mat &descriptors);




  typedef SmartPtr< ::kp::ComputeImGradientDescriptors> Ptr;
  typedef SmartPtr< ::kp::ComputeImGradientDescriptors const> ConstPtr;

};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

