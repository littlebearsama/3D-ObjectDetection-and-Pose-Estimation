/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_FREAK_HH
#define KP_FEATURE_DETECTOR_FREAK_HH

#include "FeatureDetector.hh"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace kp 
{

class FeatureDetector_D_FREAK : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    Parameter() {} 
  };

private:
  Parameter param;
  cv::Mat_<unsigned char> im_gray;  

  cv::Ptr<cv::FREAK> extractor;

public:
  FeatureDetector_D_FREAK(const Parameter &_p=Parameter());
  ~FeatureDetector_D_FREAK();

  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 

  typedef SmartPtr< ::kp::FeatureDetector_D_FREAK> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_D_FREAK const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

