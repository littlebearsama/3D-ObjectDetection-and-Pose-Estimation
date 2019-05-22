/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_HH
#define KP_FEATURE_DETECTOR_HH

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "v4r/KeypointTools/SmartPtr.hpp"



namespace kp 
{


class FeatureDetector
{
public:
  enum Type
  {
    K_MSER,
    K_HARRIS,
    KD_CVSURF,
    KD_CVSIFT,
    KD_SIFTGPU,
    D_FREAK,
    KD_ORB,
    KD_FAST_IMGD,
    KD_PSURF,
    KD_MSER_IMGD,
    KD_HARRIS_IMGD,
    KD_PSURF_FREAK,
    KD_PSURF_IMGD,
    KD_FAST_PSURF,
    KD_FAST_SIFTGPU,
    KD_CVSURF_FREAK,
    KD_CVSURF_IMGD,
    KD_FAST_CVSURF,
    KD_SIFTGPU_IMGD,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

private:
  Type type;

public:
  FeatureDetector(Type _type=UNDEF) : type(_type) {}
  virtual ~FeatureDetector() {}

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) { 
    std::cout<<"[FeatureDetector::detect] Not implemented!]"<<std::endl; };

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys) {
    std::cout<<"[FeatureDetector::detect] Not implemented!]"<<std::endl; }

   virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors) {
    std::cout<<"[FeatureDetector::extract] Not implemented!]"<<std::endl; }
 

  Type getType() {return type;}

  typedef SmartPtr< ::kp::FeatureDetector> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector const> ConstPtr;
};

}

#endif

