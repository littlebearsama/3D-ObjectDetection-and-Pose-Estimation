/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_SIFTGPU_HH
#define KP_FEATURE_DETECTOR_SIFTGPU_HH

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include "FeatureDetector.hh"
#include "PSiftGPU.hh"


namespace kp 
{

class FeatureDetector_KD_SIFTGPU : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    float distmax;         // absolute descriptor distance (e.g. = 0.6)
    float ratiomax;        // compare best match with second best (e.g. =0.8)
    int mutual_best_match; // compare forward/backward matches (1)
    bool computeRootSIFT;  // L1 norm and square root => euc dist = hellinger dist 
    Parameter(float d=FLT_MAX, float r=1., int m=0, 
      bool _computeRootSIFT=true)
    : distmax(d), ratiomax(r), mutual_best_match(m), 
      computeRootSIFT(_computeRootSIFT) {}
  };

private:
  PSiftGPU::Parameter param;
  cv::Mat_<unsigned char> im_gray;  

  cv::Ptr<kp::PSiftGPU> sift;

public:
  FeatureDetector_KD_SIFTGPU(const Parameter &_p=Parameter());
  ~FeatureDetector_KD_SIFTGPU();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys); 
  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 

  typedef SmartPtr< ::kp::FeatureDetector_KD_SIFTGPU> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_KD_SIFTGPU const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

