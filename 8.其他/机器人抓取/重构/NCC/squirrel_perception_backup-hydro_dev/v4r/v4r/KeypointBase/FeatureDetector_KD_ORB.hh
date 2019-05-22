/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_ORB_HH
#define KP_FEATURE_DETECTOR_ORB_HH

#include "FeatureDetector.hh"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>


namespace kp 
{

class FeatureDetector_KD_ORB : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    int nfeatures;
    float scaleFactor;
    int nlevels;
    int patchSize;
    Parameter(int _nfeatures=1000, float _scaleFactor=1.44, int _nlevels=2, int _patchSize=17)
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels), patchSize(_patchSize) {}
  };

private:
  Parameter param;
  cv::Mat_<unsigned char> im_gray;  

  cv::Ptr<cv::ORB> orb;

public:
  FeatureDetector_KD_ORB(const Parameter &_p=Parameter());
  ~FeatureDetector_KD_ORB();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys); 
  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 


  typedef SmartPtr< ::kp::FeatureDetector_KD_ORB> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_KD_ORB const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

