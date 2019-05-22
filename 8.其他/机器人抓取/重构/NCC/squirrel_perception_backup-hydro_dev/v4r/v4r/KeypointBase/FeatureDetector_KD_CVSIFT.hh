/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_CVSIFT_HH
#define KP_FEATURE_DETECTOR_CVSIFT_HH

#include "FeatureDetector.hh"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>


namespace kp 
{

class FeatureDetector_KD_CVSIFT : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    int nfeatures;
    int nOctaveLayers;
    double contrastThreshold;
    double edgeThreshold;
    double sigma;
    Parameter(int _nfeatures=0, int _nOctaveLayers=4, 
      double _contrastThreshold=0.02, double _edgeThreshold=5, double _sigma=1.6)
    : nfeatures(_nfeatures), nOctaveLayers(_nOctaveLayers), 
      contrastThreshold(_contrastThreshold), edgeThreshold(_edgeThreshold), sigma(_sigma) {}
  };

private:
  Parameter param;
  cv::Mat_<unsigned char> im_gray;  

  cv::Ptr<cv::SIFT> sift;

public:
  FeatureDetector_KD_CVSIFT(const Parameter &_p=Parameter());
  ~FeatureDetector_KD_CVSIFT();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys); 
  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 


  typedef SmartPtr< ::kp::FeatureDetector_KD_CVSIFT> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_KD_CVSIFT const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

