/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_HARRIS_HH
#define KP_FEATURE_DETECTOR_HARRIS_HH

#include <opencv2/features2d/features2d.hpp>
#include "FeatureDetector.hh"
#include "ComputeImGDescOrientations.hh"



namespace kp 
{

class FeatureDetector_K_HARRIS : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    int winSize;        // e.g. 32*32 window + 2px border = 34
    int maxCorners; 
    double qualityLevel; // 0.0001
    double minDistance;
    bool computeDesc;
    bool uprightDesc;
    ComputeImGDescOrientations::Parameter goParam;

    Parameter(int _winSize=34, int _maxCorners=5000, const double &_qualityLevel=0.0002, 
      const double &_minDistance=1.,  bool _computeDesc=true, bool _uprightDesc=false,
      const ComputeImGDescOrientations::Parameter &_goParam=ComputeImGDescOrientations::Parameter())
    : winSize(_winSize), maxCorners(_maxCorners), qualityLevel(_qualityLevel), 
      minDistance(_minDistance), computeDesc(_computeDesc), uprightDesc(_uprightDesc),
      goParam(_goParam) {}
  };

private:
  Parameter param;

  cv::Mat_<unsigned char> im_gray;  

  std::vector<cv::Point2f> pts;

  ComputeImGDescOrientations::Ptr imGOri;

public:
  FeatureDetector_K_HARRIS(const Parameter &_p=Parameter());
  ~FeatureDetector_K_HARRIS();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys); 

  typedef SmartPtr< ::kp::FeatureDetector_K_HARRIS> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_K_HARRIS const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

