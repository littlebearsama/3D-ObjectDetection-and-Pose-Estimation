/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_DETECTOR_SURF_HH
#define KP_FEATURE_DETECTOR_SURF_HH

#include "FeatureDetector.hh"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>


namespace kp 
{

class FeatureDetector_KD_CVSURF : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    double hessianThreshold;
    int octaves;
    int octaveLayers;
    bool extended;
    bool upright;
    Parameter(double _hessianThreshold=400, int _octaves=3, 
      int _octaveLayers=4, bool _extended=true, bool _upright=false)
    : hessianThreshold(_hessianThreshold), octaves(_octaves),
      octaveLayers(_octaveLayers), extended(_extended), upright(_upright) {}
  };

private:
  Parameter param;
  cv::Mat_<unsigned char> im_gray;  

  cv::Ptr<cv::SURF> surf;

public:
  FeatureDetector_KD_CVSURF(const Parameter &_p=Parameter());
  ~FeatureDetector_KD_CVSURF();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys);
  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors);

  typedef SmartPtr< ::kp::FeatureDetector_KD_CVSURF> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_KD_CVSURF const> ConstPtr;
};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

