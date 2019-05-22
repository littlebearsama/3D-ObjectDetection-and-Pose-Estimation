/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "FeatureDetector_KD_CVSURF.hh"
//#include <pcl/common/time.h>



namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_KD_CVSURF::FeatureDetector_KD_CVSURF(const Parameter &_p)
 : FeatureDetector(KD_CVSURF), param(_p)
{ 
  surf = new cv::SURF(_p.hessianThreshold,_p.octaves,_p.octaveLayers,_p.extended,_p.upright); 
}

FeatureDetector_KD_CVSURF::~FeatureDetector_KD_CVSURF()
{
}

/***************************************************************************************/

/**
 * detect keypoint and extract descriptors
 * descriptors is a cv::Mat_<float>
 */
void FeatureDetector_KD_CVSURF::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  surf->detect(im_gray, keys);

  surf->compute(im_gray, keys, descriptors);
}

/**
 * detect keypoints
 */
void FeatureDetector_KD_CVSURF::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  surf->detect(im_gray, keys);
}

/**
 * extract descriptors
 * descriptors is a cv::Mat_<float>
 */
void FeatureDetector_KD_CVSURF::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  surf->compute(im_gray, keys, descriptors);
}


}












