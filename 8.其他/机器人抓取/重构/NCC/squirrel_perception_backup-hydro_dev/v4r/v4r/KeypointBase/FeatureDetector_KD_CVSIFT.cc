/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "FeatureDetector_KD_CVSIFT.hh"




namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_KD_CVSIFT::FeatureDetector_KD_CVSIFT(const Parameter &_p)
 : FeatureDetector(KD_CVSIFT), param(_p)
{ 
  sift = new cv::SIFT( _p.nfeatures, _p.nOctaveLayers, _p.contrastThreshold, 
                       _p.edgeThreshold, _p.sigma ); 
}

FeatureDetector_KD_CVSIFT::~FeatureDetector_KD_CVSIFT()
{
}

/***************************************************************************************/

/**
 * detect keypoints and descriptors
 * descriptors is a cv::Mat_<float>
 */
void FeatureDetector_KD_CVSIFT::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  sift->detect(im_gray, keys);

  sift->compute(im_gray, keys, descriptors);
}

/**
 * detect keypoints
 */
void FeatureDetector_KD_CVSIFT::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  sift->detect(im_gray, keys);
}

/**
 * extract descriptors
 * descriptors is a cv::Mat_<float>
 */
void FeatureDetector_KD_CVSIFT::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  sift->compute(im_gray, keys, descriptors);
}



}












