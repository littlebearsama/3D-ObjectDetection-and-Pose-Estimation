/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "FeatureDetector_KD_ORB.hh"




namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_KD_ORB::FeatureDetector_KD_ORB(const Parameter &_p)
 : FeatureDetector(KD_ORB), param(_p)
{ 
  //orb = new cv::ORB(10000, 1.2, 6, 13, 0, 2, cv::ORB::HARRIS_SCORE, 13); //31
  //orb = new cv::ORB(1000, 1.44, 2, 17, 0, 2, cv::ORB::HARRIS_SCORE, 17);
  orb = new cv::ORB(param.nfeatures, param.scaleFactor, param.nlevels, param.patchSize, 0, 2, cv::ORB::HARRIS_SCORE, param.patchSize);
}

FeatureDetector_KD_ORB::~FeatureDetector_KD_ORB()
{
}

/***************************************************************************************/

/**
 * detect
 * descriptors is a cv::Mat_<unsigned char>
 */
void FeatureDetector_KD_ORB::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  (*orb)(im_gray, cv::Mat(), keys, descriptors);
}

/**
 * detect
 */
void FeatureDetector_KD_ORB::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  (*orb)(im_gray, cv::Mat(), keys);
}

/**
 * detect
 */
void FeatureDetector_KD_ORB::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  (*orb)(im_gray, cv::Mat(), keys, descriptors, true);
}



}












