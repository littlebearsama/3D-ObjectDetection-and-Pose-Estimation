/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "FeatureDetector_D_FREAK.hh"




namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_D_FREAK::FeatureDetector_D_FREAK(const Parameter &_p)
 : FeatureDetector(D_FREAK), param(_p)
{ 
  extractor = new cv::FREAK();  // (true, true, 22, 4, std::vector<int>())
}

FeatureDetector_D_FREAK::~FeatureDetector_D_FREAK()
{
}

/***************************************************************************************/

/**
 * detect
 * descriptors is a cv::Mat_<unsigned char>
 */
void FeatureDetector_D_FREAK::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  extractor->compute(im_gray, keys, descriptors);
}



}












