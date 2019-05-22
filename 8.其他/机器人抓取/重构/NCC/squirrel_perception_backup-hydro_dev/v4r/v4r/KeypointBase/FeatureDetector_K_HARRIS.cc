/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "FeatureDetector_K_HARRIS.hh"




namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_K_HARRIS::FeatureDetector_K_HARRIS(const Parameter &_p)
 : FeatureDetector(K_HARRIS), param(_p)
{ 
  imGOri.reset(new ComputeImGDescOrientations(param.goParam));
}

FeatureDetector_K_HARRIS::~FeatureDetector_K_HARRIS()
{
}

/***************************************************************************************/

/**
 * detect
 */
void FeatureDetector_K_HARRIS::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  cv::goodFeaturesToTrack(im_gray, pts, param.maxCorners, param.qualityLevel, param.minDistance, cv::Mat(), 3, true, 0.01 );

  keys.resize(pts.size());
  for (unsigned i=0; i<keys.size(); i++)
  {
    keys[i] = cv::KeyPoint(pts[i], param.winSize-2);
  }

  imGOri->compute(im_gray, keys);
}



}












