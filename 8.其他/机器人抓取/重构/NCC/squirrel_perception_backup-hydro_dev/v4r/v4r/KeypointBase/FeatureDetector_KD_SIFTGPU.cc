/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "FeatureDetector_KD_SIFTGPU.hh"




namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
FeatureDetector_KD_SIFTGPU::FeatureDetector_KD_SIFTGPU(const Parameter &_p)
 : FeatureDetector(KD_SIFTGPU), 
   param( PSiftGPU::Parameter(_p.distmax,_p.ratiomax,_p.mutual_best_match,_p.computeRootSIFT) )
{ 
  sift = new PSiftGPU(param);
}

FeatureDetector_KD_SIFTGPU::~FeatureDetector_KD_SIFTGPU()
{
}

/***************************************************************************************/

/**
 * detect keypoints and extract descriptors
 */
void FeatureDetector_KD_SIFTGPU::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  sift->detect(im_gray, keys, descriptors);
}

/**
 * detect keypoints
 */
void FeatureDetector_KD_SIFTGPU::detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  sift->detect(im_gray, keys);
}

/**
 * detect keypoints and extract descriptors
 */
void FeatureDetector_KD_SIFTGPU::extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors)
{
  if( image.type() != CV_8U ) cv::cvtColor( image, im_gray, CV_RGB2GRAY );
  else im_gray = image;  

  sift->compute(im_gray, keys, descriptors);
}



}












