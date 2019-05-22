/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_KEYPOINT_DETECTOR_ORB_IMGDESC_HH
#define KP_KEYPOINT_DETECTOR_ORB_IMGDESC_HH

#include <opencv2/features2d/features2d.hpp>
#include "FeatureDetector.hh"
#include "ComputeImGradientDescriptors.hh"



namespace kp 
{

class FeatureDetector_KD_FAST_IMGD : public FeatureDetector
{
public:
  class Parameter
  {
  public:
    int nfeatures;
    float scaleFactor;
    int nlevels;
    int patchSize;
    int tiles;
    ComputeImGradientDescriptors::Parameter gdParam;

    Parameter(int _nfeatures=1000, float _scaleFactor=1.44, 
      int _nlevels=2, int _patchSize=17, int _tiles=1,
      const ComputeImGradientDescriptors::Parameter &_gdParam=ComputeImGradientDescriptors::Parameter()) 
    : nfeatures(_nfeatures), scaleFactor(_scaleFactor), 
      nlevels(_nlevels), patchSize(_patchSize), tiles(_tiles),
      gdParam(_gdParam) {}
  };

private:
  Parameter param;

  const static int PATCH_SIZE = 15;

  int tile_size_w, tile_size_h;

  cv::Mat_<unsigned char> im_gray;  

  std::vector<cv::Point2f> pts;

  cv::Ptr<cv::ORB> orb;
  ComputeImGradientDescriptors::Ptr imGDesc;

  inline void getExpandedRect(int u, int v, int rows, int cols, cv::Rect &rect);

public:
  FeatureDetector_KD_FAST_IMGD(const Parameter &_p=Parameter());
  ~FeatureDetector_KD_FAST_IMGD();

  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 
  virtual void detect(const cv::Mat &image, std::vector<cv::KeyPoint> &keys); 
  virtual void extract(const cv::Mat &image, std::vector<cv::KeyPoint> &keys, cv::Mat &descriptors); 


  typedef SmartPtr< ::kp::FeatureDetector_KD_FAST_IMGD> Ptr;
  typedef SmartPtr< ::kp::FeatureDetector_KD_FAST_IMGD const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

/**
 * getExpandedRect
 */
inline void FeatureDetector_KD_FAST_IMGD::getExpandedRect(int u, int v, int rows, int cols, cv::Rect &rect)
{
  int border = PATCH_SIZE;

  int x1 = u*tile_size_w;
  int y1 = v*tile_size_h;
  int x2 = x1 + tile_size_w;
  int y2 = y1 + tile_size_h;

  x1 -= border;
  y1 -= border;
  x2 += border;
  y2 += border;

  if (x1<0) x1 = 0;
  if (y1<0) y1 = 0;
  if (x2>=cols) x2 = cols-1;
  if (y2>=rows) y2 = rows-1;

  rect = cv::Rect(x1, y1, x2-x1, y2-y1);
}


} //--END--

#endif

