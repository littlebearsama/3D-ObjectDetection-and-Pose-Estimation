/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_COMPUTE_GDESC_ORIENTATIONS_HH
#define KP_COMPUTE_GDESC_ORIENTATIONS_HH

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ImGDescOrientation.hh"


namespace kp 
{

class ComputeImGDescOrientations
{
public:
  class Parameter
  {
  public:
    int win_size;
    ImGDescOrientation::Parameter goParam;
    Parameter(int _win_size=34, 
      const ImGDescOrientation::Parameter &_goParam=ImGDescOrientation::Parameter())
    : win_size(_win_size), goParam(_goParam) {}
  };

private:
  Parameter param;

  int h_win;

public:
 

  ComputeImGDescOrientations(const Parameter &p=Parameter());
  ~ComputeImGDescOrientations();

  void compute(const cv::Mat_<unsigned char> &image, const std::vector<cv::Point2f> &pts, 
        std::vector<cv::KeyPoint> &keys);
  void compute(const cv::Mat_<unsigned char> &image, std::vector<cv::KeyPoint> &keys);
  //void compute(const cv::Mat_<unsigned char> &image, std::vector<AffKeypoint> &keys);


  typedef SmartPtr< ::kp::ComputeImGDescOrientations> Ptr;
  typedef SmartPtr< ::kp::ComputeImGDescOrientations const> ConstPtr;

};



/*************************** INLINE METHODES **************************/



} //--END--

#endif

