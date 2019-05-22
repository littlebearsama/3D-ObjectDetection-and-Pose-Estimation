/**
 * $Id$
 */

#ifndef AR_COMPLEX_PATTERN_DETECTOR_HH
#define AR_COMPLEX_PATTERN_DETECTOR_HH

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <stdexcept>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "armarker.h"


namespace ar 
{


/**
 * ARComplexPatternDetector
 */
class ARComplexPatternDetector
{
private:
  cv::Mat imGray;

  V4R::MarkerDetection ardetector;
  std::map<int, std::vector<cv::Point3f> > markerPoints;

  void setup(const std::string &config_file);


public:
  int paramMinPoints;
  cv::Mat dbg;

  ARComplexPatternDetector(const std::string &config_file, int _paramMinPoints=4);
  ~ARComplexPatternDetector();

  void detectMarker(const cv::Mat &image, std::vector<cv::Point3f> &objPoints, 
        std::vector<cv::Point2f> &imgPoints);
  bool getPose(const cv::Mat &image, const cv::Mat& _intrinsic, const cv::Mat& _distCoeffs, cv::Mat &R, cv::Mat &T);
};




} //--END--

#endif

