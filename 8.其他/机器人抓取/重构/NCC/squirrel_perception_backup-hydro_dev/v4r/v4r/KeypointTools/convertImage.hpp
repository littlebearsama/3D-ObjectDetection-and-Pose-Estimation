/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CONVERT_IMAGE_HPP
#define KP_CONVERT_IMAGE_HPP

#include <float.h>
#include <opencv2/core/core.hpp>
#include <stdexcept>
#include "DataMatrix2D.hpp"
#include "PointTypes.hpp"


namespace kp
{


void convertImage(const DataMatrix2D<PointXYZRGB> &cloud, cv::Mat &image)
{
  if (cloud.rows<=1)
    throw std::runtime_error("[convertCloudToImage] Need an organized point cloud!");

  image = cv::Mat_<cv::Vec3b>(cloud.rows, cloud.cols);

  int z=0;
  for (int v = 0; v < cloud.rows; v++) 
  {
    for (int u = 0; u < cloud.cols; u++,z++) 
    {
      cv::Vec3b &cv_pt = image.at<cv::Vec3b> (v, u);
      const PointXYZRGB &pt = cloud.data[z];

      cv_pt[2] = pt.r;
      cv_pt[1] = pt.g;
      cv_pt[0] = pt.b;
    }
  }
}



} //--END--

#endif

