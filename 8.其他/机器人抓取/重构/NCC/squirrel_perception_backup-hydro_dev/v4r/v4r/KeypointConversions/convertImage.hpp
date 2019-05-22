/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CONVERT_CLOUDTOIMAGE_HPP
#define KP_CONVERT_CLOUDTOIMAGE_HPP

#include <float.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace kp
{


void convertImage(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat &image)
{
  image = cv::Mat_<cv::Vec3b>(cloud.height, cloud.width);

  for (unsigned v = 0; v < cloud.height; v++) 
  {
    for (unsigned u = 0; u < cloud.width; u++) 
    {
      cv::Vec3b &cv_pt = image.at<cv::Vec3b> (v, u);
      const pcl::PointXYZRGB &pt = cloud(u,v);

      cv_pt[2] = pt.r;
      cv_pt[1] = pt.g;
      cv_pt[0] = pt.b;
    }
  }
}



} //--END--

#endif

