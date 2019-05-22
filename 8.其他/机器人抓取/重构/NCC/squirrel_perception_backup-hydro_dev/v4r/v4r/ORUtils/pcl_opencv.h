/*
 * pcl_opencv.h
 *
 *  Created on: Oct 17, 2013
 *      Author: aitor
 */

#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>

#ifndef PCL_OPENCV_H_
#define PCL_OPENCV_H_

namespace PCLOpenCV
{

  template<class PointT>
  void
  ConvertPCLCloud2Image (typename pcl::PointCloud<PointT>::Ptr &pcl_cloud, cv::Mat_<cv::Vec3b> &image)
  {
    unsigned pcWidth = pcl_cloud->width;
    unsigned pcHeight = pcl_cloud->height;
    unsigned position = 0;

    image = cv::Mat_<cv::Vec3b> (pcHeight, pcWidth);

    for (unsigned row = 0; row < pcHeight; row++)
    {
      for (unsigned col = 0; col < pcWidth; col++)
      {
        cv::Vec3b & cvp = image.at<cv::Vec3b> (row, col);
        position = row * pcWidth + col;
        const PointT &pt = pcl_cloud->points[position];

        cvp[0] = pt.b;
        cvp[1] = pt.g;
        cvp[2] = pt.r;
      }
    }
  }

  template<class PointT>
  void
  ConvertPCLCloud2Image (const typename pcl::PointCloud<PointT>::Ptr &pcl_cloud, cv::Mat_<cv::Vec3b> &image)
  {
    unsigned pcWidth = pcl_cloud->width;
    unsigned pcHeight = pcl_cloud->height;
    unsigned position = 0;

    image = cv::Mat_<cv::Vec3b> (pcHeight, pcWidth);

    for (unsigned row = 0; row < pcHeight; row++)
    {
      for (unsigned col = 0; col < pcWidth; col++)
      {
        cv::Vec3b & cvp = image.at<cv::Vec3b> (row, col);
        position = row * pcWidth + col;
        const PointT &pt = pcl_cloud->points[position];

        cvp[0] = pt.b;
        cvp[1] = pt.g;
        cvp[2] = pt.r;
      }
    }
  }

  template<class PointT>
  void
  ConvertPCLCloud2DepthImage (typename pcl::PointCloud<PointT>::Ptr &pcl_cloud, cv::Mat_<float> &image)
  {
    unsigned pcWidth = pcl_cloud->width;
    unsigned pcHeight = pcl_cloud->height;
    unsigned position = 0;

    image = cv::Mat_<float> (pcHeight, pcWidth);

    for (unsigned row = 0; row < pcHeight; row++)
    {
      for (unsigned col = 0; col < pcWidth; col++)
      {
        //cv::Vec3b & cvp = image.at<cv::Vec3b> (row, col);
        position = row * pcWidth + col;
        const PointT &pt = pcl_cloud->points[position];
        image.at<float>(row,col) = 1.f / pt.z;
      }
    }
  }
}

#endif /* PCL_OPENCV_H_ */
