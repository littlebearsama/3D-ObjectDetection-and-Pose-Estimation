/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */


#include "v4r/EPUtils/EPUtils.hpp"

#include <iostream>

int main(int argc, char** argv)
{
  if(argc != 5)
  {
    std::cerr << "Usage: XYZRGB disparity width height" << std::endl;
    return(0);
  }
  
  std::string XYZRGB_name(argv[1]);
  std::string disparity_name(argv[2]);
  int width = atoi(argv[3]);
  int height = atoi(argv[4]);
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (XYZRGB_name,*cloud) == -1)
  {
    printf("[ERROR]: Couldn't read point cloud.\n");
    return -1;
  }
  
  cv::Mat disparity_image;
  EPUtils::pointCloud_2_disparity(disparity_image,cloud,width,height,pcl::PointIndices::Ptr(new pcl::PointIndices()),525,0.075,0.05);
  
  disparity_image.convertTo(disparity_image,CV_8U,1.0);
  
  cv::Mat disparity_image2, disparity_image_mask;
  disparity_image_mask = cv::Mat_<uchar>::zeros(disparity_image.rows,disparity_image.cols);
  
  for(int i = 0; i < disparity_image.rows; ++i)
  {
    for(int j = 0; j < disparity_image.cols; ++j)
    {
      if(disparity_image.at<uchar>(i,j) >= 255)
	disparity_image_mask.at<uchar>(i,j) = 1;
    }
  }
  
//   cv::imshow("disparity_image",disparity_image);
//   cv::imshow("disparity_image_mask",disparity_image_mask*255);
  
  cv::inpaint(disparity_image,disparity_image_mask,disparity_image2,5,cv::INPAINT_TELEA);
  
//   cv::imshow("disparity_image2",disparity_image2);
//   cv::waitKey(-1);

  cv::imwrite(disparity_name,disparity_image2);

  return 0;
}