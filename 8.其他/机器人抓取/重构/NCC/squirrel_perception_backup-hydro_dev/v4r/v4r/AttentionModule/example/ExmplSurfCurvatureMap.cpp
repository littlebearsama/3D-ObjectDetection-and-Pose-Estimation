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


#include <opencv2/opencv.hpp>

#include "v4r/AttentionModule/AttentionModule.hpp"
#include "v4r/EPUtils/EPUtils.hpp"

// This program shows the use of Surface Curvature Saliency Map on one image

int main(int argc, char** argv)
{
  if(argc != 3)
  {
    std::cerr << "Usage: image cloud" << std::endl;
    return(0);
  }

  // read image
  std::string image_name(argv[1]);
  cv::Mat image = cv::imread(image_name,-1);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  std::string cloud_name(argv[2]);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloud_name,*cloud) == -1)
  {
    printf("[ERROR]: Couldn't read point cloud.\n");
    return -1;
  }

  // start creating parameters
  AttentionModule::SurfaceCurvatureMap surfaceCurvatureMap;
  surfaceCurvatureMap.setWidth(image.cols);
  surfaceCurvatureMap.setHeight(image.rows);

  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
    return(pclAddOns::FILTER);

  // segment plane
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
  pcl::PointIndices::Ptr objects_indices(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  if(!pclAddOns::SegmentPlane<pcl::PointXYZRGB>(cloud,indices,plane_indices,objects_indices,coefficients))
  {
    return(pclAddOns::SEGMENT);
  }
  
  //calculate point cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,objects_indices,normals))
  {
    return(pclAddOns::NORMALS);
  }

  surfaceCurvatureMap.setCloud(cloud);
  surfaceCurvatureMap.setIndices(objects_indices);
  surfaceCurvatureMap.setNormals(normals);

  cv::Mat map;

  printf("[INFO]: Computing surface curvature map FLAT.\n");
  surfaceCurvatureMap.setCurvatureType(AttentionModule::AM_FLAT);
  surfaceCurvatureMap.calculate();

  if(surfaceCurvatureMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Surface Curvature Map FLAT",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }

  printf("[INFO]: Computing surface curvature map CONVEX.\n");
  surfaceCurvatureMap.setCurvatureType(AttentionModule::AM_CONVEX);
  surfaceCurvatureMap.calculate();

  if(surfaceCurvatureMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Surface Curvature Map CONVEX",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  return(0);
}