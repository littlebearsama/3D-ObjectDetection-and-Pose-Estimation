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


#include "v4r/AttentionModule/AttentionModule.hpp"
#include "v4r/EPUtils/EPUtils.hpp"

int main(int argc, char** argv)
{
  if(argc != 3)
  {
    std::cerr << "Usage: image cloud" << std::endl;
    return(0);
  }
  
  std::string image_name(argv[1]);
  std::string cloud_name(argv[2]);
  // read image
  cv::Mat image = cv::imread(image_name,-1);
    
  AttentionModule::Symmetry3DMap symmetry3DMap;
    
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>() );
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (cloud_name,*(cloud)) == -1)
  {
    std::cerr << "[ERROR] Couldn't read point cloud." << std::endl;
    return -1;
  }
  
  // start creating parameters
  symmetry3DMap.setCloud(cloud);
  symmetry3DMap.setWidth(image.cols);
  symmetry3DMap.setHeight(image.rows);
    
  //filter just obtained point cloud
  pcl::PointIndices::Ptr indices (new pcl::PointIndices() );
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
    return(pclAddOns::FILTER);
  symmetry3DMap.setIndices(indices);
    
  //calculate point cloud normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>() );
  if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,indices,normals,50))
    return(pclAddOns::NORMALS);
  symmetry3DMap.setNormals(normals);

  symmetry3DMap.setPyramidMode(false);
  symmetry3DMap.compute();  
  cv::Mat symmetry_map_one_level;
  symmetry3DMap.getMap(symmetry_map_one_level);
  
  symmetry3DMap.setPyramidMode(true);
  symmetry3DMap.compute();  
  cv::Mat symmetry_map_pyramid;
  symmetry3DMap.getMap(symmetry_map_pyramid);
  
  cv::imshow("symmetry_map_one_level",symmetry_map_one_level);
  cv::imshow("symmetry_map_pyramid",symmetry_map_pyramid);
  cv::waitKey();
    
  return(0);
}