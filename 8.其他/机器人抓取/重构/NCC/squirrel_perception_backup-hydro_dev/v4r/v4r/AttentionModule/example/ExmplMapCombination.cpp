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


#include "AttentionExampleUtils.hpp"

#include <opencv2/opencv.hpp>

#include "v4r/AttentionModule/AttentionModule.hpp"
#include "AttentionExampleUtils.hpp"

// This program shows the use of Map Combinations on one image

void printUsage(const char *argv0)
{
  printf(
    "Calculates 3D Symmentry Saliency Map\n"
    "usage: %s image.png cloud.pcd combination_type normalization_type result.png [color_saliencymap.png]\n"
    "  image.png             ... color image\n"
    "  cloud.pcd             ... point cloud\n"
    "  combination_type      ... 0 -- SUM; 1 -- MUL; 2 -- MAX\n"
    "  normalization_type    ... 0 -- LIN; 1 -- NMS; 2 -- NLM\n"
    "  result.png            ... output file name\n"
    "  color_saliencymap.png ... color saliency map\n", argv0);
  printf(" Example: %s image.png cloud.pcd 0 0 result.png [color_saliencymap.png]\n",argv0);
}

int main(int argc, char** argv)
{
  if( (argc != 6) && (argc != 7) )
  {
    printUsage(argv[0]);
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

  cloud->width = image.cols;
  cloud->height = image.rows;
  
  int combination_type = atoi(argv[3]);
  int normalization_type = atoi(argv[4]);
  
  std::string output_name(argv[5]);
  
  std::vector<cv::Mat> maps;
  maps.resize(3);
  
  std::string color_saliencymap_name;
  bool have_color_saliencymap = false;
  
  if(argc == 7)
  {
    color_saliencymap_name = argv[6];
    have_color_saliencymap = true;
    maps.resize(4);
  }
  
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  pcl::PointIndices::Ptr object_indices_in_the_hull(new pcl::PointIndices());
  preparePointCloud(cloud,coefficients,normals,object_indices_in_the_hull);

  // start creating parameters
  AttentionModule::RelativeSurfaceOrientationMap relativeSurfaceOrientationMap;
  relativeSurfaceOrientationMap.setWidth(image.cols);
  relativeSurfaceOrientationMap.setHeight(image.rows);
  relativeSurfaceOrientationMap.setCloud(cloud);
  relativeSurfaceOrientationMap.setIndices(object_indices_in_the_hull);
  relativeSurfaceOrientationMap.setNormals(normals);
  
  pcl::Normal orientation_normal;
  orientation_normal.normal[0] = coefficients->values[0];
  orientation_normal.normal[1] = coefficients->values[1];
  orientation_normal.normal[2] = coefficients->values[2];
  relativeSurfaceOrientationMap.setOrientationNormal(orientation_normal);

  // start creating parameters
  AttentionModule::SurfaceHeightSaliencyMap surfaceHeightSaliencyMap;
  surfaceHeightSaliencyMap.setWidth(image.cols);
  surfaceHeightSaliencyMap.setHeight(image.rows);
  surfaceHeightSaliencyMap.setCloud(cloud);
  surfaceHeightSaliencyMap.setIndices(object_indices_in_the_hull);
  surfaceHeightSaliencyMap.setNormals(normals);
  surfaceHeightSaliencyMap.setModelCoefficients(coefficients);
  
  printf("Normalization Type: ");
  switch(normalization_type)
  {
    case(0):
      printf("LIN \n");
      relativeSurfaceOrientationMap.setNormalizationType(EPUtils::NT_NONE);
      surfaceHeightSaliencyMap.setNormalizationType(EPUtils::NT_NONE);
      normalization_type = EPUtils::NT_NONE;
      break;
    case(1):
      printf("NMS \n");
      relativeSurfaceOrientationMap.setNormalizationType(EPUtils::NT_NONMAX);
      surfaceHeightSaliencyMap.setNormalizationType(EPUtils::NT_NONMAX);
      normalization_type = EPUtils::NT_NONMAX;
      break;
    case(2):
      printf("NLM \n");
      relativeSurfaceOrientationMap.setNormalizationType(EPUtils::NT_FRINTROP_NORM);
      surfaceHeightSaliencyMap.setNormalizationType(EPUtils::NT_FRINTROP_NORM);
      normalization_type = EPUtils::NT_FRINTROP_NORM;
      break;
    default:
      printf("UNDEFINED -- return \n");
      return(0);
  }
  
  printf("[INFO]: Computing surface orientation map HORIZONTAL.\n");
  relativeSurfaceOrientationMap.setOrientationType(AttentionModule::AM_HORIZONTAL);
  relativeSurfaceOrientationMap.calculate();
  
  if(!relativeSurfaceOrientationMap.getMap(maps.at(0)))
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing surface distance map.\n");
  surfaceHeightSaliencyMap.setHeightType(AttentionModule::AM_DISTANCE);
  surfaceHeightSaliencyMap.calculate();
  
  if(!surfaceHeightSaliencyMap.getMap(maps.at(1)))
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing surface TALL map.\n");
  surfaceHeightSaliencyMap.setHeightType(AttentionModule::AM_TALL);
  surfaceHeightSaliencyMap.calculate();
  
  if(!surfaceHeightSaliencyMap.getMap(maps.at(2)))
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  
  printf("Combination Type: ");
  switch(combination_type)
  {
    case(0):
      printf("SUM \n");
      combination_type = AttentionModule::AM_COMB_SUM;
      break;
    case(1):
      printf("MUL \n");
      combination_type = AttentionModule::AM_COMB_MUL;
      break;
    case(2):
      printf("MAX \n");
      combination_type = AttentionModule::AM_COMB_MAX;
      break;
    default:
      printf("UNDEFINED -- return \n");
      return(0);
  }
  
  if(have_color_saliencymap)
  {
    cv::Mat color_saliencymap = cv::imread(color_saliencymap_name,-1);
    color_saliencymap.convertTo(color_saliencymap,CV_32F,1.0f/255);
    color_saliencymap.copyTo(maps.at(3));
  }
  
  
  cv::Mat map_sum;
  if(AttentionModule::CombineMaps(maps,map_sum,combination_type,normalization_type) == AttentionModule::AM_OK)
  {
    EPUtils::normalize(map_sum);
    cv::Mat temp;
    map_sum.convertTo(temp,CV_8U,255);
    cv::imwrite(output_name,temp);
  }
  else
  {
    printf("[ERROR]: Failed to compute combination.\n");
  }
  
  
  
  return(0);
}