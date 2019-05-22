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


#include "BaseMap.hpp"

namespace AttentionModule
{

BaseMap::BaseMap()
{
  reset();
}

BaseMap::~BaseMap()
{
}

void BaseMap::reset()
{
  image = cv::Mat_<float>::zeros(0,0);
  mask  = cv::Mat_<uchar>::zeros(0,0);
  cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr ( new pcl::PointCloud<pcl::PointXYZRGB>() );
  normals = pcl::PointCloud<pcl::Normal>::Ptr ( new pcl::PointCloud<pcl::Normal>() );
  indices = pcl::PointIndices::Ptr ( new pcl::PointIndices() );
  normalization_type = EPUtils::NT_NONE;
  combination_type = AM_COMB_SUM;
  filter_size = 5;
  map = cv::Mat_<float>::zeros(0,0);
  width = 0;
  height = 0;
  calculated = false;
  
  refine = true;

  mapName = "BaseMap";
  
  haveImage = false;
  haveCloud = false;
  haveNormals = false;
  haveIndices = false;
  haveMask = false;
}

void BaseMap::print()
{
  printf("[%s]: mapName            = %s\n",mapName.c_str(),mapName.c_str());
  printf("[%s]: normalization_type = %d\n",mapName.c_str(),normalization_type);
  printf("[%s]: combination_type   = %d\n",mapName.c_str(),combination_type);
  printf("[%s]: filter_size        = %d\n",mapName.c_str(),filter_size);
  printf("[%s]: width              = %d\n",mapName.c_str(),width);
  printf("[%s]: height             = %d\n",mapName.c_str(),height);
  printf("[%s]: calculated         = %s\n",mapName.c_str(),calculated ? "yes" : "no");
  printf("[%s]: haveImage          = %s\n",mapName.c_str(),haveImage ? "yes" : "no");
  printf("[%s]: haveCloud          = %s\n",mapName.c_str(),haveCloud ? "yes" : "no");
  printf("[%s]: haveNormals        = %s\n",mapName.c_str(),haveNormals ? "yes" : "no");
  printf("[%s]: haveIndices        = %s\n",mapName.c_str(),haveIndices ? "yes" : "no");
  printf("[%s]: haveMask           = %s\n",mapName.c_str(),haveMask ? "yes" : "no");
  printf("[%s]: refine             = %s\n",mapName.c_str(),refine ? "yes" : "no");
}

int BaseMap::checkParameters()
{
  printf("[INFO]: %s: Please implement checkParameters function! Further results are undefined!\n",mapName.c_str());
  return(AM_OK);
}

int BaseMap::calculatePyramid(int pyramidType)
{
  switch (pyramidType) {
    case SIMPLE_PYRAMID:
      printf("[INFO]: %s: Simple pyramid will be calculated!\n",mapName.c_str());
      return(calculatePyramidSimple());
    case ITTI_PYRAMID:
      printf("[INFO]: %s: Itti pyramid will be calculated!\n",mapName.c_str());
      return(calculatePyramidItti());
    case FRINTROP_PYRAMID:
      printf("[INFO]: %s: Frintrop pyramid will be calculated!\n",mapName.c_str());
      return(calculatePyramidFrintrop());
    default: 
      printf("[INFO]: %s: Pyramid type wasn't detected, Simple pyramid will be calculated instead!\n",mapName.c_str());
      return(calculatePyramidSimple());
  }
  
  return(AM_OK);
}

int BaseMap::calculatePyramidSimple()
{
  printf("[INFO]: %s: Sorry, but Simple pyramid calculation is not available!\n",mapName.c_str());
  calculated = false;
  return(AM_OK);
}

int BaseMap::calculatePyramidItti()
{
  printf("[INFO]: %s: Sorry, but Itti pyramid calculation is not available!\n",mapName.c_str());
  calculated = false;
  return(AM_OK);
}

int BaseMap::calculatePyramidFrintrop()
{
  printf("[INFO]: %s: Sorry, but Frintrop pyramid calculation is not available!\n",mapName.c_str());
  calculated = false;
  return(AM_OK);
}

int BaseMap::combinePyramid(BasePyramid::Ptr pyramid)
{
  printf("[INFO]: %s: Sorry, but combinePyramid calculation is not available!\n",mapName.c_str());
  calculated = false;
  return(AM_OK);
}

void BaseMap::refineMap()
{
  if(!haveIndices)
  {
    printf("[INFO]: %s: Map refinement is available only when indices are set!\n",mapName.c_str());
    return;
  }
  
  if(!refine)
  {
    printf("[INFO]: %s: You should at first allow refinement!\n",mapName.c_str());
    return;
  }
  
  cv::Mat temp_mask = cv::Mat_<float>::ones(map.rows,map.cols);
  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    
    int r = idx / map.cols;
    int c = idx % map.cols;
    
    temp_mask.at<float>(r,c) = 0;
  }
  
  for(int i = 0; i < map.rows; ++i)
  {
    for(int j = 0; j < map.cols; ++j)
    {
      if(temp_mask.at<float>(i,j) > 0)
      {
	map.at<float>(i,j) = 0;
      }
    }
  }
  
  printf("[INFO]: %s: Map was refined!\n",mapName.c_str());
  
}

bool BaseMap::getMap(cv::Mat &map_)
{
  if(!calculated)
    return(false);
  
  map.copyTo(map_);
  return(true);
}

void BaseMap::setMask(const cv::Mat &mask_)
{
  mask_.copyTo(mask);
  haveMask = true;
  calculated = false;

  printf("[INFO]: %s: got mask.\n",mapName.c_str());
}

void BaseMap::setNormalizationType(int normalization_type_)
{
  normalization_type = normalization_type_;
  calculated = false;

  printf("[INFO]: %s: normalization_type is set to: %d\n",mapName.c_str(),normalization_type);
}

void BaseMap::setCombinationType(int combination_type_)
{
  combination_type = combination_type_;
  calculated = false;

  printf("[INFO]: %s: combination_type is set to: %d\n",mapName.c_str(),combination_type);
}

void BaseMap::setFilterSize(int filter_size_)
{
  filter_size = filter_size_;
  calculated = false;

  printf("[INFO]: %s: filter_size is set to: %d\n",mapName.c_str(),filter_size);
}

void BaseMap::setWidth(int width_)
{
  width = width_;
  calculated = false;

  printf("[INFO]: %s: width is set to: %d\n",mapName.c_str(),width);
}

void BaseMap::setHeight(int height_)
{
  height = height_;
  calculated = false;

  printf("[INFO]: %s: height is set to: %d\n",mapName.c_str(),height);
}

void BaseMap::setMapName(std::string mapName_)
{
  mapName = mapName_;

  printf("[INFO]: %s: mapName is set to: %s\n",mapName.c_str(),mapName.c_str());
}

void BaseMap::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_)
{
  cloud = cloud_;
  haveCloud = true;
  calculated = false;

  printf("[INFO]: %s: got point cloud.\n",mapName.c_str());
}

void BaseMap::setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_)
{
  normals = normals_;
  haveNormals = true;
  calculated = false;
  
  printf("[INFO]: %s: got normals.\n",mapName.c_str());
}

void BaseMap::setIndices(pcl::PointIndices::Ptr indices_)
{
  indices = indices_;
  haveIndices = true;
  calculated = false;

  printf("[INFO]: %s: got indices.\n",mapName.c_str());
}

void BaseMap::setImage(const cv::Mat &image_)
{
  image_.copyTo(image);

  width = image.cols;
  height = image.rows;
  
  haveImage = true;

  calculated = false;

  printf("[INFO]: %s: got image.\n",mapName.c_str());
}

bool BaseMap::getImage(cv::Mat &image_)
{
  if(!haveImage)
    return(false);
  
  image.copyTo(image_);
  return(true);
}

bool BaseMap::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_)
{
  if(!haveCloud)
    return(false);
  
  cloud_ = cloud;
  return(true);
}

bool BaseMap::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_)
{
  if(!haveNormals)
    return(false);
  
  normals_ = normals;
  return(true);
}

bool BaseMap::getIndices(pcl::PointIndices::Ptr &indices_)
{
  if(!haveIndices)
    return(false);
  
  indices_ = indices;
  return(true);
}

bool BaseMap::getMask(cv::Mat &mask_)
{
  if(!haveMask)
    return(false);
  
  mask.copyTo(mask_);
  return(true);
}

int BaseMap::getNormalizationType()
{
  return(normalization_type);
}

int BaseMap::getCombinationType()
{
  return(combination_type);
}

int  BaseMap::getFilterSize()
{
  return(filter_size);
}

int  BaseMap::getWidth()
{
  return(width);
}

int  BaseMap::getHeight()
{
  return(height);
}

std::string BaseMap::getMapName()
{
  return(mapName);
}

void BaseMap::setRefine(bool refine_)
{
  refine = refine_;
}

bool BaseMap::getRefine()
{
  return(refine);
}

} //namespace AttentionModule