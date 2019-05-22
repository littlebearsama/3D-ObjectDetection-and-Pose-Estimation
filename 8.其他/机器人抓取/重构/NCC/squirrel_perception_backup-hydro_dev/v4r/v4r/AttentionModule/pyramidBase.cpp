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


#include "pyramidBase.hpp"

namespace AttentionModule
{

BasePyramid::BasePyramid()
{
  reset();
}

void BasePyramid::reset()
{
  start_level = 0;
  max_level = 4;
  sm_level = 0;
  normalization_type = EPUtils::NT_NONE;
  width = 0;
  height = 0;
  combination_type = AM_COMB_SUM;
  pyramidImages.clear();
  pyramidFeatures.clear();
  map = cv::Mat_<float>::zeros(0,0);

  haveImage = false;
  haveCloud = false;
  haveIndices = false;
  haveNormals = false;
  calculated = false;
  haveImagePyramid = false;
  haveDepthPyramid = false;
  haveNormalPyramid = false;
  haveIndicePyramid = false;

  pyramidName = "BasePyramid";
}

void BasePyramid::setStartLevel(int start_level_)
{
  start_level = start_level_;
  calculated = false;
  haveImagePyramid = false;
  haveDepthPyramid = false;
  haveNormalPyramid = false;
  haveIndicePyramid = false;
  printf("[INFO]: %s: start_level is set to: %d\n",pyramidName.c_str(),start_level);
}

int BasePyramid::getStartLevel()
{
  return(start_level);
}

void BasePyramid::setMaxLevel(int max_level_)
{
  max_level = max_level_;
  calculated = false;
  haveImagePyramid = false;
  haveDepthPyramid = false;
  haveNormalPyramid = false;
  haveIndicePyramid = false;
  printf("[INFO]: %s: max_level is set to: %d\n",pyramidName.c_str(),max_level);
}

int BasePyramid::getMaxLevel()
{
  return(max_level);
}

void BasePyramid::setSMLevel(int sm_level_)
{
  sm_level = sm_level_;
  calculated = false;
  haveImagePyramid = false;
  haveDepthPyramid = false;
  haveNormalPyramid = false;
  printf("[INFO]: %s: sm_level is set to: %d\n",pyramidName.c_str(),sm_level);
}

int BasePyramid::getSMLevel()
{
  return(sm_level);
}

void BasePyramid::setNormalizationType(int normalization_type_)
{
  normalization_type = normalization_type_;
  calculated = false;
  haveImagePyramid = false;
  haveDepthPyramid = false;
  haveNormalPyramid = false;
  haveIndicePyramid = false;
  printf("[INFO]: %s: normalization_type is set to: %d\n",pyramidName.c_str(),normalization_type);
}

int BasePyramid::getNormalizationType()
{
  return(normalization_type);
}

void BasePyramid::setWidth(int width_)
{
  width = width_;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: width is set to: %d\n",pyramidName.c_str(),width);
}

int BasePyramid::getWidth()
{
  return(width);
}

int BasePyramid::getWidth(unsigned int level)
{
  if(!haveImagePyramid)
    return(0);

  if(pyramidImages.size() <= level)
    return(0);
  
  return(pyramidImages.at(level).cols);
}

void BasePyramid::setHeight(int height_)
{
  height = height_;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: height is set to: %d\n",pyramidName.c_str(),height);
}

int BasePyramid::getHeight()
{
  return(height);
}

int BasePyramid::getHeight(unsigned int level)
{
  if(!haveImagePyramid)
    return(0);

  if(pyramidImages.size() <= level)
    return(0);
  
  return(pyramidImages.at(level).rows);
}

void BasePyramid::setCombinationType(int combination_type_)
{
  combination_type = combination_type_;
  calculated = false;
  printf("[INFO]: %s: combination_type is set to: %d\n",pyramidName.c_str(),combination_type);
}

int BasePyramid::getCombinationType()
{
  return(combination_type);
}

void BasePyramid::setImage(cv::Mat &image_)
{
  image_.copyTo(image);
  printf("[INFO]: %s: image is set\n",pyramidName.c_str());
  width = image.cols;
  height = image.rows;
  haveImage = true;
  calculated = false;
  haveImagePyramid = false;
}

bool BasePyramid::getImage(cv::Mat &image_)
{
  if(!haveImage)
    return(false);

  image.copyTo(image_);
  return(true);
}

bool BasePyramid::getImage(unsigned int level, cv::Mat &image_)
{
  if(!haveImagePyramid)
  {
    return(false);
  }

  if(pyramidImages.size() <= level)
  {
    return(false);
  }
  
  pyramidImages.at(level).copyTo(image_);
  return(true);
}

void BasePyramid::setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_)
{ 
  cloud = cloud_;
  printf("[INFO]: %s: cloud is set\n",pyramidName.c_str());
  //width = cloud->width;
  //height = cloud->height;
  haveCloud = true;
  calculated = false;
  haveDepthPyramid = false;
}

bool BasePyramid::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_)
{
  if(!haveCloud)
    return(false);

  cloud_ = cloud;
  return(true);
}

bool BasePyramid::getCloud(unsigned int level, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_)
{
  if(!haveDepthPyramid)
    return(false);

  if(pyramidCloud.size() <= level)
    return(false);
  
  cloud_ = pyramidCloud.at(level);
  return(true);
}

void BasePyramid::setIndices(pcl::PointIndices::Ptr &indices_)
{ 
  indices = indices_;
  printf("[INFO]: %s: indices are set\n",pyramidName.c_str());
  haveIndices = true;
  calculated = false;
  haveDepthPyramid = false;
  haveIndicePyramid = false;
}

bool BasePyramid::getIndices(pcl::PointIndices::Ptr &indices_)
{
  if(!haveIndices)
    return(false);

  indices_ = indices;
  return(true);
}

bool BasePyramid::getIndices(unsigned int level, pcl::PointIndices::Ptr &indices_)
{
  if(!haveIndicePyramid)
    return(false);

  if(pyramidIndices.size() <= level)
    return(false);
  
  indices_ = pyramidIndices.at(level);
  return(true);
}

void BasePyramid::setNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_)
{
  if((!haveCloud) || (!haveIndices))
  {
    printf("[INFO]: %s: Please set cloud and indices before normals\n",pyramidName.c_str());
  }
  normals = normals_;
  printf("[INFO]: %s: normals are set\n",pyramidName.c_str());
  haveNormals = true;
  calculated = false;
  haveNormalPyramid = false;
}

bool BasePyramid::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_)
{
  if(!haveNormals)
    return(false);

  normals_ = normals;
  return(true);
}

bool BasePyramid::getNormals(unsigned int level, pcl::PointCloud<pcl::Normal>::Ptr &normals_)
{
  if(!haveNormalPyramid)
    return(false);

  if(pyramidNormals.size() <= level)
    return(false);
  
  normals_ = pyramidNormals.at(level);
  return(true);
}

void BasePyramid::setMaxMapValue(float max_map_value_)
{
  max_map_value = max_map_value_;
  calculated = false;
  printf("[INFO]: %s: max_map_value is set to: %f\n",pyramidName.c_str(),max_map_value);
}

float BasePyramid::getMaxMapValue()
{
  return(max_map_value);
}

std::vector<cv::Mat> BasePyramid::getPyramidImages()
{
  return(pyramidImages);
}

std::vector<cv::Mat> BasePyramid::getPyramidFeatures()
{
  return(pyramidFeatures);
}

bool BasePyramid::setFeatureMap(unsigned int level, cv::Mat &featureMap_)
{
  if(!haveImagePyramid)
    return(false);
  
  if(pyramidFeatures.size() <= level)
    return(false);
  
  featureMap_.copyTo(pyramidFeatures.at(level));
  return(true);
}

bool BasePyramid::getMap(cv::Mat &map_)
{
  if(!calculated)
    return(false);

  map.copyTo(map_);
  return(true);
}

//TODO!!!

void BasePyramid::print()
{
  printf("[PyramidParameters]: start_level            = %d\n",start_level);
  printf("[PyramidParameters]: max_level              = %d\n",max_level);
  printf("[PyramidParameters]: sm_level               = %d\n",sm_level);
  printf("[PyramidParameters]: normalization_type     = %d\n",normalization_type);
  printf("[PyramidParameters]: width                  = %d\n",width);
  printf("[PyramidParameters]: height                 = %d\n",height);
  printf("[PyramidParameters]: pyramidImages.size()   = %ld\n",pyramidImages.size());
  printf("[PyramidParameters]: combination_type       = %d\n",combination_type);
}

int BasePyramid::checkParameters(bool isDepth)
{
  if( (!isDepth) && (!haveImage) )
  {
    printf("[ERROR]: %s: Seems like there is no image.\n",pyramidName.c_str());
    return(AM_IMAGE);
  }
  
  if( (isDepth) && ((!haveCloud) || (!haveNormals)) )
  {
    printf("[ERROR]: %s: Seems like there is no point cloud or normals.\n",pyramidName.c_str());
    return(AM_POINTCLOUD);
  }

  checkLevels();

  return(AM_OK);
}

int BasePyramid::buildPyramid()
{
  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Pyramid computation started.\n",pyramidName.c_str());

  int max_level_ = max_level;
  pyramidImages.clear();
  cv::buildPyramid(image,pyramidImages,max_level_);
  
  pyramidFeatures.resize(pyramidImages.size());

  haveImagePyramid = true;
  
  printf("[INFO]: %s: Pyramid computation succeed.\n",pyramidName.c_str());

  return(AM_OK);
}

int BasePyramid::buildDepthPyramid()
{
  int rt_code = checkParameters(true);
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Depth pyramid computation started.\n",pyramidName.c_str());

  cv::Mat xchannel, ychannel, zchannel;
  EPUtils::pointCloud_2_channels(xchannel,ychannel,zchannel,cloud,width,height,indices);
  
  EPUtils::buildDepthPyramid(xchannel,pyramidX,zchannel,max_level);
  EPUtils::buildDepthPyramid(ychannel,pyramidY,zchannel,max_level);
  EPUtils::buildDepthPyramid(zchannel,pyramidZ,zchannel,max_level);
  
  cv::Mat xnormals, ynormals, znormals;
  EPUtils::normals_2_channels(xnormals,ynormals,znormals,normals,width,height,indices);
  
  EPUtils::buildDepthPyramid(xnormals,pyramidNx,zchannel,max_level);
  EPUtils::buildDepthPyramid(ynormals,pyramidNy,zchannel,max_level);
  EPUtils::buildDepthPyramid(znormals,pyramidNz,zchannel,max_level);
  
  cv::Mat indices_image;
  EPUtils::indices_2_image(indices_image,width,height,indices);
  
  EPUtils::buildDepthPyramid(indices_image,pyramidImages,indices_image,max_level);
  
  EPUtils::createIndicesPyramid(pyramidImages,pyramidIndices);
  EPUtils::createPointCloudPyramid(pyramidX,pyramidY,pyramidZ,pyramidImages,pyramidCloud);
  EPUtils::createNormalPyramid(pyramidNx,pyramidNy,pyramidNz,pyramidImages,pyramidNormals);
  
//   for(int i = 0; i < pyramidCloud.at(6)->points.size(); ++i)
//   {
//     std::cerr << "(" << pyramidCloud.at(6)->points.at(i).x << "," << pyramidCloud.at(6)->points.at(i).y << "," << pyramidCloud.at(6)->points.at(i).z << ") ";
//   }
//   std::cerr << std::endl << std::endl;
  
  pyramidFeatures.resize(pyramidImages.size());
  
  haveImagePyramid = true;
  haveDepthPyramid = true;
  haveNormalPyramid = true;
  haveIndicePyramid = true;
  
  printf("[INFO]: %s: Depth pyramid computation succeed.\n",pyramidName.c_str());
  
  return(AM_OK);
}

void BasePyramid::combinePyramid(bool standard)
{
  printf("[INFO]: %s: Sory, but combinePyramid() is not implemented.\n",pyramidName.c_str());
}

void BasePyramid::calculate()
{
  printf("[INFO]: %s: Sory, but calculate() is not implemented.\n",pyramidName.c_str());
}

void BasePyramid::checkLevels()
{
  int nw = width;
  int nh = height;
  int current_level = 0;
  while (((nw/2) >= 1) && ((nh/2) >= 1))
  {
    ++current_level;
    nw = nw/2;
    nh = nh/2;
  }
  
  max_level = (current_level>max_level ? max_level:current_level);
  
  if(start_level > max_level)
    start_level = max_level;

  //if((sm_level < start_level) || (sm_level > max_level))
  if(sm_level > max_level)
    sm_level = start_level;
  
}

void BasePyramid::combineConspicuityMaps(cv::Mat &sm_map, cv::Mat &consp_map)
{
  assert( sm_map.cols == consp_map.cols );
  assert( sm_map.rows == consp_map.rows );
  
  switch(combination_type)
  {
    case AM_COMB_SUM:
      cv::add(consp_map,sm_map,sm_map);
      return;
    case AM_COMB_MUL:
      double maxValue, minValue;
      cv::minMaxLoc(consp_map,&minValue,&maxValue);
      if(maxValue > 0.05)
        cv::multiply(consp_map,sm_map,sm_map);
      return;
    case AM_COMB_MAX:
      sm_map = cv::max(consp_map,sm_map);
      return;
    default:
      printf("[ERROR]: Check combination type!");
      return;
  }
}

}