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


#include "RelativeSurfaceOrientationMap.hpp"

namespace AttentionModule
{

RelativeSurfaceOrientationMap::RelativeSurfaceOrientationMap():
BaseMap()  
{
  reset();
}

RelativeSurfaceOrientationMap::~RelativeSurfaceOrientationMap()
{
}

void RelativeSurfaceOrientationMap::reset()
{
  BaseMap::reset();
  
  orientation_normal.normal[0] = -1.0; orientation_normal.normal[1] = 0.0; orientation_normal.normal[2] = 0.0;
  orientationType = AM_HORIZONTAL;
  normal_threshold = 0.000001;
  //cameraParametrs.clear();

  mapName = "RelativeSurfaceOrientationMap";

  haveOrientationNormal = false;
  //haveCameraParameters = false;
}

void RelativeSurfaceOrientationMap::print()
{
  BaseMap::print();
  printf("[%s]: orientation_normal = (%f,%f,%f)\n",mapName.c_str(),orientation_normal.normal_x,orientation_normal.normal_y,orientation_normal.normal_z);
  printf("[%s]: orientationType    = %d\n",mapName.c_str(),orientationType);
  printf("[%s]: normal_threshold    = %f\n",mapName.c_str(),normal_threshold);
  //printf("[%s]: cameraParametrs    [ ",mapName.c_str());
//   for(size_t i = 0; i < cameraParametrs.size(); ++i)
//   {
//     printf("%f, ",cameraParametrs.at(i));
//   }
//   printf("]\n");
}

void RelativeSurfaceOrientationMap::setOrientationNormal(pcl::Normal orientation_normal_)
{
  orientation_normal = orientation_normal_;
  haveOrientationNormal = true;
  calculated = false;
  printf("[INFO]: %s: orientation_normal is set to: (%f,%f,%f)\n",mapName.c_str(),orientation_normal.normal[0],orientation_normal.normal[1],orientation_normal.normal[2]);
}

void RelativeSurfaceOrientationMap::setOrientationType(int orientationType_)
{
  orientationType = orientationType_;
  calculated = false;
  printf("[INFO]: %s: orientationType is set to: %d\n",mapName.c_str(),orientationType);
}

void RelativeSurfaceOrientationMap::setNormalThreshold(float normal_threshold_)
{
  normal_threshold = normal_threshold_;
  calculated = false;
  printf("[INFO]: %s: heightType is set to: %f\n",mapName.c_str(),normal_threshold);
}

// void RelativeSurfaceOrientationMap::setCameraParameters(std::vector<float> &cameraParametrs_)
// {
//   if(cameraParametrs_.size() != 4)
//   {
//     printf("[INFO]: %s: There should be 4 camera parameters! You have %ld\n",mapName.c_str(),cameraParametrs_.size());
//     return;
//   }
//   
//   cameraParametrs = cameraParametrs_;
//   haveCameraParameters = true;
//   calculated = false;
//   printf("[INFO]: %s: cameraParametrs is set to: [%f,%f,%f,%f]\n",mapName.c_str(),cameraParametrs.at(0),cameraParametrs.at(1),cameraParametrs.at(2),cameraParametrs.at(3));
// }

bool RelativeSurfaceOrientationMap::getOrientationNormal(pcl::Normal &orientation_normal_)
{
  if(!haveOrientationNormal)
    return(false);
  
  orientation_normal_ = orientation_normal;
  return(true);
}

int RelativeSurfaceOrientationMap::getOrientationType()
{
  return(orientationType);
}

float RelativeSurfaceOrientationMap::getNormalThreshold()
{
  return(normal_threshold);
}

// bool RelativeSurfaceOrientationMap::getCameraParameters(std::vector<float> &cameraParametrs_)
// {
//   if(!haveCameraParameters)
//     return(false);
//   
//   cameraParametrs_ = cameraParametrs;
//   return(true);
// }

int RelativeSurfaceOrientationMap::checkParameters()
{

  if(!haveCloud)
  {
    printf("[ERROR]: %s: Seems like there is no cloud.\n",mapName.c_str());
    return(AM_POINTCLOUD);
  }
  
  if(!haveIndices)
  {
    printf("[ERROR]: %s: Seems like there are no indices.\n",mapName.c_str());
    return(AM_POINTCLOUD);
  }
  
  if(!haveNormals)
  {
    printf("[ERROR]: %s: Seems like there are no normals.\n",mapName.c_str());
    return(AM_NORMALCLOUD);
  }
  
  if(indices->indices.size() != normals->points.size())
  {
    printf("[ERROR]: %s: Seems like there is different number of indices and normals.\n",mapName.c_str());
    return(AM_DIFFERENTSIZES);
  }
  
  if( (width == 0) || (height == 0) )
  {
    printf("[ERROR]: %s: Seems like image size is wrong.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }
  
  return(AM_OK);
}

void RelativeSurfaceOrientationMap::orientationMap(pcl::PointCloud<pcl::Normal>::Ptr normals_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
						   float a, float b, float c, float orientationCoefficient, cv::Mat &map_cur)
{
  assert( normals_cur->points.size() == indices_cur->indices.size() );
  
  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  for(unsigned int pi = 0; pi < indices_cur->indices.size(); ++pi)
  {
    int idx = indices_cur->indices.at(pi);
    
    int y = idx / image_width;
    int x = idx % image_width;
    
    //if(mask.at<uchar>(y,x))
    {
      float nx = normals_cur->points.at(pi).normal[0];
      float ny = normals_cur->points.at(pi).normal[1];
      float nz = normals_cur->points.at(pi).normal[2];
      
      if(std::isnan(nx) || std::isnan(ny) || std::isnan(nz))
      {
        continue;
      }

      float value = nx*a + ny*b + nz*c;
      float n_mod = sqrt(nx*nx + ny*ny + nz*nz);
      float t_mod = sqrt(a*a + b*b + c*c);
      value = value / (n_mod*t_mod);
      value = value>0 ? value:-value;
      
      float t1 = 1.0-orientationCoefficient;
      float t2 = orientationCoefficient;
      value = t1*value + t2*(1.0-value);

      map_cur.at<float>(y,x) = value;
    }
  }
}

int RelativeSurfaceOrientationMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  // Retrieve normal values
  float a = orientation_normal.normal[0];
  float b = orientation_normal.normal[1];
  float c = orientation_normal.normal[2];
  
  if(a*a + b*b + c*c < normal_threshold)
  {
    printf("[ERROR]: %s: It seems like you have zero normal.\n",mapName.c_str());
    return(AM_NORMALCOEFFICIENTS);
  }

  float orientationCoefficient = getOrientationType(orientationType);

  orientationMap(normals,indices,width,height,a,b,c,orientationCoefficient,map);

  cv::blur(map,map,cv::Size(filter_size,filter_size));

  refineMap();
  
  EPUtils::normalize(map,normalization_type);
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());

  return(AM_OK);
}

int RelativeSurfaceOrientationMap::combinePyramid(BasePyramid::Ptr pyramid)
{
  // Retrieve normal values
  float a = orientation_normal.normal[0];
  float b = orientation_normal.normal[1];
  float c = orientation_normal.normal[2];
  
  if(a*a + b*b + c*c < normal_threshold)
  {
    printf("[ERROR]: %s: It seems like you have zero normal.\n",mapName.c_str());
    return(AM_NORMALCOEFFICIENTS);
  }

  float orientationCoefficient = getOrientationType(orientationType);
  
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    // start creating parameters
    pcl::PointCloud<pcl::Normal>::Ptr current_normals;
    if(!pyramid->getNormals(i,current_normals))
    {
      printf("[ERROR]: Something went wrong! Can't get normals for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    pcl::PointIndices::Ptr current_indices;
    if(!pyramid->getIndices(i,current_indices))
    {
      printf("[ERROR]: Something went wrong! Can't get indices for level %d!\n",i);
      return(AM_CUSTOM);
    }    
    
    int current_width = pyramid->getWidth(i);
    if(current_width <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get width for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    int current_height = pyramid->getHeight(i);
    if(current_height <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get height for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat current_map;
    orientationMap(current_normals,current_indices,current_width,current_height,a,b,c,orientationCoefficient,current_map);
    
    if(!pyramid->setFeatureMap(i,current_map))
    {
     printf("[ERROR]: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramid->combinePyramid();
  
  if(!pyramid->getMap(map))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  
  return(AM_OK);
}

int RelativeSurfaceOrientationMap::calculatePyramidSimple()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Simple pyramid started.\n",mapName.c_str());
  
  SimplePyramid::Ptr pyramid( new SimplePyramid() );
  
  pyramid->setStartLevel(0);
  pyramid->setMaxLevel(4);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  pyramid->setCombinationType(combination_type);
  
  pyramid->setCloud(cloud);
  pyramid->setIndices(indices);
  pyramid->setNormals(normals);
  
  pyramid->buildDepthPyramid();
  pyramid->print();

  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  refineMap();
  
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int RelativeSurfaceOrientationMap::calculatePyramidItti()
{
  
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Itti pyramid started.\n",mapName.c_str());
  
  IttiPyramid::Ptr pyramid( new IttiPyramid() );
  
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  pyramid->setCombinationType(combination_type);
  
  pyramid->setLowestC(2);
  pyramid->setHighestC(4);
  pyramid->setSmallestCS(3);
  pyramid->setLargestCS(4);
  
  pyramid->setChangeSign(false);
  
  pyramid->setCloud(cloud);
  pyramid->setIndices(indices);
  pyramid->setNormals(normals);
  
  pyramid->buildDepthPyramid();
  pyramid->print();

  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  refineMap();
  
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int RelativeSurfaceOrientationMap::calculatePyramidFrintrop()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Frintrop pyramid started.\n",mapName.c_str());
  
  FrintropPyramid::Ptr pyramid( new FrintropPyramid() );
  
  pyramid->setStartLevel(0);
  pyramid->setMaxLevel(6);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  pyramid->setCombinationType(combination_type);
  
  std::vector<int> R;
  R.resize(2); R.at(0) = 3; R.at(1) = 7;
  pyramid->setR(R);
  pyramid->setOnSwitch(true);
  
  pyramid->setCloud(cloud);
  pyramid->setIndices(indices);
  pyramid->setNormals(normals);
  
  pyramid->buildDepthPyramid();
  pyramid->print();

  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  cv::Mat map_on;
  map.copyTo(map_on);
  
  float maxIntensityValue = pyramid->getMaxMapValue();
  
  //OFF pyramid
  pyramid->setOnSwitch(false);
  
  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  cv::Mat map_off;
  map.copyTo(map_off);
  
  maxIntensityValue = std::max(maxIntensityValue,pyramid->getMaxMapValue());
  map = map_on + map_off;
  EPUtils::normalize(map,EPUtils::NT_NONE,maxIntensityValue);
  EPUtils::normalize(map,normalization_type);
  
  refineMap();
  
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

float RelativeSurfaceOrientationMap::getOrientationType(int orientationType_)
{
  switch(orientationType_)
  {
    case AM_HORIZONTAL:
      return(0.0);
    case AM_VERTICAL:
      return(1.0);
  }
  return(0.0);
}

} //namespace AttentionModule