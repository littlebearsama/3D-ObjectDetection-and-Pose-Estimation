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


#include "SurfaceCurvatureMap.hpp"

namespace AttentionModule
{

SurfaceCurvatureMap::SurfaceCurvatureMap():
BaseMap()  
{
  reset();
}

SurfaceCurvatureMap::~SurfaceCurvatureMap()
{
}

void SurfaceCurvatureMap::reset()
{
  BaseMap::reset();
  
  curvatureType = AM_CONVEX;

  mapName = "SurfaceCurvatureMap";
}

void SurfaceCurvatureMap::print()
{
  BaseMap::print();
  printf("[%s]: curvatureType      = %d\n",mapName.c_str(),curvatureType);
}

int SurfaceCurvatureMap::checkParameters()
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

void SurfaceCurvatureMap::setCurvatureType(int curvatureType_)
{
  curvatureType = curvatureType_;
  calculated = false;
  printf("[INFO]: %s: curvatureType: %d.\n",mapName.c_str(),curvatureType);
}

int SurfaceCurvatureMap::getCurvatureType()
{
  return(curvatureType);
}

int SurfaceCurvatureMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  float curvatureCoefficient = getCurvatureCoefficient(curvatureType);

  curvatureMap(normals,indices,width,height,curvatureCoefficient,map);

  cv::blur(map,map,cv::Size(filter_size,filter_size));

  refineMap();
  
  EPUtils::normalize(map,normalization_type);
  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());

  return(AM_OK);
}

void SurfaceCurvatureMap::curvatureMap(pcl::PointCloud<pcl::Normal>::Ptr normals_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
		                      float curvatureCoefficient, cv::Mat &map_cur)
{
  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  for(unsigned int pi = 0; pi < indices_cur->indices.size(); ++pi)
  {
    int idx = indices_cur->indices.at(pi);
    int r = idx / image_width;
    int c = idx % image_width;
    
    //if(mask.at<uchar>(r,c))
    {
      float nx = normals_cur->points.at(pi).normal[0];
      float ny = normals_cur->points.at(pi).normal[1];
      float nz = normals_cur->points.at(pi).normal[2];

      if(std::isnan(nx) || std::isnan(ny) || std::isnan(nz))
      {
        continue;
      }
      
      float value = normals_cur->points.at(pi).curvature;
      float t1 = 1.0-curvatureCoefficient;
      float t2 = curvatureCoefficient;
      value = t1*value + t2*(1.0-value);
      
      map_cur.at<float>(r,c) = value;
    }
  }
}

float SurfaceCurvatureMap::getCurvatureCoefficient(int curvatureType_)
{
  switch(curvatureType_)
  {
    case AM_FLAT:
      return(1.0);
    case AM_CONVEX:
      return(0.0);
  }
  return(0.0);
}

} //namespace AttentionModule