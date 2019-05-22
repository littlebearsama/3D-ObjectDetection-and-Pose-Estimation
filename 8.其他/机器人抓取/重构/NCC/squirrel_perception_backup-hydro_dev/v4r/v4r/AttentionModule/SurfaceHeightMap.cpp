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


#include "SurfaceHeightMap.hpp"
#include <sys/time.h>

namespace AttentionModule
{

SurfaceHeightSaliencyMap::SurfaceHeightSaliencyMap():
BaseMap()
{
  reset();
}

SurfaceHeightSaliencyMap::~SurfaceHeightSaliencyMap()
{
}

void SurfaceHeightSaliencyMap::reset()
{
  BaseMap::reset();
  
  coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
  distance_from_top = 0;
  max_distance = 0.005;
  heightType = AM_TALL;
  
  haveModelCoefficients = false;
  
  mapName = "SurfaceHeightSaliencyMap";
}

void SurfaceHeightSaliencyMap::print()
{
  BaseMap::print();
  printf("[%s]: coefficients       [ ",mapName.c_str());
  for(size_t i = 0; i < coefficients->values.size(); ++i)
  {
    printf("%f, ",coefficients->values.at(i));
  }
  printf("]\n");
  printf("[%s]: distance_from_top  = %f\n",mapName.c_str(),distance_from_top);
  printf("[%s]: max_distance       = %f\n",mapName.c_str(),max_distance);
  printf("[%s]: heightType         = %d\n",mapName.c_str(),heightType);
}

void SurfaceHeightSaliencyMap::setModelCoefficients(pcl::ModelCoefficients::Ptr coefficients_)
{
  coefficients = coefficients_;
  haveModelCoefficients = true;
  calculated = false;
  printf("[INFO]: %s: model coefficients are [ ",mapName.c_str());
  for(size_t i = 0; i < coefficients->values.size(); ++i)
  {
    printf("%f, ",coefficients->values.at(i));
  }
  printf("]\n");
}

void SurfaceHeightSaliencyMap::setDistanceFromTop(float distance_from_top_)
{
  distance_from_top = distance_from_top_;
  calculated = false;
  printf("[INFO]: %s: distance_from_top is set to: %f\n",mapName.c_str(),distance_from_top);
}

void SurfaceHeightSaliencyMap::setMaxDistance(int max_distance_)
{
  max_distance = max_distance_;
  calculated = false;
  printf("[INFO]: %s: max_distance is set to: %f\n",mapName.c_str(),max_distance);
}

void SurfaceHeightSaliencyMap::setHeightType(int heightType_)
{
  heightType = heightType_;
  calculated = false;
  printf("[INFO]: %s: heightType is set to: %d\n",mapName.c_str(),heightType);
}

bool SurfaceHeightSaliencyMap::getModelCoefficients(pcl::ModelCoefficients::Ptr &coefficients_)
{
  if(!haveModelCoefficients)
    return(false);
  
  coefficients_ = coefficients;
  return(true);
}

float SurfaceHeightSaliencyMap::getDistanceFromTop()
{
  return(distance_from_top);
}

int SurfaceHeightSaliencyMap::getMaxDistance()
{
  return(max_distance);
}

int SurfaceHeightSaliencyMap::getHeightType()
{
  return(heightType);
}

int SurfaceHeightSaliencyMap::checkParameters()
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
  
  if(!haveModelCoefficients)
  {
    printf("[ERROR]: %s: Seems like there are no coefficients.\n",mapName.c_str());
    return(AM_POINTCLOUD);
  }
  
  if( (width == 0) || (height == 0) )
  {
    printf("[ERROR]: %s: Seems like image size is wrong.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if(coefficients->values.size() != 4)
  {
    printf("[ERROR]: %s: Wrong number of coefficients, this shouldn't happent (there is a check before!) --> BUG.\n",mapName.c_str());
    return(AM_PLANECOEFFICIENTS);
  }

  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }
  
  return(AM_OK);
}

int SurfaceHeightSaliencyMap::calculate()
{
  switch(heightType)
  {
    case AM_SHORT:
      return(calculateHeightMap());
    case AM_TALL:
      return(calculateHeightMap());
    case AM_DISTANCE:
      return(calculatePointDistanceMap());
    default:
      return(calculatePointDistanceMap());
  }
  return(calculatePointDistanceMap());
}

int SurfaceHeightSaliencyMap::calculateHeightMap()
{
  calculated = false;
  
  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  float heightCoefficient = getHeightCoefficient(heightType);
  if(heightCoefficient < 0)
  {
    printf("[ERROR]: %s: Sorry, but you can select between TALL and SHORT types of height map ONLY!\n",mapName.c_str());
    return(AM_OK);
  }
  
  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  calculateHeightMap(cloud,indices,width,height,heightCoefficient,map);

  cv::blur(map,map,cv::Size(filter_size,filter_size));

  refineMap();
  
  EPUtils::normalize(map,normalization_type);
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

void SurfaceHeightSaliencyMap::calculateHeightMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
						  float heightCoefficient, cv::Mat &map_cur)
{
//   for(int i = 0; i < cloud_cur->points.size(); ++i)
//   {
//     std::cerr << "(" << cloud_cur->points.at(i).x << "," << cloud_cur->points.at(i).y << "," << cloud_cur->points.at(i).z << ") ";
//   }
//   std::cerr << std::endl << std::endl;
  
  // calculate projections on the plane
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_projected (new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<float> distances;
  // calculate coordinates of the projections and normalized distances to the plane
  EPUtils::ProjectPointsOnThePlane(coefficients,cloud_cur,points_projected,distances,indices_cur);

  if(cloud_cur->points.size() < 10)
  {
    map_cur = cv::Mat_<float>::zeros(image_height,image_width);
    return;
  }
  
  // create kd-tree to search for neighbouring points
  pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud(points_projected);

  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  
  // max distance from the plane in the current line of points
  std::vector<float> max_distance;
  max_distance.resize(distances.size(),0);
  // are point already used
  std::vector<bool> used_points;
  used_points.resize(distances.size(),false);

  float radius = 0.009;
  float shift_from_plane = 0.05;
  
  for(unsigned int i = 0; i < used_points.size(); ++i)
  {
    if(used_points.at(i))
    {
      continue;
    }
    
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
      
    pcl::PointXYZRGB searchPoint = points_projected->points.at(i);
//     std::cerr << searchPoint << std::endl;
    //std::cerr << cloud->points.at(indices_cur->indices.at(i)) << std::endl;
    
    // if we found points close to the given point
    if(kdtree.radiusSearch(searchPoint,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance))
    {
      float max_distance_val = 0;
      float average_distance = 0;
      int average_counter = 0;
      // find point with biggest distance to plane
      for(unsigned int j = 0; j < pointIdxRadiusSearch.size(); ++j)
      {
        int index = pointIdxRadiusSearch.at(j);
        average_distance += distances.at(index);
        average_counter += 1;
        if(!used_points.at(index))
        {
	  if((distances.at(index) > max_distance_val) && (distances.at(index) > shift_from_plane))
	  {
	    max_distance_val = distances.at(index);
          }
        }
      }
      if(average_counter>0)
      {
        average_distance /= average_counter;
      }
	
      // set biggest distance
      for(unsigned int j = 0; j < pointIdxRadiusSearch.size(); ++j)
      {
	int index = pointIdxRadiusSearch.at(j);
	if(!used_points.at(index))
	{
	  max_distance.at(index) = max_distance_val;
	  //max_distance.at(index) = average_distance;
          used_points.at(index) = true;
	}
      }
    }
  }

  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  for(unsigned int i = 0; i < max_distance.size(); ++i)
  {
    int idx = indices_cur->indices.at(i);
    int c = idx % image_width;
    int r = idx / image_width;
    
    float value = (max_distance.at(i)-shift_from_plane)/(1.0-shift_from_plane);
    //float value = max_distance.at(i);
    if(value > 0)
    {
      float t1 = 1.0-heightCoefficient;
      float t2 = heightCoefficient;
      value = t1*value + t2*(1-value);
      map_cur.at<float>(r,c) = value;
    }
    else
    {
      map_cur.at<float>(r,c) = 0;
    }
  }
}

int SurfaceHeightSaliencyMap::calculatePointDistanceMap()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation started.\n",mapName.c_str());

  rt_code = calculatePointDistanceMap(cloud,indices,width,height,map);
  if(rt_code != AM_OK)
    return(rt_code);

  cv::blur(map,map,cv::Size(filter_size,filter_size));
  
  refineMap();
  
  EPUtils::normalize(map,normalization_type);
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int SurfaceHeightSaliencyMap::calculatePointDistanceMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
						         cv::Mat &map_cur)
{
  // Retrieve Ground Plane Coefficients
  float a = coefficients->values.at(0);
  float b = coefficients->values.at(1);
  float c = coefficients->values.at(2);
  float d = coefficients->values.at(3);

  float max_dist = 0;
  cv::Mat distance_map = cv::Mat_<float>::zeros(image_height,image_width);

  for(unsigned int pi = 0; pi < indices_cur->indices.size(); ++pi)
  {
    int index = indices_cur->indices.at(pi);
    pcl::PointXYZRGB current_point = cloud_cur->points.at(index);

    float dist = pcl::pointToPlaneDistance(current_point,a,b,c,d);

    if(dist > max_dist)
      max_dist = dist;

    int xx = index % image_width;
    int yy = index / image_width;
    distance_map.at<float>(yy,xx) = dist;
  }

  if(max_dist < max_distance)
  {
    map_cur = cv::Mat_<float>::zeros(image_height,image_width);
    return(AM_OK);
  }

  float d0 = distance_from_top * max_dist;
  float a_param = -1.0/((max_dist-d0)*(max_dist-d0));
  float b_param = 2.0/(max_dist-d0);

  map_cur = cv::Mat_<float>::zeros(image_height,image_width);

  for(int r = 0; r < image_height; ++r)
  {
    for(int c = 0; c < image_width; ++c)
    {
      float dist = distance_map.at<float>(r,c);
      if(dist > 0)
      {
        map_cur.at<float>(r,c) = (a_param * dist * dist + b_param * dist);

        if(map_cur.at<float>(r,c) < 0)
        {
          map_cur.at<float>(r,c) = 0;
        }
      }
    }
  }
  
  return(AM_OK);
}

int SurfaceHeightSaliencyMap::calculatePyramidSimple()
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

int SurfaceHeightSaliencyMap::calculatePyramidItti()
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

int SurfaceHeightSaliencyMap::calculatePyramidFrintrop()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Frintrop pyramid started.\n",mapName.c_str());
  
  //ON pyramid
  
  FrintropPyramid::Ptr pyramid( new FrintropPyramid() );
  
  pyramid->setStartLevel(2);
  pyramid->setMaxLevel(4);
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
  
int SurfaceHeightSaliencyMap::combinePyramid(BasePyramid::Ptr pyramid)
{
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    // start creating parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud;
    if(!pyramid->getCloud(i,current_cloud))
    {
      printf("[ERROR]: Something went wrong! Can't get cloud for level %d!\n",i);
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
    
//     cv::Mat depth;
//     EPUtils::pointCloud_2_depth(depth,current_cloud,current_width,current_height,current_indices);
//     cv::imshow("depth",depth);
//     cv::waitKey(-1);
    
    cv::Mat current_map;
    if( (heightType == AM_SHORT) || (heightType == AM_TALL) )
    {
      float heightCoefficient = getHeightCoefficient(heightType);
      if(heightCoefficient < 0)
      {
        printf("[ERROR]: %s: Sorry, but you can select between TALL and SHORT types of height map ONLY for level %d!\n",mapName.c_str(),i);
        return(AM_OK);
      }
      calculateHeightMap(current_cloud,current_indices,current_width,current_height,heightCoefficient,current_map);
    }
    else if (heightType == AM_DISTANCE)
    {
      int rt_code = calculatePointDistanceMap(current_cloud,current_indices,current_width,current_height,current_map);
      if(rt_code != AM_OK)
      {
        return(rt_code);
      }
    }
    else
    {  
      printf("[ERROR]: Something went wrong! Can't find height type for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
//     cv::imshow("current_map",current_map);
//     cv::Mat upscaled_current_map;
//     EPUtils::upscalePyramid(current_map,upscaled_current_map);
//     cv::imshow("current_map",upscaled_current_map);
//     cv::waitKey(-1);
    
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

float SurfaceHeightSaliencyMap::getHeightCoefficient(int heightType_)
{
  switch(heightType_)
  {
    case AM_SHORT:
      return(1.0);
    case AM_TALL:
      return(0.0);
    case AM_DISTANCE:
      return(-1.0);
    default:
      return(-1.0);
  }
  return(-1.0);
}

} //namespace AttentionModule