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

bool checkIsNaN(const pcl::PointXYZRGB &p)
{
  if(std::isnan(p.x) ||
     std::isnan(p.y) ||
     std::isnan(p.z))
  {
    return(true);
  }
  return(false);
}

int preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, 
		      pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr object_indices_in_the_hull, bool useStandartNormals)
{
  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
  {
    return(pclAddOns::FILTER);
  }
  
  // segment plane
  pcl::PointIndices::Ptr plane_indices(new pcl::PointIndices());
  pcl::PointIndices::Ptr objects_indices(new pcl::PointIndices());
  if(!pclAddOns::SegmentPlane<pcl::PointXYZRGB>(cloud,indices,plane_indices,objects_indices,coefficients))
  {
    return(pclAddOns::SEGMENT);
  }
  
  pcl::PointIndices::Ptr object_indices_in_the_hull_all(new pcl::PointIndices());
  if(!pclAddOns::ConvexHullExtract<pcl::PointXYZRGB>(cloud,plane_indices,objects_indices,object_indices_in_the_hull_all,coefficients))
  {
    return(pclAddOns::CONVEXHULL);
  }
  
  //calculate point cloud normals
  if(useStandartNormals)
  {
    if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,object_indices_in_the_hull_all,normals))
    {
      return(pclAddOns::NORMALS);
    }
    object_indices_in_the_hull->indices = object_indices_in_the_hull_all->indices;
  }
  else
  {
    // calcuate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_all(new pcl::PointCloud<pcl::Normal>);
    surface::ZAdaptiveNormals<pcl::PointXYZRGB>::Parameter param;
    param.adaptive = true;
    surface::ZAdaptiveNormals<pcl::PointXYZRGB> nor(param);
    nor.setInputCloud(cloud);
    nor.compute();
    nor.getNormals(normals_all);
    
    normals->points.reserve(object_indices_in_the_hull_all->indices.size());
    object_indices_in_the_hull->indices.reserve(object_indices_in_the_hull_all->indices.size());
    
    for(unsigned int i = 0; i < object_indices_in_the_hull_all->indices.size(); ++i)
    {
      int idx = object_indices_in_the_hull_all->indices.at(i);
      
      if( checkIsNaN(cloud->points.at(idx)) )
	continue;
      
      normals->points.push_back(normals_all->points.at(idx));
      object_indices_in_the_hull->indices.push_back(idx);
    }
    
  }
  
  return(0);
}

int preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr object_indices, bool useStandartNormals)
{
  // create filtered point cloud
  pcl::PointIndices::Ptr indices(new pcl::PointIndices());
  if(!pclAddOns::FilterPointCloud<pcl::PointXYZRGB>(cloud,indices))
  {
    return(pclAddOns::FILTER);
  }
  
  if(useStandartNormals)
  {
    if(!pclAddOns::ComputePointNormals<pcl::PointXYZRGB>(cloud,indices,normals))
    {
      return(pclAddOns::NORMALS);
    }
    object_indices->indices = indices->indices;
  }
  else
  {
    // calcuate normals
    pcl::PointCloud<pcl::Normal>::Ptr normals_all(new pcl::PointCloud<pcl::Normal>);
    surface::ZAdaptiveNormals<pcl::PointXYZRGB>::Parameter param;
    param.adaptive = true;
    surface::ZAdaptiveNormals<pcl::PointXYZRGB> nor(param);
    nor.setInputCloud(cloud);
    nor.compute();
    nor.getNormals(normals_all);
    
    normals->points.reserve(indices->indices.size());
    object_indices->indices.reserve(indices->indices.size());
    
    for(unsigned int i = 0; i < indices->indices.size(); ++i)
    {
      int idx = indices->indices.at(i);
      
      if( checkIsNaN(cloud->points.at(idx)) )
	continue;
      
      normals->points.push_back(normals_all->points.at(idx));
      object_indices->indices.push_back(idx);
    }
    
  }
  
  return(0);
}