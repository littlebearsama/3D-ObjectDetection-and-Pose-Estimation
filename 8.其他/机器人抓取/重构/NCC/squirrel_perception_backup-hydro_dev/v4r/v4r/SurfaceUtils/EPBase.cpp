/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
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

/**
 * @file EPBase.cpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Base class.
 */

#include "EPBase.hpp"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

EPBase::EPBase()
{
  have_cloud = false;
  have_normals = false;
  have_indices = false;
  
  computed = false;
  
  ClassName = "EPBase";
}

EPBase::~EPBase()
{
}

void EPBase::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if ( (_cloud->height<=1) || (_cloud->width<=1) || (!_cloud->isOrganized()) )
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::setInputCloud()]: Invalid point cloud (height must be > 1; width must be > 1; point cloud should be organized).",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  have_cloud = true;
  have_normals = false;
  have_indices = false;
  computed = false;
}

void EPBase::setNormals(const pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  if(!have_cloud)
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::setNormals()]: I suggest you first set the point cloud.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  if ( (_normals->height!=(unsigned int)height) || (_normals->width!=(unsigned int)width) )
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::setNormals()]: Invalid normals (not for this point cloud).",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  normals = _normals;
  have_normals = true;
}

void EPBase::setIndices(const pcl::PointIndices::Ptr &_indices)
{
  if(!have_cloud)
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::setNormals()]: I suggest you first set the point cloud.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  indices = _indices->indices;
  have_indices = true;
}

void EPBase::setIndices(const std::vector<int> &_indices)
{
  if(!have_cloud)
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::setNormals()]: I suggest you first set the point cloud.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  indices = _indices;
  have_indices = true;
}

void EPBase::compute()
{
  char* error_message = new char[200];
  sprintf(error_message,"[%s::setNormals()]: Why do you call me? I am just the base class!.",ClassName.c_str());
  throw std::runtime_error(error_message);
}

} // end surface












