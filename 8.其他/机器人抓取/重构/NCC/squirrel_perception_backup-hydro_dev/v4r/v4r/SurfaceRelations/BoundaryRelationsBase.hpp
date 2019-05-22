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
 * @file BoundaryRelationsBase.hpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Base class to calculate boundary relations between patches.
 */

#ifndef BOUNDARY_RELATIONS_BASE_H
#define BOUNDARY_RELATIONS_BASE_H

#include <stdio.h>

#include <opencv2/opencv.hpp>

#include <pcl/filters/project_inliers.h>
#include <pcl/io/pcd_io.h>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"


namespace surface
{

class BoundaryRelationsBase
{
public:

protected:

  bool computed;
  bool have_cloud;
  bool have_normals;
  bool have_boundary;

  pcl::PointIndices::Ptr indices;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;               ///< Input cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;                  ///< Normals (set from outside or from surfaces)
  std::vector<surface::neighboringPair> boundary;

  int width, height;

  bool checkNaN(const pcl::PointXYZRGB p)
  {
    if(std::isnan(p.x) ||
       std::isnan(p.y) ||
       std::isnan(p.z))
    {
      return(true);
    }
    return(false);
  }
  
public:

  typedef boost::shared_ptr<BoundaryRelationsBase> Ptr;

  BoundaryRelationsBase();
  ~BoundaryRelationsBase();

  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud);
  //sets normals
  void setNormals(pcl::PointCloud<pcl::Normal>::Ptr _normals);

  // sets indices
  void setBoundary(std::vector<surface::neighboringPair> &_boundary);

  /** Compare patches **/
  virtual surface::meanVal compute();

};

}

#endif //BOUNDARY_RELATIONS_BASE_H

