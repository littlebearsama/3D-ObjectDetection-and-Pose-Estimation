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
 * @file AddGroundTruth.h
 * @author Richtsfeld, Potapova
 * @date Dezember 2012
 * @version 0.1
 * @brief Add ground truth to relations.
 */

#ifndef ADD_GROUND_TRUTH_H
#define ADD_GROUND_TRUTH_H

#include <stdio.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "opencv2/opencv.hpp"

#include "SurfaceModel.hpp"

namespace surface
{

class AddGroundTruth
{
public:

private:
  pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud;            ///< Input cloud
  bool have_cloud;
  int width, height;

  std::vector<SurfaceModel::Ptr> surfaces;
  bool have_surfaces;
  
  std::vector<Relation> relations;
  bool have_relations;
  
  bool computed;
  
public:
  AddGroundTruth();
  ~AddGroundTruth();

  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud);

  /** Set relations **/
  void setSurfaces(std::vector<SurfaceModel::Ptr> _surfaces);
  
  /** Set relations **/
  void setRelations(std::vector<surface::Relation> _relations);

  /** Return modified surfaces **/
  inline std::vector<SurfaceModel::Ptr> getSurfaces();

  /** Return modified surfaces **/
  inline std::vector<Relation> getRelations();

  /** Add ground truth of 'type' to the relations **/
  void compute(int type = 1);
  
};

inline std::vector<SurfaceModel::Ptr> AddGroundTruth::getSurfaces()
{
  return surfaces;
}

inline std::vector<Relation> AddGroundTruth::getRelations()
{
  return relations;
}

} //--END--

#endif

