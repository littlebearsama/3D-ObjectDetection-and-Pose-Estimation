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


#ifndef ATTENTION_MODULE_EXAMPLES_UTILS_HPP
#define ATTENTION_MODULE_EXAMPLES_UTILS_HPP

#include "v4r/EPUtils/EPUtils.hpp"
#include "v4r/SurfaceClustering/ZAdaptiveNormals.hh"

int preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients, 
                      pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr object_indices_in_the_hull, bool useStandartNormals = false);
int preparePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, pcl::PointIndices::Ptr object_indices, 
		      bool useStandartNormals = false);

bool checkIsNaN(const pcl::PointXYZRGB &p);

#endif //ATTENTION_MODULE_EXAMPLES_UTILS_HPP