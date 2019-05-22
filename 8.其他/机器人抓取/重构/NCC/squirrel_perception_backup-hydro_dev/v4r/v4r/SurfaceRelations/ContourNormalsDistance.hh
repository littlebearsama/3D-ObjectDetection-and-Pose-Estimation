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
 * @file ContourNormalsDistance.hh
 * @author Richtsfeld
 * @date February 2013
 * @version 0.1
 * @brief Calculate normal distance from two contours using a octree.
 */

#ifndef SURFACE_CONTOUR_NORMALS_DISTANCE_HH
#define SURFACE_CONTOUR_NORMALS_DISTANCE_HH

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/octree/octree.h>

#include "v4r/SurfaceClustering/PPlane.h"
#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/SurfaceUtils/EPBase.hpp"

namespace surface 
{


class ContourNormalsDistance: public EPBase
{
public:
  class Parameter
  {
  public:
    float pcntContourPoints;  // percentage of nearest contour points used 
    Parameter(float _pcntContourPoints=.2)
     : pcntContourPoints(_pcntContourPoints) {} 
  };

private:

  Parameter param;

  bool computeOctree(surface::SurfaceModel::Ptr &in1, surface::SurfaceModel::Ptr &in2,
                     float &cosDeltaAngle, float &distNormal, float &minDist, float &occlusion);

  float calculateOclusion(surface::SurfaceModel::Ptr &in1, surface::SurfaceModel::Ptr &in2,
                          int contIdx1, int contIdx2, int surf1NP, int surf2NP);

  void drawLine(int x1, int y1, int x2, int y2, std::vector<int> &_x, std::vector<int> &_y);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContourNormalsDistance(Parameter p=Parameter());
  ~ContourNormalsDistance();

  void setParameter(Parameter &p);

  bool compute(surface::SurfaceModel::Ptr &in1,
               surface::SurfaceModel::Ptr &in2,
               float &cosDeltaAngle,
               float &distNormal,
               float &minDist, 
               float &occlusion);


};

}

#endif

