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


#ifndef SURFACE_CURVATURE_HPP
#define SURFACE_CURVATURE_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

/**
 * surface curvature saliency map
 * */

enum CurvatureTypes
{
  AM_FLAT      = 0,
  AM_CONVEX,
};

class SurfaceCurvatureMap: public BaseMap
{
public:
  SurfaceCurvatureMap();
  ~SurfaceCurvatureMap();
  
  void setCurvatureType(int curvatureType_);
  int getCurvatureType();
  
  /**
   * calculates single curvature map
   * */
  
  virtual int calculate();
  
  virtual void reset();//
  virtual void print();//
  
private:
  
  int curvatureType;
  float getCurvatureCoefficient(int curvatureType_);
  void curvatureMap(pcl::PointCloud<pcl::Normal>::Ptr normals_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
	           float curvatureCoefficient, cv::Mat &map_cur);
  
protected:
  
  virtual int checkParameters();//
};

} // namespace AttentionModule

#endif //SURFACE_CURVATURE_HPP