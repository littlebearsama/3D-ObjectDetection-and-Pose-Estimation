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


#ifndef RELATIVE_SURFACE_ORIENATION_HPP
#define RELATIVE_SURFACE_ORIENATION_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

enum OrientationTypes
{
  AM_VERTICAL      = 0,
  AM_HORIZONTAL,
};

/**
 * class for relative surface orientation saliency map
 * */

class RelativeSurfaceOrientationMap: public BaseMap
{
public:
  RelativeSurfaceOrientationMap();
  ~RelativeSurfaceOrientationMap();
  
  void setOrientationNormal(pcl::Normal orientation_normal_);
  bool getOrientationNormal(pcl::Normal &orientation_normal_);
  
  void setOrientationType(int orientationType_);
  int getOrientationType();
  
  void setNormalThreshold(float normal_threshold_);
  float getNormalThreshold();
  
  /**
   * calculates single surface height map
   * */
  
  virtual int calculate();
  
  virtual void reset();
  virtual void print();
  
private:
  pcl::Normal        orientation_normal;
  int                orientationType;
  float              normal_threshold;
  
  bool               haveOrientationNormal;
  
  float getOrientationType(int orientationType_);
  void orientationMap(pcl::PointCloud<pcl::Normal>::Ptr normals_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
		      float a, float b, float c, float orientationCoefficient, cv::Mat &map_cur);
  
protected:
  virtual int checkParameters();
  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);
};

} // namespace AttentionModule

#endif //RELATIVE_SURFACE_ORIENATION_HPP