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


#ifndef SURFACEHEIGHT_HPP
#define SURFACEHEIGHT_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

/**
 * parameters for surface height saliency map
 * */

enum SurfaceTypes
{
  AM_TALL      = 0,
  AM_SHORT,
  AM_DISTANCE,
};

class SurfaceHeightSaliencyMap: public BaseMap 
{
public:
  SurfaceHeightSaliencyMap();
  ~SurfaceHeightSaliencyMap();
  
  void setModelCoefficients(pcl::ModelCoefficients::Ptr coefficients_);
  bool getModelCoefficients(pcl::ModelCoefficients::Ptr &coefficients_);
  
  void setDistanceFromTop(float distance_from_top_);
  float getDistanceFromTop();
  
  void setMaxDistance(int max_distance_);
  int getMaxDistance();
  
  void setHeightType(int heightType_);
  int getHeightType();
  
/**
* calculates single surface height map
* */

  virtual int calculate();
  
  virtual void reset();
  virtual void print();
  
private:
  pcl::ModelCoefficients::Ptr coefficients;
  float                       distance_from_top;
  float                       max_distance;
  int                         heightType;
  
  bool                        haveModelCoefficients;
  
  float getHeightCoefficient(int heightType_);

  int calculateHeightMap();
  void calculateHeightMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
			  float heightCoefficient, cv::Mat &map_cur);
  int calculatePointDistanceMap();
  int calculatePointDistanceMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur, pcl::PointIndices::Ptr indices_cur, int image_width, int image_height, 
				 cv::Mat &map_cur);

protected:
  virtual int checkParameters();
  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);
};

} // namespace AttentionModule

#endif //SURFACEHEIGHT_HPP