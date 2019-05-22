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


#ifndef SYMMETRY3D_MAP_HPP
#define SYMMETRY3D_MAP_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

struct MiddlePoint {
    int num;
    float distance;
    pcl::Normal normal;
    pcl::PointXYZ point;
    MiddlePoint();
  };
  
MiddlePoint::MiddlePoint()
{
  num = 0;
  distance = 0;
  normal.normal[0] = 0;
  normal.normal[1] = 0;
  normal.normal[2] = 0;
  point.x = 0;
  point.y = 0;
  point.z = 0;
}
  
class Symmetry3DMap: public BaseMap
{
private:
  int R;
  
  void createLookUpMap(pcl::PointIndices::Ptr indices_cur, unsigned int width_cur, unsigned int height_cur, cv::Mat &lookupMap);//
  void symmetry3DMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur, pcl::PointCloud<pcl::Normal>::Ptr normals_cur, pcl::PointIndices::Ptr indices_cur, 
		     int image_width, int image_height, int R_cur, cv::Mat &map_cur, int filter_size_);//
  
public:
  Symmetry3DMap();//
  ~Symmetry3DMap();//
  
  void setR(int R_);//
  int getR();//
  
  virtual int calculate();
  
  virtual void reset();//
  virtual void print();//
  
protected:
  virtual int checkParameters();//
  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);
};

} //namespace AttentionModule

#endif //SYMMETRY3D_MAP_HPP
