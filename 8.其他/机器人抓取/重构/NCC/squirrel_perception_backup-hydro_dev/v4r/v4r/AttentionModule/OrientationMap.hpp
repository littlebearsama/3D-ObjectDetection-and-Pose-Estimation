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


#ifndef ORIENTATION_MAP_HPP
#define ORIENTATION_MAP_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

/**
 * parameters for orientation saliency map
 * */

class OrientationSaliencyMap: public BaseMap
{
public:
  OrientationSaliencyMap();//
  ~OrientationSaliencyMap();//
  
  void setAngle(float angle_);//
  float getAngle();//
  
  void setBandwidth(float bandwidth_);//
  float getBandwidth();//
  
  float getMaxSum();//
  
  virtual int calculate();
  virtual void reset();//
  virtual void print();//
  
private:
  float angle;
  float max_sum;
  float bandwidth;
  
  void orientationMap(cv::Mat &image_cur, int image_width, int image_height, float angle, float max_sum, float bandwidth, cv::Mat &map_cur);
  
protected:  
  virtual int checkParameters();//
  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);

};

} // namespace AttentionModule

#endif //ORIENTATION_MAP_HPP