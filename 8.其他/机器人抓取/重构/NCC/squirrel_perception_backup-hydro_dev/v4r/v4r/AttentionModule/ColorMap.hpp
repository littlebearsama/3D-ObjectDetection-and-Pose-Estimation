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


#ifndef COLOR_MAP_HPP
#define COLOR_MAP_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

/**
 * parameters for color saliency map
 * */

class ColorSaliencyMap: public BaseMap
{
public:
  ColorSaliencyMap();
  ~ColorSaliencyMap();
  
  void setUseLAB(bool useLAB_);
  bool getUseLAB();
  
  void setColor(cv::Scalar color_);
  cv::Scalar getColor();
/**
* calculates single color map
* */
  virtual int calculate();
  virtual void reset();
  virtual void print();
  
private:
  bool              useLAB;
  cv::Scalar        color;
  
  float getMaxColorDistance(float &r_color, float &g_color, float &b_color, float &a_color);
  void LabColorMap(cv::Mat &image_cur, int image_width, int image_height, float max_dist, float a_color, float b_color, cv::Mat &map_cur);
  void RGBColorMap(cv::Mat &image_cur, int image_width, int image_height, float max_dist, float r_color, float g_color, float b_color, cv::Mat &map_cur);

protected:  
  virtual int checkParameters();
  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);
};

} // namespace AttentionModule

#endif //COLOR_MAP_HPP