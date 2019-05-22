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


#ifndef FRINTROP_SALIENCYMAP_HPP
#define FRINTROP_SALIENCYMAP_HPP

#include "BaseMap.hpp"
#include "ColorMap.hpp"

namespace AttentionModule
{

class FrintropSaliencyMap: public BaseMap
{
public:
  FrintropSaliencyMap();//
  ~FrintropSaliencyMap();//
  
  void setNumberOfOrientations(int numberOfOrientations_);//
  int getNumberOfOrientations();//
  
  virtual int calculate();
  virtual void reset();//
  virtual void print();//
  
private:
  cv::Mat R, G, B, Y, I;
  int     numberOfOrientations;
  
  void initializePyramid(FrintropPyramid::Ptr pyramid, cv::Mat &IM, bool onSwitch_);//
  void initializePyramid(SimplePyramid::Ptr pyramid, cv::Mat &IM);//
  void createColorChannels();//
  int createFeatureMapsI(FrintropPyramid::Ptr pyramid);//
  int createFeatureMapsO(SimplePyramid::Ptr pyramid, float angle);//
  
protected:  
  virtual int checkParameters();//
};

} // AttentionModule

#endif //FRINTROP_SALIENCYMAP_HPP
  
