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


#ifndef IKN_SALIENCYMAP
#define IKN_SALIENCYMAP

#include "BaseMap.hpp"

namespace AttentionModule
{

class IKNSaliencyMap: public BaseMap
{
public:
  IKNSaliencyMap();//
  ~IKNSaliencyMap();//
  
  void setWeights(int weightOfColor_, int weightOfIntensities_, int weightOfOrientations_);//
  void getWeights(int &weightOfColor_, int &weightOfIntensities_, int &weightOfOrientations_);//
  
  void setNumberOfOrientations(int numberOfOrientations_);//
  int getNumberOfOrientations();//

  virtual int calculate();
  virtual void reset();//
  virtual void print();//
  
private:
  
  cv::Mat R, G, B, Y, I;
  int     weightOfColor;
  int     weightOfIntensities;
  int     weightOfOrientations;
  int     numberOfOrientations;
  
  void initializePyramid(IttiPyramid::Ptr pyramid, cv::Mat &IM, bool changeSign_ = false);
  void createColorChannels();
  int createFeatureMapsI(IttiPyramid::Ptr pyramid);
  int createFeatureMapsO(IttiPyramid::Ptr pyramidO, float angle);
  int createFeatureMapsRG(IttiPyramid::Ptr pyramidR, IttiPyramid::Ptr pyramidG);
  
protected:  
  virtual int checkParameters();//
  
};

} // AttentionModule

#endif //IKN_SALIENCYMAP