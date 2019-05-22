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


#include "pyramidFrintrop.hpp"

namespace AttentionModule
{

FrintropPyramid::FrintropPyramid():
BasePyramid()
{
  reset();
}

FrintropPyramid::~FrintropPyramid()
{
}

void FrintropPyramid::reset()
{
  BasePyramid::reset();
  
  R.resize(2); R.at(0) = 3; R.at(1) = 7;
  onSwitch = true;
  
  pyramidName = "FrintropPyramid";
}

void FrintropPyramid::setR(std::vector<int> &R_)
{
  R = R_;
  calculated = false;
  haveImagePyramid = false;
  printf("[INFO]: %s: R is set to: [ ",pyramidName.c_str());
  for(size_t i = 0; i < R.size(); ++i)
  {
    printf("%d ",R.at(i));
  }
  printf("]\n");
}

std::vector<int> FrintropPyramid::getR()
{
  return(R);
}

void FrintropPyramid::setOnSwitch(bool onSwitch_)
{
  onSwitch = onSwitch_;
  calculated = false;
  //haveImagePyramid = false;
  printf("[INFO]: %s: onSwitch is set to: %s\n",pyramidName.c_str(),onSwitch ? "yes" : "no");
}

bool FrintropPyramid::getOnSwitch()
{
  return(onSwitch);
}

void FrintropPyramid::print()
{
  BasePyramid::print();
  
  printf("[PyramidParameters]: onSwitch             = %s\n",onSwitch ? "yes" : "no");
  printf("[PyramidParameters]: R                    = [ ");
  for(unsigned int i = 0; i < R.size(); ++i)
  {
    printf("%d ",R.at(i));
  }
  printf("]\n");
}

void FrintropPyramid::combinePyramid(bool standard)
{
  calculated = false;
  
  int number_of_features = (max_level-start_level+1)*R.size();
  
  pyramidConspicuities.resize(number_of_features);
  
  std::vector<int> trueLevel;
  
  for(int s = start_level; s <= max_level; ++s)
  {
    for(unsigned int r = 0; r < R.size(); ++r)
    {
      cv::Mat kernel = cv::Mat_<float>::ones(R.at(r),R.at(r));
      kernel = kernel / (R.at(r)*R.at(r));
      
      int current = (R.size()-0)*(s-start_level)+(r-0);
      if (current < number_of_features)
      {
        cv::Mat temp;
        filter2D(pyramidFeatures.at(s),temp,pyramidFeatures.at(s).depth(),kernel);

        if(onSwitch)
	{
          temp = pyramidFeatures.at(s) - temp;
	}
        else
	{
          temp = temp - pyramidFeatures.at(s);
	}
  
        cv::max(temp,0.0,pyramidConspicuities.at(current));
	EPUtils::normalize(pyramidConspicuities.at(current),normalization_type);
	trueLevel.push_back(s);
      }
    }
  }
  
  if(combination_type == AM_COMB_MUL)
  {
    map = cv::Mat_<float>::ones(pyramidImages.at(sm_level).rows,pyramidImages.at(sm_level).cols);
  }
  else
  {
    map = cv::Mat_<float>::zeros(pyramidImages.at(sm_level).rows,pyramidImages.at(sm_level).cols);
  }
  
  for (int i=0; i < number_of_features; ++i)
  {
    cv::Mat temp;
    if(standard)
    {
      cv::resize(pyramidConspicuities.at(i),temp,cv::Size(pyramidImages.at(sm_level).cols,pyramidImages.at(sm_level).rows));
    }
    else
    {
      EPUtils::scaleImage(pyramidImages,pyramidConspicuities.at(i),temp,trueLevel.at(i),sm_level);
    }
//     cv::imshow("temp",temp);
//     cv::waitKey(-1);
    combineConspicuityMaps(map,temp);
  }
  
  double maxValue, minValue;
  cv::minMaxLoc(map,&minValue,&maxValue);
  max_map_value = maxValue;
  
  EPUtils::normalize(map,normalization_type);
  
//   cv::minMaxLoc(map,&minValue,&maxValue);
//   std::cerr << maxValue << std::endl;
//   cv::imshow("map5",map);
//   cv::waitKey(-1);
  
  calculated = true;
}

}