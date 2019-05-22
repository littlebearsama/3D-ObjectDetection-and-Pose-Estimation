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


#include "pyramidSimple.hpp"

namespace AttentionModule
{

SimplePyramid::SimplePyramid():
BasePyramid()
{
  reset();
}

SimplePyramid::~SimplePyramid()
{
}

void SimplePyramid::reset()
{
  BasePyramid::reset();

  pyramidName = "SimplePyramid";
}

void SimplePyramid::combinePyramid(bool standard)
{
  int sm_width = pyramidImages.at(sm_level).cols;
  int sm_height = pyramidImages.at(sm_level).rows;
  if(combination_type == AM_COMB_MUL)
  {
    map = cv::Mat_<float>::ones(sm_height,sm_width);
  }
  else
  {
    map = cv::Mat_<float>::zeros(sm_height,sm_width);
  }

  for(int i = start_level; i <= max_level; ++i)
  {
    cv::Mat temp;
    
//     cv::imshow("pyramidFeatures.at(i)",pyramidFeatures.at(i));
//     cv::waitKey(-1);
    
    if(standard)
    {
      cv::resize(pyramidFeatures.at(i),temp,map.size());
    }
    else
    {
      EPUtils::scaleImage(pyramidFeatures,pyramidFeatures.at(i),temp,i,sm_level);
    }
    
//     cv::imshow("temp",temp);
//     cv::waitKey(-1);
    
    EPUtils::normalize(temp,normalization_type);
//     cv::imshow("temp",temp);
//     cv::waitKey(-1);
    combineConspicuityMaps(map,temp);
    
//     cv::imshow("map",map);
//     cv::waitKey(-1);
  }

  double maxValue, minValue;
  cv::minMaxLoc(map,&minValue,&maxValue);
  max_map_value = maxValue;
  EPUtils::normalize(map,normalization_type);
  
  calculated = true;
}

}