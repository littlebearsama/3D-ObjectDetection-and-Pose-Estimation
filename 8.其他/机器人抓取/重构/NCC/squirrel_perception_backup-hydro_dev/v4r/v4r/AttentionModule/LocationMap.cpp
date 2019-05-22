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


#include "LocationMap.hpp"

namespace AttentionModule
{

LocationSaliencyMap::LocationSaliencyMap():
BaseMap()
{
  reset();
}

LocationSaliencyMap::~LocationSaliencyMap()
{
}

void LocationSaliencyMap::reset()
{
  BaseMap::reset();
  location = AM_CENTER;
  center_point = cv::Point(0,0);

  mapName = "LocationSaliencyMap";
}

void LocationSaliencyMap::print()
{
  BaseMap::print();
  printf("[%s]: location           = %d\n",mapName.c_str(),location);
  printf("[%s]: center_point       = (%d,%d)\n",mapName.c_str(),center_point.x,center_point.y);
}

void LocationSaliencyMap::setLocation(int location_)
{
  location = location_;
  calculated = false;
  printf("[INFO]: %s: location is set to: %d\n",mapName.c_str(),location);
}

void LocationSaliencyMap::setCenter(cv::Point _center_point)
{
  center_point = _center_point;
  calculated = false;
  printf("[INFO]: %s: center_point is set to: (%d,%d)\n",mapName.c_str(),center_point.x,center_point.y);
}

int LocationSaliencyMap::checkParameters()
{
  if( (width == 0) || (height == 0) )
  {
    printf("[ERROR]: %s: Seems like image is empty.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }
  
  return(AM_OK);
  
}

int LocationSaliencyMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  cv::Point center;
  float a = 1;
  float b = 1;

  switch(location)
  {
    case AM_CENTER:
      center = cv::Point(width/2,height/2);
      break;
    case AM_LEFT_CENTER:
      center = cv::Point(width/4,height/2);
      break;
    case AM_LEFT:
      center = cv::Point(width/4,height/4);
      b = 0;
      break;
    case AM_RIGHT_CENTER:
      center = cv::Point(4*width/4,height/2);
      break;
    case AM_RIGHT:
      center = cv::Point(3*width/4,3*height/2);
      b = 0;
      break;
    case AM_TOP_CENTER:
      center = cv::Point(width/2,height/4);
      break;
    case AM_TOP:
      center = cv::Point(width/2,height/4);
      a = 0;
      break;
    case AM_BOTTOM_CENTER:
      center = cv::Point(width/2,3*height/4);
      break;
    case AM_BOTTOM:
      center = cv::Point(width/2,3*height/4);
      a = 0;
      break;
    case AM_LOCATION_CUSTOM:
      center = center_point;
      break;
    default:
      center = cv::Point(width/2,height/2);
      break;
  }
  
  map = cv::Mat_<float>::zeros(height,width);
  
  for(int r = 0; r < height; ++r)
  {
    for(int c = 0; c < width; ++c)
    {
      if(mask.at<uchar>(r,c))
      {
        float dx = c-center.x;
        dx = a*(dx/width);
        float dy = r-center.y;
        dy = b*(dy/height);
        float value = dx*dx + dy*dy;
        map.at<float>(r,c) = exp(-11*value);
      }
    }
  }
  
  cv::blur(map,map,cv::Size(filter_size,filter_size));

  EPUtils::normalize(map,normalization_type);

  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());
  return(AM_OK);
}
} //namespace AttentionModule