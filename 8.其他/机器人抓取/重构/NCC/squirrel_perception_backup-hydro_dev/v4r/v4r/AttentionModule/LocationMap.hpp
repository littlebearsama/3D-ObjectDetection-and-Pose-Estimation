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


#ifndef LOCATION_MAP_HPP
#define LOCATION_MAP_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

enum LocationTypes
{
  AM_CENTER       = 0,
  AM_LEFT_CENTER     ,
  AM_LEFT            ,
  AM_RIGHT_CENTER    ,
  AM_RIGHT           ,
  AM_TOP_CENTER      ,
  AM_TOP             ,
  AM_BOTTOM_CENTER   ,
  AM_BOTTOM          ,
  AM_LOCATION_CUSTOM ,
};
  
class LocationSaliencyMap: public BaseMap
{
public:
  
  LocationSaliencyMap();
  ~LocationSaliencyMap();
  
  void setLocation(int location_);
  void setCenter(cv::Point _center_point);
/**
 * calculates location map
 * */
  virtual int calculate();
  
  virtual void reset();
  virtual void print();

private:

/**
 * parameters for location saliency map
 * */

  int location;
  cv::Point center_point;

protected:
  virtual int checkParameters();

};

} // namespace AttentionModule

#endif //LOCATION_MAP_HPP