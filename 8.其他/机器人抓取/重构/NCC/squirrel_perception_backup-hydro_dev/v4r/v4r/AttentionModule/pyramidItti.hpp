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


#ifndef PYRAMID_ITTI_HPP
#define PYRAMID_ITTI_HPP

#include "pyramidBase.hpp"

namespace AttentionModule
{

class IttiPyramid: public BasePyramid
{
public:
  IttiPyramid();
  virtual ~IttiPyramid();
  typedef boost::shared_ptr<IttiPyramid> Ptr;

  void setLowestC(int lowest_c_);
  int getLowestC();

  void setHighestC(int highest_c_);
  int getHighestC();

  void setSmallestCS(int smallest_cs_);
  int getSmallestCS();

  void setLargestCS(int largest_cs_);
  int getLargestCS();

  void setNumberOfFeatures(int number_of_features_);
  int getNumberOfFeatures();

  void setChangeSign(bool changeSign_);
  bool getChangeSign();

  virtual void reset();
  virtual void print();
  virtual void combinePyramid(bool standard = false);
  
private:
  int                  lowest_c;
  int                  highest_c;
  int                  smallest_cs;
  int                  largest_cs;
  int                  number_of_features;
  bool                 changeSign;
  
  std::vector<cv::Mat> pyramidConspicuities;

  virtual void checkLevels();
};

}
#endif //PYRAMID_ITTI_HPP