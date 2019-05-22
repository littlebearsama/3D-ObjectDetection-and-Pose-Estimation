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


#include "v4r/EPUtils/EPUtils.hpp"

#include <iostream>

int main(int argc, char** argv)
{
  if(argc != 4)
  {
    std::cerr << "Usage: attention_points_list image new_image_name" << std::endl;
    return(0);
  }
  
  std::string attention_points_file_name(argv[1]);
  std::string image_name(argv[2]);
  std::string new_image_name(argv[3]);
  
  // read attention points
  std::vector<cv::Point> attentionPoints;
  EPUtils::readAttentionPoints(attentionPoints,attention_points_file_name);
  // read labeling
  cv::Mat image = cv::imread(image_name,-1);
    
  EPUtils::drawAttentionPoints(image,attentionPoints,10);
  
  cv::imwrite(new_image_name,image);

  return 0;
}