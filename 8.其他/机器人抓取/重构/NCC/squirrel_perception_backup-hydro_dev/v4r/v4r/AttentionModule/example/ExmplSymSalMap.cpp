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


#include "v4r/AttentionModule/AttentionModule.hpp"

// This program shows the use of Symmetry Saliency Map on one image

void printUsage(const char *argv0)
{
  printf(
    "Calculates 3D Symmentry Saliency Map\n"
    "usage: %s image.png result.png\n"
    "  image.png             ... color image\n"
    "  result.png            ... output file name\n", argv0);
  printf(" Example: %s image.png result.png\n",argv0);
}

int main(int argc, char** argv)
{
  if(argc != 3)
  {
    printUsage(argv[0]);
    return(0);
  }
  
  // read image
  std::string image_name(argv[1]);
  cv::Mat image = cv::imread(image_name,-1);
  
  std::string output_name(argv[2]);
  
  // start creating parameters
  AttentionModule::SymmetryMap symmetryMap;
  symmetryMap.setImage(image);
      
  cv::Mat map;
  
  symmetryMap.calculatePyramid();
  
  if(symmetryMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::Mat temp;
    map.convertTo(temp,CV_8U,255);
    cv::imwrite(output_name,temp);
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  return(0);
}