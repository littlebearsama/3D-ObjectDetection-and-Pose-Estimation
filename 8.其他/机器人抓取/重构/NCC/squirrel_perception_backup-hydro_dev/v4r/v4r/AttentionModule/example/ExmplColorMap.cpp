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

// This program shows the use of Color Saliency Map on one image

int main(int argc, char** argv)
{
  if(argc != 2)
  {
    std::cerr << "Usage: image" << std::endl;
    return(0);
  }
  
  // read image
  std::string image_name(argv[1]);
  cv::Mat image = cv::imread(image_name,-1);
  
  cv::imshow("Original Image",image);
    
  // start creating parameters
  AttentionModule::ColorSaliencyMap colorSaliencyMap;
  colorSaliencyMap.setImage(image);
  // blue color
  colorSaliencyMap.setColor(cv::Scalar(0,0,255));
      
  cv::Mat map;
  
  printf("[INFO]: Computing color map (blue) in RGB space.\n");
  
  colorSaliencyMap.calculate();
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) RGB",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in RGB space using Simple pyramid.\n");
  
  colorSaliencyMap.calculatePyramid();
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) RGB Simple Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in RGB space using Itti pyramid.\n");
  
  colorSaliencyMap.calculatePyramid(AttentionModule::ITTI_PYRAMID);
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) RGB Itti Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in RGB space using Frintrop pyramid.\n");
  
  colorSaliencyMap.calculatePyramid(AttentionModule::FRINTROP_PYRAMID);
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) RGB Frintrop Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in Lab space.\n");
  
  // blue color
  colorSaliencyMap.setColor(cv::Scalar(0,0.5,0));
  colorSaliencyMap.setUseLAB(true);
  colorSaliencyMap.calculate();
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) Lab",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in Lab space using Simple pyramid.\n");

  colorSaliencyMap.calculatePyramid();
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) Lab Simple Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in Lab space using Itti pyramid.\n");
  
  colorSaliencyMap.calculatePyramid(AttentionModule::ITTI_PYRAMID);
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) Lab Itti Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing color map (blue) in Lab space using Frintrop pyramid.\n");
  
  colorSaliencyMap.calculatePyramid(AttentionModule::FRINTROP_PYRAMID);
  
  if(colorSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Color Map (blue) Lab Frintrop Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  return(0);
}