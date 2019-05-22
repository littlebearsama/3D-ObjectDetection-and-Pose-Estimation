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
  AttentionModule::OrientationSaliencyMap orientationSaliencyMap;
  orientationSaliencyMap.setImage(image);
  
  orientationSaliencyMap.setBandwidth(2);
      
  cv::Mat map;
  
  printf("[INFO]: Computing orientation map angle 0.\n");
  
  orientationSaliencyMap.setAngle(0);
  orientationSaliencyMap.calculate();
  
  if(orientationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Orientation Map angle 0",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing orientation map angle 0 using Simple pyramid.\n");
  
  orientationSaliencyMap.calculatePyramid();
  
  if(orientationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Orientation Map angle 0 Simple Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing orientation map angle 0 using Itti pyramid.\n");
  
  orientationSaliencyMap.calculatePyramid(AttentionModule::ITTI_PYRAMID);
  
  if(orientationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Orientation Map angle 0 Itti Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }
  
  printf("[INFO]: Computing orientation map angle 0 using Frintrop pyramid.\n");
  
  orientationSaliencyMap.calculatePyramid(AttentionModule::FRINTROP_PYRAMID);
  
  if(orientationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Orientation Map angle 0 Frintrop Pyramid",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[ERROR]: Computation failed.\n");
  }

//   printf("[INFO]: Computing orientation map angle 45.\n");
//   
//   orientationSaliencyMap.setAngle(45);
//   orientationSaliencyMap.calculate();
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 45",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 45 using Simple pyramid.\n");
//   
//   orientationSaliencyMap.calculatePyramid();
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 45 Simple Pyramid",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 45 using Itti pyramid.\n");
//   
//   orientationSaliencyMap.calculatePyramid(AttentionModule::ITTI_PYRAMID);
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 45 Itti Pyramid",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 45 using Frintrop pyramid.\n");
//   
//   orientationSaliencyMap.calculatePyramid(AttentionModule::FRINTROP_PYRAMID);
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 45 Frintrop Pyramid",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 90.\n");
//   
//   orientationSaliencyMap.setAngle(90);
//   orientationSaliencyMap.calculate();
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 90",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 90 using Simple pyramid.\n");
//   
//   orientationSaliencyMap.calculatePyramid();
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 90 Simple Pyramid",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 90 using Itti pyramid.\n");
//   
//   orientationSaliencyMap.calculatePyramid(AttentionModule::ITTI_PYRAMID);
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 90 Itti Pyramid",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
//   
//   printf("[INFO]: Computing orientation map angle 90 using Frintrop pyramid.\n");
//   
//   orientationSaliencyMap.calculatePyramid(AttentionModule::FRINTROP_PYRAMID);
//   
//   if(orientationSaliencyMap.getMap(map))
//   {
//     printf("[INFO]: Computation completed.\n");
//     cv::imshow("Orientation Map angle 90 Frintrop Pyramid",map);
//     printf("[INFO]: Press any key to continue.\n");
//     cv::waitKey();
//   }
//   else
//   {
//     printf("[ERROR]: Computation failed.\n");
//   }
  
  return(0);
}
  
