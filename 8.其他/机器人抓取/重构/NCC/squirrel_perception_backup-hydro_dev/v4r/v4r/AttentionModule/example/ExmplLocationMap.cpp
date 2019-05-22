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
    
  // start creating parameters
  AttentionModule::LocationSaliencyMap locationSaliencyMap;
  locationSaliencyMap.setWidth(image.cols);
  locationSaliencyMap.setHeight(image.rows);

  cv::Mat map;
  
  locationSaliencyMap.setLocation(AttentionModule::AM_TOP);
  printf("[INFO]: Computing location map AM_TOP.\n");
  locationSaliencyMap.calculate();
  
  if(locationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Location Map",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[INFO]: Computation failed.\n");
  }

  locationSaliencyMap.setLocation(AttentionModule::AM_CENTER);
  printf("[INFO]: Computing location map AM_CENTER.\n");
  locationSaliencyMap.calculate();
  
  if(locationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Location Map",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[INFO]: Computation failed.\n");
  }

  locationSaliencyMap.setLocation(AttentionModule::AM_RIGHT_CENTER);
  printf("[INFO]: Computing location map AM_RIGHT_CENTER.\n");
  locationSaliencyMap.calculate();
  
  if(locationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Location Map",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[INFO]: Computation failed.\n");
  }

  locationSaliencyMap.setLocation(AttentionModule::AM_BOTTOM_CENTER);
  printf("[INFO]: Computing location map AM_BOTTOM_CENTER.\n");
  locationSaliencyMap.calculate();
  
  if(locationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Location Map",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[INFO]: Computation failed.\n");
  }

  locationSaliencyMap.setLocation(AttentionModule::AM_RIGHT);
  printf("[INFO]: Computing location map AM_RIGHT.\n");
  locationSaliencyMap.calculate();
  
  if(locationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Location Map",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[INFO]: Computation failed.\n");
  }

  locationSaliencyMap.setLocation(AttentionModule::AM_CUSTOM);
  locationSaliencyMap.setCenter(cv::Point(50,50));
  printf("[INFO]: Computing location map AM_CUSTOM, cv::Point(50,50).\n");
  locationSaliencyMap.calculate();
  
  if(locationSaliencyMap.getMap(map))
  {
    printf("[INFO]: Computation completed.\n");
    cv::imshow("Location Map",map);
    printf("[INFO]: Press any key to continue.\n");
    cv::waitKey();
  }
  else
  {
    printf("[INFO]: Computation failed.\n");
  }
  
  return(0);
}