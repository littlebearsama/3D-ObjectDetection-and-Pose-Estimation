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


#include <opencv2/opencv.hpp>

#include "v4r/AttentionModule/AttentionModule.hpp"
#include "v4r/EPUtils/EPUtils.hpp"

// This program calculates HitRatio from attention points

void printUsage(const char *argv0)
{
  printf(
    "Calculates HitRatio from attention points\n"
    "usage: %s points.txt mask.png hitratio.txt\n"
    "  points.txt            ... input text file with points\n"
    "  mask.png              ... mask image\n"
    "  hitratio.txt          ... output text file with hitratios\n", argv0);
  printf(" Example: %s points.txt mask.png hitratio.txt\n",argv0);
}

int main(int argc, char** argv)
{
  srand ( time(NULL) );

  if(argc != 4)
  {
    printUsage(argv[0]);
    return(0);
  }
  
  std::string attention_points_txt(argv[1]);
  std::string mask_image_name(argv[2]);
  std::string output_hitration_txt(argv[3]);
  
  std::vector<cv::Point> attentionPoints;
  EPUtils::readAttentionPoints(attentionPoints,attention_points_txt);
  cv::Mat mask = cv::imread(mask_image_name,0);
  
  AttentionModule::AttentionPointsEvaluation attentionPointsEvaluation;
  attentionPointsEvaluation.setAttentionPoints(attentionPoints);
  attentionPointsEvaluation.setMask(mask);
  
  if(attentionPointsEvaluation.calculate())
  {
    attentionPointsEvaluation.writeToFile(output_hitration_txt);
  }
  else
    printf("[ERROR]: Couldn't calculate Hit Ratio!\n");

  return(0);
}