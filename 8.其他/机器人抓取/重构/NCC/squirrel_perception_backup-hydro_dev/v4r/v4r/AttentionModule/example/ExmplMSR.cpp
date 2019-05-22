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

// This program shows the use of MSR to extract attention points

void printUsage(const char *argv0)
{
  printf(
    "Extracts attention points using MSR\n"
    "usage: %s image.png saliency.png points.txt points.png useMorphologyOpenning\n"
    "  image.png             ... color image\n"
    "  saliency.png          ... saliency image\n"
    "  points.txt            ... output text file with points\n"
    "  points.png            ... output image with points\n"
    "  useMorphologyOpenning ... 0 -- useMorphologyOpenning = false; !=0 -- useMorphologyOpenning = true\n", argv0);
  printf(" Example: %s 1 image.png saliency.png points.txt points.png 0\n",argv0);
}

int main(int argc, char** argv)
{
  srand ( time(NULL) );

  if(argc != 6)
  {
    printUsage(argv[0]);
    return(0);
  }
  
  std::string image_name(argv[1]);
  std::string map_name(argv[2]);
  std::string output_file_name(argv[3]);
  std::string output_png_name(argv[4]);
  
  int useMorphologyOpenning = atoi(argv[5]);
  
  // read image
  cv::Mat image = cv::imread(image_name,-1);
  
  // read saliency map
  cv::Mat saliencyMap = cv::imread(map_name,0);
  saliencyMap.convertTo(saliencyMap,CV_32F,1.0/255);
  
  AttentionModule::MRSParams msrParams;
  AttentionModule::defaultParamsMSR(msrParams);
  //modify wta
  msrParams.useMorphologyOpenning = (useMorphologyOpenning == 0 ? false : true);

  std::vector<cv::Point> attentionPoints;
  attentionPoints.clear();
  AttentionModule::detectMSR(attentionPoints,saliencyMap,msrParams);

  EPUtils::writeAttentionPoints(attentionPoints,output_file_name);
    
  //save image with attantion points
  EPUtils::drawAttentionPoints(image,attentionPoints,10);
    
  cv::imwrite(output_png_name,image);
  //cv::waitKey();

  return(0);
}