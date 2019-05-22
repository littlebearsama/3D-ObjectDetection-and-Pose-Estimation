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

// This program shows the use of WTA to extract attention points

void printUsage(const char *argv0)
{
  printf(
    "Extracts attention points using WTA\n"
    "usage: %s image.png saliency.png output.txt output.png useRandom useCentering useMorphologyOpenning\n"
    "  image.png             ... color image\n"
    "  saliency.png          ... saliency image\n"
    "  points.txt            ... output text file with points\n"
    "  points.png            ... output image with points\n"
    "  useRandom             ... 0 -- false; 1 -- true;\n"
    "  useCentering          ... 0 -- false; 1 -- true;\n"
    "  useMorphologyOpenning ... 0 -- false; 1 -- true;\n", argv0);
  printf(" Example: %s 1 image.png saliency.png points.txt points.png 0 0 0\n",argv0);
}

int main(int argc, char** argv)
{
  srand ( time(NULL) );

  if(argc != 8)
  {
    printUsage(argv[0]);
    return(0);
  }
  
  std::string image_name(argv[1]);
  std::string map_name(argv[2]);
  std::string output_file_name(argv[3]);
  std::string output_png_name(argv[4]);
  
  int useRandom = atoi(argv[5]);
  int useCentering = atoi(argv[6]);
  int useMorphologyOpenning = atoi(argv[7]);
  
  // read image
  cv::Mat image = cv::imread(image_name,-1);
  
  // read saliency map
  cv::Mat saliencyMap = cv::imread(map_name,0);
  saliencyMap.convertTo(saliencyMap,CV_32F,1.0/255);
    
  AttentionModule::Params wtaParams;
  AttentionModule::defaultParams(wtaParams);
  //modify wta
  wtaParams.useRandom = (useRandom == 0 ? false : true);
  wtaParams.useCentering = (useCentering == 0 ? false : true);
  wtaParams.useMorphologyOpenning = (useMorphologyOpenning == 0 ? false : true);

  std::vector<cv::Point> attentionPoints;
  attentionPoints.clear();
  cv::Mat temp;
  image.copyTo(temp);
  temp.convertTo(temp,CV_32F,1.0/255);
  AttentionModule::CalculateWTA(temp,saliencyMap,attentionPoints,0,wtaParams);

  EPUtils::writeAttentionPoints(attentionPoints,output_file_name);
    
  //save image with attantion points
  EPUtils::drawAttentionPoints(image,attentionPoints,10);
  
  cv::imwrite(output_png_name,image);
  //cv::waitKey();

  return(0);
}