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

// This program shows the use of TJ to extract attention points

void printUsage(const char *argv0)
{
  printf(
    "Extracts attention points using TJ\n"
    "usage: %s image.png saliency.png output.txt result.png\n"
    "  image.png             ... color image\n"
    "  saliency.png          ... saliency image\n"
    "  points.txt            ... output text file with points\n"
    "  result.png            ... output file name\n", argv0);
  printf(" Example: %s 1 image.png saliency.png points.txt result.png\n",argv0);
}

int main(int argc, char** argv)
{
  if(argc != 5)
  {
    printUsage(argv[0]);
    return(0);
  }
  
  srand ( time(NULL) );
  
  std::string image_name(argv[1]);
  std::string saliency_map_name(argv[2]);
  std::string output_file_name(argv[3]);
  std::string output_png_name(argv[4]);
    
  cv::Mat image = cv::imread(image_name,-1);
  cv::Mat map = cv::imread(saliency_map_name,-1);
  map.convertTo(map,CV_32F,1.0f/255);
    
  std::vector<EPUtils::ConnectedComponent> connectedComponents;
    
  float th = 0.1;
  EPUtils::extractConnectedComponents(map,connectedComponents,th);
  EPUtils::drawConnectedComponents(connectedComponents,image,cv::Scalar(255,0,0));
  //std::cerr << "Number of connected components: " << connectedComponents.size() << std::endl;
    
  std::vector<AttentionModule::SaliencyLine> saliencyLine;
  cv::Mat points_image = cv::Mat_<uchar>::zeros(image.rows,image.cols);
  std::vector<AttentionModule::PointSaliency> saliencyPoints;
    
  for(unsigned int i = 0; i < connectedComponents.size(); ++ i)
  {
    //std::cerr << i << " " << connectedComponents.size() << std::endl;
    cv::Mat mask = cv::Mat_<uchar>::zeros(image.rows,image.cols);
    EPUtils::drawConnectedComponent(connectedComponents.at(i),mask,cv::Scalar(1));
    //cv::imshow("mask",255*mask);
    //cv::waitKey();
      
    AttentionModule::SaliencyLine saliencyLineCurent;
    AttentionModule::PointSaliency pointSaliencyCurrent;
    //std::cerr << "here 0" << std::endl;
    if(AttentionModule::extractSaliencyLine(mask,map,saliencyLineCurent))
    {
      //std::vector<cv::Point> saliencyLineCurent_points;
      //AttentionModule::createSimpleLine(saliencyLineCurent,saliencyLineCurent_points);
      //std::cerr << saliencyLineCurent_points.size() << std::endl;
      //EPUtils::drawAttentionPoints(image,saliencyLineCurent_points);
      //cv::imshow("skeleton",image);
      //cv::waitKey();
      
      saliencyLine.push_back(saliencyLineCurent);
      //std::cerr << "here 1" << std::endl;
      AttentionModule::selectSaliencyCenterPoint(saliencyLineCurent,pointSaliencyCurrent);
      //std::cerr << "here 2" << std::endl;
      saliencyPoints.push_back(pointSaliencyCurrent);
      
    }
    //std::cerr << "here 3" << std::endl;
  }
    
  std::sort(saliencyPoints.begin(),saliencyPoints.end(),AttentionModule::saliencyPointsSort);
    
  std::vector<cv::Point> attentionPoints;
  AttentionModule::createAttentionPoints(saliencyPoints,attentionPoints);
  
  EPUtils::writeAttentionPoints(attentionPoints,output_file_name);
  
  EPUtils::drawAttentionPoints(image,attentionPoints,10);
  
  cv::imwrite(output_png_name,image);
  
  //cv::imshow("attention points", image);
  //cv::imshow("saliency map", map);
  //cv::waitKey();
}