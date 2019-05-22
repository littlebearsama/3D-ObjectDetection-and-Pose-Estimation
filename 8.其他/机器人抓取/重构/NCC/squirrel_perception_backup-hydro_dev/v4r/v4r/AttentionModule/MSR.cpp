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


#include "MSR.hpp"

namespace AttentionModule
{

void winnerToImgCoords(cv::Point& p_new, cv::Point& p, int mapLevel)
{
  float x = p.x;
  float y = p.y;
  p_new.x = (int)(x * (pow(2,mapLevel-1)));
  p_new.y = (int)(y * (pow(2,mapLevel-1)));
}

void defaultParamsMSR(MRSParams &params)
{
  params.th = 0.25;
  params.mapLevel = 5;
  params.useMorphologyOpenning = false;
}

void detectMSR(std::vector<cv::Point> &centers, cv::Mat map_, MRSParams params)
{
  assert(map_.type() == CV_32FC1);
  
  cv::Mat map, temp;
  map_.copyTo(temp);
  
  // perform morphological operations if necessary
  if(params.useMorphologyOpenning)
  {
    // calculate the size of the kernel
    int kernel_size = pow(2,params.mapLevel-1);
    cv::Mat element = cv::Mat_<uchar>::ones(kernel_size,kernel_size);
    cv::erode(map_,temp,element);
  }
  
  cv::resize(temp,map,cv::Size(map_.cols/(pow(2,params.mapLevel-1)),map_.rows/(pow(2,params.mapLevel-1))));
  
  double maxVal=0;
  cv::Point maxLoc;
  cv::minMaxLoc(map,0,&maxVal,0,&maxLoc);
  
  while(maxVal > 0)
  {
    
    cv::Point maxLoc_new;
    winnerToImgCoords(maxLoc_new,maxLoc,params.mapLevel);
    centers.push_back(maxLoc_new);
    
    float maxValTh = (1-params.th)*maxVal;
    
    std::list<cv::Point> points;
    points.push_back(maxLoc);
    cv::Mat used = cv::Mat_<uchar>::zeros(map.rows,map.cols);
    used.at<uchar>(maxLoc.y,maxLoc.x) = 1;
    map.at<float>(maxLoc.y,maxLoc.x) = 0;
    while(points.size())
    {
      cv::Point p = points.front();
      points.pop_front();
      
      if(((p.x+1) < map.cols) && (!used.at<uchar>(p.y,p.x+1)) && (map.at<float>(p.y,p.x+1)>=maxValTh))
      {
        points.push_back(cv::Point(p.x+1,p.y));
        used.at<uchar>(p.y,p.x+1) = 1;
	map.at<float>(p.y,p.x+1) = 0;
	//count++;
      }
      if(((p.x-1) >= 0) && (!used.at<uchar>(p.y,p.x-1)) && (map.at<float>(p.y,p.x-1)>=maxValTh))
      {
        points.push_back(cv::Point(p.x-1,p.y));
        used.at<uchar>(p.y,p.x-1) = 1;
	map.at<float>(p.y,p.x-1) = 0;
	//count++;
      }
      if(((p.y+1) < map.rows) && (!used.at<uchar>(p.y+1,p.x)) && (map.at<float>(p.y+1,p.x)>=maxValTh))
      {
        points.push_back(cv::Point(p.x,p.y+1));
        used.at<uchar>(p.y+1,p.x) = 1;
	map.at<float>(p.y+1,p.x) = 0;
	//count++;
      }
      if(((p.y-1) >= 0) && (!used.at<uchar>(p.y-1,p.x)) && (map.at<float>(p.y-1,p.x)>=maxValTh))
      {
        points.push_back(cv::Point(p.x,p.y-1));
        used.at<uchar>(p.y-1,p.x) = 1;
	map.at<float>(p.y-1,p.x) = 0;
	//count++;
      }
    }
    cv::minMaxLoc(map,0,&maxVal,0,&maxLoc);
    
//     cv::imshow("map",map);
//     cv::waitKey();
  }
}

} //namespace AttentionModule