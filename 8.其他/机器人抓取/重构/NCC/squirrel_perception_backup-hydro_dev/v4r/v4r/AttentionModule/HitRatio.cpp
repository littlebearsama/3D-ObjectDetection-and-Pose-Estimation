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


#include "HitRatio.hpp"
#include <list>

namespace AttentionModule {

AttentionPointsEvaluation::AttentionPointsEvaluation()
{ 
  haveMask = false;
}

AttentionPointsEvaluation::~AttentionPointsEvaluation()
{
}

void AttentionPointsEvaluation::setAttentionPoints(std::vector<cv::Point> attention_points_)
{
  attention_points = attention_points_;
}

std::vector<cv::Point> AttentionPointsEvaluation::getAttentionPoints()
{
  return(attention_points);
}
  
void AttentionPointsEvaluation::setMask(cv::Mat &mask_)
{
  mask_.copyTo(mask);
  haveMask = true;
}

bool AttentionPointsEvaluation::getMask(cv::Mat &mask_)
{
  if(haveMask)
  {
    mask.copyTo(mask_);
    return(true);
  }
  else
    return(false);
}
  
bool AttentionPointsEvaluation::calculate()
{
  if(!haveMask)
    return(false);
  
  calculateCenters();
  
  evaluated_points.resize(attention_points.size());
  
  for(unsigned int i = 0; i < attention_points.size(); ++i)
  {
    cv::Point p = attention_points.at(i);
    uchar objNum = mask.at<uchar>(p.y,p.x);
    
    if(objNum == 0)
    {
      evaluated_points.at(i).objectIdx = -1;
      evaluated_points.at(i).distance = -1;
      continue;
    }
    
    //calculate distance to the center
    float distance = EPUtils::calculateDistance(centers.at(objNum-1),p);
    
    evaluated_points.at(i).objectIdx = objNum;
    evaluated_points.at(i).distance = distance / maxDist2Center.at(objNum-1);
  }
  
  hitRatio();
  calculateDistance2objects();
  
  return(true);
}
  
std::vector<PointEvaluation> AttentionPointsEvaluation::getEvaluatedPoints()
{
  return(evaluated_points);
}

void AttentionPointsEvaluation::calculateCenters()
{
  if(!haveMask)
    return;
  
  double maxVal=0;
  cv::minMaxLoc(mask,0,&maxVal,0,0);
  
  //calculate centers
  centers.resize((int)maxVal);
  maxDist2Center.resize((int)maxVal);
  
  for(int i = 0; i < (int)maxVal; ++i)
  {
    cv::Mat mask_i;
    labeling2Mask(mask_i,i+1);
    cv::Point center_i;
    EPUtils::calculateObjectCenter(mask_i,center_i);
    centers.at(i) = center_i;
    
    
    float max_dist = 0;
    for(int r = 0; r < mask_i.rows; ++r)
    {
      for(int c = 0; c < mask_i.cols; ++c)
      {
	if(mask_i.at<uchar>(r,c) > 0)
	{
	  float cur_dist = sqrt( (r-center_i.y)*(r-center_i.y) + (c-center_i.x)*(c-center_i.x) );
	  if(cur_dist > max_dist)
	    max_dist = cur_dist;
	}
      }
    }
    maxDist2Center.at(i) = max_dist;
  }
}

void AttentionPointsEvaluation::labeling2Mask(cv::Mat &mask_i, int maskNum)
{
  assert(mask.type() == CV_8UC1);
  
  mask_i = cv::Mat_<uchar>::zeros(mask.rows,mask.cols);
  
  for(int i = 0; i < mask.rows; ++i)
  {
    for(int j = 0; j < mask.cols; ++j)
    {
      if(mask.at<uchar>(i,j) == maskNum)
	mask_i.at<uchar>(i,j) = 1;
    }
  }
}

void AttentionPointsEvaluation::hitRatio()
{
  assert(mask.type() == CV_8UC1);
  
  std::vector<bool> used_evaluated_points(attention_points.size(),false);
  
  double maxVal=0;
  cv::minMaxLoc(mask,0,&maxVal,0,0);
  
  float visitedObjects = 0;
  
  std::vector<bool> usedObjects(maxVal,false);
  
  for(unsigned int i = 0; i < attention_points.size(); ++i)
  {
    cv::Point p = attention_points.at(i);
    uchar objNum = mask.at<uchar>(p.y,p.x);
    
    if(objNum > 0)
    {
    
      if(!(usedObjects.at(objNum-1)))
      {
        visitedObjects = visitedObjects + 1;
        usedObjects.at(objNum-1) = true;
      }
    }
    
    evaluated_points.at(i).hitRatio = visitedObjects / (i+1);
  }
  
}

void AttentionPointsEvaluation::calculateDistance2objects()
{
  double maxVal=0;
  cv::minMaxLoc(mask,0,&maxVal,0,0);
  
  std::vector<ObjectDistance> best(maxVal);
  std::vector<ObjectDistance> first(maxVal);
  
  for(unsigned int i = 0; i < (unsigned int)maxVal; ++i)
  {
    first.at(i).attentionPointIdx = -1;
    best.at(i).attentionPointIdx = -1;
  }
  
  for(unsigned int i = 0; i < evaluated_points.size(); ++i)
  {
    int objIdx = evaluated_points.at(i).objectIdx;
    
    if(objIdx == -1)
    {
      continue;
    }
    
    //first distance
    if(first.at(objIdx-1).attentionPointIdx < 0)
    {
      first.at(objIdx-1).distance = evaluated_points.at(i).distance;
      first.at(objIdx-1).attentionPointIdx = i;
    }
    
    //first distance
    if(best.at(objIdx-1).attentionPointIdx < 0)
    {
      best.at(objIdx-1).distance = evaluated_points.at(i).distance;
      best.at(objIdx-1).attentionPointIdx = i;
    }
    else if(best.at(objIdx-1).distance > evaluated_points.at(i).distance)
    {
      best.at(objIdx-1).distance = evaluated_points.at(i).distance;
      best.at(objIdx-1).attentionPointIdx = i;
    }
    
  }
  
  float distance_before = 0;
  float object_counted = 0;
  
  for(unsigned int i = 0; i < evaluated_points.size(); ++i)
  {
    int objIdx = evaluated_points.at(i).objectIdx;
    
    if(objIdx == -1)
    {
      evaluated_points.at(i).firstDistance = distance_before;
      continue;
    }
    
    if(first.at(objIdx-1).attentionPointIdx == (int)i)
    {
      distance_before = (distance_before * object_counted + first.at(objIdx-1).distance)/(object_counted+1);
      object_counted = object_counted + 1;
    }
    
    evaluated_points.at(i).firstDistance = distance_before;
  }
  
  distance_before = 0;
  object_counted = 0;
  
  for(unsigned int i = 0; i < evaluated_points.size(); ++i)
  {
    int objIdx = evaluated_points.at(i).objectIdx;
    
    if(objIdx == -1)
    {
       evaluated_points.at(i).bestDistance = distance_before;
      continue;
    }
    
    if(best.at(objIdx-1).attentionPointIdx == (int)i)
    {
      distance_before = (distance_before * object_counted + best.at(objIdx-1).distance)/(object_counted+1);
      object_counted = object_counted + 1;
    }
    
    evaluated_points.at(i).bestDistance = distance_before;
  }
  
  return;
}

bool AttentionPointsEvaluation::writeToFile(std::string file_name)
{
  FILE * pFile;
  pFile = fopen (file_name.c_str(),"w");

  if(pFile == 0)
  {
    // file couldn't be opened
    std::cerr << "ERROR: file " << file_name << " could not be created." << std::endl;
    return(false);
  }
  
  fprintf(pFile,"%ld\n",evaluated_points.size());
  
  for(unsigned int i = 0; i < evaluated_points.size(); ++i)
  {
    PointEvaluation p = evaluated_points.at(i);
    fprintf(pFile,"%4d %10.4f %10.4f %10.4f %10.4f\n",p.objectIdx,p.distance,p.hitRatio,p.firstDistance,p.bestDistance);
  }
  
  fclose(pFile);
  return(true);
}

}//namespace EPEvaluation