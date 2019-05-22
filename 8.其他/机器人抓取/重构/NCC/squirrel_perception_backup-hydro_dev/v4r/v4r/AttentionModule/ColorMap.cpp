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


#include "ColorMap.hpp"

namespace AttentionModule
{

ColorSaliencyMap::ColorSaliencyMap():
BaseMap()
{
  reset();
}

ColorSaliencyMap::~ColorSaliencyMap()
{
}

void ColorSaliencyMap::reset()
{
  BaseMap::reset();
  useLAB = false;
  color = cv::Scalar(0,0,0);

  mapName = "ColorSaliencyMap";
}

void ColorSaliencyMap::print()
{
  BaseMap::print();
  printf("[%s]: color              = (%f,%f,%f)\n",mapName.c_str(),color(0),color(0),color(0));
  printf("[%s]: useLAB             = %s\n",mapName.c_str(),useLAB ? "yes" : "no");
}

void ColorSaliencyMap::setUseLAB(bool useLAB_)
{
  useLAB = useLAB_;
  calculated = false;
  printf("[INFO]: %s: Use LAB color: %s.\n",mapName.c_str(),useLAB ? "yes" : "no");
}

void ColorSaliencyMap::setColor(cv::Scalar color_)
{
  color = color_;
  calculated = false;
  printf("[INFO]: %s: Base color: %f,%f,%f.\n",mapName.c_str(),color(0),color(1),color(2));
}

bool ColorSaliencyMap::getUseLAB()
{
  return(useLAB);
}

cv::Scalar ColorSaliencyMap::getColor()
{
  return(color);
}

int ColorSaliencyMap::checkParameters()
{
  if(!haveImage)
  {
    printf("[ERROR]: %s: No image set.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if( (width == 0) || (height == 0) || (image.rows == 0) || (image.cols == 0) )
  {
    printf("[ERROR]: %s: Seems like image is empty.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if((image.cols != width) || (image.rows != height))
  {
    printf("[ERROR]: %s: Problem with image sizes.\n",mapName.c_str());
    return(AM_IMAGE);
  }

  if(image.channels() != 3)
  {
    printf("[ERROR]: %s: Image should have 3 channels.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }

  return(AM_OK);
}

float ColorSaliencyMap::getMaxColorDistance(float &r_color, float &g_color, float &b_color, float &a_color)
{
  r_color = 0;
  g_color = 0;
  b_color = 0;
  a_color = 0;
  
  float max_dist;
  if(useLAB)
  {
    a_color = color(0);
    b_color = color(1);
    a_color = a_color/255;
    b_color = b_color/255;
    
    // red color (1,0.5)
    float max2red = sqrt((a_color-1)*(a_color-1) + (b_color-0.5)*(b_color-0.5));
    // green color (0,0.5)
    float max2green = sqrt((a_color-0)*(a_color-0) + (b_color-0.5)*(b_color-0.5));
    // blue color (0.5,0)
    float max2blue = sqrt((a_color-0.5)*(a_color-0.5) + (b_color-0)*(b_color-0));
    // yellow color (0.5,1)
    float max2yellow = sqrt((a_color-0.5)*(a_color-0.5) + (b_color-1)*(b_color-1));
    
    max_dist = std::max(max2red,max2green);
    max_dist = std::max(max_dist,max2yellow);
    max_dist = std::max(max_dist,max2blue);
  }
  else
  {
    r_color = color(0);
    g_color = color(1);
    b_color = color(2);
    r_color = r_color/255;
    g_color = g_color/255;
    b_color = b_color/255;
    
    // red color (1,0,0)
    float max2red = sqrt((r_color-1)*(r_color-1) + (g_color-0)*(g_color-0) + (b_color-0)*(b_color-0));
    // green color (0,1,0)
    float max2green = sqrt((r_color-0)*(r_color-0) + (g_color-1)*(g_color-1) + (b_color-0)*(b_color-0));
    // blue color (0,0,1)
    float max2blue = sqrt((r_color-0)*(r_color-0) + (g_color-0)*(g_color-0) + (b_color-1)*(b_color-1));
    // red-green color (1,1,0)
    float max2red_green = sqrt((r_color-1)*(r_color-1) + (g_color-1)*(g_color-1) + (b_color-0)*(b_color-0));
    // red-blue color (1,0,1)
    float max2red_blue = sqrt((r_color-1)*(r_color-1) + (g_color-0)*(g_color-0) + (b_color-1)*(b_color-1));
    // green-blue color (0,1,1)
    float max2green_blue = sqrt((r_color-0)*(r_color-0) + (g_color-1)*(g_color-1) + (b_color-1)*(b_color-1));
    // black color (0,0,0)
    float max2black = sqrt((r_color-0)*(r_color-0) + (g_color-0)*(g_color-0) + (b_color-0)*(b_color-0));
    // white color (1,1,1)
    float max2white = sqrt((r_color-1)*(r_color-1) + (g_color-1)*(g_color-1) + (b_color-1)*(b_color-1));
    
    max_dist = std::max(max2red,max2green);
    max_dist = std::max(max_dist,max2blue);
    max_dist = std::max(max_dist,max2red_green);
    max_dist = std::max(max_dist,max2red_blue);
    max_dist = std::max(max_dist,max2green_blue);
    max_dist = std::max(max_dist,max2black);
    max_dist = std::max(max_dist,max2white);
  }
  
  return(max_dist);
}

void ColorSaliencyMap::LabColorMap(cv::Mat &image_cur, int image_width, int image_height, float max_dist, float a_color, float b_color, cv::Mat &map_cur)
{
  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  for(int r = 0; r < image_height; ++r)
  {
    for(int c = 0; c < image_width; ++c)
    {
      if(mask.at<uchar>(r,c))
      {
	float aa = image_cur.at<uchar>(r,3*c+1);
        float bb = image_cur.at<uchar>(r,3*c+2);
	aa = aa/255;
	bb = bb/255;
	
        float dist = (aa - a_color)*(aa - a_color) + (bb - b_color)*(bb - b_color);
	dist = sqrt(dist);
        map_cur.at<float>(r,c) = 1.0 - dist/max_dist;
      }
    }
  }
}

void ColorSaliencyMap::RGBColorMap(cv::Mat &image_cur, int image_width, int image_height, float max_dist, float r_color, float g_color, float b_color, cv::Mat &map_cur)
{
  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  for(int r = 0; r < image_height; ++r)
  {
    for(int c = 0; c < image_width; ++c)
    {
      //if(mask.at<uchar>(r,c))
      {
	float rr = image_cur.at<uchar>(r,3*c+2);
        float gg = image_cur.at<uchar>(r,3*c+1);
        float bb = image_cur.at<uchar>(r,3*c+0);
        rr = rr/255;
	gg = gg/255;
	bb = bb/255;
	
        float dist = (rr - r_color)*(rr - r_color) + (gg - g_color)*(gg - g_color) + (bb - b_color)*(bb - b_color);
	dist = sqrt(dist);
        map_cur.at<float>(r,c) = 1.0 - dist/max_dist;
      }
    }
  }
}

int ColorSaliencyMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  cv::Mat image_cur;
  image.copyTo(image_cur);
  
  cv::blur(image_cur,image_cur,cv::Size(filter_size,filter_size));
  
  if(useLAB)
  {
    cvtColor(image_cur,image_cur,CV_BGR2Lab);
  }

  float r_color = 0;
  float g_color = 0;
  float b_color = 0;
  float a_color = 0;
  float max_dist = getMaxColorDistance(r_color,g_color,b_color,a_color);
  
  if(useLAB)
  {
    LabColorMap(image_cur,width,height,max_dist,a_color,b_color,map);
  }
  else
  {
    RGBColorMap(image_cur,width,height,max_dist,r_color,g_color,b_color,map);
  }
  
  cv::blur(map,map,cv::Size(filter_size,filter_size));

  EPUtils::normalize(map,normalization_type);

  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int ColorSaliencyMap::calculatePyramidSimple()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Simple pyramid started.\n",mapName.c_str());
  
  SimplePyramid::Ptr pyramid( new SimplePyramid() );
  
  pyramid->setStartLevel(0);
  pyramid->setMaxLevel(6);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  
  cv::Mat image_cur;
  image.copyTo(image_cur);
  
  if(useLAB)
  {
    cvtColor(image_cur,image_cur,CV_BGR2Lab);
  }
  
  pyramid->setImage(image_cur);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int ColorSaliencyMap::calculatePyramidItti()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Itti pyramid started.\n",mapName.c_str());
  
  IttiPyramid::Ptr pyramid( new IttiPyramid() );
  
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  
  pyramid->setLowestC(2);
  pyramid->setHighestC(4);
  pyramid->setSmallestCS(3);
  pyramid->setLargestCS(4);
  
  pyramid->setChangeSign(false);
  
  cv::Mat image_cur;
  image.copyTo(image_cur);
  
  if(useLAB)
  {
    cvtColor(image_cur,image_cur,CV_BGR2Lab);
  }
  
  pyramid->setImage(image_cur);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int ColorSaliencyMap::calculatePyramidFrintrop()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Frintrop pyramid started.\n",mapName.c_str());
  
  FrintropPyramid::Ptr pyramid( new FrintropPyramid() );
  
  pyramid->setStartLevel(0);
  pyramid->setMaxLevel(6);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  
  std::vector<int> R;
  R.resize(2); R.at(0) = 3; R.at(1) = 7;
  pyramid->setR(R);
  pyramid->setOnSwitch(true);
  
  cv::Mat image_cur;
  image.copyTo(image_cur);
  
  if(useLAB)
  {
    cvtColor(image_cur,image_cur,CV_BGR2Lab);
  }
  
  pyramid->setImage(image_cur);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int ColorSaliencyMap::combinePyramid(BasePyramid::Ptr pyramid)
{
  float r_color = 0;
  float g_color = 0;
  float b_color = 0;
  float a_color = 0;
  float max_dist = getMaxColorDistance(r_color,g_color,b_color,a_color);
  
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    // start creating parameters
    cv::Mat current_image;
    if(!pyramid->getImage(i,current_image))
    {
      printf("[ERROR]: Something went wrong! Can't get image for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    int current_width = pyramid->getWidth(i);
    if(current_width <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get width for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    int current_height = pyramid->getHeight(i);
    if(current_height <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get height for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat current_map;
    if(useLAB)
    {
      LabColorMap(current_image,current_width,current_height,max_dist,a_color,b_color,current_map);
    }
    else
    {
      RGBColorMap(current_image,current_width,current_height,max_dist,r_color,g_color,b_color,current_map);
    }
    
    if(!pyramid->setFeatureMap(i,current_map))
    {
     printf("[ERROR]: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramid->combinePyramid(true);
  
  if(!pyramid->getMap(map))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  
  return(AM_OK);
}

} //namespace AttentionModule