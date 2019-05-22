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


#include "OrientationMap.hpp"

namespace AttentionModule
{

OrientationSaliencyMap::OrientationSaliencyMap():
BaseMap()
{
  reset();
}

OrientationSaliencyMap::~OrientationSaliencyMap()
{
}

void OrientationSaliencyMap::reset()
{
  BaseMap::reset();
  angle = 0;
  max_sum = 1;
  bandwidth = 2;

  mapName = "OrientationSaliencyMap";
}

void OrientationSaliencyMap::print()
{
  BaseMap::print();
  printf("[%s]: angle              = %f\n",mapName.c_str(),angle);
  printf("[%s]: max_sum            = %f\n",mapName.c_str(),max_sum);
  printf("[%s]: bandwidth          = %f\n",mapName.c_str(),bandwidth);
}

void OrientationSaliencyMap::setAngle(float angle_)
{
  angle = angle_;
  calculated = false;
  printf("[INFO]: %s: angle: %f.\n",mapName.c_str(),angle);
}

void OrientationSaliencyMap::setBandwidth(float bandwidth_)
{
  bandwidth = bandwidth_;
  calculated = false;
  printf("[INFO]: %s: bandwidth: %f.\n",mapName.c_str(),bandwidth);
}

float OrientationSaliencyMap::getAngle()
{
  return(angle);
}

float OrientationSaliencyMap::getBandwidth()
{
  return(bandwidth);
}

float OrientationSaliencyMap::getMaxSum()
{
  return(max_sum);
}

int OrientationSaliencyMap::checkParameters()
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

int OrientationSaliencyMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  cv::Mat image_cur;
  image.copyTo(image_cur);
  image_cur.convertTo(image_cur,CV_32F,1.0f/255);
  cv::Mat image_gray;
  cv::cvtColor(image_cur,image_gray,CV_BGR2GRAY);
  
  cv::blur(image_gray,image_gray,cv::Size(filter_size,filter_size));
  
  orientationMap(image_gray,width,height,angle,max_sum,bandwidth,map);
  
  cv::blur(map,map,cv::Size(filter_size,filter_size));

  EPUtils::normalize(map,normalization_type);

  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

void OrientationSaliencyMap::orientationMap(cv::Mat &image_cur, int image_width, int image_height, float angle, float max_sum, float bandwidth, cv::Mat &map_cur)
{
  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  //create Gabor kernel
  cv::Mat gaborKernel;
  EPUtils::makeGaborKernel2D(gaborKernel,max_sum,angle,bandwidth);
  assert (gaborKernel.rows == gaborKernel.cols);
  assert (gaborKernel.rows % 2 == 1);
  int gaborFilerSize = gaborKernel.rows / 2;
  
  for(int r = 0; r < image_height; ++r)
  {
    //for(int c = gaborFilerSize; c < width-gaborFilerSize; ++c)
    for(int c = 0; c < image_width; ++c)
    {
      float value = 0;
      for(int j = r-gaborFilerSize; j <= r+gaborFilerSize; ++j) // rows
      {
	int yy = j;
	if(j < 0)
	{
	  yy = -j;
	}
	if(j >= image_height)
	{
	  yy = image_height-(j-image_height+1)-1;
	}
	  
	for(int i = c-gaborFilerSize; i <= c+gaborFilerSize; ++i) // cols
	{
	  int xx = i;
	  if(i < 0)
	  {
	    xx = -i;
	  }
	  if(i >= image_width)
	  {
	    xx = image_width-(i-image_width+1)-1;
	  }
	    
	  value += image_cur.at<float>(yy,xx)*gaborKernel.at<float>(j-(r-gaborFilerSize),(i-(c-gaborFilerSize)));
        }
      }

      map_cur.at<float>(r,c) = value;
      
    }
  }
  
  map_cur = cv::abs(map_cur);
  map_cur = map_cur / max_sum;
  cv::sqrt(map_cur,map_cur);

  return;
}

int OrientationSaliencyMap::calculatePyramidSimple()
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
  image_cur.convertTo(image_cur,CV_32F,1.0f/255);
  cv::Mat image_gray;
  cv::cvtColor(image_cur,image_gray,CV_BGR2GRAY);
  
  pyramid->setImage(image_gray);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int OrientationSaliencyMap::calculatePyramidItti()
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
  image_cur.convertTo(image_cur,CV_32F,1.0f/255);
  cv::Mat image_gray;
  cv::cvtColor(image_cur,image_gray,CV_BGR2GRAY);
  
  pyramid->setImage(image_gray);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int OrientationSaliencyMap::calculatePyramidFrintrop()
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
  image_cur.convertTo(image_cur,CV_32F,1.0f/255);
  cv::Mat image_gray;
  cv::cvtColor(image_cur,image_gray,CV_BGR2GRAY);
  
  pyramid->setImage(image_gray);
  pyramid->buildPyramid();
  pyramid->print();

  combinePyramid(pyramid);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int OrientationSaliencyMap::combinePyramid(BasePyramid::Ptr pyramid)
{
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
    
    orientationMap(current_image,current_width,current_height,angle,max_sum,bandwidth,current_map);
    
//     cv::imshow("current_image",current_image);
//     cv::imshow("current_map",current_map);
//     cv::waitKey(-1);
    
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