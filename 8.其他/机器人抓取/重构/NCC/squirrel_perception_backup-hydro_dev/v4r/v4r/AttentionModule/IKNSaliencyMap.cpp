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


#include "IKNSaliencyMap.hpp"

namespace AttentionModule
{
  
IKNSaliencyMap::IKNSaliencyMap():
BaseMap()
{
  reset();
}

IKNSaliencyMap::~IKNSaliencyMap()
{
}

void IKNSaliencyMap::reset()
{
  BaseMap::reset();
  weightOfColor = 1;
  weightOfIntensities = 1;
  weightOfOrientations = 1;
  numberOfOrientations = 4;

  mapName = "IKNSaliencyMap";
}

void IKNSaliencyMap::print()
{
  BaseMap::print();
  printf("[%s]: weightOfColor        = %d\n",mapName.c_str(),weightOfColor);
  printf("[%s]: weightOfIntensities  = %d\n",mapName.c_str(),weightOfIntensities);
  printf("[%s]: weightOfOrientations = %d\n",mapName.c_str(),weightOfOrientations);
  printf("[%s]: numberOfOrientations = %d\n",mapName.c_str(),numberOfOrientations);
}

void IKNSaliencyMap::setWeights(int weightOfColor_, int weightOfIntensities_, int weightOfOrientations_)
{
  weightOfColor = weightOfColor_;
  weightOfIntensities = weightOfIntensities_;
  weightOfOrientations = weightOfOrientations_;
  calculated = false;
  printf("[INFO]: %s: weightOfColor: %d.\n",mapName.c_str(),weightOfColor);
  printf("[INFO]: %s: weightOfIntensities: %d.\n",mapName.c_str(),weightOfIntensities);
  printf("[INFO]: %s: weightOfOrientations: %d.\n",mapName.c_str(),weightOfOrientations);
}

void IKNSaliencyMap::getWeights(int &weightOfColor_, int &weightOfIntensities_, int &weightOfOrientations_)
{
  weightOfColor_ = weightOfColor;
  weightOfIntensities_ = weightOfIntensities;
  weightOfOrientations_ = weightOfOrientations;
}

void IKNSaliencyMap::setNumberOfOrientations(int numberOfOrientations_)
{
  numberOfOrientations = numberOfOrientations_;
  calculated = false;
  printf("[INFO]: %s: numberOfOrientations: %d.\n",mapName.c_str(),numberOfOrientations);
}

int IKNSaliencyMap::getNumberOfOrientations()
{
  return(numberOfOrientations);
}

int IKNSaliencyMap::checkParameters()
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
  
  if((weightOfColor <= 0) || (weightOfIntensities  <= 0) || (weightOfOrientations <= 0))
  {
    return(AM_PARAMETERS);
  }
  
  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }

  return(AM_OK);
}

void IKNSaliencyMap::initializePyramid(IttiPyramid::Ptr pyramid, cv::Mat &IM, bool changeSign_)
{
  pyramid->setStartLevel(0);//
  pyramid->setMaxLevel(8);//
  pyramid->setSMLevel(4);//
  pyramid->setWidth(width);//
  pyramid->setHeight(height);//
  pyramid->setNormalizationType(normalization_type);//EPUtils::NT_NONMAX
  pyramid->setLowestC(2);//
  pyramid->setHighestC(4);//
  pyramid->setSmallestCS(3);//
  pyramid->setLargestCS(4);//
  
  pyramid->setChangeSign(changeSign_);
  
  pyramid->setImage(IM);
  pyramid->buildPyramid();
  pyramid->print();
}

int IKNSaliencyMap::calculate()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);

  printf("[INFO]: %s: Computation started.\n",mapName.c_str());

  createColorChannels();
  
  //----
  // I
  IttiPyramid::Ptr pyramidI( new IttiPyramid() );
  initializePyramid(pyramidI,I);
  
  // R
  IttiPyramid::Ptr pyramidR( new IttiPyramid() );
  initializePyramid(pyramidR,R);
  
  // G
  IttiPyramid::Ptr pyramidG( new IttiPyramid() );
  initializePyramid(pyramidG,G);
  
  // B
  IttiPyramid::Ptr pyramidB( new IttiPyramid() );
  initializePyramid(pyramidB,B);
  
  // Y
  IttiPyramid::Ptr pyramidY( new IttiPyramid() );
  initializePyramid(pyramidY,Y);
  
  // O
  std::vector<IttiPyramid::Ptr> pyramidO;
  pyramidO.resize(numberOfOrientations);
  for(int i = 0; i < numberOfOrientations; ++ i)
  {
    pyramidO.at(i) = IttiPyramid::Ptr( new IttiPyramid() );
    initializePyramid(pyramidO.at(i),I);
  }
  
  // create feature maps
  rt_code = createFeatureMapsI(pyramidI);
  if(rt_code != AM_OK)
    return(rt_code);
    
  for(int i = 0; i < numberOfOrientations; ++ i)
  {
    float angle = i*180.0/numberOfOrientations;
    rt_code = createFeatureMapsO(pyramidO.at(i),angle);
    if(rt_code != AM_OK)
      return(rt_code);
  }
  
  rt_code = createFeatureMapsRG(pyramidR,pyramidG);
  if(rt_code != AM_OK)
    return(rt_code);
  
  rt_code = createFeatureMapsRG(pyramidB,pyramidY);
  if(rt_code != AM_OK)
    return(rt_code);
  
//   for(unsigned int i = pyramidG->getStartLevel(); i <= (unsigned int)pyramidG->getMaxLevel(); ++i)
//   {
//     cv::Mat tempG;
//     if(pyramidG->getImage(i,tempG))
//     {
//       cv::imshow("G",tempG);
//       cv::waitKey(-1);
//     }
//   }
  
  float totalWeight = weightOfColor + weightOfIntensities + weightOfOrientations;
  
  cv::Mat intensity;
  if(!pyramidI->getMap(intensity))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  intensity = weightOfIntensities*intensity/totalWeight;
  
//   cv::imshow("intensity",intensity);
//   cv::waitKey(-1);
  
  cv::Mat orientation;
  for(int i = 0; i < numberOfOrientations; ++i)
  {
    cv::Mat orientation_temp;
    
    if(!pyramidO.at(i)->getMap(orientation_temp))
    {
      printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
      return(AM_CUSTOM);
    }
    
    if(i==0)
      orientation_temp.copyTo(orientation);
    else
      orientation = orientation + orientation_temp;
  }
  EPUtils::normalize(orientation,normalization_type);
  orientation = weightOfOrientations*orientation/totalWeight;
  
//   cv::imshow("orientation",orientation);
//   cv::waitKey(-1);
  
  cv::Mat colorRG;
  if(!pyramidR->getMap(colorRG))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  
//   cv::imshow("colorRG",colorRG);
//   cv::waitKey(-1);
  
  cv::Mat colorBY;
  if(!pyramidB->getMap(colorBY))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  
//   cv::imshow("colorBY",colorBY);
//   cv::waitKey(-1);
  
  cv::Mat color = colorRG + colorBY;
  EPUtils::normalize(color,normalization_type);
  color = weightOfColor*color/totalWeight;
  
//   cv::imshow("color",color);
//   cv::waitKey(-1);
  
  map = intensity + color + orientation;
  
  calculated = true;
  
  printf("[INFO]: %s: Computation finished.\n",mapName.c_str());

  return(AM_OK);
}

void IKNSaliencyMap::createColorChannels()
{
  I = cv::Mat_<float>::zeros(height,width);
  R = cv::Mat_<float>::zeros(height,width);
  G = cv::Mat_<float>::zeros(height,width);
  B = cv::Mat_<float>::zeros(height,width);
  Y = cv::Mat_<float>::zeros(height,width);

  float rr,gg,bb;
  float Imax = 0;
  
  for(int r = 0; r < height; ++r)
  {
    for (int c = 0; c < width; c++)
    {
      rr = image.at<uchar>(r,3*c+2);
      gg = image.at<uchar>(r,3*c+1);
      bb = image.at<uchar>(r,3*c+0);
      
      rr /= 255;
      gg /= 255;
      bb /= 255;
      
      if (Imax < (rr+gg+bb)/3)
      {
        Imax = (rr+gg+bb)/3;
      }
    }
  }

  for(int r = 0; r < height; ++r)
  {
    for (int c = 0; c < width; c++)
    {
      rr = image.at<uchar>(r,3*c+2);
      gg = image.at<uchar>(r,3*c+1);
      bb = image.at<uchar>(r,3*c+0);
      
      rr /= 255;
      gg /= 255;
      bb /= 255;
      
      float dI = (rr+gg+bb)/3;
      float dR = rr-(gg+bb)/2;
      float dG = gg-(rr+bb)/2;
      float dB = bb-(gg+rr)/2;
      float dY = (rr+gg)/2-(rr-gg>0? rr-gg:gg-rr)/2-bb;
      
      if (dI <= 0.1*Imax)
      {
        dI = 0;
        dR = 0;
        dG = 0;
        dB = 0;
        dY = 0;
      }
      else
      {
        dR /= dI;
        dG /= dI;
        dB /= dI;
        dY /= dI;
      }

      I.at<float>(r,c) = dI;
      R.at<float>(r,c) = dR;
      G.at<float>(r,c) = dG;
      B.at<float>(r,c) = dB;
      Y.at<float>(r,c) = dY;
    }
  }
}

int IKNSaliencyMap::createFeatureMapsI(IttiPyramid::Ptr pyramid)
{
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    cv::Mat current_image;
    if(!pyramid->getImage(i,current_image))
    {
      printf("[ERROR]: Something went wrong! Can't get image for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    if(!pyramid->setFeatureMap(i,current_image))
    {
     printf("[ERROR]: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramid->combinePyramid(true); 
  
  return(AM_OK);
  
}
  
int IKNSaliencyMap::createFeatureMapsO(IttiPyramid::Ptr pyramid, float angle)
{
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    cv::Mat current_image;
    if(!pyramid->getImage(i,current_image))
    {
      printf("[ERROR]: Something went wrong! Can't get image for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat gaborKernel0, gaborKernel90;
    EPUtils::makeGaborFilter(gaborKernel0,gaborKernel90,angle);
    cv::Mat temp0, temp90;
    cv::filter2D(current_image,temp0,-1,gaborKernel0);
    temp0 = cv::abs(temp0);
    cv::filter2D(current_image,temp90,-1,gaborKernel90);
    temp90 = cv::abs(temp90);
    cv::Mat current_map;
    cv::add(temp0,temp90,current_map);
    
    if(!pyramid->setFeatureMap(i,current_map))
    {
     printf("[ERROR]: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramid->combinePyramid(true); 
  
  return(AM_OK);
  
}
  
int IKNSaliencyMap::createFeatureMapsRG(IttiPyramid::Ptr pyramidR, IttiPyramid::Ptr pyramidG)
{
  for(unsigned int i = pyramidR->getStartLevel(); i <= (unsigned int)pyramidR->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s::createFeatureMapsRG: Computating feature map for level %d.\n",mapName.c_str(),i);

    cv::Mat current_imageR;
    if(!pyramidR->getImage(i,current_imageR))
    {
      printf("[ERROR]: createFeatureMapsRG: Something went wrong! Can't get image for level %d (pyramid 1)!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat current_imageG;
    if(!pyramidG->getImage(i,current_imageG))
    {
      printf("[ERROR]: createFeatureMapsRG: Something went wrong! Can't get image for level %d (pyramid 2)!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat current_map = current_imageR - current_imageG;
    
    if(!pyramidR->setFeatureMap(i,current_map))
    {
     printf("[ERROR]: createFeatureMapsRG: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramidR->combinePyramid(true); 
  
  return(AM_OK);
  
} 

} //AttentionModule