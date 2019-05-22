/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
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

/**
 * @file ColorHistogram.cpp
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Color histogram class.
 */

#include <string.h>
#include "ColorHistogram.h"

namespace surface
{

ColorHistogram::ColorHistogram(int _nrBins, double _UVthreshold)
{
  nrBins = _nrBins;
  computed = false;
  have_input_cloud = false;
  have_indices = false;
  colorModel = YUV_MODEL;
  histogramType = HIST_3D;
  maxVal = 255.;
  UVthreshold = _UVthreshold;
  useUVthreshold = false;
  hist = cv::Mat_<double>::zeros(1,1);
}

void ColorHistogram::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{ 
  if ( (_cloud->height<=1) || (_cloud->width<=1) || (!_cloud->isOrganized()) )
    throw std::runtime_error("[ColorHistogram::setInputCloud] Invalid point cloud (height must be > 1)");

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;
  
  have_input_cloud = true;
  computed = false;
  
  indices.reset(new pcl::PointIndices);
  for(unsigned i = 0; i < cloud->points.size(); i++)
  {
    indices->indices.push_back(i);
  }
}

void ColorHistogram::setIndices(pcl::PointIndices::Ptr _indices)
{
  if(!have_input_cloud) {
    printf("[ColorHistogram::setIndices]: Error: No input cloud available.\n");
    return;
  }
    
  indices = _indices;
  have_indices = true;
}

void ColorHistogram::setIndices(std::vector<int> &_indices)
{
  indices.reset(new pcl::PointIndices);
  indices->indices = _indices;
}

void ColorHistogram::setIndices(cv::Rect _rect)
{
  if(!have_input_cloud) {
    printf("[ColorHistogram::setIndices]: Error: No input cloud available.\n");
    return;
  }
  
  if(_rect.y >= height)
  {
    _rect.y = height-1;
  }
  
  if( (_rect.y + _rect.height) >= height)
  {
    _rect.height = height-_rect.y;
  }
  
  if(_rect.x >= width)
  {
    _rect.x = width-1;
  }
  
  if( (_rect.x + _rect.width) >= width)
  {
    _rect.width = width-_rect.x;
  }
  
  printf("[ColorHistogram] _rect = %d,%d,%d,%d.\n",_rect.x,_rect.y,_rect.x+_rect.width,_rect.y+_rect.height);
  
  indices.reset(new pcl::PointIndices);
  for(int r = _rect.y; r < (_rect.y+_rect.height); r++)
  {
    for(int c = _rect.x; c < (_rect.x+_rect.width); c++)
    {
      indices->indices.push_back(r*width+c);
    }
  }
}

bool ColorHistogram::init()
{
  hist = cv::Mat_<double>::zeros(1,1);

  switch(histogramType)
  {
    case HIST_AVERAGE_HIST:
      hist = cv::Mat_<double>::zeros(nrBins,3);
      return(true);
    case HIST_2D:
      hist = cv::Mat_<double>::zeros(nrBins,nrBins);
      return(true);
    case HIST_3D:
      hist = cv::Mat_<double>::zeros(nrBins,nrBins*nrBins);
      return(true);
    default:
      return(false);
      
  }
}

void ColorHistogram::getYUV(pclA::RGBValue &color, Color3C &convColor)
{
  //@ep: Wikipedia says that transformation matrix is different
  //@ep: why +128???
  //@ep: where exactly those numbers are comming from?
  //@ep: why yuv and not hsl or hsv???
  //@ep: where is the colors ordering stored???
  convColor.ch1 =  (0.257 * color.r) + (0.504 * color.g) + (0.098 * color.b) + 16;
  convColor.ch2 = -(0.148 * color.r) - (0.291 * color.g) + (0.439 * color.b) + 128;
  convColor.ch3 =  (0.439 * color.r) - (0.368 * color.g) - (0.071 * color.b) + 128;
  /*
   int Y =  (0.257 * pt.r) + (0.504 * pt.g) + (0.098 * pt.b) + 16;
   i nt U = -(0.148 * pt.r) - (0.291 * pt.g) + (0.439 * pt.b) + 128;*
   int V =  (0.439 * pt.r) - (0.368 * pt.g) - (0.071 * pt.b) + 128;
   */
}

void ColorHistogram::getRGB(pclA::RGBValue &color, Color3C &convColor)
{
  convColor.ch1 = color.r;
  convColor.ch2 = color.g;
  convColor.ch3 = color.b;
}

bool ColorHistogram::getColor(pclA::RGBValue color, Color3C &convColor)
{
  switch(colorModel)
  {
    case YUV_MODEL:
      getYUV(color,convColor);
      return(true);
    case RGB_MODEL:
      getRGB(color,convColor);
      return(true);
    case BGR_MODEL:
      return(false);
    default:
      return(false);
  }
}

bool ColorHistogram::buildHistogram3D()
{
  if(colorModel != YUV_MODEL)
  {
    printf("[ColorHistogram::compute::buildHistogram3D] Warning: only yuv model is supported for 3D histogram at the moment!\n");
    return(false);
  }

  int noCol = 0;
  for(size_t i=0; i<indices->indices.size(); i++)
  {
    //@ep: is PointXYZRGB::r correct?
    //pcl::RGBValue color = ....   // out of date! instead use PointXYZRGB::r directly
    pclA::RGBValue color;
    color.float_value = cloud->points.at(indices->indices.at(i)).rgb;
    
    Color3C convColor;
    if(!getColor(color,convColor))
      return(false);

    int U2 = convColor.ch2-128;        // shifted to the middle
    int V2 = convColor.ch3-128;        // shifted to the middle
    if((U2*U2 + V2*V2) < UVthreshold)
    {
      noCol++;
    }
    else
    {
      double yBin = convColor.ch1*(double)nrBins/maxVal;
      double uBin = convColor.ch2*(double)nrBins/maxVal;
      double vBin = convColor.ch3*(double)nrBins/maxVal;

      hist.at<double>((int)yBin,((int)uBin)*nrBins + (int)vBin) += 1;
    }
  }
  
  double normalization = indices->indices.size() - noCol;
  if(normalization != 0)
  {
    for(int i=0; i<nrBins; i++)
    {
      for(int j=0; j<nrBins; j++)
      {
        for(int k=0; k<nrBins; k++)
        {
          hist.at<double>((int)i,((int)j)*nrBins + (int)k) /= normalization;
        }
      }
    }
  }
  else
  {
    for(int i=0; i<nrBins; i++)
    {
      for(int j=0; j<nrBins; j++)
      {
        for(int k=0; k<nrBins; k++)
        {
          hist.at<double>((int)i,((int)j)*nrBins + (int)k) = 0;
        }
      }
    }
  }
      
  computed = true;
  return(true);
}

bool ColorHistogram::buildHistogram2D()
{
  if(colorModel != YUV_MODEL)
  {
    printf("[ColorHistogram::compute::buildHistogram2D] Warning: only yuv model is supported for 2D histogram at the moment!\n");
    return(false);
  }

  int noCol = 0;
  for(unsigned i=0; i<indices->indices.size(); i++)
  {
    pclA::RGBValue color;
    color.float_value = cloud->points.at(indices->indices.at(i)).rgb;

    Color3C convColor;
    if(!getColor(color,convColor))
      return(false);
    
    int U2 = convColor.ch2-128;        /// shifted to the middle
    int V2 = convColor.ch3-128;        /// shifted to the middle

    if((U2*U2 + V2*V2) < UVthreshold)
    {
      noCol++;
    }
    else
    {
      double uBin = convColor.ch2 * (double)nrBins/maxVal;
      double vBin = convColor.ch3 * (double)nrBins/maxVal;
      hist.at<double>((int)uBin,(int)vBin) += 1;
    }
  }
    
  double normalization = indices->indices.size() - noCol;
  if(normalization != 0)
  {
    for(int i=0; i<nrBins; i++)
    {
      for(int j=0; j<nrBins; j++)
      {
        hist.at<double>(i,j) /= normalization;
      }
    }
  }
  else
  {
    for(int i=0; i<nrBins; i++)
    {
      for(int j=0; j<nrBins; j++)
      {
        hist.at<double>(i,j) = 0;
      }
    }
  }
    
  return(true);
}


bool ColorHistogram::buildHistogramAverage()
{
  for(unsigned i = 0; i < indices->indices.size(); i++)
  {
    pclA::RGBValue color;
    color.float_value = cloud->points.at(indices->indices.at(i)).rgb;

    Color3C convColor;
    if(!getColor(color,convColor))
      return(false);
    
    if(colorModel != YUV_MODEL)
    {
      double bin1 = convColor.ch1*(double)nrBins/maxVal;
      hist.at<double>((int)bin1,0) += 1;
    }
    double bin2 = convColor.ch2*(double)nrBins/maxVal;
    hist.at<double>((int)bin2,1) += 1;
    double bin3 = convColor.ch3*(double)nrBins/maxVal;
    hist.at<double>((int)bin3,2) += 1;
  }
  
  for(int i = 0; i < nrBins; i++)
  {
    hist.at<double>(i,0) /= indices->indices.size();
    hist.at<double>(i,1) /= indices->indices.size();
    hist.at<double>(i,2) /= indices->indices.size();
  }

  return(true); 
}

bool ColorHistogram::buildHistogram()
{
  switch(histogramType)
  {
    case HIST_AVERAGE_HIST:
      return(buildHistogramAverage());
    case HIST_2D:
      return(buildHistogram2D());
    case HIST_3D:
      return(buildHistogram3D());
    default:
      return(false);
      
  }
}

void ColorHistogram::compute()
{
  if(!init())
  {
    printf("[ColorHistogram::compute::init]: Error: Cannot init histograms. Check histogram type.\n");
    return;
  }
  
  if(!buildHistogram())
  {
    printf("[ColorHistogram::compute::buildHistogram]: Error: Cannot init histograms. Check histogram type.\n");
    return;
  }
  
  computed = true;
}

double ColorHistogram::compareAverage(ColorHistogram::Ptr ch)
{
  //@ep: why fidelity metric???
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for(int i=0; i<nrBins; i++)
  {
    if(colorModel != YUV_MODEL)
    {
      overall_sum += sqrt(hist.at<double>(i,0) * (ch->getHist()).at<double>(i,0));
    }
    overall_sum += sqrt(hist.at<double>(i,1) * (ch->getHist()).at<double>(i,1));
    overall_sum += sqrt(hist.at<double>(i,2) * (ch->getHist()).at<double>(i,2));
  }

  switch(colorModel)
  {
    case YUV_MODEL:
      return(overall_sum / 2);
    case RGB_MODEL:
      return(overall_sum / 3);
    default:
      return(0.0);
  }

  return(0.0);
  
}

double ColorHistogram::compare2D(ColorHistogram::Ptr ch)
{
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for(int i=0; i<nrBins; i++)
  {
    for(int j=0; j<nrBins; j++)
    {
      overall_sum += sqrt(hist.at<double>(i,j) * (ch->hist).at<double>(i,j));
    }
  }
      
  if(overall_sum > 1.)
  {
    printf("Warning: ColorHistogram::compare2D: Value larger than 1!!!\n");
    printHistogram();
  }
        
  double fidelity = overall_sum;
  return fidelity;
}

double ColorHistogram::compare3D(ColorHistogram::Ptr ch)
{
  // YUV: Fidelity d=(SUM(sqrt(Pi*Qi)))
  double overall_sum = 0;
  for(int i=0; i<nrBins; i++)
  {
    for(int j=0; j<nrBins; j++)
    {
      for(int k=0; k<nrBins; k++)
      {
        overall_sum += sqrt( (hist.at<double>((int)i,((int)j)*nrBins + (int)k)) * (ch->hist).at<double>((int)i,((int)j)*nrBins + (int)k) );
      }
    }
  }
  
  if(overall_sum > 1.)
  {
    printf("Warning: ColorHistogram::compare3D: Value larger than 1!!!\n");
    printHistogram();
  }

  double fidelity = overall_sum;
  //double bhattacharyya = -log(overall_sum);
  
  return fidelity;
}

double ColorHistogram::compare(ColorHistogram::Ptr ch)
{
  if(nrBins != ch->getBinNumber())
  {
    printf("[ColorHistogram::compare]: Error: Cannot compare histograms with different bin sizes.\n");
    return 0.;
  }
  if(!computed || !ch->getComputed()) 
  {
    printf("[ColorHistogram::compare]: Error: Color histogram not computed.\n");
    return 0.;
  }

  if(colorModel != ch->getColorModel())
  {
    printf("[ColorHistogram::compare]: Error: Different color models.\n");
    return 0.;
  }

  if(histogramType != ch->getHistogramType())
  {
    printf("[ColorHistogram::compare]: Error: Different histogram types.\n");
    return 0.;
  }
  
  switch(histogramType)
  {
    case HIST_AVERAGE_HIST:
      return(compareAverage(ch));
    case HIST_2D:
      return(compare2D(ch));
    case HIST_3D:
      return(compare3D(ch));
    default:
      return 0.;
  }
}

void ColorHistogram::printHistogramAverage()
{
  printf("[ColorHistogram::printHistogram]:\n");

  switch(colorModel)
  {
    case YUV_MODEL:
      for(int i=0; i<nrBins; i++)
        printf(" y[%u]: %4.3f\n", i, hist.at<double>(i,0));
      for(int i=0; i<nrBins; i++)
        printf(" u[%u]: %4.3f\n", i, hist.at<double>(i,1));
      for(int i=0; i<nrBins; i++)
        printf(" v[%u]: %4.3f\n", i, hist.at<double>(i,2));
      return;
    case RGB_MODEL:
      for(int i=0; i<nrBins; i++)
        printf(" red[%u]: %4.3f\n", i, hist.at<double>(i,0));
      for(int i=0; i<nrBins; i++)
        printf(" gre[%u]: %4.3f\n", i, hist.at<double>(i,1));
      for(int i=0; i<nrBins; i++)
        printf(" blu[%u]: %4.3f\n", i, hist.at<double>(i,2));
      return;
    default:
      return;
  }

  return;
}

void ColorHistogram::printHistogram2D()
{
  printf("[ColorHistogram::printHistogram]:\n");
  
  switch(colorModel)
  {
    case YUV_MODEL:
      for(int i=0; i<nrBins; i++)
        for(int j=0; j<nrBins; j++)
          printf(" yuv[%u][%u]: %4.3f\n", i, j, hist.at<double>(i,j));
      return;
    default:
      return;
  }
  
  return;
}

void ColorHistogram::printHistogram3D()
{
  printf("[ColorHistogram::printHistogram]:\n");
  
  for(int i=0; i<nrBins; i++)
    for(int j=0; j<nrBins; j++)
      for(int k=0; k<nrBins; k++)
        printf(" yuv[%u][%u][%u]: %4.3f\n", i, j, k, hist.at<double>(i,j*nrBins+k));

  return;
}

void ColorHistogram::printHistogram()
{
  switch(histogramType)
  {
    case HIST_AVERAGE_HIST:
      printHistogramAverage();
      return;
    case HIST_2D:
      printHistogram2D();
      return;
    case HIST_3D:
      printHistogram3D();
      return;
    default:
      return;
  }
}

}

