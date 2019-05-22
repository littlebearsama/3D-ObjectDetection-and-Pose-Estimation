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
 * @file Gabor.cpp
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use garbor filter to compare surface texture.
 */

#include "Gabor.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

Gabor::Gabor()
{
  useDilation = false;
  
  have_image = false;
  have_indices = false;
  computed = false;
  have_gabor_filters = false;

  F = sqrt(1.5);    // Spatial frequency (usually 2, but 1.5 works better)
  Sigma = 2*PI;     //
  N = 6;            // Number of orientations 0,30,60,90,120,150
  M_min = -2;       // Minimum scale factor (icpr=-2)
  M_max = 2;        // Maximum scale factor (icpr=2)
  filtersNumber = N*abs(M_max - M_min +1);
}

Gabor::~Gabor()
{
}

// ================================= Private functions ================================= //


// ================================= Public functions ================================= //

void Gabor::setDilation(int _dilationsize)
{
  useDilation = true;
  dilationsize = _dilationsize;
}

void Gabor::setInputImage(cv::Mat& _image)
{
  if ( (_image.cols<=0) || (_image.rows<=0) )
    throw std::runtime_error("[Gabor::setInputImage] Invalid image (height|width must be > 1)");
  
  if ( _image.type() != CV_8UC1 )
    throw std::runtime_error("[Gabor::setInputImage] Invalid image type (must be 8UC1)");

  image = _image;
  width = image.cols;
  height = image.rows;
  
  have_image = true;
  computed = false;
  
  indices.reset(new pcl::PointIndices);
  for(int i = 0; i < width*height; i++)
  {
    indices->indices.push_back(i);
  }
}

void Gabor::setIndices(pcl::PointIndices::Ptr _indices)
{
  if(!have_image) {
    printf("[Gabor::setIndices]: Error: No image available.\n");
    return;
  }
    
  indices = _indices;
  have_indices = true;
}

void Gabor::setIndices(std::vector<int> &_indices)
{
  indices.reset(new pcl::PointIndices);
  indices->indices = _indices;
  
  have_indices = true;
}

void Gabor::setIndices(cv::Rect _rect)
{
  if(!have_image) {
    printf("[Gabor::setIndices]: Error: No image available.\n");
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
  
  indices.reset(new pcl::PointIndices);
  for(int r = _rect.y; r < (_rect.y+_rect.height); r++)
  {
    for(int c = _rect.x; c < (_rect.x+_rect.width); c++)
    {
      indices->indices.push_back(r*width+c);
    }
  }
  
  have_indices = true;
}

// void Gabor::setGaborFilters(std::vector<cv::Mat> _gaborFilters)
// {
//   gaborFilters.resize(_gaborFilters.size());
//   
//   for(int i = 0; i < _gaborFilters.size(); ++i)
//   {
//     _gaborFilters.at(i).copyTo(gaborFilters.at(i));
//   }
//   
//   have_gabor_filters = true;
// }

void Gabor::computeGaborFilters()
{
  gaborFilters.resize(filtersNumber);
  
  CvGabor gabor;
  // orientation
  for(int ori = 0; ori < N; ori++) 
  {
    // scale
    for(int scale = M_min; scale <= M_max; scale++) 
    {
      int idx = ori*abs(M_max - M_min +1) + (scale-M_min);
      
      double orientation = (((double)(PI*ori))/N);  
      gabor.Init(orientation,scale,Sigma,F);
      //@ep: BUG this is not optimal at all, we should either compute gabor filter once, or only for each patch
      //@ep: TODO figure out what type to use
      IplImage temp_image = image;
      IplImage* temp_filter = cvCreateImage(cvGetSize(&temp_image), 8, 1);
      gabor.conv_img(&temp_image,temp_filter,CV_GABOR_MAG);
      gaborFilters.at(idx) = cv::Mat(temp_filter,true);
      cvReleaseImage(&temp_filter);
    }
  }
  
  have_gabor_filters = true;
  
}

void Gabor::compute()
{
  if(!have_image) {
    printf("[Gabor::compute]: Error: No image available.\n");
    return;
  }
  
  if(!have_gabor_filters) {
    printf("[Gabor::compute]: Error: No gabor filters available.\n");
    return;
  }
  
  featureVector.resize(2*filtersNumber);
  
  double normalise = 1.0f;    // normalise = 255
  
  double mean = 0.0f;
  double stddev = 0.0f;
  
  // calculate mean value
  for(int fi = 0; fi < filtersNumber; fi++) 
  {
    for(unsigned int idx = 0; idx < indices->indices.size(); idx++)
    {
      int i = indices->indices.at(idx) / width;
      int j = indices->indices.at(idx) % width;
      //@ep: BUG there is a bug here potentially because of char/uchar
      mean += ((double) gaborFilters.at(fi).at<char>(i,j) / normalise);
    }
    
    mean /= ((double) indices->indices.size());
    
    featureVector.at(2*fi) = mean;
    
  }
  
  // calculate standard deviation
  for(int fi = 0; fi < filtersNumber; fi++) 
  {
    for(unsigned int idx = 0; idx < indices->indices.size(); idx++)
    {
      int i = indices->indices.at(idx) / width;
      int j = indices->indices.at(idx) % width;
      stddev += pow(((double) gaborFilters.at(fi).at<char>(i,j) / normalise) - mean, 2);
    }
    
    stddev /= ((double) (indices->indices.size()-1));
    stddev = sqrt(stddev);

    featureVector.at(2*fi+1) = stddev;
  }
  
  // find the orientation with highest energy
  // sum of magnitude (energy) for one orientation
  std::vector<double> ori_mag_sum;
  ori_mag_sum.resize(N);
  for(int ori = 0; ori < N; ori++) 
  {
    ori_mag_sum.at(ori) = 0.0f;
  }
  
  for(int ori = 0; ori < N; ori++) 
  {
    // scale
    for(int scale = M_min; scale <= M_max; scale++) 
    {
      int idx = ori*abs(M_max - M_min +1) + (scale-M_min);
      ori_mag_sum.at(ori) += featureVector.at(2*idx);
    }
  }
  
  max_ori_nr = 0;
  max_ori = 0.0f;
  
  //@ep: BUG potentially, if response of the filter can be negative
  for(int ori = 0; ori < N; ori++)
  {
    if(ori_mag_sum.at(ori) > max_ori) 
    {
      max_ori = ori_mag_sum.at(ori);
      max_ori_nr = ori;
    }
  }
  
  // Shift, when orientation is different
  if(max_ori_nr != 0)
  {
    std::vector<double> shiftedFeatureVector;
    shiftedFeatureVector.resize(2*filtersNumber);
    
    int shift = 2*(max_ori_nr*abs(M_max - M_min +1));
    
    for(int idx = 0; idx < (2*filtersNumber); idx++) 
    {
      int shiftedIdx = (idx+shift) % (2*filtersNumber);
      shiftedFeatureVector.at(idx) = featureVector.at(shiftedIdx);
    }
    
    featureVector = shiftedFeatureVector;
  }
  
  computed = true; 
}

double Gabor::compare(Gabor::Ptr g)
{
  bool normalisaton_by_energy = true;
  
  /// NOW calculate the distance of the feature vectors
  /// sqrt((mu_n-mu_m)² + (si_n-si_m)²)
  double gabor_distance = 0.0f;
  for(int idx = 0; idx < 2*filtersNumber; idx += 2) 
  {
    double mu = pow(featureVector.at(idx) - g->featureVector.at(idx), 2);
    double sig = pow(featureVector.at(idx+1) - g->featureVector.at(idx+1), 2);
    gabor_distance += sqrt(mu + sig);
  }
  
  // normalisaton value is the maximum energy in one direction (orientation)
  double norm_energy = fmax(max_ori, g->max_ori);
  
  if(normalisaton_by_energy) 
  {
    if(norm_energy > 0)
    {
      return gabor_distance/norm_energy;
    }
    else 
    {
      return 0;
    }
  }
  else
  {
    return gabor_distance;
  }
}

}
