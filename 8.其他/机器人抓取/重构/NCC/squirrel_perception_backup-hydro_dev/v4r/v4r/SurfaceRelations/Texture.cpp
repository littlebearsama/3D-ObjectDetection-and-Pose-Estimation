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
 * @file Texture.cpp
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate texture feature to compare surface texture.
 */

#include "Texture.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

Texture::Texture()
{
  computed = false;
  have_edges = false;
  have_indices = false;
}

Texture::~Texture()
{
}

void Texture::setInputEdges(cv::Mat &_edges)
{
  if ( (_edges.cols<=0) || (_edges.rows<=0) )
    throw std::runtime_error("[ColorHistogram::setInputEdges] Invalid image (height|width must be > 1)");
  
  if ( _edges.type() != CV_8UC1 )
    throw std::runtime_error("[ColorHistogram::setInputEdges] Invalid image type (must be 8UC1)");

  edges = _edges;
  width = edges.cols;
  height = edges.rows;
  
  have_edges = true;
  computed = false;
  
  indices.reset(new pcl::PointIndices);
  for(int i = 0; i < width*height; i++)
  {
    indices->indices.push_back(i);
  }
}

void Texture::setIndices(pcl::PointIndices::Ptr _indices)
{
  if(!have_edges) {
    printf("[ColorHistogram::setIndices]: Error: No edges available.\n");
    return;
  }
    
  indices = _indices;
  have_indices = true;
}

void Texture::setIndices(std::vector<int> &_indices)
{
  indices.reset(new pcl::PointIndices);
  indices->indices = _indices;
}

void Texture::setIndices(cv::Rect _rect)
{
  if(!have_edges) {
    printf("[ColorHistogram::setIndices]: Error: No edges available.\n");
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
  
  have_indices = true;
}

void Texture::compute()
{
  if(!have_edges) {
    printf("[Texture::compute] Error: No edges set.\n");
    return;
  }

  textureRate = 0.0;
  
  if( (indices->indices.size()) <= 0) 
  {
    computed = true;
    return;
  }
  
  int texArea = 0;
  for(unsigned int i=0; i<indices->indices.size(); i++) 
  {
    int x = indices->indices.at(i) % width;
    int y = indices->indices.at(i) / width;
    if(edges.at<uchar>(y,x) == 255)
      texArea++;
  }
  
  textureRate = (double) ((double)texArea / indices->indices.size());
  
  computed = true;
}

double Texture::compare(Texture::Ptr t)
{
  if(!computed || !(t->getComputed())) 
  {
    printf("[Texture::compare]: Error: Texture is not computed.\n");
    return 0.;
  }
  
  return 1. - fabs(textureRate - t->getTextureRate());
}


} // end surface












