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
 * @file Texture.h
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate texture feature to compare surface texture.
 */

#ifndef SURFACE_TEXTURE_H
#define SURFACE_TEXTURE_H

#include <vector>
#include <stdio.h>

#include <opencv2/opencv.hpp>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"

namespace surface
{

class Texture
{
private:

  bool computed;
  bool have_edges;
  cv::Mat edges;
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  
  int width, height;
  
  double textureRate;                                      ///< Texture rate for each surface

public:
  
  typedef boost::shared_ptr<Texture> Ptr;
  
  Texture();
  ~Texture();
  
  /** Set input point cloud **/
  // sets input cloud
  void setInputEdges(cv::Mat &_edges);
  // sets indices
  void setIndices(pcl::PointIndices::Ptr _indices);
  void setIndices(std::vector<int> &_indices);
  void setIndices(cv::Rect rect);

  double getTextureRate() {return textureRate;};
  bool getComputed() {return computed;};
  
  /** Compute the texture **/
  void compute();
  
  /** Compare surface texture **/
  double compare(Texture::Ptr t);
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

