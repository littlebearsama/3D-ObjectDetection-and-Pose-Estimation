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
 * @file Fourier.h
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use fourier filter to compare surface texture.
 */

#ifndef SURFACE_FOURIER_H
#define SURFACE_FOURIER_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include <omp.h>

#include <opencv2/opencv.hpp>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"

namespace surface
{

class Fourier
{
public:
  
protected:

private:
  
  bool computed;
  bool have_image;
  cv::Mat image;
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  
  int width, height;
  
  int N;            // Number of neighbors
  int kmax;         // maximum number of discrete fourier transformation coefficient
  int nbins;        // number of histogram bins
  int binWidth;     // Width of one bin (32 width => 8x32 = 256)
  int binStretch;   // Stretch factor for bins of higher order (k=1,...)
  
public:
  
  typedef boost::shared_ptr<Fourier> Ptr;
  
  Fourier();
  ~Fourier();
  
  /** Set input point cloud **/
  // sets input cloud
  //
  void setInputImage(cv::Mat &_image);
  // sets indices
  void setIndices(pcl::PointIndices::Ptr _indices);
  void setIndices(std::vector<int> &_indices);
  void setIndices(cv::Rect rect);

  bool getComputed() {return computed;};
  
  /** Compute the texture **/
  void compute();
  
  /** Compare surface texture **/
  double compare(Fourier::Ptr f);

  /** Check the results visually on images for each dft-component **/
//   void check();

  double *bins;
  uchar *dft;         // dft results for kmax = 5 coefficients
  bool *used;
  
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

