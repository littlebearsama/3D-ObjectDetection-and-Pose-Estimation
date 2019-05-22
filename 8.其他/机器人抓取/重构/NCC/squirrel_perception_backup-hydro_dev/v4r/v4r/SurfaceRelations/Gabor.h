/**
 * @file Gabor.h
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use garbor filter to compare surface texture.
 */

#ifndef SURFACE_GABOR_H
#define SURFACE_GABOR_H

#include <vector>
#include <stdio.h>

#include <opencv2/opencv.hpp>
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

#include "cvgabor.h"

#include "v4r/SurfaceUtils/SurfaceModel.hpp"

namespace surface
{

class Gabor
{
public:
  
protected:

private:
  bool useDilation;                // set true to use dilation of mask
  int dilationsize;          // size of dilation

  bool have_image;
  cv::Mat image;                   // gray level image in cv-Mat
  bool have_indices;
  pcl::PointIndices::Ptr indices;
  int width;
  int height;

  bool have_gabor_filters;
  
  bool computed;                   // true, when results available
  
  //@ep: TODO create a structeure for all parameters
  
  double F;                        // dF ... The spatial frequency
  double Sigma;                    // TODO Sigma
  int N;                           // Number of orientations
  int M_min, M_max;                // minimum and maximum scale factor
  int filtersNumber;               // size of gabor filters (orientations * number scales)
//   CvGabor gabor;
  
public:
  
  typedef boost::shared_ptr<Gabor> Ptr;
  
  Gabor();
  ~Gabor();
  
  /** Activate dilation and set size **/
  void setDilation(int _dilationsize);
  
  /** Set input image **/
  void setInputImage(cv::Mat& _image);
  // sets indices
  void setIndices(pcl::PointIndices::Ptr _indices);
  void setIndices(std::vector<int> &_indices);
  void setIndices(cv::Rect rect);
  
//   void setGaborFilters(std::vector<cv::Mat> _gaborFilters);

  /** Compute the gabor features **/
  void compute();
  
  /** Compare surfaces with gabor filter **/
  double compare(Gabor::Ptr g);
  
  std::vector<double> featureVector;
  
  int max_ori_nr;
  double max_ori;
  
  std::vector<cv::Mat> gaborFilters;         // Gabor filter image saved as IplImage 
  
  void computeGaborFilters();
  
};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

