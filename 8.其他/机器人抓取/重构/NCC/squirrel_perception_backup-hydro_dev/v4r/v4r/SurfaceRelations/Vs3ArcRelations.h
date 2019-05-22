/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
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
 * @file Vs3ArcRelations.h
 * @author Richtsfeld
 * @date March 2013
 * @version 0.1
 * @brief Relations based on arc groupings from canny edges.
 */

#ifndef SURFACE_VS3_ARC_RELATION_H
#define SURFACE_VS3_ARC_RELATION_H

// #include <omp.h>
// #include <vector>
// #include <cstdio>
// 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
// #include "v4r/SurfaceBoundary/VisionCore.hh"
// 
// #include "ColorHistogram3D.h"
// #include "Texture.h"
// #include "Fourier.h"
// #include "Gabor.h"
// #include "BoundaryRelations.h"
// #include "ContourNormalsDistance.hh"


namespace surface
{
  
class Vs3ArcRelations
{
public:
// EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
protected:

private:
  
  bool have_input_image;
  bool have_view;
  bool preprocessed;
  
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;             ///< Input cloud
  cv::Mat edges_image;                                                ///< Canny edge image
  surface::View *view;                                          ///< Surface models

  cv::Mat_<cv::Vec3b> matImage;                                 ///< Image as Mat
  IplImage *iplImage;                                           ///< Image as IplImage

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);
  
public:
  Vs3ArcRelations();
  ~Vs3ArcRelations();

  /** Set input point cloud **/
  void setInputImage(cv::Mat &_matImage);
  
  /** Set input surface patches **/
  void setView(surface::View *_view);

  /** Preprocess the vs3 primitives **/
  void preprocess();

  /** Compute relations for the segmenter **/
//   void computeRelations();

};

/*************************** INLINE METHODES **************************/
/** Return index for coordinates x,y **/
inline int Vs3ArcRelations::GetIdx(short x, short y)
{
  return y*view->width + x;
}

/** Return x coordinate for index **/
inline short Vs3ArcRelations::X(int idx)
{
  return idx%view->width;
}

/** Return y coordinate for index **/
inline short Vs3ArcRelations::Y(int idx)
{
  return idx/view->width;
}

} //--END--

#endif

