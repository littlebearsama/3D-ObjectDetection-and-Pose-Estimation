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
 * $Id$
 * Johann Prankl, 12.1.2012
 */

#ifndef PCLA_CONTOUR_DETECTION_HH
#define PCLA_CONTOUR_DETECTION_HH

#include <iostream>
#include <stdexcept>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/feature.h>
#include <boost/shared_ptr.hpp>




namespace pclA 
{

/**
 * Contour detection from an segment of an organized point cloud
 */
class ContourDetection
{
public:
  class Parameter
  {
    public:
      bool orderPoints;
      bool approxContour;
      unsigned randTrials;

      Parameter(bool order=true, bool approx=false, unsigned _randTrials=5) 
       :  orderPoints(order), approxContour(approx), randTrials(_randTrials) {}
  };

private:
  Parameter param;

  int width, height;                                    ///< point cloud width and height // TODO ersetzen mit cloud->width and cloud->height
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;         ///< point cloud

  static unsigned idcnt;                                ///< id counter
  std::vector<unsigned> idMap;                          ///< id map
  std::vector<unsigned char> contourMap;                ///< contour map

  void GetContourPoints(const std::vector<int> &indices, std::vector<int> &contour);
  unsigned GetNeighbour(std::vector<unsigned char> &contourMap, unsigned idx, unsigned char tag);
  void TraceContour(std::vector<int> &in, std::vector<int> &out, unsigned idxStart);

  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);



public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ContourDetection(Parameter p=Parameter());
  ~ContourDetection();

  /** Set parameter for contour detection **/
  void setParameter(Parameter &p);

  /** Set input point cloud **/
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);

  /** Compute results **/
  void compute(const std::vector<int> &in, std::vector<int> &out);
};




/*********************** INLINE METHODES **************************/
/** Return index for coordinates x,y **/
inline int ContourDetection::GetIdx(short x, short y)
{
  return y*width+x;
}

/** Return x coordinate for index **/
inline short ContourDetection::X(int idx)
{
  return idx%width;
}

/** Return y coordinate for index **/
inline short ContourDetection::Y(int idx)
{
  return idx/width;
}



}

#endif

