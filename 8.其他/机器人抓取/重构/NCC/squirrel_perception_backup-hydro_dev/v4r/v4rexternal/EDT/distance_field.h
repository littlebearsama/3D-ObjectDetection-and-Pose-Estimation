/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author Mrinal Kalakrishnan */

#ifndef DF_DISTANCE_FIELD_H_
#define DF_DISTANCE_FIELD_H_


#include "voxel_grid.h"
#include <vector>
#include <list>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>

namespace distance_field
{

/**
* \brief A VoxelGrid that can convert a set of obstacle points into a distance field.
*
* It computes the distance transform of the input points, and stores the distance to
* the closest obstacle in each voxel. Also available is the location of the closest point,
* and the gradient of the field at a point. Expansion of obstacles is performed upto a given
* radius.
*
* This is an abstract base class, current implementations include PropagationDistanceField
* and PFDistanceField.
*/
template <typename T, typename PointT>
class DistanceField: public VoxelGrid<T>
{
public:

  /**
   * \brief Constructor for the VoxelGrid.
   *
   * @param size_x Size (x axis) of the container in meters
   * @param size_y Size (y axis) of the container in meters
   * @param size_z Size (z axis) of the container in meters
   * @param resolution: resolution (size of a single cell) in meters
   * @param origin_x Origin (x axis) of the container
   * @param origin_y Origin (y axis) of the container
   * @param origin_z Origin (z axis) of the container
   * @param default_object The object to return for an out-of-bounds query
   */
  DistanceField(double size_x, double size_y, double size_z, double resolution,
      double origin_x, double origin_y, double origin_z, T default_object);

  DistanceField(double resolution);

  virtual ~DistanceField();


  /**
   * \brief Add (and expand) a set of points to the distance field.
   *
   * This function will incrementally add the given points and update the distance field
   * correspondingly. Use the reset() function if you need to remove all points and start
   * afresh.
   */
  virtual void addPointsToField(typename pcl::PointCloud<PointT>::ConstPtr & points)=0;

  /**
   * \brief Resets the distance field to the max_distance.
   */
  virtual void reset()=0;

  /**
   * \brief Gets the distance to the closest obstacle at the given location.
   */
  double getDistance(double x, double y, double z) const;

  /**
   * \brief Gets the distance at a location and the gradient of the field.
   */
  double getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const;

  /**
   * \brief Gets the distance to the closest obstacle at the given integer cell location.
   */
  double getDistanceFromCell(int x, int y, int z) const;

protected:
  virtual double getDistance(const T& object) const=0;
  using VoxelGrid<T>::isCellValid;
  using VoxelGrid<T>::getCell;
  using VoxelGrid<T>::num_cells_;
  using VoxelGrid<T>::DIM_X;
  using VoxelGrid<T>::DIM_Y;
  using VoxelGrid<T>::DIM_Z;
  using VoxelGrid<T>::initializeVoxelGrid;
  using VoxelGrid<T>::getLocationFromCell;
  using VoxelGrid<T>::ref;
  using VoxelGrid<T>::gridToWorld;
  using VoxelGrid<T>::origin_;
  using VoxelGrid<T>::size_;

private:
  int inv_twice_resolution_;
};

//////////////////////////// template function definitions follow //////////////

template <typename T, typename PointT>
DistanceField<T, PointT>::~DistanceField()
{

}

template <typename T, typename PointT>
DistanceField<T, PointT>::DistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, T default_object):
      VoxelGrid<T>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, default_object)
{
  inv_twice_resolution_ = 1.0/(2.0*resolution);
}

template <typename T, typename PointT>
DistanceField<T, PointT>::DistanceField(double resolution):
      VoxelGrid<T>(resolution)
{
  inv_twice_resolution_ = 1.0/(2.0*resolution);
}

template <typename T, typename PointT>
double DistanceField<T, PointT>::getDistance(double x, double y, double z) const
{
  return getDistance((*this)(x,y,z));
}

template <typename T, typename PointT>
double DistanceField<T, PointT>::getDistanceGradient(double x, double y, double z, double& gradient_x, double& gradient_y, double& gradient_z) const
{
  int gx, gy, gz;

  this->worldToGrid(x, y, z, gx, gy, gz);

  // if out of bounds, return 0 distance, and 0 gradient
  // we need extra padding of 1 to get gradients
  if (gx<1 || gy<1 || gz<1 || gx>=this->num_cells_[this->DIM_X]-1 || gy>=this->num_cells_[this->DIM_Y]-1 || gz>=this->num_cells_[this->DIM_Z]-1)
  {
    gradient_x = 0.0;
    gradient_y = 0.0;
    gradient_z = 0.0;
    return 0;
  }

  gradient_x = (getDistanceFromCell(gx+1,gy,gz) - getDistanceFromCell(gx-1,gy,gz))*inv_twice_resolution_;
  gradient_y = (getDistanceFromCell(gx,gy+1,gz) - getDistanceFromCell(gx,gy-1,gz))*inv_twice_resolution_;
  gradient_z = (getDistanceFromCell(gx,gy,gz+1) - getDistanceFromCell(gx,gy,gz-1))*inv_twice_resolution_;

  return getDistanceFromCell(gx,gy,gz);

}

template <typename T, typename PointT>
double DistanceField<T, PointT>::getDistanceFromCell(int x, int y, int z) const
{
  return getDistance(this->getCell(x,y,z));
}

}
#endif /* DF_DISTANCE_FIELD_H_ */
