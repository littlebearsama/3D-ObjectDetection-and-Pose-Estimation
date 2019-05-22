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

/** \author Mrinal Kalakrishnan, Ken Anderson */

#include "propagation_distance_field.h"
#include <pcl/common/io.h>
#include <pcl/search/octree.h>
//#include <pcl/visualization/pcl_visualizer.h>

namespace distance_field
{

template <typename PointT>
PropagationDistanceField<PointT>::~PropagationDistanceField()
{
}

template <typename PointT>
PropagationDistanceField<PointT>::PropagationDistanceField(double resolution):
      DistanceField<PropDistanceFieldVoxel, PointT>(resolution)
{
  resolution_ = resolution;
  extend_distance_ = 0.05f;
  huber_sigma_ = 0.003f;
}

template <typename PointT>
PropagationDistanceField<PointT>::PropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      DistanceField<PropDistanceFieldVoxel, PointT>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, PropDistanceFieldVoxel(max_distance))
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  bucket_queue_.resize(max_distance_sq_+1);

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (unsigned int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution;

  extend_distance_ = 0.05f;
  huber_sigma_ = 0.003f;
}

template <typename PointT>
unsigned int PropagationDistanceField<PointT>::eucDistSq(int3 point1, int3 point2)
{
  int dx = point1.x() - point2.x();
  int dy = point1.y() - point2.y();
  int dz = point1.z() - point2.z();
  return dx*dx + dy*dy + dz*dz;
}

template <typename PointT>
void PropagationDistanceField<PointT>::initialize(double size_x, double size_y, double size_z, double origin_x, double origin_y, double origin_z, double max_distance)
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution_);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  bucket_queue_.resize(max_distance_sq_+1);

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (unsigned int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution_;

  initializeVoxelGrid(size_x, size_y, size_z, origin_x, origin_y, origin_z, PropDistanceFieldVoxel(max_distance_sq_));
}

template <typename PointT>
void
PropagationDistanceField<PointT>::setInputCloud (typename pcl::PointCloud<PointT>::ConstPtr & cloud)
{
  //reset();
  cloud_ = cloud;

  //compute min,max to initialize the underlying voxel grid structure
  PointT min_pt_all, max_pt_all;
  pcl::getMinMax3D (*cloud_, min_pt_all, max_pt_all);
  float extend = extend_distance_;
  min_pt_all.x -= extend;
  min_pt_all.y -= extend;
  min_pt_all.z -= extend;
  max_pt_all.x += extend;
  max_pt_all.y += extend;
  max_pt_all.z += extend;

  float max_distance = (max_pt_all.getVector3fMap() - min_pt_all.getVector3fMap()).norm();
  //max_distance = 0.1f;
  initialize(std::abs(max_pt_all.x - min_pt_all.x), std::abs(max_pt_all.y - min_pt_all.y), std::abs(max_pt_all.z - min_pt_all.z), min_pt_all.x, min_pt_all.y, min_pt_all.z, max_distance);

  /*std::cout << num_cells_[DIM_X] << " " << num_cells_[DIM_Y] << " " << num_cells_[DIM_Z] << std::endl;
  std::cout << size_[DIM_X] << " " << size_[DIM_Y] << " " << size_[DIM_Z] << std::endl;
  std::cout << origin_[DIM_X] << " " << origin_[DIM_Y] << " " << origin_[DIM_Z] << std::endl;
  std::cout<< resolution_ << std::endl;*/
}

inline double huberFitzigibon(double d, double sigma)
{
  if(d <= sigma)
  {
    return (d*d)/2.0;
  }
  else
  {
    return sigma * d - sigma * sigma / 2.0;
  }
}

template <typename PointT>
float
PropagationDistanceField<PointT>::getDistance(int x, int y, int z)
{
    std::map<int3, int>::iterator it;
    PropDistanceFieldVoxel cell = getCell(x,y,z);
    int3 voxel_loc(x,y,z);
    double px,py,pz;
    gridToWorld(x,y,z,px,py,pz);
    pcl::PointXYZ p;
    p.getVector3fMap() = Eigen::Vector3f(px,py,pz);

    int idx;
    float dist;
    if(cell.occupied_)
    {
      it = voxel_locations_to_voxelized_cloud.find(voxel_loc);
      idx = cell.idx_to_input_cloud_; //it->second;
      dist = (p.getVector3fMap () - cloud_->points[idx].getVector3fMap()).norm ();
      //*dist = (p.getVector3fMap () - p_voxel.getVector3fMap()).norm ();
    }
    else
    {
      PropDistanceFieldVoxel nn = getCell(cell.closest_point_[0],cell.closest_point_[1],cell.closest_point_[2]);
      idx = nn.idx_to_input_cloud_;
      dist = (p.getVector3fMap () - cloud_->points[idx].getVector3fMap()).norm ();
    }

    return dist;
}

template <typename PointT>
void
PropagationDistanceField<PointT>::computeFiniteDifferences()
{
  finite_differences_cloud_.reset (new pcl::PointCloud<pcl::PointXYZ>);
  int pidx = 0;
  int B = 1;
  for (int x = B; x < (num_cells_[DIM_X] - B); x++)
  {
    for (int y = B; y < (num_cells_[DIM_Y] - B); y++)
    {
      for (int z = B; z < (num_cells_[DIM_Z] - B); z++)
      {
        int3 voxel_loc(x,y,z);
        pcl::PointXYZ dx;

        //double inlier = huber_sigma_;
        //double res_sq = resolution_* resolution_;

        //Lz = sqrt(huberFitzigibon(distance_plus, huber_sigma)) - sqrt(huberFitzigibon(distance_minus, huber_sigma));

        /*dx.x = sqrt (huberFitzigibon (sqrt (getCell (x + 1, y, z).distance_square_ * res_sq), huber_sigma_))
                - sqrt (huberFitzigibon (sqrt (getCell (x - 1, y, z).distance_square_ * res_sq), huber_sigma_));

        dx.y = sqrt (huberFitzigibon (sqrt (getCell (x, y + 1, z).distance_square_ * res_sq), huber_sigma_))
            - sqrt (huberFitzigibon (sqrt (getCell (x, y - 1, z).distance_square_ * res_sq), huber_sigma_));

        dx.z = sqrt (huberFitzigibon (sqrt (getCell (x, y, z + 1).distance_square_ * res_sq), huber_sigma_))
            - sqrt (huberFitzigibon (sqrt (getCell (x, y, z - 1).distance_square_ * res_sq), huber_sigma_));
        dx.getVector3fMap() /= 2;*/

        dx.x = sqrt (huberFitzigibon (getDistance(x + 1, y, z), huber_sigma_))
                - sqrt (huberFitzigibon (getDistance(x - 1, y, z), huber_sigma_));

        dx.y = sqrt (huberFitzigibon (getDistance (x, y + 1, z), huber_sigma_))
                - sqrt (huberFitzigibon (getDistance (x, y - 1, z), huber_sigma_));

        dx.z = sqrt (huberFitzigibon (getDistance (x, y, z + 1), huber_sigma_))
                - sqrt (huberFitzigibon (getDistance (x, y, z - 1), huber_sigma_));
        dx.getVector3fMap() /= 2;



        /*dx.x = sqrt (getCell (x + 1, y, z).distance_square_ * resolution_) - sqrt (getCell (x - 1, y, z).distance_square_ * resolution_);
        dx.y = sqrt (getCell (x, y  + 1, z).distance_square_ * resolution_) - sqrt (getCell (x, y - 1, z).distance_square_ * resolution_);
        dx.z = sqrt (getCell (x, y, z  + 1).distance_square_ * resolution_) - sqrt (getCell (x, y, z - 1).distance_square_ * resolution_);
        dx.getVector3fMap() /= 2;*/

        //std::cout << dx.getVector3fMap() << std::endl;
        finite_differences_cloud_->push_back(dx);
        voxel_locations_to_finite_differences_cloud.insert(std::pair<int3, int>(voxel_loc, pidx));
        pidx++;

      }
    }
  }
}

template <typename PointT>
void
PropagationDistanceField<PointT>::compute ()
{
  /*std::cout << num_cells_[DIM_X] << " " << num_cells_[DIM_Y] << " " << num_cells_[DIM_Z] << std::endl;
  std::cout << size_[DIM_X] << " " << size_[DIM_Y] << " " << size_[DIM_Z] << std::endl;
  std::cout << origin_[DIM_X] << " " << origin_[DIM_Y] << " " << origin_[DIM_Z] << std::endl;
  std::cout<< resolution_ << std::endl;
  std::cout << "Going to compute PropagationDistanceField at resolution: " << resolution_ << std::endl;*/

  reset();
  addPointsToField(cloud_);
  //std::cout << "Added points to field...: " << resolution_ << std::endl;

  pcl::octree::OctreePointCloudSearch<PointT> octree (0.001);
  octree.setInputCloud (cloud_);
  octree.addPointsFromInputCloud ();

  //voxelized_cloud_.reset (new pcl::PointCloud<PointT>);
  int pidx = 0;
  for (int x = 0; x < num_cells_[DIM_X]; x++)
  {
    for (int y = 0; y < num_cells_[DIM_Y]; y++)
    {
      for (int z = 0; z < num_cells_[DIM_Z]; z++)
      {
        int3 voxel_loc(x,y,z);
        bool already_obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
        PropDistanceFieldVoxel cell_ = getCell(x,y,z);
        cell_.occupied_ = already_obstacle_voxel;
        cell_.idx_to_input_cloud_ = -1;
        if(already_obstacle_voxel)
        {
          PointT p;
          p.getVector3fMap ()[DIM_X] = getLocationFromCell(DIM_X, x);
          p.getVector3fMap ()[DIM_Y] = getLocationFromCell(DIM_Y, y);
          p.getVector3fMap ()[DIM_Z] = getLocationFromCell(DIM_Z, z);

          std::vector<int> pointIdxNKNSearch;
          std::vector<float> pointNKNSquaredDistance;

          /*voxelized_cloud_->push_back (p);
          voxel_locations_to_voxelized_cloud.insert(std::pair<int3, int>(voxel_loc, pidx));
          pidx++;*/
          if (octree.nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
          {
            //voxelized_cloud_->push_back (cloud_->points[pointIdxNKNSearch[0]]);
            voxel_locations_to_voxelized_cloud.insert(std::pair<int3, int>(voxel_loc, pointIdxNKNSearch[0]));
            cell_.idx_to_input_cloud_ =  pointIdxNKNSearch[0];
            pidx++;
          }
          else
          {
            std::cout << "No nearest neighbour!!!" << std::endl;
          }
        }

        setCell(x,y,z, cell_);
        //std::cout << cell_.occupied_ << " " << cell_.idx_to_input_cloud_ << std::endl;
      }
    }
  }
}

template <typename PointT>
VoxelGrid<PropDistanceFieldVoxel> *
PropagationDistanceField<PointT>::extractVoxelGrid()
{
  double max_distance = std::numeric_limits<double>::max();
  VoxelGrid<PropDistanceFieldVoxel> * vx =
      new VoxelGrid<PropDistanceFieldVoxel>(size_[DIM_X], size_[DIM_Y], size_[DIM_Z], resolution_, origin_[DIM_X], origin_[DIM_Y], origin_[DIM_Z], PropDistanceFieldVoxel(max_distance));

  std::map<int3, int>::iterator it;
  for(int i=0; i < num_cells_[DIM_X]; i++)
  {
    for(int j=0; j < num_cells_[DIM_Y]; j++)
    {
      for(int k=0; k < num_cells_[DIM_Z]; k++)
      {
        if(!isCellValid(i,j,k))
          continue;

        PropDistanceFieldVoxel t = getCell(i,j,k);
        int3 voxel_loc(i,j,k);
        bool obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
        if(obstacle_voxel)
        {
          t.occupied_ = true;
          it = voxel_locations_to_voxelized_cloud.find(voxel_loc);
          t.idx_to_input_cloud_ = it->second;
        }
        else
        {
          t.occupied_ = false;
        }

        if(vx->isCellValid(i,j,k))
          vx->setCell(i,j,k, t);
      }
    }
  }

  return vx;
}

float lerp(float x1, float x2, float x, float y1, float y2)
{
  return y1 + (y2 - y1) / (x2 - x1) * (x - x1);
}

template <typename PointT>
void
PropagationDistanceField<PointT>::getDerivatives (const PointT & p, Eigen::Vector3f & d)
{
  int x,y,z;
  bool valid = this->worldToGrid(p.x, p.y, p.z, x, y, z);
  bool low_high = x < 1 || y < 1 || z < 1;
  if (!valid || low_high) {
    d[0] = d[1] = d[2] = std::numeric_limits<float>::quiet_NaN ();
    return;
  }

  int3 voxel_loc(x,y,z);
  std::map<int3, int>::iterator it;
  it = voxel_locations_to_finite_differences_cloud.find(voxel_loc);
  if(it != voxel_locations_to_finite_differences_cloud.end())
  {
    int idx = it->second;
    d = finite_differences_cloud_->points[idx].getVector3fMap();
    //TODO: trilinear interpolation?
    /*int x_c = (std::abs(p.x - (float)getLocationFromCell(DIM_X, x+1)) <= std::abs(p.x - (float)getLocationFromCell(DIM_X, x-1))) ? (x+1) : (x-1);
    int y_c = (std::abs(p.y -  (float)getLocationFromCell(DIM_Y, y+1)) <= std::abs(p.y -  (float)getLocationFromCell(DIM_Y, y-1))) ? (y+1) : (y-1);
    int z_c = (std::abs(p.z -  (float)getLocationFromCell(DIM_Z, z+1)) <= std::abs(p.z -  (float)getLocationFromCell(DIM_Z, z-1))) ? (z+1) : (z-1);

    Eigen::Vector3f d_x_c, d_y_c, d_z_c;

    {
      int3 voxel_loc(x_c,y,z);
      it = voxel_locations_to_finite_differences_cloud.find(voxel_loc);
      if(it != voxel_locations_to_finite_differences_cloud.end())
      {
        d_x_c = finite_differences_cloud_->points[it->second].getVector3fMap();
        d[0] = (x_c > x) ? lerp((float)getLocationFromCell(DIM_X, x),(float)getLocationFromCell(DIM_X, x_c), p.x, d[0], d_x_c[0])
                  : lerp((float)getLocationFromCell(DIM_X, x_c),(float)getLocationFromCell(DIM_X, x), p.x, d_x_c[0], d[0]);
      }
    }

    {
      int3 voxel_loc(x,y_c,z);
      it = voxel_locations_to_finite_differences_cloud.find(voxel_loc);
      if(it != voxel_locations_to_finite_differences_cloud.end())
      {
        d_y_c = finite_differences_cloud_->points[it->second].getVector3fMap();
        d[1] = (y_c > y) ? lerp((float)getLocationFromCell(DIM_Y, y),(float)getLocationFromCell(DIM_Y, y_c), p.y, d[1], d_y_c[1])
                  : lerp((float)getLocationFromCell(DIM_Y, y_c),(float)getLocationFromCell(DIM_Y, y), p.y, d_y_c[1], d[1]);
      }
    }

    {
      int3 voxel_loc(x,y,z_c);
      it = voxel_locations_to_finite_differences_cloud.find(voxel_loc);
      if(it != voxel_locations_to_finite_differences_cloud.end())
      {
        d_z_c = finite_differences_cloud_->points[it->second].getVector3fMap();
        d[2] = (z_c > z) ? lerp((float)getLocationFromCell(DIM_Z, z),(float)getLocationFromCell(DIM_Z, z_c), p.z, d[2], d_z_c[2])
                          : lerp((float)getLocationFromCell(DIM_Z, z_c),(float)getLocationFromCell(DIM_Z, z), p.z, d_z_c[2], d[2]);
      }
    }*/
  }
  else
  {
    d[0] = d[1] = d[2] = std::numeric_limits<float>::quiet_NaN ();
  }
}

template <typename PointT>
void
PropagationDistanceField<PointT>::getCorrespondence(const PointT & p, int * idx, float * dist, float sigma, float * color_distance)
{
  //std::cout << "Called getCorrespondence on PropagationDistanceField" << std::endl;
  int x,y,z;
  bool valid = this->worldToGrid(p.x, p.y, p.z, x, y, z);
  if (!valid /*|| !isCellValid(x,y,z)*/) {
    *dist = std::numeric_limits<float>::max ();
    *idx = -1;
    return;
  }

  int3 voxel_loc(x,y,z);
  std::map<int3, int>::iterator it;

  PropDistanceFieldVoxel cell = getCell(x,y,z);
  //std::cout << "occupied:" << cell.occupied_ << " " << isCellValid(x,y,z) << std::endl;
  //bool obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
  if(cell.occupied_)
  {
    /*PointT p_voxel;
    p_voxel.getVector3fMap ()[DIM_X] = getLocationFromCell(DIM_X, x);
    p_voxel.getVector3fMap ()[DIM_Y] = getLocationFromCell(DIM_Y, y);
    p_voxel.getVector3fMap ()[DIM_Z] = getLocationFromCell(DIM_Z, z);*/

    it = voxel_locations_to_voxelized_cloud.find(voxel_loc);
    *idx = cell.idx_to_input_cloud_; //it->second;
    //*idx = it->second;
    if(*idx != it->second)
      std::cout << *idx << " " << it->second << std::endl;

    *dist = (p.getVector3fMap () - cloud_->points[*idx].getVector3fMap()).norm ();
    //*dist = (p.getVector3fMap () - p_voxel.getVector3fMap()).norm ();
  }
  else
  {
      //if(isCellValid(cell.closest_point_[0],cell.closest_point_[1],cell.closest_point_[2]))
      //{
        PropDistanceFieldVoxel nn = getCell(cell.closest_point_[0],cell.closest_point_[1],cell.closest_point_[2]);
        *idx = nn.idx_to_input_cloud_;
        //*dist = (p.getVector3fMap () - p_voxel.getVector3fMap()).norm ();
        *dist = (p.getVector3fMap () - cloud_->points[*idx].getVector3fMap()).norm ();
      /*}
      else
      {
        *dist = std::numeric_limits<float>::max ();
        *idx = -1;
      }*/
  }
}

template <typename PointT>
void PropagationDistanceField<PointT>::updatePointsInField(typename pcl::PointCloud<PointT>::ConstPtr & points, bool iterative)
{
  if( iterative )
  {
    VoxelSet points_added;
    VoxelSet points_removed(object_voxel_locations_);

    // Compare and figure out what points are new,
    // and what points are to be deleted
    for( unsigned int i=0; i<points->points.size(); i++)
    {
      // Convert to voxel coordinates
      int3 voxel_loc;
      bool valid = this->worldToGrid(points->points[i].x, points->points[i].y, points->points[i].z, voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );
      if( valid )
      {
        if( iterative )
        {
          bool already_obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
          if( !already_obstacle_voxel )
          {
            // Not already in set of existing obstacles, so add to voxel list
            object_voxel_locations_.insert(voxel_loc);

            // Add point to the set or expansion
            points_added.insert(voxel_loc);
          }
          else
          {
            // Already an existing obstacle, so take off removal list
            points_removed.erase(voxel_loc);
          }
        }
        else
        {
          object_voxel_locations_.insert(voxel_loc);
          points_added.insert(voxel_loc);
        }
      }
    }

   removeObstacleVoxels( points_removed );
   addNewObstacleVoxels( points_added );
  }

  else	// !iterative
  {
    VoxelSet points_added;
    reset();

    for( unsigned int i=0; i<points->points.size(); i++)
    {
      // Convert to voxel coordinates
      int3 voxel_loc;
      bool valid = this->worldToGrid(points->points[i].x, points->points[i].y, points->points[i].z, voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );
      if( valid )
      {
        object_voxel_locations_.insert(voxel_loc);
        points_added.insert(voxel_loc);
      }
    }
    addNewObstacleVoxels( points_added );
  }
}

template <typename PointT>
void PropagationDistanceField<PointT>::addPointsToField(typename pcl::PointCloud<PointT>::ConstPtr & points)
{
  VoxelSet voxel_locs;

  for( unsigned int i=0; i<points->points.size(); i++)
  {
    if(!pcl_isfinite(points->points[i].x))
      continue;
    // Convert to voxel coordinates
    int3 voxel_loc;
    bool valid = this->worldToGrid(points->points[i].x, points->points[i].y, points->points[i].z,
                              voxel_loc.x(), voxel_loc.y(), voxel_loc.z() );

    if( valid )
    {
      bool already_obstacle_voxel = ( object_voxel_locations_.find(voxel_loc) != object_voxel_locations_.end() );
      if( !already_obstacle_voxel )
      {
        // Not already in set of existing obstacles, so add to voxel list
        object_voxel_locations_.insert(voxel_loc);

        // Add point to the queue for expansion
        voxel_locs.insert(voxel_loc);
      }
    }
  }

  addNewObstacleVoxels( voxel_locs );
}

template <typename PointT>
void PropagationDistanceField<PointT>::addNewObstacleVoxels(const VoxelSet& locations)
{
  int x, y, z;
  int initial_update_direction = getDirectionNumber(0,0,0);
  bucket_queue_[0].reserve(locations.size());

  VoxelSet::const_iterator it = locations.begin();
  for( it=locations.begin(); it!=locations.end(); ++it)
  {
    int3 loc = *it;
    x = loc.x();
    y = loc.y();
    z = loc.z();
    bool valid = isCellValid( x, y, z);
    if (!valid)
      continue;
    PropDistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.distance_square_ = 0;
    voxel.closest_point_ = loc;
    voxel.location_ = loc;
    voxel.update_direction_ = initial_update_direction;
    bucket_queue_[0].push_back(&voxel);
  }

  propogate();
}

template <typename PointT>
void PropagationDistanceField<PointT>::propogate()
{
  int x, y, z, nx, ny, nz;
  int3 loc;

  // now process the queue:
  for (unsigned int i=0; i<bucket_queue_.size(); ++i)
  {
    std::vector<PropDistanceFieldVoxel*>::iterator list_it = bucket_queue_[i].begin();
    //std::cout << bucket_queue_[i].size() << "Bucket:" << i << " " << max_distance_sq_ << std::endl;
    while(list_it!=bucket_queue_[i].end())
    {
      PropDistanceFieldVoxel* vptr = *list_it;
      if ( vptr == NULL)
      {
        std::cout << "Invalid pointer..." << std::endl;
        ++list_it;
        continue;
      }
      else
      {
        /*std::cout << "Valid pointer..." << std::endl;
        std::cout << "Valid pointer..." << vptr->location_.x() << std::endl;
        std::cout << "Valid pointer..." << vptr->location_.y() << std::endl;
        std::cout << "Valid pointer..." << vptr->location_.z() << std::endl;*/

        if (!isCellValid(vptr->location_.x(),vptr->location_.y(),vptr->location_.z()))
        {
          ++list_it;
          continue;
        }
      }

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      //std::cout << D << " " << vptr->update_direction_ << std::endl;
      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        PropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        if(!neighbor)
          std::cout << "Neighbor has a bad pointer..." << std::endl;
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        unsigned int new_distance_sq = eucDistSq(vptr->closest_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;

        if(vptr->closest_point_.x() == PropDistanceFieldVoxel::UNINITIALIZED)
          continue;

        if(vptr->closest_point_.y() == PropDistanceFieldVoxel::UNINITIALIZED)
          continue;

        if(vptr->closest_point_.z() == PropDistanceFieldVoxel::UNINITIALIZED)
          continue;

        if(new_distance_sq < i)
        {
          if (new_distance_sq < neighbor->distance_square_)
          {
            continue;
            std::cout << "This should not happen" << new_distance_sq << " " << i << " " << neighbor->distance_square_ << std::endl;
          }
        }

        if (new_distance_sq < neighbor->distance_square_)
        {
          // update the neighboring voxel
          neighbor->distance_square_ = new_distance_sq;
          neighbor->closest_point_ = vptr->closest_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    bucket_queue_[i].clear();
  }

}

template <typename PointT>
void PropagationDistanceField<PointT>::removeObstacleVoxels(const VoxelSet& locations )
{
  std::vector<int3> stack;
  int initial_update_direction = getDirectionNumber(0,0,0);

  stack.reserve( num_cells_[DIM_X] * num_cells_[DIM_Y] * num_cells_[DIM_Z] );
  bucket_queue_[0].reserve(locations.size());

  // First reset the obstacle voxels,
  VoxelSet::const_iterator it = locations.begin();
  for( it=locations.begin(); it!=locations.end(); ++it)
  {
    int3 loc = *it;
    bool valid = isCellValid( loc.x(), loc.y(), loc.z());
    if (!valid)
      continue;
    PropDistanceFieldVoxel& voxel = getCell(loc.x(), loc.y(), loc.z());
    voxel.distance_square_ = max_distance_sq_;
    voxel.closest_point_ = loc;
    voxel.location_ = loc;
    voxel.update_direction_ = initial_update_direction;
    stack.push_back(loc);
  }

  // Reset all neighbors who's closest point is now gone.
  while(stack.size() > 0)
  {
    int3 loc = stack.back();
    stack.pop_back();

    for( int neighbor=0; neighbor<27; neighbor++ )
    {
      int3 diff = getLocationDifference(neighbor);
      int3 nloc( loc.x() + diff.x(), loc.y() + diff.y(), loc.z() + diff.z() );

      if( isCellValid(nloc.x(), nloc.y(), nloc.z()) )
      {
        PropDistanceFieldVoxel& nvoxel = getCell(nloc.x(), nloc.y(), nloc.z());
        int3& close_point = nvoxel.closest_point_;
        PropDistanceFieldVoxel& closest_point_voxel = getCell( close_point.x(), close_point.y(), close_point.z() );

        if( closest_point_voxel.distance_square_ != 0 )
        {       // closest point no longer exists
          if( nvoxel.distance_square_!=max_distance_sq_)
          {
            nvoxel.distance_square_ = max_distance_sq_;
            nvoxel.closest_point_ = nloc;
            nvoxel.location_ = nloc;
            nvoxel.update_direction_ = initial_update_direction;
            stack.push_back(nloc);
          }
        }
        else
        {       // add to queue so we can propogate the values
          bucket_queue_[0].push_back(&nvoxel);
        }
      }
    }
  }

  propogate();
}

template <typename PointT>
void PropagationDistanceField<PointT>::reset()
{
  VoxelGrid<PropDistanceFieldVoxel>::reset(PropDistanceFieldVoxel(max_distance_sq_));
}

template <typename PointT>
void PropagationDistanceField<PointT>::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx=-1; dx<=1; ++dx)
  {
    for (int dy=-1; dy<=1; ++dy)
    {
      for (int dz=-1; dz<=1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        int3 n_point( dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n=0; n<2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx=-1; dx<=1; ++dx)
    {
      for (int dy=-1; dy<=1; ++dy)
      {
        for (int dz=-1; dz<=1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx=-1; tdx<=1; ++tdx)
          {
            for (int tdy=-1; tdy<=1; ++tdy)
            {
              for (int tdz=-1; tdz<=1; ++tdz)
              {
                if (tdx==0 && tdy==0 && tdz==0)
                  continue;
                if (n>=1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                    continue;
                  if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                    continue;
                }
                int3 n_point(tdx,tdy,tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          //printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz, neighborhoods_[n][direction_number].size());
        }
      }
    }
  }

}

template <typename PointT>
int PropagationDistanceField<PointT>::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx+1)*9 + (dy+1)*3 + dz+1;
}

template <typename PointT>
int3 PropagationDistanceField<PointT>::getLocationDifference(int directionNumber) const
{
  return direction_number_to_direction_[ directionNumber ];
}

template <typename PointT>
SignedPropagationDistanceField<PointT>::~SignedPropagationDistanceField()
{
}

template <typename PointT>
SignedPropagationDistanceField<PointT>::SignedPropagationDistanceField(double size_x, double size_y, double size_z, double resolution,
    double origin_x, double origin_y, double origin_z, double max_distance):
      DistanceField<SignedPropDistanceFieldVoxel, PointT>(size_x, size_y, size_z, resolution, origin_x, origin_y, origin_z, SignedPropDistanceFieldVoxel(max_distance,0))
{
  max_distance_ = max_distance;
  int max_dist_int = ceil(max_distance_/resolution);
  max_distance_sq_ = (max_dist_int*max_dist_int);
  initNeighborhoods();

  // create a sqrt table:
  sqrt_table_.resize(max_distance_sq_+1);
  for (int i=0; i<=max_distance_sq_; ++i)
    sqrt_table_[i] = sqrt(double(i))*resolution;
}

template <typename PointT>
int SignedPropagationDistanceField<PointT>::eucDistSq(int3 point1, int3 point2)
{
  int dx = point1.x() - point2.x();
  int dy = point1.y() - point2.y();
  int dz = point1.z() - point2.z();
  return dx*dx + dy*dy + dz*dz;
}

template <typename PointT>
void SignedPropagationDistanceField<PointT>::addPointsToField(typename pcl::PointCloud<PointT>::Ptr & points)
{
  // initialize the bucket queue
  positive_bucket_queue_.resize(max_distance_sq_+1);
  negative_bucket_queue_.resize(max_distance_sq_+1);

  positive_bucket_queue_[0].reserve(points->points.size());
  negative_bucket_queue_[0].reserve(points->points.size());

  for(int x = 0; x < num_cells_[0]; x++)
  {
    for(int y = 0; y < num_cells_[1]; y++)
    {
      for(int z = 0; z < num_cells_[2]; z++)
      {
        SignedPropDistanceFieldVoxel& voxel = getCell(x,y,z);
        voxel.closest_negative_point_.x() = x;
        voxel.closest_negative_point_.y() = y;
        voxel.closest_negative_point_.z() = z;
        voxel.negative_distance_square_ = 0;
      }
    }
  }

  // first mark all the points as distance=0, and add them to the queue
  int x, y, z, nx, ny, nz;
  int3 loc;
  int initial_update_direction = getDirectionNumber(0,0,0);
  for (unsigned int i=0; i<points.size(); ++i)
  {
    bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
    if (!valid)
      continue;
    SignedPropDistanceFieldVoxel& voxel = getCell(x,y,z);
    voxel.positive_distance_square_ = 0;
    voxel.negative_distance_square_ = max_distance_sq_;
    voxel.closest_positive_point_.x() = x;
    voxel.closest_positive_point_.y() = y;
    voxel.closest_positive_point_.z() = z;
    voxel.closest_negative_point_.x() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.closest_negative_point_.y() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.closest_negative_point_.z() = SignedPropDistanceFieldVoxel::UNINITIALIZED;
    voxel.location_.x() = x;
    voxel.location_.y() = y;
    voxel.location_.z() = z;
    voxel.update_direction_ = initial_update_direction;
    positive_bucket_queue_[0].push_back(&voxel);
  }

  // now process the queue:
  for (unsigned int i=0; i<positive_bucket_queue_.size(); ++i)
  {
    std::vector<SignedPropDistanceFieldVoxel*>::iterator list_it = positive_bucket_queue_[i].begin();
    while(list_it!=positive_bucket_queue_[i].end())
    {
      SignedPropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_positive_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->positive_distance_square_)
        {
          // update the neighboring voxel
          neighbor->positive_distance_square_ = new_distance_sq;
          neighbor->closest_positive_point_ = vptr->closest_positive_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          positive_bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    positive_bucket_queue_[i].clear();
  }


  for(unsigned int i = 0; i < points.size(); i++)
    {
      bool valid = worldToGrid(points[i].x(), points[i].y(), points[i].z(), x, y, z);
      if(!valid)
        continue;

      for(int dx = -1; dx <= 1; dx ++)
      {
        for(int dy = -1; dy<= 1; dy ++)
        {
          for(int dz = -1; dz <= 1; dz++)
          {
            nx = x + dx;
            ny = y + dy;
            nz = z + dz;

            if(!isCellValid(nx, ny, nz))
              continue;

            SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);

            if(neighbor->closest_negative_point_.x() != SignedPropDistanceFieldVoxel::UNINITIALIZED)
            {
              neighbor->update_direction_ = initial_update_direction;
              negative_bucket_queue_[0].push_back(neighbor);
            }
          }
        }
      }

    }

  for (unsigned int i=0; i<negative_bucket_queue_.size(); ++i)
  {
    std::vector<SignedPropDistanceFieldVoxel*>::iterator list_it = negative_bucket_queue_[i].begin();
    while(list_it!=negative_bucket_queue_[i].end())
    {
      SignedPropDistanceFieldVoxel* vptr = *list_it;

      x = vptr->location_.x();
      y = vptr->location_.y();
      z = vptr->location_.z();

      // select the neighborhood list based on the update direction:
      std::vector<int3 >* neighborhood;
      int D = i;
      if (D>1)
        D=1;
      // avoid a possible segfault situation:
      if (vptr->update_direction_<0 || vptr->update_direction_>26)
      {
   //     ROS_WARN("Invalid update direction detected: %d", vptr->update_direction_);
        ++list_it;
        continue;
      }

      neighborhood = &neighborhoods_[D][vptr->update_direction_];

      for (unsigned int n=0; n<neighborhood->size(); n++)
      {
        int dx = (*neighborhood)[n].x();
        int dy = (*neighborhood)[n].y();
        int dz = (*neighborhood)[n].z();
        nx = x + dx;
        ny = y + dy;
        nz = z + dz;
        if (!isCellValid(nx,ny,nz))
          continue;

        // the real update code:
        // calculate the neighbor's new distance based on my closest filled voxel:
        SignedPropDistanceFieldVoxel* neighbor = &getCell(nx, ny, nz);
        loc.x() = nx;
        loc.y() = ny;
        loc.z() = nz;
        int new_distance_sq = eucDistSq(vptr->closest_negative_point_, loc);
        if (new_distance_sq > max_distance_sq_)
          continue;
        if (new_distance_sq < neighbor->negative_distance_square_)
        {
          // update the neighboring voxel
          neighbor->negative_distance_square_ = new_distance_sq;
          neighbor->closest_negative_point_ = vptr->closest_negative_point_;
          neighbor->location_ = loc;
          neighbor->update_direction_ = getDirectionNumber(dx, dy, dz);

          // and put it in the queue:
          negative_bucket_queue_[new_distance_sq].push_back(neighbor);
        }
      }

      ++list_it;
    }
    negative_bucket_queue_[i].clear();
  }

}

template <typename PointT>
void SignedPropagationDistanceField<PointT>::reset()
{
  VoxelGrid<SignedPropDistanceFieldVoxel>::reset(SignedPropDistanceFieldVoxel(max_distance_sq_, 0));
}

template <typename PointT>
void SignedPropagationDistanceField<PointT>::initNeighborhoods()
{
  // first initialize the direction number mapping:
  direction_number_to_direction_.resize(27);
  for (int dx=-1; dx<=1; ++dx)
  {
    for (int dy=-1; dy<=1; ++dy)
    {
      for (int dz=-1; dz<=1; ++dz)
      {
        int direction_number = getDirectionNumber(dx, dy, dz);
        int3 n_point( dx, dy, dz);
        direction_number_to_direction_[direction_number] = n_point;
      }
    }
  }

  neighborhoods_.resize(2);
  for (int n=0; n<2; n++)
  {
    neighborhoods_[n].resize(27);
    // source directions
    for (int dx=-1; dx<=1; ++dx)
    {
      for (int dy=-1; dy<=1; ++dy)
      {
        for (int dz=-1; dz<=1; ++dz)
        {
          int direction_number = getDirectionNumber(dx, dy, dz);
          // target directions:
          for (int tdx=-1; tdx<=1; ++tdx)
          {
            for (int tdy=-1; tdy<=1; ++tdy)
            {
              for (int tdz=-1; tdz<=1; ++tdz)
              {
                if (tdx==0 && tdy==0 && tdz==0)
                  continue;
                if (n>=1)
                {
                  if ((abs(tdx) + abs(tdy) + abs(tdz))!=1)
                    continue;
                  if (dx*tdx<0 || dy*tdy<0 || dz*tdz <0)
                    continue;
                }
                int3 n_point(tdx,tdy,tdz);
                neighborhoods_[n][direction_number].push_back(n_point);
              }
            }
          }
          //printf("n=%d, dx=%d, dy=%d, dz=%d, neighbors = %d\n", n, dx, dy, dz, neighborhoods_[n][direction_number].size());
        }
      }
    }
  }



}

template <typename PointT>
int SignedPropagationDistanceField<PointT>::getDirectionNumber(int dx, int dy, int dz) const
{
  return (dx+1)*9 + (dy+1)*3 + dz+1;
}


}

template class distance_field::PropagationDistanceField<pcl::PointXYZ>;
template class distance_field::PropagationDistanceField<pcl::PointXYZRGB>;
template class distance_field::PropagationDistanceField<pcl::PointXYZRGBA>;
template class distance_field::PropagationDistanceField<pcl::PointNormal>;
template class distance_field::PropagationDistanceField<pcl::PointXYZRGBNormal>;
