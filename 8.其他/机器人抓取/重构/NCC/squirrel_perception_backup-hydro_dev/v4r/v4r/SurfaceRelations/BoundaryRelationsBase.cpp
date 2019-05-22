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
 * @file BoundaryRelationsBase.cpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Base class to calculate boundary relations between patches.
 */

#include "BoundaryRelationsBase.hpp"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

BoundaryRelationsBase::BoundaryRelationsBase()
{
  have_cloud = false;
  have_normals = false;
  have_boundary = false;

  computed = false;
}

BoundaryRelationsBase::~BoundaryRelationsBase()
{
}

void BoundaryRelationsBase::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _cloud)
{
  if ( (_cloud->height<=1) || (_cloud->width<=1) || (!_cloud->isOrganized()) )
    throw std::runtime_error("[BoundaryRelationsBase::setInputCloud] Invalid point cloud (height must be > 1)");

  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  have_cloud = true;
  have_normals = false;
  have_boundary = false;
  computed = false;
}

void BoundaryRelationsBase::setNormals(pcl::PointCloud<pcl::Normal>::Ptr _normals)
{
  if(!have_cloud)
    throw std::runtime_error("[BoundaryRelationsBase::setNormals] I suggest you first set the point cloud.");
  
  if ( (_normals->height!=(unsigned int)height) || (_normals->width!=(unsigned int)width) )
    throw std::runtime_error("[BoundaryRelationsBase::setNormals] Invalid normals (not for this point cloud).");

  normals = _normals;
  have_normals = true;
}

void BoundaryRelationsBase::setBoundary(std::vector<surface::neighboringPair> &_boundary)
{
  boundary = _boundary;
  have_boundary = true;
}

surface::meanVal BoundaryRelationsBase::compute()
{
  throw std::runtime_error("[BoundaryRelationsBase::compute] Why do you call me? I am just the base class!.");
  surface::meanVal m;
  return m;
}

/*bool BoundaryRelations::compare(BoundaryRelations::Ptr br)
{
  //@ep: TODO check reconditions
  if(!have_cloud) {
    printf("[BoundaryRelations::compute] Error: No input cloud set.\n");
    exit(0);
  }

  //if(!have_surfaces) {
  //  printf("[BoundaryRelations::compute] Error: No surface models set.\n");
  //  exit(0);
  //}

  cv::Mat patches = cv::Mat_<uchar>::zeros(height, width);
  uchar p1 = 1;
  uchar p2 = 2;
  createPatchImage(indices,p1,patches);
  createPatchImage(br->getIndices(),p2,patches);

  //rel_value.resize(0);

  // @ep::this should be pairs (structure), not simply numbers, it makes no sense
  std::vector<int> first_ngbr;
  std::vector<int> second_ngbr;
  std::vector<int> first_3D_ngbr;
  std::vector<int> second_3D_ngbr;
  std::vector<int> dir_3D_ngbr;

  // get neighbouring pixel-pairs in 2D and in 3D
  for(int i=1; i<height; i++)
  {
    for(int j=1; j<width; j++)
    {
      int b_first = 0;
      int b_second = 0;
      //@ep: this value should be settable
      double b_distance = 10.;
      int dir_3D = 0;

     //  priorities
     //  4 1 3
     //  2 x .
     //  . . .

      int dx[4] = {0,-1,1,-1};
      int dy[4] = {-1,0,-1,-1};
      int dir[4] = {1,3,0,2};

      bool found = false;
      for(int k = 0; k < 4; ++k)
      {
        int i1 = i;
        int i2 = i + dy[k];
        int j1 = j;
        int j2 = j + dx[k];

        if((patches.at<uchar>(i1,j1) == p1 && patches.at<uchar>(i2,j2) == p2) ||
           (patches.at<uchar>(i1,j1) == p2 && patches.at<uchar>(i2,j2) == p1))
        {
          int idx1 = i1*width + j1;
          int idx2 = i2*width + j2;
          // distance between
          double distance = fabs(cloud->points.at(idx1).z - cloud->points.at(idx2).z);
          //check if the distance is valid
          //@ep:: are we sure that 10 m is a valid distance? or its simply check for nans???
          if(distance < b_distance)
          {
            b_first = idx1;
            b_second = idx2;
            b_distance = distance;
          }
          dir_3D = dir[k];
          found = true;
          break;
        }
      }

      if(found)
      {
        first_ngbr.push_back(b_first);
        second_ngbr.push_back(b_second);
        // z coordinate of the point
        double z_dist = cloud->points.at(i*width + j).z;
        double adaptive_distance = max3DDistancePerMeter*z_dist;
        if(b_distance < adaptive_distance) {
          first_3D_ngbr.push_back(b_first);
          second_3D_ngbr.push_back(b_second);
          dir_3D_ngbr.push_back(dir_3D);
        }
      }
    }
  }

  // Write number of neighbours into Surface model
  //@ep:: TODO: this should be done separately, really, messing things up is not a good strategy
  //if(view->surfaces[p0]->neighbors2D.find(p1) != view->surfaces[p0]->neighbors2D.end())
  //  view->surfaces[p0]->neighbors2DNrPixel[p1] = first_ngbr.size();
  //if(view->surfaces[p1]->neighbors2D.find(p0) != view->surfaces[p1]->neighbors2D.end())
  //  view->surfaces[p1]->neighbors2DNrPixel[p0] = first_ngbr.size();

  int nr_valid_points_color = 0;
  int nr_valid_points_depth = 0;
  double sum_uv_color_distance = 0.0f;
  double sum_2D_curvature = 0.0f;
  double sum_depth = 0.0f;
  double sum_depth_var = 0.0f;
  std::vector<double> depth_vals;

  // calculate mean depth
  for(int i=0; i<first_ngbr.size(); i++)
  {
    double p0_z, p1_z;
    if (projectPts)
    {
      p0_z = cloud_model->points.at(first_ngbr.at(i)).z;
      p1_z = cloud_model->points.at(second_ngbr.at(i)).z;
    } else
    {
      p0_z = cloud->points.at(first_ngbr.at(i)).z;
      p1_z = cloud->points.at(second_ngbr.at(i)).z;
    }
    //@ep: check for nan should be here, I have no clue what will happen if I subtract two nan values
    double depth = fabs(p0_z - p1_z);
    // no nan
    //@ep: there should be a special function for this
    if(depth == depth)
    {
      depth_vals.push_back(depth);
      nr_valid_points_depth++;
      sum_depth += depth;
    }
    else
      printf("[BoundaryRelations::compare] Warning: Invalid depht points (nan): Should not happen! Why?\n");
  }

  // normalize depth sum and calculate depth variance
  //@ep: this shoule be separate function in the utils
  if(nr_valid_points_depth != 0)
  {
    sum_depth /= nr_valid_points_depth;
    for(unsigned i=0; i<depth_vals.size(); i++)
      sum_depth_var += fabs(depth_vals.at(i) - sum_depth);
    sum_depth_var /= nr_valid_points_depth;
  }
  else
    std::printf("[BoundaryRelations::compare] Warning: Number of valid depth points is zero: sum_depth: %4.3f\n", sum_depth);


  /// calcuate curvature / depth
  int nr_valid_points_curvature3D = 0;
  double sum_3D_curvature_mean = 0.0f;
  double sum_3D_curvature_var = 0.0f;
  std::vector<double> curvature_vals;  // single curvature values
  for(unsigned i=0; i<first_3D_ngbr.size(); i++)
  {
    /// calculate color similarity on 3D border
    nr_valid_points_color++;

    //double p0_Y =  (0.257 * p0_color.b) + (0.504 * p0_color.g) + (0.098 * p0_color.r) + 16;
    double p0_U = -(0.148 * cloud->points.at(first_3D_ngbr.at(i)).b) -
                   (0.291 * cloud->points.at(first_3D_ngbr.at(i)).g) +
                   (0.439 * cloud->points.at(first_3D_ngbr.at(i)).r) + 128;    // use bgr
    double p0_V =  (0.439 * cloud->points.at(first_3D_ngbr.at(i)).b) -
                   (0.368 * cloud->points.at(first_3D_ngbr.at(i)).g) -
                   (0.071 * cloud->points.at(first_3D_ngbr.at(i)).r) + 128;
    //double p1_Y =  (0.257 * p1_color.b) + (0.504 * p1_color.g) + (0.098 * p1_color.r) + 16;
    double p1_U = -(0.148 * cloud->points.at(second_3D_ngbr.at(i)).b) -
                   (0.291 * cloud->points.at(second_3D_ngbr.at(i)).g) +
                   (0.439 * cloud->points.at(second_3D_ngbr.at(i)).r) + 128;
    double p1_V =  (0.439 * cloud->points.at(second_3D_ngbr.at(i)).b) -
                   (0.368 * cloud->points.at(second_3D_ngbr.at(i)).g) -
                   (0.071 * cloud->points.at(second_3D_ngbr.at(i)).r) + 128;

    double u_1 = p0_U/255 - p1_U/255;
    double u_2 = u_1 * u_1;
    double v_1 = p0_V/255 - p1_V/255;
    double v_2 = v_1 * v_1;
    double cDist = sqrt(u_2 + v_2);
    sum_uv_color_distance += cDist;

    /// calculate mean 3D curvature
    cv::Vec3f pt0, pt1;
    if (projectPts)
    {
      pt0[0]= cloud_model->points.at(first_3D_ngbr.at(i)).x;
      pt0[1]= cloud_model->points.at(first_3D_ngbr.at(i)).y;
      pt0[2]= cloud_model->points.at(first_3D_ngbr.at(i)).z;
      pt1[0]= cloud_model->points.at(second_3D_ngbr.at(i)).x;
      pt1[1]= cloud_model->points.at(second_3D_ngbr.at(i)).y;
      pt1[2]= cloud_model->points.at(second_3D_ngbr.at(i)).z;
    } else {
      pt0[0]= cloud->points.at(first_3D_ngbr.at(i)).x;
      pt0[1]= cloud->points.at(first_3D_ngbr.at(i)).y;
      pt0[2]= cloud->points.at(first_3D_ngbr.at(i)).z;
      pt1[0]= cloud->points.at(second_3D_ngbr.at(i)).x;
      pt1[1]= cloud->points.at(second_3D_ngbr.at(i)).y;
      pt1[2]= cloud->points.at(second_3D_ngbr.at(i)).z;
    }

    //@ep: is it check for nan??? than there should be && not ||
    if(pt0 == pt0 || pt1 == pt1)
    {
      cv::Vec3f p0_normal;
      p0_normal[0] = normals->points.at(first_3D_ngbr.at(i)).normal_x;
      p0_normal[1] = normals->points.at(first_3D_ngbr.at(i)).normal_y;
      p0_normal[2] = normals->points.at(first_3D_ngbr.at(i)).normal_z;
      cv::Vec3f p1_normal;
      p1_normal[0] = normals->points.at(second_3D_ngbr.at(i)).normal_x;
      p1_normal[1] = normals->points.at(second_3D_ngbr.at(i)).normal_y;
      p1_normal[2] = normals->points.at(second_3D_ngbr.at(i)).normal_z;

      cv::Vec3f pp;
      if(dir_3D_ngbr[i] == 0) {
        pp[0] = -1.0; pp[1] = 0.0; pp[2] = 0.0;
      }
      else if(dir_3D_ngbr[i] == 1) {
        pp[0] = -1.0; pp[1] = -1.0; pp[2] = 0.0;
      }
      else if(dir_3D_ngbr[i] == 2) {
        pp[0] = 0.0; pp[1] = -1.0; pp[2] = 0.0;
      }
      else if(dir_3D_ngbr[i] == 3) {
        pp[0] = 1.0; pp[1] = -1.0; pp[2] = 0.0;
      }
      cv::Vec3f pp_dir = cv::normalize(pp);

      double a_p0_pp = acos(p0_normal.ddot(pp_dir));
      pp_dir = -pp_dir; // invert direction between points
      double a_p1_pp = acos(p1_normal.ddot(pp_dir));

      double curvature = 0.0;
      //@ep: why do you want to check for nans here?
      if(a_p0_pp == a_p0_pp && a_p1_pp == a_p1_pp) {
        nr_valid_points_curvature3D++;
        //@ep: I think it should be fabs here
        curvature = a_p0_pp + a_p1_pp - M_PI;
        curvature_vals.push_back(curvature);
        sum_3D_curvature_mean += curvature;
      }
#ifdef DEBUG
      else
        printf("[BoundaryRelations::compare] Warning: Invalid curvature points (nan): Should not happen! DO SOMETHING!\n");
#endif
    }
  }

  if(nr_valid_points_color != 0)
    sum_uv_color_distance /= nr_valid_points_color;
#ifdef DEBUG
  else
    printf("[BoundaryRelations::compare] Warning: Number of valid color points is zero: sum_color: %4.3f\n", sum_uv_color_distance);
#endif

  if(nr_valid_points_curvature3D != 0) {
    sum_3D_curvature_mean /= nr_valid_points_curvature3D;
    for(unsigned i=0; i<curvature_vals.size(); i++) {
      sum_3D_curvature_var += fabs(curvature_vals[i] - sum_3D_curvature_mean);
    }
    sum_3D_curvature_var /= nr_valid_points_depth;
  }
#ifdef DEBUG
  else
    printf("[BoundaryRelations::compare] Warning: Number of valid 3D curvature points is zero: sum_3D_curvature: %4.3f\n", sum_3D_curvature_mean);
#endif

  //rel_value.push_back(1.-sum_uv_color_distance);
  //rel_value.push_back(sum_depth);
  //rel_value.push_back(sum_depth_var);
  //rel_value.push_back(sum_2D_curvature);            /// TODO We do not use that: Remove that at one point
  //rel_value.push_back(sum_3D_curvature_mean);
  //rel_value.push_back(sum_3D_curvature_var);
  //rel_value.push_back((double)first_3D_ngbr.size() / (double) first_ngbr.size());



  return true;
}*/



} // end surface












