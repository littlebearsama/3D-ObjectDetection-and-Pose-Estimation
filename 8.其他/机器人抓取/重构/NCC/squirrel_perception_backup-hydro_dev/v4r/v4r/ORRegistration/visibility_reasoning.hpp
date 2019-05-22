/*
 * visibility_reasoning.hpp
 *
 *  Created on: Mar 19, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_REGISTRATION_VISIBILITY_REASONING_HPP_
#define FAAT_PCL_REGISTRATION_VISIBILITY_REASONING_HPP_

#include <v4r/ORRegistration/visibility_reasoning.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>

template<typename PointT>
  int
  faat_pcl::registration::VisibilityReasoning<PointT>::computeRangeDifferencesWhereObserved (const typename pcl::PointCloud<PointT>::ConstPtr & im1,
                                                                                             const typename pcl::PointCloud<PointT>::ConstPtr & im2,
                                                                                                 std::vector<float> & range_diff)
  {
    float cx, cy;
    cx = static_cast<float> (cx_) / 2.f; //- 0.5f;
    cy = static_cast<float> (cy_) / 2.f; // - 0.5f;

    range_diff.resize (im2->points.size ());
    int keep = 0;
    for (size_t i = 0; i < im2->points.size (); i++)
    {
      if (!pcl_isfinite(im2->points[i].z))
        continue;

      float x = im2->points[i].x;
      float y = im2->points[i].y;
      float z = im2->points[i].z;
      int u = static_cast<int> (focal_length_ * x / z + cx);
      int v = static_cast<int> (focal_length_ * y / z + cy);

      if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
        continue;

      //Check if point depth (distance to camera) is greater than the (u,v) meaning that the point is not visible
      if (!pcl_isfinite(im1->at(u,v).z))
        continue;

      range_diff[keep] = (im1->at (u, v).z - z);
      keep++;
    }

    range_diff.resize (keep);
    return keep;
  }

  template<typename PointT>
    int
    faat_pcl::registration::VisibilityReasoning<PointT>::computeRangeDifferencesWhereObservedWithIndicesBack (const typename pcl::PointCloud<PointT>::ConstPtr & im1,
                                                                                                              const typename pcl::PointCloud<PointT>::ConstPtr & im2,
                                                                                                              std::vector<float> & range_diff,
                                                                                                              std::vector<int> & indices)
    {
      float cx, cy;
      cx = static_cast<float> (cx_) / 2.f; //- 0.5f;
      cy = static_cast<float> (cy_) / 2.f; // - 0.5f;

      range_diff.resize (im2->points.size ());
      indices.resize (im2->points.size ());
      int keep = 0;
      for (size_t i = 0; i < im2->points.size (); i++)
      {
        if (!pcl_isfinite(im2->points[i].z))
          continue;

        float x = im2->points[i].x;
        float y = im2->points[i].y;
        float z = im2->points[i].z;
        int u = static_cast<int> (focal_length_ * x / z + cx);
        int v = static_cast<int> (focal_length_ * y / z + cy);

        if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
          continue;

        //Check if point depth (distance to camera) is greater than the (u,v) meaning that the point is not visible
        if (!pcl_isfinite(im1->at(u,v).z))
          continue;

        range_diff[keep] = (im1->at (u, v).z - z);
        indices[keep] = static_cast<int>(i);
        keep++;
      }

      range_diff.resize (keep);
      indices.resize (keep);
      return keep;
    }

template<typename PointT>
float
faat_pcl::registration::VisibilityReasoning<PointT>::computeFocalLength (int cx_, int cy_, const typename pcl::PointCloud<PointT>::ConstPtr & scene)
{
  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f - 0.5f;
  cy = static_cast<float> (cy_) / 2.f - 0.5f;
  float f;
  //compute the focal length
  float max_u, max_v, min_u, min_v;
  max_u = max_v = std::numeric_limits<float>::max () * -1;
  min_u = min_v = std::numeric_limits<float>::max ();

  for (size_t i = 0; i < scene->points.size (); i++)
  {
    float b_x = scene->points[i].x / scene->points[i].z;
    if (b_x > max_u)
      max_u = b_x;
    if (b_x < min_u)
      min_u = b_x;

    float b_y = scene->points[i].y / scene->points[i].z;
    if (b_y > max_v)
      max_v = b_y;
    if (b_y < min_v)
      min_v = b_y;
  }

  float maxC = std::max (std::max (std::abs (max_u), std::abs (max_v)), std::max (std::abs (min_u), std::abs (min_v)));
  f = (cx) / maxC;
  return f;
}

template<typename PointT>
void
faat_pcl::registration::VisibilityReasoning<PointT>::computeRangeImage (int cx_, int cy_, float f_, const typename pcl::PointCloud<PointT>::ConstPtr & cloud,
                                                                            typename pcl::PointCloud<PointT>::Ptr & range_image)
{
  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f; //- 0.5f;
  cy = static_cast<float> (cy_) / 2.f; // - 0.5f;

  float * depth_;
  depth_ = new float[cx_ * cy_];
  for (int i = 0; i < (cx_ * cy_); i++)
    depth_[i] = std::numeric_limits<float>::quiet_NaN ();

  range_image.reset(new pcl::PointCloud<PointT>);
  range_image->width = cx_;
  range_image->height = cy_;
  range_image->points.resize(cx_ * cy_);

  for(size_t u=0; u < cx_; u++)
  {
    for(size_t v=0; v < cy_; v++)
    {
      range_image->at(u,v).x = std::numeric_limits<float>::quiet_NaN();
      range_image->at(u,v).y = std::numeric_limits<float>::quiet_NaN();
      range_image->at(u,v).z = std::numeric_limits<float>::quiet_NaN();
    }
  }

  for (size_t i = 0; i < cloud->points.size (); i++)
  {
    float x = cloud->points[i].x;
    float y = cloud->points[i].y;
    float z = cloud->points[i].z;
    int u = static_cast<int> (f_ * x / z + cx);
    int v = static_cast<int> (f_ * y / z + cy);

    if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
      continue;

    if ((z < depth_[u * cy_ + v]) || (!pcl_isfinite(depth_[u * cy_ + v])))
    {
      depth_[u * cx_ + v] = z;
      range_image->at(u,v).x = x;
      range_image->at(u,v).y = y;
      range_image->at(u,v).z = z;
    }
  }

  delete[] depth_;

  /*if (smooth)
  {
    //Dilate and smooth the depth map
    int ws = wsize;
    int ws2 = int (std::floor (static_cast<float> (ws) / 2.f));
    float * depth_smooth = new float[cx_ * cy_];
    for (int i = 0; i < (cx_ * cy_); i++)
      depth_smooth[i] = std::numeric_limits<float>::quiet_NaN ();

    for (int u = ws2; u < (cx_ - ws2); u++)
    {
      for (int v = ws2; v < (cy_ - ws2); v++)
      {
        float min = std::numeric_limits<float>::max ();
        for (int j = (u - ws2); j <= (u + ws2); j++)
        {
          for (int i = (v - ws2); i <= (v + ws2); i++)
          {
            if (pcl_isfinite(depth_[j * cx_ + i]) && (depth_[j * cx_ + i] < min))
            {
              min = depth_[j * cx_ + i];
            }
          }
        }

        if (min < (std::numeric_limits<float>::max () - 0.1))
        {
          depth_smooth[u * cx_ + v] = min;
        }
      }
    }

    memcpy (depth_, depth_smooth, sizeof(float) * cx_ * cy_);
    delete[] depth_smooth;
  }*/
}

template<typename PointT>
float faat_pcl::registration::VisibilityReasoning<PointT>::computeOSV(const typename pcl::PointCloud<PointT>::ConstPtr & im1,
                                                                      const typename pcl::PointCloud<PointT>::ConstPtr & im2,
                                                                      Eigen::Matrix4f pose_2_to_1)
{
  //check for OSV violation, a surface is observed by sensor2
  //and not observed by sensor1, even though it is in the FOV of sensor1
  PointCloudPtr im2_trans;
//  if (pose_2_to_1.isIdentity (0.00001f))
//  {
//    im2_trans = im2;
//  }
//  else
//  {
    im2_trans.reset (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud (*im2, *im2_trans, pose_2_to_1);
//  }

  float cx, cy;
  cx = static_cast<float> (cx_) / 2.f - 0.5f;
  cy = static_cast<float> (cy_) / 2.f - 0.5f;

  int osv = 0;
  for (size_t i = 0; i < im2_trans->points.size (); i++)
  {
    float x = im2_trans->points[i].x;
    float y = im2_trans->points[i].y;
    float z = im2_trans->points[i].z;
    int u = static_cast<int> (focal_length_ * x / z + cx);
    int v = static_cast<int> (focal_length_ * y / z + cy);

    if (u >= cx_ || v >= cy_ || u < 0 || v < 0)
      continue;

    //TODO: Probably not only if its finite but also if behind by some threhsold
    //although this will be covered by FSV (INVERSE)
    if (!pcl_isfinite(im1->at(u,v).z))
    {
      //point not observed in sensor1 but observed in sensor2
      osv++;
    }
  }

  //std::cout << osv / static_cast<float>(im2_trans->points.size ()) << std::endl;
  return (osv / static_cast<float>(im2_trans->points.size ()));
}

template<typename PointT>
  float
  faat_pcl::registration::VisibilityReasoning<PointT>::computeFSV (const typename pcl::PointCloud<PointT>::ConstPtr & im1,
                                                                   const typename pcl::PointCloud<PointT>::ConstPtr & im2,
                                                                   Eigen::Matrix4f pose_2_to_1)
  {
    std::vector<float> range_diff_1_to_2;

    if (pose_2_to_1.isIdentity (0.00001f))
    {
      fsv_used_ = computeRangeDifferencesWhereObserved (im1, im2, range_diff_1_to_2);
    }
    else
    {
      PointCloudPtr im2_trans (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud (*im2, *im2_trans, pose_2_to_1);
      fsv_used_ = computeRangeDifferencesWhereObserved (im1, im2_trans, range_diff_1_to_2);
    }

    float tss, fsv_val;
    int xss, xfsv;
    tss = tss_;
    xss = 0;
    xfsv = 0;
    for (size_t i = 0; i < range_diff_1_to_2.size (); i++)
    {
      if (std::abs (range_diff_1_to_2[i]) <= tss)
      {
        xss++;
      }

      if (range_diff_1_to_2[i] > tss)
      {
        xfsv++;
      }
    }

    fsv_used_ = xss;

    fsv_val = xfsv / (static_cast<float> (xfsv + xss));
    return fsv_val;
  }

  template<typename PointT>
    float
    faat_pcl::registration::VisibilityReasoning<PointT>::computeFSVWithNormals (const typename pcl::PointCloud<PointT>::ConstPtr & im1,
                                                                                const typename pcl::PointCloud<PointT>::ConstPtr & im2,
                                                                                pcl::PointCloud<pcl::Normal>::Ptr & normals)
    {
      std::vector<float> range_diff_1_to_2;
      std::vector<int> im2_indices_observed;
      fsv_used_ = computeRangeDifferencesWhereObservedWithIndicesBack (im1, im2, range_diff_1_to_2, im2_indices_observed);

      float tss, fsv_val;
      float xss, xfsv;
      tss = tss_;
      xss = 0;
      xfsv = 0;
      for (size_t i = 0; i < range_diff_1_to_2.size (); i++)
      {
        if (std::abs (range_diff_1_to_2[i]) <= tss)
        {
          xss++;
        }

        if (range_diff_1_to_2[i] > tss)
        {
            Eigen::Vector3f normal_p = normals->points[im2_indices_observed[i]].getNormalVector3fMap();
            Eigen::Vector3f normal_vp = Eigen::Vector3f::UnitZ() * -1.f;
            normal_p.normalize ();
            normal_vp.normalize ();

            float dot = normal_vp.dot(normal_p);
            float angle = pcl::rad2deg(acos(dot));
            if (angle < 60.f)
            {
                xfsv++;
            }
        }
      }

      fsv_used_ = xss;

      fsv_val = xfsv / (static_cast<float> (xfsv + xss));
      return fsv_val;
    }

#endif /* VISIBILITY_REASONING_HPP_ */
