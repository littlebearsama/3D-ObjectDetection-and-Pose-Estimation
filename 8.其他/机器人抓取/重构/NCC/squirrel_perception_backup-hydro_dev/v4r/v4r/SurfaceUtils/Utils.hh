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

#ifndef SURFACE_UTILS_HH
#define SURFACE_UTILS_HH

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>

#include "SurfaceModel.hpp"

namespace surface
{

  /**
   * @brief RGBValue of point clouds, accessable as float or long value.
   */
  typedef union
  {
    struct
    {
      unsigned char b; // Blue channel
      unsigned char g; // Green channel
      unsigned char r; // Red channel
      unsigned char a; // Alpha channel
    };
    float float_value;
    long long_value;
  } RGBValue;

  /**
   * some utils for surface modeling
   */
  class Utils
  {
  private:
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Utils ();
    ~Utils ();

    static void
    ConvertCloud2Image (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, cv::Mat_<cv::Vec3b> &image);

    static bool
    convertSurfaces2Brep (const std::vector<surface::SurfaceModel::Ptr> &surfaces, ON_Brep &brep);

    static bool
    convertSurfaces2Brep (const std::vector<surface::SurfaceModel::Ptr> &surfaces,
                          const pcl::PointCloud<pcl::PointXYZRGB> &cloud, ON_Brep &brep);

    static bool
    convertBrep2Surfaces (const ON_Brep &brep, std::vector<surface::SurfaceModel::Ptr> &surfaces);

    static bool
    convertBrep2Surfaces (const ON_Brep &brep, std::vector<surface::SurfaceModel::Ptr> &surfaces,
                          pcl::PointCloud<pcl::PointXYZRGB> &cloud);
  };

  /*********************** INLINE METHODES **************************/

  inline float
  GetRandomColor ()
  {
    RGBValue x;
    x.b = std::rand () % 255;
    x.g = std::rand () % 255;
    x.r = std::rand () % 255;
    x.a = 0.;
    return x.float_value;
  }

}

#endif

