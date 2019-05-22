/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer, Thomas Mörwald
//
// All rights reserved.
//
// This file is part of the R4R library.
//
// The R4R library is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// The R4R library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with the R4R library. If not, see <http://www.gnu.org/licenses/>.
//
////////////////////////////////////////////////////////////////////////////////*/

#ifndef _V4R_CALIBRATION_DEPTH_COLOR_CAM_H_
#define _V4R_CALIBRATION_DEPTH_COLOR_CAM_H_

#include "cam.h"

class CDepthColorCam
{
public:
  CDepthColorCam();
  CDepthColorCam(CCam rgb, CDepthCam depth);

  //! Sets parameters of the RGB sensor.
  void ConfigureRGB(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F);

  //! Sets parameters of the depth sensor.
  void ConfigureDepth(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& range, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a);

  //! Converts disparity into 3d point in depth cam coordinates.
  cv::Point3f GetPoint(const size_t& i, const size_t& j, const float &d) const;

  //! Converts disparity into 3d point in depth cam coordinates.
  cv::Vec3f GetNormal(size_t i, size_t j, const cv::Mat& disp) const;

  //! Gets the color at a point in world (depth) coordinates.
  cv::Vec3b GetColor(cv::Point3f x, const cv::Mat& rgb) const;

  //! Warps an RGB image to the image plane of the depth sensor.
  cv::Mat WarpRGBToDepth(const cv::Mat& disp, const cv::Mat& rgb);

  /*! \brief Warps a depth image to the image plane of the RGB sensor.
   *
   * \details This is a HACK. Needs to be done with spline interpolation.
   */
  cv::Mat WarpDepthToRGB(const cv::Mat& disp, const cv::Mat& rgb);

  /*! \brief Warps a point image to the image plane of the RGB sensor.
   *
   * \details This is a HACK. Needs to be done with spline interpolation.
   */
  cv::Mat WarpDisparityToRGBPointsUndistorted(const cv::Mat& disp, const cv::Mat& rgb) const;

  void DisparityToPoints(const cv::Mat& disp, cv::Mat& result);

  //! Access to maximum disparity
  float DisparityToDepth(int d);

  //! Gets disparity range.
  void GetDisparityRange(size_t& min, size_t& max);

  //! Access to camera.
  CCam& GetRGBCam() { return m_rgb_cam; }

  //! Access to depth camera.
  CDepthCam& GetDepthCam() { return m_depth_cam; }

  //! Write sensor configuration to a stream.
  friend std::ostream& operator << (std::ostream& os, const CDepthColorCam& x);

  //! Reads sensor configuration from a stream.
  friend std::istream& operator >> (std::istream& is, CDepthColorCam& x);

  void set_default_params_asus_pro_live();
  void set_default_params_primesense();

public:

  CCam m_rgb_cam;
  CDepthCam m_depth_cam;

};

#endif
