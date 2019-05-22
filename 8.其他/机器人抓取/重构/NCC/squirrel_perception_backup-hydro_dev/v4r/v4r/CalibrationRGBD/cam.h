/*////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2013, Jonathan Balzer
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

#ifndef _V4R_CALIBRATION_RGB_CAM_H_
#define _V4R_CALIBRATION_RGB_CAM_H_

#include <stdlib.h>
#include <vector>

#include <opencv2/opencv.hpp>

#define Z_MIN_MM 300

class CDepthColorSensor;
class Params;

/*! \brief camera model
 *
 *
 *
 */
class CCam {

    friend class CDepthColorCam;
    friend class CDepthColorSensor;
    friend class Params;

public:

  //! Constructor.
  CCam();

    //! Assignment operator.
    CCam(const CCam& cam);

    //! Parametrized constructor.
    CCam(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F);

  /*! \brief Projects a point into the image plane.
   *
   * \param[in] x point in world coordinates
   * \returns point in pixel coordinates
   *
   */
    cv::Vec2f Project(const cv::Vec3f& x) const;

    //! Transforms a point into the local coordinate system.
    cv::Vec3f TransformTo(const cv::Vec3f& x) const;

    /*! \brief Projects a point into the image plane without using the extrinsics and distortion model.
     *
     * \param[in] xc point in camera coordinates
     * \returns point in pixel coordinates
     *
     */
    cv::Vec2f ProjectPinhole(const cv::Vec3f& xc) const;

  /*! \brief Projects a point into the image plane without using the extrinsics but with distortion model.
   *
   * \param[in] xc point in camera coordinates
   * \returns point in pixel coordinates
   *
   */
    cv::Vec2f ProjectLocal(const cv::Vec3f& xc) const;

  /*! \brief Converts a pixel into a viewing direction (in world coordinates).
   *
   * \param[in] u location w.r.t. the pixel coordinate system
   * \returns direction vector, normalized s.t. z-component equals 1
   *
   */
    cv::Vec3f UnProject(const cv::Vec2i& u) const;

  /*! \brief Converts a pixel into a viewing direction (in camera coordinates).
   *
   * \param[in] u location w.r.t. the pixel coordinate system
   * \returns direction vector, normalized s.t. z-component equals 1
   *
   */
    cv::Vec3f UnProjectLocal(const cv::Vec2i& u) const;

    /*! \brief Undistorts the image.
     *
     * \param[in] src original image
     * \returns undistorted image w.r.t. dist-coefficients k[0-4]
     *
     */
    cv::Mat GetUndistorted(const cv::Mat& src) const;

  //! Writes the camera parameters to a stream.
  friend std::ostream& operator << (std::ostream& os, const CCam& x);

    //! Reads the camera parameters from a stream.
    friend std::istream& operator >> (std::istream& is, CCam& x);

    //! Access to trafo.
    cv::Mat& GetExtrinsics() { return m_F; }

    void GetIntrinsics(size_t size[2], float f[2], float c[2], float& alpha, float k[5]) const;

protected:

    size_t m_size[2];			//!< pixel size
    float m_f[2];				//!< focal length
    float m_c[2];				//!< principle point
    float m_alpha;				//!< skew coefficient
    float m_k[5];				//!< distortion coefficients
    cv::Mat m_F;                //!< frame world-to-cam
    cv::Mat m_Finv;				//!< inverse cam-to-world frame

};

class CDepthCam:public CCam {

    friend class CDepthColorCam;
    friend class CDepthColorSensor;
    friend class Params;

public:

    //! Standard constructor.
    CDepthCam();

    //! Parametrized constructor.
    CDepthCam(const std::vector<size_t>& size, const std::vector<float>& f, const std::vector<float>& c, const float& alpha, const std::vector<float>& k, const cv::Mat& F, const std::vector<float>& range, const std::vector<float>& d, const cv::Mat& D, const std::vector<float>& a);

    //! Converts the disparity from the depth sensor into a metric depth.
    float DisparityToDepth(size_t i, size_t j, float d) const;

    //! Writes the camera parameters to a stream.
    friend std::ostream& operator << (std::ostream& os, const CDepthCam& x);

    //! Reads the camera parameters from a stream.
    friend std::istream& operator >> (std::istream& is, CDepthCam& x);

    void GetRange(float range[2]) const;

private:

    float m_range[2];           //! depth range
    float m_d[2];               //! disparity inversion parameters
    cv::Mat m_D;                //! spatial distortion pattern
    float m_a[2];               //! distance weights of distortion pattern

};

#endif /* CAM_H_ */
