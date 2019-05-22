/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_POINT_TYPES_HPP
#define KP_POINT_TYPES_HPP

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <stdint.h>

namespace kp 
{

const float NaNf = std::numeric_limits<float>::quiet_NaN();

/**
 * PointXYZRGB
 */
class PointXYZRGB
{
public:
  Eigen::Vector4f pt;
  union
  {
    struct
    {
      uint8_t b;
      uint8_t g;
      uint8_t r;
      uint8_t a;
    };
    float rgb;
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXYZRGB() : pt(Eigen::Vector4f(NaNf,NaNf,NaNf,1.)) {}

  inline Eigen::Map<Eigen::Vector3f> getVector3fMap() { 
    return Eigen::Vector3f::Map(&pt[0]); 
  }
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap() const { 
    return Eigen::Vector3f::Map(&pt[0]); 
  }
  inline Eigen::Vector4f &getVector4fMap() {
    return pt; 
  }
  inline const Eigen::Vector4f &getVector4fMap() const { 
    return pt;
  }

  inline float& operator[](int i) { return pt[i]; }
  inline const float& operator[](int i) const { return pt[i]; }
};

/**
 * PointXYZNormalRGB
 */
class PointXYZNormalRGB
{
public:
  Eigen::Vector4f pt;
  Eigen::Vector4f n;
  union
  {
    struct
    {
      uint8_t b;
      uint8_t g;
      uint8_t r;
      uint8_t a;
    };
    float rgb;
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PointXYZNormalRGB() : pt(Eigen::Vector4f(NaNf,NaNf,NaNf,1.)), n(Eigen::Vector4f(NaNf,NaNf,NaNf,1.)) {}

  inline Eigen::Map<Eigen::Vector3f> getVector3fMap() { 
    return Eigen::Vector3f::Map(&pt[0]); 
  }
  inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap() const { 
    return Eigen::Vector3f::Map(&pt[0]); 
  }
  inline Eigen::Vector4f &getVector4fMap() {
    return pt; 
  }
  inline const Eigen::Vector4f &getVector4fMap() const { 
    return pt;
  }
  inline Eigen::Map<Eigen::Vector3f> getNormalVector3fMap() { 
    return Eigen::Vector3f::Map(&n[0]); 
  }
  inline const Eigen::Map<const Eigen::Vector3f> getNormalVector3fMap() const { 
    return Eigen::Vector3f::Map(&n[0]); 
  }
  inline Eigen::Vector4f &getNormalVector4fMap() {
    return n; 
  }
  inline const Eigen::Vector4f &getNormalVector4fMap() const { 
    return n;
  }

  inline float& operator[](int i) { return pt[i]; }
  inline const float& operator[](int i) const { return pt[i]; }
};


} //--END--

#endif

