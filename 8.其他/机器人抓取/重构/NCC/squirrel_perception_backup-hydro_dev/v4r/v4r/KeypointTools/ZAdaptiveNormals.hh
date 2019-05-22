/**
 * $Id$
 *
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

#ifndef KP_ZADAPTIVE_NORMALS_HH
#define KP_ZADAPTIVE_NORMALS_HH

#include <iostream>
#include <stdexcept>
#include <omp.h>
#include <math.h>
#include <Eigen/Dense>
#include "DataMatrix2D.hpp"
#include "SmartPtr.hpp"


namespace kp 
{
/**
 * Surface normals estimation
 */
class ZAdaptiveNormals
{
public:
  class Parameter
  {
    public:
      double radius;            // euclidean inlier radius
      int kernel;               // kernel radius [px]
      bool adaptive;            // Activate z-adaptive normals calcualation
      float kappa;              // gradient
      float d;                  // constant
      float kernel_radius[8];   // Kernel radius for each 0.5 meter intervall (0-4m)
      Parameter(double _radius=0.02, int _kernel=5, bool _adaptive=false, float _kappa=0.005125, float _d = 0.0)
       : radius(_radius), kernel(_kernel), adaptive(_adaptive), kappa(_kappa), d(_d) {}
  };

private:
  Parameter param;

  static float NaN;
  int width, height;

  float sqr_radius;

  void computeCovarianceMatrix (const kp::DataMatrix2D<Eigen::Vector3f> &cloud,
        const std::vector<int> &indices, const Eigen::Vector3f &mean, Eigen::Matrix3f &cov);
  void estimateNormals(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, 
        kp::DataMatrix2D<Eigen::Vector3f> &normals);
  void getIndices(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, int u, int v, int kernel, 
        std::vector<int> &indices);
  float computeNormal(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, std::vector<int> &indices,
        Eigen::Matrix3f &eigen_vectors);
  void estimateNormals(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, 
        const std::vector<int> &normals_indices, std::vector<Eigen::Vector3f> &normals);


  inline int getIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);



public:
  ZAdaptiveNormals(const Parameter &p = Parameter());
  ~ZAdaptiveNormals();

  void setParameter(const Parameter &p);

  void compute(const kp::DataMatrix2D<Eigen::Vector3f> &cloud,
          kp::DataMatrix2D<Eigen::Vector3f> &normals);

  void compute(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &indices,
          std::vector<Eigen::Vector3f> &normals);

  typedef SmartPtr< ::kp::ZAdaptiveNormals> Ptr;
  typedef SmartPtr< ::kp::ZAdaptiveNormals const> ConstPtr;
};




/*********************** INLINE METHODES **************************/

inline int ZAdaptiveNormals::getIdx(short x, short y)
{
  return y*width+x;
}

inline short ZAdaptiveNormals::X(int idx)
{
  return idx%width;
}

inline short ZAdaptiveNormals::Y(int idx)
{
  return idx/width;
}


}

#endif

