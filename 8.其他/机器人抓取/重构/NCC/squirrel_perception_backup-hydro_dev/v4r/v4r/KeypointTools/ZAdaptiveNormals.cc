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


#include "ZAdaptiveNormals.hh"
#include "eigen.h"

namespace kp 
{

// using namespace std;

float ZAdaptiveNormals::NaN  = std::numeric_limits<float>::quiet_NaN(); 


/********************** ZAdaptiveNormals ************************
 * Constructor/Destructor
 */
ZAdaptiveNormals::ZAdaptiveNormals(const Parameter &p)
{
  setParameter(p);
  param.kernel_radius[0] = 3;
  param.kernel_radius[1] = 3;
  param.kernel_radius[2] = 3;
  param.kernel_radius[3] = 3;
  param.kernel_radius[4] = 4;
  param.kernel_radius[5] = 5;
  param.kernel_radius[6] = 6;
  param.kernel_radius[7] = 7;
}

ZAdaptiveNormals::~ZAdaptiveNormals()
{
}


/************************** PRIVATE ************************/


/**
 * ComputeCovarianceMatrix
 */
void ZAdaptiveNormals::computeCovarianceMatrix (const kp::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &indices, const Eigen::Vector3f &mean, Eigen::Matrix3f &cov)
{
  Eigen::Vector3f pt;
  cov.setZero ();

  for (unsigned i = 0; i < indices.size (); ++i)
  {
    pt = cloud.data[indices[i]] - mean;

    cov(1,1) += pt[1] * pt[1];
    cov(1,2) += pt[1] * pt[2];
    cov(2,2) += pt[2] * pt[2];

    pt *= pt[0];
    cov(0,0) += pt[0];
    cov(0,1) += pt[1];
    cov(0,2) += pt[2];
  }

  cov(1,0) = cov(0,1);
  cov(2,0) = cov(0,2);
  cov(2,1) = cov(1,2);
}

/**
 * GetIndices
 */
void ZAdaptiveNormals::getIndices(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, int u, int v, int kernel, std::vector<int> &indices)
{
  int idx;
  const Eigen::Vector3f &pt = cloud.data[getIdx(u,v)]; 

  for (int vkernel=0-kernel; vkernel<=kernel; vkernel++) {
    for (int ukernel=0-kernel; ukernel<=kernel; ukernel++) {
      int y = v + vkernel;
      int x = u + ukernel;
      
      float center_dist = sqrt(vkernel*vkernel + ukernel*ukernel);
      if (x>0 && y>0 && x<width && y<height) {
        idx = getIdx(x,y);
        const Eigen::Vector3f &pt1 = cloud.data[idx];
        if(!isnan(pt1[2])) {
          float new_sqr_radius = sqr_radius;
          if(param.adaptive) {
            float val = param.kappa * center_dist * pt1[2] + param.d;
            new_sqr_radius = val*val;
          }
        
          if ((pt-pt1).squaredNorm() < new_sqr_radius)
            indices.push_back(idx);
        }
      }
    }
  }
}

/**
 * ComputeNormal
 */
float ZAdaptiveNormals::computeNormal(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, std::vector<int> &indices, Eigen::Matrix3f &eigen_vectors)
{
  if (indices.size()<4)
    return NaN;

  Eigen::Vector3f mean;
  mean.setZero();
  for (unsigned j=0; j<indices.size(); j++)
    mean += cloud.data[indices[j]];
  mean /= (float)indices.size();

  Eigen::Matrix3f cov;
  computeCovarianceMatrix (cloud, indices, mean, cov);

  Eigen::Vector3f eigen_values;
  kp::eigen33 (cov, eigen_vectors, eigen_values);
  float eigsum = eigen_values.sum();
  if (eigsum != 0)
    return fabs (eigen_values[0] / eigsum );
  
  return NaN;
}


/**
 * EstimateNormals
 */
void ZAdaptiveNormals::estimateNormals(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, kp::DataMatrix2D<Eigen::Vector3f> &normals)
{
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  std::vector< int > indices;

  #pragma omp parallel for private(eigen_vectors,indices)
  for (int v=0; v<height; v++) {
    for (int u=0; u<width; u++) {
      indices.clear();
      int idx = getIdx(u,v);
      const Eigen::Vector3f &pt = cloud.data[idx];
      Eigen::Vector3f &n = normals.data[idx];
      if(!isnan(pt[0]) && !isnan(pt[1]) && !isnan(pt[2])) {      
        if(param.adaptive) {
          int dist = (int) (pt[2]*2); // *2 => every 0.5 meter another kernel radius
          getIndices(cloud, u,v, param.kernel_radius[dist], indices);
        }
        else
          getIndices(cloud, u,v, param.kernel, indices);
      }

      if (indices.size()<4) {
        n[0] = NaN;
        continue;
      }

      /* curvature= */ computeNormal(cloud, indices, eigen_vectors);

      n[0] = eigen_vectors (0,0);
      n[1] = eigen_vectors (1,0);
      n[2] = eigen_vectors (2,0);


      if (n.dot(pt) > 0) {
        n *= -1;
        //n.getNormalVector4fMap()[3] = 0;
        //n.getNormalVector4fMap()[3] = -1 * n.getNormalVector4fMap().dot(pt.getVector4fMap());
      }
    }
  }
}

/**
 * estimateNormals
 */
void ZAdaptiveNormals::estimateNormals(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &normals_indices, std::vector<Eigen::Vector3f> &normals)
{
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  std::vector< int > indices;

  #pragma omp parallel for private(eigen_vectors,indices)
  for (unsigned i=0; i<normals_indices.size(); i++) {
      indices.clear();
      int idx = normals_indices[i];
      int u = idx%width;
      int v = idx/width;
      const Eigen::Vector3f &pt = cloud.data[idx];
      Eigen::Vector3f &n = normals[i];
      if(!isnan(pt[0]) && !isnan(pt[1]) && !isnan(pt[2])) {      
        if(param.adaptive) {
          int dist = (int) (pt[2]*2); // *2 => every 0.5 meter another kernel radius
          getIndices(cloud, u,v, param.kernel_radius[dist], indices);
        }
        else
          getIndices(cloud, u,v, param.kernel, indices);
      }

      if (indices.size()<4) {
        n[0] = NaN;
        continue;
      }

      /* curvature= */ computeNormal(cloud, indices, eigen_vectors);

      n[0] = eigen_vectors (0,0);
      n[1] = eigen_vectors (1,0);
      n[2] = eigen_vectors (2,0);


      if (n.dot(pt) > 0) {
        n *= -1;
        //n.getNormalVector4fMap()[3] = 0;
        //n.getNormalVector4fMap()[3] = -1 * n.getNormalVector4fMap().dot(pt.getVector4fMap());
      }
  }
}



/************************** PUBLIC *************************/

/**
 * compute the normals
 */
void ZAdaptiveNormals::compute(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, kp::DataMatrix2D<Eigen::Vector3f> &normals)
{
  width = cloud.cols;
  height = cloud.rows;

  normals.resize(height, width);

  estimateNormals(cloud, normals);
}

/**
 * compute the normals using a mask
 */
void ZAdaptiveNormals::compute(const kp::DataMatrix2D<Eigen::Vector3f> &cloud, const std::vector<int> &indices, std::vector<Eigen::Vector3f> &normals)
{
  width = cloud.cols;
  height = cloud.rows;

  normals.resize(indices.size());

  estimateNormals(cloud, indices, normals);
}


/**
 * setParameter
 */
void ZAdaptiveNormals::setParameter(const Parameter &p)
{
  param = p;
  sqr_radius = p.radius*p.radius;
}


} //-- THE END --

