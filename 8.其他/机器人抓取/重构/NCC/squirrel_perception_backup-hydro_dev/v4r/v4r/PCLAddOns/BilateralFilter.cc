/**
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

/**
 * $Id$
 * Thomas Mörwald
 * Johann Prankl, 6.12.2011
 */

#include "BilateralFilter.hh"

namespace pclA {

using namespace std;

float BilateralFilter::NaN = std::numeric_limits<float>::quiet_NaN();

/********************** BilateralFilter ************************
 * Constructor/Destructor
 */
BilateralFilter::BilateralFilter(Parameter p) : param(p)
{
}

BilateralFilter::~BilateralFilter()
{
}

/************************** PRIVATE ************************/

/**
 * FilterCloud
 */
void BilateralFilter::FilterCloud(const pcl::PointCloud<pcl::PointXYZRGB> &in,
    pcl::PointCloud<pcl::PointXYZRGB> &out)
{
#pragma omp parallel for
  for (int v = 0; v < height; v++) {
    float k, tmp, h[3];
    const float *pt_i;

    for (int u = 0; u < width; u++) {
      const pcl::PointXYZRGB &pt = in(u, v);
      pcl::PointXYZRGB &ptOut = out(u, v);

      if (pt.x == pt.x) {
        k = h[0] = h[1] = h[2] = 0;

        for (int y = v - param.radius; y <= v + param.radius; y++) {
          for (int x = u - param.radius; x <= u + param.radius; x++) {
            if (x < 0 || x >= width || y < 0 || y >= height) {
              pt_i = &pt.x;
            } else {
              pt_i = &in(x, y).x;

              if (pt_i[0] != pt_i[0]) {
                pt_i = &pt.x;
              }
            }

            //            PVec::Sub3(pt_i, &pt.x, df);
            Eigen::Vector3f df = Eigen::Vector3f(pt_i[0] - pt.x, pt_i[1] - pt.y, pt_i[2] - pt.z);

            tmp = DistC(x - u, y - v) * DistS(df);

            k += tmp;
            h[0] += (pt_i[0] * tmp);
            h[1] += (pt_i[1] * tmp);
            h[2] += (pt_i[2] * tmp);
          }
        }

        if (k > std::numeric_limits<float>::min()) {
          k = (1.0 / k);

          ptOut.x = h[0] * k;
          ptOut.y = h[1] * k;
          ptOut.z = h[2] * k;
          ptOut.rgb = pt.rgb;
        } else {
          ptOut = pt;
        }

      } else
        ptOut = pt;
    }
  }
}

/**
 * FilterCloud
 */
void BilateralFilter::FilterCloud(const pcl::PointCloud<pcl::PointXYZRGB> &in,
    pcl::PointCloud<pcl::PointXYZRGB> &out, const std::vector<int> &indices)
{
  Eigen::Vector3f h, df;
  float k, tmp;
  short u, v;
  int idx;

#pragma omp parallel for private(k, tmp, h, df, u, v)
  for (unsigned i = 0; i < indices.size(); i++) {
    const float *pt_i;

    {
      idx = indices[i];
      const pcl::PointXYZRGB &pt = in.points[idx];
      pcl::PointXYZRGB &ptOut = out.points[idx];

      if (pt.x == pt.x) {
        k = h[0] = h[1] = h[2] = 0;
        u = X(idx);
        v = Y(idx);

        for (int y = v - param.radius; y <= v + param.radius; y++) {
          for (int x = u - param.radius; x <= u + param.radius; x++) {
            if (x < 0 || x >= width || y < 0 || y >= height) {
              pt_i = &in(u, v).x;
            } else {
              pt_i = &in(x, y).x;

              if (pt_i[0] != pt_i[0]) {
                pt_i = &in(u, v).x;
              }
            }

            Eigen::Vector3f df = Eigen::Vector3f(pt_i[0] - pt.x, pt_i[1] - pt.y, pt_i[2] - pt.z);
            tmp = DistC(x - u, y - v) * DistS(&df[0]);

            k += tmp;
            h[0] += (pt_i[0] * tmp);
            h[1] += (pt_i[1] * tmp);
            h[2] += (pt_i[2] * tmp);
          }
        }

        if (k > std::numeric_limits<float>::min()) {
          k = (1.0 / k);

          ptOut.x = h[0] * k;
          ptOut.y = h[1] * k;
          ptOut.z = h[2] * k;
          ptOut.rgb = pt.rgb;
        } else {
          ptOut = pt;
        }

      } else
        ptOut = pt;
    }
  }
}

/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void BilateralFilter::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error("[BilateralFilter::setInputCloud] Point cloud needs to be organized!");

  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;
}

/**
 * resize the point cloud
 */
void BilateralFilter::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error("[BilateralFilter::compute] No point cloud available!");

  if (outCloud.get() == 0)
    outCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  outCloud->header = cloud->header;
  outCloud->width = cloud->width;
  outCloud->height = cloud->height;
  outCloud->is_dense = cloud->is_dense;
  outCloud->resize(outCloud->width * outCloud->height);

  invSqrSigmaD = 1. / (param.dSigma * param.dSigma);
  invSqrSigmaR = 1. / (param.rSigma * param.rSigma);

  FilterCloud(*cloud, *outCloud);
}

/**
 * compute
 */
void BilateralFilter::compute(const std::vector<int> &indices)
{
  if (cloud.get() == 0)
    throw std::runtime_error("[BilateralFilter::compute] No point cloud available!");

  if (outCloud.get() == 0)
    outCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  outCloud->header = cloud->header;
  outCloud->width = cloud->width;
  outCloud->height = cloud->height;
  outCloud->is_dense = cloud->is_dense;
  outCloud->points.clear();
  outCloud->points.resize(outCloud->width * outCloud->height, pcl::PointXYZRGB(NaN, NaN, NaN));

  invSqrSigmaD = 1. / (param.dSigma * param.dSigma);
  invSqrSigmaR = 1. / (param.rSigma * param.rSigma);

  FilterCloud(*cloud, *outCloud, indices);
}

/**
 * get resized cloud
 */
void BilateralFilter::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  _cloud = outCloud;
}

/**
 * setParameter
 */
void BilateralFilter::setParameter(Parameter p)
{
  param = p;
}

} //-- THE END --

