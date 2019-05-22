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
 * Johann Prankl, 17.5.2013
 */

#include "HoleFillingBF.hh"
#include <pcl/common/time.h>

namespace pclA {

using namespace std;

float HoleFillingBF::NaN = std::numeric_limits<float>::quiet_NaN();

/********************** HoleFillingBF ************************
 * Constructor/Destructor
 */
HoleFillingBF::HoleFillingBF(const Parameter &p) 
 : param(p)
{
}

HoleFillingBF::~HoleFillingBF()
{
}

/************************** PRIVATE ************************/

/**
 * FilterCloud
 */
int HoleFillingBF::fillHoles(const pcl::PointCloud<pcl::PointXYZRGB> &in,
    pcl::PointCloud<pcl::PointXYZRGB> &out)
{
  int cnt_nan=0;
  double k, tmp, h;
  Eigen::Vector3i dcol;
  double inv_fx = 1. / param.fx;
  double inv_fy = 1. / param.fy;

  #pragma omp parallel for  private(k, tmp, h, dcol)
  for (int v = 0; v < height; v++) 
  {
    for (int u = 0; u < width; u++) 
    {
      const pcl::PointXYZRGB &pt_in = in(u, v);
      if (isnan(pt_in.x))
      {
        h = k = 0;
        pcl::PointXYZRGB &pt_out = out(u, v);

        for (int y = v - param.radius; y <= v + param.radius; y++) 
        {
          if (y<0 || y >= height)
            continue;

          for (int x = u - param.radius; x <= u + param.radius; x++) 
          {
            if (x < 0 || x >= width)
              continue;

            const pcl::PointXYZRGB &pt = in(x, y);

            if (isnan(pt.x))
              continue;

            dcol = Eigen::Vector3i(pt.r-pt_in.r, pt.g-pt_in.g, pt.b-pt_in.b);

            tmp = DistC(x - u, y - v) * DistCol(dcol);

            k += tmp;
            h += pt.z*tmp;
          }
        }

        if (k > std::numeric_limits<float>::min()) 
        {
          k = (1.0 / k);
          pt_out.z = h*k;
          pt_out.x = static_cast<float> (u-param.cx) * pt_out.z * inv_fx; 
          pt_out.y = static_cast<float> (v-param.cy) * pt_out.z * inv_fy;  
        } 
        else
        {
          #pragma omp critical 
          cnt_nan++;
        }
      }
    }
  }

  return cnt_nan;
}


/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void HoleFillingBF::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error("[HoleFillingBF::setInputCloud] Point cloud needs to be organized!");

  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;
}

/**
 * resize the point cloud
 */
void HoleFillingBF::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error("[HoleFillingBF::compute] No point cloud available!");

  //pcl::ScopeTime t("HoleFillingBF::compute");

  out_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  if (tmp_cloud.get()==0) tmp_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

  *out_cloud = *cloud;
  *tmp_cloud = *cloud;

  negInvSqrSigmaR = -1. / (2. * param.rSigma * param.rSigma);
  negInvSqrSigmaD = -1. / (2. * param.dSigma * param.dSigma);
  negInvSqrSigmaCol = -1. / (2. * param.colSigma * param.colSigma);

  int z=0, num_nan=INT_MAX;
  while(num_nan>0 && z<20) 
  {
    {
    //pcl::ScopeTime t("HoleFillingBF::compute-fill");
    num_nan = fillHoles(*tmp_cloud, *out_cloud);
    }

    if (num_nan>0)
    {
      pcl::PointCloud<pcl::PointXYZRGB> &ref_t = *tmp_cloud;
      pcl::PointCloud<pcl::PointXYZRGB> &ref_o = *out_cloud;

      for (unsigned i=0; i<ref_t.points.size(); i++)
        if (isnan(ref_t.points[i].x) && !isnan(ref_o.points[i].x))
          ref_t.points[i].getVector3fMap() = ref_o.points[i].getVector3fMap();
    }
    z++;
    //cout<<"("<<z<<","<<num_nan<<") ";
  };
}

/**
 * get resized cloud
 */
void HoleFillingBF::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  _cloud = out_cloud;
}

/**
 * setParameter
 */
void HoleFillingBF::setParameter(const Parameter &p)
{
  param = p;
}

} //-- THE END --

