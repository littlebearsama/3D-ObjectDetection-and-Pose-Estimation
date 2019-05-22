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
 * Johann Prankl, 27.11.2011
 */


#include "SubsamplePointCloud.hh"


namespace pclA 
{

using namespace std;

float SubsamplePointCloud::NaN  = std::numeric_limits<float>::quiet_NaN(); 

/********************** SubsamplePointCloud ************************
 * Constructor/Destructor
 */
SubsamplePointCloud::SubsamplePointCloud(Parameter p)
{
  setParameter(p);
}

SubsamplePointCloud::~SubsamplePointCloud()
{
}


/************************** PRIVATE ************************/


/**
 * SubsamplePointCloud
 */
void SubsamplePointCloud::SubsampleMean(pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out)
{
  #pragma omp parallel for
  for (int v=0; v<(int)out.height; v++)
  {
    Eigen::Vector4f mean;
    int uIn, vIn, cnt;

    for (int u=0; u<(int)out.width; u++)
    {
      cnt=0;
      mean.setZero();
      uIn = u*factor;
      vIn = v*factor;
      pcl::PointXYZRGB &pt0 = in(uIn, vIn);

      if (!IsNaN(pt0))
      {
        for (int y = (vIn)-param.radius; y<=(vIn)+param.radius; y++)
        {
          for (int x = (uIn)-param.radius; x<=(uIn)+param.radius; x++)
          {
            if (x>=0 && x<width && y>=0 && y<height)
            {
              pcl::PointXYZRGB &pt = in(x,y);
              if (!IsNaN(pt) && SqrDistance(pt0,pt) < sqrDist)
              {
                mean += pt.getVector4fMap();
                cnt++;
              }
            }
          }
        }
      }

      pcl::PointXYZRGB &ptOut = out(u,v);
      if (cnt>0)
      {
        mean/=(float)cnt;
        ptOut.x = mean[0];
        ptOut.y = mean[1];
        ptOut.z = mean[2];
        ptOut.rgb = pt0.rgb;
        
      }
      else
      {
        ptOut = pt0;
      }
    }
  }
}

/**
 * SubsamplePointCloud
 */
void SubsamplePointCloud::Subsample(pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZRGB> &out)
{
  #pragma omp parallel for
  for (unsigned v=0; v<out.height; v++)
  {
    for (unsigned u=0; u<out.width; u++)
    {
      out(u,v) = in(u*factor,v*factor);
    }
  }
}



/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void SubsamplePointCloud::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error ("[SubsamplePointCloud::setInputCloud] Point cloud needs to be organized!");

  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;
}

/**
 * resize the point cloud
 */
void SubsamplePointCloud::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[SubsamplePointCloud::compute] No point cloud available!");

  // init
  if (resCloud.get()==0 || (&*resCloud) == (&*cloud) )
    resCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  resCloud->header   = cloud->header;
  resCloud->width    = cloud->width/factor;
  resCloud->height   = cloud->height/factor;
  resCloud->is_dense = cloud->is_dense;
  resCloud->resize(resCloud->width*resCloud->height);

  // start subsampling
  if (param.useMean)
  {
    SubsampleMean(*cloud,*resCloud);
  }
  else
  {
    Subsample(*cloud,*resCloud);
  }
}

/**
 * get resized cloud
 */
void SubsamplePointCloud::getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  _cloud = resCloud;
}

/**
 * get normals
 */
void SubsamplePointCloud::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  _normals = normals;
}

/**
 * setParameter
 */
void SubsamplePointCloud::setParameter(Parameter p)
{
  param = p;
  sqrDist = pow(param.dist, 2);
  factor = param.subsamplingFactor;
}


} //-- THE END --

