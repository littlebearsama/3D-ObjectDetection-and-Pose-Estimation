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

#ifndef EP_BASE_HH
#define EP_BASE_HH

#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
//#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>

namespace surface
{

/**
 * EPBase
 */
class EPBase
{
public:
  
  typedef boost::shared_ptr<EPBase> Ptr;

protected:
  
  bool computed;
  bool have_cloud;
  bool have_normals;
  bool have_indices;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;               ///< Input cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;                  ///< Normals (set from outside or from surfaces)
  std::vector<int> indices;                             ///< Indices to be used
  
  int width, height;

  inline int getIdx(int x, int y) const;
  inline int X(int idx);
  inline int Y(int idx);
  inline bool isNaN(const pcl::PointXYZRGB p);
  inline bool isNaN(const pcl::Normal n);
  inline bool isNaN(const Eigen::Vector3f p);
  
  std::string ClassName;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  EPBase();
  ~EPBase();

  /** Set input cloud **/
  virtual void setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud);
  /** Set normals **/
  void setNormals(const pcl::PointCloud<pcl::Normal>::Ptr &_normals);
  /** Set indices **/
  void setIndices(const pcl::PointIndices::Ptr &_indices);
  void setIndices(const std::vector<int> &_indices);

  /** General compute method **/
  virtual void compute();
  
  /** Return normals **/
  inline pcl::PointCloud<pcl::Normal>::Ptr getNormals();

};



/*********************** INLINE METHODES **************************/
inline bool EPBase::isNaN(const pcl::PointXYZRGB p)
{
  if(std::isnan(p.x) ||
     std::isnan(p.y) ||
     std::isnan(p.z))
  {
    return(true);
  }
  return(false);
}

inline bool EPBase::isNaN(const pcl::Normal n)
{
  if(std::isnan(n.normal[0]) ||
     std::isnan(n.normal[1]) ||
     std::isnan(n.normal[2]))
  {
    return(true);
  }
  return(false);
}

inline bool EPBase::isNaN(const Eigen::Vector3f p)
{
  if(std::isnan(p[0]) ||
     std::isnan(p[1]) ||
     std::isnan(p[2]))
  {
    return(true);
  }
  return(false);
}

inline int EPBase::getIdx(int x, int y) const
{
  return y*width+x; 
}

inline int EPBase::X(int idx)
{
  return idx%width;
}

inline int EPBase::Y(int idx)
{
  return idx/width;
}

inline pcl::PointCloud<pcl::Normal>::Ptr EPBase::getNormals()
{
  return normals;
}

}

#endif //BASE_HH

