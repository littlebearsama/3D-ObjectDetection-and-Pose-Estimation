/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
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

#ifndef PCL_PREPROCESSING_XYZRC_HPP
#define PCL_PREPROCESSING_XYZRC_HPP

#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
 #define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif

// PCL includes
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <pcl/common/pca.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

#ifndef EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 #define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#endif

namespace pclAddOns
{
  
enum PCL_SELFMADE_EXCEPTIONS
{
  FILTER    = 1,
  SEGMENT      ,
  NORMALS      ,
  CURVATURE    ,
  CONVEXHULL   ,
};

//ep:begin revision 17-07-2014
template <class T>
bool readPointCloud(std::string filename, typename pcl::PointCloud<T>::Ptr &cloud)
{
  if (cloud.get() == 0)
    cloud.reset(new pcl::PointCloud<T> ());

  if (pcl::io::loadPCDFile<T> (filename,*cloud) == -1)
  {
    printf("[ERROR] Couldn't read point cloud %s.\n",filename.c_str());
    return false;
  }

  return(true);
}
//ep:end revision 17-07-2014

template <class T>
bool FilterPointCloud(typename pcl::PointCloud<T>::ConstPtr cloud, 
                      pcl::PointIndices::Ptr indices,
                      std::string dim = "z", float minLimit = 0.0, float maxLimit = 1.0) 
{
  pcl::PointCloud<T> temp;
  try {
    // Filter PointCloud
    pcl::PassThrough<T> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName(dim.c_str());
    pass.setFilterLimits(minLimit, maxLimit);
    pass.setKeepOrganized(true);
    //pass.setUserFilterValue(std::numeric_limits<float>::quiet_NaN());
    
    pass.filter(temp);
    
    if (!temp.size()) 
    {
      std::cerr << "[ERROR] After filtering not enough data." << std::endl;
      throw FILTER;
    }
  }

  catch (...) 
  {
    return (false);
  }
  
  // @ep: change this copying
  indices->indices.clear();
  for(unsigned int idx = 0; idx < temp.size(); ++idx)
  {
    if(!std::isnan(temp.points.at(idx).x))
      indices->indices.push_back(idx);
  }
  
  return (true);
}

template <class T>
bool FilterPointCloud2(typename pcl::PointCloud<T>::ConstPtr cloud, 
                      pcl::PointIndices::Ptr indices) 
{
  //std::cerr << cloud->points.size() << std::endl;
  //std::cerr << cloud->points.size() << std::endl;
  
  // @ep: change this copying
  indices->indices.clear();
  for(unsigned int idx = 0; idx < cloud->points.size(); ++idx)
  {
    if(std::isnan(cloud->points.at(idx).x) ||
       std::isnan(cloud->points.at(idx).y) ||
       std::isnan(cloud->points.at(idx).z))
    {
      continue;
    }
    
    if(cloud->points.at(idx).z <= 0)
    {
      continue;
    }
    
    //std::cerr << cloud->points.at(idx) << std::endl;
    
    indices->indices.push_back(idx);
  }
  
  return (true);
}

template <class T>
bool SegmentPlane(typename pcl::PointCloud<T>::ConstPtr cloud,
                  pcl::PointIndices::ConstPtr indices,
                  pcl::PointIndices::Ptr plane_indices,
                  pcl::PointIndices::Ptr objects_indices, 
                  pcl::ModelCoefficients::Ptr coefficients,
                  float distanceThreshold = 0.01) {
  try {
    // Segment Plane
    // Create the segmentation object
    pcl::SACSegmentation<T> seg;
    pcl::ExtractIndices<T> extract;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.setIndices(indices);
    seg.segment(*plane_indices, *coefficients);

    if(!plane_indices->indices.size())
    {
      std::cerr << "[ERROR] Not enough points in the plane." << std::endl;
      throw 1;
    }

    // Extract Objects Indices
    std::vector<int> set;
    set.resize(cloud->size());
    
    for(unsigned int idx = 0; idx < set.size(); ++idx)
    {
      set.at(idx) = 0;
    }
    
    for(unsigned int idx = 0; idx < plane_indices->indices.size(); ++idx)
    {
      set.at(plane_indices->indices.at(idx)) = 1;
    }
    
    objects_indices->indices.clear();
    for(unsigned int idx = 0; idx < indices->indices.size(); ++idx)
    {
      if(set.at(indices->indices.at(idx)) == 0)
      {
	objects_indices->indices.push_back(indices->indices.at(idx));
      }
      
    }
  }
  catch (...)
  {
    return(false);
  }
  return(true);
}

template <class T>
bool ComputePointNormals(typename pcl::PointCloud<T>::ConstPtr cloud,
                         pcl::PointIndices::ConstPtr indices,
                         pcl::PointCloud<pcl::Normal>::Ptr cloud_normal,
                         int ksearch_radius = 50) 
{
  try {
    // Extract point cloud normals
    pcl::NormalEstimation<T, pcl::Normal> ne;
    typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T > ());
    pcl::PointCloud<pcl::Normal> temp;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setIndices(indices);
    ne.setKSearch(ksearch_radius);
    // calculate the min value of the 
    ne.setViewPoint(0.0,0.0,0.0);
    ne.compute(*cloud_normal);
    
    if (!cloud_normal->size()) 
    {
      throw NORMALS;
    }
    
  } 
  catch (...) 
  {
    return (false);
  }
  
  return (true);
}

template <class T>
bool ConvexHullExtract(typename pcl::PointCloud<T>::ConstPtr cloud,
                       pcl::PointIndices::ConstPtr plane_indices,
		       pcl::PointIndices::ConstPtr objects_indices,
		       pcl::PointIndices::Ptr object_in_the_hull_indices,
                       pcl::ModelCoefficients::Ptr coefficients) 
{
  try {
    
//     if (!cloud->size()) 
//     {
//       throw CONVEXHULL;
//     }
    
    // Project plane the model inliers
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (plane_indices);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    proj.filter (*plane_projected);
    
    // Create a Convex Hull representation of the projected inliers
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConvexHull<pcl::PointXYZRGB> chull;
    std::vector<pcl::Vertices> polygons;
    chull.setInputCloud (plane_projected);
    chull.reconstruct (*cloud_hull,polygons);
    
    //project objects
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setIndices (objects_indices);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients (coefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
    proj.filter (*cloud_projected);
    
    pcl::CropHull<pcl::PointXYZRGB> cropHull;
    cropHull.setInputCloud(cloud_projected);
    cropHull.setHullCloud(cloud_hull);
    cropHull.setHullIndices(polygons);
    cropHull.setDim(2); 
    cropHull.setCropOutside(true);

    std::vector<int> indices_check;
    cropHull.filter(indices_check);
    
    object_in_the_hull_indices->indices.resize(indices_check.size());
    for(unsigned int i = 0; i < indices_check.size(); ++i)
    {
      object_in_the_hull_indices->indices.at(i) = objects_indices->indices.at(indices_check.at(i));
    }

  } 
  catch (...) 
  {
    return (false);
  }
  
  return (true);
}

/*template <class T>
bool euclideanClusterExtraction(typename pcl::PointCloud<T>::Ptr &cloud,
                                std::vector<pcl::PointCloud<PointXYZRGBRC>::Ptr> &clusters,
                                float clusteringTolerance = 0.02, int minClusterSize = 100, int maxClusterSize = 100000);*/

/*template <class T>
bool euclideanClusterExtraction(typename pcl::PointCloud<T>::Ptr &cloud,
                                std::vector<typename pcl::PointCloud<T>::Ptr> &clusters,
                                float clusteringTolerance = 0.02, int minClusterSize = 100, int maxClusterSize = 100000)
{
  // Creating the KdTree object for the search method of the extraction
  typename pcl::search::KdTree<T>::Ptr tree (new pcl::search::KdTree<T>());
  
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<T> ec;
  ec.setClusterTolerance (clusteringTolerance);
  ec.setMinClusterSize (minClusterSize);
  ec.setMaxClusterSize (maxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  clusters.resize(cluster_indices.size());
  
  for (size_t i = 0; i < cluster_indices.size(); ++i)
  {
    typename pcl::PointCloud<T>::Ptr cloud_cluster (new pcl::PointCloud<T>);
    clusters.at(i) = cloud_cluster;
    for (std::vector<int>::const_iterator pit = cluster_indices.at(i).indices.begin(); 
                                    pit != cluster_indices.at(i).indices.end (); pit++)
      cloud_cluster->points.push_back(cloud->points[*pit]);
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
  }

  return (true);
}*/

} //namespace pclAddOns

#endif //PCL_PREPROCESSING_XYZRC_HPP
