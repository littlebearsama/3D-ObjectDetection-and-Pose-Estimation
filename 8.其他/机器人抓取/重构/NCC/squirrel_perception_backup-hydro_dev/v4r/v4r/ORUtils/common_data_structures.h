#ifndef FAAT_PCL_COMMON_DATA_STR
#define FAAT_PCL_COMMON_DATA_STR

#include <pcl/common/common.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/voxel_grid.h>
namespace faat_pcl
{
  template<typename PointT>
  struct PlaneModel
  {
    pcl::ModelCoefficients coefficients_;
    typename pcl::PointCloud<PointT>::Ptr plane_cloud_;
    pcl::PolygonMeshPtr convex_hull_;
    typename pcl::PointCloud<PointT>::Ptr convex_hull_cloud_;
    typename pcl::PointCloud<PointT>::Ptr cloud_;
    pcl::PointIndices inliers_;

    void projectPlaneCloud(float resolution=0.001f)
    {
      Eigen::Vector4f model_coefficients;
      model_coefficients[0] = coefficients_.values[0];
      model_coefficients[1] = coefficients_.values[1];
      model_coefficients[2] = coefficients_.values[2];
      model_coefficients[3] = coefficients_.values[3];

      pcl::SampleConsensusModelPlane<PointT> sacmodel (cloud_);
      typename pcl::PointCloud<PointT>::Ptr projected(new pcl::PointCloud<PointT>);
      sacmodel.projectPoints (inliers_.indices, model_coefficients, *projected, false);

      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud (projected);
      float leaf_size_ = resolution;
      vg.setLeafSize (leaf_size_, leaf_size_, leaf_size_);
      vg.filter (*plane_cloud_);
    }
  };
}

#endif
