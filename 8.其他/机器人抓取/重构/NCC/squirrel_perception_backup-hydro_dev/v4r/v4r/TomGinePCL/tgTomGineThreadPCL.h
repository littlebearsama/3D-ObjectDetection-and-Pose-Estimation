#ifndef TG_TOMGINE_THREAD_PCL_H
#define TG_TOMGINE_THREAD_PCL_H

#include "v4r/TomGine/tgTomGineThread.h"

#undef Success

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace TomGine
{

  class tgTomGineThreadPCL : public tgTomGineThread
  {
  public:
    /** @brief Initialize class and start threads. */
    tgTomGineThreadPCL (int w, int h, std::string windowname = std::string ("TomGine"), bool bfc = false,
                        float depth_min = 0.01, float depth_max = 10.0);

    void
    SetCameraPCL (const Eigen::Matrix3f &intrinsic);
    void
    SetCameraPCL (const Eigen::Matrix4f &extrinsic);
    void
    SetCameraPCL (const Eigen::Matrix3d &intrinsic);
    void
    SetCameraPCL (const Eigen::Matrix4d &extrinsic);

    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZ> &cloud, short r = 255, short g = 255, short b = 255,
                      float point_size = 1.0);

    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZL> &cloud);

    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, float point_size = 1.0);
    void
    SetPointCloudPCL (int id, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, float point_size = 1.0);

    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, short r, short g, short b, float point_size = 1);

    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGBL> &cloud, float point_size = 1.0f, float blending = 0.0f);

    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointNormal> &cloud, float normal_scale = 1.0, short r = 255,
                      short g = 255, short b = 255);

    /** @brief Draw a single coloured point cloud (with intensity from PointXYZI). **/
    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZI> &cloud);

    /** @brief Draw a point cloud with normals. **/
    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, float normal_length = 1.0,
                      vec3 normal_color = vec3 (0.0, 0.0, 1.0), float normal_width = 1.0, float point_size = 1.0);

    /** @brief Draw a point cloud with normals. **/
    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, short r, short g, short b,
                      float normal_scale = 1.0);

    /** @brief Draw a point cloud with normals. **/
    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
                      float normal_scale = 1.0);

    /** @brief Draw a point cloud with normals. **/
    int
    AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, const pcl::PointCloud<pcl::Normal> &normals,
                      short r, short g, short b, float normal_scale = 1.0);

    int
    AddModelPCL (pcl::PolygonMesh &mesh, short r = 255, short g = 255, short b = 255, short a = 255);

    void
    SetModelPCL (int id, pcl::PolygonMesh &mesh);

  };

}

#endif
