#include "tgPCL2TomGine.h"
#include <pcl/ros/conversions.h>

using namespace TomGine;

tgModel
tgPCL2TomGine::convert (pcl::PolygonMesh &mesh)
{
  tgModel model;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
//  pcl::fromROSMsg (mesh.cloud, cloud);

  for (size_t i = 0; i < cloud.size (); i++)
  {
    pcl::PointXYZ &pt = cloud.at (i);
    tgVertex v;
    v.pos = vec3 (pt.x, pt.y, pt.z);
    //    v.normal = vec3(pt.normal_x, pt.normal_y, pt.normal_z);
    model.m_vertices.push_back (v);
  }

  for (size_t i = 0; i < mesh.polygons.size (); i++)
  {
    pcl::Vertices &triangle = mesh.polygons[i];
    tgFace face;
    for (size_t j = 0; j < triangle.vertices.size (); j++)
    {
      face.v.push_back (triangle.vertices[j]);
    }
    model.m_faces.push_back (face);
  }

  model.ComputeNormals ();
  return model;
}
