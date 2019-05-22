#include "tgTomGineThreadPCL.h"
#include "tgPCL2TomGine.h"
#include <pcl/ros/conversions.h>

#include <iostream>

using namespace TomGine;

tgTomGineThreadPCL::tgTomGineThreadPCL (int w, int h, std::string windowname, bool bfc, float depth_min,
                                        float depth_max) :
  tgTomGineThread (w, h, windowname, bfc, depth_min, depth_max)
{

}

void
tgTomGineThreadPCL::SetCameraPCL (const Eigen::Matrix3f &i)
{
  cv::Mat
          cv_int =
              (cv::Mat_<double> (3, 3) << i (0, 0), i (0, 1), i (0, 2), i (1, 0), i (1, 1), i (1, 2), i (2, 0), i (2, 1), i (
                                                                                                                             2,
                                                                                                                             2));
  SetCamera (cv_int);
}
void
tgTomGineThreadPCL::SetCameraPCL (const Eigen::Matrix3d &i)
{
  cv::Mat
          cv_int =
              (cv::Mat_<double> (3, 3) << i (0, 0), i (0, 1), i (0, 2), i (1, 0), i (1, 1), i (1, 2), i (2, 0), i (2, 1), i (
                                                                                                                             2,
                                                                                                                             2));
  SetCamera (cv_int);
}
void
tgTomGineThreadPCL::SetCameraPCL (const Eigen::Matrix4f &e)
{
  cv::Mat
          R =
              (cv::Mat_<double> (3, 3) << e (0, 0), e (0, 1), e (0, 2), e (1, 0), e (1, 1), e (1, 2), e (2, 0), e (2, 1), e (
                                                                                                                             2,
                                                                                                                             2));
  cv::Mat t = (cv::Mat_<double> (3, 1) << e (0, 3), e (1, 3), e (2, 3));
  SetCamera (R, t);
}
void
tgTomGineThreadPCL::SetCameraPCL (const Eigen::Matrix4d &e)
{
  cv::Mat
          R =
              (cv::Mat_<double> (3, 3) << e (0, 0), e (0, 1), e (0, 2), e (1, 0), e (1, 1), e (1, 2), e (2, 0), e (2, 1), e (
                                                                                                                             2,
                                                                                                                             2));
  cv::Mat t = (cv::Mat_<double> (3, 1) << e (0, 3), e (1, 3), e (2, 3));
  SetCamera (R, t);
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZ> &cloud, short r, short g, short b,
                                      float point_size)
{
  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_point_size = point_size;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZ &pt = cloud.at (i);
    TomGine::tgColorPoint cpt;
    cpt.color[0] = r;
    cpt.color[1] = g;
    cpt.color[2] = b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZL> &cloud)
{
  tgModel* tg_cloud = new tgModel;
  std::map<int, tgRGBValue> colors;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZL &pt = cloud.at (i);

    if (colors.count (pt.label) == 0)
    {
      vec3 c;
      c.random ();
      colors[pt.label].Red = (short)(255.0 * c.x);
      colors[pt.label].Green = (short)(255.0 * c.y);
      colors[pt.label].Blue = (short)(255.0 * c.z);
    }

    TomGine::tgColorPoint cpt;
    cpt.color[0] = colors[pt.label].Red;
    cpt.color[1] = colors[pt.label].Green;
    cpt.color[2] = colors[pt.label].Blue;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, float point_size)
{
  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_point_size = point_size;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGB &pt = cloud.at (i);
    TomGine::tgColorPoint cpt;
    cpt.color[0] = pt.r;
    cpt.color[1] = pt.g;
    cpt.color[2] = pt.b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

void
tgTomGineThreadPCL::SetPointCloudPCL (int id, const pcl::PointCloud<pcl::PointXYZRGB> &cloud, float point_size)
{
  if (this->m_renderingStopped)
    return;

  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_point_size = point_size;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGB &pt = cloud.at (i);
    TomGine::tgColorPoint cpt;
    cpt.color[0] = pt.r;
    cpt.color[1] = pt.g;
    cpt.color[2] = pt.b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  if (id < 0 || id >= (int)m_pointclouds.size ())
  {
    pthread_mutex_unlock (&dataMutex);
    printf ("[tgTomGineThread::SetPointCloudPCL] Warning index out of bounds: %d.\n", id);
    return;
  }
  if (m_pointclouds[id] != NULL)
    delete m_pointclouds[id];
  m_pointclouds[id] = tg_cloud;
  pthread_mutex_unlock (&dataMutex);
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud, short r, short g, short b,
                                      float point_size)
{
  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_point_size = point_size;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGB &pt = cloud.at (i);
    TomGine::tgColorPoint cpt;
    cpt.color[0] = r;
    cpt.color[1] = g;
    cpt.color[2] = b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGBL> &cloud, float point_size, float blending)
{
  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_point_size = point_size;
  std::map<int, tgRGBValue> colors;
  std::map<int, bool> labels;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGBL &pt = cloud.at (i);
    if (colors.count (pt.label) == 0)
    {
      vec3 c;
      c.random ();
      colors[pt.label].Red = (short)(255.0 * c.x);
      colors[pt.label].Green = (short)(255.0 * c.y);
      colors[pt.label].Blue = (short)(255.0 * c.z);
    }

    if (labels.count (pt.label) == 0)
      if (!isnan (pt.x))
      {
        labels[pt.label] = true;
        std::ostringstream os;
        os << pt.label;
        this->AddLabel3D (os.str (), 14, pt.x, pt.y, pt.z);
      }

    TomGine::tgColorPoint cpt;
    //    if (pt.label == 0)
    //    {
    //      cpt.color[0] = pt.r;
    //      cpt.color[1] = pt.g;
    //      cpt.color[2] = pt.b;
    //    }
    //    else
    //    {
    cpt.color[0] = pt.r * blending + colors[pt.label].Red * (1.0f - blending);
    cpt.color[1] = pt.g * blending + colors[pt.label].Green * (1.0f - blending);
    cpt.color[2] = pt.b * blending + colors[pt.label].Blue * (1.0f - blending);
    //    }
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointNormal> &cloud, float normal_scale, short r,
                                      short g, short b)
{
  tgModel* tg_cloud = new tgModel;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointNormal &pt = cloud.at (i);

    TomGine::tgColorPoint cpt;
    cpt.color[0] = r;
    cpt.color[1] = g;
    cpt.color[2] = b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);

    TomGine::tgLine line;
    line.start = vec3 (pt.x, pt.y, pt.z);
    line.end = vec3 (pt.x + normal_scale * pt.normal_x, pt.y + normal_scale * pt.normal_y,
                     pt.z + normal_scale * pt.normal_z);
    tg_cloud->m_lines.push_back (line);
    tg_cloud->m_line_color = vec3 (0.0, 0.0, 1.0);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  tgModel* tg_cloud = new tgModel;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZI &pt = cloud.at (i);
    TomGine::tgColorPoint cpt;
    cpt.color[0] = static_cast<short> (pt.intensity * 255);
    cpt.color[1] = static_cast<short> (pt.intensity * 255);
    cpt.color[2] = static_cast<short> (pt.intensity * 255);
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, float normal_length,
                                      vec3 normal_color, float normal_width, float point_size)
{
  tgModel* tg_cloud = new tgModel;
  tg_cloud->m_line_color = normal_color;
  tg_cloud->m_line_width = normal_width;
  tg_cloud->m_point_size = point_size;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGBNormal &pt = cloud.at (i);

    TomGine::tgColorPoint cpt;
    cpt.color[0] = pt.r;
    cpt.color[1] = pt.g;
    cpt.color[2] = pt.b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);

    TomGine::tgLine line;
    line.start = vec3 (pt.x, pt.y, pt.z);
    line.end = vec3 (pt.x + normal_length * pt.normal_x, pt.y + normal_length * pt.normal_y,
                     pt.z + normal_length * pt.normal_z);
    tg_cloud->m_lines.push_back (line);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud, short r, short g, short b,
                                      float normal_scale)
{
  tgModel* tg_cloud = new tgModel;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGBNormal &pt = cloud.at (i);

    TomGine::tgColorPoint cpt;
    cpt.color[0] = r;
    cpt.color[1] = g;
    cpt.color[2] = b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);

    TomGine::tgLine line;
    line.start = vec3 (pt.x, pt.y, pt.z);
    line.end = vec3 (pt.x + normal_scale * pt.normal_x, pt.y + normal_scale * pt.normal_y,
                     pt.z + normal_scale * pt.normal_z);
    tg_cloud->m_lines.push_back (line);
    tg_cloud->m_line_color = vec3 (0.0, 0.0, 1.0);
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                      const pcl::PointCloud<pcl::Normal> &normals, float normal_scale)
{
  tgModel* tg_cloud = new tgModel;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGB &pt = cloud.at (i);

    TomGine::tgColorPoint cpt;
    cpt.color[0] = pt.r;
    cpt.color[1] = pt.g;
    cpt.color[2] = pt.b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);

    if (i < normals.size ())
    {
      const pcl::Normal &n = normals.at (i);
      TomGine::tgLine line;
      line.start = vec3 (pt.x, pt.y, pt.z);
      line.end = vec3 (pt.x + normal_scale * n.normal_x, pt.y + normal_scale * n.normal_y,
                       pt.z + normal_scale * n.normal_z);
      tg_cloud->m_lines.push_back (line);
      tg_cloud->m_line_color = vec3 (0.0, 0.0, 1.0);
    }
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddPointCloudPCL (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
                                      const pcl::PointCloud<pcl::Normal> &normals, short r, short g, short b,
                                      float normal_scale)
{
  tgModel* tg_cloud = new tgModel;

  for (size_t i = 0; i < cloud.size (); i++)
  {
    const pcl::PointXYZRGB &pt = cloud.at (i);

    TomGine::tgColorPoint cpt;
    cpt.color[0] = r;
    cpt.color[1] = g;
    cpt.color[2] = b;
    cpt.pos = vec3 (pt.x, pt.y, pt.z);
    tg_cloud->m_colorpoints.push_back (cpt);

    if (i < normals.size ())
    {
      const pcl::Normal &n = normals.at (i);
      TomGine::tgLine line;
      line.start = vec3 (pt.x, pt.y, pt.z);
      line.end = vec3 (pt.x + normal_scale * n.normal_x, pt.y + normal_scale * n.normal_y,
                       pt.z + normal_scale * n.normal_z);
      tg_cloud->m_lines.push_back (line);
      tg_cloud->m_line_color = vec3 (0.0, 0.0, 1.0);
    }
  }

  pthread_mutex_lock (&dataMutex);
  this->m_pointclouds.push_back (tg_cloud);
  int id = (this->m_pointclouds.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

int
tgTomGineThreadPCL::AddModelPCL (pcl::PolygonMesh &mesh, short r, short g, short b, short a)
{
  tgRenderModel model = tgPCL2TomGine::convert (mesh);
  model.CheckFaces();

  model.SetColor (float (r) / 255.0f, float (g) / 255.0f, float (b) / 255.0f, float (a) / 255.0f);

  pthread_mutex_lock (&dataMutex);
  this->m_models3D.push_back (new TomGine::tgTextureModel (model));
  int id = (this->m_models3D.size () - 1);
  pthread_mutex_unlock (&dataMutex);
  return id;
}

void
tgTomGineThreadPCL::SetModelPCL (int id, pcl::PolygonMesh &mesh)
{
  pthread_mutex_lock (&dataMutex);
  if (id < 0 || id >= (int)this->m_models3D.size ())
  {
    pthread_mutex_unlock (&dataMutex);
    printf ("[tgTomGineThread::SetModelPCL] Warning index out of bounds: %d.\n", id);
    return;
  }
  pthread_mutex_unlock (&dataMutex);

  tgRenderModel model = tgPCL2TomGine::convert (mesh);

  pthread_mutex_lock (&dataMutex);
  model.m_material = this->m_models3D[id]->m_material;
  if (m_models3D[id] != NULL)
    delete m_models3D[id];
  m_models3D[id] = new TomGine::tgTextureModel (model);
  pthread_mutex_unlock (&dataMutex);
}

