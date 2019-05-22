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
 * @file PCLUtils.hh
 * @author Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Utils for calculations with PCL and openCV prototypes.
 */

#include "PCLUtils.h"
#include "cmath"

namespace pclA {

  
void ConvertPCLCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &in,
                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out)
{
  out.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  out->width = in->width;
  out->height = in->height;
  out->points.resize(in->width*in->height);
  for (unsigned row = 0; row < in->height; row++) {
    for (unsigned col = 0; col < in->width; col++) {
      int idx = row * in->width + col;
      pcl::PointXYZRGBL &pt = in->points[idx];
      pcl::PointXYZRGB &npt = out->points[idx];
      npt.x = pt.x;
      npt.y = pt.y;
      npt.z = pt.z;
      npt.rgb = pt.rgb;
    }
  }
}
  
void ConvertCvVec2PCLCloud(const std::vector<cv::Vec4f> &cv_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  if (pcl_cloud.get() == 0)
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_cloud->width = cv_cloud.size();
  pcl_cloud->height = 1;
  pcl_cloud->points.resize(cv_cloud.size());
  for (unsigned idx = 0; idx < cv_cloud.size(); idx++) {
    pcl_cloud->points[idx].x = (float) cv_cloud[idx][0];
    pcl_cloud->points[idx].y = (float) cv_cloud[idx][1];
    pcl_cloud->points[idx].z = (float) cv_cloud[idx][2];
    pcl_cloud->points[idx].rgb = (float) cv_cloud[idx][3];
  }
}

void ConvertCvMat2PCLCloud(const cv::Mat_<cv::Vec4f> &cv_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  int pos = 0;
  if (pcl_cloud.get() == 0)
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl_cloud->width = cv_cloud.cols;
  pcl_cloud->height = cv_cloud.rows;
  pcl_cloud->points.resize(cv_cloud.cols * cv_cloud.rows);

  for (int row = 0; row < cv_cloud.rows; row++) {
    for (int col = 0; col < cv_cloud.cols; col++) {
      pos = row * cv_cloud.cols + col;
      pcl_cloud->points[pos].x = cv_cloud(row, col)[0];
      pcl_cloud->points[pos].y = cv_cloud(row, col)[1];
      pcl_cloud->points[pos].z = cv_cloud(row, col)[2];
      pcl_cloud->points[pos].rgb = cv_cloud(row, col)[3];
    }
  }
}

void ConvertCvMat2PCLCloud(const cv::Mat_<cv::Vec4f> &cv_cloud, pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &pcl_cloud)
{
  int pos = 0;
  if (pcl_cloud.get() == 0)
    pcl_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBL>);

  pcl_cloud->width = cv_cloud.cols;
  pcl_cloud->height = cv_cloud.rows;
  pcl_cloud->points.resize(cv_cloud.cols * cv_cloud.rows);

  for (int row = 0; row < cv_cloud.rows; row++) {
    for (int col = 0; col < cv_cloud.cols; col++) {
      pos = row * cv_cloud.cols + col;
      pcl_cloud->points[pos].x = cv_cloud(row, col)[0];
      pcl_cloud->points[pos].y = cv_cloud(row, col)[1];
      pcl_cloud->points[pos].z = cv_cloud(row, col)[2];
      pcl_cloud->points[pos].rgb = cv_cloud(row, col)[3];
      pcl_cloud->points[pos].label = 0;
    }
  }
}

void ConvertCvMat2PCLNormals(const cv::Mat_<cv::Vec4f> &cv_normals, 
                             pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  int pos = 0;
  if (normals.get() == 0)
    normals.reset(new pcl::PointCloud<pcl::Normal>);

  normals->width = cv_normals.cols;
  normals->height = cv_normals.rows;
  normals->points.resize(cv_normals.cols * cv_normals.rows);

  for (int row = 0; row < cv_normals.rows; row++) {
    for (int col = 0; col < cv_normals.cols; col++) {
      pos = row * cv_normals.cols + col;
      normals->points[pos].normal_x = cv_normals(row, col)[0];
      normals->points[pos].normal_y = cv_normals(row, col)[1];
      normals->points[pos].normal_z = cv_normals(row, col)[2];
      normals->points[pos].curvature = cv_normals(row, col)[3]; /// curvature?
    }
  }
}

void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                           std::vector<cv::Vec4f> &cvCloud,
                           bool random_colors)
{
  int pcWidth = pcl_cloud->width;
  int pcHeight = pcl_cloud->height;
  unsigned position = 0;

  pclA::RGBValue color;
  if (random_colors) {
    color.r = rand() % 255;
    color.g = rand() % 255;
    color.b = rand() % 255;
    color.a = 0;
  }
  for (int row = 0; row < pcHeight; row++) {
    for (int col = 0; col < pcWidth; col++) {
      cv::Vec4f p;
      position = row * pcWidth + col;
      p[0] = pcl_cloud->points[position].x;
      p[1] = pcl_cloud->points[position].y;
      p[2] = pcl_cloud->points[position].z;
      if (random_colors)
        p[3] = color.float_value;
      else
        p[3] = pcl_cloud->points[position].rgb;
      cvCloud.push_back(p);
    }
  }
}

void ConvertPCLCloud2CvVec(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                           std::vector<cv::Vec3f> &cvCloud)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec3f p;
      position = row * pcWidth + col;
      p[0] = pcl_cloud->points[position].x;
      p[1] = pcl_cloud->points[position].y;
      p[2] = pcl_cloud->points[position].z;

      cvCloud.push_back(p);
    }
  }
}

void ConvertPCLClouds2CvVecs(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcl_clouds,
                             std::vector<std::vector<cv::Vec4f> > &cv_clouds, 
                             bool random_colors)
{
  std::vector<cv::Vec4f> cv_cloud;
  for (unsigned idx = 0; idx < pcl_clouds.size(); idx++) {
    ConvertPCLCloud2CvVec(pcl_clouds[idx], cv_cloud, random_colors);
    cv_clouds.push_back(cv_cloud);
    cv_cloud.clear();
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGBL>::Ptr &cloud, 
                           cv::Mat_<cv::Vec4f> &cvCloud,
                           bool random_colors)
{
  unsigned pcWidth = cloud->width;
  unsigned pcHeight = cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth);

  RGBValue color;
  if (random_colors) {
    color.r = rand() % 255;
    color.g = rand() % 255;
    color.b = rand() % 255;
    color.a = 0;
  }

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      p[0] = cloud->points[position].x;
      p[1] = cloud->points[position].y;
      p[2] = cloud->points[position].z;
      if (random_colors)
        p[3] = color.float_value;
      else
        p[3] = cloud->points[position].rgb;
    }
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                           cv::Mat_<cv::Vec4f> &cvCloud,
                           bool random_colors)
{
  unsigned pcWidth = cloud->width;
  unsigned pcHeight = cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth);

  RGBValue color;
  if (random_colors) {
    color.r = rand() % 255;
    color.g = rand() % 255;
    color.b = rand() % 255;
    color.a = 0;
  }

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      p[0] = cloud->points[position].x;
      p[1] = cloud->points[position].y;
      p[2] = cloud->points[position].z;
      if (random_colors)
        p[3] = color.float_value;
      else
        p[3] = cloud->points[position].rgb;
    }
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, cv::Mat_<cv::Vec4f> &cvCloud,
    float z_min, float z_max)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth); // rows = height / cols = width

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {

      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];
      if (pt.z <= z_max && pt.z >= z_min) {
        p[0] = pt.x;
        p[1] = pt.y;
        p[2] = pt.z;
        p[3] = pt.rgb;
      } else {
        p = cv::Vec4f(NAN, NAN, NAN, NAN);
      }
    }
  }
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, cv::Mat_<cv::Vec4f> &cvCloud,
    RGBValue color)
{
  printf("ConvertPCLCloud2CvMat: %d %d\n", cloud->width, cloud->height);
  unsigned pcWidth = cloud->width;
  unsigned pcHeight = cloud->height;
  unsigned position = 0;

  Eigen::Vector3f v_max(0.0, 0.0, 0.0);
  Eigen::Vector3f v_min(DBL_MAX, DBL_MAX, DBL_MAX);

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth); // rows = height / cols = width

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      p[0] = cloud->points[position].x;
      p[1] = cloud->points[position].y;
      p[2] = cloud->points[position].z;
      p[3] = color.float_value;

      if (p[0] > v_max(0))
        v_max(0) = p[0];
      if (p[1] > v_max(1))
        v_max(1) = p[1];
      if (p[2] > v_max(2))
        v_max(2) = p[2];
      if (p[0] < v_min(0))
        v_min(0) = p[0];
      if (p[1] < v_min(1))
        v_min(1) = p[1];
      if (p[2] < v_min(2))
        v_min(2) = p[2];
    }
  }

  printf("min: %f %f %f  max: %f %f %f\n", v_min(0), v_min(1), v_min(2), v_max(0), v_max(1), v_max(2));

  printf("ConvertPCLCloud2CvMat: done\n");
}

void ConvertPCLCloud2CvMat(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud, cv::Mat_<cv::Vec4f> &cvCloud)
{
  unsigned pcWidth = cloud->width;
  unsigned pcHeight = cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth); // rows = height / cols = width

  RGBValue color;
  color.r = 0;
  color.g = 0;
  color.b = 0;
  color.a = 0;

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      p[0] = cloud->points[position].normal_x;
      p[1] = cloud->points[position].normal_y;
      p[2] = cloud->points[position].normal_z;
      p[3] = color.float_value;
    }
  }
}

void ConvertPCLCloud2CvMatCol(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, 
                              cv::Mat_<cv::Vec4f> &cvCloud,
                              float scale)
{
  unsigned pcWidth = cloud->width;
  unsigned pcHeight = cloud->height;
  unsigned position = 0;

  cvCloud = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth); // rows = height / cols = width

  RGBValue color;
  color.a = 0;
  float fcol;

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec4f &p = cvCloud.at<cv::Vec4f> (row, col);
      position = row * pcWidth + col;
      p[0] = cloud->points[position].x;
      p[1] = cloud->points[position].y;
      p[2] = cloud->points[position].z;
      fcol = cloud->points[position].intensity*scale;
      if (fcol>255) fcol=255;
      color.r=color.g=color.b = fcol;
      p[3] = color.float_value;
    }
  }
}

void ConvertPCLNormals2CvMat(const pcl::PointCloud<pcl::Normal>::Ptr &pcl_normals,
                             cv::Mat_<cv::Vec4f> &cvNormals)
{
  unsigned pcWidth = pcl_normals->width;
  unsigned pcHeight = pcl_normals->height;
  unsigned position = 0;

  cvNormals = cv::Mat_<cv::Vec4f>(pcHeight, pcWidth);
    for (unsigned row = 0; row < pcHeight; row++) {
      for (unsigned col = 0; col < pcWidth; col++) {
        position = row * pcWidth + col;
        cv::Vec4f &p = cvNormals.at<cv::Vec4f> (row, col);
        p[0] = pcl_normals->points[position].normal_x;
        p[1] = pcl_normals->points[position].normal_y;
        p[2] = pcl_normals->points[position].normal_z;
        p[3] = pcl_normals->points[position].curvature;
      }
    }
}

void ConvertPCLClouds2CvMats(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcl_clouds,
    std::vector<cv::Mat_<cv::Vec4f> > &cv_clouds, bool random_colors)
{
  cv::Mat_<cv::Vec4f> cv_cloud;
  for (unsigned idx = 0; idx < pcl_clouds.size(); idx++) {
    ConvertPCLCloud2CvMat(pcl_clouds[idx], cv_cloud, random_colors);
    cv_clouds.push_back(cv_cloud);
  }
}

void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  
  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      cvp[2] = pt.r;
      cvp[1] = pt.g;
      cvp[0] = pt.b;
    }
  }
}

void ConvertPCLCloud2Image(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_cloud, cv::Mat_<cv::Vec3b> &image)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;

  image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      cv::Vec3b &cvp = image.at<cv::Vec3b> (row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      cvp[0] = pt.r;
      cvp[1] = pt.g;
      cvp[2] = pt.b;
    }
  }
}

void ConvertCvMat2Image(const cv::Mat_<float> &mat_image, cv::Mat_<cv::Vec3b> &image, bool invert_y_coordinate)
{
  int w = 0;
  for (int v = 0; v < (int) mat_image.rows; ++v) {
    for (int u = 0; u < (int) mat_image.cols; ++u) {
      if (invert_y_coordinate)
        w = mat_image.rows - v - 1;
      else
        w = v;
      const float &value = mat_image(w, u);
      cv::Vec3b &pt = image(w, u);

      pt[0] = (uchar) (value * 255.);
      pt[1] = (uchar) (value * 255.);
      pt[2] = (uchar) (value * 255.);
    }
  }
}

void ConvertCvMat2Image(const cv::Mat_<uchar> &mat_image, cv::Mat_<cv::Vec3b> &image, bool invert_y_coordinate)
{
  int w = 0;
  for (int v = 0; v < (int) mat_image.rows; ++v) {
    for (int u = 0; u < (int) mat_image.cols; ++u) {
      if (invert_y_coordinate)
        w = mat_image.rows - v - 1;
      else
        w = v;
      const uchar &value = mat_image(w, u);
      cv::Vec3b &pt = image(w, u);

      pt[0] = (uchar) (value);
      pt[1] = (uchar) (value);
      pt[2] = (uchar) (value);
    }
  }
}

void ConvertCvMat2Image(const cv::Mat_<cv::Vec3f> &mat_image, cv::Mat_<cv::Vec3b> &image, bool invert_y_coordinate)
{
  printf("VisionUtils::ConvertCVMat2Image: Time to implement!\n");
  //   int w = 0;
  //   for( int v = 0; v < (int) mat_image.rows; ++v )
  //   {
  //     for( int u = 0; u < (int) mat_image.cols; ++u )
  //     {
  //       if(invert_y_coordinate) w = mat_image.rows-v-1;
  //       else w = v;
  //       const cv::Vec3f &value = mat_image(w,u);
  //       cv::Vec3b &pt = image(w, u);
  //
  //       pt[0] = (uchar) (value);
  //       pt[1] = (uchar) (value);
  //       pt[2] = (uchar) (value);
  //     }
  //   }
}

void ConvertIndexes2Mask(std::vector<int> &indexes, cv::Mat_<cv::Vec3b> &patch_mask, int width, int height)
{
  if (patch_mask.empty())
    patch_mask = cv::Mat_<cv::Vec3b>::zeros(height, width);

  cv::Mat_<cv::Vec3b> patch_edge = cv::Mat_<cv::Vec3b>::zeros(patch_mask.rows, patch_mask.cols);

  for (size_t i = 0; i < indexes.size(); i++) {
    int row = indexes[i] / patch_mask.cols;
    int col = indexes[i] % patch_mask.cols;
    cv::Vec3b &pt = patch_mask(row, col);
    pt[0] = 255;
    pt[1] = 255;
    pt[2] = 255;
  }
}

void ConvertCvMask2PCLIndices(const cv::Mat_<uchar> &mask, std::vector<int> &indices, unsigned downsample)
{
  unsigned numIndices(0);

  if (mask.empty()) {
    printf("PCLUtils::ConvertCvMask2PCLIndices: Error: mask is empty!\n");
    return;
  }

  indices.resize(0);

  for (int i = 0; i < mask.rows; i++) {
    if (i % downsample)
      continue;
    for (int j = 0; j < mask.cols; j++) {
      if (j % downsample)
        continue;

      if (mask(i, j) > 128) {
        indices.push_back(mask.cols * i + j);
        numIndices++;
      }

    }
  }

}

void ConvertMask2Edges(const cv::Mat_<cv::Vec3b> &mask, cv::Mat_<cv::Vec3b> &edge)
{
  if (mask.empty()) {
    printf("PCLUtils::ConvertMask2Edges: Error: mask is empty!\n");
    return;
  }
  if (edge.empty())
    edge = cv::Mat_<cv::Vec3b>::zeros(mask.rows, mask.cols);

  for (int v = 1; v < (int) mask.cols - 1; ++v) {
    for (int u = 1; u < (int) mask.rows - 1; ++u) {
      cv::Vec3b &pt_e = edge(u, v);
      if (mask(u, v)[0] != 0) {
        if (mask(u + 1, v)[0] == 0 || mask(u - 1, v)[0] == 0 || mask(u, v + 1)[0] == 0 || mask(u, v - 1)[0] == 0) {
          pt_e[0] = 255;
          pt_e[1] = 255;
          pt_e[2] = 255;
        }
      }
    }
  }
}

void ConvertEdges2Indexes(const cv::Mat_<cv::Vec3b> &edge, std::vector<int> &indexes)
{
  if (edge.empty()) {
    printf("PCLUtils::ConvertMask2Edges: Error: mask is empty!\n");
    return;
  }
  if (indexes.size() != 0)
    indexes.resize(0);

  for (int v = 0; v < (int) edge.cols; ++v) {
    for (int u = 0; u < (int) edge.rows; ++u) {
      if (edge(u, v)[0] != 0)
        indexes.push_back(u * edge.cols + v);
    }
  }
}

// void ShowNormalImage(const pcl::PointCloud<pcl::Normal>::Ptr &normals)
// {
//   unsigned pcWidth = normals->width;
//   unsigned pcHeight = normals->height;
//   int pos = 0;
// 
//   cv::Mat_<cv::Vec3b> x_image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);
//   cv::Mat_<cv::Vec3b> y_image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);
//   cv::Mat_<cv::Vec3b> z_image = cv::Mat_<cv::Vec3b>(pcHeight, pcWidth);
// 
//   float x, y, z;
//   int i_x, i_y, i_z;
//   for( int v = 0; v < (int) pcWidth; ++v ) {
//     for( int u = 0; u < (int) pcHeight; ++u ) {
//       pos = u * pcWidth + v;
// //       const pcl::Normal &normal = normals->points[pos];  /// TODO Unused
//       cv::Vec3b &x_pt = x_image(u, v);
//       cv::Vec3b &y_pt = y_image(u, v);
//       cv::Vec3b &z_pt = z_image(u, v);
// 
//       x = (normals->points[pos].normal_x + 1) * 255. / 2.;
//       y = (normals->points[pos].normal_y + 1) * 255. / 2.;
//       z = (normals->points[pos].normal_z + 1) * 255. / 2.;
// 
//       i_x = (int) x;
//       i_y = (int) y;
//       i_z = (int) z;
// 
//       // printf("values: %4.3f - %4.3f - %4.3f\n", (normals->points[pos].normal_x + 1)/2., (normals->points[pos].normal_y + 1)/2., (normals->points[pos].normal_z + 1)/2.);
//       x_pt[0] = (uchar) i_x;
//       x_pt[1] = 0;
//       x_pt[2] = 0;
// 
//       y_pt[0] = 0;
//       y_pt[1] = (uchar) i_y;
//       y_pt[2] = 0;
// 
//       z_pt[0] = 0;
//       z_pt[1] = 0;
//       z_pt[2] = (uchar) i_z;
// 
//       //printf("values: %4.3f - %4.3f - %4.3f\n", normals->points[pos].normal_x, normals->points[pos].normal_y, normals->points[pos].normal_z);
//       // printf("values: %4.3f - %4.3f - %4.3f\n", x, y, z);
//       // printf("values: %u - %u - %u\n", i_x, i_y, i_z);
//       // printf("values: %u - %u - %u\n", x_pt[0], y_pt[1], z_pt[2]);
//     }
//   }
// 
//   cv::imshow("X-Normals", x_image);
//   cv::imshow("Y-Normals", y_image);
//   cv::imshow("Z-Normals", z_image);
// }

void ConvertPCLCloud2Mask(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, cv::Mat_<uchar> &mask,
    bool treat_zeros_as_nan, bool treat_floatmax_as_nan, bool use_z_filter, double zMin, double zMax)
{
  unsigned pcWidth = pcl_cloud->width;
  unsigned pcHeight = pcl_cloud->height;
  unsigned position = 0;
  mask = cv::Mat_<uchar>(pcHeight, pcWidth);

  for (unsigned row = 0; row < pcHeight; row++) {
    for (unsigned col = 0; col < pcWidth; col++) {
      uchar &m = mask(row, col);
      position = row * pcWidth + col;
      const pcl::PointXYZRGB &pt = pcl_cloud->points[position];

      if (pt.z < zMin || pt.z > zMax)
        m = 0;
      else if (pt.x != pt.x || pt.y != pt.y || pt.z != pt.z)
        m = 0;
      else if (treat_zeros_as_nan && pt.x == 0.0 && pt.y == 0.0 && pt.z == 0.0)
        m = 0;
      else if (treat_floatmax_as_nan && pt.x == FLT_MAX && pt.y == FLT_MAX && pt.z == FLT_MAX)
        m = 0;
      else
        m = 255;
    }
  }
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src, pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  dst.header = src.header;
  dst.width = src.width;
  dst.height = src.height;
  dst.is_dense = src.is_dense;
  dst.points = src.points;
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGBA> &src, pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  dst.header = src.header;
  dst.width = src.width;
  dst.height = src.height;
  dst.is_dense = src.is_dense;
  
  dst.points.resize(src.points.size());
  for (unsigned i=0; i<src.points.size(); i++)
  {
    const pcl::PointXYZRGBA &ptSrc = src.points[i];
    pcl::PointXYZRGB &ptDst = dst.points[i];

    ptDst.getVector4fMap() = ptSrc.getVector4fMap();
    ptDst.r = ptSrc.r, ptDst.g = ptSrc.g, ptDst.b = ptSrc.b;
  }
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst)
{
  if (dst.get() == 0)
    dst.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  dst->header = src->header;
  dst->width = src->width;
  dst->height = src->height;
  dst->is_dense = src->is_dense;
  dst->points = src->points;
}

void CopyPointCloud(const pcl::PointCloud<pcl::PointXYZRGB> &src, const pcl::PointIndices indices,
    pcl::PointCloud<pcl::PointXYZRGB> &dst)
{
  printf("PCLUtils::CopyPointCloud: Experimental function!\n");
  dst.header = src.header;
  dst.width = src.width;
  dst.height = src.height;
  dst.is_dense = src.is_dense;
  dst.points = src.points;

  for (unsigned i = 0; i < src.points.size(); i++) {
    dst.points[i].x = NAN;
    dst.points[i].y = NAN;
    dst.points[i].z = NAN;
    dst.points[i].rgb = 0;
  }

  for (unsigned i = 0; i < indices.indices.size(); i++)
    dst.points[i] = src.points[i];
  printf("PCLUtils::CopyPointCloud: Experimental function => ended!\n");
}

void PrintPCLCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  printf("PCL Cloud:\n");
  for (unsigned idx = 0; idx < pcl_cloud->points.size(); idx++) {
    RGBValue color;
    color.float_value = pcl_cloud->points[idx].rgb;
    printf(" point[%u]: %4.4f / %4.4f / %4.4f with rgb: %u / %u / %u\n", idx, pcl_cloud->points[idx].x,
        pcl_cloud->points[idx].y, pcl_cloud->points[idx].z, color.r, color.g, color.b);
  }
}

void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (unsigned idx = 0; idx < pcl_cloud->points.size(); idx++) {
    if (pcl_cloud->points[idx].z != 0. && (pcl_cloud->points[idx].x == pcl_cloud->points[idx].x))
      newCloud->points.push_back(pcl_cloud->points[idx]);
  }
  pcl_cloud = newCloud;
}

void RemoveZeros(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, std::vector<int> &indices)
{
  if (pcl_cloud->points.size() != indices.size()) {
    printf("PCLUtils::RemoveZeros: Warning: Size of point cloud and indices differ: Re-indexing.\n");

    indices.resize(pcl_cloud->points.size());
    for (unsigned i = 0; i < pcl_cloud->points.size(); i++)
      indices[i] = i;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<int> new_indices;
  for (unsigned idx = 0; idx < pcl_cloud->points.size(); idx++) {
    if (pcl_cloud->points[idx].z != 0. && (pcl_cloud->points[idx].x == pcl_cloud->points[idx].x)) {
      newCloud->points.push_back(pcl_cloud->points[idx]);
      new_indices.push_back(indices[idx]);
    }
  }
  pcl_cloud = newCloud;
  indices = new_indices;
}

void RemoveNormalZeros(pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
  pcl::PointCloud<pcl::Normal>::Ptr newCloud(new pcl::PointCloud<pcl::Normal>);
  for (unsigned idx = 0; idx < normals->points.size(); idx++) {
    if (normals->points[idx].normal_z != 0.)
      newCloud->points.push_back(normals->points[idx]);
  }
  normals = newCloud;
}

void FilterZ(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double minZ, double maxZ)
{
  pcl::PassThrough<pcl::PointXYZRGB> zFilter;
  zFilter.setFilterFieldName("z");
  zFilter.setFilterLimits(minZ, maxZ);
  zFilter.setKeepOrganized(true);
  zFilter.setInputCloud(pcl_cloud);
  zFilter.filter(*pcl_cloud);
}

double CalculateOptimalSACDistanceKinect(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double &sac_distance,
    double weight_factor)
{
  double mean_distance;
  //   pclA::GetMeanPointCloudDistance(pcl_cloud, mean_distance);
  pclA::GetMaxPointCloudDistance(pcl_cloud, mean_distance); /// TODO Changed to max-value
  double optimal_sac_distance = sac_distance * mean_distance;
  optimal_sac_distance *= weight_factor;
  return optimal_sac_distance;
}

void GetMeanPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double &distance, int nrOfPoints)
{
  distance = 0.;
  int p[nrOfPoints];
  for (int i = 0; i < nrOfPoints; i++)
    p[i] = (int) (pcl_cloud->points.size() - 1) * i / (nrOfPoints - 1);

  for (int i = 0; i < nrOfPoints; i++)
    distance += sqrt(
        pow((double) pcl_cloud->points[p[i]].x, 2) + pow((double) pcl_cloud->points[p[i]].y, 2) + pow(
            (double) pcl_cloud->points[p[i]].z, 2));
  distance /= nrOfPoints;
}

void GetMaxPointCloudDistance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, double &distance, int nrOfPoints)
{
  distance = 0.;
  int counter = pcl_cloud->points.size() / nrOfPoints;
  for (size_t i = 0; i < pcl_cloud->points.size(); i += counter)
    if (pcl_cloud->points[i].z > distance)
      distance = pcl_cloud->points[i].z;
}

void SubstituteNanValues(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  for (unsigned idx = 0; idx < pcl_cloud->points.size(); idx++) {
    if (pcl_cloud->points[idx].x == FLT_MAX) {
      pcl_cloud->points[idx].x = NAN;
      pcl_cloud->points[idx].y = NAN;
      pcl_cloud->points[idx].z = NAN;
    }
  }
}

void Dilation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dst,
    double fx, double fy, double cx, double cy, int valid_nghbr)
{
  if (src->width < 2 || src->height < 2)
    printf("PCLUtils::Dilation: Warning: Maybe unordered source point cloud.\n");

  dst.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  dst->header = src->header;
  dst->width = src->width;
  dst->height = src->height;
  dst->is_dense = src->is_dense;
  dst->points = src->points;

  int pos = 0;
  for (unsigned v = 1; v < src->height - 1; v++) {
    for (unsigned u = 1; u < src->width - 1; u++) {
      pos = v * src->width + u;
      if (src->points[pos].z != src->points[pos].z) {
        // Check the four neighbors
        double mean = 0;
        int nr_valid = 0;
        pcl::PointXYZRGB values[4];
        bool isValid[4] = { false, false, false, false };
        if (src->points[pos - src->width].z == src->points[pos - src->width].z && src->points[pos - src->width].z != 0.) {
          values[0] = src->points[pos - src->width];
          mean += values[0].z;
          nr_valid++;
          isValid[0] = true;
          values[0].rgb = src->points[pos - src->width].rgb;
        }
        if (src->points[pos + src->width].z == src->points[pos + src->width].z && src->points[pos + src->width].z != 0.) {
          values[1] = src->points[pos + src->width];
          mean += values[1].z;
          nr_valid++;
          isValid[1] = true;
          values[0].rgb = src->points[pos + src->width].rgb;
        }
        if (src->points[pos - 1].z == src->points[pos - 1].z && src->points[pos - 1].z != 0.) {
          values[2] = src->points[pos - 1];
          mean += values[2].z;
          nr_valid++;
          isValid[2] = true;
          values[0].rgb = src->points[pos - 1].rgb;
        }
        if (src->points[pos + 1].z == src->points[pos + 1].z && src->points[pos + 1].z != 0.) {
          values[3] = src->points[pos + 1];
          mean += values[3].z;
          nr_valid++;
          isValid[3] = true;
          values[0].rgb = src->points[pos + 1].rgb;
        }
        if (nr_valid >= valid_nghbr) {
          mean /= nr_valid;
          int nr_pos = 0;
          int nr_neg = 0;
          int is_pos[4] = { 0, 0, 0, 0 };
          if (isValid[0]) {
            if (values[0].z > mean) {
              nr_pos++;
              is_pos[0] = 1;
            } else {
              nr_neg++;
              is_pos[0] = -1;
            }
          }
          if (isValid[1]) {
            if (values[1].z > mean && isValid[1]) {
              nr_pos++;
              is_pos[1] = 1;
            } else {
              nr_neg++;
              is_pos[1] = -1;
            }
          }
          if (isValid[2]) {
            if (values[2].z > mean && isValid[2]) {
              nr_pos++;
              is_pos[2] = 1;
            } else {
              nr_neg++;
              is_pos[2] = -1;
            }
          }
          if (isValid[3]) {
            if (values[3].z > mean && isValid[3]) {
              nr_pos++;
              is_pos[3] = 1;
            } else {
              nr_neg++;
              is_pos[3] = -1;
            }
          }

          bool done = false;
          if (nr_pos >= nr_neg) {
            for (unsigned i = 0; i < 4; i++)
              if (is_pos[i] == 1 && !done) {
                double camRay_x, camRay_y;
                camRay_x = (float) (u - cx) / fx;
                camRay_y = (float) (v - cy) / fy;
                values[i].x = camRay_x * values[i].z;
                values[i].y = camRay_y * values[i].z;
                done = true;
                dst->points[pos] = values[i];
              }
          } else
            for (unsigned i = 0; i < 4; i++)
              if (is_pos[i] == -1 && !done) {
                double camRay_x, camRay_y;
                camRay_x = (float) (u - cx) / fx;
                camRay_y = (float) (v - cy) / fy;
                values[i].x = camRay_x * values[i].z;
                values[i].y = camRay_y * values[i].z;
                done = true;
                dst->points[pos] = values[i];
              }
        }
      }
    }
  }
}

void Anno2Image(std::vector<int> anno, int max_anno, int width, int height, cv::Mat_<cv::Vec3b> &anno_show)
{
  if (width * height != (int) anno.size()) {
    printf("PCLUtils::Anno2Image: Warning: Image size mismatch (w-h: %u-%u and size: %lu. Abort.\n", width, height,
        anno.size());
    return;
  }

  RGBValue color[max_anno + 1];
  for (int i = 0; i <= max_anno; i++)
    color[i].float_value = GetRandomColor();

  anno_show = cv::Mat_<cv::Vec3b>(height, width);
  for (int row = 0; row < height; row++) {
    for (int col = 0; col < width; col++) {
      if (anno[row * width + col] != -1 && anno[row * width + col] != 0) {
        cv::Vec3b &p = anno_show.at<cv::Vec3b> (row, col);
        p[0] = color[anno[row * width + col]].r;
        p[1] = color[anno[row * width + col]].g;
        p[2] = color[anno[row * width + col]].b;
      } else {
        cv::Vec3b &p = anno_show.at<cv::Vec3b> (row, col);
        p[0] = 255;
        p[1] = 255;
        p[2] = 255;
      }
    }
  }
}

void ProjectPC2Model(const int model, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud,
    const pcl::PointIndices::Ptr &_idxs, const pcl::ModelCoefficients::Ptr &_mc)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ProjectInliers<pcl::PointXYZRGB> proj;
  proj.setModelType(model);
  proj.setInputCloud(_pcl_cloud);
  proj.setIndices(_idxs);
  proj.setModelCoefficients(_mc);
  proj.filter(*cloud);
  for (unsigned i = 0; i < cloud->points.size(); i++)
    _pcl_cloud->points[_idxs->indices[i]] = cloud->points[i];
  //   _pcl_cloud->points = cloud->points;
}

void NormalSpaceSampling(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr _in,
                         pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_out,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals,
                         std::vector< int > &indices)
{
  if(_in.get() == 0) {
    printf("[PCLUtils::NormalSpaceSampling] Error: Input cloud is uninitialized. Abort.\n");
    return;
  }
  
  pcl::NormalSpaceSampling<pcl::PointXYZRGB, pcl::Normal> nss;
  nss.setInputCloud(_in);
  nss.setNormals(normals);
  nss.setSample(307200);
  nss.setBins(100, 100, 100);
//   nss.setKeepOrganized(true);
//   nss.setModelType();
  nss.filter(*_out);    
}

void ClipDepthImage(pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pcl_cloud)
{
  for(unsigned row=0; row<_pcl_cloud->height; row++) {
    int idx = row*_pcl_cloud->width;
    _pcl_cloud->points[idx].x = NAN;
    _pcl_cloud->points[idx].y = NAN;
    _pcl_cloud->points[idx].z = NAN;
  }
  for(unsigned col=0; col<_pcl_cloud->width; col++) {
    int idx = (_pcl_cloud->height-1)*_pcl_cloud->width + col;
    _pcl_cloud->points[idx].x = NAN;
    _pcl_cloud->points[idx].y = NAN;
    _pcl_cloud->points[idx].z = NAN;
  }
}

} // namespace pclA

