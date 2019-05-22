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


#include "convertions.hpp"

namespace EPUtils
{

#ifndef NOT_USE_PCL
  
//
void Depth2PointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr indices, const cv::Mat depth, 
                      const std::vector<float> cameraParametrs, cv::Mat mask, float th)
{
  if( mask.empty() )
  {
    mask = cv::Mat_<uchar>::ones(depth.size());
  }
  
  // check types
  assert(mask.type() == CV_8UC1);
  assert(depth.type() == CV_32FC1);
  assert(mask.size() == depth.size());
  
  float cx, cy, fx, fy;
  fx = cameraParametrs.at(0);
  fy = cameraParametrs.at(1);
  cx = cameraParametrs.at(2);
  cy = cameraParametrs.at(3);

  cloud->points.clear();
  cloud->is_dense = false;
  cloud->width  = depth.cols;
  cloud->height = depth.rows;
  cloud->points.reserve(cloud->width * cloud->height);
  indices->indices.clear();
  
  for(int i = 0; i < depth.rows; ++i)
  {
    for(int j = 0; j < depth.cols; ++j)
    {
      if(mask.at<uchar>(i,j) == 0)
	continue;
      
      float z_ir = depth.at<float>(i,j);
      if(z_ir < th)
      {
        pcl::PointXYZRGB point(std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::quiet_NaN());
        cloud->points.push_back(point);
        continue; 
      }

      float x_ir = ((j - cx) / fx) * z_ir;
      float y_ir = ((i - cy) / fy) * z_ir;

      pcl::PointXYZRGB point(x_ir,y_ir,z_ir);
      cloud->points.push_back(point);
      int idx = i*(depth.cols)+j;
      indices->indices.push_back(idx);
    }
  }
  
  cloud->width  = cloud->points.size();
  cloud->height = 1;
}

//ep:begin: revision at 17-07-2014
void pointCloud_2_depth(cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGBL>::ConstPtr cloud,
                        unsigned int width, unsigned int height, pcl::PointIndices::Ptr indices, float th)
{
  if( (indices->indices.size()) == 0 )
  {
    indices->indices.resize(cloud->size());
    for(unsigned int i = 0; i < cloud->size(); ++i)
    {
      indices->indices.at(i) = i;
    }
  }
  
  // check types
  assert( indices->indices.size() <= cloud->size() );
  assert( (width*height) == cloud->size() );
  assert( cloud->width == width );
  assert( cloud->height == height );
    
  depth = cv::Mat_<float>::zeros(height,width);

  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    
    if(std::isnan(cloud->points.at(idx).x) ||
       std::isnan(cloud->points.at(idx).y) ||
       std::isnan(cloud->points.at(idx).z))
    {
      continue;
    }
    
    float z_ir = cloud->points.at(idx).z;

    if(z_ir < th)
    {
      continue;
    }

    int r = idx / width;
    int c = idx % width;
    depth.at<float>(r,c) = z_ir;
  }
}

void pointCloud_2_depth(cv::Mat &depth, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                        unsigned int width, unsigned int height, pcl::PointIndices::Ptr indices, float th)
{
  if( (indices->indices.size()) == 0 )
  {
    indices->indices.resize(cloud->size());
    for(unsigned int i = 0; i < cloud->size(); ++i)
    {
      indices->indices.at(i) = i;
    }
  }
  
  // check types
  assert( indices->indices.size() <= cloud->size() );
  assert( (width*height) == cloud->size() );
  assert( cloud->width == width );
  assert( cloud->height == height );
    
  depth = cv::Mat_<float>::zeros(height,width);

  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    
    if(std::isnan(cloud->points.at(idx).x) ||
       std::isnan(cloud->points.at(idx).y) ||
       std::isnan(cloud->points.at(idx).z))
    {
      continue;
    }
    
    float z_ir = cloud->points.at(idx).z;

    if(z_ir < th)
    {
      continue;
    }

    int r = idx / width;
    int c = idx % width;
    depth.at<float>(r,c) = z_ir;
  }
}

void pointCloud_2_rgb(cv::Mat &RGB, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud_xyzrgb,
                      unsigned int width, unsigned int height, pcl::PointIndices::Ptr indices)
{
  if( (indices->indices.size()) == 0 )
  {
    indices->indices.resize(cloud_xyzrgb->size());
    for(unsigned int i = 0; i < cloud_xyzrgb->size(); ++i)
    {
      indices->indices.at(i) = i;
    }
  }
  
  // check types
  assert( indices->indices.size() <= cloud_xyzrgb->size() );
  assert( (width*height) == cloud_xyzrgb->size() );
  assert( cloud_xyzrgb->width == width );
  assert( cloud_xyzrgb->height == height );
    
  RGB = cv::Mat::zeros(height,width,CV_8UC3);

  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    
    int r = idx / width;
    int c = idx % width;
    
    RGB.at<uchar>(r,3*c+2) = cloud_xyzrgb->points.at(idx).r;
    RGB.at<uchar>(r,3*c+1) = cloud_xyzrgb->points.at(idx).g;
    RGB.at<uchar>(r,3*c+0) = cloud_xyzrgb->points.at(idx).b;
  }
}

void pointCloud_2_channels(cv::Mat &xchannel, cv::Mat &ychannel, cv::Mat &zchannel, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
                         unsigned int width, unsigned int height, pcl::PointIndices::Ptr indices, float th)
{
  if( (indices->indices.size()) == 0 )
  {
    indices->indices.resize(cloud->size());
    for(unsigned int i = 0; i < cloud->size(); ++i)
    {
      indices->indices.at(i) = i;
    }
  }
  
  // check types
  assert(indices->indices.size() <= cloud->size());
  assert((size_t)(width*height) == cloud->size());
  
  xchannel = cv::Mat_<float>::zeros(height,width);
  ychannel = cv::Mat_<float>::zeros(height,width);
  zchannel = cv::Mat_<float>::zeros(height,width);
  
  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    
    if(std::isnan(cloud->points.at(idx).x) ||
       std::isnan(cloud->points.at(idx).y) ||
       std::isnan(cloud->points.at(idx).z))
    {
      continue;
    }

    float x_ir = cloud->points.at(idx).x;
    float y_ir = cloud->points.at(idx).y;
    float z_ir = cloud->points.at(idx).z;
    
    if(z_ir < th)
    {
      continue;
    }
    
    int r = idx / width;
    int c = idx % width;
    xchannel.at<float>(r,c) = x_ir;
    ychannel.at<float>(r,c) = y_ir;
    zchannel.at<float>(r,c) = z_ir;
  }  
}

void normals_2_channels(cv::Mat &xnormals, cv::Mat &ynormals, cv::Mat &znormals, pcl::PointCloud<pcl::Normal>::ConstPtr normals,
                      unsigned int width, unsigned int height, pcl::PointIndices::Ptr indices)
{
  // check types
  assert(indices->indices.size() == normals->points.size());
  
  xnormals = cv::Mat_<float>::zeros(height,width);
  ynormals = cv::Mat_<float>::zeros(height,width);
  znormals = cv::Mat_<float>::zeros(height,width);
  
  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    
    float x_n = normals->points.at(i).normal[0];
    float y_n = normals->points.at(i).normal[1];
    float z_n = normals->points.at(i).normal[2];
    
    int r = idx / width;
    int c = idx % width;
    xnormals.at<float>(r,c) = x_n;
    ynormals.at<float>(r,c) = y_n;
    znormals.at<float>(r,c) = z_n;
  }  
}

void pointCloud_2_disparity(cv::Mat &disparity, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, 
                          int width, int height, pcl::PointIndices::Ptr indices, float f, float b, float th)
{
  if( (indices->indices.size()) == 0 )
  {
    indices->indices.resize(cloud->size());
    for(unsigned int i = 0; i < cloud->size(); ++i)
    {
      indices->indices.at(i) = i;
    }
  }
  
  // check types
  assert(indices->indices.size() <= cloud->size());
  assert((size_t)(width*height) == cloud->size());
  
  cv::Scalar defaultDisparityValue(255);
  disparity = cv::Mat(height,width,CV_32FC1,defaultDisparityValue);
  
  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    if(std::isnan(cloud->points.at(idx).x) ||
       std::isnan(cloud->points.at(idx).y) ||
       std::isnan(cloud->points.at(idx).z))
    {
      continue;
    }
    
    float z_ir = cloud->points.at(idx).z;
    
    if(z_ir < th)
    {
      continue;
    }

    float disp = b*f/z_ir;
    
    if(disp < 0)
    {
      continue;
    }

    int r = idx / width;
    int c = idx % width;
    
    disparity.at<float>(r,c) = disp;
  }
}
//ep:end: revision at 17-07-2014

//
void indices_2_image(cv::Mat &mask, unsigned int width, unsigned int height, pcl::PointIndices::Ptr indices)
{
  if( (indices->indices.size()) == 0 )
  {
    indices->indices.resize(width*height);
    for(unsigned int i = 0; i < width*height; ++i)
    {
      indices->indices.at(i) = i;
    }
  }
  
  // check types
  assert(indices->indices.size() <= (size_t)(width*height));
  
  mask = cv::Mat_<float>::zeros(height,width);

  for(unsigned int i = 0; i < indices->indices.size(); ++i)
  {
    int idx = indices->indices.at(i);
    int r = idx / width;
    int c = idx % width;
    mask.at<float>(r,c) = 255;
  }
}

//ep:begin: revision at 17-07-2014
void pointCloudXYZimageRGB_2_cloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, 
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, cv::Mat &RGB, unsigned int width, unsigned int height)
{
  // check types
  assert( cloud_xyz->height == (unsigned int)RGB.rows );
  assert( cloud_xyz->width  == (unsigned int)RGB.cols );
  assert( cloud_xyz->height == height );
  assert( cloud_xyz->width  == width );

  cloud_xyzrgb->points.resize(cloud_xyz->points.size());
  cloud_xyzrgb->width = width;
  cloud_xyzrgb->height = height;
  
  for(unsigned int i = 0; i < height; ++i)
  {
    for(unsigned int j = 0; j < width; ++j)
    {
      int position = i*width + j;
      
      cloud_xyzrgb->points.at(position).x = cloud_xyz->points.at(position).x;
      cloud_xyzrgb->points.at(position).y = cloud_xyz->points.at(position).y;
      cloud_xyzrgb->points.at(position).z = cloud_xyz->points.at(position).z;
      
      cloud_xyzrgb->points.at(position).r = RGB.at<uchar>(i,3*j+2);
      cloud_xyzrgb->points.at(position).g = RGB.at<uchar>(i,3*j+1);
      cloud_xyzrgb->points.at(position).b = RGB.at<uchar>(i,3*j+0);
    }
  }
}

void depthRGB_2_cloudXYZRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, const cv::Mat &depth, const cv::Mat &RGB,
                            const std::vector<float> &cameraParametrs, unsigned int width, unsigned int height, float th)
{
  // check types
  assert(RGB.type() == CV_8UC3);
  assert(depth.type() == CV_32FC1);
  assert(RGB.size() == depth.size());
  assert( (unsigned int)depth.rows == height );
  assert( (unsigned int)depth.cols == width );
  
  float cx, cy, fx, fy;
  fx = cameraParametrs.at(0);
  fy = cameraParametrs.at(1);
  cx = cameraParametrs.at(2);
  cy = cameraParametrs.at(3);

  cloud_xyzrgb->points.clear();
  cloud_xyzrgb->is_dense = false;
  cloud_xyzrgb->width = depth.cols;
  cloud_xyzrgb->height = depth.rows;
  cloud_xyzrgb->points.resize (cloud_xyzrgb->width * cloud_xyzrgb->height);
  
  for(unsigned int i = 0; i < height; ++i)
  {
    for(unsigned int j = 0; j < width; ++j)
    {
      float z_ir = depth.at<float>(i,j);
      
      uchar rr = RGB.at<uchar>(i,3*j+2);//(r,3*c+2)
      uchar gg = RGB.at<uchar>(i,3*j+1);
      uchar bb = RGB.at<uchar>(i,3*j+0);
      
      pcl::PointXYZRGB point;
      
      if(z_ir < th)
      {
	point.x = std::numeric_limits<float>::quiet_NaN();
        point.y = std::numeric_limits<float>::quiet_NaN();
        point.z = std::numeric_limits<float>::quiet_NaN();
      }
      else
      {
        float x_ir = ((j - cx) / fx) * z_ir;
        float y_ir = ((i - cy) / fy) * z_ir;
	
	point.x = x_ir;
        point.y = y_ir;
        point.z = z_ir;
      }

      point.r = rr;
      point.g = gg;
      point.b = bb;
      
      int idx = i*width + j;
      cloud_xyzrgb->points.at(idx) = point;
    }
  }
}

void pointCloudXYZRGB_2_cloudXYZimageRGB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, 
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, cv::Mat &RGB, unsigned int width, unsigned int height)
{
  assert( height == cloud_xyzrgb->height );
  assert( width == cloud_xyzrgb->width );
  assert( width*height == cloud_xyzrgb->points.size() );

  //cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud_xyz->points.resize(cloud_xyzrgb->points.size());
  cloud_xyz->width = width;
  cloud_xyz->height = height;
  
  RGB = cv::Mat::zeros(height,width,CV_8UC3);
  
  for (unsigned int i = 0; i < height; i++)
  {
    for (unsigned int j = 0; j < width; j++)
    {
      int position = i * width + j;
      const pcl::PointXYZRGB pt = cloud_xyzrgb->points.at(position);
      
      cloud_xyz->points.at(position).x = pt.x;
      cloud_xyz->points.at(position).y = pt.y;
      cloud_xyz->points.at(position).z = pt.z;
      
      RGB.at<uchar>(i,3*j+2) = pt.r;
      RGB.at<uchar>(i,3*j+1) = pt.g;
      RGB.at<uchar>(i,3*j+0) = pt.b;
    }
  }
}

void pointCloudXYZimageRGBimageL_2_cloudXYZRGBL(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_xyzrgbl, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz,
						cv::Mat &RGB, cv::Mat &L, unsigned int width, unsigned int height)
{
  assert( cloud_xyz->height == (unsigned int)RGB.rows );
  assert( cloud_xyz->width  == (unsigned int)RGB.cols );
  assert( cloud_xyz->height == (unsigned int)L.rows );
  assert( cloud_xyz->width  == (unsigned int)L.cols );
  assert( cloud_xyz->height == height );
  assert( cloud_xyz->width  == width );

  //cloud_xyzrgbl = pcl::PointCloud<pcl::PointXYZRGBL>::Ptr( new pcl::PointCloud<pcl::PointXYZRGBL>() );
  cloud_xyzrgbl->points.resize(cloud_xyz->points.size());
  cloud_xyzrgbl->width = width;
  cloud_xyzrgbl->height = height;
  
  for (unsigned int i = 0; i < height; i++)
  {
    for (unsigned int j = 0; j < width; j++)
    {
      int position = i * width + j;
      
      cloud_xyzrgbl->points.at(position).x = cloud_xyz->points.at(position).x;
      cloud_xyzrgbl->points.at(position).y = cloud_xyz->points.at(position).y;
      cloud_xyzrgbl->points.at(position).z = cloud_xyz->points.at(position).z;
      
      cloud_xyzrgbl->points.at(position).r = RGB.at<uchar>(i,3*j+2);
      cloud_xyzrgbl->points.at(position).g = RGB.at<uchar>(i,3*j+1);
      cloud_xyzrgbl->points.at(position).b = RGB.at<uchar>(i,3*j+0);
      
      cloud_xyzrgbl->points.at(position).label = L.at<uchar>(i,j);
    }
  }
}

void pointCloudXYZRGBL_2_cloudXYZimageRGBlableL(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_xyzrgbl, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, 
						cv::Mat &RGB, cv::Mat &L, unsigned int width, unsigned int height)
{
  assert( height == cloud_xyzrgbl->height );
  assert( width == cloud_xyzrgbl->width );
  assert( width*height == cloud_xyzrgbl->points.size() );

  //cloud_xyz = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
  cloud_xyz->points.resize(cloud_xyzrgbl->points.size());
  cloud_xyz->width = width;
  cloud_xyz->height = height;
  
  RGB = cv::Mat::zeros(height,width,CV_8UC3);
  L   = cv::Mat::zeros(height,width,CV_8UC1);
  
  for (unsigned int i = 0; i < height; i++)
  {
    for (unsigned int j = 0; j < width; j++)
    {
      int position = i * width + j;
      const pcl::PointXYZRGBL pt = cloud_xyzrgbl->points.at(position);
      
      cloud_xyz->points.at(position).x = pt.x;
      cloud_xyz->points.at(position).y = pt.y;
      cloud_xyz->points.at(position).z = pt.z;
      
      RGB.at<uchar>(i,3*j+2) = pt.r;
      RGB.at<uchar>(i,3*j+1) = pt.g;
      RGB.at<uchar>(i,3*j+0) = pt.b;
      
      L.at<uchar>(i,j) = pt.label;
    }
  }
}

void pointCloudXYZRGBimageL_2_cloudXYZRGBL(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_xyzrgbl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb,
					   cv::Mat &L, unsigned int width, unsigned int height)
{
  assert( cloud_xyzrgb->height == (unsigned int)L.rows );
  assert( cloud_xyzrgb->width  == (unsigned int)L.cols );
  assert( cloud_xyzrgb->height == height );
  assert( cloud_xyzrgb->width  == width );

  cloud_xyzrgbl->points.resize(cloud_xyzrgb->points.size());
  cloud_xyzrgbl->width = width;
  cloud_xyzrgbl->height = height;
  
  for (unsigned int i = 0; i < height; i++)
  {
    for (unsigned int j = 0; j < width; j++)
    {
      int position = i * width + j;
      
      cloud_xyzrgbl->points.at(position).x = cloud_xyzrgb->points.at(position).x;
      cloud_xyzrgbl->points.at(position).y = cloud_xyzrgb->points.at(position).y;
      cloud_xyzrgbl->points.at(position).z = cloud_xyzrgb->points.at(position).z;
      
      cloud_xyzrgbl->points.at(position).r = cloud_xyzrgb->points.at(position).r;
      cloud_xyzrgbl->points.at(position).g = cloud_xyzrgb->points.at(position).g;
      cloud_xyzrgbl->points.at(position).b = cloud_xyzrgb->points.at(position).b;
      
      cloud_xyzrgbl->points.at(position).label = L.at<uchar>(i,j);
    }
  }
}

void pointCloudXYZRGBL_2_cloudXYZRGBlableL(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr cloud_xyzrgbl, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb, 
					   cv::Mat &L, unsigned int width, unsigned int height)
{
  assert( height == cloud_xyzrgbl->height );
  assert( width == cloud_xyzrgbl->width );
  assert( width*height == cloud_xyzrgbl->points.size() );

  cloud_xyzrgb->points.resize(cloud_xyzrgbl->points.size());
  cloud_xyzrgb->width = width;
  cloud_xyzrgb->height = height;
  
  L   = cv::Mat::zeros(height,width,CV_8UC1);
  
  for (unsigned int i = 0; i < height; i++)
  {
    for (unsigned int j = 0; j < width; j++)
    {
      int position = i * width + j;
      const pcl::PointXYZRGBL pt = cloud_xyzrgbl->points.at(position);
      
      cloud_xyzrgb->points.at(position).x = pt.x;
      cloud_xyzrgb->points.at(position).y = pt.y;
      cloud_xyzrgb->points.at(position).z = pt.z;
      
      cloud_xyzrgb->points.at(position).r = pt.r;
      cloud_xyzrgb->points.at(position).g = pt.g;
      cloud_xyzrgb->points.at(position).b = pt.b;
      
      L.at<uchar>(i,j) = pt.label;
    }
  }
}
//ep:end: revision at 17-07-2014

//
void binMasksFromClusters(std::vector<cv::Mat> &binMasks, std::vector<pcl::PointIndices::ConstPtr> clusters)
{
  binMasks.clear();
  binMasks.resize(clusters.size());
  for(unsigned int k = 0; k < clusters.size(); ++k)
  {
    binMasks.at(k) = cv::Mat_<uchar>::zeros(480,640);
    for(unsigned int i = 0; i < clusters.at(k)->indices.size(); ++i)
    {
      int idx = clusters.at(k)->indices.at(i);
      int r = idx / 640;
      int c = idx % 640;
      binMasks.at(k).at<uchar>(r,c) = 255;
    }
  }
}

#endif

//
void Disparity2Depth(cv::Mat &depth, const cv::Mat disparity, float f, float b)
{
  depth = cv::Mat::zeros(disparity.size(),CV_32FC1);
  
  for(int i = 0; i < disparity.rows; ++i)
  {
    for(int j = 0; j < disparity.cols; ++j)
    {
      if(disparity.at<float>(i,j) >= 255)
      {
        continue;
      }

      float disp = disparity.at<float>(i,j);
      float z = b*f/disp;
      depth.at<float>(i,j) = z;
    }
  }
}

void FloatMap2UcharMap(cv::Mat &map_u, const cv::Mat map_f)
{
  cv::convertScaleAbs(map_f,map_u,255,0);
}

} //namespace EPUtils