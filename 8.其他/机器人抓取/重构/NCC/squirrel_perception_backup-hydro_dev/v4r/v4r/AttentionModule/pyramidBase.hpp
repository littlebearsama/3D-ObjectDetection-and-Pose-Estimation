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


#ifndef PYRAMID_BASE_HPP
#define PYRAMID_BASE_HPP

#include "headers.hpp"

namespace AttentionModule
{
  
class BasePyramid
{
public:
  BasePyramid();
  typedef boost::shared_ptr<BasePyramid> Ptr;

  void setStartLevel(int start_level_);
  int getStartLevel();

  void setMaxLevel(int max_level_);
  int getMaxLevel();

  void setSMLevel(int sm_level_);
  int getSMLevel();

  void setNormalizationType(int normalization_type_);
  int getNormalizationType();

  void setWidth(int width_);
  int getWidth();
  int getWidth(unsigned int level);

  void setHeight(int height_);
  int getHeight();
  int getHeight(unsigned int level);
  
  void setCombinationType(int combination_type_);
  int getCombinationType();

  void setImage(cv::Mat &image_);
  bool getImage(cv::Mat &image_);
  bool getImage(unsigned int level, cv::Mat &image_);
  
  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);
  bool getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);
  bool getCloud(unsigned int level, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);
  
  void setIndices(pcl::PointIndices::Ptr &indices_);
  bool getIndices(pcl::PointIndices::Ptr &indices_);
  bool getIndices(unsigned int level, pcl::PointIndices::Ptr &indices_);
  
  void setNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_);
  bool getNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_);
  bool getNormals(unsigned int level, pcl::PointCloud<pcl::Normal>::Ptr &normals_);

  void setMaxMapValue(float max_map_value_);
  float getMaxMapValue();

  std::vector<cv::Mat> getPyramidImages();
  std::vector<cv::Mat> getPyramidFeatures();
  bool setFeatureMap(unsigned int level, cv::Mat &featureMap_);

  bool getMap(cv::Mat &map_);

  virtual void print();
  virtual void reset();

  virtual int checkParameters(bool isDepth = false);

  int buildPyramid();
  int buildDepthPyramid();
  virtual void combinePyramid(bool standard = false);

protected:
  int                  start_level;
  int                  max_level;
  int                  sm_level;
  int                  normalization_type;
  int                  width;
  int                  height;
  int                  combination_type;

  cv::Mat              image;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointIndices::Ptr indices;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  std::vector<cv::Mat> pyramidImages;
  std::vector<cv::Mat> pyramidX;
  std::vector<cv::Mat> pyramidY;
  std::vector<cv::Mat> pyramidZ;
  std::vector<cv::Mat> pyramidNx;
  std::vector<cv::Mat> pyramidNy;
  std::vector<cv::Mat> pyramidNz;
  std::vector<pcl::PointIndices::Ptr> pyramidIndices;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pyramidCloud;
  std::vector<pcl::PointCloud<pcl::Normal>::Ptr > pyramidNormals;
  
  std::vector<cv::Mat> pyramidFeatures;
  float                max_map_value;
  cv::Mat              map;

  bool calculated;

  bool haveImage;
  bool haveCloud;
  bool haveIndices;
  bool haveNormals;
  bool haveImagePyramid;
  bool haveDepthPyramid;
  bool haveNormalPyramid;
  bool haveIndicePyramid;

  std::string pyramidName;

  virtual void calculate();
  virtual void checkLevels();
  virtual void combineConspicuityMaps(cv::Mat &sm_map, cv::Mat &consp_map);
};

}
#endif //PYRAMID_BASE_HPP