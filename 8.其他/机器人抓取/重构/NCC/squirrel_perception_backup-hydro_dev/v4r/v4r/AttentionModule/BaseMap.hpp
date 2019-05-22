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


#ifndef BASE_MAP_HPP
#define BASE_MAP_HPP

#include "headers.hpp"

#include "pyramidBase.hpp"
#include "pyramidSimple.hpp"
#include "pyramidItti.hpp"
#include "pyramidFrintrop.hpp"

namespace AttentionModule
{

enum PyramidType
{
  SIMPLE_PYRAMID  = 0,
  ITTI_PYRAMID    = 1,
  FRINTROP_PYRAMID,
};
  
class BaseMap
{
public:
  
  BaseMap();
  virtual ~BaseMap();
  
  void setImage(const cv::Mat &image_);
  bool getImage(cv::Mat &image_);
  
  void setCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_);
  bool getCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_);

  void setNormals(pcl::PointCloud<pcl::Normal>::Ptr normals_);
  bool getNormals(pcl::PointCloud<pcl::Normal>::Ptr &normals_);
  
  void setIndices(pcl::PointIndices::Ptr indices_);
  bool getIndices(pcl::PointIndices::Ptr &indices_);
  
  void setMask(const cv::Mat &mask_);
  bool getMask(cv::Mat &mask_);
  
  void setNormalizationType(int normalization_type_);
  int  getNormalizationType();
  
  void setCombinationType(int combination_type_);
  int  getCombinationType();
  
  void setFilterSize(int filter_size_);
  int  getFilterSize();
  
  void setWidth(int width_);
  int  getWidth();
  
  void setHeight(int height_);
  int  getHeight();
  
  void setRefine(bool refine_);
  bool getRefine();

  void setMapName(std::string mapName_);
  std::string getMapName();
  
  virtual int calculate() = 0;
  virtual int calculatePyramid(int pyramidType = SIMPLE_PYRAMID);
  
  bool getMap(cv::Mat &map_);
  
  virtual void reset();
  virtual void print();

protected:

/**
 * parameters for base map
 * */

  cv::Mat                                  mask;//
  cv::Mat                                  image;//
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr   cloud;//
  pcl::PointCloud<pcl::Normal>::Ptr        normals;//
  pcl::PointIndices::Ptr                   indices;//
  int                                      filter_size;//
  int                                      normalization_type;//
  int                                      combination_type;//
  int                                      width;//
  int                                      height;//
  bool                                     calculated;//
  cv::Mat                                  map;//
  bool                                     refine;
  //BasePyramid::Ptr                         pyramid;

  std::string mapName;
  
  bool haveImage;
  bool haveCloud;
  bool haveNormals;
  bool haveIndices;
  bool haveMask;

  virtual int checkParameters();
  
  virtual int calculatePyramidSimple();
  virtual int calculatePyramidItti();
  virtual int calculatePyramidFrintrop();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);
  
  virtual void refineMap();

};

} // namespace AttentionModule

#endif //BASE_MAP_HPP