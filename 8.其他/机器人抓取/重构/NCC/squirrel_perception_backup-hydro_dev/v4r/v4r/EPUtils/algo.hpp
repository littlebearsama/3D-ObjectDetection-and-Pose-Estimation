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


#ifndef EPALGO_H
#define EPALGO_H

#include "headers.hpp"

namespace EPUtils
{
//ep:begin: revision at 17-07-2014
void filterGaussian(cv::Mat &input, cv::Mat &output, cv::Mat &mask);
void buildDepthPyramid(cv::Mat &image, std::vector<cv::Mat> &pyramid, cv::Mat &mask, unsigned int levelNumber);
void createPointCloudPyramid(std::vector<cv::Mat> &pyramidX, std::vector<cv::Mat> &pyramidY, std::vector<cv::Mat> &pyramidZ, 
			     std::vector<cv::Mat> &pyramidIndices, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pyramidCloud);
void createNormalPyramid(std::vector<cv::Mat> &pyramidNx, std::vector<cv::Mat> &pyramidNy, std::vector<cv::Mat> &pyramidNz, std::vector<cv::Mat> &pyramidIndices, std::vector<pcl::PointCloud<pcl::Normal>::Ptr > &pyramidNormal);
void createIndicesPyramid(std::vector<cv::Mat> &pyramidIndices, std::vector<pcl::PointIndices::Ptr> &pyramidIndiceSets);
void upscaleImage(cv::Mat &input, cv::Mat &output);
void downscaleImage(cv::Mat &input, cv::Mat &output, unsigned int width, unsigned int height);
void scaleImage(std::vector<cv::Mat> &inputPyramid, cv::Mat &input, cv::Mat &output, int inLevel, int outLevel);
//ep:end: revision at 17-07-2014
  
/**
 * checks if point in the polygon
 * */
bool inPoly(std::vector<cv::Point> &poly, cv::Point p);
/**
 * creates polygon map
 * */
void buildPolygonMap(cv::Mat &polygonMap, std::vector<std::vector<cv::Point> > &polygons);
/**
 * builds contour map
 * */
void buildCountourMap(cv::Mat &polygonMap, std::vector<std::vector<cv::Point> > &polygons,
                      cv::Scalar color = cv::Scalar(255,255,255));
/**
 * adds noise to the image
 * */
void addNoise(cv::Mat &image, cv::Mat &nImage, cv::RNG &rng,float min, float max);
/**
 * returns number of the point in the cammulative distribution for 
 * given x value
 * */
long commulativeFunctionArgValue(float x, std::vector<float> &A);
/**
 * calculates normal pdf of the image given mean and stddev
 * */
void normPDF(cv::Mat &mat, float mean, float stddev, cv::Mat &res);
float normPDF(float x, float mean, float stddev);
float normPDF(std::vector<float> x, std::vector<float> mean, cv::Mat stddev);
/**
 * normalizes distribution
 * */
void normalizeDist(std::vector<float> &dist);
/**
 * calculates mean
 * */
float getMean(std::vector<float> dist, float total_num = 1);
/**
 * calculates std
 * */
float getStd(std::vector<float> dist, float mean, float total_num = 1);
/**
 * creates contours from masks
 * */
void createContoursFromMasks(std::vector<cv::Mat> &masks, std::vector<std::vector<cv::Point> > &contours);
/**
 * checks transitions from 0 to 1 in the neighbourhood
 * */
uchar Num01Transitions(cv::Mat s, int j, int i);
/**
 * removes extra connections according to M-connectivity
 * */
void MConnectivity(cv::Mat &s, uchar *element);
/**
 * extracts skeleton
 * */
void Skeleton(cv::Mat a, cv::Mat &s);
/**
 * calculates distance between two points using eucledian normal
 * */
float calculateDistance(cv::Point center, cv::Point point);
/**
 * calculates gaussian distance
 * */
float calculateDistance(cv::Point center, cv::Point point, float sigma);
/**
 * calculate object center
 * */
void calculateObjectCenter(cv::Mat mask, cv::Point &center);
void calculateObjectCenter(std::vector<cv::Point> contour, cv::Mat mask, cv::Point &center);
/**
 * calculate neigbors in 2D
 * */
void get2DNeighbors(const cv::Mat &patches, cv::Mat &neighbors, int patchesNumber);
/**
 * calculate neigbors in 3D
 * */
#ifndef NOT_USE_PCL

template <typename T>
void get3DNeighbors(const cv::Mat &patches, cv::Mat &neighbors, int patchesNumber, 
                    typename pcl::PointCloud<T>::Ptr cloud, double z_max_dist)
{
  neighbors = cv::Mat_<bool>(patchesNumber,patchesNumber);
  neighbors.setTo(false);
  
  int width = patches.cols;
  int height = patches.rows;
  
  //@ep TODO: uncomment?
//   int dr[4] = {-1,-1, 0, 1};
//   int dc[4] = { 0,-1,-1,-1};
  
  int dr[4] = {-1,0,-1};
  int dc[4] = { 0,-1,-1};
  
  for(int r = 1; r < patches.rows-1; r++) 
  {
    for(int c = 1; c < patches.cols-1; c++) 
    {
      // if the patch exist
      if(patches.at<int>(r,c) != -1) 
      {
	
	int patchIdx =  patches.at<int>(r,c);
	
	//@ep: why we did not use 1,-1 shift???
	for(int i = 0; i < 3; ++i) //@ep: TODO 3->4
	{
	
	  int nr = r + dr[i];
	  int nc = c + dc[i];
	  
	  int currentPatchIdx =  patches.at<int>(nr,nc);
	  if(currentPatchIdx == -1)
	    continue;
	  
	  if(patchIdx != currentPatchIdx)
	  {
	    int idx0 =  r*width+c;
            int idx1 = nr*width+nc;
            // @ep:: are we wure that this should be distance only in z direction, and not Euclidean distance???
	    double dis = fabs(cloud->points.at(idx0).z - cloud->points.at(idx1).z);
            if( dis < z_max_dist ) 
	    {
              neighbors.at<bool>(currentPatchIdx,patchIdx) = true;
              neighbors.at<bool>(patchIdx,currentPatchIdx) = true;
            }
	  }
	}   
      }
    }
  }
}

#endif
} //namespace EPUtils

#endif // EPALGO_H