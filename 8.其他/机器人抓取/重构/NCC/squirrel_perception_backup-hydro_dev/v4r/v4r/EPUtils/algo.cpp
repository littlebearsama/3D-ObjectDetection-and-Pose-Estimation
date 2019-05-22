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


#include "algo.hpp"

#include <Eigen/Dense>

namespace EPUtils
{

void filterGaussian(cv::Mat &input, cv::Mat &output, cv::Mat &mask)
{
  float kernel[5] = {1.0,4.0,6.0,4.0,1.0};
  
  cv::Mat temp = cv::Mat_<float>::zeros(input.rows, input.cols);
  
  for(int i = 0; i < input.rows; ++i)
  { 
    for(int j = 0; j < input.cols; j = j+2)
    {
      if(mask.at<float>(i,j) > 0)
      {
      
	float value = 0;
        float kernel_sum = 0;
      
        for(int k = 0; k < 5; ++k)
        {
	  int jj = j + (k-2);
	  if( (jj >=0) && (jj < input.cols) )
	  {
	    if(mask.at<float>(i,jj) > 0)
	    {
	      value += kernel[k] * input.at<float>(i,jj);
	      kernel_sum += kernel[k];
	    }
	  }
        }
      
        if(kernel_sum > 0)
        {
	  temp.at<float>(i,j) = value / kernel_sum;
        }
      }
    }
  }
  
  cv::Mat temp2 = cv::Mat_<float>::zeros(temp.rows, temp.cols);
  
  for(int i = 0; i < temp.rows; i = i+2)
  { 
    for(int j = 0; j < temp.cols; j = j+2)
    {
      
      if(mask.at<float>(i,j) > 0)
      {
     
	float value = 0;
        float kernel_sum = 0;
	
        for(int k = 0; k < 5; ++k)
        {
	  int ii = i + (k-2);
	  if( (ii >=0) && (ii < temp.rows) )
	  {
	    if(mask.at<float>(ii,j) > 0)
	    {
	      value += kernel[k] * temp.at<float>(ii,j);
	      kernel_sum += kernel[k];
	    }
	  }
        }
      
        if(kernel_sum > 0)
        {
	  temp2.at<float>(i,j) = value / kernel_sum;
        }
      }
    }
  }
  
  int new_width = input.cols / 2;
  int new_height = input.rows / 2;
  
  cv::Mat mask_output = cv::Mat_<float>::zeros(new_height,new_width);
  output = cv::Mat_<float>::zeros(new_height,new_width);
  
  for(int i = 0; i < new_height; ++i)
  {
    for(int j = 0; j < new_width; ++j)
    {
      if(mask.at<float>(2*i,2*j) > 0)
      {
	mask_output.at<float>(i,j) = 1.0;
	output.at<float>(i,j) = temp2.at<float>(2*i,2*j);
      }
    }
  }
  
  mask_output.copyTo(mask);
}
  
void buildDepthPyramid(cv::Mat &image, std::vector<cv::Mat> &pyramid, cv::Mat &mask, unsigned int levelNumber)
{
  pyramid.resize(levelNumber+1);
  image.copyTo(pyramid.at(0));
  
  cv::Mat oldMask;
  mask.copyTo(oldMask);
  
  for(unsigned int i = 1; i <= levelNumber; ++i)
  {
    cv::Mat tempImage;
    
    filterGaussian(pyramid.at(i-1),tempImage,oldMask);    
    tempImage.copyTo(pyramid.at(i));

  }
}

void createPointCloudPyramid(std::vector<cv::Mat> &pyramidX, std::vector<cv::Mat> &pyramidY, std::vector<cv::Mat> &pyramidZ, 
			     std::vector<cv::Mat> &pyramidIndices, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pyramidCloud)
{
  assert( pyramidX.size() == pyramidY.size() );
  assert( pyramidX.size() == pyramidZ.size() );
  assert( pyramidX.size() == pyramidIndices.size() );
  
  unsigned int num_levels = pyramidX.size();
  
  pyramidCloud.resize(num_levels);
  
  for(unsigned int idx = 0; idx < num_levels; ++idx)
  {
    assert( pyramidX.at(idx).rows == pyramidY.at(idx).rows );
    assert( pyramidX.at(idx).cols == pyramidY.at(idx).cols );
    assert( pyramidX.at(idx).rows == pyramidZ.at(idx).rows );
    assert( pyramidX.at(idx).cols == pyramidZ.at(idx).cols );
    assert( pyramidX.at(idx).rows == pyramidIndices.at(idx).rows );
    assert( pyramidX.at(idx).cols == pyramidIndices.at(idx).cols );
    
    pyramidCloud.at(idx) = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>() );
    
    unsigned int cur_height = pyramidX.at(idx).rows;
    unsigned int cur_width = pyramidX.at(idx).cols;
    
    pyramidCloud.at(idx)->points.resize(cur_height*cur_width);
    pyramidCloud.at(idx)->width = cur_width;
    pyramidCloud.at(idx)->height = cur_height;
    
//     cv::imshow("pyramidZ.at(idx)",pyramidZ.at(idx));
//     cv::waitKey(-1);
    
    for(unsigned int i = 0 ; i < cur_height; ++i)
    {
      for(unsigned int j = 0; j < cur_width; ++j)
      {	
	pcl::PointXYZRGB p_cur;
	
	//std::cerr << "(" << pyramidIndices.at(idx).at<float>(i,j) << "-- " << pyramidX.at(idx).at<float>(i,j) << "," << pyramidY.at(idx).at<float>(i,j) << "," << pyramidZ.at(idx).at<float>(i,j) << ") ";
	
        //if(pyramidIndices.at(idx).at<float>(i,j) > 0)
	//{
	  p_cur.x = pyramidX.at(idx).at<float>(i,j);
	  p_cur.y = pyramidY.at(idx).at<float>(i,j);
	  p_cur.z = pyramidZ.at(idx).at<float>(i,j);
	//}
	//else
// 	{
// 	  p_cur.x = std::numeric_limits<float>::quiet_NaN();
// 	  p_cur.y = std::numeric_limits<float>::quiet_NaN();
// 	  p_cur.z = std::numeric_limits<float>::quiet_NaN();
// 	}
	
	int p_idx = i*cur_width + j;
	pyramidCloud.at(idx)->points.at(p_idx) = p_cur;
      }
    }
    
    //std::cerr <<  std::endl << std::endl;
  
//   cv::imshow("output",output);
//   cv::waitKey(-1);
    
  }
}

void createNormalPyramid(std::vector<cv::Mat> &pyramidNx, std::vector<cv::Mat> &pyramidNy, std::vector<cv::Mat> &pyramidNz, std::vector<cv::Mat> &pyramidIndices, std::vector<pcl::PointCloud<pcl::Normal>::Ptr > &pyramidNormal)
{
  assert( pyramidNx.size() == pyramidNy.size() );
  assert( pyramidNx.size() == pyramidNz.size() );
  assert( pyramidIndices.size() == pyramidNz.size() );
  
  unsigned int num_levels = pyramidNx.size();
  
  pyramidNormal.resize(num_levels);
  
  for(unsigned int idx = 0; idx < num_levels; ++idx)
  {
    assert( pyramidNx.at(idx).rows == pyramidNy.at(idx).rows );
    assert( pyramidNx.at(idx).cols == pyramidNy.at(idx).cols );
    assert( pyramidNx.at(idx).rows == pyramidNz.at(idx).rows );
    assert( pyramidNx.at(idx).cols == pyramidNz.at(idx).cols );
    assert( pyramidNx.at(idx).rows == pyramidIndices.at(idx).rows );
    assert( pyramidNx.at(idx).cols == pyramidIndices.at(idx).cols );
    
    pyramidNormal.at(idx) = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>() );
    
    unsigned int cur_height = pyramidNx.at(idx).rows;
    unsigned int cur_width = pyramidNx.at(idx).cols;
    
    pyramidNormal.at(idx)->points.clear();
    
    for(unsigned int i = 0 ; i < cur_height; ++i)
    {
      for(unsigned int j = 0; j < cur_width; ++j)
      {	
        if(pyramidIndices.at(idx).at<float>(i,j) > 0)
	{
	  pcl::Normal p_cur;
	  p_cur.normal[0] = pyramidNx.at(idx).at<float>(i,j);
	  p_cur.normal[1] = pyramidNy.at(idx).at<float>(i,j);
	  p_cur.normal[2] = pyramidNz.at(idx).at<float>(i,j);
	  
	  pyramidNormal.at(idx)->points.push_back(p_cur);
	}
      }
    }
    
    pyramidNormal.at(idx)->width = pyramidNormal.at(idx)->points.size();
    pyramidNormal.at(idx)->height = 1;
    
  }
}

void createIndicesPyramid(std::vector<cv::Mat> &pyramidIndices, std::vector<pcl::PointIndices::Ptr> &pyramidIndiceSets)
{
  unsigned int num_levels = pyramidIndices.size();
  
  pyramidIndiceSets.resize(num_levels);
  
  for(unsigned int idx = 0; idx < num_levels; ++idx)
  {
    pyramidIndiceSets.at(idx) = pcl::PointIndices::Ptr(new pcl::PointIndices() );
    
    unsigned int cur_height = pyramidIndices.at(idx).rows;
    unsigned int cur_width = pyramidIndices.at(idx).cols;
    
    pyramidIndiceSets.at(idx)->indices.clear();
    
    for(unsigned int i = 0 ; i < cur_height; ++i)
    {
      for(unsigned int j = 0; j < cur_width; ++j)
      {	
        if(pyramidIndices.at(idx).at<float>(i,j) > 0)
	{
	  int p_idx = i*cur_width + j;
	  pyramidIndiceSets.at(idx)->indices.push_back(p_idx);
	}
      }
    }
    
  }
}

void upscaleImage(cv::Mat &input, cv::Mat &output, unsigned int width, unsigned int height)
{
  assert( width >= (unsigned int)(2*input.cols) );
  assert( height >= (unsigned int)(2*input.rows) );
 
  output = cv::Mat_<float>::zeros(height, width);
  
  for(unsigned int i = 0; i < (unsigned int)input.rows; ++i)
  {
    for(unsigned int j = 0; j < (unsigned int)input.cols; ++j)
    {
      output.at<float>(2*i,2*j) = input.at<float>(i,j);
    }
  }
  
  //cv::imshow("output",output);
  //cv::waitKey(-1);
  
  for(unsigned int i = 0; i < (unsigned int)output.rows; i=i+2)
  {
    for(unsigned int j = 1; j < (unsigned int)output.cols-1; j=j+2)
    {
      output.at<float>(i,j) = ( output.at<float>(i,j-1) + output.at<float>(i,j+1) ) / 2.0;
    }
  }
  for(unsigned int i = 1; i < (unsigned int)output.rows-1; i=i+2)
  {
    for(unsigned int j = 0; j < (unsigned int)output.cols; j=j+2)
    {
      output.at<float>(i,j) = ( output.at<float>(i-1,j) + output.at<float>(i+1,j) ) / 2.0;
    }
  }
  for(unsigned int i = 1; i < (unsigned int)output.rows-1; i=i+2)
  {
    for(unsigned int j = 1; j < (unsigned int)output.cols-1; j=j+2)
    {
      output.at<float>(i,j) = ( output.at<float>(i-1,j-1) + output.at<float>(i-1,j+1) + output.at<float>(i+1,j-1) + output.at<float>(i+1,j+1) ) / 4.0;
    }
  }
}

void downscaleImage(cv::Mat &input, cv::Mat &output, unsigned int width, unsigned int height)
{
  assert( 2*width <= (unsigned int)(input.cols) );
  assert( 2*height <= (unsigned int)(input.rows) );
  
  output = cv::Mat_<float>::zeros(height, width);
  
  for(unsigned int i = 0; i < (unsigned int)output.rows; ++i)
  {
    for(unsigned int j = 0; j < (unsigned int)output.cols; ++j)
    {
      output.at<float>(i,j) = input.at<float>(2*i,2*j);
    }
  }
}

void scaleImage(std::vector<cv::Mat> &inputPyramid, cv::Mat &input, cv::Mat &output, int inLevel, int outLevel)
{
  assert(input.cols == inputPyramid.at(inLevel).cols);
  assert(input.rows == inputPyramid.at(inLevel).rows);
  
  if( inLevel < outLevel )
  {
    input.copyTo(output);
    for(int l = inLevel+1; l <= outLevel; ++l)
    {
      cv::Mat temp;
      downscaleImage(output,temp,inputPyramid.at(l).cols,inputPyramid.at(l).rows);
      temp.copyTo(output);
    }
  }
  else if ( inLevel > outLevel )
  {
    input.copyTo(output);
    for(int l = inLevel-1; l >= outLevel; --l)
    {
      cv::Mat temp;
      upscaleImage(output,temp,inputPyramid.at(l).cols,inputPyramid.at(l).rows);
      temp.copyTo(output);
    }
  }
  else
  {
    input.copyTo(output);
  }
}

bool inPoly(std::vector<cv::Point> &poly, cv::Point p)
{
  cv::Point newPoint;
  cv::Point oldPoint;
  cv::Point p1, p2;

  bool inside = false;

  if (poly.size() < 3)
  {
    return(false);
  }


  oldPoint = poly.at(poly.size()-1);
  for(unsigned int i = 0 ; i < poly.size(); i++)
  {
    newPoint = poly.at(i);
    if(newPoint.y > oldPoint.y)
    {
      p1 = oldPoint;
      p2 = newPoint;
    }
    else
    {
      p1 = newPoint;
      p2 = oldPoint;
    }

    if((newPoint.y < p.y) == (p.y <= oldPoint.y)          /* edge "open" at one end */
      && ((long)p.x-(long)p1.x)*(long)(p2.y-p1.y) < ((long)p2.x-(long)p1.x)*(long)(p.y-p1.y))
    {
      inside = !inside;
    }
    oldPoint = newPoint;
  }
  return(inside);
}

void buildPolygonMap(cv::Mat &polygonMap, std::vector<std::vector<cv::Point> > &polygons)
{
  for(int i = 0; i < polygonMap.rows; ++i)
  {
    for(int j = 0; j < polygonMap.cols; ++j)
    {
      unsigned int k = 0;
      while((polygonMap.at<uchar>(i,j) == 0) && (k < polygons.size()))
      {
        if(inPoly(polygons.at(k),cv::Point(j,i)))
          polygonMap.at<uchar>(i,j) = k+1;
        k += 1;
      }
    }
  }
}

void buildCountourMap(cv::Mat &polygonMap, std::vector<std::vector<cv::Point> > &polygons, cv::Scalar color)
{
  for(unsigned int i = 0; i < polygons.size(); ++i)
  {
    for(unsigned int j = 0; j < polygons.at(i).size(); ++j)
    {
      cv::line(polygonMap,polygons.at(i).at(j),polygons.at(i).at((j+1)%polygons.at(i).size()),color,2);
    }
  }
}

float normPDF(float x, float mean, float stddev)
{
  float val = exp(-(x-mean)*(x-mean)/(2*stddev*stddev));
  val /= sqrt(2*3.14)*stddev;
  return val;
}

float normPDF(std::vector<float> x, std::vector<float> mean, cv::Mat stddev)
{
  int dim = mean.size();
  
  EIGEN_ALIGN16 Eigen::MatrixXf _stddev = Eigen::MatrixXf::Zero(dim,dim);
  EIGEN_ALIGN16 Eigen::VectorXf _x = Eigen::VectorXf::Zero(dim);
  
  for(int i = 0; i < dim; ++i)
  {
    _x[i] = x.at(i) - mean.at(i);
    for(int j = 0; j < dim; ++j)
    {
      _stddev(i,j) = stddev.at<float>(i,j);
    }
  }
  
  //std::cerr << "_x.transpose() = " << _x.transpose() << std::endl;
  //std::cerr << "_stddev = " << _stddev << std::endl;
  //std::cerr << "_x = " << _x << std::endl;
  
  float value = (_x.transpose())*_stddev*_x;
  value /= -2;
  value = exp(value);
  
  float det = _stddev.determinant();
  
  value /= sqrt(pow(2*3.14,dim)*det);
  
  return(value);
  
}

void addNoise(cv::Mat &image, cv::Mat &nImage, cv::RNG &rng,float min, float max)
{
  nImage = cv::Mat_<float>::zeros(image.rows, image.cols);
  for(int i = 0; i < image.rows; ++i)
  {
    for(int j = 0; j < image.cols; ++j)
    {
      float val = image.at<float>(i,j) + rng.uniform(min,max);
      nImage.at<float>(i,j) = (val < 0 ? 0 : (val > 1 ? 1 : val));
    }
  }
}

void normPDF(cv::Mat &mat, float mean, float stddev, cv::Mat &res)
{
  res = cv::Mat_<float>::zeros(mat.rows,mat.cols);
  res = mat - mean;
  res = res.mul(res);
  float b = - 1.0f / (2 * stddev * stddev);
  res = res * b;

  cv::exp(res,res);
  float a = 1.0f/(stddev*sqrt(2*M_PI));
  res = res * a;
}

void normalizeDist(std::vector<float> &dist)
{
  float total_sum = 0;
  for(unsigned int i = 0; i < dist.size(); ++i)
  {
    total_sum += dist.at(i);
  }
  
  for(unsigned int i = 0; i < dist.size(); ++i)
  {
    dist.at(i) /= total_sum;
  }
}

float getMean(std::vector<float> dist, float total_num)
{
  float mean = 0;
  for(unsigned int i = 0; i < dist.size(); ++i)
  {
    mean += (dist.at(i)/total_num)*i;
  }
  return mean;
}

float getStd(std::vector<float> dist, float mean, float total_num)
{
  float stdDev = 0;
  
  for(unsigned int i = 0; i < dist.size(); ++i)
  {
    stdDev += (i-mean)*(i-mean)*(dist.at(i)/total_num);
  }
  
  stdDev = sqrt(stdDev);
  
  return stdDev;
}

long commulativeFunctionArgValue(float x, std::vector<float> &A)
{
  long min = 0;
  long max = A.size()-1;
  while(max!=min+1)
  {
    long mid = (min+max)/2;
    if(x <= A.at(mid))
    {
      max = mid;
    }
    else
    {
      min = mid; 
    }
  }
  
  if(A.at(min) >= x)
    return(min);
  else
    return(max);
}

void createContoursFromMasks(std::vector<cv::Mat> &masks, std::vector<std::vector<cv::Point> > &contours)
{
  // to extract contours
  contours.clear();
  contours.resize(masks.size());

  for(unsigned int i = 0; i < masks.size(); ++i)
  {
    std::vector<std::vector<cv::Point> > temp;
    cv::Mat temp_image;
    masks.at(i).copyTo(temp_image);
    cv::findContours(temp_image,temp,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
    if(temp.size())
    {
      long totalPointsNum = 0;
      for(unsigned int j = 0; j < temp.size(); ++j)
      {
	totalPointsNum += temp.at(j).size();
      }
      contours.at(i).resize(totalPointsNum);
      totalPointsNum = 0;
      for(unsigned int j = 0; j < temp.size(); ++j)
      {
	for(unsigned int k = 0; k < temp.at(j).size(); ++k)
	{
	  contours.at(i).at(totalPointsNum) = temp.at(j).at(k);
	  totalPointsNum += 1;
	}
      }
    }
  }
}

uchar Num01Transitions(cv::Mat s, int j, int i)
{

  uchar p2 = s.at<uchar>(j-1,i);
  uchar p3 = s.at<uchar>(j-1,i+1);
  uchar p4 = s.at<uchar>(j,i+1);
  uchar p5 = s.at<uchar>(j+1,i+1);
  uchar p6 = s.at<uchar>(j+1,i);
  uchar p7 = s.at<uchar>(j+1,i-1);
  uchar p8 = s.at<uchar>(j,i-1);
  uchar p9 = s.at<uchar>(j-1,i-1);

  uchar Nt = 0;
  
  if((p3-p2) == 1)
    Nt++;
  if((p4-p3) == 1)
    Nt++;
  if((p5-p4) == 1)
    Nt++;
  if((p6-p5) == 1)
    Nt++;
  if((p7-p6) == 1)
    Nt++;
  if((p8-p7) == 1)
    Nt++;
  if((p9-p8) == 1)
    Nt++;
  if((p2-p9) == 1)
    Nt++;
  
  return Nt;
}

void MConnectivity(cv::Mat &s, uchar *element)
{
  for(int i = 1; i < s.rows-1; ++i)
  {
    for(int j = 1; j < s.cols-1; ++j)
    {
      if(s.at<uchar>(i,j) > 0)
      {
	//s.at<uchar>(i,j) = 0;
	
	bool remove = true;
	for(int p = 0; p < 8; ++p)
        {
          int new_x = j + dx8[p];
          int new_y = i + dy8[p];
      
          uchar value = s.at<uchar>(new_y,new_x);
	  
	  if(element[p] < 2)
	  {
	    if(value != element[p])
	    {
	      remove = false;
	    }
	  }
        }
        
	if(remove)
	{
	  s.at<uchar>(i,j) = 0;
	}
      }
    }
  }
}

void Skeleton(cv::Mat a, cv::Mat &s)
{
  int width  = a.cols;
  int height = a.rows;
  
  a.copyTo(s);
  
  cv::Scalar prevsum(0);

  while(true)
  {

    cv::Mat m = cv::Mat_<uchar>::ones(height,width);
    
    for(int j = 1; j < height-1; ++j)
    {
      for(int i = 1; i < width-1; ++i)
      {
        if (s.at<uchar>(j, i) == 1)
	{
	  uchar p2 = s.at<uchar>(j-1,i);
	  uchar p3 = s.at<uchar>(j-1,i+1);
	  uchar p4 = s.at<uchar>(j,i+1);
	  uchar p5 = s.at<uchar>(j+1,i+1);
	  uchar p6 = s.at<uchar>(j+1,i);
	  uchar p7 = s.at<uchar>(j+1,i-1);
	  uchar p8 = s.at<uchar>(j,i-1);
	  uchar p9 = s.at<uchar>(j-1,i-1);
	  
	  uchar condA = p2+p3+p4+p5+p6+p7+p8+p9;
          uchar condB = Num01Transitions(s,j,i);
          uchar condC = p2 * p4 * p6;
          uchar condD = p4 * p6 * p8;

          if ((condA >= 2) && (condA <= 6) && (condB == 1) && (condC == 0) && (condD == 0))
             m.at<uchar>(j, i) = 0;
	}

      }
    }

    for(int j = 1; j < height-1; ++j)
    {
      for(int i = 1; i < width-1; ++i)
      {
	s.at<uchar>(j,i) = s.at<uchar>(j,i)*m.at<uchar>(j,i);
      }
    }

    m = cv::Mat_<uchar>::ones(height,width);
    for(int j = 1; j < height-1; ++j)
    {
      for(int i = 1; i < width-1; ++i)
      {

        if (s.at<uchar>(j, i) == 1)
	{
          uchar p2 = s.at<uchar>(j-1,i);
	  uchar p3 = s.at<uchar>(j-1,i+1);
	  uchar p4 = s.at<uchar>(j,i+1);
	  uchar p5 = s.at<uchar>(j+1,i+1);
	  uchar p6 = s.at<uchar>(j+1,i);
	  uchar p7 = s.at<uchar>(j+1,i-1);
	  uchar p8 = s.at<uchar>(j,i-1);
	  uchar p9 = s.at<uchar>(j-1,i-1);
	  
	  uchar condA = p2+p3+p4+p5+p6+p7+p8+p9;
          uchar condB = Num01Transitions(s,j,i);
          uchar condC = p2 * p4 * p8;
          uchar condD = p2 * p6 * p8;
                
          if ((condA >= 2) && (condA <= 6) && (condB == 1) && (condC == 0) && (condD == 0))
            m.at<uchar>(j, i) = 0;
	}
      }
    }

    for(int j = 1; j < height-1; ++j)
    {
      for(int i = 1; i < width-1; ++i)
      {
	s.at<uchar>(j,i) = s.at<uchar>(j,i)*m.at<uchar>(j,i);
      }
    }

    cv::Scalar newsum = cv::sum(s);

    if (newsum(0) == prevsum(0))
      break;
    
    prevsum = newsum;

  }
  
  prevsum = cv::sum(s);
  
  uchar e1[8] = {2,1,2,1,2,0,0,0};
  uchar e2[8] = {2,1,2,0,0,0,2,1};
  uchar e3[8] = {0,0,2,1,2,1,2,0};
  uchar e4[8] = {2,0,0,0,2,1,2,1};
  
  while(true)
  {
    MConnectivity(s,e1);
    MConnectivity(s,e2);
    MConnectivity(s,e3);
    MConnectivity(s,e4);
    
    cv::Scalar newsum = cv::sum(s);

    if (newsum(0) == prevsum(0))
      break;
    
    prevsum = newsum;
  }
}

float calculateDistance(cv::Point center, cv::Point point)
{
  float distance = sqrt((center.x-point.x)*(center.x-point.x) + (center.y-point.y)*(center.y-point.y));
  return(distance);
}

float calculateDistance(cv::Point center, cv::Point point, float sigma)
{
  float distance = (center.x-point.x)*(center.x-point.x) + (center.y-point.y)*(center.y-point.y);
  distance = exp(-distance/(2*sigma*sigma));
  return(distance);
}

void calculateObjectCenter(cv::Mat mask, cv::Point &center)
{
  assert(mask.type() == CV_8UC1);
  
  float x_cen = 0;
  float y_cen = 0;
  cv::Scalar area = cv::sum(mask);
  
  for(int i = 0; i < mask.rows; ++i)
  {
    for(int j = 0; j < mask.cols; ++j)
    {
      uchar value = mask.at<uchar>(i,j);
      if(value > 0)
      {
	x_cen = x_cen + ((float)j)/area(0);
        y_cen = y_cen + ((float)i)/area(0);
      }
    }
  }
  
  if(mask.at<uchar>(y_cen,x_cen) == 0)
  {
    // search for closes 4-neighbour point
    std::list<cv::Point> points;
    points.push_back(cv::Point(x_cen,y_cen));
    cv::Mat used = cv::Mat_<uchar>::zeros(mask.rows,mask.cols);
    used.at<uchar>(y_cen,x_cen) = 1;
    while(points.size())
    {
      cv::Point p = points.front();
      points.pop_front();
      if(mask.at<uchar>(p.y,p.x) > 0)
      {
        x_cen = p.x;
	y_cen = p.y;
        break;
      }
      
      for(int i = 0; i < 8; ++i)
      {
        int new_x = p.x + dx8[i];
        int new_y = p.y + dy8[i];
	    
	if((new_x < 0) || (new_y < 0) || (new_x >= used.cols) || (new_y >= used.rows))
	  continue;
	
	if(used.at<uchar>(new_y,new_x) <= 0)
	{
          points.push_back(cv::Point(new_x,new_y));
	  used.at<uchar>(new_y,new_x) = 1;
	}
      }
    }
  }
    
  center.x = x_cen;
  center.y = y_cen;
}

void calculateObjectCenter(std::vector<cv::Point> contour, cv::Mat mask, cv::Point &center)
{
  float x_cen = 0;
  float y_cen = 0;
  
  for(unsigned int i = 0; i < contour.size(); ++i)
  {
    x_cen = x_cen + ((float)contour.at(i).x)/contour.size();
    y_cen = y_cen + ((float)contour.at(i).y)/contour.size();
  }
  
  if(mask.at<uchar>(y_cen,x_cen) == 0)
  {
    // search for closes 4-neighbour point
    std::list<cv::Point> points;
    points.push_back(cv::Point(x_cen,y_cen));
    cv::Mat used = cv::Mat_<uchar>::zeros(mask.rows,mask.cols);
    used.at<uchar>(y_cen,x_cen) = 1;
    while(points.size())
    {
      cv::Point p = points.front();
      points.pop_front();
      if(mask.at<uchar>(p.y,p.x) > 0)
      {
        x_cen = p.x;
	y_cen = p.y;
        break;
      }
      
      for(int i = 0; i < 8; ++i)
      {
        int new_x = p.x + dx8[i];
        int new_y = p.y + dy8[i];
	    
	if((new_x < 0) || (new_y < 0) || (new_x >= used.cols) || (new_y >= used.rows))
	  continue;
	
	if(used.at<uchar>(new_y,new_x) <= 0)
	{
          points.push_back(cv::Point(new_x,new_y));
	  used.at<uchar>(new_y,new_x) = 1;
	}
      }
    }
  }
    
  center.x = x_cen;
  center.y = y_cen;
}

void get2DNeighbors(const cv::Mat &patches, cv::Mat &neighbors, int patchesNumber)
{
  neighbors = cv::Mat_<bool>(patchesNumber,patchesNumber);
  neighbors.setTo(false);
  
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
	    neighbors.at<bool>(currentPatchIdx,patchIdx) = true;
            neighbors.at<bool>(patchIdx,currentPatchIdx) = true;
	  }
	}   
      }
    }
  }
}

} //namespace EPUtils