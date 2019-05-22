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


#include <opencv2/opencv.hpp>

#include "Symmetry3DMap.hpp"

namespace AttentionModule {

// boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
//     pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
// {
//   // --------------------------------------------------------
//   // -----Open 3D viewer and add point cloud and normals-----
//   // --------------------------------------------------------
//   boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
//   viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 1, 0.05, "normals");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   return (viewer);
// }
  
pcl::PointXYZRGB operator+(const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2) 
{
  pcl::PointXYZRGB p;
  p.x = p1.x + p2.x;
  p.y = p1.y + p2.y;
  p.z = p1.z + p2.z;
  return(p);
}

pcl::Normal operator+(const pcl::Normal n1, const pcl::Normal n2) 
{
  pcl::Normal n;
  n.normal[0] = n1.normal[0] + n2.normal[0];
  n.normal[1] = n1.normal[1] + n2.normal[1];
  n.normal[2] = n1.normal[2] + n2.normal[2];
  return(n);
}

pcl::Normal PointsPair2Vector(const pcl::PointXYZRGB p1, const pcl::PointXYZRGB p2) 
{
  pcl::Normal vect;
  vect.normal[0] = p1.x - p2.x;
  vect.normal[1] = p1.y - p2.y;
  vect.normal[2] = p1.z - p2.z;
  
  return(vect);
}

float Distance2PlaneSigned(pcl::Normal vect, pcl::Normal norm)
{
  float dist = vect.normal[0]*norm.normal[0] + vect.normal[1]*norm.normal[1] + vect.normal[2]*norm.normal[2];
  return(dist);
}

Symmetry3DMap::Symmetry3DMap():
BaseMap()
{
  reset();
}

Symmetry3DMap::~Symmetry3DMap()
{
}

void Symmetry3DMap::reset()
{
  BaseMap::reset();
  
  R = 7;
  
  mapName = "Symmetry3DMap";
}

void Symmetry3DMap::print()
{
  BaseMap::print();
  printf("[%s]: R                  = %d\n",mapName.c_str(),R);
}

int Symmetry3DMap::checkParameters()
{
  if(!haveCloud)
  {
    printf("[ERROR]: %s: Seems like there is no cloud.\n",mapName.c_str());
    return(AM_POINTCLOUD);
  }
  
  if(!haveIndices)
  {
    printf("[ERROR]: %s: Seems like there are no indices.\n",mapName.c_str());
    return(AM_POINTCLOUD);
  }
  
  if(!haveNormals)
  {
    printf("[ERROR]: %s: Seems like there are no normals.\n",mapName.c_str());
    return(AM_POINTCLOUD);
  }
  
  if(indices->indices.size() != normals->points.size())
  {
    printf("[ERROR]: %s: Seems like there is different number of indices and normals.\n",mapName.c_str());
    return(AM_DIFFERENTSIZES);
  }
  
  if( (width == 0) || (height == 0) )
  {
    printf("[ERROR]: %s: Seems like image size is wrong.\n",mapName.c_str());
    return(AM_IMAGE);
  }
  
  if(!haveMask)
    mask = cv::Mat_<uchar>::ones(height,width);
  
  if((mask.cols != width) || (mask.rows != height))
  {
    mask = cv::Mat_<uchar>::ones(height,width);
  }
  
  return(AM_OK);
}

void Symmetry3DMap::setR(int R_)
{
  R = R_;
  calculated = false;
  printf("[INFO]: %s: R: %d.\n",mapName.c_str(),R);
}

int Symmetry3DMap::getR()
{
  return(R);
}
  
void Symmetry3DMap::createLookUpMap(pcl::PointIndices::Ptr indices_cur, unsigned int width_cur, unsigned int height_cur, cv::Mat &lookupMap)
{
  // create normals lookup map
  lookupMap = cv::Mat_<int>::zeros(height_cur,width_cur);
  lookupMap = lookupMap - 1;
  for(unsigned int pi = 0; pi < indices_cur->indices.size(); ++pi)
  {
    int index = indices_cur->indices.at(pi);
    
    int r = index / width_cur;
    int c = index % width_cur;
    
    lookupMap.at<int>(r,c) = pi;
  }
}

int Symmetry3DMap::calculate()
{
  
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation started.\n",mapName.c_str());
  
  symmetry3DMap(cloud,normals,indices,width,height,R,map,filter_size);
  
  //cv::blur(map,map,cv::Size(filter_size,filter_size));

  refineMap();
  
  EPUtils::normalize(map,normalization_type);
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  calculated = true;
  printf("[INFO]: %s: Computation succeed.\n",mapName.c_str());

  return(AM_OK);
}

void Symmetry3DMap::symmetry3DMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cur, pcl::PointCloud<pcl::Normal>::Ptr normals_cur, pcl::PointIndices::Ptr indices_cur, 
				  int image_width, int image_height, int R_cur, cv::Mat &map_cur, int filter_size_)
{
  // create normals lookup map
  cv::Mat lookupTableNormals = cv::Mat_<int>::zeros(image_height,image_width);
  
  createLookUpMap(indices_cur,image_width,image_height,lookupTableNormals);
  
  std::vector<cv::Point> shifts;
  
  // create pairs of points
  for(int rr = 0; rr < R_cur/2; ++rr)
  {
    for(int cc = 0; cc < R_cur/2; ++cc)
    {
      int distr = R_cur/2 - rr;
      int distc = R_cur/2 - cc;
      
      shifts.push_back(cv::Point(-distc,-distr));
      shifts.push_back(cv::Point( distc, distr));
      shifts.push_back(cv::Point( distc,-distr));
      shifts.push_back(cv::Point(-distc, distr));
    }
  }
  
  map_cur = cv::Mat_<float>::zeros(image_height,image_width);
  
  #pragma omp parallel for
  for(unsigned int idx = 0; idx < indices_cur->indices.size(); ++idx)
  {
    int rr = indices_cur->indices.at(idx) / image_width;
    int cc = indices_cur->indices.at(idx) % image_width;
    
    
    if( (rr < R_cur/2) || (rr >= image_height - R_cur/2) || (cc < R_cur/2) || (cc >= image_width - R_cur/2) )
    {
      continue;
    }
  
    pcl::PointCloud<pcl::Normal>::Ptr small_normals(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr small_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    for(unsigned int pi = 0; pi < shifts.size(); ++pi)
    {
      int r1 = rr + shifts.at(pi).y;
      int c1 = cc + shifts.at(pi).x;
     
      int index = lookupTableNormals.at<int>(r1,c1); //number of the indece
    
      if(index >= 0)
      {
        small_cloud->points.push_back(cloud_cur->points.at(indices_cur->indices.at(index)));
        small_normals->points.push_back(normals_cur->points.at(index));
      }
    }

    small_cloud->width = small_cloud->points.size();
    small_cloud->height = 1;

    if((small_cloud->points.size() <= 0) || (small_normals->points.size() <= 0))
      continue;
    
    std::vector<pcl::Normal> axis;
    EPUtils::principleAxis(small_normals,axis);

    std::vector<float> W;
    W.resize(axis.size());

    for(unsigned int axis_num = 0; axis_num < axis.size(); ++axis_num)
    {  
      W.at(axis_num) = 0;
  
      // create plane
      pcl::Normal plane_normal;
      plane_normal = axis.at(axis_num);
      plane_normal = EPUtils::normalize(plane_normal);

      pcl::PointXYZRGB point0 = cloud_cur->points.at(indices_cur->indices.at(idx));
     
      float a = plane_normal.normal[0];
      float b = plane_normal.normal[1];
      float c = plane_normal.normal[2];
      float d = -(a*point0.x + b*point0.y + c*point0.z);
 
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
      coefficients->values.resize(4);
      coefficients->values.at(0) = a;
      coefficients->values.at(1) = b;
      coefficients->values.at(2) = c;
      coefficients->values.at(3) = d;
      
//       boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
//       viewer = normalsVis(small_cloud,small_normals);
//       viewer->addPlane(*coefficients);
//       while (!viewer->wasStopped ())
//       {
//         viewer->spinOnce (100);
//         boost::this_thread::sleep (boost::posix_time::microseconds (100000));
//       }
    
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_projected (new pcl::PointCloud<pcl::PointXYZRGB>);
      std::vector<float> distances;
      pcl::PointIndices::Ptr small_indices(new pcl::PointIndices());
      EPUtils::ProjectPointsOnThePlane(coefficients,small_cloud,points_projected,distances,small_indices,false);

      MiddlePoint leftPoint, rightPoint;
  
      for(unsigned int pi = 0; pi < small_cloud->size(); ++pi)
      {
        pcl::PointXYZRGB point_pi = small_cloud->points.at(pi);
       
        pcl::Normal pip0 = PointsPair2Vector(point_pi,point0);
       
        float dist_to_the_plane = Distance2PlaneSigned(pip0,plane_normal);
	
	if(dist_to_the_plane > 0)
        {
          leftPoint.num += 1;
          leftPoint.distance += distances.at(pi);
    
          leftPoint.normal.normal[0] += small_normals->points.at(pi).normal[0];
          leftPoint.normal.normal[1] += small_normals->points.at(pi).normal[1];
          leftPoint.normal.normal[2] += small_normals->points.at(pi).normal[2];
     
          leftPoint.point.x += point_pi.x;
          leftPoint.point.y += point_pi.y;
          leftPoint.point.z += point_pi.z;
        }
        else if(dist_to_the_plane < 0)
        {
          rightPoint.num += 1;
          rightPoint.distance += distances.at(pi);
   
          rightPoint.normal.normal[0] += small_normals->points.at(pi).normal[0];
          rightPoint.normal.normal[1] += small_normals->points.at(pi).normal[1];
          rightPoint.normal.normal[2] += small_normals->points.at(pi).normal[2];
   
          rightPoint.point.x += point_pi.x;
          rightPoint.point.y += point_pi.y;
          rightPoint.point.z += point_pi.z;
        }
      }

      if((leftPoint.num > 0) && (rightPoint.num > 0))
      {
        leftPoint.distance /= leftPoint.num;
        rightPoint.distance /= rightPoint.num;
 
        float Wi = rightPoint.distance - leftPoint.distance;
        Wi = (Wi > 0 ? Wi : -Wi);
    
        leftPoint.normal.normal[0] /= leftPoint.num;
        leftPoint.normal.normal[1] /= leftPoint.num;
        leftPoint.normal.normal[2] /= leftPoint.num;
     
        rightPoint.normal.normal[0] /= rightPoint.num;
        rightPoint.normal.normal[1] /= rightPoint.num;
        rightPoint.normal.normal[2] /= rightPoint.num;
    
        leftPoint.point.x /= leftPoint.num;
        leftPoint.point.y /= leftPoint.num;
        leftPoint.point.z /= leftPoint.num;
     
        rightPoint.point.x /= rightPoint.num;
        rightPoint.point.y /= rightPoint.num;
        rightPoint.point.z /= rightPoint.num;
       
        pcl::Normal lineNormal;
        lineNormal.normal[0] = leftPoint.point.x - rightPoint.point.x;
        lineNormal.normal[1] = leftPoint.point.y - rightPoint.point.y;
        lineNormal.normal[2] = leftPoint.point.z - rightPoint.point.z;
        lineNormal = EPUtils::normalize(lineNormal);
     
        //float Ci;
        //EPUtils::calculateCosine(leftPoint.normal,N,Ci);
        //Ci = Ci > 0 ? Ci : -Ci;
        //Ci = sqrt(1-Ci*Ci);
	pcl::Normal N, N2;
        N = EPUtils::calculatePlaneNormal(leftPoint.normal,rightPoint.normal);
	N = EPUtils::normalize(N);
// 	N2 = EPUtils::crossProduct(N,lineNormal);
// 	float Ci = sqrt(N2.normal[0]*N2.normal[0] + N2.normal[1]*N2.normal[1] + N2.normal[2]*N2.normal[2]);
	float Ci = EPUtils::calculateCosine(lineNormal,N);
        //Ci = Ci > 0 ? Ci : -Ci;
        Ci = sqrt(1-Ci*Ci);
     
        float d=plane_normal.normal[0]*(leftPoint.point.x-point0.x)+plane_normal.normal[1]*(leftPoint.point.y-point0.y)+plane_normal.normal[2]*(leftPoint.point.z-point0.z);
        float cos_left=0, cos_right=0;
	cos_left = EPUtils::calculateCosine(leftPoint.normal,plane_normal);
	cos_right = EPUtils::calculateCosine(rightPoint.normal,plane_normal);
	bool point_is_ok=false;
        if (d<0)
        {
          // cos_left > 90deg && cos_right < 90deg
          point_is_ok = (cos_left<0) && (cos_right>0); 
        }
        else if (d>0)
        {
          // cos_left < 90deg && cos_right > 90deg
          point_is_ok = (cos_left>0) && (cos_right<0);
        }
     
        float cos1, cos2;
        cos1 = EPUtils::calculateCosine(lineNormal,leftPoint.normal);
        cos2 = EPUtils::calculateCosine(lineNormal,rightPoint.normal);
        //cos2 = -cos2;
    
        float alpha1 = acos(cos1);
        float alpha2 = acos(cos2);
    
        float Si;
        Si = (1-cos(alpha1+alpha2))*(1-cos(alpha1-alpha2));
     
        float Di = rightPoint.point.z - leftPoint.point.z;
        Di = Di > 0 ? Di : -Di;
     
        if((leftPoint.point.z > 0) && (rightPoint.point.z > 0) && (rightPoint.distance > 0) && (leftPoint.distance > 0) && point_is_ok)
        {
          W.at(axis_num) = exp(-1000*Wi)*exp(-1000*Di)*Si*Ci;
        }
        
      }
    }

    // calculate number of valid principle planes
    float W_max = 0;
    for(unsigned int axis_num = 0; axis_num < axis.size(); ++axis_num)
    {
      if(W.at(axis_num) > W_max)
      {
        W_max = W.at(axis_num);
      }
    }
  
    map_cur.at<float>(rr,cc) = W_max;

  }
  
  cv::blur(map_cur,map_cur,cv::Size(filter_size_,filter_size_));
  
  double maxValue, minValue;
  cv::minMaxLoc(map_cur,&minValue,&maxValue);
  for(int i = 0; i < map_cur.rows; ++i)
  {
    for(int j = 0; j < map_cur.cols; ++j)
    {
      if(map_cur.at<float>(i,j) < (0.1 * maxValue) )
	map_cur.at<float>(i,j) = 0;
    }
  }
}

int Symmetry3DMap::calculatePyramidSimple()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Simple pyramid started.\n",mapName.c_str());
  
  SimplePyramid::Ptr pyramid( new SimplePyramid() );
  
  pyramid->setStartLevel(0);
  pyramid->setMaxLevel(2);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  pyramid->setCombinationType(combination_type);
  
  pyramid->setCloud(cloud);
  pyramid->setIndices(indices);
  pyramid->setNormals(normals);
  
  pyramid->buildDepthPyramid();
  pyramid->print();

  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  refineMap();
  
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int Symmetry3DMap::calculatePyramidItti()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Itti pyramid started.\n",mapName.c_str());
  
  IttiPyramid::Ptr pyramid( new IttiPyramid() );
  
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  pyramid->setCombinationType(combination_type);
  
  pyramid->setLowestC(2);
  pyramid->setHighestC(4);
  pyramid->setSmallestCS(3);
  pyramid->setLargestCS(4);
  
  pyramid->setChangeSign(false);
  
  pyramid->setCloud(cloud);
  pyramid->setIndices(indices);
  pyramid->setNormals(normals);
  
  pyramid->buildDepthPyramid();
  pyramid->print();

  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  refineMap();
  
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int Symmetry3DMap::calculatePyramidFrintrop()
{
  calculated = false;

  int rt_code = checkParameters();
  if(rt_code != AM_OK)
    return(rt_code);
  
  printf("[INFO]: %s: Computation Frintrop pyramid started.\n",mapName.c_str());
  
  FrintropPyramid::Ptr pyramid( new FrintropPyramid() );
  
  pyramid->setStartLevel(0);
  pyramid->setMaxLevel(4);
  pyramid->setSMLevel(0);
  pyramid->setWidth(width);
  pyramid->setHeight(height);
  pyramid->setNormalizationType(normalization_type);
  pyramid->setCombinationType(combination_type);
  
  std::vector<int> R;
  R.resize(2); R.at(0) = 3; R.at(1) = 7;
  pyramid->setR(R);
  pyramid->setOnSwitch(true);
  
  pyramid->setCloud(cloud);
  pyramid->setIndices(indices);
  pyramid->setNormals(normals);
  
  pyramid->buildDepthPyramid();
  pyramid->print();

  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  cv::Mat map_on;
  map.copyTo(map_on);
  
  float maxIntensityValue = pyramid->getMaxMapValue();
  
  //OFF pyramid
  pyramid->setOnSwitch(false);
  
  rt_code = combinePyramid(pyramid);
  if(rt_code != AM_OK)
    return(rt_code);
  
  cv::Mat map_off;
  map.copyTo(map_off);
  
  maxIntensityValue = std::max(maxIntensityValue,pyramid->getMaxMapValue());
  map = map_on + map_off;
  EPUtils::normalize(map,EPUtils::NT_NONE,maxIntensityValue);
  EPUtils::normalize(map,normalization_type);
  
  refineMap();
  
  //EPUtils::normalize(map,EPUtils::NT_NONE);
  
  calculated = true;
  printf("[INFO]: %s: Pyramid computation succeed.\n",mapName.c_str());
  return(AM_OK);
}

int Symmetry3DMap::combinePyramid(BasePyramid::Ptr pyramid)
{
  for(unsigned int i = pyramid->getStartLevel(); i <= (unsigned int)pyramid->getMaxLevel(); ++i)
  {
    printf("[INFO]: %s: Computating feature map for level %d.\n",mapName.c_str(),i);

    // start creating parameters
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud;
    if(!pyramid->getCloud(i,current_cloud))
    {
      printf("[ERROR]: Something went wrong! Can't get cloud for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    pcl::PointCloud<pcl::Normal>::Ptr current_normals;
    if(!pyramid->getNormals(i,current_normals))
    {
      printf("[ERROR]: Something went wrong! Can't get normals for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    pcl::PointIndices::Ptr current_indices;
    if(!pyramid->getIndices(i,current_indices))
    {
      printf("[ERROR]: Something went wrong! Can't get indices for level %d!\n",i);
      return(AM_CUSTOM);
    }    
    
    int current_width = pyramid->getWidth(i);
    if(current_width <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get width for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    int current_height = pyramid->getHeight(i);
    if(current_height <= 0)
    {
      printf("[ERROR]: Something went wrong! Can't get height for level %d!\n",i);
      return(AM_CUSTOM);
    }
    
    cv::Mat current_map;
    symmetry3DMap(current_cloud,current_normals,current_indices,current_width,current_height,R,current_map,filter_size);
    
//     cv::Mat current_map_temp;
//     current_map.copyTo(current_map_temp);
//     cv::imshow("current_map_temp",current_map_temp);
//     EPUtils::normalize(current_map_temp,normalization_type);
//     cv::waitKey(-1);
    
    if(!pyramid->setFeatureMap(i,current_map))
    {
     printf("[ERROR]: Something went wrong! Can't set feature map for level %d!\n",i);
     return(AM_CUSTOM);
    }
    
    printf("[INFO]: %s: Feature map at level %d is set.\n",mapName.c_str(),i);
  }
  // combine saliency maps
  pyramid->combinePyramid();
  
  if(!pyramid->getMap(map))
  {
    printf("[ERROR]: Something went wrong! Can't get saliency map from the pyramid!\n");
    return(AM_CUSTOM);
  }
  
  return(AM_OK);
}

} //namespace AttentionModule
