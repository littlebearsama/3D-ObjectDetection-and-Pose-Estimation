/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
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
 * @file ContourNormalsDistance.cc
 * @author Richtsfeld, Potapova
 * @date February 2013
 * @version 0.1
 * @brief Calculate normal distance from two contours using a octree.
 */

#include "ContourNormalsDistance.hh"

namespace surface 
{

float CONTOUR_MAX_DISTANCE = 1.15;   // maximum distance compared to min-distance

/********************** ContourNormalsDistance ************************
 * Constructor/Destructor
 */
ContourNormalsDistance::ContourNormalsDistance(Parameter p)
:EPBase()
{
  setParameter(p);

  ClassName = "ContourNormalsDistance";
}

ContourNormalsDistance::~ContourNormalsDistance()
{}



/************************** PRIVATE ************************/

bool ContourNormalsDistance::computeOctree(surface::SurfaceModel::Ptr &in1, surface::SurfaceModel::Ptr &in2,
                                           float &cosDeltaAngle, float &distNormal, float &minDist, float &occlusion)
{
  if(in1->contours.size() == 0 || in2->contours.size() == 0) {
    printf("[ContourNormalsDistance::computeOctree] Warning: No contour available: %u-%u.\n", in1->idx, in2->idx);
    return false;
  }

//   std::cerr << "1" << std::endl;
  
  int contIdx1 = 0;
  for(unsigned int i = 0; i < in1->contours.size(); ++i)
  {
    if(in1->contours.at(i).size() > in1->contours.at(contIdx1).size())
      contIdx1 = i;
  }

//   std::cerr << "2" << std::endl;
  
  int contIdx2 = 0;
  for(unsigned int i = 0; i < in2->contours.size(); ++i)
  {
    if(in2->contours.at(i).size() > in2->contours.at(contIdx2).size())
      contIdx2 = i;
  }

//   std::cerr << "3" << std::endl;
  
  if(in1->contours.at(contIdx1).size() == 0 || in2->contours.at(contIdx2).size() == 0) {
    printf("[ContourNormalsDistance::computeOctree] Warning: Input surface contour is empty: Return false.\n");
    return false;
  }

  distNormal = 0.0f;
  cosDeltaAngle = 0.0f;
  int distNormalCnt = 0;
    
  surface::SurfaceModel::Ptr surf1;   // => octree: Take the one as octree with more contour pixels!!!
  surface::SurfaceModel::Ptr surf2;   // => search points

  if(in1->contours.at(contIdx1).size() > in2->contours.at(contIdx2).size())
  {
    surf1 = in1;
    surf2 = in2;
  }
  else
  {
    surf1 = in2;
    surf2 = in1;

    //swap contour idxs
    int tempContIdx = contIdx1;
    contIdx1 = contIdx2;
    contIdx2 = tempContIdx;
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr octreeCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud,surf1->contours.at(contIdx1),*octreeCloud);
  
  // create octree from octreecloud
  float resolution = 4.0f;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree (resolution);
  octree.setInputCloud(octreeCloud);
  octree.addPointsFromInputCloud();

//   std::cerr << "4" << std::endl;
  
  // find indices of nearest points
  // nearest point indices
  int surf1NP = -1; 
  int surf2NP = -1;
  minDist = 10.0f;
  for(unsigned int i = 0; i < surf2->contours.at(contIdx2).size(); i++)
  {
    pcl::PointXYZRGB searchPoint = cloud->points.at(surf2->contours.at(contIdx2).at(i));
    
    // K-nearest neighbour search
    int K = 1;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
    octree.nearestKSearch(searchPoint,K,pointIdxNKNSearch,pointNKNSquaredDistance);

    if(sqrt(pointNKNSquaredDistance[0]) < minDist)
    {
      minDist = sqrt(pointNKNSquaredDistance[0]);
      surf1NP = pointIdxNKNSearch[0];
      surf2NP = i;
    }
  }

//   std::cerr << "5" << std::endl;
  
  // calculate the occlusion value between the nearest contour points
  occlusion = calculateOclusion(surf1, surf2, contIdx1, contIdx2, surf1NP, surf2NP);
  
  // estimate direction
  bool equal = true;
  int n1l, n1r, n2l, n2r;
  n1l = surf1NP;
  n1r = surf1NP;
  
  n2l = surf2NP;
  n2r = surf2NP;
  float normal, cross;
    
  int maxNr = surf2->contours.at(contIdx2).size()/2;
  while(equal)
  {
    maxNr--;
    n1l--;
    n1r++;
    n2l--;
    n2r++;

    // check validity of points
    if(n1l < 0)
    {
      n1l = surf1->contours.at(contIdx1).size()-1;
    }
    if(n1r >= (int) surf1->contours.at(contIdx1).size())
    {
      n1r = 0;
    }
    if(n2l < 0)
    {
      n2l = surf2->contours.at(contIdx2).size()-1;
    }
    if(n2r >= (int) surf2->contours.at(contIdx2).size())
    {
      n2r = 0;
    }
    
    Eigen::Vector3f dist_normal_l = cloud->points.at(surf1->contours.at(contIdx1).at(n1l)).getVector3fMap() - cloud->points.at(surf2->contours.at(contIdx2).at(n2l)).getVector3fMap();
    Eigen::Vector3f dist_normal_r = cloud->points.at(surf1->contours.at(contIdx1).at(n1r)).getVector3fMap() - cloud->points.at(surf2->contours.at(contIdx2).at(n2r)).getVector3fMap();
    Eigen::Vector3f dist_cross_l  = cloud->points.at(surf1->contours.at(contIdx1).at(n1l)).getVector3fMap() - cloud->points.at(surf2->contours.at(contIdx2).at(n2r)).getVector3fMap();
    Eigen::Vector3f dist_cross_r  = cloud->points.at(surf1->contours.at(contIdx1).at(n1r)).getVector3fMap() - cloud->points.at(surf2->contours.at(contIdx2).at(n2l)).getVector3fMap();
    
    normal = dist_normal_l.norm() + dist_normal_r.norm();
    cross  = dist_cross_l.norm()  + dist_cross_r.norm();
    
    if(fabs(normal - cross) > 0.001 || maxNr < 0)
    {
      equal = false;
    }
  }

//   std::cerr << "6" << std::endl;
  
  // create all relations between neighbors
  int dir;
  if(normal < cross)
  {
    dir = 1;
  }
  else
  {
    dir = -1;
  }

  // left side (with minimum point)
  bool left_run = true;
  maxNr = surf2->contours.at(contIdx2).size()/2;
  //@ep: why are we adding this?
  n1l = surf1NP+=1;
  n2l = surf2NP+=dir;

//   std::cerr << "6.1" << std::endl;
  
  while(left_run)
  {
    maxNr--;
    n1l-=1;
    n2l-=dir;

//     std::cerr << "6.2" << std::endl;
    
    // check validity of points
    if(n1l < 0)
    {
      n1l = surf1->contours.at(contIdx1).size()-1;
    }

//     std::cerr << "6.3" << std::endl;
    
    if(n1l >= (int) surf1->contours.at(contIdx1).size())
    {
      n1l = 0;
    }

//     std::cerr << "6.4" << std::endl;
    
    if(n2l < 0)
    {
      n2l = surf2->contours.at(contIdx2).size()-1;
    }

//     std::cerr << "6.5" << std::endl;
    
    if(n2l >= (int) surf2->contours.at(contIdx2).size())
    {
      n2l = 0;
    }

//     std::cerr << "6.6" << std::endl;
    
    float distance_l = (cloud->points.at(surf1->contours.at(contIdx1).at(n1l)).getVector3fMap() - cloud->points.at(surf2->contours.at(contIdx2).at(n2l)).getVector3fMap()).norm();

    distNormalCnt++;

    Eigen::Vector3d sufr1_normal;
    sufr1_normal[0] = normals->points.at(surf1->contours.at(contIdx1).at(n1l)).normal_x;
    sufr1_normal[1] = normals->points.at(surf1->contours.at(contIdx1).at(n1l)).normal_y;
    sufr1_normal[2] = normals->points.at(surf1->contours.at(contIdx1).at(n1l)).normal_z;

    Eigen::Vector3d sufr2_normal;
    sufr2_normal[0] = normals->points.at(surf2->contours.at(contIdx2).at(n2l)).normal_x;
    sufr2_normal[1] = normals->points.at(surf2->contours.at(contIdx2).at(n2l)).normal_y;
    sufr2_normal[2] = normals->points.at(surf2->contours.at(contIdx2).at(n2l)).normal_z;

    
    Eigen::Vector3f p2p_distance = cloud->points.at(surf2->contours.at(contIdx2).at(n2l)).getVector3fMap() - cloud->points.at(surf1->contours.at(contIdx1).at(n1l)).getVector3fMap();
    //float p2p_normal_projection = p2p_distance.dot(sufr1_normal);
    float p2p_normal_projection = p2p_distance[0]*((sufr1_normal)[0]) + p2p_distance[1]*((sufr1_normal)[1]) + p2p_distance[2]*((sufr1_normal)[2]);
    
//     std::cerr << "6.7" << std::endl;
    
    distNormal += fabs(p2p_normal_projection);
    
//     distNormal += fabs( Plane::NormalPointDist(&cloud->points.at(surf1->contours.at(contIdx1).at(n1l)).x,
//                                                &sufr1_normal[0],
//                                                &cloud->points.at(surf2->contours.at(contIdx2).at(n2l)).x));

//     std::cerr << "6.7.1" << std::endl;
    
    float cosAlpha = sufr1_normal.dot(sufr2_normal);

//     std::cerr << "6.8" << std::endl;
    
    //@ep: why is it 1+ and not abs???
    cosDeltaAngle += (cosAlpha<0 ? 1.+cosAlpha : cosAlpha);    

    if( (distance_l > minDist*CONTOUR_MAX_DISTANCE) || (maxNr < 0) )
    {
      left_run=false;
    }
  } 

//   std::cerr << "7" << std::endl;
  
  // calculate right side relations
  n1r = surf1NP;
  n2r = surf2NP;
  maxNr = surf2->contours.at(contIdx2).size();
  bool right_run = true;
  while(right_run) {
    maxNr--;
    n1r+=1;
    n2r+=dir;
    
    // check validity of points
    if(n1r < 0)
    {
      n1r = surf1->contours.at(contIdx1).size()-1;
    }
    if(n1r >= (int) surf1->contours.at(contIdx1).size())
    {
      n1r = 0;
    }
    if(n2r < 0)
    {
      n2r = surf2->contours.at(contIdx2).size()-1;
    }
    if(n2r >= (int) surf2->contours.at(contIdx2).size())
    {
      n2r = 0;
    }

    float distance_r = (cloud->points.at(surf1->contours.at(contIdx1).at(n1r)).getVector3fMap() - cloud->points.at(surf2->contours.at(contIdx2).at(n2r)).getVector3fMap()).norm();

    distNormalCnt++;

    Eigen::Vector3d sufr1_normal;
    sufr1_normal[0] = normals->points.at(surf1->contours.at(contIdx1).at(n1r)).normal_x;
    sufr1_normal[1] = normals->points.at(surf1->contours.at(contIdx1).at(n1r)).normal_y;
    sufr1_normal[2] = normals->points.at(surf1->contours.at(contIdx1).at(n1r)).normal_z;
    
    Eigen::Vector3d sufr2_normal;
    sufr2_normal[0] = normals->points.at(surf2->contours.at(contIdx2).at(n2r)).normal_x;
    sufr2_normal[1] = normals->points.at(surf2->contours.at(contIdx2).at(n2r)).normal_y;
    sufr2_normal[2] = normals->points.at(surf2->contours.at(contIdx2).at(n2r)).normal_z;
    
    Eigen::Vector3f p2p_distance = cloud->points.at(surf2->contours.at(contIdx2).at(n2r)).getVector3fMap() - cloud->points.at(surf1->contours.at(contIdx1).at(n1r)).getVector3fMap();
//     float p2p_normal_projection = p2p_distance.dot(sufr1_normal);
    float p2p_normal_projection = p2p_distance[0]*((sufr1_normal)[0]) + p2p_distance[1]*((sufr1_normal)[1]) + p2p_distance[2]*((sufr1_normal)[2]);
    
    distNormal += fabs(p2p_normal_projection);
    
//     distNormal += fabs(Plane::NormalPointDist(&cloud->points[surf1->contours[0][n1r]].x, 
//                                               &surf1->normals[n1r][0], 
//                                               &cloud->points[surf2->contours[0][n2r]].x));      

    float cosAlpha = sufr1_normal.dot(sufr2_normal);

    //@ep: why is it 1+ and not abs???
    cosDeltaAngle += (cosAlpha<0? 1.+cosAlpha : cosAlpha);

    if( (distance_r > minDist*CONTOUR_MAX_DISTANCE) || (maxNr < 0) )
    {
      right_run=false;
    }
  } 

//   std::cerr << "8" << std::endl;
  
  if(distNormalCnt == 0)
    printf("[ContourNormalsDistance::computeOctree] Warning: Division by zero?\n");

  distNormal /= (float) distNormalCnt;
  cosDeltaAngle /= (float) distNormalCnt;

  std::cerr << "finish" << std::endl;
  
  return(true);
}


float ContourNormalsDistance::calculateOclusion(surface::SurfaceModel::Ptr &in1, surface::SurfaceModel::Ptr &in2,
                                                int contIdx1, int contIdx2, int surf1NP, int surf2NP)
{
  float occlusion = 0.0f;

  printf("[ContourNormalsDistance::CalculateOclusion] Start: surf: %u-%u / idx: %u-%u\n", in1->idx, in2->idx, surf1NP, surf2NP);

  int pt1idx = in1->contours.at(contIdx1).at(surf1NP);
  int pt2idx = in2->contours.at(contIdx2).at(surf2NP);

  printf("[ContourNormalsDistance::CalculateOclusion] idx: %u-%u\n", pt1idx, pt2idx);

  int x1 = X(pt1idx);
  int y1 = Y(pt1idx);
  int x2 = X(pt2idx);
  int y2 = Y(pt2idx);

  printf("[ContourNormalsDistance::CalculateOclusion] x-y: %u-%u to %u-%u\n", x1, y1, x2, y2);
  
  Eigen::Vector3f p00, p10;
  p00 = cloud->points.at(pt1idx).getVector3fMap();
  p10 = cloud->points.at(pt2idx).getVector3fMap();

  unsigned nr_valid_pts = 0;          // number of valid 3d points (if occlusion nan's are in the point cloud)
  double z_dist_mean = 0.0f;          // mean z-distance of hidden line (gap) to orginal point cloud
  Eigen::Vector3f dir3D;              // direction of 3D line
  std::vector<int> x_2D, y_2D;        // x and y of 2D line
  drawLine(x1,y1,x2,y2,x_2D,y_2D);
  dir3D = (p10-p00).normalized();

  // calculate mean of z-distance from hypothesized 3D line to orignal point cloud data
  if(x_2D.size() == 0)
    return 0.0;
  
  for(unsigned int m = 1; m < x_2D.size()-1; m++)
  {
    int index = getIdx(x_2D.at(m), y_2D.at(m));
    Eigen::Vector3f p3D_org = cloud->points.at(index).getVector3fMap();

    if(!isNaN(p3D_org))
    {
      Eigen::Vector3f p3D = p00 + (dir3D * m) / x_2D.size();
      z_dist_mean += (p3D[2] - p3D_org[2]);
      nr_valid_pts++;
    }
  }
  
  if(nr_valid_pts != 0)
  {
    z_dist_mean /= nr_valid_pts;
    occlusion = z_dist_mean;
  }

printf("Occlusion: %8.8f\n", occlusion);
  return occlusion;
}

//@ep: personally for me I do not like this implementation, it is too much over the head;
//@ep: there are simpler implementations easier to get the first time
void ContourNormalsDistance::drawLine(int x1, int y1, int x2, int y2, std::vector<int> &_x, std::vector<int> &_y)
{
  int dx, dy, inc_x, inc_y, x, y, err;

  dx = x2 - x1;
  dy = y2 - y1;
  if((dx == 0) && (dy == 0))  // line might be clipped to length 0
    return;

  x = x1;
  y = y1;
  
  if(dx >= 0)
  {
    inc_x = 1;
  }
  else
  {
    dx = -dx;
    inc_x = -1;
  }
  
  if(dy >= 0)
  {
    inc_y = 1;
  }
  else
  {
    dy = -dy;
    inc_y = -1;
  }
  // octants 1,4,5,8

  if(dx >= dy)
  {
    // first octant bresenham
    err = -dx/2;
    do
    {
      _x.push_back(x);
      _y.push_back(y);
      err += dy;
      if(err >= 0)
      {
        y += inc_y;
        err -= dx;
        //@ep: I think this is the bug!
        if(x + inc_x != x2)
        {
          // make line dense
          _x.push_back(x);
          _y.push_back(y);
        }
      }
      x += inc_x;
    } while(x != x2);
  }
  // octants 2,3,6,7
  else // dx < dy
  {
    // second octant bresenham
    err = -dy/2;
    do
    {
      _x.push_back(x);
      _y.push_back(y);
      err += dx;
      if(err >= 0)
      {
        x += inc_x;
        err -= dy;
        if(y + inc_y != y2)
        {
          // make line dense
          _x.push_back(x);
          _y.push_back(y);
        }
      }
      y += inc_y;
    } while(y != y2);
  }
}

/************************** PUBLIC *************************/

/**
 * compute
 */
bool ContourNormalsDistance::compute(surface::SurfaceModel::Ptr &in1,
                                     surface::SurfaceModel::Ptr &in2,
                                     float &cosDeltaAngle,
                                     float &distNormal,
                                     float &minDist,
                                     float &occlusion)
{
  if( (!have_cloud) || (!have_normals))
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  return computeOctree(in1,in2,cosDeltaAngle,distNormal,minDist,occlusion);
}

/**
 * setParameter
 */
void ContourNormalsDistance::setParameter(Parameter &p)
{
  param = p;
}



} //-- THE END --

