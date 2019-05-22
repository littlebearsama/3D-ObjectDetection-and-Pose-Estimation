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
 * $Id$
 * Johann Prankl, 6.12.2011
 */


#include "ContourDetection.hh"


namespace pclA 
{

using namespace std;

unsigned ContourDetection::idcnt = 0;


/********************** ContourDetection ************************
 * Constructor/Destructor
 */
ContourDetection::ContourDetection(Parameter p)
{
  setParameter(p);
}

ContourDetection::~ContourDetection()
{
}


/************************** PRIVATE ************************/

/**
 * GetContourPoints
 */
void ContourDetection::GetContourPoints(const std::vector<int> &indices, std::vector<int> &contour)
{
  int x, y;

  contour.clear();

  for( unsigned i = 0; i < indices.size(); i++ ) {
    x = X(indices[i]);
    y = Y(indices[i]);

    if( x==0 || idMap[GetIdx(x - 1, y)] != ContourDetection::idcnt )
      contour.push_back(indices[i]);
    else if( x==width-1 || idMap[GetIdx(x + 1, y)] != ContourDetection::idcnt )
      contour.push_back(indices[i]);
    else if( y==0 || idMap[GetIdx(x, y - 1)] != ContourDetection::idcnt )
      contour.push_back(indices[i]);
    else if( y==height-1 || idMap[GetIdx(x, y + 1)] != ContourDetection::idcnt)
      contour.push_back(indices[i]);
  }
}

/**
 * GetNeighbour
 */
unsigned ContourDetection::GetNeighbour(std::vector<unsigned char> &contourMap, unsigned idx, unsigned char tag)
{
  short x = X(idx);
  short y = Y(idx);

  idx = GetIdx(x-1,y);
  if (x>0 && contourMap[idx]==tag)
    return idx;
  idx = GetIdx(x,y+1);
  if (y<height-1 && contourMap[idx]==tag)
    return idx;
  idx = GetIdx(x+1,y);
  if (x < width-1 && contourMap[idx]==tag)
    return idx;
  idx = GetIdx(x,y-1);
  if (y>0 && contourMap[idx]==tag)
    return idx;

  idx = GetIdx(x-1,y-1);
  if (x>0 && y>0 && contourMap[idx]==tag)
    return idx;
  idx = GetIdx(x-1,y+1);
  if (x>0 && y<height-1 && contourMap[idx]==tag)
    return idx;
  idx = GetIdx(x+1,y+1);
  if (x<width-1 && y<height-1 && contourMap[idx]==tag)
    return idx;
  idx = GetIdx(x+1,y-1);
  if (x<width-1 && y>0 && contourMap[idx]==tag)
    return idx;

  return UINT_MAX;
}

/**
 * TraceContour
 */
void ContourDetection::TraceContour(std::vector<int> &in, std::vector<int> &out, unsigned idxStart)
{
  if (in.size()<3)
    return;
  if (in.size()==3)
  {
    out=in;
    return;
  }

  bool end=false;
  unsigned idx0, idx;
  std::vector<int> queue;

  //draw contour
  for (unsigned i=0; i<in.size(); i++)
    contourMap[in[i]] = 1;

  // trace contour
  idx = idx0 = in[idxStart];
  contourMap[idx0] = 3;
  queue.push_back(idx0);
  unsigned z=0;
  do{
    idx = GetNeighbour(contourMap, idx, 1);
    if (idx==UINT_MAX)
    {
      if (queue.size()>0)    // && z<10
      {
        z++;
        idx = queue.back();
        queue.pop_back();     //this deletes the spikes...
      }
      else end=true;
    }
    else
    {
      z=0;
      contourMap[idx]=2;
      queue.push_back(idx);
      if (queue.size()>3 && GetNeighbour(contourMap,idx,3) == idx0)
        end=true;
    }
  }while(!end);

  //reset contourMap
  for (unsigned i=0; i<in.size(); i++)
    contourMap[in[i]] = 0;

  //return contour
  out = queue;
}




/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void ContourDetection::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (!_cloud->isOrganized())
    throw std::runtime_error ("[ContourDetection::setInputCloud] Point cloud must be organized!");

  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;

  idMap.clear();
  idMap.resize(width*height,0);
  ContourDetection::idcnt=0;

  contourMap.clear();
  contourMap.resize(width*height,0);
}

/**
 * setParameter
 */
void ContourDetection::setParameter(Parameter &p)
{
  param = p;
}

/**
 * resize the point cloud
 */
void ContourDetection::compute(const std::vector<int> &in, std::vector<int> &out)
{
  ContourDetection::idcnt++;

  vector<int> indices, tmp;

  if (cloud.get() == 0 || width==0 || height==0)
    throw std::runtime_error ("[ContourDetection::compute] No point cloud available!");

  // set map
  for (unsigned j=0; j<in.size(); j++)
    idMap[in[j]] = ContourDetection::idcnt;

  // detect unordered contour points
  GetContourPoints(in, indices);

  if (param.orderPoints)
  {
    TraceContour(indices, out, 0);
    for (unsigned j=0; j<param.randTrials; j++)
    {
      TraceContour(indices, tmp, rand()%indices.size());
      if (tmp.size() > out.size())
        out = tmp;
    }
  }
  else out = indices;
}


} //-- THE END --

