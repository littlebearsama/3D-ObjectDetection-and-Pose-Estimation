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
 * Johann Prankl, 2012-02-18
 * prankl@acin.tuwien.ac.at
 */


#include "CCEuclideanClustering.hh"


namespace pclA
{


/********************** CCEuclideanClustering ************************
 * Constructor/Destructor
 */
CCEuclideanClustering::CCEuclideanClustering(Parameter _param)
 : param(_param)
{
}

CCEuclideanClustering::~CCEuclideanClustering()
{
}


/**
 * disjoint-set find root
 */
CCEuclideanClustering::Label* CCEuclideanClustering::Find(Label *x)
{
  if (x->parent == x)
  {
    return x;
  }
  else
  {
    x->parent = Find(x->parent);
    return x->parent;
  }
}

/**
 * disjoint-set make union
 */
unsigned short CCEuclideanClustering::Union(Label *x, Label* y)
{
  Label *xRoot = Find(x);
  Label *yRoot = Find(y);

  if (xRoot->rank > yRoot->rank)
  {
    yRoot->parent = xRoot;
  }
  else if (xRoot != yRoot)
  {
    xRoot->parent = yRoot;
    if (xRoot->rank == yRoot->rank)
      yRoot->rank = yRoot->rank+1;
  }

  return (x->id<y->id?x->id:y->id);
}

/**
 * Clustering
 */
void CCEuclideanClustering::Clustering(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<unsigned short> &labels)
{
  labels.header   = cloud.header;
  labels.width    = cloud.width;
  labels.height   = cloud.height;
  labels.is_dense = cloud.is_dense;

  labels.points.clear();
  labels.points.resize(cloud.points.size(),0);
  
  sizeClusters.clear();

  std::vector<Label*> setLabels;
  setLabels.push_back(new Label(setLabels.size()));
  sqrThr = param.thr*param.thr;
  int idx;  

  //first pass...
  for (int v = 0; v < height; ++v)
  {
    for (int u = 0; u < width; ++u)
    {
      idx = GetIdx(u,v);
      pcl::PointXYZRGB &pt = cloud.points[idx];
      unsigned short &la = labels.points[idx];

      if (!IsNaN(pt))     // check for NaN
      {      
        if (u>0)    //check left
        {
          idx = GetIdx(u-1,v);
          if (SqrDistance(pt, cloud.points[idx]) < sqrThr)
          {
             la = labels.points[idx];
          }
        } 
        if (v>0)    //check upper
        {
          idx = GetIdx(u,v-1);
          if (SqrDistance(pt, cloud.points[idx]) < sqrThr && labels.points[idx] != la)
          {
            if (la==0)
            {
              la = labels.points[idx];
            }
            else
            {
              la = Union(setLabels[la], setLabels[labels.points[idx]]);
            }
          }
        }
        if (la==0)  // distance bigger => create new label
        {
          la = setLabels.size();
          setLabels.push_back(new Label(la));
        }
      }
    }
  }

  // second pass...
  unsigned short la1, cnt=0;
  std::map<unsigned short, unsigned short> smartLabels;           // <source, target> 
  std::map<unsigned short, unsigned short>::iterator it;
  std::vector<unsigned short> minLabels(setLabels.size());  // id look up table
  for (unsigned i=0; i<setLabels.size(); i++)
  {
    la1 = Find(setLabels[i])->id;
    it = smartLabels.find(la1);
    if (it==smartLabels.end())
    {
      smartLabels[la1] = cnt;
      minLabels[i] = cnt;
      cnt++;
      sizeClusters.push_back(0);
    }
    else minLabels[i] = it->second;
  }
  
  for (int v = 0; v < height; ++v)
  {
    for (int u = 0; u < width; ++u)
    {
      unsigned short &la = labels(u,v);
      if (la!=0)
      {
        la = minLabels[la];
        sizeClusters[la]++;
      }
    }
  }

  for (unsigned short i=0; i<setLabels.size(); i++)
    delete setLabels[i];
}


/*************************** public *****************************************/
/**
 * setInputCloud
 */
void CCEuclideanClustering::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  if (_cloud.get() == 0)
    throw std::runtime_error ("[CCEuclideanClustering::setInputCloud] No point cloud available!");

  if (!_cloud->isOrganized())
    throw std::runtime_error ("[CCEuclideanClustering::setInputCloud] Point cloud must be organized!");

  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;
}

/**
 * compute
 */
void CCEuclideanClustering::compute()
{
  if (cloud.get() == 0)
    throw std::runtime_error ("[CCEuclideanClustering::compute] No point cloud available!");

  labels.reset(new pcl::PointCloud<unsigned short>() );

  Clustering(*cloud, *labels);
}

/**
 * getLabels
 */
void CCEuclideanClustering::getLabels(pcl::PointCloud<unsigned short>::Ptr &_labels)
{
  _labels = labels;
}

/**
 * getSizeClusters
 */
void CCEuclideanClustering::getSizeClusters(std::vector<unsigned> &_sizeClusters)
{
  _sizeClusters = sizeClusters;
}


}

