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
 * Johann Prankl, 2010-12-01
 * prankl@acin.tuwien.ac.at
 */


#include "CCLabeling.hh"


namespace pclA
{


/********************** CCLabeling ************************
 * Constructor/Destructor
 */
CCLabeling::CCLabeling(Parameter _param)
 : param(_param)
{
}

CCLabeling::~CCLabeling()
{
}


/**
 * disjoint-set find root
 */
Label* CCLabeling::Find(Label *x)
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
ushort CCLabeling::Union(Label *x, Label* y)
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


/*************************** public *****************************************
 * Operate
 * @param cluster_size <label,size_of_cluster>
 */
void CCLabeling::Operate(const cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, std::vector<unsigned> &cluster_size)
{
  //struct timespec start1, end1, start2, end2;
  //clock_gettime(CLOCK_REALTIME, &start1);
  //clock_gettime(CLOCK_REALTIME, &start2);
  cluster_size.clear();
  labels = cv::Mat_<ushort>(cloud.rows, cloud.cols);
  labels.setTo(0);
  
  std::vector<Label*> setLabels;
  setLabels.push_back(new Label(setLabels.size()));
  float sqrThr = param.thr*param.thr;

  //first pass...
  for (int v = 0; v < cloud.rows; ++v)
  {
    for (int u = 0; u < cloud.cols; ++u)
    {
      const cv::Vec4f &pt = cloud(v,u);
      ushort &la = labels(v,u);

      if (pt[0]==pt[0] && pt[1]==pt[1] && pt[2]==pt[2])     // check for NaN
      {      
        if (u>0)    //check left
        {
          if (SqrDistanceZ(pt, cloud(v,u-1)) < sqrThr)
          {
             la = labels(v,u-1);
          }
        } 
        if (v>0)    //check upper
        {
          if (SqrDistanceZ(pt, cloud(v-1,u)) < sqrThr && labels(v-1,u) != la)
          {
            if (la==0)
            {
              la = labels(v-1,u);
            }
            else
            {
              la = Union(setLabels[la], setLabels[labels(v-1,u)]);
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
  //clock_gettime(CLOCK_REALTIME, &end2);
  //cout<<"Time [s]: "<<P::timespec_diff(&end2, &start2)<<endl;

  // second pass...
  ushort la1, cnt=0;
  std::map<ushort, ushort> smartLabels;           // <source, target> 
  std::map<ushort, ushort>::iterator it;
  std::vector<ushort> minLabels(setLabels.size());  // id look up table
  for (unsigned i=0; i<setLabels.size(); i++)
  {
    la1 = Find(setLabels[i])->id;
    it = smartLabels.find(la1);
    if (it==smartLabels.end())
    {
      smartLabels[la1] = cnt;
      minLabels[i] = cnt;
      cnt++;
      cluster_size.push_back(0);
    }
    else minLabels[i] = it->second;
  }
  
  for (int v = 0; v < labels.rows; ++v)
  {
    for (int u = 0; u < labels.cols; ++u)
    {
      ushort &la = labels(v,u);
      if (la!=0)
      {
        la = minLabels[la];
        cluster_size[la]++;
      }
    }
  }

  for (ushort i=0; i<setLabels.size(); i++)
    delete setLabels[i];

  //clock_gettime(CLOCK_REALTIME, &end1);
  //cout<<"Time [s]: "<<P::timespec_diff(&end1, &start1)<<endl;

}

/**
 * FilterClusterSize
 */
void CCLabeling::FilterClusterSize(cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, std::vector<unsigned> &cluster_size)
{
  if (cloud.size()!=labels.size())
    throw std::runtime_error ("CCLabeling::FilterClusterSize: Point cloud mat and label mat must have equal size!");

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  //filter largest cluster
  for (int v = 0; v < labels.rows; ++v)
  {
    for (int u = 0; u < labels.cols; ++u)
    {
      ushort &la = labels(v,u);
      if (la!=0 && cluster_size[la]<param.minClusterSize)
      {
        la=0;
        cv::Vec4f &pt = cloud(v,u);
        pt[0] = bad_point;
        pt[1] = bad_point;
        pt[2] = bad_point;
      }
    }
  }
}

/**
 * Filter depending on a minimum cluster size and create mask
 */
void CCLabeling::CreateMask(const cv::Mat_<ushort> &labels, const std::vector<unsigned> &cluster_size, cv::Mat_<uchar> &mask)
{
  mask = cv::Mat::zeros(labels.rows, labels.cols, CV_8U);

  //filter largest cluster
  for (int v = 0; v < labels.rows; ++v)
  {
    for (int u = 0; u < labels.cols; ++u)
    {
      const ushort &la = labels(v,u);
      if (la!=0 && cluster_size[la]>=param.minClusterSize)
      {
        mask(v,u) = 255;
      }
    }
  }
}


/**
 * FilterLargestCluster
 */
void CCLabeling::FilterLargestCluster(cv::Mat_<cv::Vec4f> &cloud, cv::Mat_<ushort> &labels, std::vector<unsigned> &cluster_size)
{
  if (cloud.size()!=labels.size())
    throw std::runtime_error ("CCLabeling::FilterLargestCluster: Point cloud mat and label mat must have equal size!");

  ushort maxLabel=0;
  ushort max=0;
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  //find largest cluster
  for (unsigned i=0; i<cluster_size.size(); i++)
  {
    if (max<cluster_size[i])
    {
      max=cluster_size[i];
      maxLabel=i;
    }
  }

  //filter largest cluster
  for (int v = 0; v < labels.rows; ++v)
  {
    for (int u = 0; u < labels.cols; ++u)
    {
      ushort &la = labels(v,u);
      if (la!=0 && la!=maxLabel)
      {
        la=0;
        cv::Vec4f &pt = cloud(v,u);
        pt[0] = pt[1] = pt[2] = bad_point;
      }
    }
  }
}

}

