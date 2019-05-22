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
 * @file Graph.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Create graph from 3D features in KinectCore.
 */

#ifndef GC_GRAPH_H
#define GC_GRAPH_H

#include <vector>
#include <stdio.h>
#include <cstdlib>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "Edge.h"
#include "v4r/SurfaceUtils/Relation.h"
#include "v4r/SurfaceUtils/SurfaceModel.hpp"

namespace gc
{

template<typename T1,typename T2>
extern T1 Dot3(const T1 v1[3], const T2 v2[3]);
  
template<typename T1,typename T2, typename T3>
extern void Add3(const T1 v1[3], const T2 v2[3], T3 r[3]);

/**
 * @brief Class Graph
 */
class Graph
{
private:  
  unsigned nodes;                                               ///< number of surface patches
  std::vector<gc::Edge> edges;                                  ///< edges of the graph (wiht node numbers and probability)

  std::vector<surface::Relation> relations;                     ///< relations between features
  std::vector<surface::SurfaceModel::Ptr> surfaces;                     ///< relations between features
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud;             ///< point cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals;                    ///< Normals of the point cloud
  
  std::vector<int> surfaces_reindex;
  //std::vector<int> surfaces_reindex2;
  
public:
  
private:
  
  inline int GetIdx(short x, short y);
  inline short X(int idx);
  inline short Y(int idx);
  
public:
  Graph();
  Graph(unsigned nrNodes, std::vector<surface::Relation> &rel);
  Graph(std::vector<surface::SurfaceModel::Ptr> &surf, std::vector<surface::Relation> &rel);
  ~Graph();
  
  void BuildFromSVM(std::vector<gc::Edge> &e, unsigned &num_edges, std::vector<int> &surfaces_reindex2);
  void BuildFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_pcl_cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr &_normals,
                           std::vector<gc::Edge> &e, unsigned &num_edges);
};

/*************************** INLINE METHODES **************************/
/** Return index for coordinates x,y **/
inline int Graph::GetIdx(short x, short y)
{
  return y*pcl_cloud->width + x;
}

/** Return x coordinate for index **/
inline short Graph::X(int idx)
{
  return idx%pcl_cloud->width;
}

/** Return y coordinate for index **/
inline short Graph::Y(int idx)
{
  return idx/pcl_cloud->width;
}

}

#endif

