/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef FAAT_PCL_RECOGNITION_GRAPH_GEOMETRIC_CONSISTENCY_IMPL_H_
#define FAAT_PCL_RECOGNITION_GRAPH_GEOMETRIC_CONSISTENCY_IMPL_H_

#include "graph_geometric_consistency.h"
#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/common/io.h>
#include <pcl/common/time.h>
#include <boost/unordered_map.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/biconnected_components.hpp>
#include <boost/graph/prim_minimum_spanning_tree.hpp>
#include <exception>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
gcGraphCorrespSorter (pcl::Correspondence i, pcl::Correspondence j)
{
  return (i.distance < j.distance);
}

struct Vertex
{
  int idx_;
  size_t degree_;

  bool
  operator< (const Vertex & j) const
  {
    if (degree_ == j.degree_)
    {
      return (idx_ < j.idx_);
    }

    return degree_ > j.degree_;
  }
};

struct vertexDegreeSorter
{
  bool
  operator() (const Vertex & i, const Vertex & j) const
  {
    return i < j;
  }
};

template<typename Graph>
  class save_cliques
  {

  public:
    /*save_cliques (std::size_t& max, std::size_t& maximum_clique, std::size_t& n_cliques, std::vector<std::vector<void *> *> & cliquess) :
     min_size (max), maximum (maximum_clique), n_cliques (n_cliques), cliques (cliquess)
     {
     }*/

    save_cliques (std::size_t& max, std::size_t& maximum_clique, std::size_t& n_cliques, std::vector<std::vector<int> *> & cliquess) :
      min_size (max), maximum (maximum_clique), n_cliques (n_cliques), cliques (cliquess)
    {
    }

    template<typename Clique, typename Graph2>
      inline void
      clique (const Clique& c, Graph2& g)
      {

        if (c.size () >= min_size)
        {
          BOOST_USING_STD_MAX();
          maximum = std::max BOOST_PREVENT_MACRO_SUBSTITUTION (maximum, c.size());

          //save clique...
          typename Clique::const_iterator i, end = c.end ();
          //std::vector<void *> * cc = new std::vector<void *> (c.size ());
          std::vector<int> * cc = new std::vector<int> (c.size ());
          cliques.push_back (cc);
          size_t p;
          for (i = c.begin (); i != end; ++i, ++p)
          {
            //cc->at (p) = static_cast<void *> (*i);
            cc->at (p) = (*i);
          }

          n_cliques++;
        }
        else
        {
          return;
        }

        // Simply assert that each vertex in the clique is connected
        // to all others in the clique.
        /*typename Clique::const_iterator i, j, end = c.end();
         for(i = c.begin(); i != end; ++i) {
         for(j = c.begin(); j != end; ++j) {
         if(i != j) {
         BOOST_ASSERT(edge(*i, *j, g).second);
         }
         }
         }*/
      }

    std::size_t& min_size;
    std::size_t& maximum;
    std::size_t& n_cliques;
    //std::vector<std::vector<void *> *> & cliques;
    std::vector<std::vector<int> *> & cliques;
  };

  class FAATPCL_CliquesException: public std::exception
  {
    virtual const char* what() const throw()
    {
      return "My exception happened";
    }
  } myex;

template<typename Graph>
  class Tomita
  {
    typedef std::set<typename boost::graph_traits<Graph>::vertex_descriptor> SetType;
    typedef std::vector<typename boost::graph_traits<Graph>::vertex_descriptor> VectorType;
    std::vector<VectorType *> cliques_found_;
    size_t min_clique_size_;
    typedef boost::unordered_map<typename boost::graph_traits<Graph>::vertex_descriptor, int> MapType;
    MapType used_ntimes_in_cliques_;
    std::vector<SetType> nnbrs;
    float max_time_allowed_;
    pcl::StopWatch time_elapsed_;
    bool max_time_reached_;

    void
    addClique (VectorType & clique)
    {
      if (clique.size () >= min_clique_size_)
      {
        VectorType * vt = new VectorType (clique);
        cliques_found_.push_back (vt);
        for (size_t i = 0; i < clique.size (); i++)
        {
          used_ntimes_in_cliques_[clique[i]]++;
        }

      }
    }

    void
    printSet (SetType & s)
    {
      typename SetType::iterator vertexIt, vertexEnd;
      SetType tmp;

      vertexIt = s.begin ();
      vertexEnd = s.end ();

      for (; vertexIt != vertexEnd; ++vertexIt)
      {
        std::cout << *vertexIt + 1 << " ";
      }

      std::cout << std::endl;
    }
    //_extend(nnbrs,cand,done,clique_so_far,cliques);
    void
    extend (SetType & cand, SetType & done, VectorType & clique_so_far)
    {
      SetType small_cand, pivot_nbrs;
      int maxconn = -1;
      int num_cand = static_cast<int> (cand.size ());

      //iterate over done and compute maximum intersection between candidates and the adjacents of done (nnbrs)
      typename SetType::iterator vertexIt, vertexEnd;
      SetType tmp;

      vertexIt = done.begin ();
      vertexEnd = done.end ();

      for (; vertexIt != vertexEnd; ++vertexIt)
      {
        std::set_intersection (cand.begin (), cand.end (), nnbrs[*vertexIt].begin (), nnbrs[*vertexIt].end (), std::inserter (tmp, tmp.begin ()));

        if (static_cast<int> (tmp.size ()) > maxconn)
        {
          maxconn = static_cast<int> (tmp.size ());
          pivot_nbrs = tmp;
          if (maxconn == num_cand)
          {
            //All possible cliques already found
            return;
          }
        }

        tmp.clear ();
      }

      //same for candidates
      vertexIt = cand.begin ();
      vertexEnd = cand.end ();

      for (; vertexIt != vertexEnd; ++vertexIt)
      {
        std::set_intersection (cand.begin (), cand.end (), nnbrs[*vertexIt].begin (), nnbrs[*vertexIt].end (), std::inserter (tmp, tmp.begin ()));

        if (static_cast<int> (tmp.size ()) > maxconn)
        {
          maxconn = static_cast<int> (tmp.size ());
          pivot_nbrs = tmp;
        }
        tmp.clear ();
      }

      //      std::cout << "cand is:" << std::endl;
      //      printSet(cand);
      //
      //      std::cout << "pivot_nbrs is:" << std::endl;
      //      printSet(pivot_nbrs);

      std::set_difference (cand.begin (), cand.end (), pivot_nbrs.begin (), pivot_nbrs.end (), std::inserter (small_cand, small_cand.begin ()));
      vertexIt = small_cand.begin ();
      vertexEnd = small_cand.end ();

      /*std::cout << "small_cand is:" << std::endl;
       printSet(small_cand);*/

      for (; vertexIt != vertexEnd; ++vertexIt)
      {
        //std::cout << (*vertexIt+1) << std::endl;
        cand.erase (*vertexIt);
        clique_so_far.push_back (*vertexIt);
        SetType new_cand, new_done;
        std::set_intersection (cand.begin (), cand.end (), nnbrs[*vertexIt].begin (), nnbrs[*vertexIt].end (),
                               std::inserter (new_cand, new_cand.begin ()));

        std::set_intersection (done.begin (), done.end (), nnbrs[*vertexIt].begin (), nnbrs[*vertexIt].end (),
                               std::inserter (new_done, new_done.begin ()));

        if (new_done.size () == 0 && new_cand.size () == 0)
        {
          addClique (clique_so_far);
        }
        else if (new_done.size () == 0 && (new_cand.size () == 1))
        {
          if ((clique_so_far.size () + 1) >= min_clique_size_)
          {
            VectorType tt = clique_so_far;
            tt.push_back (*(new_cand.begin ()));
            addClique (tt);
          }
        }
        else
        {
          float t_elapsed = static_cast<float>(time_elapsed_.getTime());
          if(t_elapsed > max_time_allowed_)
          {
              max_time_reached_ = true;

              /*for (size_t p = 0; p < cliques_found_.size (); p++) //ATTENTION!
                delete cliques_found_[p];*/

              return;
          }

          extend (new_cand, new_done, clique_so_far);
        }

        clique_so_far.erase (clique_so_far.begin () + (clique_so_far.size () - 1));
        done.insert (*vertexIt);

      }
    }

  public:

    Tomita (size_t mins = 3)
    {
      min_clique_size_ = mins;
      max_time_allowed_ = std::numeric_limits<float>::infinity();
      max_time_reached_ = false;
    }

    bool
    getMaxTimeReached()
    {
        return max_time_reached_;
    }

    void
    setMaxTimeAllowed(float t)
    {
        max_time_allowed_ = t;
    }

    void
    find_cliques (Graph & G, int num_v)
    {
      SetType cand, done;
      VectorType clique_so_far;
      nnbrs.clear ();
      used_ntimes_in_cliques_.clear ();
      cliques_found_.clear ();
      time_elapsed_.reset();
      max_time_reached_ = false;

      typename boost::graph_traits<Graph>::vertex_iterator vertexIt, vertexEnd;
      boost::tie (vertexIt, vertexEnd) = vertices (G);
      nnbrs.resize (num_v);

      int i = 0;
      for (; vertexIt != vertexEnd; ++vertexIt, ++i)
      {
        typename boost::graph_traits<Graph>::adjacency_iterator vi, vi_end;
        int k = 0;
        for (boost::tie (vi, vi_end) = boost::adjacent_vertices (*vertexIt, G); vi != vi_end; ++vi, ++k)
        {
          nnbrs[i].insert (*vi);
          cand.insert (*vi);
        }

        used_ntimes_in_cliques_[*vertexIt] = 0;
      }

      extend (cand, done, clique_so_far);
    }

    int
    getNumCliquesFound ()
    {
      return cliques_found_.size ();
    }

    void
    getCliques (std::vector<VectorType *> & cliques)
    {
      cliques = cliques_found_;
    }
  };

struct ExtendedClique
{
    std::vector<long unsigned int> * correspondences_;
    float avg_descriptor_distance_;
    float avg_pair_3D_distance_;
    float normalized_clique_size_;
    float avg_pair_3D_distance_unnormalized_;
    float far_away_correspondences_weight_;
};

bool
less_clique_vectors (const std::vector<long unsigned int> * a, const std::vector<long unsigned int> * b)
{
  return a->size () < b->size ();
}

bool
best_clique_vectors (const std::pair<float, std::vector<long unsigned int> *> a,
                     const std::pair<float, std::vector<long unsigned int> *> b)
{
    if(a.second->size() == b.second->size())
    {
        return a.first > b.first;
    }

    return a.second->size () > b.second->size ();
}

bool
best_extended_cliques (const ExtendedClique & a,
                     const ExtendedClique & b)
{
    /*float a_value = static_cast<float>(a.correspondences_->size()) * 0.5f + a.avg_descriptor_distance_ * 0.25f + a.avg_pair_3D_distance_ * 0.25f;
    float b_value = static_cast<float>(b.correspondences_->size()) * 0.5f + b.avg_descriptor_distance_ * 0.25f + b.avg_pair_3D_distance_ * 0.25f;*/

    /*float a_value = a.avg_pair_3D_distance_ * 0.5f + a.avg_descriptor_distance_ * 0.5f;
    float b_value = b.avg_pair_3D_distance_ * 0.5f + b.avg_descriptor_distance_ * 0.5f;*/

    /*float a_value = a.avg_pair_3D_distance_ * 0.5f;
    float b_value = b.avg_pair_3D_distance_ * 0.5f;*/

    /*float a_value = static_cast<float>(a.normalized_clique_size_) * 0.5f + a.avg_pair_3D_distance_ * 0.5f;
    float b_value = static_cast<float>(b.normalized_clique_size_) * 0.5f + b.avg_pair_3D_distance_ * 0.5f;*/

    /*float a_value = static_cast<float>(a.normalized_clique_size_) * 0.25f + a.avg_descriptor_distance_ * 0.25f + a.avg_pair_3D_distance_ * 0.25f + a.far_away_correspondences_weight_ * 0.25f;
    float b_value = static_cast<float>(b.normalized_clique_size_) * 0.25f + b.avg_descriptor_distance_ * 0.25f + b.avg_pair_3D_distance_ * 0.25f + b.far_away_correspondences_weight_ * 0.25f;*/

    float a_value = static_cast<float>(a.normalized_clique_size_) * 0.25f + a.avg_descriptor_distance_ * 0.25f; //+ a.far_away_correspondences_weight_ * 0.1f;
    float b_value = static_cast<float>(b.normalized_clique_size_) * 0.25f + b.avg_descriptor_distance_ * 0.25f; //+ b.far_away_correspondences_weight_ * 0.1f;

    return a_value > b_value;

    /*if(a.second->size() == b.second->size())
    {
        return a.first > b.first;
    }

    return a.second->size () > b.second->size ();*/
}

template<typename PointModelT, typename PointSceneT>
void
faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::cleanGraph2(GraphGGCG & g, int gc_thres)
{
  typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
  std::vector<typename boost::graph_traits<GraphGGCG>::vertex_descriptor> to_be_removed;
  int iter = 0;

  do
  {
    to_be_removed.clear();
    boost::tie (vertexIt, vertexEnd) = vertices (g);
    for (; vertexIt != vertexEnd; ++vertexIt)
    {
      int deg = static_cast<int>(boost::out_degree (*vertexIt, g));
      if ((deg > 0) && (deg < (gc_thres - 1)))
        to_be_removed.push_back (*vertexIt);
    }

    for (size_t i = 0; i < to_be_removed.size (); i++)
      clear_vertex (to_be_removed[i], g);

    //std::cout << "edges:" << num_edges (g)  << " iteration:" << iter << std::endl;
    iter++;
  } while(to_be_removed.size() > 0);
}

template<typename PointModelT, typename PointSceneT>
void
faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::cleanGraph(GraphGGCG & g, int gc_thres)
{

  cleanGraph2(g, gc_thres);

  /*std::vector<typename boost::graph_traits<GraphGGCG>::vertex_descriptor> art_points;
  do
  {


    //Biconnected components
    art_points.clear();
    typename boost::property_map < GraphGGCG, edge_component_t>::type component = get(edge_component, g);
    std::size_t num_comps = biconnected_components(g, component);
    boost::articulation_points(g, std::back_inserter(art_points));
    //std::cout << "Found " << art_points.size() << " articulation points.\n";

    for (size_t i = 0; i < art_points.size (); i++)
      clear_vertex (art_points[i], g);

  } while (art_points.size() > 0);*/
}

/*template<typename PointModelT, typename PointSceneT>
void
faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::visualizeCorrespondences(const pcl::Correspondences & correspondences)
{
  pcl::visualization::PCLVisualizer vis("model and scene keypoints");
  int v1, v2;
  vis.createViewPort(0,0,0.5,1.0,v1);
  vis.createViewPort(0.5,0,1,1.0,v2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_model(new pcl::PointCloud<pcl::PointXYZ>);
  keypoints_model->points.resize(correspondences.size());

  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_scene(new pcl::PointCloud<pcl::PointXYZ>);
  keypoints_scene->points.resize(correspondences.size());

  pcl::PointCloud<pcl::Normal>::Ptr keypoints_model_n(new pcl::PointCloud<pcl::Normal>);
  keypoints_model_n->points.resize(correspondences.size());

  pcl::PointCloud<pcl::Normal>::Ptr keypoints_scene_n(new pcl::PointCloud<pcl::Normal>);
  keypoints_scene_n->points.resize(correspondences.size());

  for (size_t k = 0; k < correspondences.size (); ++k)
  {
    int model_index_k = correspondences.at (k).index_query;
    keypoints_model->points[k].getVector3fMap() = input_->at (model_index_k).getVector3fMap();
    keypoints_model_n->points[k].getNormalVector3fMap() = input_normals_->at (model_index_k).getNormalVector3fMap();

    int scene_index_k = correspondences.at (k).index_match;
    keypoints_scene->points[k].getVector3fMap() = scene_->at (scene_index_k).getVector3fMap();
    keypoints_scene_n->points[k].getNormalVector3fMap() = scene_normals_->at (scene_index_k).getNormalVector3fMap();
  }

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*keypoints_model, centroid);
  pcl::demeanPointCloud(*keypoints_model, centroid, *keypoints_model);
  pcl::compute3DCentroid(*keypoints_scene, centroid);
  pcl::demeanPointCloud(*keypoints_scene, centroid, *keypoints_scene);

  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_handler(keypoints_model, 255, 0, 0);
    vis.addPointCloud(keypoints_model, scene_handler, "model", v1);
    vis.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(keypoints_model, keypoints_model_n, 1, 0.02, "model_normals", v1);
  }

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scene_handler(keypoints_scene, 255, 0, 0);
  vis.addPointCloud(keypoints_scene, scene_handler, "scene", v2);
  vis.addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(keypoints_scene, keypoints_scene_n, 1, 0.02, "scene normals", v2);
  vis.spin();
}*/

/*template<typename PointModelT, typename PointSceneT>
void
faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::visualizeGraph(GraphGGCG & g, std::string title)
{
  pcl::visualization::PCLVisualizer vis(title);
  int v1, v2, v3;
  vis.createViewPort(0,0,0.33,1.0,v1);
  vis.createViewPort(0.33,0,0.66,1.0,v2);
  vis.createViewPort(0.66,0,1.0,1.0,v3);
  vis.setBackgroundColor(255,255,255);

  typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
  std::vector<typename boost::graph_traits<GraphGGCG>::vertex_descriptor> to_be_removed;
  boost::tie (vertexIt, vertexEnd) = boost::vertices (g);
  int s=0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud, scene_cloud, scene_cloud_non_zero;
  pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>);

  model_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  scene_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  scene_cloud_non_zero.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::Correspondences valid_correspondences;
  std::vector<int> valid_correspondences_to_vertex_id;
  for (; vertexIt != vertexEnd; ++vertexIt, s++)
  {

    float r=255; float green=0; float b=0;
    float radius = 0.005f;
    if(boost::out_degree(*vertexIt, g) < (gc_threshold_ - 1))
    {
      r = 0;
      green = 125;
      radius = 0.0025f;
    }

    int model_index_k = model_scene_corrs_->at (*vertexIt).index_query;
    int scene_index_k = model_scene_corrs_->at (*vertexIt).index_match;
    pcl::PointXYZ mp, sp;
    mp.getVector3fMap() = input_->at (model_index_k).getVector3fMap();
    sp.getVector3fMap()  = scene_->at (scene_index_k).getVector3fMap();

//    std::stringstream name;
//    name << "sphere_" << s;
//    vis.addSphere(mp, radius, r, green, b, name.str(), v1);
//    name << "scene";
//    vis.addSphere(sp, radius, r, green, b, name.str(), v2);

    pcl::PointXYZRGB mpp, spp;
    mpp.getVector3fMap() = mp.getVector3fMap();
    spp.getVector3fMap() = sp.getVector3fMap();
    mpp.r = r; mpp.g = green; mpp.b = b;
    spp.r = r; spp.g = green; spp.b = b;
    model_cloud->points.push_back(mpp);
    scene_cloud->points.push_back(spp);
    scene_cloud_non_zero->points.push_back(spp);
    model_normals->points.push_back(input_normals_->at (model_index_k));

//    if(boost::out_degree(*vertexIt, g) > 0)
//    {
//      valid_correspondences.push_back(model_scene_corrs_->at (*vertexIt));
//      valid_correspondences_to_vertex_id.push_back(*vertexIt);
//    }
  }

  Eigen::Vector4f centroid_model, centroid_scene;

  pcl::compute3DCentroid(*model_cloud, centroid_model);
  pcl::compute3DCentroid(*scene_cloud, centroid_scene);
  Eigen::Vector4f demean = centroid_model + (centroid_scene * -1.f);
  pcl::demeanPointCloud(*model_cloud, demean, *model_cloud);

  vis.addPointCloud(model_cloud, "model", v1);
  vis.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(model_cloud, model_normals, 3,0.01f, "model_normals", v1);
  vis.addPointCloud(scene_cloud, "scene", v2);
  vis.addPointCloud(scene_cloud, "scene_v3", v3);
  //iterate over edges and draw lines
  typedef typename GraphGGCG::edge_iterator EdgeIterator;
  std::pair<EdgeIterator, EdgeIterator> edges = boost::edges(g);
  EdgeIterator edge;
  int edge_skip=5;
  int n_edge = 0;

  edge_skip = (boost::num_edges(g) / 2000);
//  float average_weight = 0.f;
//  std::vector<float> weights;
//  for (edge = edges.first; edge != edges.second; edge++, n_edge++) {
//    typename boost::graph_traits<GraphGGCG>::vertex_descriptor s, t;
//    s = boost::source(*edge, g);
//    t = boost::target(*edge, g);

//    float w = model_scene_corrs_->at (s).distance * model_scene_corrs_->at (t).distance;
//    weights.push_back(w);
//    average_weight += w;
//  }

//  std::sort(weights.begin(), weights.end());
//  average_weight /= n_edge;
//  average_weight = weights[weights.size() / 2];

  n_edge = 0;
  int n_discarded = 0;
  for (edge = edges.first; edge != edges.second; edge++, n_edge++) {
    if(edge_skip != 0 && (n_edge % edge_skip) != 0)
      continue;

    typename boost::graph_traits<GraphGGCG>::vertex_descriptor s, t;
    s = boost::source(*edge, g);
    t = boost::target(*edge, g);

    float w = model_scene_corrs_->at (s).distance * model_scene_corrs_->at (t).distance;

//    if(w > average_weight)
//    {
//      n_discarded++;
//      continue;
//    }

    int scene_index_k = model_scene_corrs_->at (s).index_match;
    int scene_index_j = model_scene_corrs_->at (t).index_match;

    int model_index_k = model_scene_corrs_->at (s).index_query;
    int model_index_j = model_scene_corrs_->at (t).index_query;

    pcl::PointXYZ p1,p2;
    p1.getVector3fMap() = scene_->at (scene_index_j).getVector3fMap();
    p2.getVector3fMap()  = scene_->at (scene_index_k).getVector3fMap();

    std::stringstream name;
    name << "line_" << n_edge;
    vis.addLine(p1,p2, name.str(), v2);

//    {
//        Eigen::Vector3f demean3f(demean[0],demean[1],demean[2]);
//        pcl::PointXYZ p1,p2;
//        p1.getVector3fMap() = input_->at (model_index_j).getVector3fMap();
//        p2.getVector3fMap()  = input_->at (model_index_k).getVector3fMap();
//        p1.getVector3fMap() = p1.getVector3fMap() - demean3f;
//        p2.getVector3fMap() = p2.getVector3fMap() - demean3f;
//        std::stringstream name;
//        name << "line_model_" << n_edge;
//        vis.addLine(p1,p2, name.str(), v1);
//    }
  }

  vis.spin();
}*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT>
  void
  faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::clusterCorrespondences (std::vector<pcl::Correspondences> &model_instances)
  {
    //pcl::ScopeTime t_cluster("cluster correspondences");
    //pcl::StopWatch time_watch;
    //time_watch.reset();

    if(prune_)
    {
      PCL_WARN("Prunning based on pose is activated...\n");
    }

    model_instances.clear ();
    found_transformations_.clear ();

    //for the old gc...
    pcl::CorrespondencesPtr sorted_corrs (new pcl::Correspondences (*model_scene_corrs_));
    std::sort (sorted_corrs->begin (), sorted_corrs->end (), gcGraphCorrespSorter);
    model_scene_corrs_ = sorted_corrs;

    if (!model_scene_corrs_)
    {
      PCL_ERROR(
          "[pcl::GeometricConsistencyGrouping::clusterCorrespondences()] Error! Correspondences not set, please set them before calling again this function.\n");
      return;
    }

    //temp copy of scene cloud with the type cast to ModelT in order to use Ransac
    PointCloudPtr temp_scene_cloud_ptr (new PointCloud ());
    pcl::copyPointCloud<PointSceneT, PointModelT> (*scene_, *temp_scene_cloud_ptr);

    GraphGGCG correspondence_graph (model_scene_corrs_->size ());
    float min_dist_for_cluster = gc_size_ * dist_for_cluster_factor_;
    //float max_dist_model = 0.f;

    {
      /*for (size_t k = 0; k < model_scene_corrs_->size (); ++k)
      {

        int model_index_k = model_scene_corrs_->at (k).index_query;
        const Eigen::Vector3f& model_point_k = input_->at (model_index_k).getVector3fMap ();
        const Eigen::Vector3f& model_normal_k = input_normals_->at (model_index_k).getNormalVector3fMap ();

        for (size_t j = (k + 1); j < model_scene_corrs_->size (); ++j)
        {
          int model_index_j = model_scene_corrs_->at (j).index_query;
          const Eigen::Vector3f& model_normal_j = input_normals_->at (model_index_j).getNormalVector3fMap ();

          double dot_distance = -1.0;
          if (pcl_isnan(model_normal_k.dot (model_normal_j)))
            dot_distance = 0.f;
          else
          {
            dot_distance = model_normal_k.dot (model_normal_j);
          }

          if(dot_distance >= 0)
          {
            const Eigen::Vector3f& model_point_j = input_->at (model_index_j).getVector3fMap ();

            Eigen::Vector3f dist_trg = model_point_k - model_point_j;
            max_dist_model = std::max (max_dist_model, dist_trg.norm ());
          }
        }
      }*/

      //std::cout << "max_dist_model is:" << max_dist_model << " " << gc_size_ << " " << gc_threshold_ << std::endl;
      //max_dist_model += gc_size_;

      for (size_t k = 0; k < model_scene_corrs_->size (); ++k)
      {
        int scene_index_k = model_scene_corrs_->at (k).index_match;
        int model_index_k = model_scene_corrs_->at (k).index_query;
        const Eigen::Vector3f& scene_point_k = scene_->at (scene_index_k).getVector3fMap ();
        const Eigen::Vector3f& model_point_k = input_->at (model_index_k).getVector3fMap ();
        const Eigen::Vector3f& scene_normal_k = scene_normals_->at (scene_index_k).getNormalVector3fMap ();
        const Eigen::Vector3f& model_normal_k = input_normals_->at (model_index_k).getNormalVector3fMap ();

        for (size_t j = (k + 1); j < model_scene_corrs_->size (); ++j)
        {
          int scene_index_j = model_scene_corrs_->at (j).index_match;
          int model_index_j = model_scene_corrs_->at (j).index_query;

          //same scene or model point constraint
          if(scene_index_j == scene_index_k || model_index_j == model_index_k)
            continue;

          const Eigen::Vector3f& scene_point_j = scene_->at (scene_index_j).getVector3fMap ();
          const Eigen::Vector3f& model_point_j = input_->at (model_index_j).getVector3fMap ();

          const Eigen::Vector3f& scene_normal_j = scene_normals_->at (scene_index_j).getNormalVector3fMap ();
          const Eigen::Vector3f& model_normal_j = input_normals_->at (model_index_j).getNormalVector3fMap ();

          Eigen::Vector3f dist_trg = model_point_k - model_point_j;
          Eigen::Vector3f dist_ref = scene_point_k - scene_point_j;

          //minimum distance constraint
          if ((dist_trg.norm () < min_dist_for_cluster) || (dist_ref.norm () < min_dist_for_cluster))
            continue;

          assert((dist_trg.norm () >= min_dist_for_cluster) && (dist_ref.norm () >= min_dist_for_cluster));

          double distance = fabs (dist_trg.norm () - dist_ref.norm());
          double dot_distance = 0;
          if (pcl_isnan(scene_normal_k.dot (scene_normal_j)) || pcl_isnan(model_normal_k.dot (model_normal_j)))
            dot_distance = 0.f;
          else
          {
            float dot_model = model_normal_k.dot (model_normal_j);
            /*if(dot_model < 0)
            {
              dot_distance = std::numeric_limits<double>::max();
            }
            else
            {*/
            dot_distance = std::abs (scene_normal_k.dot (scene_normal_j) - dot_model);
            //}
          }

          //Model normals should be consistently oriented! otherwise reject!
          float dot_distance_model = model_normal_k.dot (model_normal_j);
          if(dot_distance_model < -0.1f) continue;

          //gc constraint and dot_product constraint!
          if ((distance < gc_size_) && (dot_distance <= thres_dot_distance_))
          {
            //max_model distance constraint
            /*if(dist_ref.norm () > max_dist_model)
            {
              PCL_WARN("Max distance constraint\n");
              continue;
            }*/

            boost::add_edge (k, j, correspondence_graph);
          }
        }
      }
    }

    /*{
        double ms = time_watch.getTime();
        std::cout << "Built graph:" << ms << std::endl;
    }*/

    //std::cout << "edges before cleaning:" << num_edges (correspondence_graph) << std::endl;
    /*cleanGraph (correspondence_graph, gc_threshold_);
    std::cout << "edges:" << num_edges (correspondence_graph) << std::endl;*/

    /*int num_edges_start;
    do
    {
      num_edges_start = num_edges (correspondence_graph);
      //pcl::ScopeTime t ("checking for additional edges...\n");
      //std::cout << "edges:" << num_edges (correspondence_graph) << std::endl;
      for (size_t k = 0; k < model_scene_corrs_->size (); ++k)
      {
        for (size_t j = (k + 1); j < model_scene_corrs_->size (); ++j)
        {
          if (boost::edge (k, j, correspondence_graph).second)
          {
            //check if there is another correspondence different than k and j, that fullfills the gc constraint with both k and j
            bool edge_available = false;
            for(size_t c=0; c < model_scene_corrs_->size(); c++)
            {
              if (c == j || c == k)
                continue;

              if (boost::edge (k, c, correspondence_graph).second && boost::edge (j, c, correspondence_graph).second)
              {
                edge_available = true;
                break;
              }
            }

            if (!edge_available)
              boost::remove_edge(k, j, correspondence_graph);
          }
        }
      }
      //std::cout << "edges:" << num_edges (correspondence_graph) << std::endl;
      cleanGraph(correspondence_graph, gc_threshold_);
      //std::cout << "edges:" << num_edges (correspondence_graph) << std::endl;
    } while (num_edges_start != num_edges (correspondence_graph));*/

    /*{
        double ms = time_watch.getTime();
        std::cout << "Built graph + clean graph:" << ms << std::endl;
    }*/

    /*boost::vector_property_map<int> components (boost::num_vertices (correspondence_graph));
    int n_cc = static_cast<int> (boost::connected_components (correspondence_graph, &components[0]));
    std::cout << "Number of connected components..." << n_cc << std::endl;*/

    typename boost::property_map < GraphGGCG, edge_component_t>::type components = get(edge_component, correspondence_graph);
    int n_cc = static_cast<int>(biconnected_components(correspondence_graph, components));

    if(n_cc < 1)
      return;

    std::vector<int> model_instances_kept_indices;

    std::vector< std::set<int> > unique_vertices_per_cc;
    std::vector<int> cc_sizes;
    cc_sizes.resize (n_cc, 0);
    unique_vertices_per_cc.resize (n_cc);

    typename boost::graph_traits<GraphGGCG>::edge_iterator edgeIt, edgeEnd;
    boost::tie (edgeIt, edgeEnd) = edges (correspondence_graph);
    for (; edgeIt != edgeEnd; ++edgeIt)
    {
      int c = components[*edgeIt];
      unique_vertices_per_cc[c].insert(boost::source(*edgeIt, correspondence_graph));
      unique_vertices_per_cc[c].insert(boost::target(*edgeIt, correspondence_graph));
    }

    for(size_t i=0; i < unique_vertices_per_cc.size(); i++)
      cc_sizes[i] = unique_vertices_per_cc[i].size();

    //for (size_t i = 0; i < model_scene_corrs_->size (); i++)
    //cc_sizes[components[i]]++;

    /*{
      //remove edges from those connected components smaller than threshold
      typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
      std::vector<typename boost::graph_traits<GraphGGCG>::vertex_descriptor> to_be_removed;
      boost::tie (vertexIt, vertexEnd) = vertices (correspondence_graph);
      for (; vertexIt != vertexEnd; ++vertexIt)
      {
        if (cc_sizes[components[*vertexIt]] < gc_threshold_)
          to_be_removed.push_back (*vertexIt);
      }

      for (size_t i = 0; i < to_be_removed.size (); i++)
        clear_vertex (to_be_removed[i], correspondence_graph);
    }*/

    pcl::registration::CorrespondenceRejectorSampleConsensus<PointModelT> corr_rejector;
    corr_rejector.setMaximumIterations (10000);
    corr_rejector.setInlierThreshold (ransac_threshold_);
    corr_rejector.setInputSource (input_);
    corr_rejector.setInputTarget (temp_scene_cloud_ptr);
    corr_rejector.setSaveInliers(true);

    //Go through the connected components and decide whether to use CliqueGC or usualGC or ignore (cc_sizes[i] < gc_threshold_)
    //Decision based on the number of vertices in the connected component and graph arbocity...

    /*{
        double ms = time_watch.getTime();
        std::cout << "Built graph + clean graph + CC:" << ms << std::endl;
    }*/

    //std::cout << "Number of connected components over threshold..." << over_gc << std::endl;
    int analyzed_ccs = 0;
    std::vector<bool> cliques_computation_possible_;
    cliques_computation_possible_.resize(n_cc, use_graph_);
    for (int c = 0; c < n_cc; c++)
    {
      //ignore if not enough vertices...
      int num_v_in_cc = cc_sizes[c];
      if (num_v_in_cc < gc_threshold_)
      {
        continue;
      }

      analyzed_ccs++;

      //pcl::ScopeTime ttt("Processing connected component");
      GraphGGCG connected_graph(correspondence_graph);

      //do i need to do this again?
      /*{
        typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
        std::vector<typename boost::graph_traits<GraphGGCG>::vertex_iterator> to_be_removed;
        to_be_removed.resize(boost::num_vertices(connected_graph));
        boost::tie (vertexIt, vertexEnd) = vertices (connected_graph);

        int r = 0;
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
          if (components[*vertexIt] != c)
          {
            to_be_removed[r] = vertexIt;
            r++;
          }
        }

        to_be_removed.resize(r);
        //std::cout << "size to be removed:" << to_be_removed.size() << std::endl;
        for (size_t i = 0; i < to_be_removed.size (); i++)
          clear_vertex (*(to_be_removed[i]), connected_graph);
      }*/

      //iterate over edges and remove those not belonging to this biconnected component
      typename boost::graph_traits<GraphGGCG>::edge_iterator edgeIt, edgeEnd;
      boost::tie (edgeIt, edgeEnd) = edges (connected_graph);
      for (; edgeIt != edgeEnd; ++edgeIt)
      {
        if (components[*edgeIt] != c)
        {
          boost::remove_edge(*edgeIt, connected_graph);
        }
      }
      //std::cout << "Num edges connnected component:" << boost::num_edges(connected_graph) << std::endl;
      //visualizeGraph(connected_graph, "connected component");

      float arboricity = num_edges (connected_graph) / static_cast<float>(num_v_in_cc - 1);
      //std::cout << "arboricity:" << arboricity << " num_v:" << num_v_in_cc << " edges:" << num_edges (connected_graph) << std::endl;
      std::vector<std::pair<int, int> > edges_used;
      std::set<int> correspondences_used;

      std::vector< std::vector<int> > correspondence_to_instance;
      correspondence_to_instance.resize(model_scene_corrs_->size());

      if (cliques_computation_possible_[c] && arboricity < 25 /*&& (num_v_in_cc < 400) && (num_edges (connected_graph) < 8000) && arboricity < 10*/)
      {
        //std::cout << "Using cliques" << std::endl;
        //std::cout << "N edges: " << num_edges (connected_graph) << " vertices:" << num_v_in_cc << " arboricity:" << arboricity <<  std::endl;

        std::vector<std::vector<long unsigned int> *> cliques;
        {
          //pcl::ScopeTime t ("tomita cliques...");
          Tomita<GraphGGCG> tom (static_cast<size_t> (gc_threshold_));
          tom.setMaxTimeAllowed(max_time_allowed_cliques_comptutation_);
          tom.find_cliques (connected_graph, static_cast<int> (model_scene_corrs_->size ()));
          if(tom.getMaxTimeReached())
          {
              PCL_ERROR("Max time reached during clique computation %f!!\n", max_time_allowed_cliques_comptutation_);
              cliques_computation_possible_[c] = false;
              c--;
              analyzed_ccs--;

              //free memory for cliques
              tom.getCliques (cliques);
              for (size_t p = 0; p < cliques.size (); p++)
                delete cliques[p];

              continue;
          }
          //std::cout << "Number of cliques found by tomita..." << tom.getNumCliquesFound () << std::endl;
          tom.getCliques (cliques);
        }

        std::vector< ExtendedClique > extended_cliques;
        std::vector<std::pair<float, std::vector<long unsigned int> * > > cliques_with_average_weight;
        for(size_t k = 0; k < cliques.size(); k++)
        {
            float avg_dist = 0.f;
            float max_dist_ = 0.03f; //3 centimeters
            float far_away_average_weight_ = 0.f;

            for(size_t jj=0; jj < cliques[k]->size(); jj++)
            {
                avg_dist += model_scene_corrs_->at(cliques[k]->at(jj)).distance;
            }

            avg_dist /= static_cast<float>(cliques[k]->size());
            cliques_with_average_weight.push_back(std::make_pair(avg_dist, cliques[k]));

            float avg_3D_dist = 0.f;

            for(size_t jj=0; jj < cliques[k]->size(); jj++)
            {

                int scene_index_j = model_scene_corrs_->at (cliques[k]->at(jj)).index_match;
                int model_index_j = model_scene_corrs_->at (cliques[k]->at(jj)).index_query;
                const Eigen::Vector3f& scene_point_j = scene_->at (scene_index_j).getVector3fMap ();
                const Eigen::Vector3f& model_point_j = input_->at (model_index_j).getVector3fMap ();

                for(size_t kk=(jj+1); kk < cliques[k]->size(); kk++)
                {
                    //for each pair, average 3D distance

                    int scene_index_k = model_scene_corrs_->at (cliques[k]->at(kk)).index_match;
                    int model_index_k = model_scene_corrs_->at (cliques[k]->at(kk)).index_query;

                    const Eigen::Vector3f& scene_point_k = scene_->at (scene_index_k).getVector3fMap ();
                    const Eigen::Vector3f& model_point_k = input_->at (model_index_k).getVector3fMap ();

                    Eigen::Vector3f dist_trg = model_point_k - model_point_j;
                    Eigen::Vector3f dist_ref = scene_point_k - scene_point_j;

                    float distance_ref_norm = dist_ref.norm();
                    float distance = fabs (dist_trg.norm () - dist_ref.norm());
                    avg_3D_dist += distance;

                    far_away_average_weight_ += std::min((distance_ref_norm / max_dist_), 1.f);
                }
            }

            avg_3D_dist /= (static_cast<float>(cliques[k]->size()) * static_cast<float>(cliques[k]->size() - 1)) / 2.f;
            far_away_average_weight_ /= (static_cast<float>(cliques[k]->size()) * static_cast<float>(cliques[k]->size() - 1)) / 2.f;

            ExtendedClique ec;
            ec.correspondences_ = cliques[k];
            ec.avg_pair_3D_distance_ = avg_3D_dist;
            ec.avg_descriptor_distance_ = avg_dist;
            ec.avg_pair_3D_distance_unnormalized_ = avg_3D_dist;
            ec.far_away_correspondences_weight_ = far_away_average_weight_;
            extended_cliques.push_back(ec);
        }

        float max_avg_3D_dist = 0;
        float max_avg_descriptor_dist = 0;
        size_t max_clique_size = 0;

        for(size_t k = 0; k < cliques.size(); k++)
        {
            if(extended_cliques[k].avg_pair_3D_distance_ > max_avg_3D_dist)
            {
                max_avg_3D_dist = extended_cliques[k].avg_pair_3D_distance_;
            }

            if(extended_cliques[k].correspondences_->size() > max_clique_size)
            {
                max_clique_size = extended_cliques[k].correspondences_->size();
            }

            if(extended_cliques[k].avg_descriptor_distance_ > max_avg_descriptor_dist)
            {
                max_avg_descriptor_dist = extended_cliques[k].avg_descriptor_distance_;
            }
        }

        for(size_t k = 0; k < cliques.size(); k++)
        {
            extended_cliques[k].avg_pair_3D_distance_ = 1.f - (extended_cliques[k].avg_pair_3D_distance_ / max_avg_3D_dist);
            extended_cliques[k].avg_descriptor_distance_ = 1.f - (extended_cliques[k].avg_descriptor_distance_ / max_avg_descriptor_dist);
            extended_cliques[k].normalized_clique_size_ = static_cast<float>(extended_cliques[k].correspondences_->size()) / static_cast<float>(max_clique_size);
        }

        //process cliques to remove similar ones...
        //sort (cliques.begin (), cliques.end (), less_clique_vectors); //cliques are sorted in increasing order (smaller cliques first)

        /*sort (cliques_with_average_weight.begin (), cliques_with_average_weight.end (), best_clique_vectors);
        for(size_t k = 0; k < cliques.size(); k++)
        {
            cliques[k] = cliques_with_average_weight[k].second;
        }*/

        sort (extended_cliques.begin (), extended_cliques.end (), best_extended_cliques);

        /*for(size_t k = 0; k < cliques.size(); k++)
        {
            cliques[k] = extended_cliques[k].correspondences_;
            std::cout << extended_cliques[k].correspondences_->size() << " normed size:" << extended_cliques[k].normalized_clique_size_ << " 3d:" << extended_cliques[k].avg_pair_3D_distance_  << " descriptor:" << extended_cliques[k].avg_descriptor_distance_ << " AVG PAIR 3D (unnormalized):" << extended_cliques[k].avg_pair_3D_distance_unnormalized_ << " correspondences:"; //<< std::endl;
            for(size_t kk=0; kk < cliques[k]->size(); kk++)
            {
                std::cout << extended_cliques[k].correspondences_->at(kk) << " ";
            }

            std::cout << std::endl;
        }*/

        std::vector<std::vector<long unsigned int> *>::iterator it;
        std::vector<int> taken_corresps (model_scene_corrs_->size (), 0);
        int max_taken = max_taken_correspondence_;

        if(!cliques_big_to_small_)
          std::reverse (cliques.begin (), cliques.end ());

        for (it = cliques.begin (); it != cliques.end (); it++)
        {
          //std::cout << "clique size:" << (*it)->size () << std::endl;
          //create a new clique based on how many time the correspondences in *it clique were used
          std::vector<long unsigned int> * new_clique = new std::vector<long unsigned int>;
          new_clique->reserve ((*it)->size ());
          int used = 0;
          for (size_t i = 0; i < (*it)->size (); i++)
          {
            if (taken_corresps[(**it)[i]] < max_taken)
            {
              new_clique->push_back ((**it)[i]); //(**it)
              used++;
            }
          }

          if (used >= gc_threshold_)
          {
            new_clique->resize (used);

            //do ransac with these correspondences...
            pcl::Correspondences temp_corrs, filtered_corrs;
            temp_corrs.reserve (used);
            for (size_t j = 0; j < new_clique->size (); j++)
            {
              assert(new_clique->at (j) < model_scene_corrs_->size());
              temp_corrs.push_back (model_scene_corrs_->at (new_clique->at (j)));
            }

            corr_rejector.getRemainingCorrespondences (temp_corrs, filtered_corrs);

            std::vector<int> inlier_indices;
            corr_rejector.getInliersIndices (inlier_indices);

            //check if corr_rejector.getBestTransformation () was not found already
            bool found = poseExists (corr_rejector.getBestTransformation ());

            if (((int)filtered_corrs.size () >= gc_threshold_) && !found && (inlier_indices.size() != 0))
            {
              Eigen::Matrix4f trans = corr_rejector.getBestTransformation ();

              //check if the normals are ok after applying the transformation
              bool all_wrong = check_normals_orientation_;

              if(check_normals_orientation_)
              {
                  for(size_t j=0; j < filtered_corrs.size(); j++)
                  {
                    //transform normal
                    const Eigen::Vector3f& model_normal = input_normals_->at (filtered_corrs[j].index_query).getNormalVector3fMap ();
                    const Eigen::Vector3f& scene_normal = scene_normals_->at (filtered_corrs[j].index_match).getNormalVector3fMap ();
                    if(!pcl_isfinite(model_normal[0]) || !pcl_isfinite(scene_normal[0]) ||
                        !pcl_isfinite(model_normal[1]) || !pcl_isfinite(scene_normal[1]) ||
                        !pcl_isfinite(model_normal[2]) || !pcl_isfinite(scene_normal[2]))
                    {
                      continue;
                    }

                    Eigen::Vector3f nt;
                    nt[0] = static_cast<float> (trans (0, 0) * model_normal[0] + trans (0, 1) * model_normal[1] + trans (0, 2) * model_normal[2]);
                    nt[1] = static_cast<float> (trans (1, 0) * model_normal[0] + trans (1, 1) * model_normal[1] + trans (1, 2) * model_normal[2]);
                    nt[2] = static_cast<float> (trans (2, 0) * model_normal[0] + trans (2, 1) * model_normal[1] + trans (2, 2) * model_normal[2]);
                    if(nt.dot(scene_normal) >= (1.f - thres_dot_distance_))
                      all_wrong = false;

                  }
              }

              if(!all_wrong)
              {
                //PCL_INFO("Normals are consistent %d!!\n", static_cast<int>(all_nans));
                found_transformations_.push_back (trans);
                model_instances.push_back (filtered_corrs);

                //mark all inliers
                for (size_t j = 0; j < inlier_indices.size (); j++)
                {
                  taken_corresps[new_clique->at (inlier_indices[j])]++;
                  correspondence_to_instance[new_clique->at (inlier_indices[j])].push_back(model_instances.size() - 1);
                }

                for (size_t j = 0; j < inlier_indices.size (); j++)
                {
                  correspondences_used.insert(new_clique->at (inlier_indices[j]));
                  for (size_t k = (j+1); k < inlier_indices.size (); k++)
                  {
                    edges_used.push_back(std::make_pair(new_clique->at (inlier_indices[j]),new_clique->at (inlier_indices[k])));
                  }
                }
              }

              /*found_transformations_.push_back (corr_rejector.getBestTransformation ());
              model_instances.push_back (filtered_corrs);

              //mark all
              for (size_t j = 0; j < inlier_indices.size (); j++)
              {
                taken_corresps[new_clique->at (inlier_indices[j])]++;
              }*/

              delete new_clique;
            }
            else
            {
              delete new_clique;
            }
          }
          else
          {
            delete new_clique;
          }
        }

        for (size_t p = 0; p < cliques.size (); p++)
          delete cliques[p];
      }
      else
      {
        //use iterative gc for simple cases with lots of correspondences...
        PCL_WARN("Problem is too hard to solve it using cliques...\n");
        std::cout << "N edges: " << num_edges (connected_graph) << " vertices:" << num_v_in_cc << " arboricity:" << arboricity <<  std::endl;

        std::vector<size_t> consensus_set;
        consensus_set.resize(model_scene_corrs_->size ());
        std::vector<bool> taken_corresps (model_scene_corrs_->size (), false);
        //std::vector<bool> taken_corresps (model_scene_corrs_->size (), true);
        //using only the correspondences in the connected component
        /*typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
        boost::tie (vertexIt, vertexEnd) = vertices (correspondence_graph);
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
          if (components[*vertexIt] == c)
          {
            taken_corresps[*vertexIt] = false;
          }
        }*/

        GraphGGCG connected_graph(correspondence_graph);
        //iterate over edges and remove those not belonging to this biconnected component
        typename boost::graph_traits<GraphGGCG>::edge_iterator edgeIt, edgeEnd;
        boost::tie (edgeIt, edgeEnd) = edges (connected_graph);
        for (; edgeIt != edgeEnd; ++edgeIt)
        {
          if (components[*edgeIt] != c)
          {
            boost::remove_edge(*edgeIt, connected_graph);
          }
        }

        typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
        boost::tie (vertexIt, vertexEnd) = vertices (connected_graph);
        for (; vertexIt != vertexEnd; ++vertexIt)
        {
          if ( boost::out_degree(*vertexIt, connected_graph) < (gc_threshold_ - 1))
            taken_corresps[*vertexIt] = true;
        }

        for (size_t i = 0; i < model_scene_corrs_->size (); ++i)
        {
          if (taken_corresps[i])
            continue;

          int consensus_size = 0;
          consensus_set[consensus_size++] = i;

          for (size_t j = 0; j < model_scene_corrs_->size (); ++j)
          {
            if (j != i && !taken_corresps[j])
            {
              //Let's check if j fits into the current consensus set
              bool is_a_good_candidate = true;

              for (int k = 0; k < consensus_size; ++k)
              {
                //check if edge (j, consensus_set[k] exists in the graph, if it does not, is_a_good_candidate = false!...
                if (!(boost::edge (j, consensus_set[k], correspondence_graph).second))
                {
                  is_a_good_candidate = false;
                  break;
                }
              }

              if (is_a_good_candidate)
                consensus_set[consensus_size++] = j;
            }
          }

          if (consensus_size >= gc_threshold_)
          {
            pcl::Correspondences temp_corrs, filtered_corrs;
            temp_corrs.reserve (consensus_size);
            for (int j = 0; j < consensus_size; j++)
            {
              temp_corrs.push_back (model_scene_corrs_->at (consensus_set[j]));
              //taken_corresps[consensus_set[j]] = true;
            }

            if (ransac_threshold_ > 0)
            {
              //pcl::ScopeTime tt("ransac filtering");
              //ransac filtering
              corr_rejector.getRemainingCorrespondences (temp_corrs, filtered_corrs);
              //check if corr_rejector.getBestTransformation () was not found already
              bool found = poseExists (corr_rejector.getBestTransformation ());

              std::vector<int> inlier_indices;
              corr_rejector.getInliersIndices (inlier_indices);

              //save transformations for recognize
              if ((filtered_corrs.size () >= gc_threshold_) && !found && (inlier_indices.size() != 0))
              {
                Eigen::Matrix4f trans = corr_rejector.getBestTransformation ();

                //check if the normals are ok after applying the transformation
                bool all_wrong = check_normals_orientation_;

                if(check_normals_orientation_)
                {
                    for(size_t j=0; j < filtered_corrs.size(); j++)
                    {
                      //transform normal
                      const Eigen::Vector3f& model_normal = input_normals_->at (filtered_corrs[j].index_query).getNormalVector3fMap ();
                      const Eigen::Vector3f& scene_normal = scene_normals_->at (filtered_corrs[j].index_match).getNormalVector3fMap ();
                      if(!pcl_isfinite(model_normal[0]) || !pcl_isfinite(scene_normal[0]) ||
                          !pcl_isfinite(model_normal[1]) || !pcl_isfinite(scene_normal[1]) ||
                          !pcl_isfinite(model_normal[2]) || !pcl_isfinite(scene_normal[2]))
                      {
                        continue;
                      }

                      Eigen::Vector3f nt;
                      nt[0] = static_cast<float> (trans (0, 0) * model_normal[0] + trans (0, 1) * model_normal[1] + trans (0, 2) * model_normal[2]);
                      nt[1] = static_cast<float> (trans (1, 0) * model_normal[0] + trans (1, 1) * model_normal[1] + trans (1, 2) * model_normal[2]);
                      nt[2] = static_cast<float> (trans (2, 0) * model_normal[0] + trans (2, 1) * model_normal[1] + trans (2, 2) * model_normal[2]);
                      if(nt.dot(scene_normal) >= (1.f - thres_dot_distance_))
                        all_wrong = false;

                    }
                }

                if(all_wrong)
                {
                  //PCL_ERROR("Normals are not consistent %d %d!!\n", static_cast<int>(all_wrong), static_cast<int>(all_nans));
                  for (int j = 0; j < consensus_size; j++)
                    taken_corresps[consensus_set[j]] = false;
                }
                else
                {
                  //PCL_INFO("Normals are consistent %d!!\n", static_cast<int>(all_nans));
                  found_transformations_.push_back (trans);
                  model_instances.push_back (filtered_corrs);

                  //for (int j = 0; j < consensus_size; j++)
                  //taken_corresps[consensus_set[j]] = true;

                  //if (consensus_size > static_cast<int>(filtered_corrs.size ()))
                  //{
                    //free taken_corresps...
                    //PCL_WARN("RANSAC rejected %d correspondences from a valid set of %d ... remaining... %d \n", consensus_set.size () - filtered_corrs.size(), consensus_set.size (), filtered_corrs.size());

                  //mark all inliers
                  for (size_t j = 0; j < inlier_indices.size (); j++)
                  {
                    taken_corresps[consensus_set[inlier_indices[j]]] = true;
                    correspondence_to_instance[consensus_set[inlier_indices[j]]].push_back(model_instances.size() - 1);
                  }

                  for (size_t j = 0; j < inlier_indices.size (); j++)
                  {
                    correspondences_used.insert(consensus_set[inlier_indices[j]]);
                    for (size_t k = (j+1); k < inlier_indices.size (); k++)
                    {
                      edges_used.push_back(std::make_pair(consensus_set[inlier_indices[j]],consensus_set[inlier_indices[k]]));
                    }
                  }
                  //}
                }
              }
              else
              {
                //Free the correspondences so they can be used in another set...
                //PCL_ERROR("Freeing %d correspondences from invalid set...\n", consensus_set.size ());
                for (int j = 0; j < consensus_size; j++)
                  taken_corresps[consensus_set[j]] = false;
              }
            }
            /*else
            {
              //found_transformations_.push_back (corr_rejector.getBestTransformation ());
              model_instances.push_back (temp_corrs);
              for (int j = 0; j < consensus_size; j++)
                taken_corresps[consensus_set[j]] = true;
            }*/
          }
        }
      }

      if(prune_by_CC_)
      {
          //pcl::ScopeTime t("final post-processing...");
          GraphGGCG connected_graph_used_edges(connected_graph);
          typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
          std::vector<typename boost::graph_traits<GraphGGCG>::vertex_descriptor> to_be_removed;
          boost::tie (vertexIt, vertexEnd) = vertices (connected_graph_used_edges);
          for (; vertexIt != vertexEnd; ++vertexIt)
          {
            std::set<int>::iterator it;
            it = correspondences_used.find(*vertexIt);
            if (it == correspondences_used.end())
              to_be_removed.push_back (*vertexIt);
          }

          for (size_t i = 0; i < to_be_removed.size (); i++)
            clear_vertex (to_be_removed[i], connected_graph_used_edges);


          //std::cout << "Used VS connected:" << num_edges(connected_graph_used_edges) << " " << num_edges(connected_graph) << std::endl;

          {
            boost::vector_property_map<int> components (boost::num_vertices (connected_graph_used_edges));
            int n_cc = static_cast<int> (boost::connected_components (connected_graph_used_edges, &components[0]));

            std::vector<int> cc_sizes;
            cc_sizes.resize (n_cc, 0);
            for (size_t i = 0; i < model_scene_corrs_->size (); i++)
              cc_sizes[components[i]]++;

            int ncc_overthres = 0;
            for (int i = 0; i < n_cc; i++)
            {
              if(cc_sizes[i] >= gc_threshold_)
                ncc_overthres++;
            }

            //std::cout << "Number of connected components over threshold: " << ncc_overthres << std::endl;

            //somehow now i need to do a Nonmax supression of the model_instances that are in the same CC
            //gather instances that were generated with correspondences found in a specific CC
            //correspondence_to_instance maps correspondences (vertices) to instance, we can use that i guess

            for (int internal_c = 0; internal_c < n_cc; internal_c++)
            {
              //ignore if not enough vertices...
              int num_v_in_cc = cc_sizes[internal_c];
              if (num_v_in_cc < gc_threshold_)
                continue;

              std::set<int> instances_for_this_cc;
              {
                typename boost::graph_traits<GraphGGCG>::vertex_iterator vertexIt, vertexEnd;
                boost::tie (vertexIt, vertexEnd) = vertices (connected_graph_used_edges);

                for (; vertexIt != vertexEnd; ++vertexIt)
                {
                  if (components[*vertexIt] == internal_c)
                  {
                    for(size_t k=0; k < correspondence_to_instance[*vertexIt].size(); k++)
                    {
                      instances_for_this_cc.insert(correspondence_to_instance[*vertexIt][k]);
                    }
                  }
                }
              }

              //std::cout << "instances in this cc:" << instances_for_this_cc.size() << std::endl;
              std::set<int>::iterator it;
              int max_size = 0;
              //int max_idx = 0;
              for(it = instances_for_this_cc.begin(); it != instances_for_this_cc.end(); it++)
              {
                //std::cout << *it << " " << model_instances[*it].size() << std::endl;
                if(max_size <= static_cast<int>(model_instances[*it].size()))
                {
                  max_size = static_cast<int>(model_instances[*it].size());
                  //max_idx = *it;
                }
              }

              //std::cout << std::endl;

              float thres = 0.5f;
              for(it = instances_for_this_cc.begin(); it != instances_for_this_cc.end(); it++)
              {
                int size = static_cast<int>(model_instances[*it].size());
                if( (size) > (max_size * thres))
                  model_instances_kept_indices.push_back(*it);
              }
            }
          }

          /*if(visualize_graph_ && correspondences_used.size() > 0)
            visualizeGraph(connected_graph_used_edges, "used edges");*/

      }
    }

    if(prune_by_CC_)
    {
      for(size_t i=0; i < model_instances_kept_indices.size(); i++)
      {
        model_instances[i] = model_instances[model_instances_kept_indices[i]];
        found_transformations_[i] = found_transformations_[model_instances_kept_indices[i]];
      }

      model_instances.resize(model_instances_kept_indices.size());
      found_transformations_.resize(model_instances_kept_indices.size());
    }

    //visualizeCorrespondences(*model_scene_corrs_);

  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT>
  bool
  faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::recognize (
                                                                                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<
                                                                                        Eigen::Matrix4f> > &transformations)
  {
    std::vector<pcl::Correspondences> model_instances;
    return (this->recognize (transformations, model_instances));
  }

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointModelT, typename PointSceneT>
  bool
  faat_pcl::GraphGeometricConsistencyGrouping<PointModelT, PointSceneT>::recognize (
                                                                                    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<
                                                                                        Eigen::Matrix4f> > &transformations,
                                                                                    std::vector<pcl::Correspondences> &clustered_corrs)
  {
    transformations.clear ();
    if (!this->initCompute ())
    {
      PCL_ERROR(
          "[faat_pcl::GraphGeometricConsistencyGrouping::recognize()] Error! Model cloud or Scene cloud not set, please set them before calling again this function.\n");
      return (false);
    }

    clusterCorrespondences (clustered_corrs);

    transformations = found_transformations_;

    this->deinitCompute ();
    return (true);
  }

#endif // FAAT_PCL_RECOGNITION_SI_GEOMETRIC_CONSISTENCY_IMPL_H_
