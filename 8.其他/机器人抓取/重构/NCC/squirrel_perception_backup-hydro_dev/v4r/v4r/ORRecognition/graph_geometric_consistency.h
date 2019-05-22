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

#ifndef FAAT_PCL_RECOGNITION_GRAPH_GEOMETRIC_CONSISTENCY_H_
#define FAAT_PCL_RECOGNITION_GRAPH_GEOMETRIC_CONSISTENCY_H_

#include "correspondence_grouping.h"
#include <pcl/point_cloud.h>
#include <pcl/common/angles.h>
#include <pcl/common/centroid.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_graph.hpp>

namespace faat_pcl
{
  template<typename PointModelT, typename PointSceneT>
  class GraphGeometricConsistencyGrouping : public faat_pcl::CorrespondenceGrouping<PointModelT, PointSceneT>
  {

    struct edge_component_t
    {
      enum
      { num = 555 };
      typedef boost::edge_property_tag kind;
    }
    edge_component;

    typedef boost::adjacency_matrix<boost::undirectedS, int, boost::property<edge_component_t, std::size_t> > GraphGGCG;
    void cleanGraph2(GraphGGCG & g, int gc_thres);
    void cleanGraph(GraphGGCG & g, int gc_thres);
    float max_time_allowed_cliques_comptutation_;

    public:
      typedef pcl::PointCloud<PointModelT> PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;

      typedef typename faat_pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::SceneCloudConstPtr SceneCloudConstPtr;

      /** \brief Constructor */
      GraphGeometricConsistencyGrouping ()
        : gc_threshold_ (3)
        , gc_size_ (1.0)
        , found_transformations_ ()
      {
        use_graph_ = true;
        require_normals_ = true;
        prune_ = false;
        thres_dot_distance_ = 0.05f;
        dist_for_cluster_factor_ = 3.f;
        visualize_graph_ = false;
        prune_by_CC_ = false;
        max_taken_correspondence_ = 5;
        cliques_big_to_small_ = false;
        check_normals_orientation_ = true;
        max_time_allowed_cliques_comptutation_ = std::numeric_limits<float>::infinity();
      }

      inline
      void setMaxTimeForCliquesComputation(float t)
      {
          max_time_allowed_cliques_comptutation_ = t;
      }

      inline
      void setCheckNormalsOrientation(bool b)
      {
        check_normals_orientation_ = b;
      }

      inline
      void setSortCliques(bool b)
      {
        cliques_big_to_small_ = b;
      }

      inline
      void setMaxTaken(int t)
      {
        max_taken_correspondence_ = t;
      }

      inline
      void pruneByCC(bool b)
      {
        prune_by_CC_ = b;
      }

      inline
      void setDistForClusterFactor(float f)
      {
        dist_for_cluster_factor_ = f;
      }

      inline
      bool poseExists(Eigen::Matrix4f corr_rej_trans)
      {
        if(!prune_)
          return false;

        bool found = false;
        Eigen::Vector3f trans = corr_rej_trans.block<3,1>(0,3);
        Eigen::Quaternionf quat(corr_rej_trans.block<3,3>(0,0));
        quat.normalize();
        Eigen::Quaternionf quat_conj = quat.conjugate();

        for(size_t t=0; t < found_transformations_.size(); t++)
        {
          Eigen::Vector3f trans_found = found_transformations_[t].block<3,1>(0,3);
          if((trans - trans_found).norm() < gc_size_)
          {
            found = true;
            break;

            Eigen::Quaternionf quat_found(found_transformations_[t].block<3,3>(0,0));
            quat_found.normalize();
            Eigen::Quaternionf quat_prod = quat_found * quat_conj;
            double angle = acos(quat_prod.z());
            if(std::abs(pcl::rad2deg(angle)) < 5.0)
            {
            }
          }
        }

        return found;
      }

      inline
      void setPrune(bool b)
      {
        prune_ = b;
      }

      inline void
      setUseGraph (bool b)
      {
        use_graph_ = b;
      }

      inline
      void setDotDistance(double t)
      {
        thres_dot_distance_ = t;
      }

      /** \brief Sets the minimum cluster size
        * \param[in] threshold the minimum cluster size
        */
      inline void
      setRansacThreshold (double threshold)
      {
        ransac_threshold_ = threshold;
      }
      /** \brief Sets the minimum cluster size
        * \param[in] threshold the minimum cluster size 
        */
      inline void
      setGCThreshold (int threshold)
      {
        gc_threshold_ = threshold;
      }

      /** \brief Gets the minimum cluster size.
        * 
        * \return the minimum cluster size used by GC.
        */
      inline int
      getGCThreshold () const
      {
        return (gc_threshold_);
      }

      /** \brief Sets the consensus set resolution. This should be in metric units.
        * 
        * \param[in] gc_size consensus set resolution.
        */
      inline void
      setGCSize (double gc_size)
      {
        gc_size_ = gc_size;
      }

      /** \brief Gets the consensus set resolution.
        * 
        * \return the consensus set resolution.
        */
      inline double
      getGCSize () const
      {
        return (gc_size_);
      }

      /** \brief The main function, recognizes instances of the model into the scene set by the user.
        * 
        * \param[out] transformations a vector containing one transformation matrix for each instance of the model recognized into the scene.
        *
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      bool
      recognize (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations);

      /** \brief The main function, recognizes instances of the model into the scene set by the user.
        * 
        * \param[out] transformations a vector containing one transformation matrix for each instance of the model recognized into the scene.
        * \param[out] clustered_corrs a vector containing the correspondences for each instance of the model found within the input data (the same output of clusterCorrespondences).
        *
        * \return true if the recognition had been successful or false if errors have occurred.
        */
      bool
      recognize (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &transformations, std::vector<pcl::Correspondences> &clustered_corrs);

      void setInputAndSceneNormals(pcl::PointCloud<pcl::Normal>::Ptr & input_n, pcl::PointCloud<pcl::Normal>::Ptr & scene_n)
      {
        input_normals_ = input_n;
        scene_normals_ = scene_n;
      }

      void
      setVisualizeGraph(bool vis)
      {
        visualize_graph_ = vis;
      }

    protected:
      using faat_pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::input_;
      using faat_pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::scene_;
      using faat_pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::model_scene_corrs_;
      using faat_pcl::CorrespondenceGrouping<PointModelT, PointSceneT>::require_normals_;

      pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;

      pcl::PointCloud<pcl::Normal>::Ptr input_normals_;
      /** \brief Minimum cluster size. It shouldn't be less than 3, since at least 3 correspondences are needed to compute the 6DOF pose */
      int gc_threshold_;

      /** \brief Resolution of the consensus set used to cluster correspondences together*/
      double gc_size_;

      double ransac_threshold_;

      double thres_dot_distance_;

      bool use_graph_;
      bool visualize_graph_;

      bool prune_;
      bool prune_by_CC_;

      float dist_for_cluster_factor_;
      int max_taken_correspondence_;
      bool cliques_big_to_small_;
      bool check_normals_orientation_;

      /** \brief Transformations found by clusterCorrespondences method. */
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > found_transformations_;

      /** \brief Cluster the input correspondences in order to distinguish between different instances of the model into the scene.
        * 
        * \param[out] model_instances a vector containing the clustered correspondences for each model found on the scene.
        * \return true if the clustering had been successful or false if errors have occurred.
        */ 
      void
      clusterCorrespondences (std::vector<pcl::Correspondences> &model_instances);

      /*void visualizeCorrespondences(const pcl::Correspondences & correspondences);

      void visualizeGraph(GraphGGCG & g, std::string title="Correspondence Graph");*/
  };
}

#endif // FAAT_PCL_RECOGNITION_SI_GEOMETRIC_CONSISTENCY_H_
