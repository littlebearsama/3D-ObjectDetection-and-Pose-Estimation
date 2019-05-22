/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2012 Aitor Aldoma, Federico Tombari
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
 */

#include <pcl/visualization/pcl_visualizer.h>
#include "ghv.h"
#include <functional>
#include <numeric>
#include <pcl/common/time.h>
#include <boost/graph/connected_components.hpp>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/common/angles.h>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_01.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/adjacency_matrix.hpp>

template<typename ModelT, typename SceneT>
mets::gol_type
faat_pcl::GHV<ModelT, SceneT>::evaluateSolution (const std::vector<bool> & active, int changed)
{
    //boost::posix_time::ptime start_time (boost::posix_time::microsec_clock::local_time ());
    float sign = 1.f;
    //update explained_by_RM
    if (active[changed])
    {
        //it has been activated
        updateExplainedVector (recognition_models_[changed]->explained_, recognition_models_[changed]->explained_distances_, explained_by_RM_,
                               explained_by_RM_distance_weighted, 1.f, changed);

        if(detect_clutter_) {
            updateUnexplainedVector (recognition_models_[changed]->unexplained_in_neighborhood,
                                     recognition_models_[changed]->unexplained_in_neighborhood_weights, unexplained_by_RM_neighboorhods,
                                     recognition_models_[changed]->explained_, explained_by_RM_, 1.f);
        }

        updateCMDuplicity (recognition_models_[changed]->complete_cloud_occupancy_indices_, complete_cloud_occupancy_by_RM_, 1.f);
    }
    else
    {
        //it has been deactivated
        updateExplainedVector (recognition_models_[changed]->explained_, recognition_models_[changed]->explained_distances_, explained_by_RM_,
                               explained_by_RM_distance_weighted, -1.f, changed);

        if(detect_clutter_) {
            updateUnexplainedVector (recognition_models_[changed]->unexplained_in_neighborhood,
                                     recognition_models_[changed]->unexplained_in_neighborhood_weights, unexplained_by_RM_neighboorhods,
                                     recognition_models_[changed]->explained_, explained_by_RM_, -1.f);
        }
        updateCMDuplicity (recognition_models_[changed]->complete_cloud_occupancy_indices_, complete_cloud_occupancy_by_RM_, -1.f);
        sign = -1.f;
    }

    double duplicity = static_cast<double> (getDuplicity ());
    //duplicity = 0.f; //ATTENTION!!
    double good_info = getExplainedValue ();

    double unexplained_info = getPreviousUnexplainedValue ();
    if(!detect_clutter_) {
        unexplained_info = 0;
    }

    double bad_info = static_cast<double> (getPreviousBadInfo ()) + (recognition_models_[changed]->outliers_weight_
                                                                     * static_cast<double> (recognition_models_[changed]->bad_information_)) * sign;

    setPreviousBadInfo (bad_info);

    double duplicity_cm = static_cast<double> (getDuplicityCM ()) * w_occupied_multiple_cm_;
    //float duplicity_cm = 0;

    //boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
    //std::cout << (end_time - start_time).total_microseconds () << " microsecs" << std::endl;
    double cost = (good_info - bad_info - duplicity - unexplained_info - duplicity_cm - countActiveHypotheses (active) - countPointsOnDifferentPlaneSides(active)) * -1.f;
    if(cost_logger_) {
        cost_logger_->increaseEvaluated();
        cost_logger_->addCostEachTimeEvaluated(cost);
    }

    //ntimes_evaluated_++;
    return static_cast<mets::gol_type> (cost); //return the dual to our max problem
}


template<typename ModelT, typename SceneT>
double
faat_pcl::GHV<ModelT, SceneT>::countActiveHypotheses (const std::vector<bool> & sol)
{
    double c = 0;
    for (size_t i = 0; i < sol.size (); i++)
    {
        if (sol[i]) {
            //c++;
            //c += static_cast<double>(recognition_models_[i]->explained_.size()) * active_hyp_penalty_ + min_contribution_;
            c += static_cast<double>(recognition_models_[i]->explained_.size()) / 2.f * recognition_models_[i]->hyp_penalty_ + min_contribution_;
        }
    }

    return c;
    //return static_cast<float> (c) * active_hyp_penalty_;
}

template<typename ModelT, typename SceneT>
double
faat_pcl::GHV<ModelT, SceneT>::
countPointsOnDifferentPlaneSides (const std::vector<bool> & sol,
                                  bool print)
{
    if(!use_points_on_plane_side_)
        return 0;

    size_t recog_models = recognition_models_.size() - planar_models_.size();
    double c=0;
    for(size_t i=0; i < planar_models_.size(); i++)
    {
        if( (recog_models + i) >= recognition_models_.size() )
        {
            std::cout << "i:" << i << std::endl;
            std::cout << "recog models:" << recog_models << std::endl;
            std::cout << "recogition models:" << recognition_models_.size() << std::endl;
            std::cout << "solution size:" << sol.size() << std::endl;
            std::cout << "planar models size:" << planar_models_.size() << std::endl;
        }

        assert( (recog_models + i) < recognition_models_.size());
        assert( (recog_models + i) < sol.size());
        if(sol[recog_models + i])
        {
            std::map<int, int>::iterator it1;
            it1 = model_to_planar_model_.find(static_cast<int>(recog_models + i));

            if(print)
                std::cout << "plane is active:" << recognition_models_[recog_models + i]->id_s_ << std::endl;

            for(size_t j=0; j < points_one_plane_sides_[it1->second].size(); j++)
            {
                if(sol[j])
                {
                    c += points_one_plane_sides_[it1->second][j];
                    if(print)
                    {
                        std::cout << "Adding to c:" << points_one_plane_sides_[it1->second][j] << " " << recognition_models_[j]->id_s_ << " " << recognition_models_[j]->id_ << " plane_id:" << it1->second << std::endl;
                    }
                }
            }
        }
    }

    if(print)
    {
        for(size_t kk=0; kk < points_one_plane_sides_.size(); kk++)
        {
            for(size_t kkk=0; kkk < points_one_plane_sides_[kk].size(); kkk++)
            {
                std::cout << "i:" << kkk << " val:" << points_one_plane_sides_[kk][kkk] << " ";
            }

            std::cout << std::endl;
        }
    }
    return c;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::addPlanarModels(std::vector<faat_pcl::PlaneModel<ModelT> > & models)
{
    planar_models_ = models;
    model_to_planar_model_.clear();
    //iterate through the planar models and append them to complete_models_?

    size_t size_start = visible_models_.size();
    for(size_t i=0; i < planar_models_.size(); i++)
    {
        model_to_planar_model_[static_cast<int>(size_start + i)] = static_cast<int>(i);
        complete_models_.push_back(planar_models_[i].plane_cloud_);

        faat_pcl::occlusion_reasoning::ZBuffering<ModelT, SceneT> zbuffer_scene (zbuffer_scene_resolution_, zbuffer_scene_resolution_, 1.f);
        if (!occlusion_cloud_->isOrganized ())
        {
            zbuffer_scene.computeDepthMap (occlusion_cloud_, true);
        }

        //self-occlusions
        typename pcl::PointCloud<ModelT>::Ptr filtered (new pcl::PointCloud<ModelT> (*(planar_models_[i].plane_cloud_)));
        typename pcl::PointCloud<ModelT>::ConstPtr const_filtered(new pcl::PointCloud<ModelT> (*filtered));

        std::vector<int> indices_cloud_occlusion;
        if (occlusion_cloud_->isOrganized ())
        {
            filtered = faat_pcl::occlusion_reasoning::filter<ModelT,SceneT> (occlusion_cloud_, const_filtered, 525.f, occlusion_thres_, indices_cloud_occlusion);
        }
        else
        {
            zbuffer_scene.filter (const_filtered, filtered, occlusion_thres_);
        }

        visible_models_.push_back (filtered);

        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
        normals->points.resize(filtered->points.size());
        Eigen::Vector3f plane_normal;
        plane_normal[0] = planar_models_[i].coefficients_.values[0];
        plane_normal[1] = planar_models_[i].coefficients_.values[1];
        plane_normal[2] = planar_models_[i].coefficients_.values[2];

        for(size_t k=0; k < filtered->points.size(); k++)
        {
            normals->points[k].getNormalVector3fMap() = plane_normal;
        }

        visible_normal_models_.push_back(normals);
    }
}

//clutter segmentation performed by supervoxels is only allowed with pcl::PointXYZRGB types
//this happens only for PCL versions below 1.7.2, since pcl::SuperVoxelClustering cannot be instantiated
//thats the reason for this hacks...

template<typename SceneT>
bool
isSuperVoxelClutterSegmentationPossible()
{
    return true;
}

template<>
bool
isSuperVoxelClutterSegmentationPossible<pcl::PointXYZ>()
{
    return false;
}

template<typename SceneT>
void
superVoxelClutterSegmentation(typename pcl::PointCloud<SceneT>::Ptr & scene_cloud_downsampled_,
                              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & clusters_cloud_rgb_,
                              pcl::PointCloud<pcl::PointXYZL>::Ptr & clusters_cloud_,
                              float radius_neighborhood_GO_)
{
    float voxel_resolution = 0.005f;
    float seed_resolution = radius_neighborhood_GO_;
    typename pcl::SupervoxelClustering<SceneT> super (voxel_resolution, seed_resolution, false);
    super.setInputCloud (scene_cloud_downsampled_);
    super.setColorImportance (0.f);
    super.setSpatialImportance (1.f);
    super.setNormalImportance (1.f);
    std::map <uint32_t, typename pcl::Supervoxel<SceneT>::Ptr > supervoxel_clusters;
    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());
    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxels_labels_cloud = super.getLabeledCloud();

    //merge faces...
    uint32_t max_label = super.getMaxLabel();

    pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

    std::vector<int> label_to_idx;
    label_to_idx.resize(max_label + 1, -1);
    typename std::map <uint32_t, typename pcl::Supervoxel<SceneT>::Ptr>::iterator sv_itr,sv_itr_end;
    sv_itr = supervoxel_clusters.begin ();
    sv_itr_end = supervoxel_clusters.end ();
    int i=0;
    for ( ; sv_itr != sv_itr_end; ++sv_itr, i++)
    {
        label_to_idx[sv_itr->first] = i;
    }

    std::vector< std::vector<bool> > adjacent;
    adjacent.resize(supervoxel_clusters.size());
    for(size_t i=0; i < (supervoxel_clusters.size()); i++)
        adjacent[i].resize(supervoxel_clusters.size(), false);

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    std::cout << "super voxel adjacency size:" << supervoxel_adjacency.size() << std::endl;
    for ( ; label_itr != supervoxel_adjacency.end (); )
    {
        //First get the label
        uint32_t supervoxel_label = label_itr->first;
        Eigen::Vector3f normal_super_voxel = sv_normal_cloud->points[label_to_idx[supervoxel_label]].getNormalVector3fMap();
        normal_super_voxel.normalize();
        //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
        std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
        for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
        {
            Eigen::Vector3f normal_neighbor_supervoxel = sv_normal_cloud->points[label_to_idx[adjacent_itr->second]].getNormalVector3fMap();
            normal_neighbor_supervoxel.normalize();

            if(normal_super_voxel.dot(normal_neighbor_supervoxel) > 0.95f)
            {
                adjacent[label_to_idx[supervoxel_label]][label_to_idx[adjacent_itr->second]] = true;
            }
        }

        //Move iterator forward to next label
        label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    typedef boost::adjacency_matrix<boost::undirectedS, int> Graph;
    Graph G(supervoxel_clusters.size());
    for(size_t i=0; i < supervoxel_clusters.size(); i++)
    {
        for(size_t j=(i+1); j < supervoxel_clusters.size(); j++)
        {
            if(adjacent[i][j])
                boost::add_edge(i, j, G);
        }
    }

    std::vector<int> components (boost::num_vertices (G));
    int n_cc = static_cast<int> (boost::connected_components (G, &components[0]));
    std::cout << "Number of connected components..." << n_cc << std::endl;

    std::vector<int> cc_sizes;
    std::vector<std::vector<int> > ccs;
    std::vector<uint32_t> original_labels_to_merged;
    original_labels_to_merged.resize(supervoxel_clusters.size());

    ccs.resize(n_cc);
    cc_sizes.resize (n_cc, 0);
    typename boost::graph_traits<Graph>::vertex_iterator vertexIt, vertexEnd;
    boost::tie (vertexIt, vertexEnd) = vertices (G);
    for (; vertexIt != vertexEnd; ++vertexIt)
    {
        int c = components[*vertexIt];
        cc_sizes[c]++;
        ccs[c].push_back(*vertexIt);
        original_labels_to_merged[*vertexIt] = c;
    }

    for(size_t i=0; i < supervoxels_labels_cloud->points.size(); i++)
    {
        //std::cout << supervoxels_labels_cloud->points[i].label << " " << label_to_idx.size() << " " << original_labels_to_merged.size() << " " << label_to_idx[supervoxels_labels_cloud->points[i].label] << std::endl;
        if(label_to_idx[supervoxels_labels_cloud->points[i].label] < 0)
            continue;

        supervoxels_labels_cloud->points[i].label = original_labels_to_merged[label_to_idx[supervoxels_labels_cloud->points[i].label]];
    }

    std::cout << scene_cloud_downsampled_->points.size () << " " << supervoxels_labels_cloud->points.size () << std::endl;

    //clusters_cloud_rgb_= super.getColoredCloud();

    clusters_cloud_.reset (new pcl::PointCloud<pcl::PointXYZL>(*supervoxels_labels_cloud));

    std::vector<uint32_t> label_colors_;
    //int max_label = label;
    label_colors_.reserve (max_label + 1);
    srand (static_cast<unsigned int> (time (0)));
    while (label_colors_.size () <= max_label )
    {
        uint8_t r = static_cast<uint8_t>( (rand () % 256));
        uint8_t g = static_cast<uint8_t>( (rand () % 256));
        uint8_t b = static_cast<uint8_t>( (rand () % 256));
        label_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    }

    clusters_cloud_rgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    clusters_cloud_rgb_->points.resize (scene_cloud_downsampled_->points.size ());
    clusters_cloud_rgb_->width = scene_cloud_downsampled_->points.size();
    clusters_cloud_rgb_->height = 1;

    {
        for(size_t i=0; i < clusters_cloud_->points.size(); i++)
        {
            //if (clusters_cloud_->points[i].label != 0)
            //{
            clusters_cloud_rgb_->points[i].getVector3fMap() = clusters_cloud_->points[i].getVector3fMap();
            clusters_cloud_rgb_->points[i].rgb = label_colors_[clusters_cloud_->points[i].label];

            //}
        }
    }
}

template<>
void
superVoxelClutterSegmentation<pcl::PointXYZ>
                              (pcl::PointCloud<pcl::PointXYZ>::Ptr & /*scene_cloud_downsampled_*/,
                              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr & /*clusters_cloud_rgb_*/,
                              pcl::PointCloud<pcl::PointXYZL>::Ptr & /*clusters_cloud_*/,
                              float /*radius_neighborhood_GO_*/)
{

    PCL_WARN("Super voxel clutter segmentation not allowed for pcl::PointXYZ scene types\n");

}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::initialize ()
{

    //clear stuff
    recognition_models_.clear ();
    unexplained_by_RM_neighboorhods.clear ();
    explained_by_RM_distance_weighted.clear ();
    previous_explained_by_RM_distance_weighted.clear ();
    explained_by_RM_.clear ();
    explained_by_RM_model.clear();
    mask_.clear ();
    //indices_.clear ();
    complete_cloud_occupancy_by_RM_.clear ();

    // initialize mask to false
    mask_.resize (complete_models_.size ());
    for (size_t i = 0; i < complete_models_.size (); i++)
        mask_[i] = false;

    //indices_.resize (complete_models_.size ());

    if(!scene_and_normals_set_from_outside_)
    {
        pcl::ScopeTime t("compute scene normals");
        NormalEstimator_ n3d;
        scene_normals_.reset (new pcl::PointCloud<pcl::Normal> ());

        int j = 0;
        for (size_t i = 0; i < scene_cloud_downsampled_->points.size (); ++i) {
            if (!pcl_isfinite (scene_cloud_downsampled_->points[i].x) || !pcl_isfinite (scene_cloud_downsampled_->points[i].y)
                    || !pcl_isfinite (scene_cloud_downsampled_->points[i].z)) {
                std::cout << "Not finite..." << std::endl;
                continue;
            }

            scene_cloud_downsampled_->points[j] = scene_cloud_downsampled_->points[i];

            j++;
        }

        scene_cloud_downsampled_->points.resize(j);
        scene_cloud_downsampled_->width = j;
        scene_cloud_downsampled_->height = 1;

        std::cout << "scene points after removing NaNs:" << scene_cloud_downsampled_->points.size() << std::endl;

        typename pcl::search::KdTree<SceneT>::Ptr normals_tree (new pcl::search::KdTree<SceneT>);
        normals_tree->setInputCloud (scene_cloud_downsampled_);

        n3d.setRadiusSearch (radius_normals_);
        n3d.setSearchMethod (normals_tree);
        n3d.setInputCloud (scene_cloud_downsampled_);
        n3d.compute (*scene_normals_);

        //check nans...
        j = 0;
        for (size_t i = 0; i < scene_normals_->points.size (); ++i)
        {
            if (!pcl_isfinite (scene_normals_->points[i].normal_x) || !pcl_isfinite (scene_normals_->points[i].normal_y)
                    || !pcl_isfinite (scene_normals_->points[i].normal_z))
                continue;

            scene_normals_->points[j] = scene_normals_->points[i];
            scene_cloud_downsampled_->points[j] = scene_cloud_downsampled_->points[i];
            scene_sampled_indices_[j] = scene_sampled_indices_[i];
            j++;
        }

        scene_sampled_indices_.resize(j);

        scene_normals_->points.resize (j);
        scene_normals_->width = j;
        scene_normals_->height = 1;

        scene_cloud_downsampled_->points.resize (j);
        scene_cloud_downsampled_->width = j;
        scene_cloud_downsampled_->height = 1;
    }
    else
    {
        PCL_WARN("Scene and normals set from outside\n");
        scene_sampled_indices_.resize(scene_cloud_downsampled_->points.size());
        for(size_t k=0; k < scene_cloud_downsampled_->points.size(); k++)
        {
            scene_sampled_indices_[k] = k;
        }
    }

    scene_curvature_.resize (scene_cloud_downsampled_->points.size (), 0);
    for(size_t k=0; k < scene_normals_->points.size(); k++)
    {
        scene_curvature_[k] = scene_normals_->points[k].curvature;
    }

    explained_by_RM_.resize (scene_cloud_downsampled_->points.size (), 0);
    duplicates_by_RM_weighted_.resize (scene_cloud_downsampled_->points.size (), 0);
    explained_by_RM_model.resize (scene_cloud_downsampled_->points.size (), -1);
    explained_by_RM_distance_weighted.resize (scene_cloud_downsampled_->points.size (), 0);
    previous_explained_by_RM_distance_weighted.resize (scene_cloud_downsampled_->points.size ());
    unexplained_by_RM_neighboorhods.resize (scene_cloud_downsampled_->points.size (), 0.f);

    octree_scene_downsampled_.reset(new pcl::octree::OctreePointCloudSearch<SceneT>(0.01f));
    octree_scene_downsampled_->setInputCloud(scene_cloud_downsampled_);
    octree_scene_downsampled_->addPointsFromInputCloud();

    //compute segmentation of the scene if detect_clutter_
    if (detect_clutter_)
    {
        pcl::ScopeTime t("Smooth segmentation of the scene");
        //initialize kdtree for search

        scene_downsampled_tree_.reset (new pcl::search::KdTree<SceneT>);
        scene_downsampled_tree_->setInputCloud (scene_cloud_downsampled_);

        if(use_super_voxels_)
        {
            //check if its possible at all
            bool possible = isSuperVoxelClutterSegmentationPossible<SceneT>();
            if(!possible)
            {
                PCL_WARN("Not possible to use superVoxelClutter segmentation for pcl::PointXYZ types with pcl version < 1.7.2\n");
                PCL_WARN("See comments in code (ghv.hpp)\n");
                use_super_voxels_ = false;
            }
        }

        if(use_super_voxels_)
        {
            superVoxelClutterSegmentation<SceneT>(scene_cloud_downsampled_, clusters_cloud_rgb_,
                                                  clusters_cloud_, radius_neighborhood_GO_);
        }
        else
        {

            clusters_cloud_.reset (new pcl::PointCloud<pcl::PointXYZL>);
            clusters_cloud_rgb_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

            if(scene_cloud_->isOrganized() && scene_normals_for_clutter_term_ && (scene_normals_for_clutter_term_->points.size() == scene_cloud_->points.size()))
            {
                PCL_WARN("scene cloud is organized, filter points with high curvature and cluster the rest in smooth patches\n");

                typename pcl::EuclideanClusterComparator<SceneT, pcl::Normal, pcl::Label>::Ptr
                        euclidean_cluster_comparator (new pcl::EuclideanClusterComparator<SceneT, pcl::Normal, pcl::Label> ());

                pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
                labels->points.resize(scene_cloud_->points.size());
                labels->width = scene_cloud_->width;
                labels->height = scene_cloud_->height;
                labels->is_dense = scene_cloud_->is_dense;

                for (size_t j = 0; j < scene_cloud_->points.size (); j++)
                {
                    const Eigen::Vector3f& xyz_p = scene_cloud_->points[j].getVector3fMap ();
                    if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
                    {
                        //label_indices[0].indices[label_count[0]++] = static_cast<int>(j);
                        labels->points[j].label = 0;
                        continue;
                    }

                    //check normal
                    const Eigen::Vector3f& normal = scene_normals_for_clutter_term_->points[j].getNormalVector3fMap ();
                    if (!pcl_isfinite (normal[0]) || !pcl_isfinite (normal[1]) || !pcl_isfinite (normal[2]))
                    {
                        //label_indices[0].indices[label_count[0]++] = static_cast<int>(j);
                        labels->points[j].label = 0;
                        continue;
                    }

                    //check curvature
                    float curvature = scene_normals_for_clutter_term_->points[j].curvature;
                    if(curvature > (curvature_threshold_ * (std::min(1.f,scene_cloud_->points[j].z))))
                    {
                        //label_indices[0].indices[label_count[0]++] = static_cast<int>(j);
                        labels->points[j].label = 0;
                        continue;
                    }

                    //label_indices[1].indices[label_count[1]++] = static_cast<int>(j);
                    labels->points[j].label = 1;
                }

                std::vector<bool> excluded_labels;
                excluded_labels.resize (2, false);
                excluded_labels[0] = true;

                euclidean_cluster_comparator->setInputCloud (scene_cloud_);
                euclidean_cluster_comparator->setLabels (labels);
                euclidean_cluster_comparator->setExcludeLabels (excluded_labels);
                euclidean_cluster_comparator->setDistanceThreshold (cluster_tolerance_, true);
                euclidean_cluster_comparator->setAngularThreshold(0.017453 * 5.f); //5 degrees

                pcl::PointCloud<pcl::Label> euclidean_labels;
                std::vector<pcl::PointIndices> clusters;
                pcl::OrganizedConnectedComponentSegmentation<SceneT, pcl::Label> euclidean_segmentation (euclidean_cluster_comparator);
                euclidean_segmentation.setInputCloud (scene_cloud_);
                euclidean_segmentation.segment (euclidean_labels, clusters);

                std::cout << "Number of clusters:" << clusters.size() << std::endl;
                std::vector<bool> good_cluster(clusters.size(), false);
                for (size_t i = 0; i < clusters.size (); i++)
                {
                    if (clusters[i].indices.size () >= 100)
                        good_cluster[i] = true;
                }

                clusters_cloud_->points.resize (scene_sampled_indices_.size ());
                clusters_cloud_->width = scene_sampled_indices_.size();
                clusters_cloud_->height = 1;

                clusters_cloud_rgb_->points.resize (scene_sampled_indices_.size ());
                clusters_cloud_rgb_->width = scene_sampled_indices_.size();
                clusters_cloud_rgb_->height = 1;

                pcl::PointCloud<pcl::PointXYZL>::Ptr clusters_cloud (new pcl::PointCloud<pcl::PointXYZL>);
                clusters_cloud->points.resize (scene_cloud_->points.size ());
                clusters_cloud->width = scene_cloud_->points.size();
                clusters_cloud->height = 1;

                for (size_t i = 0; i < scene_cloud_->points.size (); i++)
                {
                    pcl::PointXYZL p;
                    p.getVector3fMap () = scene_cloud_->points[i].getVector3fMap ();
                    p.label = 0;
                    clusters_cloud->points[i] = p;
                    //clusters_cloud_rgb_->points[i].getVector3fMap() = p.getVector3fMap();
                    //clusters_cloud_rgb_->points[i].r = clusters_cloud_rgb_->points[i].g = clusters_cloud_rgb_->points[i].b = 100;
                }

                uint32_t label = 1;
                for (size_t i = 0; i < clusters.size (); i++)
                {
                    if(!good_cluster[i])
                        continue;

                    for (size_t j = 0; j < clusters[i].indices.size (); j++)
                    {
                        clusters_cloud->points[clusters[i].indices[j]].label = label;
                    }
                    label++;
                }

                std::vector<uint32_t> label_colors_;
                int max_label = label;
                label_colors_.reserve (max_label + 1);
                srand (static_cast<unsigned int> (time (0)));
                while (label_colors_.size () <= max_label )
                {
                    uint8_t r = static_cast<uint8_t>( (rand () % 256));
                    uint8_t g = static_cast<uint8_t>( (rand () % 256));
                    uint8_t b = static_cast<uint8_t>( (rand () % 256));
                    label_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                }

                if(scene_cloud_downsampled_->points.size() != scene_sampled_indices_.size())
                {
                    std::cout << scene_cloud_downsampled_->points.size() << " " << scene_sampled_indices_.size() << std::endl;
                    assert(scene_cloud_downsampled_->points.size() == scene_sampled_indices_.size());
                }


                for(size_t i=0; i < scene_sampled_indices_.size(); i++)
                {
                    clusters_cloud_->points[i] = clusters_cloud->points[scene_sampled_indices_[i]];
                    clusters_cloud_rgb_->points[i].getVector3fMap() = clusters_cloud->points[scene_sampled_indices_[i]].getVector3fMap();

                    if(clusters_cloud->points[scene_sampled_indices_[i]].label == 0)
                    {
                        clusters_cloud_rgb_->points[i].r = clusters_cloud_rgb_->points[i].g = clusters_cloud_rgb_->points[i].b = 100;
                    }
                    else
                    {
                        clusters_cloud_rgb_->points[i].rgb = label_colors_[clusters_cloud->points[scene_sampled_indices_[i]].label];
                    }
                }
            }
            else
            {

                std::vector<pcl::PointIndices> clusters;
                extractEuclideanClustersSmooth<SceneT, pcl::Normal> (*scene_cloud_downsampled_, *scene_normals_, cluster_tolerance_,
                                                                     scene_downsampled_tree_, clusters, eps_angle_threshold_, curvature_threshold_, min_points_);

                clusters_cloud_->points.resize (scene_cloud_downsampled_->points.size ());
                clusters_cloud_->width = scene_cloud_downsampled_->width;
                clusters_cloud_->height = 1;

                clusters_cloud_rgb_->points.resize (scene_cloud_downsampled_->points.size ());
                clusters_cloud_rgb_->width = scene_cloud_downsampled_->width;
                clusters_cloud_rgb_->height = 1;

                for (size_t i = 0; i < scene_cloud_downsampled_->points.size (); i++)
                {
                    pcl::PointXYZL p;
                    p.getVector3fMap () = scene_cloud_downsampled_->points[i].getVector3fMap ();
                    p.label = 0;
                    clusters_cloud_->points[i] = p;
                    clusters_cloud_rgb_->points[i].getVector3fMap() = p.getVector3fMap();
                    clusters_cloud_rgb_->points[i].r = clusters_cloud_rgb_->points[i].g = clusters_cloud_rgb_->points[i].b = 100;
                }

                uint32_t label = 1;
                for (size_t i = 0; i < clusters.size (); i++)
                {
                    for (size_t j = 0; j < clusters[i].indices.size (); j++)
                        clusters_cloud_->points[clusters[i].indices[j]].label = label;
                    label++;
                }

                std::vector<uint32_t> label_colors_;
                int max_label = label;
                label_colors_.reserve (max_label + 1);
                srand (static_cast<unsigned int> (time (0)));
                while (label_colors_.size () <= max_label )
                {
                    uint8_t r = static_cast<uint8_t>( (rand () % 256));
                    uint8_t g = static_cast<uint8_t>( (rand () % 256));
                    uint8_t b = static_cast<uint8_t>( (rand () % 256));
                    label_colors_.push_back (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
                }

                {
                    for(size_t i=0; i < clusters_cloud_->points.size(); i++)
                    {
                        if (clusters_cloud_->points[i].label != 0)
                        {
                            clusters_cloud_rgb_->points[i].rgb = label_colors_[clusters_cloud_->points[i].label];
                        }
                    }
                }
            }
        }

        max_label_clusters_cloud_ = 0;
        for(size_t i=0; i < clusters_cloud_->points.size(); i++)
        {
            if (clusters_cloud_->points[i].label > max_label_clusters_cloud_)
            {
                max_label_clusters_cloud_ = clusters_cloud_->points[i].label;
            }
        }

        std::cout << "MAX LABEL clusters cloud:" << max_label_clusters_cloud_ << std::endl;
    }

    //compute scene LAB values
    if(!ignore_color_even_if_exists_)
    {
        bool exists_s;
        float rgb_s;
        scene_LAB_values_.resize(scene_cloud_downsampled_->points.size());
        scene_RGB_values_.resize(scene_cloud_downsampled_->points.size());
        scene_GS_values_.resize(scene_cloud_downsampled_->points.size());
        for(size_t i=0; i < scene_cloud_downsampled_->points.size(); i++)
        {
            pcl::for_each_type<FieldListS> (
                        pcl::CopyIfFieldExists<typename CloudS::PointType, float> (scene_cloud_downsampled_->points[i],
                                                                                   "rgb", exists_s, rgb_s));

            if (exists_s)
            {

                uint32_t rgb = *reinterpret_cast<int*> (&rgb_s);
                unsigned char rs = (rgb >> 16) & 0x0000ff;
                unsigned char gs = (rgb >> 8) & 0x0000ff;
                unsigned char bs = (rgb) & 0x0000ff;

                float LRefs, aRefs, bRefs;

                RGB2CIELAB (rs, gs, bs, LRefs, aRefs, bRefs);
                LRefs /= 100.0f; aRefs /= 120.0f; bRefs /= 120.0f;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

                scene_LAB_values_[i] = (Eigen::Vector3f(LRefs, aRefs, bRefs));

                float rsf,gsf,bsf;
                rsf = static_cast<float>(rs) / 255.f;
                gsf = static_cast<float>(gs) / 255.f;
                bsf = static_cast<float>(bs) / 255.f;
                scene_RGB_values_[i] = (Eigen::Vector3f(rsf,gsf,bsf));

                scene_GS_values_[i] = (rsf + gsf + bsf) / 3.f;
            }
        }
    }

    //compute cues
    {
        std::cout << "specify histogram:" << use_histogram_specification_ << " smooth faces size:" << models_smooth_faces_.size() << " color space:" << color_space_ << std::endl;

        valid_model_.resize(complete_models_.size (), true);
        {
            pcl::ScopeTime tcues ("Computing cues");
            recognition_models_.resize (complete_models_.size ());
#pragma omp parallel for schedule(dynamic, 1) num_threads(std::min(max_threads_, omp_get_num_procs()))
            for (int i = 0; i < static_cast<int> (complete_models_.size ()); i++)
            {
                //create recognition model
                recognition_models_[i].reset (new GHVRecognitionModel<ModelT> ());
                if(!addModel(i, recognition_models_[i])) {
                    valid_model_[i] = false;
                    PCL_WARN("Model is not valid\n");
                }
            }
        }

        //compute the bounding boxes for the models
        {
            pcl::ScopeTime tcues ("complete_cloud_occupancy_by_RM_");
            ModelT min_pt_all, max_pt_all;
            min_pt_all.x = min_pt_all.y = min_pt_all.z = std::numeric_limits<float>::max ();
            max_pt_all.x = max_pt_all.y = max_pt_all.z = (std::numeric_limits<float>::max () - 0.001f) * -1;

            for (size_t i = 0; i < recognition_models_.size (); i++)
            {
                if(!valid_model_[i])
                    continue;

                ModelT min_pt, max_pt;
                pcl::getMinMax3D (*complete_models_[i], min_pt, max_pt);
                if (min_pt.x < min_pt_all.x)
                    min_pt_all.x = min_pt.x;

                if (min_pt.y < min_pt_all.y)
                    min_pt_all.y = min_pt.y;

                if (min_pt.z < min_pt_all.z)
                    min_pt_all.z = min_pt.z;

                if (max_pt.x > max_pt_all.x)
                    max_pt_all.x = max_pt.x;

                if (max_pt.y > max_pt_all.y)
                    max_pt_all.y = max_pt.y;

                if (max_pt.z > max_pt_all.z)
                    max_pt_all.z = max_pt.z;
            }

            int size_x, size_y, size_z;
            size_x = static_cast<int> (std::ceil (std::abs (max_pt_all.x - min_pt_all.x) / res_occupancy_grid_)) + 1;
            size_y = static_cast<int> (std::ceil (std::abs (max_pt_all.y - min_pt_all.y) / res_occupancy_grid_)) + 1;
            size_z = static_cast<int> (std::ceil (std::abs (max_pt_all.z - min_pt_all.z) / res_occupancy_grid_)) + 1;

            complete_cloud_occupancy_by_RM_.resize (size_x * size_y * size_z, 0);

            for (size_t i = 0; i < recognition_models_.size (); i++)
            {
                if(!valid_model_[i])
                    continue;

                std::vector<bool> banned_vector(size_x * size_y * size_z, false);

                recognition_models_[i]->complete_cloud_occupancy_indices_.resize(complete_models_[i]->points.size ());
                int used = 0;

                for (size_t j = 0; j < complete_models_[i]->points.size (); j++)
                {
                    int pos_x, pos_y, pos_z;
                    pos_x = static_cast<int> (std::floor ((complete_models_[i]->points[j].x - min_pt_all.x) / res_occupancy_grid_));
                    pos_y = static_cast<int> (std::floor ((complete_models_[i]->points[j].y - min_pt_all.y) / res_occupancy_grid_));
                    pos_z = static_cast<int> (std::floor ((complete_models_[i]->points[j].z - min_pt_all.z) / res_occupancy_grid_));

                    int idx = pos_z * size_x * size_y + pos_y * size_x + pos_x;

                    assert(banned_vector.size() > idx);
                    if (!banned_vector[idx])
                    {
                        recognition_models_[i]->complete_cloud_occupancy_indices_[used] = idx;
                        banned_vector[idx] = true;
                        used++;
                    }
                }

                recognition_models_[i]->complete_cloud_occupancy_indices_.resize(used);
            }
        }
    }

    {
        pcl::ScopeTime tcues("Compute clutter cue at once");
        computeClutterCueAtOnce();
    }

    points_explained_by_rm_.clear ();
    points_explained_by_rm_.resize (scene_cloud_downsampled_->points.size ());
    for (size_t j = 0; j < recognition_models_.size (); j++)
    {
        boost::shared_ptr<GHVRecognitionModel<ModelT> > recog_model = recognition_models_[j];
        for (size_t i = 0; i < recog_model->explained_.size (); i++)
        {
            points_explained_by_rm_[recog_model->explained_[i]].push_back (recog_model);
        }
    }
}

template<typename ModelT, typename SceneT>
float
faat_pcl::GHV<ModelT, SceneT>::getCurvWeight(float p_curvature)
{

    if(multiple_assignment_penalize_by_one_ == 2)
        return 1.f;

    //return 1.f;

    /*if(p_curvature > duplicity_curvature_max)
        return 0.f;*/

    /*if(p_curvature < duplicity_curvature_max)
        return 1.f;*/

    float w = 1.f - std::min(1.f, p_curvature / duplicity_curvature_max_);
    return w;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::updateExplainedVector (std::vector<int> & vec,
                                                      std::vector<float> & vec_float, std::vector<int> & explained_,
                                                      std::vector<double> & explained_by_RM_distance_weighted, float sign, int model_id)
{
    double add_to_explained = 0;
    double add_to_duplicity_ = 0;
    int id_model_ = recognition_models_[model_id]->id_;

    for (size_t i = 0; i < vec.size (); i++)
    {
        bool prev_dup = explained_[vec[i]] > 1;
        //bool prev_explained = explained_[vec[i]] == 1;
        int prev_explained = explained_[vec[i]];
        double prev_explained_value = explained_by_RM_distance_weighted[vec[i]];

        explained_[vec[i]] += static_cast<int> (sign);
        //explained_by_RM_distance_weighted[vec[i]] += vec_float[i] * sign;

        if(sign > 0)
        {
            //adding, check that after adding the hypothesis, explained_by_RM_distance_weighted[vec[i]] is not higher than 1
            /*if(prev_explained_value + vec_float[i] > 1.f)
            {
                add_to_explained += std::max(0.0, 1 - prev_explained_value);
            }
            else
            {
                add_to_explained += vec_float[i];
            }*/

            if(prev_explained == 0)
            {
                //point was unexplained
                explained_by_RM_distance_weighted[vec[i]] = vec_float[i];
                previous_explained_by_RM_distance_weighted[vec[i]].push(std::make_pair(id_model_, vec_float[i]));
            }
            else
            {
                //point was already explained
                if(vec_float[i] > prev_explained_value)
                {
                    previous_explained_by_RM_distance_weighted[vec[i]].push(std::make_pair(id_model_, vec_float[i]));
                    explained_by_RM_distance_weighted[vec[i]] = (double)vec_float[i];
                }
                else
                {

                    //size_t anfang = previous_explained_by_RM_distance_weighted[vec[i]].size();

                    //if it is smaller, we should keep the value in case the greater value gets removed
                    //need to sort the stack
                    if(previous_explained_by_RM_distance_weighted[vec[i]].size() == 0)
                    {
                        previous_explained_by_RM_distance_weighted[vec[i]].push(std::make_pair(id_model_, vec_float[i]));
                    }
                    else
                    {
                        //sort and find the appropiate position

                        std::stack<std::pair<int, float>, std::vector<std::pair<int, float> > > kept;
                        while(previous_explained_by_RM_distance_weighted[vec[i]].size() > 0)
                        {
                            std::pair<int, double> p = previous_explained_by_RM_distance_weighted[vec[i]].top();
                            if(p.second < vec_float[i])
                            {
                                //should come here
                                break;
                            }

                            kept.push(p);
                            previous_explained_by_RM_distance_weighted[vec[i]].pop();
                        }

                        previous_explained_by_RM_distance_weighted[vec[i]].push(std::make_pair(id_model_, vec_float[i]));

                        while(!kept.empty())
                        {
                            previous_explained_by_RM_distance_weighted[vec[i]].push(kept.top());
                            kept.pop();
                        }
                    }
                }
            }
        }
        else
        {
            std::stack<std::pair<int, float>, std::vector<std::pair<int, float> > > kept;

            while(previous_explained_by_RM_distance_weighted[vec[i]].size() > 0)
            {
                std::pair<int, double> p = previous_explained_by_RM_distance_weighted[vec[i]].top();

                if(p.first == id_model_)
                {
                    //found it
                }
                else
                {
                    kept.push(p);
                }

                previous_explained_by_RM_distance_weighted[vec[i]].pop();
            }

            while(!kept.empty())
            {
                previous_explained_by_RM_distance_weighted[vec[i]].push(kept.top());
                kept.pop();
            }

            if(prev_explained == 1)
            {
                //was only explained by this hypothesis
                explained_by_RM_distance_weighted[vec[i]] = 0;
            }
            else
            {
                //there is at least another hypothesis explaining this point
                //assert(previous_explained_by_RM_distance_weighted[vec[i]].size() > 0);
                std::pair<int, double> p = previous_explained_by_RM_distance_weighted[vec[i]].top();

                double previous = p.second;
                explained_by_RM_distance_weighted[vec[i]] = previous;
            }

            //}

            /*if(prev_explained_value > 1.f)
            {
                if((prev_explained_value - vec_float[i]) < 1.f)
                {
                    add_to_explained -= 1.f - (prev_explained_value - vec_float[i]);
                }
            }
            else
            {
                add_to_explained -= vec_float[i];
            }*/
        }

        //add_to_explained += vec_float[i] * sign;
        //float curv_weight = std::min(duplicity_curvature_ - scene_curvature_[vec[i]], 0.f);
        float curv_weight = getCurvWeight(scene_curvature_[vec[i]]);
        /*if (explained_[vec[i]] == 1 && !prev_explained)
        {
            if (sign > 0)
            {
                add_to_explained += vec_float[i];
            }
            else
            {
                add_to_explained += explained_by_RM_distance_weighted[vec[i]];
            }
        }

        //hypotheses being removed, now the point is not explained anymore and was explained before by this hypothesis
        if ((sign < 0) && (explained_[vec[i]] == 0) && prev_explained)
        {
            //assert(prev_explained_value == vec_float[i]);
            add_to_explained -= prev_explained_value;
        }

      //this hypothesis was added and now the point is not explained anymore, remove previous value (it is a duplicate)
      if ((sign > 0) && (explained_[vec[i]] == 2) && prev_explained)
        add_to_explained -= prev_explained_value;*/

        if(multiple_assignment_penalize_by_one_ == 1)
        {
            if ((explained_[vec[i]] > 1) && prev_dup)
            { //its still a duplicate, do nothing

            }
            else if ((explained_[vec[i]] == 1) && prev_dup)
            { //if was duplicate before, now its not, remove 2, we are removing the hypothesis
                add_to_duplicity_ -= curv_weight;
            }
            else if ((explained_[vec[i]] > 1) && !prev_dup)
            { //it was not a duplicate but it is now, add 2, we are adding a conflicting hypothesis for the point
                add_to_duplicity_ += curv_weight;
            }
        }
        else if(multiple_assignment_penalize_by_one_ == 2)
        {
            if ((explained_[vec[i]] > 1) && prev_dup)
            { //its still a duplicate, add or remove current explained value

                //float add_for_this_p = std::
                add_to_duplicity_ += curv_weight * vec_float[i] * sign;
                duplicates_by_RM_weighted_[vec[i]] += curv_weight * vec_float[i] * sign;
            }
            else if ((explained_[vec[i]] == 1) && prev_dup)
            { //if was duplicate before, now its not, remove current explained weight and old one
                add_to_duplicity_ -= duplicates_by_RM_weighted_[vec[i]];
                duplicates_by_RM_weighted_[vec[i]] = 0;
            }
            else if ((explained_[vec[i]] > 1) && !prev_dup)
            { //it was not a duplicate but it is now, add prev explained value + current explained weight
                add_to_duplicity_ += curv_weight * (prev_explained_value + vec_float[i]);
                duplicates_by_RM_weighted_[vec[i]] = curv_weight * (prev_explained_value + vec_float[i]);
            }
        }
        else
        {
            if ((explained_[vec[i]] > 1) && prev_dup)
            { //its still a duplicate
                //add_to_duplicity_ += vec_float[i] * static_cast<int> (sign); //so, just add or remove one
                //add_to_duplicity_ += vec_float[i] * static_cast<int> (sign) * duplicy_weight_test_ * curv_weight; //so, just add or remove one
                add_to_duplicity_ += static_cast<int> (sign) * duplicy_weight_test_ * curv_weight; //so, just add or remove one
            }
            else if ((explained_[vec[i]] == 1) && prev_dup)
            { //if was duplicate before, now its not, remove 2, we are removing the hypothesis
                //add_to_duplicity_ -= prev_explained_value; // / 2.f; //explained_by_RM_distance_weighted[vec[i]];
                //add_to_duplicity_ -= prev_explained_value * duplicy_weight_test_ * curv_weight;
                add_to_duplicity_ -= duplicy_weight_test_ * curv_weight * 2;
            }
            else if ((explained_[vec[i]] > 1) && !prev_dup)
            { //it was not a duplicate but it is now, add 2, we are adding a conflicting hypothesis for the point
                //add_to_duplicity_ += explained_by_RM_distance_weighted[vec[i]];
                //add_to_duplicity_ += explained_by_RM_distance_weighted[vec[i]] * duplicy_weight_test_ * curv_weight;
                add_to_duplicity_ += duplicy_weight_test_ * curv_weight  * 2;
            }
        }

        add_to_explained += explained_by_RM_distance_weighted[vec[i]] - prev_explained_value;
    }

    //update explained and duplicity values...
    previous_explained_value += add_to_explained;
    previous_duplicity_ += add_to_duplicity_;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::updateCMDuplicity (std::vector<int> & vec, std::vector<int> & occupancy_vec, float sign)
{
    int add_to_duplicity_ = 0;
    for (size_t i = 0; i < vec.size (); i++)
    {

        if(occupancy_vec.size() <= vec[i] || (vec.size() <= i))
        {
            std::cout << occupancy_vec.size() << " " << vec[i] << " " << vec.size() << " " << i << std::endl;
            assert(false);
        }

        bool prev_dup = occupancy_vec[vec[i]] > 1;
        occupancy_vec[vec[i]] += static_cast<int> (sign);
        if ((occupancy_vec[vec[i]] > 1) && prev_dup)
        { //its still a duplicate, we are adding
            add_to_duplicity_ += static_cast<int> (sign); //so, just add or remove one
        }
        else if ((occupancy_vec[vec[i]] == 1) && prev_dup)
        { //if was duplicate before, now its not, remove 2, we are removing the hypothesis
            add_to_duplicity_ -= 2;
        }
        else if ((occupancy_vec[vec[i]] > 1) && !prev_dup)
        { //it was not a duplicate but it is now, add 2, we are adding a conflicting hypothesis for the point
            add_to_duplicity_ += 2;
        }
    }

    previous_duplicity_complete_models_ += add_to_duplicity_;
}

template<typename ModelT, typename SceneT>
double
faat_pcl::GHV<ModelT, SceneT>::getTotalExplainedInformation (std::vector<int> & explained_, std::vector<double> & explained_by_RM_distance_weighted, double * duplicity_)
{
    double explained_info = 0;
    double duplicity = 0;

    for (size_t i = 0; i < explained_.size (); i++)
    {
        if (explained_[i] > 0)
            //if (explained_[i] == 1) //only counts points that are explained once
        {
            //explained_info += std::min(explained_by_RM_distance_weighted[i], 1.0);
            explained_info += explained_by_RM_distance_weighted[i];
        }

        if (explained_[i] > 1)
        {

            //duplicity += explained_by_RM_distance_weighted[i];
            //float curv_weight = std::min(duplicity_curvature_ - scene_curvature_[i], 0.f);

            float curv_weight = getCurvWeight(scene_curvature_[i]);

            if(multiple_assignment_penalize_by_one_ == 1)
            {
                duplicity += curv_weight;
            }
            else if(multiple_assignment_penalize_by_one_ == 2)
            {
                duplicity += duplicates_by_RM_weighted_[i];
                /*if(duplicates_by_RM_weighted_[i] > 1)
                {
                    PCL_WARN("duplicates_by_RM_weighted_[i] higher than one %f\n", duplicates_by_RM_weighted_[i]);
                }*/
            }
            else
            {
                //curv_weight = 1.f;
                //duplicity += explained_by_RM_distance_weighted[i] * duplicy_weight_test_ * curv_weight;
                duplicity += duplicy_weight_test_ * curv_weight * explained_[i];
            }
        }
    }

    *duplicity_ = duplicity;
    //PCL_WARN("fixed error\n");
    //std::cout << explained_info << " " << duplicity << std::endl;
    return explained_info;
}

template<typename ModelT, typename SceneT>
double
faat_pcl::GHV<ModelT, SceneT>::getExplainedByIndices(std::vector<int> & indices, std::vector<float> & explained_values,
                                                     std::vector<double> & explained_by_RM, std::vector<int> & indices_to_update_in_RM_local)
{
    float v=0;
    int indices_to_update_count = 0;
    for(size_t k=0; k < indices.size(); k++)
    {
        if(explained_by_RM_[indices[k]] == 0)
        { //in X1, the point is not explained
            if(explained_by_RM[indices[k]] == 0)
            { //in X2, this is the single hypothesis explaining the point so far
                v += explained_values[k];
                indices_to_update_in_RM_local[indices_to_update_count] = k;
                indices_to_update_count++;
            }
            else
            {
                //in X2, there was a previous hypotheses explaining the point
                //if the previous hypothesis was better, then reject this hypothesis for this point
                if(explained_by_RM[indices[k]] >= explained_values[k])
                {

                }
                else
                {
                    //add the difference
                    v += explained_values[k] - explained_by_RM[indices[k]];
                    indices_to_update_in_RM_local[indices_to_update_count] = k;
                    indices_to_update_count++;
                }
            }
        }
    }

    indices_to_update_in_RM_local.resize(indices_to_update_count);
    return v;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::fill_structures(std::vector<int> & cc_indices, std::vector<bool> & initial_solution, GHVSAModel<ModelT, SceneT> & model)
{
    for (size_t j = 0; j < recognition_models_.size (); j++)
    {
        if(!initial_solution[j])
            continue;

        boost::shared_ptr<GHVRecognitionModel<ModelT> > recog_model = recognition_models_[j];
        for (size_t i = 0; i < recog_model->explained_.size (); i++)
        {
            explained_by_RM_[recog_model->explained_[i]]++;
            //explained_by_RM_distance_weighted[recog_model->explained_[i]] += recog_model->explained_distances_[i];
            explained_by_RM_distance_weighted[recog_model->explained_[i]] = std::max(explained_by_RM_distance_weighted[recog_model->explained_[i]], (double)recog_model->explained_distances_[i]);
        }

        if (detect_clutter_)
        {
            for (size_t i = 0; i < recog_model->unexplained_in_neighborhood.size (); i++)
            {
                unexplained_by_RM_neighboorhods[recog_model->unexplained_in_neighborhood[i]] += recog_model->unexplained_in_neighborhood_weights[i];
            }
        }

        for (size_t i = 0; i < recog_model->complete_cloud_occupancy_indices_.size (); i++)
        {
            int idx = recog_model->complete_cloud_occupancy_indices_[i];
            complete_cloud_occupancy_by_RM_[idx]++;
        }
    }

    //another pass to update duplicates_by_RM_weighted_ (only if multiple_assignment_penalize_by_one_ == 2)
    for (size_t j = 0; j < recognition_models_.size (); j++)
    {
        if(!initial_solution[j])
            continue;

        boost::shared_ptr<GHVRecognitionModel<ModelT> > recog_model = recognition_models_[j];
        for (size_t i = 0; i < recog_model->explained_.size (); i++)
        {
            if(explained_by_RM_[recog_model->explained_[i]] > 1)
            {
                float curv_weight = getCurvWeight(scene_curvature_[recog_model->explained_[i]]);
                duplicates_by_RM_weighted_[recog_model->explained_[i]] += curv_weight * (double)recog_model->explained_distances_[i];
            }
        }
    }

    int occupied_multiple = 0;
    for (size_t i = 0; i < complete_cloud_occupancy_by_RM_.size (); i++)
    {
        if (complete_cloud_occupancy_by_RM_[i] > 1)
        {
            occupied_multiple += complete_cloud_occupancy_by_RM_[i];
        }
    }

    setPreviousDuplicityCM (occupied_multiple);
    //do optimization
    //Define model SAModel, initial solution is all models activated

    double duplicity;
    double good_information_ = getTotalExplainedInformation (explained_by_RM_, explained_by_RM_distance_weighted, &duplicity);
    double bad_information_ = 0;

    double unexplained_in_neighboorhod;
    if(detect_clutter_) {
        unexplained_in_neighboorhod = getUnexplainedInformationInNeighborhood (unexplained_by_RM_neighboorhods, explained_by_RM_);
    } else {
        unexplained_in_neighboorhod = 0;
    }

    for (size_t i = 0; i < initial_solution.size (); i++)
    {
        if (initial_solution[i])
            bad_information_ += recognition_models_[i]->outliers_weight_ * static_cast<double> (recognition_models_[i]->bad_information_);
    }

    setPreviousExplainedValue (good_information_);
    setPreviousDuplicity (duplicity);
    setPreviousBadInfo (bad_information_);
    setPreviousUnexplainedValue (unexplained_in_neighboorhod);

    model.cost_ = static_cast<mets::gol_type> ((good_information_ - bad_information_ - static_cast<double> (duplicity)
                                                - static_cast<double> (occupied_multiple) * w_occupied_multiple_cm_ -
                                                - unexplained_in_neighboorhod - countActiveHypotheses (initial_solution) - countPointsOnDifferentPlaneSides(initial_solution)) * -1.f);

    model.setSolution (initial_solution);
    model.setOptimizer (this);

    std::cout << "*****************************" << std::endl;
    std::cout << "Cost recomputing:" <<
                 static_cast<mets::gol_type> ((good_information_ - bad_information_ - static_cast<double> (duplicity)
                                               - static_cast<double> (occupied_multiple) * w_occupied_multiple_cm_
                                               - unexplained_in_neighboorhod - countActiveHypotheses (initial_solution) - countPointsOnDifferentPlaneSides(initial_solution)) * -1.f) << std::endl;

    //std::cout << countActiveHypotheses (initial_solution) << " points on diff plane sides:" << countPointsOnDifferentPlaneSides(initial_solution, false) << std::endl;
    std::cout << "*****************************" << std::endl;

    /*std::cout << occupied_multiple << " " << w_occupied_multiple_cm_ << std::endl;
    for (size_t i = 0; i < initial_solution.size (); i++)
    {
        std::cout << initial_solution[i] << " ";
    }*/

    /*

    double duplicity = static_cast<double> (getDuplicity ());
    //duplicity = 0.f; //ATTENTION!!
    double good_info = getExplainedValue ();

    double unexplained_info = getPreviousUnexplainedValue ();
    if(!detect_clutter_) {
      unexplained_info = 0;
    }

    double bad_info = static_cast<double> (getPreviousBadInfo ()) + (recognition_models_[changed]->outliers_weight_
        * static_cast<double> (recognition_models_[changed]->bad_information_)) * sign;

    setPreviousBadInfo (bad_info);

    double duplicity_cm = static_cast<double> (getDuplicityCM ()) * w_occupied_multiple_cm_;
    //float duplicity_cm = 0;

    double under_table = 0;
    if (model_constraints_.size () > 0)
    {
      under_table = getModelConstraintsValueForActiveSolution (active);
    }

    boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
    //std::cout << (end_time - start_time).total_microseconds () << " microsecs" << std::endl;
    double cost = (good_info - bad_info - duplicity - unexplained_info - under_table - duplicity_cm - countActiveHypotheses (active)) * -1.f;
  */

    std::cout << std::endl;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::clear_structures()
{
    size_t kk = complete_cloud_occupancy_by_RM_.size();
    explained_by_RM_.clear();
    explained_by_RM_distance_weighted.clear();
    previous_explained_by_RM_distance_weighted.clear();
    unexplained_by_RM_neighboorhods.clear();
    complete_cloud_occupancy_by_RM_.clear();
    explained_by_RM_model.clear();
    duplicates_by_RM_weighted_.clear();

    explained_by_RM_.resize (scene_cloud_downsampled_->points.size (), 0);
    duplicates_by_RM_weighted_.resize (scene_cloud_downsampled_->points.size (), 0);
    explained_by_RM_distance_weighted.resize (scene_cloud_downsampled_->points.size (), 0);
    previous_explained_by_RM_distance_weighted.resize (scene_cloud_downsampled_->points.size ());
    unexplained_by_RM_neighboorhods.resize (scene_cloud_downsampled_->points.size (), 0.f);
    complete_cloud_occupancy_by_RM_.resize(kk, 0);
    explained_by_RM_model.resize (scene_cloud_downsampled_->points.size (), -1);
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::SAOptimize (std::vector<int> & cc_indices, std::vector<bool> & initial_solution)
{

    //temporal copy of recogniton_models_
    std::vector<boost::shared_ptr<GHVRecognitionModel<ModelT> > > recognition_models_copy;
    recognition_models_copy = recognition_models_;

    recognition_models_.clear ();

    for (size_t j = 0; j < cc_indices.size (); j++)
        recognition_models_.push_back (recognition_models_copy[cc_indices[j]]);

    clear_structures();

    GHVSAModel<ModelT, SceneT> model;
    fill_structures(cc_indices, initial_solution, model);

    GHVSAModel<ModelT, SceneT> * best = new GHVSAModel<ModelT, SceneT> (model);

    GHVmove_manager<ModelT, SceneT> neigh (static_cast<int> (cc_indices.size ()), use_replace_moves_);
    boost::shared_ptr<std::map< std::pair<int, int>, bool > > intersect_map;
    intersect_map.reset(new std::map< std::pair<int, int>, bool >);

    if(use_replace_moves_ || (planar_models_.size() > 0))
    {
        pcl::ScopeTime t("compute intersection map...");

        std::vector<int> n_conflicts(recognition_models_.size() * recognition_models_.size(), 0);
        for (size_t k = 0; k < points_explained_by_rm_.size (); k++)
        {
            if (points_explained_by_rm_[k].size() > 1)
            {
                // this point could be a conflict
                for (size_t kk = 0; (kk < points_explained_by_rm_[k].size ()); kk++)
                {
                    for (size_t jj = (kk+1); (jj < points_explained_by_rm_[k].size ()); jj++)
                    {
                        //std::cout << points_explained_by_rm_[k][kk]->id_ << " " << points_explained_by_rm_[k][jj]->id_ << " " << n_conflicts.size() << std::endl;
                        //conflict, THIS MIGHT CAUSE A SEG FAULT AT SOME POINT!! ATTENTION! //todo
                        //i will probably need a vector going from id_ to recognition_models indices
                        assert(points_explained_by_rm_[k][kk]->id_ * recognition_models_.size() + points_explained_by_rm_[k][jj]->id_ < n_conflicts.size());
                        assert(points_explained_by_rm_[k][jj]->id_ * recognition_models_.size() + points_explained_by_rm_[k][kk]->id_ < n_conflicts.size());
                        n_conflicts[points_explained_by_rm_[k][kk]->id_ * recognition_models_.size() + points_explained_by_rm_[k][jj]->id_]++;
                        n_conflicts[points_explained_by_rm_[k][jj]->id_ * recognition_models_.size() + points_explained_by_rm_[k][kk]->id_]++;
                    }
                }
            }
        }

        int num_conflicts = 0;
        for (size_t i = 0; i < recognition_models_.size (); i++)
        {
            //std::cout << "id:" << recognition_models_[i]->id_ << std::endl;
            for (size_t j = (i+1); j < recognition_models_.size (); j++)
            {
                //assert(n_conflicts[i * recognition_models_.size() + j] == n_conflicts[j * recognition_models_.size() + i]);
                //std::cout << n_conflicts[i * recognition_models_.size() + j] << std::endl;
                bool conflict = (n_conflicts[i * recognition_models_.size() + j] > 10);
                std::pair<int, int> p = std::make_pair<int, int> (static_cast<int> (i), static_cast<int> (j));
                (*intersect_map)[p] = conflict;
                if(conflict)
                {
                    num_conflicts++;
                }
            }
        }

        //#define VIS_PLANES

        if(planar_models_.size() > 0 && use_points_on_plane_side_)
        {
            //compute for each planar model, how many points for the other hypotheses (if in conflict) are on each side of the plane
            //std::cout << "recognition_models size:" << recognition_models_.size() << std::endl;

#ifdef VIS_PLANES
            pcl::visualization::PCLVisualizer vis("TEST");
#endif
            points_one_plane_sides_.clear();
            points_one_plane_sides_.resize(planar_models_.size());

            for(size_t i=0; i < recognition_models_.size(); i++)
            {
                std::map<int, int>::iterator it1, it2;
                it1 = model_to_planar_model_.find(static_cast<int>(i));
                if(it1 != model_to_planar_model_.end())
                {

                    points_one_plane_sides_[it1->second].resize(recognition_models_.size() - planar_models_.size() + 1, 0.f);

                    //is a plane, check how many points from other hypotheses are at each side of the plane
                    for(size_t j=0; j < recognition_models_.size(); j++)
                    {

                        if(i == j)
                            continue;

                        it2 = model_to_planar_model_.find(static_cast<int>(j));
                        if(it2 != model_to_planar_model_.end())
                        {
                            //both are planes, ignore
                            //std::cout << recognition_models_[i]->id_s_ << " " << recognition_models_[j]->id_s_ << std::endl;
                            continue;
                        }

                        assert(recognition_models_[j]->id_ < (recognition_models_.size() - planar_models_.size() + 1));

                        bool conflict = (n_conflicts[recognition_models_[i]->id_ * recognition_models_.size() + recognition_models_[j]->id_] > 0);
                        if(!conflict)
                            continue;

                        //is not a plane and is in conflict, compute points on both sides
                        Eigen::Vector2f side_count = Eigen::Vector2f::Zero();
                        for(size_t k=0; k < complete_models_[j]->points.size(); k++)
                        {
                            std::vector<float> p = planar_models_[it1->second].coefficients_.values;
                            Eigen::Vector3f xyz_p = complete_models_[j]->points[k].getVector3fMap();
                            float val = xyz_p[0] * p[0] + xyz_p[1] * p[1] + xyz_p[2] * p[2] + p[3];

                            if(std::abs(val) <= inliers_threshold_)
                            {
                                /*if(val < 0)
                                    side_count[0]+= 0.1f;
                                else
                                    side_count[1]+= 0.1f;*/

                                continue;
                            }

                            if(val < 0)
                                side_count[0]+= 1.f;
                            else
                                side_count[1]+= 1.f;
                        }

                        float min_side = std::min(side_count[0],side_count[1]);
                        float max_side = std::max(side_count[0],side_count[1]);
                        //float ratio = static_cast<float>(min_side) / static_cast<float>(max_side); //between 0 and 1
                        if(max_side != 0)
                        {
                            assert(recognition_models_[j]->id_ < points_one_plane_sides_[it1->second].size());
                            points_one_plane_sides_[it1->second][recognition_models_[j]->id_] = min_side;
                            //std::cout << recognition_models_[i]->id_s_ << " " << recognition_models_[j]->id_s_ << std::endl;
                            //std::cout << min_side << " " << max_side << std::endl;

#ifdef VIS_PLANES
                            vis.addPointCloud<SceneT>(scene_cloud_downsampled_, "scene");
                            vis.addPointCloud<ModelT>(recognition_models_[j]->complete_cloud_, "complete_cloud");
                            vis.addPolygonMesh(*(planar_models_[it1->second].convex_hull_), "polygon");

                            vis.spin();
                            vis.removeAllPointClouds();
                            vis.removeAllShapes();
#endif
                        }
                        else
                            points_one_plane_sides_[it1->second][recognition_models_[j]->id_] = 0;
                    }
                }
            }

            /*std::cout << "recognition_models size:" << recognition_models_.size() << std::endl;
            std::cout << "complete models size:" << complete_models_.size() << std::endl;

            for(size_t kk=0; kk < points_one_plane_sides_.size(); kk++)
            {
                for(size_t kkk=0; kkk < points_one_plane_sides_[kk].size(); kkk++)
                {
                    std::cout << points_one_plane_sides_[kk][kkk] << " ";
                }

                std::cout << std::endl;
            }*/
        }

        std::cout << "num_conflicts:" << num_conflicts << " " << recognition_models_.size() * recognition_models_.size() << std::endl;
    }

    neigh.setExplainedPointIntersections(intersect_map);

    //mets::best_ever_solution best_recorder (best);
    cost_logger_.reset(new GHVCostFunctionLogger<ModelT, SceneT>(*best));
    mets::noimprove_termination_criteria noimprove (max_iterations_);

    if(visualize_go_cues_)
    {
        cost_logger_->setVisualizeFunction(visualize_cues_during_logger_);
    }


    switch(opt_type_)
    {
    case 0:
    {
        mets::local_search<GHVmove_manager<ModelT, SceneT> > local ( model, *(cost_logger_.get()), neigh, 0, LS_short_circuit_);
        {
            pcl::ScopeTime t ("local search...");
            local.search ();
        }
        break;
    }
    case 1:
    {
        //Tabu search
        //mets::simple_tabu_list tabu_list ( initial_solution.size() * sqrt ( 1.0*initial_solution.size() ) ) ;
        mets::simple_tabu_list tabu_list ( 5 * initial_solution.size()) ;
        mets::best_ever_criteria aspiration_criteria ;

        std::cout << "max iterations:" << max_iterations_ << std::endl;
        mets::tabu_search<GHVmove_manager<ModelT, SceneT> > tabu_search(model,  *(cost_logger_.get()), neigh, tabu_list, aspiration_criteria, noimprove);
        //mets::tabu_search<move_manager> tabu_search(model, best_recorder, neigh, tabu_list, aspiration_criteria, noimprove);

        {
            pcl::ScopeTime t ("TABU search...");
            try {
                tabu_search.search ();
            } catch (mets::no_moves_error e) {
                //} catch (std::exception e) {

            }
        }
        break;
    }
    case 4:
    {

        GHVmove_manager<ModelT, SceneT> neigh (static_cast<int> (cc_indices.size ()), false);
        neigh.setExplainedPointIntersections(intersect_map);

        mets::simple_tabu_list tabu_list ( initial_solution.size() * sqrt ( 1.0*initial_solution.size() ) ) ;
        mets::best_ever_criteria aspiration_criteria ;
        mets::tabu_search<GHVmove_manager<ModelT, SceneT> > tabu_search(model,  *(cost_logger_.get()), neigh, tabu_list, aspiration_criteria, noimprove);
        //mets::tabu_search<move_manager> tabu_search(model, best_recorder, neigh, tabu_list, aspiration_criteria, noimprove);

        {
            pcl::ScopeTime t ("TABU search + LS (RM)...");
            try {
                tabu_search.search ();
            } catch (mets::no_moves_error e) {

            }

            std::cout << "Tabu search finished... starting LS with RM" << std::endl;

            //after TS, we do LS with RM
            GHVmove_manager<ModelT, SceneT> neigh (static_cast<int> (cc_indices.size ()), true);
            neigh.setExplainedPointIntersections(intersect_map);

            mets::local_search<GHVmove_manager<ModelT, SceneT> > local ( model, *(cost_logger_.get()), neigh, 0, false);
            {
                pcl::ScopeTime t ("local search...");
                local.search ();
            }
        }
        break;

    }
    default:
    {
        //Simulated Annealing
        //mets::linear_cooling linear_cooling;
        mets::exponential_cooling linear_cooling;
        mets::simulated_annealing<GHVmove_manager<ModelT, SceneT> > sa (model,  *(cost_logger_.get()), neigh, noimprove, linear_cooling, initial_temp_, 1e-7, 1);
        sa.setApplyAndEvaluate (true);

        {
            pcl::ScopeTime t ("SA search...");
            sa.search ();
        }
        break;
    }
    }

    best_seen_ = static_cast<const GHVSAModel<ModelT, SceneT>&> (cost_logger_->best_seen ());
    std::cout << "*****************************" << std::endl;
    std::cout << "Final cost:" << best_seen_.cost_;
    std::cout << " Number of ef evaluations:" << cost_logger_->getTimesEvaluated();
    std::cout << std::endl;
    std::cout << "Number of accepted moves:" << cost_logger_->getAcceptedMovesSize() << std::endl;
    std::cout << "*****************************" << std::endl;

    for (size_t i = 0; i < best_seen_.solution_.size (); i++) {
        initial_solution[i] = best_seen_.solution_[i];
    }

    //pcl::visualization::PCLVisualizer vis_ ("test histograms");

    for(size_t i = 0; i < initial_solution.size(); i++) {
        if(initial_solution[i]) {
            std::cout << "id: " << recognition_models_[i]->id_s_ << std::endl;
            /*std::cout << "Median:" << recognition_models_[i]->median_ << std::endl;
            std::cout << "Mean:" << recognition_models_[i]->mean_ << std::endl;
            std::cout << "Color similarity:" << recognition_models_[i]->color_similarity_ << std::endl;*/
            std::cout << "#outliers:" << recognition_models_[i]->bad_information_ << " " << recognition_models_[i]->outliers_weight_ << std::endl;
            //std::cout << "#under table:" << recognition_models_[i]->model_constraints_value_ << std::endl;
            std::cout << "#explained:" << recognition_models_[i]->explained_.size() << std::endl;
            std::cout << "normal entropy:" << recognition_models_[i]->normal_entropy_ << std::endl;
            std::cout << "color entropy:" << recognition_models_[i]->color_entropy_ << std::endl;
            std::cout << "hyp penalty:" << recognition_models_[i]->hyp_penalty_ << std::endl;
            std::cout << "color diff:" << recognition_models_[i]->color_diff_trhough_specification_ << std::endl;
            std::cout << "Mean:" << recognition_models_[i]->mean_ << std::endl;

            typename boost::shared_ptr<GHVRecognitionModel<ModelT> > recog_model = recognition_models_[i];

            //visualize
            if(!ignore_color_even_if_exists_ && visualize_accepted_)
            {

                std::map<int, int>::iterator it1;
                it1 = model_to_planar_model_.find(static_cast<int>(i));
                if(it1 != model_to_planar_model_.end())
                {
                    continue;
                }

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud_gs(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*recog_model->cloud_, *model_cloud_gs);

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud_gs_specified(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

                if(color_space_ == 1 || color_space_ == 5)
                {
                    pcl::copyPointCloud(*recog_model->cloud_, *model_cloud_gs_specified);
                    for(size_t k=0; k < model_cloud_gs_specified->points.size(); k++)
                    {
                        model_cloud_gs_specified->points[k].r = recog_model->cloud_RGB_[k][0] * 255;
                        model_cloud_gs_specified->points[k].g = recog_model->cloud_RGB_[k][1] * 255;
                        model_cloud_gs_specified->points[k].b = recog_model->cloud_RGB_[k][2] * 255;
                    }

                    pcl::copyPointCloud(*scene_cloud_downsampled_, *scene_cloud);
                    //pcl::copyPointCloud(*scene_cloud, recognition_models_[i]->explained_, *scene_cloud);

                }
                else if(color_space_ == 0 || color_space_ == 6)
                {
                    pcl::copyPointCloud(*recog_model->cloud_, *model_cloud_gs_specified);
                    for(size_t k=0; k < model_cloud_gs_specified->points.size(); k++)
                    {
                        model_cloud_gs_specified->points[k].r = recog_model->cloud_LAB_[k][0] * 255;
                        model_cloud_gs_specified->points[k].g = (recog_model->cloud_LAB_[k][1] + 1.f) / 2.f * 255;
                        model_cloud_gs_specified->points[k].b = (recog_model->cloud_LAB_[k][2] + 1.f) / 2.f * 255;
                    }

                    for(size_t k=0; k < model_cloud_gs->points.size(); k++)
                    {
                        //float gs = (scene_cloud->points[k].r + scene_cloud->points[k].g + scene_cloud->points[k].b) / 3.f;
                        uint32_t rgb = *reinterpret_cast<int*> (&model_cloud_gs->points[k].rgb);
                        uint8_t rs = (rgb >> 16) & 0x0000ff;
                        uint8_t gs = (rgb >> 8) & 0x0000ff;
                        uint8_t bs = (rgb) & 0x0000ff;

                        float LRefs, aRefs, bRefs;
                        RGB2CIELAB (rs, gs, bs, LRefs, aRefs, bRefs);
                        LRefs /= 100.0f; aRefs /= 120.0f; bRefs /= 120.0f;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

                        model_cloud_gs->points[k].r = LRefs * 255.f;
                        model_cloud_gs->points[k].g = (aRefs + 1.f) / 2.f * 255;
                        model_cloud_gs->points[k].b = (bRefs + 1.f) / 2.f * 255;
                    }

                    pcl::copyPointCloud(*scene_cloud_downsampled_, *scene_cloud);

                    for(size_t k=0; k < scene_cloud->points.size(); k++)
                    {
                        scene_cloud->points[k].r = scene_LAB_values_[k][0] * 255;
                        scene_cloud->points[k].g = (scene_LAB_values_[k][1] + 1.f) / 2.f * 255;
                        scene_cloud->points[k].b = (scene_LAB_values_[k][2] + 1.f) / 2.f * 255;
                    }

                    //pcl::copyPointCloud(*scene_cloud, recognition_models_[i]->explained_, *scene_cloud);
                }
                else if(color_space_ == 2)
                {
                    pcl::copyPointCloud(*recog_model->cloud_, *model_cloud_gs_specified);
                    for(size_t k=0; k < model_cloud_gs_specified->points.size(); k++)
                    {
                        unsigned char c = (recog_model->cloud_GS_[k]) * 255;
                        model_cloud_gs_specified->points[k].r =
                                model_cloud_gs_specified->points[k].g =
                                model_cloud_gs_specified->points[k].b = c;
                    }

                    for(size_t k=0; k < model_cloud_gs->points.size(); k++)
                    {
                        //float gs = (scene_cloud->points[k].r + scene_cloud->points[k].g + scene_cloud->points[k].b) / 3.f;
                        uint32_t rgb = *reinterpret_cast<int*> (&model_cloud_gs->points[k].rgb);
                        uint8_t rs = (rgb >> 16) & 0x0000ff;
                        uint8_t gs = (rgb >> 8) & 0x0000ff;
                        uint8_t bs = (rgb) & 0x0000ff;

                        unsigned int c = (rs + gs + bs) / 3;

                        model_cloud_gs->points[k].r =
                                model_cloud_gs->points[k].g =
                                model_cloud_gs->points[k].b = c;
                    }

                    pcl::copyPointCloud(*scene_cloud_downsampled_, *scene_cloud);

                    for(size_t k=0; k < scene_cloud->points.size(); k++)
                    {
                        unsigned char c = scene_GS_values_[k] * 255;
                        scene_cloud->points[k].r =
                                scene_cloud->points[k].g =
                                scene_cloud->points[k].b = c;
                    }

                    //pcl::copyPointCloud(*scene_cloud, recognition_models_[i]->explained_, *scene_cloud);
                }
                else if(color_space_ == 3)
                {
                    pcl::copyPointCloud(*recog_model->cloud_, *model_cloud_gs_specified);
                    for(size_t k=0; k < model_cloud_gs_specified->points.size(); k++)
                    {
                        model_cloud_gs_specified->points[k].r =
                                model_cloud_gs_specified->points[k].g =
                                model_cloud_gs_specified->points[k].b = recog_model->cloud_LAB_[k][0] * 255;
                    }

                    for(size_t k=0; k < model_cloud_gs->points.size(); k++)
                    {
                        //float gs = (scene_cloud->points[k].r + scene_cloud->points[k].g + scene_cloud->points[k].b) / 3.f;
                        uint32_t rgb = *reinterpret_cast<int*> (&model_cloud_gs->points[k].rgb);
                        uint8_t rs = (rgb >> 16) & 0x0000ff;
                        uint8_t gs = (rgb >> 8) & 0x0000ff;
                        uint8_t bs = (rgb) & 0x0000ff;

                        float LRefs, aRefs, bRefs;
                        RGB2CIELAB (rs, gs, bs, LRefs, aRefs, bRefs);
                        LRefs /= 100.0f; aRefs /= 120.0f; bRefs /= 120.0f;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

                        model_cloud_gs->points[k].r =
                                model_cloud_gs->points[k].g =
                                model_cloud_gs->points[k].b = LRefs * 255.f;
                    }

                    pcl::copyPointCloud(*scene_cloud_downsampled_, *scene_cloud);

                    for(size_t k=0; k < scene_cloud->points.size(); k++)
                    {
                        scene_cloud->points[k].r =
                                scene_cloud->points[k].g =
                                scene_cloud->points[k].b = scene_LAB_values_[k][0] * 255;
                    }
                }


                pcl::visualization::PCLVisualizer vis("TEST");
                int v1,v2, v3, v4;
                vis.createViewPort(0,0,0.25,1,v1);
                vis.createViewPort(0.25,0,0.5,1,v2);
                vis.createViewPort(0.5,0,0.75,1,v3);
                vis.createViewPort(0.75,0,1,1,v4);
                vis.addPointCloud<pcl::PointXYZRGB>(scene_cloud, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(scene_cloud), "scene", v1);
                vis.addPointCloud<pcl::PointXYZRGB>(model_cloud_gs, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(model_cloud_gs), "model", v2);
                vis.addPointCloud<pcl::PointXYZRGB>(model_cloud_gs_specified, pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(model_cloud_gs_specified), "model_specified", v3);


                if(models_smooth_faces_.size() > i)
                {
                    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxels_labels_cloud = recognition_models_[i]->visible_labels_;
                    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZL> handler_labels(supervoxels_labels_cloud, "label");
                    vis.addPointCloud(supervoxels_labels_cloud, handler_labels, "labels_", v4);
                }

                vis.setBackgroundColor(1,1,1);
                vis.spin();
            }
        }
    }

    delete best;

    {
        //check results
        GHVSAModel<ModelT, SceneT> model;
        clear_structures();
        fill_structures(cc_indices, initial_solution, model);
    }

    recognition_models_ = recognition_models_copy;

}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::verify ()
{
    {
        pcl::StopWatch t;
        t.reset();
        initialize ();
        t_cues_ = static_cast<float>(t.getTimeSeconds());
    }

    if(visualize_go_cues_)
    {
//        visualize_cues_during_logger_ = boost::bind(&(faat_pcl::GHV<ModelT, SceneT>::visualizeGOCues), this, _1, _2, _3);
        vis_go_cues_.reset(new pcl::visualization::PCLVisualizer("visualizeGOCues"));
    }

    n_cc_ = 1;
    cc_.resize(1);
    cc_[0].resize(recognition_models_.size());
    for(size_t i=0; i < recognition_models_.size(); i++)
    {
        cc_[0][i] = static_cast<int>(i);
    }

    //compute number of visible points
    number_of_visible_points_ = 0;
    for(size_t i=0; i < recognition_models_.size(); i++)
    {
        number_of_visible_points_ += static_cast<int>(recognition_models_[i]->cloud_->points.size());
    }

    //for each connected component, find the optimal solution
    {
        pcl::StopWatch t;
        t.reset();

        for (int c = 0; c < n_cc_; c++)
        {
            //TODO: Check for trivial case...
            //TODO: Check also the number of hypotheses and use exhaustive enumeration if smaller than 10
            std::vector<bool> subsolution (cc_[c].size (), initial_status_);

            SAOptimize (cc_[c], subsolution);


            //ATTENTION: just for the paper to visualize cues!!
            /*if(visualize_go_cues_)
            {

                std::vector<bool> opt_subsolution = subsolution;
                //deactivate two hypotheses and randomly activate two others
                for(size_t k=0; k < 200; k++)
                {

                    srand((unsigned)time(NULL));
                    boost::mt19937 generator(time(0));
                    boost::uniform_01<boost::mt19937> gen(generator);

                    int deactivated = 0;
                    int max_deact = 2;
                    for(size_t i=0; i < subsolution.size(); i++)
                    {
                        if(deactivated >= max_deact)
                            break;

                        if(subsolution[i])
                        {
                            float r = gen();
                            std::cout << "randon number:" << r << std::endl;
                            if(r > 0.9f)
                            {
                                subsolution[i] = false;
                                deactivated++;
                            }
                        }
                    }

                    int act = 0;
                    int max_act = 2;

                    for (int i = 0; (i < max_act); i++)
                    {
                        int to_act = std::floor(gen() * subsolution.size());
                        subsolution[to_act] = true;
                    }

                    //check results
                    GHVSAModel<ModelT, SceneT> model;
                    clear_structures();
                    fill_structures(cc_[c], subsolution, model);

                    visualizeGOCues(subsolution, 0, 0);

                    subsolution = opt_subsolution;
                }
            }*/

            for (size_t i = 0; i < subsolution.size (); i++)
            {
                //mask_[indices_[cc_[c][i]]] = (subsolution[i]);
                mask_[cc_[c][i]] = (subsolution[i]);
            }
        }

        t_opt_ = static_cast<float>(t.getTimeSeconds());
    }
}

inline void softBining(float val, int pos1, float bin_size, int max_pos, int & pos2, float & w1, float & w2) {
    float c1 = pos1 * bin_size + bin_size / 2;
    pos2 = 0;
    float c2 = 0;
    if(pos1 == 0) {
        pos2 = 1;
        c2 = pos2 * bin_size + bin_size / 2;
    } else if(pos1 == (max_pos-1)) {
        pos2 = max_pos-2;
        c2 = pos2 * bin_size + bin_size / 2;
    } else {
        if(val > c1) {
            pos2 = pos1 + 1;
        } else {
            pos2 = pos1 - 1;
        }

        c2 = pos2 * bin_size + bin_size / 2;
    }

    w1 = (val - c1) / (c2 - c1);
    w2 = (c2 - val) / (c2 - c1);
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::computeGSHistogram
(std::vector<float> & gs_values, Eigen::MatrixXf & histogram, int hist_size)
{
    float max = 255.f;
    float min = 0.f;
    int dim = 1;

    histogram = Eigen::MatrixXf (hist_size, dim);
    histogram.setZero ();
    for (size_t j = 0; j < gs_values.size (); j++)
    {
        int pos = std::floor (static_cast<float> (gs_values[j] - min) / (max - min) * hist_size);
        if(pos < 0)
            pos = 0;

        if(pos > hist_size)
            pos = hist_size - 1;

        histogram (pos, 0)++;
    }
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::computeRGBHistograms (std::vector<Eigen::Vector3f> & rgb_values, Eigen::MatrixXf & rgb, int dim, float min, float max, bool soft)
{
    int hist_size = max - min + 1;
    //float size_bin = 1.f / hist_size;
    rgb = Eigen::MatrixXf (hist_size, dim);
    rgb.setZero ();
    for (size_t i = 0; i < dim; i++)
    {
        for (size_t j = 0; j < rgb_values.size (); j++)
        {
            int pos = std::floor (static_cast<float> (rgb_values[j][i] - min) / (max - min) * hist_size);
            if(pos < 0)
                pos = 0;

            if(pos > hist_size)
                pos = hist_size - 1;

            /*if (soft)
      {
        float c1 = pos * size_bin + size_bin / 2;
        int pos2 = 0;
        float c2 = 0;
        if (pos == 0)
        {
          pos2 = 1;
          c2 = pos2 * size_bin + size_bin / 2;
        }
        else if (pos == 24)
        {
          pos2 = 23;
          c2 = pos2 * size_bin + size_bin / 2;
        }
        else
        {
          if ((static_cast<float> (rgb_values[j][i]) / 255.f) > c1)
          {
            pos2 = pos + 1;
          }
          else
          {
            pos2 = pos - 1;
          }

          c2 = pos2 * size_bin + size_bin / 2;
        }

        rgb (pos, i) += ((static_cast<float> (rgb_values[j][i]) / 255.f) - c1) / (c2 - c1);
        rgb (pos2, i) += (c2 - (static_cast<float> (rgb_values[j][i]) / 255.f)) / (c2 - c1);
      }

      else
      {*/
            rgb (pos, i)++;
            //}

        }
    }
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::specifyRGBHistograms (Eigen::MatrixXf & src, Eigen::MatrixXf & dst, Eigen::MatrixXf & lookup, int dim)
{
    //normalize histograms
    for(size_t i=0; i < dim; i++) {
        src.col(i) /= src.col(i).sum();
        dst.col(i) /= dst.col(i).sum();
    }

    Eigen::MatrixXf src_cumulative(src.rows(), dim);
    Eigen::MatrixXf dst_cumulative(dst.rows(), dim);
    lookup = Eigen::MatrixXf(src.rows(), dim);
    lookup.setZero();

    src_cumulative.setZero();
    dst_cumulative.setZero();

    for (size_t i = 0; i < dim; i++)
    {
        src_cumulative (0, i) = src (0, i);
        dst_cumulative (0, i) = dst (0, i);
        for (size_t j = 1; j < src_cumulative.rows (); j++)
        {
            src_cumulative (j, i) = src_cumulative (j - 1, i) + src (j, i);
            dst_cumulative (j, i) = dst_cumulative (j - 1, i) + dst (j, i);
        }

        int last = 0;
        for (int k = 0; k < src_cumulative.rows (); k++)
        {
            for (int z = last; z < src_cumulative.rows (); z++)
            {
                if (src_cumulative (z, i) - dst_cumulative (k, i) >= 0)
                {
                    if (z > 0 && (dst_cumulative (k, i) - src_cumulative (z - 1, i)) < (src_cumulative (z, i) - dst_cumulative (k, i)))
                        z--;

                    lookup (k, i) = z;
                    last = z;
                    break;
                }
            }
        }

        int min = 0;
        for (int k = 0; k < src_cumulative.rows (); k++)
        {
            if (lookup (k, i) != 0)
            {
                min = lookup (k, i);
                break;
            }
        }

        for (int k = 0; k < src_cumulative.rows (); k++)
        {
            if (lookup (k, i) == 0)
                lookup (k, i) = min;
            else
                break;
        }

        //max mapping extension
        int max = 0;
        for (int k = (src_cumulative.rows () - 1); k >= 0; k--)
        {
            if (lookup (k, i) != 0)
            {
                max = lookup (k, i);
                break;
            }
        }

        for (int k = (src_cumulative.rows () - 1); k >= 0; k--)
        {
            if (lookup (k, i) == 0)
                lookup (k, i) = max;
            else
                break;
        }
    }
}

template<typename ModelT, typename SceneT>
bool
faat_pcl::GHV<ModelT, SceneT>::handlingNormals (boost::shared_ptr<GHVRecognitionModel<ModelT> > & recog_model, int i, bool is_planar_model, int object_models_size)
{
    //std::cout << visible_normal_models_.size() << " " << object_models_size << " " << complete_models_.size() << std::endl;
    if(visible_normal_models_.size() == static_cast<size_t>(object_models_size) && !use_normals_from_visible_/*&& !is_planar_model*/)
    {
        pcl::PointCloud<pcl::Normal>::ConstPtr model_normals = visible_normal_models_[i];
        //pcl::ScopeTime t("Using model normals and checking nans");

        recog_model->normals_.reset (new pcl::PointCloud<pcl::Normal> ());
        recog_model->normals_->points.resize(recog_model->cloud_->points.size ());

        //check nans...
        int j = 0;
        for (size_t i = 0; i < recog_model->cloud_->points.size (); ++i)
        {
            if (!pcl_isfinite (recog_model->cloud_->points[i].x) || !pcl_isfinite (recog_model->cloud_->points[i].y)
                    || !pcl_isfinite (recog_model->cloud_->points[i].z))
                continue;

            if (!pcl_isfinite (model_normals->points[i].normal_x) || !pcl_isfinite (model_normals->points[i].normal_y)
                    || !pcl_isfinite (model_normals->points[i].normal_z))
                continue
                ;
            recog_model->cloud_->points[j] = recog_model->cloud_->points[i];
            recog_model->normals_->points[j] = model_normals->points[i];
            j++;
        }

        recog_model->cloud_->points.resize (j);
        recog_model->cloud_->width = j;
        recog_model->cloud_->height = 1;

        recog_model->normals_->points.resize (j);
        recog_model->normals_->width = j;
        recog_model->normals_->height = 1;

        if (recog_model->cloud_->points.size () <= 0)
        {
            PCL_WARN("The model cloud has no points..\n");
            return false;
        }
    }
    else
    {

        //pcl::ScopeTime t("Computing normals and checking nans");

        int j = 0;
        for (size_t i = 0; i < recog_model->cloud_->points.size (); ++i)
        {
            if (!pcl_isfinite (recog_model->cloud_->points[i].x) || !pcl_isfinite (recog_model->cloud_->points[i].y)
                    || !pcl_isfinite (recog_model->cloud_->points[i].z))
                continue;

            recog_model->cloud_->points[j] = recog_model->cloud_->points[i];
            j++;
            //std::cout << "there are nans..." << std::endl;
        }

        recog_model->cloud_->points.resize (j);
        recog_model->cloud_->width = j;
        recog_model->cloud_->height = 1;

        if (recog_model->cloud_->points.size () <= 0)
        {
            PCL_WARN("The model cloud has no points..\n");
            return false;
        }

        {
            //compute normals unless given (now do it always...)

            std::cout << "compute normals, is planar model:" << is_planar_model << " " << radius_normals_ << std::endl;
            typename pcl::search::KdTree<ModelT>::Ptr normals_tree (new pcl::search::KdTree<ModelT>);
            typedef typename pcl::NormalEstimationOMP<ModelT, pcl::Normal> NormalEstimator_;
            NormalEstimator_ n3d;
            recog_model->normals_.reset (new pcl::PointCloud<pcl::Normal> ());
            normals_tree->setInputCloud (recog_model->cloud_);
            n3d.setRadiusSearch (radius_normals_);
            n3d.setSearchMethod (normals_tree);
            n3d.setInputCloud ((recog_model->cloud_));
            n3d.compute (*(recog_model->normals_));

            //check nans...
            int j = 0;
            for (size_t i = 0; i < recog_model->normals_->points.size (); ++i)
            {
                if (!pcl_isfinite (recog_model->normals_->points[i].normal_x) || !pcl_isfinite (recog_model->normals_->points[i].normal_y)
                        || !pcl_isfinite (recog_model->normals_->points[i].normal_z))
                    continue;

                recog_model->normals_->points[j] = recog_model->normals_->points[i];
                recog_model->cloud_->points[j] = recog_model->cloud_->points[i];
                j++;
            }

            recog_model->normals_->points.resize (j);
            recog_model->normals_->width = j;
            recog_model->normals_->height = 1;

            recog_model->cloud_->points.resize (j);
            recog_model->cloud_->width = j;
            recog_model->cloud_->height = 1;
        }
    }

    return true;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::specifyColor(int i, Eigen::MatrixXf & lookup, boost::shared_ptr<GHVRecognitionModel<ModelT> > & recog_model)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud_specified(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*recog_model->cloud_, *model_cloud_specified);

    bool smooth_faces_exist = false;
    if(models_smooth_faces_.size() > i)
    {
        smooth_faces_exist = true;
    }

    std::vector< std::vector<int> > label_indices;
    std::vector< std::set<int> > label_explained_indices_points;

    pcl::PointCloud<pcl::PointXYZL>::Ptr visible_labels(new pcl::PointCloud<pcl::PointXYZL>);

    if(smooth_faces_exist)
    {
        //use visible indices to check which points are visible
        pcl::copyPointCloud(*models_smooth_faces_[i], visible_indices_[i], *visible_labels);
        recog_model->visible_labels_ = visible_labels;

        //specify using the smooth faces

        int max_label = 0;
        for(size_t k=0; k < visible_labels->points.size(); k++)
        {
            if(visible_labels->points[k].label > max_label)
            {
                max_label = visible_labels->points[k].label;
            }
        }

        //std::cout << "max label:" << max_label << std::endl;

        //1) group points based on label
        label_indices.resize(max_label + 1);
        for(size_t k=0; k < visible_labels->points.size(); k++)
        {
            label_indices[visible_labels->points[k].label].push_back(k);
        }

        //2) for each group, find corresponding scene points and push them into label_explained_indices_points
        std::vector<std::pair<int, float> > label_index_distances;
        label_index_distances.resize(scene_cloud_downsampled_->points.size(), std::make_pair(-1, std::numeric_limits<float>::infinity()));

        //std::vector<int> nn_indices;
        //std::vector<float> nn_distances;

        for(size_t j=0; j < label_indices.size(); j++)
        {
            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                /*if (octree_scene_downsampled_->radiusSearch (recog_model->cloud_->points[label_indices[j][i]], inliers_threshold_,
                                                             nn_indices, nn_distances, std::numeric_limits<int>::max ()) > 0)*/

                std::vector<int> & nn_indices = recog_model->inlier_indices_[label_indices[j][i]];
                std::vector<float> & nn_distances = recog_model->inlier_distances_[label_indices[j][i]];

                if(nn_indices.size() > 0)
                {
                    for (size_t k = 0; k < nn_distances.size (); k++)
                    {
                        if(label_index_distances[nn_indices[k]].first == static_cast<int>(j))
                        {
                            //already explained by the same label
                        }
                        else
                        {
                            //if different labels, then take the new label if distances is smaller
                            if(nn_distances[k] < label_index_distances[nn_indices[k]].second)
                            {
                                label_index_distances[nn_indices[k]].first = static_cast<int>(j);
                                label_index_distances[nn_indices[k]].second = nn_distances[k];
                            } //otherwise, ignore new label since the older one is closer
                        }
                    }
                }
            }
        }

        //3) set label_explained_indices_points
        label_explained_indices_points.resize(max_label + 1);
        for (size_t i = 0; i < scene_cloud_downsampled_->points.size (); i++)
        {
            if(label_index_distances[i].first < 0)
                continue;

            label_explained_indices_points[label_index_distances[i].first].insert(i);
        }
    }
    else
    {
        //specify for the whole model
        label_indices.resize(1);
        label_explained_indices_points.resize(1);
        for(size_t k=0; k < recog_model->cloud_->points.size(); k++)
        {
            label_indices[0].push_back(k);
        }

        std::vector<std::pair<int, float> > label_index_distances;
        label_index_distances.resize(scene_cloud_downsampled_->points.size(), std::make_pair(-1, std::numeric_limits<float>::infinity()));

        //std::vector<int> nn_indices;
        //std::vector<float> nn_distances;

        for (size_t i = 0; i < label_indices[0].size (); i++)
        {
            /*if (octree_scene_downsampled_->radiusSearch (recog_model->cloud_->points[label_indices[0][i]], inliers_threshold_,
                                                         nn_indices, nn_distances, std::numeric_limits<int>::max ()) > 0)*/

            std::vector<int> & nn_indices = recog_model->inlier_indices_[label_indices[0][i]];
            std::vector<float> & nn_distances = recog_model->inlier_distances_[label_indices[0][i]];

            if(nn_indices.size() > 0)
            {
                for (size_t k = 0; k < nn_distances.size (); k++)
                {
                    if(label_index_distances[nn_indices[k]].first == static_cast<int>(0))
                    {
                        //already explained by the same label
                    }
                    else
                    {
                        //if different labels, then take the new label if distances is smaller
                        if(nn_distances[k] < label_index_distances[nn_indices[k]].second)
                        {
                            label_index_distances[nn_indices[k]].first = static_cast<int>(0);
                            label_index_distances[nn_indices[k]].second = nn_distances[k];
                        } //otherwise, ignore new label since the older one is closer
                    }
                }
            }
        }

        //scene indices are the explained_points of the model
        /*for(size_t k=0; k < recog_model->explained_.size(); k++)
        {
            label_explained_indices_points[0].insert(recog_model->explained_[k]);
        }*/

        for (size_t i = 0; i < scene_cloud_downsampled_->points.size (); i++)
        {
            if(label_index_distances[i].first < 0)
                continue;

            label_explained_indices_points[label_index_distances[i].first].insert(i);
        }

        /*std::cout << label_indices.size() << " " << label_explained_indices_points.size() << std::endl;
        std::cout << label_indices[0].size() << " " << label_explained_indices_points[0].size() << std::endl;*/
    }

    recog_model->cloud_LAB_original_ = recog_model->cloud_LAB_;

    //specify each label
    for(size_t j=0; j < label_indices.size(); j++)
    {
        std::set<int> explained_indices_points = label_explained_indices_points[j];

        if(color_space_ == 0 || color_space_ == 3)
        {
            std::vector<float> model_gs_values, scene_gs_values;

            //compute RGB histogram for the model points
            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                model_gs_values.push_back(recog_model->cloud_LAB_[label_indices[j][i]][0] * 255.f);
            }

            //compute RGB histogram for the explained points
            std::set<int>::iterator it;
            for(it=explained_indices_points.begin(); it != explained_indices_points.end(); it++)
            {
                scene_gs_values.push_back(scene_LAB_values_[*it][0] * 255.f);
            }

            Eigen::MatrixXf gs_model, gs_scene;
            computeGSHistogram(model_gs_values, gs_model, 100);
            computeGSHistogram(scene_gs_values, gs_scene, 100);
            int hist_size = gs_model.rows();

            /*int removed_model = 0;
            int removed_scene = 0;
            for(int i=0; i < hist_size; i++)
            {
                if(gs_model(i,0) < (0.1f * static_cast<float>(model_gs_values.size ())))
                {
                    gs_model(i,0) = 0;
                    removed_model++;
                }

                if(gs_scene(i,0) < (0.1f * static_cast<float>(scene_gs_values.size ())))
                {
                    gs_scene(i,0) = 0;
                    removed_scene++;
                }
            }

            std::cout << "removed bins:" << removed_scene << "," << removed_model << std::endl;*/

            //histogram specification, adapt model values to scene values
            specifyRGBHistograms(gs_scene, gs_model, lookup, 1);

            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                float LRefm = recog_model->cloud_LAB_[label_indices[j][i]][0] * 255.f;
                int pos = std::floor (static_cast<float> (LRefm) / 255.f * hist_size);
                assert(pos < lookup.rows());
                float gs_specified = lookup(pos, 0);
                //std::cout << "gs specified:" << gs_specified << " size:" << hist_size << std::endl;
                LRefm = gs_specified * (255.f / static_cast<float>(hist_size)) / 255.f;
                recog_model->cloud_LAB_[label_indices[j][i]][0] = LRefm;
                recog_model->cloud_indices_specified_.push_back(label_indices[j][i]);
            }
        }
        else if(color_space_ == 1 || color_space_ == 5)
        {

            std::vector<Eigen::Vector3f> model_gs_values, scene_gs_values;

            //compute RGB histogram for the model points
            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                model_gs_values.push_back(recog_model->cloud_RGB_[label_indices[j][i]] * 255.f);
            }

            //compute RGB histogram for the explained points
            std::set<int>::iterator it;
            for(it=explained_indices_points.begin(); it != explained_indices_points.end(); it++)
            {
                scene_gs_values.push_back(scene_RGB_values_[*it] * 255.f);
            }

            int dim = 3;
            Eigen::MatrixXf gs_model, gs_scene;
            computeRGBHistograms(model_gs_values, gs_model, dim);
            computeRGBHistograms(scene_gs_values, gs_scene, dim);

            //histogram specification, adapt model values to scene values
            specifyRGBHistograms(gs_scene, gs_model, lookup, dim);

            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                for(int k=0; k < dim; k++)
                {
                    float color = recog_model->cloud_RGB_[label_indices[j][i]][k] * 255.f;
                    int pos = std::floor (static_cast<float> (color) / 255.f * 256);
                    float specified = lookup(pos, k);
                    recog_model->cloud_RGB_[label_indices[j][i]][k] = specified / 255.f;
                }
            }

            if(color_space_ == 5)
            {
                //transform specified RGB to lab
                for(size_t j=0; j < recog_model->cloud_LAB_.size(); j++)
                {
                    unsigned char rm = recog_model->cloud_RGB_[j][0] * 255;
                    unsigned char gm = recog_model->cloud_RGB_[j][1] * 255;
                    unsigned char bm = recog_model->cloud_RGB_[j][2] * 255;

                    float LRefm, aRefm, bRefm;
                    RGB2CIELAB (rm, gm, bm, LRefm, aRefm, bRefm); //this is called in parallel and initially fill values on static thing...
                    LRefm /= 100.0f; aRefm /= 120.0f; bRefm /= 120.0f;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)
                    recog_model->cloud_LAB_[j] = Eigen::Vector3f(LRefm, aRefm, bRefm);
                }
            }
        }
        else if(color_space_ == 2) //gray scale
        {
            std::vector<float> model_gs_values, scene_gs_values;

            //compute RGB histogram for the model points
            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                model_gs_values.push_back(recog_model->cloud_GS_[label_indices[j][i]] * 255.f);
            }

            //compute RGB histogram for the explained points
            std::set<int>::iterator it;
            for(it=explained_indices_points.begin(); it != explained_indices_points.end(); it++)
            {
                scene_gs_values.push_back(scene_GS_values_[*it] * 255.f);
            }

            Eigen::MatrixXf gs_model, gs_scene;
            computeGSHistogram(model_gs_values, gs_model);
            computeGSHistogram(scene_gs_values, gs_scene);

            //histogram specification, adapt model values to scene values
            specifyRGBHistograms(gs_scene, gs_model, lookup, 1);

            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                float LRefm = recog_model->cloud_GS_[label_indices[j][i]] * 255.f;
                int pos = std::floor (static_cast<float> (LRefm) / 255.f * 256);
                float gs_specified = lookup(pos, 0);
                LRefm = gs_specified / 255.f;
                recog_model->cloud_GS_[label_indices[j][i]] = LRefm;
            }
        }
        else if(color_space_ == 6)
        {
            std::vector<Eigen::Vector3f> model_gs_values, scene_gs_values;
            int dim = 3;

            //compute LAB histogram for the model points
            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                Eigen::Vector3f lab = recog_model->cloud_LAB_[label_indices[j][i]] * 255.f;
                lab[1] = (lab[1] + 255.f) / 2.f;
                lab[2] = (lab[2] + 255.f) / 2.f;

                for(int k=0; k < dim; k++)
                {
                    if(!(lab[k] >= 0 && lab[k] <= 255.f))
                    {
                        std::cout << lab[k] << " dim:" << dim << std::endl;
                        assert(lab[k] >= 0 && lab[k] <= 255.f);
                    }
                }
                model_gs_values.push_back(lab);
            }

            //compute LAB histogram for the explained points
            std::set<int>::iterator it;
            for(it=explained_indices_points.begin(); it != explained_indices_points.end(); it++)
            {
                Eigen::Vector3f lab = scene_LAB_values_[*it] * 255.f;
                lab[1] = (lab[1] + 255.f) / 2.f;
                lab[2] = (lab[2] + 255.f) / 2.f;
                for(int k=0; k < dim; k++)
                {
                    assert(lab[k] >= 0 && lab[k] <= 255.f);
                }
                scene_gs_values.push_back(lab);
            }

            Eigen::MatrixXf gs_model, gs_scene;
            computeRGBHistograms(model_gs_values, gs_model, dim);
            computeRGBHistograms(scene_gs_values, gs_scene, dim);

            //histogram specification, adapt model values to scene values
            specifyRGBHistograms(gs_scene, gs_model, lookup, dim);

            for (size_t i = 0; i < label_indices[j].size (); i++)
            {
                recog_model->cloud_indices_specified_.push_back(label_indices[j][i]);

                for(int k=0; k < dim; k++)
                {
                    float LRefm = recog_model->cloud_LAB_[label_indices[j][i]][k] * 255.f;
                    if(k > 0)
                    {
                        LRefm = (LRefm + 255.f) / 2.f;
                    }

                    int pos = std::floor (static_cast<float> (LRefm) / 255.f * 256);
                    assert(pos < lookup.rows());
                    float gs_specified = lookup(pos, k);

                    float diff = std::abs(LRefm - gs_specified);
                    assert(gs_specified >= 0 && gs_specified <= 255.f);
                    LRefm = gs_specified / 255.f;
                    assert(LRefm >= 0 && LRefm <= 1.f);
                    if(k > 0)
                    {
                        LRefm *= 2.f;
                        LRefm -= 1.f;

                        if(!(LRefm >= -1.f && LRefm <= 1.f))
                        {
                            std::cout << LRefm << " dim:" << k << " diff:" << diff << std::endl;
                            assert(LRefm >= -1.f && LRefm <= 1.f);
                        }
                    }
                    else
                    {
                        if(!(LRefm >= 0.f && LRefm <= 1.f))
                        {
                            std::cout << LRefm << " dim:" << k << std::endl;
                            assert(LRefm >= 0.f && LRefm <= 1.f);
                        }
                    }

                    recog_model->cloud_LAB_[label_indices[j][i]][k] = LRefm;
                }
            }
        }
    }
}

template<typename ModelT, typename SceneT>
bool
faat_pcl::GHV<ModelT, SceneT>::addModel (int i, boost::shared_ptr<GHVRecognitionModel<ModelT> > & recog_model)
{

    int model_id = i;
    bool is_planar_model = false;
    std::map<int, int>::iterator it1;
    it1 = model_to_planar_model_.find(model_id);
    if(it1 != model_to_planar_model_.end())
        is_planar_model = true;

    recog_model->cloud_.reset (new pcl::PointCloud<ModelT> (*visible_models_[i]));
    recog_model->complete_cloud_.reset (new pcl::PointCloud<ModelT> (*complete_models_[i]));

    size_t object_models_size = complete_models_.size() - planar_models_.size();
    float extra_weight = 1.f;
    if(extra_weights_.size() != 0 && (extra_weights_.size() == object_models_size))
        extra_weight = extra_weights_[i];

    if(object_ids_.size() == complete_models_.size()) {
        recog_model->id_s_ = object_ids_[i];
    }

    //bool handling_normals_b = handlingNormals(recog_model, i, is_planar_model, object_models_size);
    bool handling_normals_b = handlingNormals(recog_model, i, is_planar_model, complete_models_.size());
    if(!handling_normals_b)
    {
        PCL_WARN("Handling normals returned false\n");
        return false;
    }

    //pcl::ScopeTime tt_nn("Computing outliers and explained points...");
    std::vector<int> explained_indices;
    std::vector<float> outliers_weight;
    std::vector<float> explained_indices_distances;

    //which point first from the scene is explained by a point j_k with dist d_k from the model
    std::map<int, boost::shared_ptr<std::vector<std::pair<int, float> > > > model_explains_scene_points;
    std::map<int, boost::shared_ptr<std::vector<std::pair<int, float> > > > model_explains_scene_points_color_weight;
    std::map<int, boost::shared_ptr<std::vector<std::pair<int, float> > > >::iterator it;

    outliers_weight.resize (recog_model->cloud_->points.size ());
    recog_model->outlier_indices_.resize (recog_model->cloud_->points.size ());
    recog_model->outliers_3d_indices_.resize (recog_model->cloud_->points.size ());
    recog_model->color_outliers_indices_.resize (recog_model->cloud_->points.size ());
    recog_model->scene_point_explained_by_hypothesis_.resize(scene_cloud_downsampled_->points.size(), false);

    float rgb_m;
    bool exists_m;

    if(!is_planar_model && !ignore_color_even_if_exists_)
    {
        //compute cloud LAB values for model visible points
        recog_model->cloud_LAB_.resize(recog_model->cloud_->points.size());
        recog_model->cloud_RGB_.resize(recog_model->cloud_->points.size());
        recog_model->cloud_GS_.resize(recog_model->cloud_->points.size());
        for(size_t j=0; j < recog_model->cloud_LAB_.size(); j++)
        {

            pcl::for_each_type<FieldListM> (
                        pcl::CopyIfFieldExists<typename CloudM::PointType, float> (
                            recog_model->cloud_->points[j],
                            "rgb", exists_m, rgb_m));

            uint32_t rgb = *reinterpret_cast<int*> (&rgb_m);
            unsigned char rm = (rgb >> 16) & 0x0000ff;
            unsigned char gm = (rgb >> 8) & 0x0000ff;
            unsigned char bm = (rgb) & 0x0000ff;

            float LRefm, aRefm, bRefm;
            RGB2CIELAB (rm, gm, bm, LRefm, aRefm, bRefm); //this is called in parallel and initially fill values on static thing...
            LRefm /= 100.0f; aRefm /= 120.0f; bRefm /= 120.0f;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)
            recog_model->cloud_LAB_[j] = Eigen::Vector3f(LRefm, aRefm, bRefm);

            float rmf,gmf,bmf;
            rmf = static_cast<float>(rm) / 255.f;
            gmf = static_cast<float>(gm) / 255.f;
            bmf = static_cast<float>(bm) / 255.f;

            recog_model->cloud_RGB_[j] = Eigen::Vector3f(rmf, gmf, bmf);

            recog_model->cloud_GS_[j] = (rmf + gmf + bmf) / 3.f;
        }
    }

    recog_model->inlier_indices_.resize(recog_model->cloud_->points.size ());
    recog_model->inlier_distances_.resize(recog_model->cloud_->points.size ());
    for (size_t kk = 0; kk < recog_model->cloud_->points.size (); kk++)
    {
        octree_scene_downsampled_->radiusSearch (recog_model->cloud_->points[kk], inliers_threshold_,
                                                 recog_model->inlier_indices_[kk], recog_model->inlier_distances_[kk],
                                                 std::numeric_limits<int>::max ());
    }

    Eigen::MatrixXf lookup;
    if(!is_planar_model && !ignore_color_even_if_exists_ && use_histogram_specification_)
    {
        specifyColor(i, lookup, recog_model);
    }

    float inliers_gaussian = 2 * std::pow(inliers_threshold_, 2);
    float inliers_gaussian_locker = 2 * std::pow(inliers_threshold_ + resolution_, 2);

    {
        //pcl::ScopeTime t("NN");
        size_t o = 0;
        int o_color = 0;
        int o_3d = 0;
        //Goes through the visible model points and finds scene points within a radius neighborhood
        //If in this neighborhood, there are no scene points, model point is considered outlier
        //If there are scene points, the model point is associated with the scene point, together with its distance
        //A scene point might end up being explained by the multiple model points

        int bad_normals = 0;
        float sigma = 2.f * color_sigma_ab_ * color_sigma_ab_;
        float sigma_y = 2.f * color_sigma_l_ * color_sigma_l_;
        Eigen::Vector3f yuvm, yuvs;

        for (size_t i = 0; i < recog_model->cloud_->points.size (); i++)
        {
            bool outlier = false;
            int outlier_type = 0;

            std::vector<int> & nn_indices = recog_model->inlier_indices_[i];
            std::vector<float> & nn_distances = recog_model->inlier_distances_[i];

            /*if (octree_scene_downsampled_->radiusSearch (recog_model->cloud_->points[i], inliers_threshold_, nn_indices, nn_distances,
                                                         std::numeric_limits<int>::max ()) > 0)*/
            if(nn_indices.size() > 0)
            {

                //std::vector<bool> outliers(nn_distances.size(), false);

                std::vector<float> weights;
                for (size_t k = 0; k < nn_distances.size () && !is_planar_model; k++)
                {
                    //check color
                    if (!ignore_color_even_if_exists_)
                    {
                        float color_weight = 1.f;

                        if(color_space_ == 0 || color_space_ == 5 || color_space_ == 6)
                        {
                            yuvm = recog_model->cloud_LAB_[i];
                            yuvs = scene_LAB_values_[nn_indices[k]];

                            color_weight = std::exp ((-0.5f * (yuvm[0] - yuvs[0]) * (yuvm[0] - yuvs[0])) / (sigma_y));
                            color_weight *= std::exp ((-0.5f * (yuvm[1] - yuvs[1]) * (yuvm[1] - yuvs[1])) / (sigma));
                            color_weight *= std::exp ((-0.5f * (yuvm[2] - yuvs[2]) * (yuvm[2] - yuvs[2])) / (sigma));
                        }
                        else if(color_space_ == 1)
                        {
                            yuvm = recog_model->cloud_RGB_[i];
                            yuvs = scene_RGB_values_[nn_indices[k]];

                            color_weight = std::exp ((-0.5f * (yuvm[0] - yuvs[0]) * (yuvm[0] - yuvs[0])) / (sigma));
                            color_weight *= std::exp ((-0.5f * (yuvm[1] - yuvs[1]) * (yuvm[1] - yuvs[1])) / (sigma));
                            color_weight *= std::exp ((-0.5f * (yuvm[2] - yuvs[2]) * (yuvm[2] - yuvs[2])) / (sigma));
                        }
                        else if(color_space_ == 2)
                        {
                            float yuvm = recog_model->cloud_GS_[i];
                            float yuvs = scene_GS_values_[nn_indices[k]];

                            color_weight = std::exp ((-0.5f * (yuvm - yuvs) * (yuvm - yuvs)) / (sigma_y));
                            //color_weight = 1.f - (std::abs(yuvm - yuvs));
                        }
                        else if(color_space_ == 3)
                        {
                            float yuvm = recog_model->cloud_LAB_[i][0];
                            float yuvs = scene_LAB_values_[nn_indices[k]][0];

                            color_weight = std::exp ((-0.5f * (yuvm - yuvs) * (yuvm - yuvs)) / (sigma_y));
                            //color_weight = 1.f - (std::abs(yuvm - yuvs));

                        }

                        float d = nn_distances[k];
                        float d_weight = std::exp( -(d / inliers_gaussian_locker));

                        //float d_weight = std::exp(-( d / (inliers_threshold_ * 3.f)));
                        /*float d = nn_distances[k];
                        float d_weight = -(d / (inliers_threshold_)) + 1;
                        color_weight *= d_weight;*/

                        //scene_LAB_values.push_back(Eigen::Vector3f(yuvs));

                        //weights.push_back(color_weight);
                        weights.push_back(color_weight * d_weight);
                    }
                }

                std::sort(weights.begin(), weights.end(), std::greater<float>());

                if(is_planar_model || ignore_color_even_if_exists_ || weights[0] > best_color_weight_) //best weight is not an outlier
                {
                    for (size_t k = 0; k < nn_distances.size (); k++)
                    {
                        std::pair<int, float> pair = std::make_pair (i, nn_distances[k]); //i is a index to a model point and then distance
                        //nn_distances is squared!!

                        it = model_explains_scene_points.find (nn_indices[k]);
                        if (it == model_explains_scene_points.end ())
                        {
                            boost::shared_ptr<std::vector<std::pair<int, float> > > vec (new std::vector<std::pair<int, float> > ());
                            vec->push_back (pair);
                            model_explains_scene_points[nn_indices[k]] = vec;
                        }
                        else
                        {
                            it->second->push_back (pair);
                        }

                        if(!is_planar_model && !ignore_color_even_if_exists_)
                        {
                            //std::pair<int, float> pair_color = std::make_pair (i, weights_not_sorted[k]);
                            std::pair<int, float> pair_color = std::make_pair (i, weights[0]);
                            it = model_explains_scene_points_color_weight.find (nn_indices[k]);
                            if (it == model_explains_scene_points_color_weight.end ())
                            {
                                boost::shared_ptr<std::vector<std::pair<int, float> > > vec (new std::vector<std::pair<int, float> > ());
                                vec->push_back (pair_color);
                                model_explains_scene_points_color_weight[nn_indices[k]] = vec;
                            }
                            else
                            {
                                it->second->push_back (pair_color);
                            }
                        }
                    }
                }
                else
                {
                    recog_model->color_outliers_indices_[o_color] = static_cast<int> (i);
                    outlier = true;
                    o_color++;
                    outlier_type = 1;
                }
            }
            else
            {
                recog_model->outliers_3d_indices_[o_3d] = static_cast<int> (i);
                outlier = true;
                o_3d++;
            }

            if(outlier)
            {
                //weight outliers based on noise model
                //model points close to occlusion edges or with perpendicular normals
                float d_weight = 1.f;
                //std::cout << "is an outlier" << is_planar_model << " " << occ_edges_available_ << std::endl;

                if(!is_planar_model)
                {
                    //std::cout << "going to weight based on normals..." << std::endl;
                    Eigen::Vector3f normal_p = recog_model->normals_->points[i].getNormalVector3fMap();
                    Eigen::Vector3f normal_vp = Eigen::Vector3f::UnitZ() * -1.f;
                    normal_p.normalize ();
                    normal_vp.normalize ();

                    float dot = normal_vp.dot(normal_p);
                    float angle = pcl::rad2deg(acos(dot));
                    if (angle > 60.f)
                    {
                        if(outlier_type)
                            o_color--;
                        else
                            o_3d--;

                        // [60,75) => 0.5
                        // [75,90) => 0.25
                        // >90 => 0

                        /*if(angle >= 90.f)
                            d_weight = 0.25f;
                        else*/
                        d_weight = d_weight_for_bad_normals_;

                        bad_normals++;
                    }
                }

                outliers_weight[o] = regularizer_ * d_weight;
                recog_model->outlier_indices_[o] = static_cast<int> (i);
                o++;
            }
        }

        outliers_weight.resize (o);
        recog_model->outlier_indices_.resize (o);
        recog_model->outliers_3d_indices_.resize (o_3d);
        recog_model->color_outliers_indices_.resize (o_color);
        //std::cout << "outliers with bad normals for sensor:" << bad_normals << std::endl;
    }

    //using mean
    recog_model->outliers_weight_ = (std::accumulate (outliers_weight.begin (), outliers_weight.end (), 0.f) / static_cast<float> (outliers_weight.size ()));
    //using median
    //std::sort(outliers_weight.begin(), outliers_weight.end());
    //recog_model->outliers_weight_ = outliers_weight[outliers_weight.size() / 2.f];

    if (outliers_weight.size () == 0)
        recog_model->outliers_weight_ = 1.f;

    int p = 0;

    //float inliers_gaussian = 2 * std::pow(inliers_threshold_ + resolution_, 2);
    for (it = model_explains_scene_points.begin (); it != model_explains_scene_points.end (); it++, p++)
    {
        //ATTENTION, TODO => use normal information to select closest!

        Eigen::Vector3f scene_p_normal = scene_normals_->points[it->first].getNormalVector3fMap ();
        scene_p_normal.normalize();

        bool use_normal_info_for_closest = false;
        size_t closest = 0;
        float min_d = std::numeric_limits<float>::infinity ();
        for (size_t i = 0; i < it->second->size (); i++)
        {
            if(use_normal_info_for_closest)
            {
                Eigen::Vector3f model_p_normal;
                if(use_normals_from_visible_ && (visible_normal_models_.size() == complete_models_.size()))
                {
                    model_p_normal = recog_model->normals_from_visible_->points[it->second->at (i).first].getNormalVector3fMap ();
                }
                else
                {
                    model_p_normal = recog_model->normals_->points[it->second->at (i).first].getNormalVector3fMap ();
                }
                model_p_normal.normalize();

//                float d = it->second->at (i).second;
                //float d_weight = std::exp( -(d / inliers_gaussian));
                //float dotp = scene_p_normal.dot (model_p_normal); //[-1,1] from antiparallel trough perpendicular to parallel

                //float w = d_weight * dotp;

            }
            else
            {
                if (it->second->at (i).second < min_d)
                {
                    min_d = it->second->at (i).second;
                    closest = i;
                }
            }
        }

        float d = it->second->at (closest).second;
        float d_weight = std::exp( -(d / inliers_gaussian));

        //it->first is index to scene point
        //using normals to weight inliers
        Eigen::Vector3f model_p_normal;

        if(use_normals_from_visible_ && (visible_normal_models_.size() == complete_models_.size()))
        {
            model_p_normal = recog_model->normals_from_visible_->points[it->second->at (closest).first].getNormalVector3fMap ();
        }
        else
        {
            model_p_normal = recog_model->normals_->points[it->second->at (closest).first].getNormalVector3fMap ();
        }
        model_p_normal.normalize();

        bool use_dot_ = false;
        float dotp = scene_p_normal.dot (model_p_normal); //[-1,1] from antiparallel trough perpendicular to parallel

        if(use_dot_)
        {
            if (dotp < 0.f)
                dotp = 0.f;
        }
        else
        {
            if(dotp < -1.f) dotp = -1.f;
            if(dotp > 1.f) dotp = 1.f;

            float angle = pcl::rad2deg(acos(dotp));

            if(angle > 90.f) //ATTENTION!
                dotp = 0;
            else
                dotp = (1.f - angle / 90.f);
        }

        //else
        //dotp = 1.f; //ATTENTION: Deactivated normal weight!

        /*if(model_p_normal.norm() > 1)
        {
            std::cout << "model_p normal:" << model_p_normal.norm() << std::endl;
        }
        assert(model_p_normal.norm() <= 1);
        assert(scene_p_normal.norm() <= 1);
        assert(std::abs(dotp) <= 1);
        assert(d_weight <= 1);
        assert(extra_weight <= 1);*/

        if (!is_planar_model && !ignore_color_even_if_exists_)
        {
            std::map<int, boost::shared_ptr<std::vector<std::pair<int, float> > > >::iterator it_color;
            it_color = model_explains_scene_points_color_weight.find(it->first);
            if(it != model_explains_scene_points_color_weight.end())
            {
                d_weight *= it_color->second->at(closest).second;
            }


            /*float rgb_s;
            bool exists_s;

            typedef pcl::PointCloud<SceneT> CloudS;
            typedef typename pcl::traits::fieldList<typename CloudS::PointType>::type FieldListS;

            pcl::for_each_type<FieldListS> (
                        pcl::CopyIfFieldExists<typename CloudS::PointType, float> (scene_cloud_downsampled_->points[it->first],
                                                                                   "rgb", exists_s, rgb_s));

            if(exists_s)
            {
                uint32_t rgb = *reinterpret_cast<int*> (&rgb_s);
                unsigned char rs = (rgb >> 16) & 0x0000ff;
                unsigned char gs = (rgb >> 8) & 0x0000ff;
                unsigned char bs = (rgb) & 0x0000ff;

                float LRefs, aRefs, bRefs;
                RGB2CIELAB (rs, gs, bs, LRefs, aRefs, bRefs);
                LRefs /= 100.0f; aRefs /= 120.0f; bRefs /= 120.0f;    //normalized LAB components (0<L<1, -1<a<1, -1<b<1)

                scene_LAB_values.push_back(Eigen::Vector3f( LRefs, (aRefs + 1) / 2.f, (bRefs + 1) / 2.f));
            }*/
        }

        if( (d_weight * dotp * extra_weight > 1.f) || pcl_isnan(d_weight * dotp * extra_weight) || pcl_isinf(d_weight * dotp * extra_weight))
        {
            std::cout << d_weight * dotp * extra_weight << std::endl;
        }

        assert((d_weight * dotp * extra_weight) <= 1.0001f);
        explained_indices.push_back (it->first);
        explained_indices_distances.push_back (d_weight * dotp * extra_weight);
        recog_model->scene_point_explained_by_hypothesis_[it->first] = true; //this scene point is explained by this hypothesis
    }

    recog_model->bad_information_ =  static_cast<int> (recog_model->outlier_indices_.size ());

    //compute the amount of information for explained scene points (color)
    float mean = std::accumulate(explained_indices_distances.begin(), explained_indices_distances.end(), 0.f) / static_cast<float>(explained_indices_distances.size());
    if(explained_indices.size() == 0)
    {
        mean = 0.f;
    }

    assert(mean <= 1.f);
    recog_model->mean_ = mean;

    //modify the explained weights for planar models if color is being used
    //ATTENTION: check this... looks weird!
    if(!ignore_color_even_if_exists_)
    {
        std::map<int, int>::iterator it1;
        it1 = model_to_planar_model_.find(static_cast<int>(i));
        if(it1 != model_to_planar_model_.end())
        {
            //PCL_WARN("Plane found... decrease weight.\n");
            for(size_t k=0; k < explained_indices_distances.size(); k++)
            {
                explained_indices_distances[k] *= best_color_weight_ / 2.f;
            }
        }
    }
    else
    {
        std::map<int, int>::iterator it1;
        it1 = model_to_planar_model_.find(static_cast<int>(i));
        if(it1 != model_to_planar_model_.end())
        {
            PCL_WARN("Plane found... decrease weight.\n");
            for(size_t k=0; k < explained_indices_distances.size(); k++)
            {
                explained_indices_distances[k] *= best_color_weight_;
            }
        }
    }

    recog_model->hyp_penalty_ = 0; //ATTENTION!

    recog_model->explained_ = explained_indices;
    recog_model->explained_distances_ = explained_indices_distances;
    recog_model->id_ = i;
    //std::cout << "Model:" << recog_model->complete_cloud_->points.size() << " " << recog_model->cloud_->points.size() << std::endl;
    return true;
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::computeClutterCueAtOnce ()
{

    //compute all scene points that are explained by the hypothesis
    std::set<int> explained_points;

    for (int j = 0; j < static_cast<int> (recognition_models_.size ()); j++)
    {
        for (int i = 0; i < static_cast<int> (recognition_models_[j]->explained_.size ()); i++)
        {
            explained_points.insert(recognition_models_[j]->explained_[i]);
        }
    }

    std::vector<int> explained_points_vec;
    explained_points_vec.assign(explained_points.begin(), explained_points.end());

    std::vector<int> scene_to_unique(scene_cloud_downsampled_->size(),-1);
    for(size_t i=0; i < explained_points_vec.size(); i++)
        scene_to_unique[explained_points_vec[i]] = i;

    float rn_sqr = radius_neighborhood_GO_ * radius_neighborhood_GO_;

    //find neighbours in clutter radius for all explaiend_points
    std::vector<std::vector<int> > nn_indices_all_points(explained_points_vec.size());
    std::vector<std::vector<float> > nn_distances_all_points(explained_points_vec.size());

#pragma omp parallel for schedule(dynamic, 1) num_threads(std::min(max_threads_, omp_get_num_procs()))
    for(size_t k=0; k < explained_points_vec.size(); k++)
    {
        octree_scene_downsampled_->radiusSearch (scene_cloud_downsampled_->points[explained_points_vec[k]],
                                                 radius_neighborhood_GO_, nn_indices_all_points[k],
                                                 nn_distances_all_points[k], std::numeric_limits<int>::max ());
    }

    float min_clutter_dist = std::pow(inliers_threshold_ * 0.f, 2.f);

#pragma omp parallel for schedule(dynamic, 1) num_threads(std::min(max_threads_, omp_get_num_procs()))
    for (int j = 0; j < static_cast<int> (recognition_models_.size ()); j++)
    {

        std::vector< std::pair<int, float> > unexplained_points_per_model;
        std::pair<int, float> def_value = std::make_pair(-1, std::numeric_limits<float>::infinity());
        unexplained_points_per_model.resize(scene_cloud_downsampled_->points.size(), def_value);

        boost::shared_ptr<GHVRecognitionModel<ModelT> > recog_model = recognition_models_[j];

        int model_id = recog_model->id_;
        bool is_planar_model = false;
        std::map<int, int>::iterator it1;
        it1 = model_to_planar_model_.find(model_id);
        if(it1 != model_to_planar_model_.end())
            is_planar_model = true;

        for (int i = 0; i < static_cast<int> (recog_model->explained_.size ()); i++)
        {
            int s_id_exp = recog_model->explained_[i];
            int idx_to_unique = scene_to_unique[s_id_exp];

            for (size_t k = 0; k < nn_indices_all_points[idx_to_unique].size (); k++)
            {
                int sidx = nn_indices_all_points[idx_to_unique][k]; //in the neighborhood of an explained point (idx_to_ep)
                if(recog_model->scene_point_explained_by_hypothesis_[sidx])
                    continue;

                assert(recog_model->scene_point_explained_by_hypothesis_[recog_model->explained_[i]]);
                assert(sidx != recog_model->explained_[i]);

                float d = (scene_cloud_downsampled_->points[recog_model->explained_[i]].getVector3fMap ()
                           - scene_cloud_downsampled_->points[sidx].getVector3fMap ()).squaredNorm ();

                //float curvature = scene_curvature_[s_id_exp];
                //std::cout << "curvature:" << curvature << std::endl;

                if((d > min_clutter_dist) && (d < unexplained_points_per_model[sidx].second))
                {
                    //there is an explained point which is closer to this unexplained point
                    unexplained_points_per_model[sidx].second = d;
                    unexplained_points_per_model[sidx].first = s_id_exp;
                }
            }
        }

        recog_model->unexplained_in_neighborhood.resize (scene_cloud_downsampled_->points.size ());
        recog_model->unexplained_in_neighborhood_weights.resize (scene_cloud_downsampled_->points.size ());

        float clutter_gaussian = 2 * rn_sqr;
        int p=0;
        for(size_t i=0; i < unexplained_points_per_model.size(); i++)
        {
            int sidx = unexplained_points_per_model[i].first;
            if(sidx < 0)
                continue;

            //sidx is the closest explained point to the unexplained point

            assert(recog_model->scene_point_explained_by_hypothesis_[sidx]);
            assert(!recog_model->scene_point_explained_by_hypothesis_[i]);

            //point i is unexplained and in the neighborhood of sidx (explained point)
            recog_model->unexplained_in_neighborhood[p] = i;

            float d = unexplained_points_per_model[i].second;
            float d_weight;
            bool use_exp_ = use_clutter_exp_;
            if(use_exp_)
            {
                d_weight = std::exp( -(d / clutter_gaussian));
            }
            else
            {
                d_weight = -(d / rn_sqr) + 1; //points that are close have a strong weight
            }

            //using normals to weight clutter points
            const Eigen::Vector3f & scene_p_normal = scene_normals_->points[sidx].getNormalVector3fMap ();
            const Eigen::Vector3f & model_p_normal = scene_normals_->points[i].getNormalVector3fMap ();
            float dotp = scene_p_normal.dot (model_p_normal); //[-1,1] from antiparallel trough perpendicular to parallel

            if (dotp < 0)
                dotp = 0.f;

            float w = d_weight * dotp;

            float curvature = scene_curvature_[sidx];

            if ( (clusters_cloud_->points[i].label != 0 || use_super_voxels_) &&
                 (clusters_cloud_->points[i].label == clusters_cloud_->points[sidx].label)
                 && !is_planar_model
                 && (curvature < 0.015)
                 /*&& (curvature < 0.01f)*/)
                //&& penalize_with_smooth_segmentation[clusters_cloud_->points[i].label]) //ATTENTION!
            {
                w = 1.f; //ATTENTION!
                assert(clusters_cloud_->points[i].label != 0 || use_super_voxels_);
                recog_model->unexplained_in_neighborhood_weights[p] = clutter_regularizer_ * w;
            }
            else
            {
                recog_model->unexplained_in_neighborhood_weights[p] = w;
            }

            p++;
        }

        recog_model->unexplained_in_neighborhood_weights.resize (p);
        recog_model->unexplained_in_neighborhood.resize (p);
    }
}

//######### VISUALIZATION FUNCTIONS #####################

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::getOutliersForAcceptedModels(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & outliers_cloud)
{
    for(size_t i=0; i < recognition_models_.size(); i++)
    {
        if(mask_[i])
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_points(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*(recognition_models_[i]->cloud_), recognition_models_[i]->outlier_indices_, *outlier_points);
            outliers_cloud.push_back(outlier_points);
        }
    }
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::getOutliersForAcceptedModels(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & outliers_cloud_color,
                                                            std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & outliers_cloud_3d)
{
    for(size_t i=0; i < recognition_models_.size(); i++)
    {
        if(mask_[i])
        {
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_points(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*(recognition_models_[i]->cloud_), recognition_models_[i]->color_outliers_indices_, *outlier_points);
                outliers_cloud_color.push_back(outlier_points);
            }

            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_points(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*(recognition_models_[i]->cloud_), recognition_models_[i]->outliers_3d_indices_, *outlier_points);
                outliers_cloud_3d.push_back(outlier_points);
            }
        }
    }
}

template<typename ModelT, typename SceneT>
void
faat_pcl::GHV<ModelT, SceneT>::visualizeGOCues (const std::vector<bool> & active_solution,
                                                float cost, int times_evaluated)
{
    std::cout << "visualizeGOCues:" << visualize_go_cues_ << std::endl;

    vis_go_cues_->removeAllPointClouds();
    vis_go_cues_->removeAllShapes();

    int viewport_scene_and_hypotheses_;
    int viewport_model_cues_;
    int viewport_smooth_seg_;
    int viewport_scene_cues_;

    vis_go_cues_->createViewPort(0, 0, 0.5, 0.5, viewport_scene_cues_);
    vis_go_cues_->createViewPort(0.5, 0, 1, 0.5, viewport_model_cues_);
    vis_go_cues_->createViewPort(0.5, 0.5, 1, 1, viewport_smooth_seg_);
    vis_go_cues_->createViewPort(0, 0.5, 0.5, 1, viewport_scene_and_hypotheses_);

    std::string cost_str;
    std::ostringstream out;

    out << "Cost: " << std::setprecision(2) << cost;
    out << " , #Evaluations: " << times_evaluated;
    cost_str = out.str();

    bool for_paper_ = true;
    bool show_weights_with_color_fading_ = true;

    if(for_paper_)
    {
        vis_go_cues_->setBackgroundColor (1, 1, 1);
    }
    else
    {
        vis_go_cues_->setBackgroundColor (0, 0, 0);
        vis_go_cues_->addText (cost_str, 1, 30, 16, 1, 1, 1, "cost_text", viewport_scene_and_hypotheses_);
        vis_go_cues_->addText ("Model inliers & outliers", 1, 30, 16, 1, 1, 1, "inliers_outliers", viewport_model_cues_);
        vis_go_cues_->addText ("Smooth segmentation", 1, 30, 16, 1, 1, 1, "smoot", viewport_smooth_seg_);
        vis_go_cues_->addText ("Explained, multiple assignment & clutter", 1, 30, 16, 1, 1, 1, "scene_cues", viewport_scene_cues_);

    }

    //scene
    pcl::visualization::PointCloudColorHandlerCustom<SceneT> random_handler_scene (scene_cloud_downsampled_, 200, 0, 0);
    vis_go_cues_->addPointCloud<SceneT> (scene_cloud_downsampled_, random_handler_scene, "scene_cloud", viewport_scene_and_hypotheses_);

    //smooth segmentation
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smooth_cloud_ =  getSmoothClustersRGBCloud();
    if(smooth_cloud_)
    {
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> random_handler (smooth_cloud_);
        vis_go_cues_->addPointCloud<pcl::PointXYZRGBA> (smooth_cloud_, random_handler, "smooth_cloud", viewport_smooth_seg_);
    }

    //display active hypotheses
    for(size_t i=0; i < active_solution.size(); i++)
    {
        if(active_solution[i])
        {
            //complete models

            std::stringstream m;
            m << "model_" << i;

            if(poses_ply_.size() == 0)
            {
                pcl::visualization::PointCloudColorHandlerCustom<ModelT> handler_model (complete_models_[i], 0, 255, 0);
                vis_go_cues_->addPointCloud<ModelT> (complete_models_[i], handler_model, m.str(), viewport_scene_and_hypotheses_);
            }
            else
            {
                int model_id = i;
                bool is_planar_model = false;
                std::map<int, int>::iterator it1;
                it1 = model_to_planar_model_.find(model_id);
                if(it1 != model_to_planar_model_.end())
                    is_planar_model = true;

                if(!is_planar_model)
                {
                    vis_go_cues_->addModelFromPLYFile (ply_paths_[i], poses_ply_[i], m.str (), viewport_scene_and_hypotheses_);
                }
                else
                {
                    vis_go_cues_->addPolygonMesh (*(planar_models_[it1->second].convex_hull_), m.str(), viewport_scene_and_hypotheses_);
                }
            }

            //model inliers and outliers
            std::stringstream cluster_name;
            cluster_name << "visible" << i;

            typename pcl::PointCloud<ModelT>::Ptr outlier_points (new pcl::PointCloud<ModelT> ());
            for (size_t j = 0; j < recognition_models_[i]->outlier_indices_.size (); j++)
            {
                ModelT c_point;
                c_point.getVector3fMap () = recognition_models_[i]->cloud_->points[recognition_models_[i]->outlier_indices_[j]].getVector3fMap ();
                outlier_points->push_back (c_point);
            }

            pcl::visualization::PointCloudColorHandlerCustom<ModelT> random_handler (recognition_models_[i]->cloud_, 255, 90, 0);
            vis_go_cues_->addPointCloud<ModelT> (recognition_models_[i]->cloud_, random_handler, cluster_name.str (), viewport_model_cues_);

            cluster_name << "_outliers";

            pcl::visualization::PointCloudColorHandlerCustom<ModelT> random_handler_out (outlier_points, 0, 94, 22);
            vis_go_cues_->addPointCloud<ModelT> (outlier_points, random_handler_out, cluster_name.str (), viewport_model_cues_);
        }
    }

    vis_go_cues_->setRepresentationToSurfaceForAllActors();

    //display scene cues (explained points, multiply explained, clutter (smooth and normal)
    vis_go_cues_->addPointCloud<SceneT> (scene_cloud_downsampled_, random_handler_scene, "scene_cloud_viewport", viewport_scene_cues_);
    vis_go_cues_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "scene_cloud_viewport");

    //clutter...
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clutter (new pcl::PointCloud<pcl::PointXYZRGB> ());
    typename pcl::PointCloud<SceneT>::Ptr clutter_smooth (new pcl::PointCloud<SceneT> ());
    for (size_t j = 0; j < unexplained_by_RM_neighboorhods.size (); j++)
    {
        if(unexplained_by_RM_neighboorhods[j] >= (clutter_regularizer_ - 0.01f) && explained_by_RM_[j] == 0 && (clusters_cloud_->points[j].label != 0 || use_super_voxels_))
        {
            SceneT c_point;
            c_point.getVector3fMap () = scene_cloud_downsampled_->points[j].getVector3fMap ();
            clutter_smooth->push_back (c_point);
        }
        else if (unexplained_by_RM_neighboorhods[j] > 0 && explained_by_RM_[j] == 0)
        {
            pcl::PointXYZRGB c_point;
            c_point.getVector3fMap () = scene_cloud_downsampled_->points[j].getVector3fMap ();

            if(show_weights_with_color_fading_)
            {
                c_point.r = round(255.0 * unexplained_by_RM_neighboorhods[j]);
                c_point.g = 40;
                c_point.b = round(255.0 * unexplained_by_RM_neighboorhods[j]);
            }
            else
            {
                c_point.r = 255.0;
                c_point.g = 40;
                c_point.b = 255.0;
            }
            clutter->push_back (c_point);
        }
    }

    //explained
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr explained_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    //typename pcl::PointCloud<SceneT>::Ptr explained_points (new pcl::PointCloud<SceneT> ());
    for (size_t j = 0; j < explained_by_RM_.size (); j++)
    {
        if (explained_by_RM_[j] == 1)
        {
            pcl::PointXYZRGB c_point;

            //if(show_weights_with_color_fading_)
            //{
            c_point.getVector3fMap () = scene_cloud_downsampled_->points[j].getVector3fMap ();
            c_point.b = 100 + explained_by_RM_distance_weighted[j] * 155;
            c_point.r = c_point.g = 0;
            //}
            //else
            //{
            //    c_point.getVector3fMap () = scene_cloud_downsampled_->points[j].getVector3fMap ();
            //    c_point.b = 255;
            //    c_point.r = c_point.g = 0;
            //}
            explained_points->push_back (c_point);
        }
    }

    //duplicity
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr duplicity_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    for (size_t j = 0; j < explained_by_RM_.size (); j++)
    {
        if (explained_by_RM_[j] > 1)
        {
            pcl::PointXYZRGB c_point;
            c_point.getVector3fMap () = scene_cloud_downsampled_->points[j].getVector3fMap ();
            float curv_weight = getCurvWeight(scene_curvature_[j]);

            if(multiple_assignment_penalize_by_one_ == 1)
            {
                c_point.r = c_point.g = c_point.b = 0;
                c_point.g = curv_weight * duplicy_weight_test_ * 255;
            }
            else if(multiple_assignment_penalize_by_one_ == 2)
            {
                if(show_weights_with_color_fading_)
                {
                    c_point.r = c_point.g = c_point.b = 0;
                    c_point.g = std::min(duplicates_by_RM_weighted_[j],1.0) * 255;
                }
                else
                {
                    c_point.r = 0;
                    c_point.g = 0;
                    c_point.b = 0;
                }
            }
            else
            {
                c_point.r = c_point.g = c_point.b = 0;
                c_point.g = 255;
            }

            duplicity_points->push_back (c_point);
        }
    }

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> random_handler_clutter (clutter);
    vis_go_cues_->addPointCloud<pcl::PointXYZRGB> (clutter, random_handler_clutter, "clutter", viewport_scene_cues_);
    vis_go_cues_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "clutter");

    pcl::visualization::PointCloudColorHandlerCustom<SceneT> random_handler_clutter_smooth (clutter_smooth, 255, 255, 0);
    vis_go_cues_->addPointCloud<SceneT> (clutter_smooth, random_handler_clutter_smooth, "clutter_smooth", viewport_scene_cues_);
    vis_go_cues_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "clutter_smooth");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> random_handler_explained (explained_points);
    vis_go_cues_->addPointCloud<pcl::PointXYZRGB> (explained_points, random_handler_explained, "explained", viewport_scene_cues_);
    vis_go_cues_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "explained");

    //pcl::visualization::PointCloudColorHandlerCustom<SceneT> random_handler_dup (duplicity_points, 200, 200, 200);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> random_handler_dup (duplicity_points);

    vis_go_cues_->addPointCloud<pcl::PointXYZRGB> (duplicity_points, random_handler_dup, "dup", viewport_scene_cues_);
    vis_go_cues_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "dup");
    vis_go_cues_->spin();
}

#define PCL_INSTANTIATE_faatGoHV_1(T1,T2) template class FAAT_REC_API faat_pcl::GlobalHypothesesVerification_1<T1,T2>;
