/*
 * noise_models.cpp
 *
 *  Created on: Oct 28, 2013
 *      Author: aitor
 */

#include "noise_model_based_cloud_integration.h"
#include <pcl/common/angles.h>
#include <v4r/OREdgeDetector/organized_edge_detection.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include "miscellaneous.h"

template<typename PointT>
faat_pcl::utils::NMBasedCloudIntegration<PointT>::NMBasedCloudIntegration ()
{
    octree_resolution_ = 0.005f;
    min_weight_ = 0.9f;
    min_points_per_voxel_ = 0;
}

template<typename PointT>
void
faat_pcl::utils::NMBasedCloudIntegration<PointT>::compute (PointTPtr & output)
{

    input_clouds_used_.resize(input_clouds_.size());
    for(size_t i=0; i < input_clouds_.size(); i++)
    {
        input_clouds_used_[i].reset(new pcl::PointCloud<PointT>(*input_clouds_[i]));
    }

    //process clouds and weights to remove points based on distance and add weights based on noise
    float bad_value = std::numeric_limits<float>::quiet_NaN();
    float max_distance_ = 5.f;
    float start_dist = 1.f;
    for(size_t i=0; i < input_clouds_used_.size(); i++)
    {
        for(size_t k=0; k < input_clouds_used_[i]->points.size(); k++)
        {
            if(!pcl_isfinite(input_clouds_used_[i]->points[k].z))
                continue;

            float dist = input_clouds_used_[i]->points[k].getVector3fMap().norm();

            if(dist > max_distance_)
            {
                input_clouds_used_[i]->points[k].x = input_clouds_used_[i]->points[k].y = input_clouds_used_[i]->points[k].z = bad_value;
                noise_weights_[i][k] = 0.f;
                continue;
            }

            if(noise_weights_[i][k] < min_weight_)
            {
                input_clouds_used_[i]->points[k].x = input_clouds_used_[i]->points[k].y = input_clouds_used_[i]->points[k].z = bad_value;
                noise_weights_[i][k] = 0.f;
            }
            else
            {
                //adapt weight based on distance
                float capped_dist = std::min(std::max(start_dist, dist), max_distance_); //[start,end]
                float w =  1.f - (capped_dist - start_dist) / (max_distance_ - start_dist);
                noise_weights_[i][k] *=  w;
            }
        }
    }

    float f_ = 525.f;
    float cx_, cy_;
    cx_ = 320;
    cy_ = 240;
    float threshold_ss = 0.003f;
    int width, height;
    width = input_clouds_[0]->width;
    height = input_clouds_[0]->height;

    PointTPtr big_cloud(new pcl::PointCloud<PointT>);
    octree_points_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    weights_points_in_octree_.resize(0);

    for(size_t i=0; i < input_clouds_used_.size(); i++)
    {
        PointTPtr cloud(new pcl::PointCloud<PointT>);
        PointNormalTPtr normal_cloud(new pcl::PointCloud<pcl::Normal>);
        faat_pcl::utils::miscellaneous::transformNormals(input_normals_[i], normal_cloud, transformations_to_global_[i]);
        pcl::transformPointCloud(*input_clouds_used_[i], *cloud, transformations_to_global_[i]);

        /*float sum_curv = 0;
        float sum_curv_orig = 0;
        for(size_t k=0; k < normal_cloud->points.size(); k++)
        {
            sum_curv += normal_cloud->points[k].curvature;
            sum_curv_orig += input_normals_[i]->points[k].curvature;
        }

        std::cout << sum_curv << " " << sum_curv_orig << std::endl;*/

        if (indices_.size() == 0)
        {
            *big_cloud += *cloud;
            weights_points_in_octree_.insert(weights_points_in_octree_.end(), noise_weights_[i].begin(), noise_weights_[i].end());
            *octree_points_normals_ += *normal_cloud;
        }
        else
        {
            pcl::copyPointCloud(*cloud, indices_[i], *cloud);
            *big_cloud += *cloud;

            for (int j=0;j<indices_[i].size();j++)
            {
                weights_points_in_octree_.push_back(noise_weights_[i][indices_[i][j]]);
            }

            PointNormalTPtr normal_cloud_filtered(new pcl::PointCloud<pcl::Normal>);
            pcl::copyPointCloud(*normal_cloud, indices_[i], *normal_cloud_filtered);
            *octree_points_normals_ += *normal_cloud_filtered;

            std::cout << "input size: " << input_clouds_used_[i]->points.size() << std::endl;
            std::cout << "filtered size: " << cloud->points.size() << std::endl;
            std::cout << "bigcloud size: " << big_cloud->points.size() << std::endl;
            std::cout << "noise weights size: " << noise_weights_[i].size() << std::endl;
            std::cout << "weights octree size: " << weights_points_in_octree_.size() << std::endl;
            std::cout << "normal cloud size: " << normal_cloud->points.size() << std::endl;
            std::cout << "normal cloud filtered size: " << normal_cloud_filtered->points.size() << std::endl;
            std::cout << std::endl;
        }
    }

    std::vector<std::pair<int, int> > big_cloud_to_input_clouds;
    big_cloud_to_input_clouds.resize(big_cloud->points.size());
    int idx = 0;
    for(size_t i=0; i < input_clouds_used_.size(); i++)
    {
        if (indices_.size() == 0)
        {
            for(size_t k=0; k < input_clouds_used_[i]->points.size(); k++, idx++)
            {
                big_cloud_to_input_clouds[idx] = std::make_pair((int)i, int(k));
            }
        }
        else
        {
            std::cout << "input point size: " << input_clouds_used_[i]->points.size() << std::endl;
            std::cout << "indices size: " << indices_[i].size() << std::endl;

            for (int k=0;k<indices_[i].size();k++, idx++)
            {
                big_cloud_to_input_clouds[idx] = std::make_pair((int)i, int(indices_[i][k]));
            }
        }
    }

    std::cout << "created indices" << std::endl;

    //std::cout << big_cloud->points.size() << " " << weights_points_in_octree_.size() << std::endl;

    std::vector<bool> indices_big_cloud_keep(big_cloud->points.size(), true);

    for(size_t i=0; i < input_clouds_used_.size(); i++)
    {
        PointTPtr organized_cloud = input_clouds_used_[i];
        Eigen::Matrix4f global_to_cloud = transformations_to_global_[i].inverse ();

        for(size_t k=0; k < big_cloud->points.size(); k++)
        {
            if(!indices_big_cloud_keep[k]) //already rejected
                continue;

            Eigen::Vector4f p = big_cloud->points[k].getVector4fMap();
            p = global_to_cloud * p;
            int u = static_cast<int> (f_ * p[0] / p[2] + cx_);
            int v = static_cast<int> (f_ * p[1] / p[2] + cy_);

            //Not out of bounds
            if ((u >= width) || (v >= height) || (u < 0) || (v < 0))
            {
              continue;
            }

            //Check for invalid depth
            if (!pcl_isfinite (organized_cloud->at (u, v).x) || !pcl_isfinite (organized_cloud->at (u, v).y)
                || !pcl_isfinite (organized_cloud->at (u, v).z))
            {
              continue;
            }

            int idx_org = v * width + u;

            Eigen::Vector3f normal_from_cloud = input_normals_[i]->at (u,v).getNormalVector3fMap();
            Eigen::Vector3f normal_from_octree = octree_points_normals_->points[k].getNormalVector3fMap();
            Eigen::Vector3f normal_octree_trans;
            faat_pcl::utils::miscellaneous::transformNormal(normal_from_octree, normal_octree_trans, global_to_cloud);

            if(normal_octree_trans.dot(normal_from_cloud) < 0)
                continue;

            float z_oc = organized_cloud->at (u, v).z;
            assert(idx_org < noise_weights_[i].size());

            //Check if point depth (distance to camera) is greater than the (u,v)
            if(p[2] < (z_oc - threshold_ss))
            {
                //in front
                if(weights_points_in_octree_[k] < noise_weights_[i][idx_org])
                {
                    //indices_big_cloud_keep[k] = false; FIX THIS
                }
            }
        }

        std::cout << "iteration " << i << " finished..." << std::endl;
    }

    {
        PointTPtr big_cloud_filtered (new pcl::PointCloud<PointT>);
        std::vector<int> indices_kept(big_cloud->points.size(), 0);
        int kept = 0;
        for(size_t k=0; k < big_cloud->points.size(); k++)
        {
            if(indices_big_cloud_keep[k])
            {
                indices_kept[kept] = k;
                kept++;
            }
        }

        indices_kept.resize(kept);
        pcl::copyPointCloud(*big_cloud, indices_kept, *big_cloud_filtered);
        pcl::copyPointCloud(*octree_points_normals_, indices_kept, *octree_points_normals_);

        std::vector<float> new_weights_(indices_kept.size());
        for(size_t k=0; k < indices_kept.size(); k++)
        {
            new_weights_[k] = weights_points_in_octree_[indices_kept[k]];
        }

        weights_points_in_octree_ = new_weights_;
        octree_.reset(new pcl::octree::OctreePointCloudPointVector<PointT>(octree_resolution_));
        octree_->setInputCloud(big_cloud_filtered);
        octree_->addPointsFromInputCloud();
    }

    std::cout << weights_points_in_octree_.size() << std::endl;

    unsigned int leaf_node_counter = 0;
    typename pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator it2;
    const typename pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator it2_end = octree_->leaf_end();

    output->points.resize(weights_points_in_octree_.size());
    output_normals_.reset(new pcl::PointCloud<pcl::Normal>);
    output_normals_->points.resize(weights_points_in_octree_.size());

    int kept = 0;
    for (it2 = octree_->leaf_begin(); it2 != it2_end; ++it2)
    {
        ++leaf_node_counter;
        pcl::octree::OctreeContainerPointIndices& container = it2.getLeafContainer();
        // add points from leaf node to indexVector
        std::vector<int> indexVector;
        container.getPointIndices (indexVector);

        if(indexVector.size() < min_points_per_voxel_)
            continue;

        if(final_resolution_ == -1)
        {
            for(size_t k=0; k < indexVector.size(); k++)
            {
                output->points[kept] = octree_->getInputCloud()->points[indexVector[k]];
                kept++;
            }
            continue;
        }

        PointT p;
        p.getVector3fMap() = Eigen::Vector3f::Zero();
        int r,g,b;
        r = g = b = 0;
        pcl::Normal n;
        n.getNormalVector3fMap() = Eigen::Vector3f::Zero();
        n.curvature = 0.f;

        int used = 0;
        for(size_t k=0; k < indexVector.size(); k++)
        {
            if(weights_points_in_octree_[indexVector[k]] < min_weight_)
                continue;

            p.getVector3fMap() = p.getVector3fMap() +  octree_->getInputCloud()->points[indexVector[k]].getVector3fMap();
            r += octree_->getInputCloud()->points[indexVector[k]].r;
            g += octree_->getInputCloud()->points[indexVector[k]].g;
            b += octree_->getInputCloud()->points[indexVector[k]].b;


            Eigen::Vector3f normal = octree_points_normals_->points[indexVector[k]].getNormalVector3fMap();
            normal.normalize();
            n.getNormalVector3fMap() = n.getNormalVector3fMap() + normal;
            n.curvature += octree_points_normals_->points[indexVector[k]].curvature;
            used++;
        }

        if(used == 0)
            continue;

        //std::cout << "n.curvature" << n.curvature << std::endl;

        p.getVector3fMap() = p.getVector3fMap() / used;
        p.r = r / used;
        p.g = g / used;
        p.b = b / used;
        output->points[kept] = p;

        n.getNormalVector3fMap() = n.getNormalVector3fMap() / used;
        n.getNormalVector3fMap()[3] = 0;
        n.curvature /= used;
        output_normals_->points[kept] = n;
        kept++;
    }

    output->points.resize(kept);
    output_normals_->points.resize(kept);
    output->width = output_normals_->width = kept;
    output->height = output_normals_->height = 1;
    output->is_dense = output_normals_->is_dense = true;

    //finally mask input_clouds_ to know which points were actually used
    //FIX This, was commented out before?
    for(size_t k=0; k < big_cloud->points.size(); k++)
    {
        if(!indices_big_cloud_keep[k])
        {
            int idx_c = big_cloud_to_input_clouds[k].first;
            int idx_p = big_cloud_to_input_clouds[k].second;
            input_clouds_used_[idx_c]->points[idx_p].x =
            input_clouds_used_[idx_c]->points[idx_p].y =
            input_clouds_used_[idx_c]->points[idx_p].z = bad_value;
        }
    }

    /*for(size_t i=1; i < input_clouds_.size(); i++)
    {
        PointTPtr cloud(new pcl::PointCloud<PointT>);
        PointNormalTPtr normal_cloud(new pcl::PointCloud<pcl::Normal>);
        faat_pcl::utils::miscellaneous::transformNormals(input_normals_[i], normal_cloud, transformations_to_global_[i]);

        pcl::transformPointCloud(*input_clouds_[i], *cloud, transformations_to_global_[i]);
        PointTPtr organized_cloud = input_clouds_[i];

        //project each octree point in the new cloud and decide
        //whether to keep the octree point or the other one, if FSV occurs
        unsigned int leaf_node_counter = 0;
        typename pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator it2;
        const typename pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator it2_end = octree_->leaf_end();

        std::vector<int> indices_octree_keep;
        std::vector<bool> indices_new_cloud_keep(organized_cloud->points.size(), true);

        Eigen::Matrix4f global_to_cloud = transformations_to_global_[i].inverse ();
        for (it2 = octree_->leaf_begin(); it2 != it2_end; ++it2)
        {
            ++leaf_node_counter;
            pcl::octree::OctreeContainerPointIndices& container = it2.getLeafContainer();
            // add points from leaf node to indexVector
            std::vector<int> indexVector;
            container.getPointIndices (indexVector);

            for(size_t k=0; k < indexVector.size(); k++)
            {
                Eigen::Vector4f p = octree_->getInputCloud()->points[indexVector[k]].getVector4fMap();
                p = global_to_cloud * p;
                int u = static_cast<int> (f_ * p[0] / p[2] + cx_);
                int v = static_cast<int> (f_ * p[1] / p[2] + cy_);

                //Not out of bounds
                if ((u >= width) || (v >= height) || (u < 0) || (v < 0))
                {
                  indices_octree_keep.push_back(indexVector[k]);
                  continue;
                }

                //Check for invalid depth
                if (!pcl_isfinite (organized_cloud->at (u, v).x) || !pcl_isfinite (organized_cloud->at (u, v).y)
                    || !pcl_isfinite (organized_cloud->at (u, v).z))
                {
                  indices_octree_keep.push_back(indexVector[k]);
                  continue;
                }

                int idx_org = v * width + u;
                //if indices_new_cloud_keep[idx_org] is already false, what happens?
                if(!indices_new_cloud_keep[idx_org])
                {
                    //PCL_WARN("Index was already rejected...\n");
                    //keep octree point and continue
                    indices_octree_keep.push_back(indexVector[k]);
                    continue;
                }

                Eigen::Vector3f normal_from_cloud = input_normals_[i]->at (u,v).getNormalVector3fMap();
                Eigen::Vector3f normal_from_octree = octree_points_normals_->points[indexVector[k]].getNormalVector3fMap();
                Eigen::Vector3f normal_octree_trans;
                faat_pcl::utils::miscellaneous::transformNormal(normal_from_octree, normal_octree_trans, global_to_cloud);
                if(normal_octree_trans.dot(normal_from_cloud) < 0)
                {
                    indices_octree_keep.push_back(indexVector[k]);
                    indices_new_cloud_keep[idx_org] = true;
                    continue;
                }

                float z_oc = organized_cloud->at (u, v).z;
                assert(idx_org < noise_weights_[i].size());

                //Check if point depth (distance to camera) is greater than the (u,v)
                if (std::abs(p[2] - z_oc) < threshold_ss)
                {
                    //same surface, keep both
                    indices_new_cloud_keep[idx_org] = true;
                    indices_octree_keep.push_back(indexVector[k]);
                }
                else if(p[2] < (z_oc - threshold_ss))
                {
                    //in front
                    if(weights_points_in_octree_[indexVector[k]] >= noise_weights_[i][idx_org])
                    {
                        indices_new_cloud_keep[idx_org] = false;
                        indices_octree_keep.push_back(indexVector[k]);
                    }
                    else
                    {
                        indices_new_cloud_keep[idx_org] = true;
                    }
                }
                else
                {
                    //behind, keep both
                    indices_new_cloud_keep[idx_org] = true;
                    indices_octree_keep.push_back(indexVector[k]);
                }
            }
        }

        //rebuild octree using points in octree that we kept + points in new cloud with a true value
        PointTPtr new_octree_cloud(new pcl::PointCloud<PointT>);
        PointNormalTPtr new_octree_normals(new pcl::PointCloud<pcl::Normal>);
        std::vector<float> new_weights_octree(width * height);

        int kept_new = 0;
        std::vector<int> indices(width * height);
        for(size_t kk=0; kk < indices_new_cloud_keep.size(); kk++)
        {
            if(indices_new_cloud_keep[kk])
            {
                indices[kept_new] = kk;
                new_weights_octree[kept_new] = noise_weights_[i][kk];
                kept_new++;
            }
        }

        new_weights_octree.resize(kept_new);
        indices.resize(kept_new);
        pcl::copyPointCloud(*cloud, indices, *new_octree_cloud);
        PointTPtr kept_from_old_octree(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*octree_->getInputCloud(), indices_octree_keep, *kept_from_old_octree);
        *new_octree_cloud += *kept_from_old_octree;

        octree_.reset(new pcl::octree::OctreePointCloudPointVector<PointT>(octree_resolution_));
        octree_->setInputCloud(new_octree_cloud);
        octree_->addPointsFromInputCloud();

        //update weights_points_in_octree_
        new_weights_octree.resize(kept_new + indices_octree_keep.size());
        for(size_t kk=0; kk < indices_octree_keep.size(); kk++)
        {
            new_weights_octree[kept_new + kk] = weights_points_in_octree_[kk];
        }

        weights_points_in_octree_ = new_weights_octree;
        std::cout << weights_points_in_octree_.size() << " " << octree_->getInputCloud()->points.size() << std::endl;
        assert(weights_points_in_octree_.size() == octree_->getInputCloud()->points.size());

        //update octree normals
        pcl::copyPointCloud(*normal_cloud, indices, *new_octree_normals);
        PointNormalTPtr kept_from_old_octree_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*octree_points_normals_, indices_octree_keep, *kept_from_old_octree_normals);
        *new_octree_normals += *kept_from_old_octree_normals;
        octree_points_normals_ = new_octree_normals;

        std::cout << "iteration " << i << " finished..." << std::endl;
    }*/

    /*unsigned int leaf_node_counter = 0;
    typename pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator it2;
    const typename pcl::octree::OctreePointCloudPointVector<PointT>::LeafNodeIterator it2_end = octree_->leaf_end();

    output->points.resize(weights_points_in_octree_.size());

    int kept = 0;
    for (it2 = octree_->leaf_begin(); it2 != it2_end; ++it2)
    {
        ++leaf_node_counter;
        pcl::octree::OctreeContainerPointIndices& container = it2.getLeafContainer();
        // add points from leaf node to indexVector
        std::vector<int> indexVector;
        container.getPointIndices (indexVector);

        if(indexVector.size() < min_points_per_voxel_)
            continue;

        if(final_resolution_ == -1)
        {
            for(size_t k=0; k < indexVector.size(); k++)
            {
                output->points[kept] = octree_->getInputCloud()->points[indexVector[k]];
                kept++;
            }
            continue;
        }

        PointT p;
        p.getVector3fMap() = Eigen::Vector3f::Zero();
        int r,g,b;
        r = g = b = 0;

        int used = 0;
        for(size_t k=0; k < indexVector.size(); k++)
        {
            //if(weights_points_in_octree_[indexVector[k]] < min_weight_)
            //continue;

            p.getVector3fMap() = p.getVector3fMap() +  octree_->getInputCloud()->points[indexVector[k]].getVector3fMap();
            r += octree_->getInputCloud()->points[indexVector[k]].r;
            g += octree_->getInputCloud()->points[indexVector[k]].g;
            b += octree_->getInputCloud()->points[indexVector[k]].b;
            used++;
        }

        if(used == 0)
            continue;

        p.getVector3fMap() = p.getVector3fMap() / used;
        p.r = r / used;
        p.g = g / used;
        p.b = b / used;

        output->points[kept] = p;
        kept++;
    }

    output->points.resize(kept);
    output->width = kept;
    output->height = 1;
    output->is_dense = true;*/
}

template class faat_pcl::utils::NMBasedCloudIntegration<pcl::PointXYZRGB>;
//template class faat_pcl::utils::noise_models::NguyenNoiseModel<pcl::PointXYZ>;
