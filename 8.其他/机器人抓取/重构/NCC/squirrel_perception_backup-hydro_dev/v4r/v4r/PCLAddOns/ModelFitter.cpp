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
 * @file ModelFitter.h
 * @author Richtsfeld
 * @date October 2011
 * @version 0.1
 * @brief Extract shape models (plane, cylinder, sphere, ...) from kinect data.
 */

#include "ModelFitter.h"
#include "v4r/PCLAddOns/PCLUtils.h"
#include "v4r/PCLAddOns/PCLFunctions.h"

namespace pclA
{


/************************************************************************************
 * Constructor/Destructor
 */

ModelFitter::ModelFitter(Parameter _param) : param(_param)
{
  have_input_cloud = false;
  have_normals = false;
}

ModelFitter::~ModelFitter()
{
}

// ================================= Private functions ================================= //

void ModelFitter::SetIndices(int cloud_size)
{
  indices.resize(cloud_size);
  for(int i=0; i<cloud_size; i++)
    indices[i] = i;
}

bool ModelFitter::FitMultipleModelRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                               std::vector<int> &indices,
                               pcl::PointCloud<pcl::Normal>::Ptr normals,
                               std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_model_clouds,
                               std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                               std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                               std::vector<int> &sac_models,
                               std::vector<int> &result_models)
{
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
  std::vector< pcl::PointCloud<pcl::Normal>::Ptr > pcl_clustered_normals;
  std::vector< std::vector<int> > pcl_clustered_indices;

  if(pcl_cloud->points.size() != indices.size()) {
    printf("[PCLFunctions::FitMultipleModelRecursive] Warning: Cloud and index size differs: %lu-%lu.\n", 
           pcl_cloud->points.size(), indices.size());
    return false;
  }
  
  if(!have_normals)
  {
    normals.reset(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setInputCloud(pcl_cloud);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.03);
    ne.compute(*normals);
  }

  if (MultipleSACModelSegmentation(pcl_cloud, indices, normals, pcl_model_clouds, pcl_model_cloud_indexes, 
                                   model_coefficients, sac_models, result_models))
  {
    if (EuclideanClustering(pcl_cloud, indices, pcl_clustered_clouds, pcl_clustered_indices, normals, pcl_clustered_normals,
                            param.ec_cluster_tolerance, param.ec_min_cluster_size, param.ec_max_cluster_size))
    {
      for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++) {
          FitMultipleModelRecursive(pcl_clustered_clouds[i], pcl_clustered_indices[i], pcl_clustered_normals[i], pcl_model_clouds, 
                                    pcl_model_cloud_indexes, model_coefficients, sac_models, result_models);
      }
    }
  }
  return true;
}


bool ModelFitter::MultipleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                               std::vector<int> &indices,
                                               pcl::PointCloud<pcl::Normal>::Ptr normals,
                                               std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_model_clouds,
                                               std::vector< std::vector<int> > &pcl_model_cloud_indices,
                                               std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                               std::vector<int> &sac_models,
                                               std::vector<int> &result_models)
{
  bool optimize = false;              /// TODO Set as parameter
  bool succeed = false;
  int tmp_inliers = 0;

  if(pcl_cloud->points.size() != indices.size()) {
    printf("[PCLFunctions::MultipleSACModelSegmentation] Warning: Cloud and index size differs.\n");
    return false;
  }

  double sac_opt_distance = param.sac_distance;
  if (param.sac_calc_optimal_distance)
     sac_opt_distance = pclA::CalculateOptimalSACDistanceKinect(pcl_cloud, 
                                                                param.sac_distance, 
                                                                param.sac_optimal_weight_factor);

  /// TODO Calculate now the support for the given models!!!
  unsigned max_inliers = 0;
  unsigned necessaryInliers = 0;           /// TODO
  int max_inlier_model = 0;
  pcl::PointIndices::Ptr result_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr result_model_coef (new pcl::ModelCoefficients);

  for (unsigned i=0; i< sac_models.size(); i++)
  {
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr model_coef (new pcl::ModelCoefficients);

    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    
    seg.setModelType(sac_models[i]);
    if(sac_models[i] == pcl::SACMODEL_CYLINDER)
    {
      tmp_inliers = param.sac_min_inliers/**2*/;
      seg.setOptimizeCoefficients (optimize);
      seg.setNormalDistanceWeight (0.005);   // 0.05-0.1 ???
      seg.setRadiusLimits (0, 0.20);  // 20cm radius maximum
      seg.setDistanceThreshold(0.008/*sac_opt_distance*1.5*/);       /// TODO Werte gut einstellen
      seg.setMaxIterations(10000/*param.sac_max_iterations*3*/);
    }
    else if(sac_models[i] == pcl::SACMODEL_SPHERE)
    {
      tmp_inliers = param.sac_min_inliers;
      seg.setOptimizeCoefficients (optimize);
      seg.setNormalDistanceWeight (0.005);
      seg.setRadiusLimits (0, 0.15);
      seg.setDistanceThreshold(sac_opt_distance/3.);                 /// TODO Werte gut einstellen
      seg.setMaxIterations(10000/*param.sac_max_iterations*/);
    } 
    else if(sac_models[i] == pcl::SACMODEL_NORMAL_PLANE)
    {
      tmp_inliers = param.sac_min_inliers;
      seg.setOptimizeCoefficients (optimize);
      seg.setNormalDistanceWeight (0.005);                           /// TODO Find good parameter 0.003?
      seg.setDistanceThreshold(sac_opt_distance);
//       seg.setEpsAngle(M_PI);
      seg.setMaxIterations(param.sac_max_iterations);
    } 
    else if(sac_models[i] == pcl::SACMODEL_PLANE)// SACMODEL_PLANE
    {
      tmp_inliers = param.sac_min_inliers;
//      seg.setOptimizeCoefficients (optimize);
      seg.setNormalDistanceWeight (0.6);                                /// TODO Find good parameter? Ganz egal hier?
      seg.setDistanceThreshold(sac_opt_distance);
      seg.setMaxIterations(param.sac_max_iterations);
    }
    else printf("[ModelFitter::MultipleSACModelSegmentation] Warning: Unsupport model type!\n");
    
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setInputCloud(pcl_cloud);
    seg.setInputNormals(normals);
    seg.segment(*inliers, *model_coef);

    if (inliers->indices.size() >= max_inliers)
    {
      max_inliers = inliers->indices.size();
      max_inlier_model = sac_models[i];
      result_inliers = inliers;
      result_model_coef = model_coef;
      necessaryInliers = tmp_inliers;
    }
  }

  if (max_inliers >= necessaryInliers)
  {
    // do euclidean clustering and check if plane is one connected component
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_cluster_clouds;
    std::vector< std::vector<int> > pcl_cluster_indices;
    pcl::copyPointCloud(*pcl_cloud, *result_inliers, *model_cloud);
    pclA::EuclideanClustering(model_cloud, result_inliers->indices, pcl_cluster_clouds, pcl_cluster_indices,
                              param.ec_cluster_tolerance,
                              /*param.ec_min_cluster_size*/1,
                              param.ec_max_cluster_size);
    
    if(pcl_cluster_clouds.size() > 1)
    {
// printf("######################## More than one cluster: %u\n", pcl_cluster_clouds.size());
//       for(unsigned i=0; i<pcl_cluster_clouds.size(); i++)
//       {
//         printf("Clustering: nr of points: %u\n", pcl_cluster_clouds[i]->points.size());
//         printf("Clustering: nr of indices: %u\n", pcl_cluster_indices[i].size()); //pcl_cluster_clouds[i]->points.size());
//       }
      
      if(pcl_cluster_clouds[0]->points.size() < necessaryInliers) {
        printf("[ModelFitter::MultipleSACModelSegmentation] Warning Not enough inliers found. Return.\n");
        return false;
      }

      result_inliers->indices = pcl_cluster_indices[0];
      pcl_model_clouds.resize(pcl_model_clouds.size() + 1);
      pcl_model_clouds[pcl_model_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::copyPointCloud(*pcl_cloud, *result_inliers, *pcl_model_clouds[pcl_model_clouds.size()-1]);
    }
    else 
    {
      pcl_model_clouds.resize(pcl_model_clouds.size() + 1);
      pcl_model_clouds[pcl_model_clouds.size()-1].reset(new pcl::PointCloud<pcl::PointXYZRGB>);
//       pcl::copyPointCloud(*pcl_cloud, *result_inliers, *pcl_model_clouds[pcl_model_clouds.size()-1]);
      pcl_model_clouds[pcl_model_clouds.size()-1] = model_cloud;
    }
    
    result_models.push_back(max_inlier_model);
    model_coefficients.push_back(result_model_coef);
    
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
    extract_inliers.setNegative(true);
    extract_inliers.setInputCloud(pcl_cloud);
    extract_inliers.setIndices(result_inliers);
    extract_inliers.filter(*pcl_cloud);

    pcl::ExtractIndices<pcl::Normal> extract_normals;
    extract_normals.setNegative(true);
    extract_normals.setInputCloud(normals);
    extract_normals.setIndices(result_inliers);
    extract_normals.filter (*normals);

    std::vector<int> model_cloud_indices;
    for(unsigned idx=0; idx<result_inliers->indices.size(); idx++)
      model_cloud_indices.push_back(indices[result_inliers->indices[idx]]);
    pcl_model_cloud_indices.push_back(model_cloud_indices);

    std::vector<int>::iterator it;
    for(int idx=result_inliers->indices.size()-1; idx>=0; idx--)
    {
      it = indices.begin() + result_inliers->indices[idx];
      indices.erase(it);
    }
    succeed = true;
  }
  if (pcl_cloud->points.size() > param.sac_min_inliers && succeed) return true;
  return false;
}


// ================================= Public functions ================================= //

void ModelFitter::addModelType(int model_type)
{
  if(model_type == pcl::SACMODEL_PLANE ||
     model_type == pcl::SACMODEL_NORMAL_PLANE ||
     model_type == pcl::SACMODEL_SPHERE ||
     model_type == pcl::SACMODEL_CYLINDER)
    sac_models.push_back(model_type);
  else
    printf("[ModelFitter::addModelType: Warning: Unsupported model type.\n");
}

void ModelFitter::setNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  have_normals = false;
  if(_normals.get() == 0) {
    printf("[ModelFitter::setNormals: Warning: Invalid normal vector.\n");
    return;
  }
  if(_normals->points.size() == 0) {
    printf("[ModelFitter::setNormals: Warning: Empty normal vector.\n");
    return;
  }
  normals = _normals;
  have_normals = true;
}

void ModelFitter::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud)
{
  if(pcl_cloud.get() == 0 || pcl_cloud->points.size() == 0) {
    printf("[ModelFitter::setInputCloud: Warning: Empty or invalid pcl-point-cloud.\n");
    return;
  } 
  input_cloud = pcl_cloud;
  SetIndices(input_cloud->points.size());
  have_input_cloud = true;
  
  // reset the result vectors
  pcl_model_types.resize(0);
  pcl_model_clouds.resize(0);
  pcl_model_cloud_indices.resize(0);
  model_coefficients.resize(0);
}

void ModelFitter::useDominantPlane(bool _use_dp)
{
  use_dp = _use_dp;
}

void ModelFitter::compute()
{
  if(!have_input_cloud) {
    printf("[ModelFitter::compute] Error: No input cloud is given.\n");
    return;
  }
  if(sac_models.size() == 0) {
    printf("[ModelFitter::compute] Warning: No input model given. Add SACMODEL_PLANE.\n");
    sac_models.push_back(pcl::SACMODEL_PLANE);
  }
  if(have_normals)
    if(normals->points.size() != input_cloud->points.size()) {
      printf("[ModelFitter::compute] Warning: Different size of point cloud and normals vector.\n");
      have_normals = false;
    }
  
  if(!use_dp)
  {
    if(have_normals) 
      pclA::PreProcessPointCloud(input_cloud, indices, normals, param.use_voxel_grid, param.voxel_grid_size,
                                 param.do_z_filtering, param.minZ, param.maxZ);
    else
    {
      if(input_cloud->height == 1)
        pclA::PreProcessPointCloud(input_cloud, indices, param.use_voxel_grid, param.voxel_grid_size, 
                                   param.do_z_filtering, param.minZ, param.maxZ);
      else  // compute normals, if ordered point cloud
      {
        printf("[ModelFitter::compute] Warning: Compute normals for orderd cloud!\n");
        pclA::NormalsFromSortedPCLCloud(input_cloud, normals, 0.02, 5.);
        have_normals = true;
        pclA::PreProcessPointCloud(input_cloud, indices, normals, param.use_voxel_grid, param.voxel_grid_size,
                                 param.do_z_filtering, param.minZ, param.maxZ);
      }
    }
    
    FitMultipleModelRecursive(input_cloud, 
                              indices,
                              normals,
                              pcl_model_clouds, 
                              pcl_model_cloud_indices,
                              model_coefficients, 
                              sac_models, 
                              pcl_model_types);
  }
  else
  {
    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_clustered_clouds;
    std::vector< std::vector<int> > pcl_clustered_indexes;

    if(!have_normals) 
    {
      printf("[ModelFitter::compute] Warning: No normals available.\n");
      if(input_cloud->height == 1)
        pclA::PreProcessPointCloud(input_cloud, indices, param.use_voxel_grid, param.voxel_grid_size, 
                                   param.do_z_filtering, param.minZ, param.maxZ);
      else  // compute normals, if ordered point cloud
      {
        printf("[ModelFitter::compute] Warning: Compute normals for orderd cloud!\n");
        pclA::NormalsFromSortedPCLCloud(input_cloud, normals, 0.02, 5.);
        have_normals = true;
        pclA::PreProcessPointCloud(input_cloud, indices, normals, param.use_voxel_grid, param.voxel_grid_size,
                                 param.do_z_filtering, param.minZ, param.maxZ);
      }

      if(pclA::SingleSACModelSegmentation(input_cloud,
                                          indices,
                                          pcl_model_clouds, 
                                          pcl_model_cloud_indices,
                                          model_coefficients, 
                                          pcl::SACMODEL_PLANE,//sac_models[0], 
                                          param.sac_calc_optimal_distance, 
                                          param.sac_optimal_weight_factor, 
                                          param.sac_distance, 
                                          param.sac_max_iterations, 
                                          param.sac_min_inliers))
        pcl_model_types.push_back(/*sac_models[0]*/pcl::SACMODEL_PLANE);
      
      if (pclA::EuclideanClustering(input_cloud, indices, pcl_clustered_clouds, pcl_clustered_indexes, 
                                    param.ec_cluster_tolerance, param.ec_min_cluster_size, param.ec_max_cluster_size))
      {
        for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++) 
        {
          FitMultipleModelRecursive(pcl_clustered_clouds[i], 
                                    pcl_clustered_indexes[i],
                                    normals,
                                    pcl_model_clouds, 
                                    pcl_model_cloud_indices,
                                    model_coefficients, 
                                    sac_models, 
                                    pcl_model_types);
        }
      }
    }
    
    else // have normals
    {
      std::vector< pcl::PointCloud<pcl::Normal>::Ptr > pcl_clustered_normals;
      pclA::PreProcessPointCloud(input_cloud, indices, normals, param.use_voxel_grid, param.voxel_grid_size, 
                                param.do_z_filtering, param.minZ, param.maxZ);
      if(pclA::SingleSACModelSegmentationWithNormals(input_cloud,
                                                     indices,
                                                     normals,
                                                     pcl_model_clouds,
                                                     pcl_model_cloud_indices,
                                                     model_coefficients, 
                                                     pcl::SACMODEL_PLANE,//sac_models[0], 
                                                     param.sac_calc_optimal_distance, 
                                                     param.sac_optimal_weight_factor, 
                                                     param.sac_distance, 
                                                     param.sac_max_iterations, 
                                                     param.sac_min_inliers))
        pcl_model_types.push_back(/*sac_models[0]*/pcl::SACMODEL_PLANE);

      if (pclA::EuclideanClustering(input_cloud, indices, pcl_clustered_clouds, pcl_clustered_indexes, normals, pcl_clustered_normals,
                                    param.ec_cluster_tolerance, param.ec_min_cluster_size, param.ec_max_cluster_size))
      {
        for (unsigned i = 0; i < pcl_clustered_clouds.size(); i++)
        {
          FitMultipleModelRecursive(pcl_clustered_clouds[i], 
                                    pcl_clustered_indexes[i],
                                    pcl_clustered_normals[i],
                                    pcl_model_clouds, 
                                    pcl_model_cloud_indices,
                                    model_coefficients, 
                                    sac_models, 
                                    pcl_model_types);
        }
      }
    }   
  }

  have_normals = false;
  have_input_cloud = false;
}

void ModelFitter::getResults(std::vector<int> &_pcl_model_types,
                             std::vector<pcl::ModelCoefficients::Ptr> &_model_coefficients,
                             std::vector<pcl::PointIndices::Ptr> &_pcl_model_cloud_indices)
{
// printf("ModelFitter::getResults: nr of planes: %u\n", pcl_model_cloud_indices.size());
  _pcl_model_types = pcl_model_types;
  _model_coefficients = model_coefficients;
  
  _pcl_model_cloud_indices.resize(0);
  for(unsigned i=0; i<pcl_model_cloud_indices.size(); i++)
  {
    pcl::PointIndices::Ptr idxs (new pcl::PointIndices);
    idxs->indices = pcl_model_cloud_indices[i];
    _pcl_model_cloud_indices.push_back(idxs);
  }
}

void ModelFitter::getError(std::vector< std::vector<double> > &_pcl_model_error,
                           std::vector<double> &_square_error)
{
  _pcl_model_error.clear();
  _square_error.clear();
  for(unsigned i=0; i<pcl_model_clouds.size(); i++)
  {
    double square_error = 0;
    std::vector<double> distances;
    if(pcl_model_types[i] == pcl::SACMODEL_PLANE || pcl_model_types[i] == pcl::SACMODEL_NORMAL_PLANE)
    {
      pclA::CalcSquareErrorsToPlane(pcl_model_clouds[i], model_coefficients[i], square_error, distances);
      _square_error.push_back(square_error);
      _pcl_model_error.push_back(distances);
    }
    else
    {
      printf("[ModelFitter::getError] Warning: Unsupported model type.\n");
      _square_error.push_back(square_error);
      _pcl_model_error.push_back(distances);      
    }
  }
}
      
void ModelFitter::getClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_pcl_model_clouds)
{
  _pcl_model_clouds = pcl_model_clouds;
}


void ModelFitter::GetResults(std::vector<int> &_pcl_model_types,
                             std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &_pcl_model_clouds,
                             std::vector< std::vector<int> > &_pcl_model_cloud_indices,
                             std::vector< pcl::ModelCoefficients::Ptr > &_model_coefficients)
{
  _pcl_model_types = pcl_model_types;
  _pcl_model_clouds = pcl_model_clouds;
  _pcl_model_cloud_indices = pcl_model_cloud_indices;
  _model_coefficients = model_coefficients;
}

}












