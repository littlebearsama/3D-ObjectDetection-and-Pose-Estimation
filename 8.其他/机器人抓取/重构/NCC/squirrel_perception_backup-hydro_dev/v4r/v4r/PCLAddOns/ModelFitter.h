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
 * @brief Extract shape models (plane, cylinder, sphere) from kinect data.
 */

#ifndef PCLA_MODEL_FITTER_HH
#define PCLA_MODEL_FITTER_HH

#include <vector>
#include <opencv2/core/core.hpp>

#include "PCLCommonHeaders.h"
#include "CCLabeling.hh"

namespace pclA
{

class ModelFitter
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
  class Parameter
  {
  public:
    bool use_voxel_grid;                // vg: use the voxel grid before fitting
    double voxel_grid_size;             // vg: voxel grid size
    bool do_z_filtering;                // z: Do filtering of input cloud in z-direction
    double minZ;                        // z: minimum z-value
    double maxZ;                        // z: maximum z-value
    bool sac_calc_optimal_distance;          // sac: use optimal distance
    double sac_optimal_weight_factor;   // sac: optimal weight factor
    double sac_distance;                // sac: distance
    int sac_max_iterations;             // sac: maximum iterations
    unsigned sac_min_inliers;           // sac: minimum point inliers
    double ec_cluster_tolerance;        // ec: cluster tolerance
    int ec_min_cluster_size;            // ec: minimal cluster size
    int ec_max_cluster_size;            // ec: maximum cluster size

    Parameter(bool _use_voxel_grid = false,
              double _voxel_grid_size = 0.005,           // 0.005 - 0.01
              bool _do_z_filtering = false,
              double _minZ = 0.3,
              double _maxZ = 1.5,
              bool _sac_optimal_distance = true,
              double _sac_optimal_weight_factor = 2.0,
              double _sac_distance = 0.004,
              int _sac_max_iterations = 250,
              unsigned _sac_min_inliers = 25,
              double _ec_cluster_tolerance = 0.015, //0.008,
              int _ec_min_cluster_size = 25,
              int _ec_max_cluster_size = 1000000)
      : use_voxel_grid(_use_voxel_grid), voxel_grid_size(_voxel_grid_size), do_z_filtering(_do_z_filtering),
        minZ(_minZ), maxZ(_maxZ), sac_calc_optimal_distance(_sac_optimal_distance),
        sac_optimal_weight_factor(_sac_optimal_weight_factor), sac_distance(_sac_distance),
        sac_max_iterations(_sac_max_iterations), sac_min_inliers(_sac_min_inliers), ec_cluster_tolerance(_ec_cluster_tolerance), 
        ec_min_cluster_size(_ec_min_cluster_size),  ec_max_cluster_size(_ec_max_cluster_size) {}
  };

  
protected:
  Parameter param;

private:
  bool have_input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud;
  
  bool have_normals;
  pcl::PointCloud<pcl::Normal>::Ptr normals;

  bool use_dp;                          // fit dominant plane before model-fitting
  std::vector<int> sac_models;          // sac models to fit
  std::vector<int> indices;             // index of the pcl_cloud                               /// TODO Change to pcl::ModelIndices

  // Results: Type of models, model point cloud and model coefficients
  std::vector<int> pcl_model_types;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > pcl_model_clouds;
  std::vector< std::vector<int> > pcl_model_cloud_indices;
  std::vector< pcl::ModelCoefficients::Ptr > model_coefficients;
  
  void SetIndices(int cloud_size);
  
  bool FitMultipleModelRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                 std::vector<int> &indices,
                                 pcl::PointCloud<pcl::Normal>::Ptr normals,
                                 std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_model_clouds,
                                 std::vector< std::vector<int> > &pcl_model_cloud_indices,
                                 std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                 std::vector<int> &sac_models,
                                 std::vector<int> &result_models);
                               
  bool MultipleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                                    std::vector<int> &indices,
                                    pcl::PointCloud<pcl::Normal>::Ptr normals,
                                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_model_clouds,
                                    std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                                    std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                    std::vector<int> &sac_models,
                                    std::vector<int> &result_models);

public:
  ModelFitter(Parameter _param = Parameter());
  ~ModelFitter();
  
  /** Add (pcl) model for fitting **/
  void addModelType(int model_type);
  
  /** Set normal point cloud **/
  void setNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals);
  
  /** Set input point cloud **/
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud);

  /** Exract dominant plane, before starting model-fitting **/
  void useDominantPlane(bool _use_dp);
  
  /** Compute point cloud and fit (multiple) models **/
  void compute();
  
  /** Get the results from the processing **/
  void getResults(std::vector<int> &_pcl_model_types,
                  std::vector<pcl::ModelCoefficients::Ptr> &_model_coefficients,
                  std::vector<pcl::PointIndices::Ptr> &_pcl_model_cloud_indices);
       
  /** Get error of each point (L2-norm of point to closest point on surface) and square-error */
  void getError(std::vector< std::vector<double> > &_pcl_model_error,
                std::vector<double> &_square_error);
  
  /** Get a copy of the point clouds related to models **/
  void getClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_pcl_model_clouds);

  /** Get the results from the processing **/
  void GetResults(std::vector<int> &_pcl_model_types,
                  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &_pcl_model_clouds,
                  std::vector< std::vector<int> > &_pcl_model_cloud_indices,
                  std::vector<pcl::ModelCoefficients::Ptr> &_model_coefficients);

};

/*************************** INLINE METHODES **************************/

} //--END--

#endif

