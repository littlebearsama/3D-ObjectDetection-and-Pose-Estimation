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
 * @file PCLFunctions.h
 * @author Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Calculations with PCL.
 */


#ifndef PCLA_PCLFUNCTIONS_H
#define PCLA_PCLFUNCTIONS_H

#include <vector>
#include "PCLCommonHeaders.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

namespace pclA
{
  
/**
 * @brief Remove zero and nan points from point cloud and 
 * @param pcl_cloud Point cloud in PCL format
 * @param indexes Indexes of point cloud
 * @param normals Normals of point clouds
 * @param useVoxelGrid True, if voxel grid should be used.
 * @param vc_size Voxel grid size.
 * @param useZFilter Use a z-filter
 * @param minZ Minium z-filter value
 * @param maxZ Maximum z-filter value
 * @return Returns true for success.
 */ 
bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                          bool useVoxelGrid = false,
                          double vg_size = 0.01,
                          bool useZFilter = false,
                          double minZ = 0.3,
                          double maxZ = 2.3);
                          
bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                          std::vector<int> &indices,
                          bool useVoxelGrid = false,
                          double vg_size = 0.01,
                          bool useZFilter = false,
                          double minZ = 0.3,
                          double maxZ = 2.3);

bool PreProcessPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                          std::vector<int> &indices,
                          pcl::PointCloud<pcl::Normal>::Ptr &normals,
                          bool useVoxelGrid = false,
                          double vg_size = 0.01,
                          bool useZFilter = false,
                          double minZ = 0.3,
                          double maxZ = 2.3);

/**
 * @brief Fit planes into a point cloud, using SAC-segmentation and euclidean clustering.
 * First SAC removes the most supported plane, then the euclidean cluster algorithm
 * splits the rest of the cloud into clusters and calls for each cluster again this
 * function (recursive). Segmented planes are stored in clustered clouds in openCV 
 * matrix style.
 * @param pcl_cloud Point cloud in PCL format.
 * @param pcl_plane_clouds Vector of point clouds with resulting models.
 * @param model_coefficient Model coefficients of the resulting models
 * @param sac_model Type of model to fit.
 * @param sac_optimal_distance Calculate optimal distance threshold for plane fitting.
 * @param sac_optimal_weight_factor Weight factor for optimal threshold processing.
 * @param sac_distance Inlier distance for SAC segmentation.
 * @param sac_max_iterations Maximum iterations for SAC.
 * @param sac_min_inliers Minimum inliers for a plane.
 * @param ec_cluster_tolerance Minimum distance for euclidean clustering.
 * @param ec_min_cluster_size Minimum cluster size.
 * @param ec_max_cluster_size Maximum cluster size.
 * @return Returns true for success.
 */
bool FitModelRecursive(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                       std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                       std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                       int sac_model = pcl::SACMODEL_PLANE,
                       bool sac_optimal_distance = true,
                       double sac_optimal_weight_factor = 1.5,
                       double sac_distance = 0.005,
                       int sac_max_iterations = 300,
                       int sac_min_inliers = 15,
                       double ec_cluster_tolerance = 0.015, 
                       int ec_min_cluster_size = 15,
                       int ec_max_cluster_size = 1000000);
                        
bool FitModelRecursiveWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                        std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                        int sac_model = pcl::SACMODEL_PLANE,
                        bool sac_optimal_distance = true,
                        double sac_optimal_weight_factor = 1.5,
                        double sac_distance = 0.005,
                        int sac_max_iterations = 300,
                        int sac_min_inliers = 15,
                        double ec_cluster_tolerance = 0.015, 
                        int ec_min_cluster_size = 15,
                        int ec_max_cluster_size = 1000000);
    
 /**
 * @brief Write docu
 */                       
bool SingleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                int sac_model = pcl::SACMODEL_PLANE,
                                bool calc_optimal_sac_distance = true,
                                double sac_optimal_weight_factor = 1.5,
                                double sac_distance = 0.005, 
                                int maxIterations = 300,
                                int minInliers = 15);
                                
bool SingleSACModelSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                std::vector<int> &indexes,
                                std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                                std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                int sac_model = pcl::SACMODEL_PLANE,
                                bool calc_optimal_sac_distance = true,
                                double sac_optimal_weight_factor = 1.5,
                                double sac_distance = 0.005, 
                                int maxIterations = 300,
                                int minInliers = 15);
                           
/**
 * @brief See SingleSACModelSegmentationWithNormals().
 * This function uses point normals to segment the point cloud.
 */
bool SingleSACModelSegmentationWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                           std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                           int sac_model = pcl::SACMODEL_PLANE,
                                           bool calc_optimal_sac_distance = true,
                                           double sac_optimal_weight_factor = 1.5,
                                           double sac_distance = 0.005, 
                                           int maxIterations = 300,
                                           int minInliers = 15);
                           
bool SingleSACModelSegmentationWithNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud, 
                                           std::vector<int> &indexes,
                                           pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                           std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_plane_clouds,
                                           std::vector< std::vector<int> > &pcl_model_cloud_indexes,
                                           std::vector< pcl::ModelCoefficients::Ptr > &model_coefficients,
                                           int sac_model = pcl::SACMODEL_PLANE,
                                           bool calc_optimal_sac_distance = true,
                                           double sac_optimal_weight_factor = 1.5,
                                           double sac_distance = 0.005, 
                                           int maxIterations = 300,
                                           int minInliers = 15);
                           

/**
 * @brief Cluster a point cloud with a euclidean threshold.
 * 2nd: Copy also the indices of the point cloud
 * 3rd: Copy also the indices and normals of the point cloud
 * @param pcl_cloud Point cloud in pcl format to cluster.
 * @param indices Indexes of the pcl-cloud
 * @param pcl_clustered_clouds The resulting clustered clouds in pcl-format.
 * @param pcl_clustered_indexes The resulting clustered indices for the clouds
 * @param normals Point cloud with normals
 * @param pcl_cluster_normals The resulting normals
 * @param cluster_tolerance Euclidean threshold for clustering points.
 * @param min_cluster_size Minimum cluster size for a object.
 * @param max_cluster_size Maximum cluster size for a object.
 * @return Returns true for success
 */  
bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         double cluster_tolerance = 0.015, 
                         double min_cluster_size = 10,
                         double max_cluster_size = 1000000);    
                         
bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector<int> &indices,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_cluster_clouds,
                         std::vector< std::vector<int> > &pcl_clustered_indexes,
                         double cluster_tolerance = 0.015, 
                         double min_cluster_size = 10,
                         double max_cluster_size = 1000000);       
                         
bool EuclideanClustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud,
                         std::vector<int> &indices,
                         std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clustered_clouds,
                         std::vector< std::vector<int> > &pcl_clustered_indexes,
                         pcl::PointCloud<pcl::Normal>::Ptr normals,
                         std::vector< pcl::PointCloud<pcl::Normal>::Ptr > &pcl_cluster_normals,
                         double cluster_tolerance = 0.015, 
                         double min_cluster_size = 10,
                         double max_cluster_size = 1000000);                           
                         
/**
 * @brief Calculate the convex hull of a point cloud. First project pcl_cloud to the plane,
 * then calculate the convex hull.
 * @param pcl_cloud The point cloud - Returned projected.
 * @param model_coefficient Model coefficients of the plane.
 * @param pcl_convex_hull Convex hull
 */                        
bool GetConvexHull(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                   pcl::ModelCoefficients::Ptr model_coefficient,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_convex_hull);
 
/**
 * @brief Calculate the convex hull of a point cloud. First project pcl_clouds to the planes,
 * then calculate the convex hull.
 * @param pcl_clouds The point clouds - Returned projected!
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_convex_hulls Convex hulls
 */                        
bool GetConvexHulls(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                    std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                    std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_convex_hulls);

                  
double GetConvexHullArea(std::vector<cv::Vec3f> &pcl_convex_hull);

/**
 * @brief Project a point cloud to the model (SAC plane).
 * @param pcl_clouds Point clouds in pcl format.
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_clouds_projected Projected point clouds
 */    
bool GetProjectedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud, 
                        pcl::ModelCoefficients::Ptr model_coefficients,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud_projected);
                           
/**
 * @brief Project point clouds to models (SAC plane).
 * @param pcl_clouds Point clouds in pcl format.
 * @param model_coefficients Model coefficients of the planes.
 * @param pcl_clouds_projected Projected point clouds
 */                            
bool GetProjectedPoints(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds, 
                        std::vector< pcl::ModelCoefficients::Ptr > model_coefficients,
                        std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &pcl_clouds_projected);                         
                         
                         
/**
 * @brief Get the maximum distance from one point of a point cloud to a given plane.
 * @param pcl_cloud Point cloud
 * @param model_coefficients Model coefficients of the plane
 * @param max_distance Maximum distance of one of the points
 */
void MaxDistanceToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                        const pcl::ModelCoefficients::Ptr &model_coefficients,
                        double &max_distance);
                        
                        
/**
 * @brief Calculates the square error of the point cloud to the plane model
 * @param pcl_cloud Point cloud
 * @param model_coefficients Model coefficients of the plane
 * @param square_error The sum of the squared error values
 * @param distances Distance of each point to the plane
 */
void CalcSquareErrorToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                            const pcl::ModelCoefficients::Ptr &model_coefficients,
                            double &square_error);

void CalcSquareErrorsToPlane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
                             const pcl::ModelCoefficients::Ptr &model_coefficients,
                             double square_error,
                             std::vector<double> &distances);
                             
/**
 * @brief Create for a given cloud with model_coefficients of a plane and a reference
 * distance a SOI.
 * @param pcl_cloud Point cloud
 * @param model_coefficients Model coefficients of the plane
 * @param distance Distance between plane and point on top of SOI.
 * @param soi Space of interest hull points (convex polygon).
 */                       
void CreateSOI(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcl_cloud,
               const pcl::ModelCoefficients::Ptr &model_coefficients,
               const double distance, 
               pcl::PointCloud<pcl::PointXYZRGB>::Ptr &soi);
 

/**
 * @brief Convert point cloud to normal map. Only for dense point clouds with correct header.
 * NOTE: All NANs will be replaced by zero-points!!!
 * @param cloud Point cloud to be transformed.
 * @param normals Normal map as openCV matrix with 4f-vectors (4th element is curvature)
 */
void NormalsFromPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                         cv::Mat_<cv::Vec4f> &normals, 
                         float radius_search = 0.03);
void NormalsFromSortedPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr &normals,
                               float max_depth_change = 0.02, 
                               float normal_smoothing_size = 5.0);
void NormalsFromSortedPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                               cv::Mat_<cv::Vec4f> &normals,
                               float max_depth_change = 0.02, 
                               float normal_smoothing_size = 5.0);
void NormalsFromSortedPCLCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
                               pcl::PointCloud<pcl::Normal>::Ptr &pcl_normals,
                               cv::Mat_<cv::Vec4f> &cv_normals,
                               float max_depth_change = 0.02, 
                               float normal_smoothing_size = 5.0);
                             
               
               
}

#endif
