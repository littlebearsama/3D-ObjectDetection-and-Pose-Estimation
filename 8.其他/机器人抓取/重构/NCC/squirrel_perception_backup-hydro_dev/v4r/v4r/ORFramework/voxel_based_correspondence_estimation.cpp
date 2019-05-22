/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include "voxel_based_correspondence_estimation.hpp"

template class faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ >;
template class faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZRGB, pcl::PointXYZRGB >;
template class faat_pcl::rec_3d_framework::VoxelBasedCorrespondenceEstimation<pcl::PointXYZRGBA, pcl::PointXYZRGBA >;
