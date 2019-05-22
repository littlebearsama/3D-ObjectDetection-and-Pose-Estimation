/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include "global_nn_classifier.hpp"
#include "metrics.h"

//Instantiation
template class faat_pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::VFHSignature308>;
template class faat_pcl::rec_3d_framework::GlobalNNPipeline<faat_pcl::Metrics::HistIntersectionUnionDistance, pcl::PointXYZ, pcl::VFHSignature308>;
template class faat_pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640>;

template class faat_pcl::rec_3d_framework::GlobalClassifier<pcl::PointXYZ>;
