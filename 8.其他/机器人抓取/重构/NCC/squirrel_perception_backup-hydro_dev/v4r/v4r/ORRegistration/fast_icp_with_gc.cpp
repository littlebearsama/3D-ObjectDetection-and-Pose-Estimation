/*
 * fast_icp_with_gc.cpp
 *
 *  Created on: Sep 8, 2013
 *      Author: aitor
 */

#include <v4r/ORRegistration/fast_icp_with_gc.h>
#include <v4r/ORRegistration/fast_icp_with_gc.hpp>

template class faat_pcl::registration::ICPNode<pcl::PointXYZRGB>;
template class faat_pcl::registration::FastIterativeClosestPointWithGC<pcl::PointXYZRGB>;

