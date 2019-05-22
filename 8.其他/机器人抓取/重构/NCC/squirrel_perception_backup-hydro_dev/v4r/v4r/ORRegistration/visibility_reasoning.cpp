/*
 * visibility_reasoning.cpp
 *
 *  Created on: Mar 19, 2013
 *      Author: aitor
 */

#include "pcl/point_types.h"
//#include <faat_pcl/registration/visibility_reasoning.h>
#include <v4r/ORRegistration/visibility_reasoning.hpp>

template class faat_pcl::registration::VisibilityReasoning<pcl::PointXYZ>;
template class faat_pcl::registration::VisibilityReasoning<pcl::PointXYZRGB>;
template class faat_pcl::registration::VisibilityReasoning<pcl::PointNormal>;
template class faat_pcl::registration::VisibilityReasoning<pcl::PointXYZRGBNormal>;
