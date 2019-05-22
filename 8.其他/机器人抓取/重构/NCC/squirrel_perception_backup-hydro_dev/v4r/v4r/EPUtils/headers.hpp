/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
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

#ifndef EPUTILS_MODULE_HEADERS_HPP
#define EPUTILS_MODULE_HEADERS_HPP

#include <string>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>
#include <time.h>

#define BOOST_FILESYSTEM_DEPRECATED

#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <eigen3/Eigen/Eigen>

// #ifndef NOT_USE_PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "PCLPreprocessingXYZRC.hpp"
// #endif

namespace EPUtils
{
  
static const int dy8[8] = {-1,-1,-1,0,1,1,1,0};
static const int dx8[8] = {-1,0,1,1,1,0,-1,-1};

static const int dx4[4] = {-1,1,0,0};
static const int dy4[4] = {0,0,-1,1};

} //namespace EPUtils

#endif //EPUTILS_MODULE_HEADERS_HPP
