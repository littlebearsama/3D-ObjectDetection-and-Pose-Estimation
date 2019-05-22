/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
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
 * @file BoundaryRelationsMeanDepth.hpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Class to calculate boundary depth and standart deviation.
 */

#include "BoundaryRelationsMeanDepth.hpp"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

BoundaryRelationsMeanDepth::BoundaryRelationsMeanDepth():
BoundaryRelationsBase()
{
}

BoundaryRelationsMeanDepth::~BoundaryRelationsMeanDepth()
{
}

surface::meanVal BoundaryRelationsMeanDepth::compute()
{
  //@ep: TODO check reconditions
  if(!have_cloud)
  {
    printf("[BoundaryRelationsMeanDepth::compute] Error: No input cloud set.\n");
    exit(0);
  }

  if(!have_boundary)
  {
    printf("[BoundaryRelationsMeanDepth::compute] Error: No input border.\n");
    exit(0);
  }
  
  surface::meanVal meanDepth;
  
  if(boundary.size() <= 0)
  {
    printf("[BoundaryRelationsMeanDepth::compute] Warning: Boundary size is 0. This means that constants are different everywhere!\n");
    meanDepth.mean = 0;
    meanDepth.stddev = 0;
    return meanDepth;
  }

  int boundaryLength = boundary.size();

  // calculate mean depth
  double totalDepth = 0.;
  double totalDepthStdDev = 0.;
  std::vector<double> valuesDepth;
  valuesDepth.reserve(boundaryLength);
  for(unsigned int i=0; i<boundary.size(); i++)
  {
    pcl::PointXYZRGB p1 = cloud->points.at(boundary.at(i).idx1);
    pcl::PointXYZRGB p2 = cloud->points.at(boundary.at(i).idx2);

    if(checkNaN(p1) || checkNaN(p2))
    {
      boundaryLength--;
      continue;
    }
 
    double depth = fabs(p1.z - p2.z);
    valuesDepth.push_back(depth);
    totalDepth += depth;
  }

  // normalize depth sum and calculate depth variance
  //@ep: this shoule be separate function in the utils
  if(boundaryLength > 0)
  {
    totalDepth /= boundaryLength;
    for(unsigned i=0; i<valuesDepth.size(); i++)
    {
      totalDepthStdDev += fabs(valuesDepth.at(i) - totalDepth);
    }
    totalDepthStdDev /= boundaryLength;
  }
  else
  {
    std::printf("[BoundaryRelationsMeanDepth::compute] Warning: Number of valid depth points is zero: totalDepth: %4.3f\n", totalDepth);
    totalDepth = 0.;
    totalDepthStdDev = 0.;
  }

  meanDepth.mean = totalDepth;
  meanDepth.stddev = totalDepthStdDev;

  return meanDepth;
}

} // end surface












