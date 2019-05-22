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
 * @file BoundaryRelationsMeanColor.hpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Class to calculate boundary color and standart deviation.
 */

#include "BoundaryRelationsMeanColor.hpp"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

BoundaryRelationsMeanColor::BoundaryRelationsMeanColor():
BoundaryRelationsBase()
{
}

BoundaryRelationsMeanColor::~BoundaryRelationsMeanColor()
{
}

//@ep: this should be done smarter, we can't infinitely rewrite the same functions
void BoundaryRelationsMeanColor::getYUV(double &Y, double &U, double &V, pcl::PointXYZRGB p)
{
  Y =  (0.257 * p.b) + (0.504 * p.g) + (0.098 * p.r) + 16;;
  U = -(0.148 * p.b) - (0.291 * p.g) + (0.439 * p.r) + 128;    // use bgr
  V =  (0.439 * p.b) - (0.368 * p.g) - (0.071 * p.r) + 128;
}

surface::meanVal BoundaryRelationsMeanColor::compute()
{
  //@ep: TODO check preconditions -- not all of them are necessary
  if(!have_cloud)
  {
    printf("[BoundaryRelationsMeanColor::compute] Error: No input cloud set.\n");
    exit(0);
  }

  if(!have_boundary)
  {
    printf("[BoundaryRelationsMeanColor::compute] Error: No input border.\n");
    exit(0);
  }
  
  surface::meanVal meanColorDistance;
  
  if(boundary.size() <= 0)
  {
    printf("[BoundaryRelationsMeanColor::compute] Warning: Boundary size is 0. This means that constants are different everywhere!\n");
    meanColorDistance.mean = 0;
    meanColorDistance.stddev = 0;
    return meanColorDistance;
  }

  int boundaryLength = boundary.size();

  // calculate mean depth
  double totalColorDistance = 0.;
  double totalColorDistanceStdDev = 0.;
  std::vector<double> valuesColorDistance;
  valuesColorDistance.reserve(boundaryLength);
  for(unsigned int i=0; i<boundary.size(); i++)
  {
    double p0Y, p0U, p0V, p1Y, p1U, p1V;
    
    getYUV(p0Y,p0U,p0V,cloud->points.at(boundary.at(i).idx1));
    getYUV(p1Y,p1U,p1V,cloud->points.at(boundary.at(i).idx2));
    
    double u1 = p0U/255 - p1U/255;
    double u2 = u1 * u1;
    double v1 = p0V/255 - p1V/255;
    double v2 = v1 * v1;
    double cDist = sqrt(u2 + v2);
    totalColorDistance += cDist;
    
    valuesColorDistance.push_back(totalColorDistance);
  }

  // normalize curvature sum and calculate curvature variance
  //@ep: this shoule be separate function in the utils
  if(boundaryLength > 0)
  {
    totalColorDistance /= boundaryLength;
    for(unsigned i=0; i<valuesColorDistance.size(); i++)
    {
      //@ep: BUG why is it standart deviation???
      totalColorDistanceStdDev += fabs(valuesColorDistance.at(i) - totalColorDistance);
    }
    totalColorDistanceStdDev /= boundaryLength;
  }
  else
  {
    std::printf("[BoundaryRelationsMeanCurvature::compute] Warning: Number of valid points is zero: totalColorDistance: %4.3f\n", totalColorDistance);
    totalColorDistance = 0.;
    totalColorDistanceStdDev = 0.;
  }

  meanColorDistance.mean = totalColorDistance;
  meanColorDistance.stddev = totalColorDistanceStdDev;

  return meanColorDistance;
}

} // end surface












