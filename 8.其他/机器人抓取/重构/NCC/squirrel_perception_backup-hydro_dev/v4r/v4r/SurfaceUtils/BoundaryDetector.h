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
 * @file BoundaryDetector.h
 * @author Andreas Richtsfeld, Ekaterina Potapova
 * @date January 2013
 * @version 0.2
 * @brief Estimate all boundary structures of surface models and views.
 */

#ifndef SURFACE_BOUNDARY_DETECTOR_H
#define SURFACE_BOUNDARY_DETECTOR_H

#include <vector>
#include <stdio.h>
#include <stdlib.h>

#include <opencv2/opencv.hpp>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"

#include "ContourDetector.h"

namespace surface
{
 
  
class BoundaryDetector : public ContourDetector
{
public:
  
protected:

private:

  std::vector<aEdgel> bd_edgels;                         ///< Edgels
  std::vector<aEdge> bd_edges;                           ///< Edge between corners

  std::vector<Edgel> edgels;                            ///< Boundary graph: Edgels
  std::vector<Corner> corners;                          ///< Boundary graph: Corners
  std::vector<Edge> edges;                              ///< Boundary graph: Edges

  /** compute edges and corners **/
  void recursiveClustering(int x, int y, int id_0, int id_1,
                           bool horizontal, aEdge &_ec);

  /** compute edges **/
  void computeEdges();
  
  /** Copy structures to view **/
  void copyEdges();
  
public:
  BoundaryDetector();
  ~BoundaryDetector();

  inline std::vector<Edgel> getEdgels();
  inline std::vector<Corner> getCorners();
  inline std::vector<Edge> getEdges();
  
  /** Compute boundaries between surfaces and create boundary network **/
  virtual void compute();

  /** Update contour based on a surface indices list **/
//   void updateContour(const std::vector<int> &_indices, 
//                      std::vector< std::vector<int> > &_contours);
};


inline std::vector<Edgel> BoundaryDetector::getEdgels()
{
  return edgels;
}

inline std::vector<Corner> BoundaryDetector::getCorners()
{
  return corners;
}

inline std::vector<Edge> BoundaryDetector::getEdges()
{
  return edges;
}


} //--END--

#endif

