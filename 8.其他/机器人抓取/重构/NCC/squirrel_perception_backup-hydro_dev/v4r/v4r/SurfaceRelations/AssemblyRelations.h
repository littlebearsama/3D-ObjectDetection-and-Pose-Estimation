/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
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
 * @file AssemblyRelations.h
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate patch relations for assembly level.
 */

#ifndef SURFACE_ASSEMBLY_RELATIONS_H
#define SURFACE_ASSEMBLY_RELATIONS_H

#include <omp.h>
#include <vector>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
// #include "v4r/SurfaceBoundary/VisionCore.hh"

#include "ColorHistogram.h"
#include "Texture.h"
#include "Fourier.h"
#include "Gabor.h"
#include "BoundaryRelations.h"
#include "ContourNormalsDistance.hh"
// #include "Vs3ArcRelations.h"


namespace surface
{
  
class AssemblyRelations: public EPBase
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory

enum UsedRelations {
  R_COS   = 0x0001,
  R_TR    = 0x0002,
  R_GS    = 0x0004,
  R_FS    = 0x0008,
  R_RS    = 0x0010,
  R_MD    = 0x0020,
  R_NM    = 0x0040,
  R_NV    = 0x0080,
  R_ANG   = 0x0100,
  R_DN    = 0x0200,
  R_OC    = 0x0400,
//   R_BR0   = 0x0800,
//   R_BR1   = 0x1000,
//   R_BR2   = 0x2000,
//   R_BR3   = 0x4000,
//   R_BR4   = 0x8000,
};

protected:

private:
  
  double z_max;                                                 ///< maximum distance for 3D neighborhood of patches

  std::vector<Eigen::Vector3d> normalsMean;                    ///< Mean of surface normals of patch
  std::vector<double> normalsVar;                              ///< Variance of surface normals of patch
  
//   boundary::VisionCore *bdry_vs3;                               ///< vs3 implementation for boundary relations

  std::vector<SurfaceModel::Ptr> surfaces;
  bool have_surfaces;

//   std::vector<surface::Edge> edges;
//   bool have_edges;
//   std::vector<surface::Edgel> edgels;
//   bool have_edgels;
//   cv::Mat_<int> patchImage;
//   bool have_patch_image;

  bool have_neighbours2D, have_neighbours3D;
  std::vector< std::vector< std::vector<neighboringPair> > > ngbr2D;
  std::vector< std::vector< std::vector<neighboringPair> > > ngbr3D;
  
  void precalculateNormalRelations();
  bool calculateNormalRelations(int i, int j, double &_nor_mean, double &_nor_var);

//   bool calculateBoundaryRelations();
  
  bool is2DNeigbor(int p0, int p1);
  bool is3DNeighbor(int p0, int p1);

  int usedRelations;

  std::vector<Relation> surfaceRelations;
  bool have_relations;

public:
  AssemblyRelations();
  ~AssemblyRelations();

  /** Set neighbours 2D **/
  void setNeighbours2D(const std::vector< std::vector< std::vector<neighboringPair> > > _ngbr2D);
  /** Set neighbours 3D **/
  void setNeighbours3D(const std::vector< std::vector< std::vector<neighboringPair> > > _ngbr3D);
  
  /** Set surfaces **/
  void setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces);
  /** Set relations **/
  void setRelations(const std::vector<Relation> _surfaceRelations);

//   void setEdges(const std::vector<surface::Edge> _edges);
//   void setEdgels(const std::vector<surface::Edgel> _edgels);
//   void setPatchImage(cv::Mat_<int> &_patchImage);
  
  /** Compute relations for the segmenter **/
  virtual void compute();

  /** Get surfaces relations **/
  inline std::vector<Relation> getRelations();
  /**Set used relations **/
  void setUsedRelations(int _usedRelations);

  /**Get used relations **/
  int getUsedRelations();

};

/*************************** INLINE METHODES **************************/
inline
bool AssemblyRelations::is2DNeigbor(int p0, int p1)
{
  return (surfaces.at(p0)->neighbors2D.find(p1) != surfaces.at(p0)->neighbors2D.end());
}

inline
bool AssemblyRelations::is3DNeighbor(int p0, int p1)
{
  return (surfaces.at(p0)->neighbors3D.find(p1) != surfaces.at(p0)->neighbors3D.end());
}

inline int AssemblyRelations::getUsedRelations()
{
  return usedRelations;
}

inline std::vector<Relation> AssemblyRelations::getRelations()
{
  return surfaceRelations;
}

} //--END--

#endif

