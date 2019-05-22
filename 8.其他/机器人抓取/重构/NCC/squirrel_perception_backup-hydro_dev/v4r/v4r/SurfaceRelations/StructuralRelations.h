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
 * @file StructuralRelations.h
 * @author Richtsfeld
 * @date December 2012
 * @version 0.1
 * @brief Calculate patch relations for structural level: Efficient version without fourier and gabor filter.
 */

#ifndef SURFACE_STRUCTURAL_RELATIONS_LIGHT_H
#define SURFACE_STRUCTURAL_RELATIONS_LIGHT_H

#include <omp.h>
#include <vector>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "ColorHistogram.h"
#include "Texture.h"
#include "BoundaryRelationsMeanDepth.hpp"
#include "BoundaryRelationsMeanColor.hpp"
#include "BoundaryRelationsMeanCurvature.hpp"
#include "Fourier.h"
#include "Gabor.h"

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/SurfaceUtils/EPBase.hpp"

#include "v4r/EPUtils/EPUtils.hpp"


namespace surface
{
  
class StructuralRelations: public EPBase
{

  enum UsedRelations {
    R_COS   = 0x0001,
    R_TR    = 0x0002,
    R_GS    = 0x0004,
    R_FS    = 0x0008,
    R_RS    = 0x0010,
    R_COS3  = 0x0020,
    R_CUM3  = 0x0040,
    R_DM2   = 0x0080,
    R_DV2   = 0x0100,
    R_CUV3  = 0x0200,
    R_3D2   = 0x0400,
  };
  
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW     /// for 32-bit systems for pcl mandatory
  
protected:

private:
  
  void computeNeighbors();
  
  bool have_surfaces;
  std::vector<SurfaceModel::Ptr> surfaces;              ///< Surfaces
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_model;
  
  bool have_neighbours2D, have_neighbours3D;
  std::map<borderIdentification,std::vector<neighboringPair> > ngbr3D_map;
  std::map<borderIdentification,std::vector<neighboringPair> > ngbr2D_map;
  
  std::vector<Relation> surfaceRelations;
  std::vector<Relation> validRelations;
//   bool have_relations;

  int usedRelations;
  bool trainMode;
  bool initialized;
  
  /** Project the datapoints of the plane surfaces to the model surface **/
  void projectPts2Model();

  //for texture
  cv::Mat_<cv::Vec3b> matImage;
  cv::Mat gray_image;
  //
  cv::Mat gray_image2;
  cv::Mat edges;

  std::vector<ColorHistogram::Ptr> hist;
  std::vector<Texture::Ptr> text;
  std::vector<Fourier::Ptr> fourier;
  std::vector<Gabor::Ptr> gabor;
  Gabor::Ptr permanentGabor;
  
public:
  StructuralRelations();
  ~StructuralRelations();
  
  /** Set neighbours 2D **/
  void setNeighbours2D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr2D_map);
  /** Set neighbours 3D **/
  void setNeighbours3D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr3D_map);
  
  /** Set surfaces **/
  void setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces);
  /** Set relations **/
//   void setRelations(const std::vector<Relation> _surfaceRelations);
  /**Set used relations **/
  void setUsedRelations(int _usedRelations);
  /**Set training mode **/
  void setTrainingMode(bool _trainMode);
  
  /** Get surfaces relations **/
//   inline std::vector<Relation> getRelations();
  /** Get surfaces relations **/
  inline std::vector<Relation> getValidRelations();
  /**Get used relations **/
  int getUsedRelations();

  void init();
  /** Compute relations for the segmenter **/
  virtual void compute();
  
};

// inline std::vector<Relation> StructuralRelations::getRelations()
// {
//   return surfaceRelations;
// }

inline std::vector<Relation> StructuralRelations::getValidRelations()
{
  return validRelations;
}

inline int StructuralRelations::getUsedRelations()
{
  return usedRelations;
}

} //--END--

#endif

