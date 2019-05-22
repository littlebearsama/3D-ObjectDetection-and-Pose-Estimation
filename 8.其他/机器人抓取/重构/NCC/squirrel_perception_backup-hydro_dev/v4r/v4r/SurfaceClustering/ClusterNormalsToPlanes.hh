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

#ifndef SURFACE_CLUSTER_NORMALS_TO_PLANES_HH
#define SURFACE_CLUSTER_NORMALS_TO_PLANES_HH

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include "v4r/SurfaceUtils/SurfaceModel.hpp"
#include "v4r/SurfaceUtils/EPBase.hpp"

#include "PPlane.h"

namespace surface
{

/**
 * ClusterNormalsToPlanes
 */
class ClusterNormalsToPlanes: public EPBase
{
public:
  
  typedef boost::shared_ptr<ClusterNormalsToPlanes> Ptr;
  
  class Parameter
  {
  public:
    float thrAngle;             // Threshold of angle for normal clustering
    float inlDist;              // Maximum inlier distance
    int minPoints;              // Minimum number of points for a plane
    
    bool adaptive;              // use adaptive functions for threshold of angle
    float d_c;                  // salient point (from constant to gradient)
    float epsilon_c;            // constant value for angle threshold (radiant)
    float epsilon_g;            // gradient value for angle threshold (radiant gradient)
    
    float omega_c;              // constant value for inlier (normal) distance threshold
    float omega_g;              // gradient value for inlier (normal) distance threshold
    
    float ra_dist;              // maximum distance for reasigning points to other patches
    
    Parameter(float thrAngleNC=0.6, float _inlDist=0.02, int _minPoints=9, bool _adaptive=false, float _d_c = 0.0, 
              float _epsilon_c=0.54, float _epsilon_g=0.1, float _omega_c=-0.004, float _omega_g=0.015, float _ra_dist = 0.01) : 
              thrAngle(thrAngleNC), inlDist(_inlDist), minPoints(_minPoints), adaptive(_adaptive), d_c(_d_c),
              epsilon_c(_epsilon_c), epsilon_g(_epsilon_g), omega_c(_omega_c), omega_g(_omega_g), ra_dist(_ra_dist) {}

    void print()
    {
      printf("[ClusterNormalsToPlanes::Parameter]\n  ");
      printf("  thrAngle: %f\n  ", thrAngle);
      printf("  inlDist: %f\n  ", inlDist);
      printf("  minPoints: %d\n  ", minPoints);
      printf("  adaptive: %s\n  ", adaptive ? "true" : "false");
      printf("  d_c: %f\n  ", d_c);
      printf("  epsilon_c: %f\n  ", epsilon_c);
      printf("  epsilon_g: %f\n  ", epsilon_g);
      printf("  omega_c: %f\n  ", omega_c);
      printf("  omega_g: %f\n  ", omega_g);
      printf("  ra_dist: %f\n  ", ra_dist);
    }
  };

private:
  Parameter param;
  float cosThrAngleNC;
  
  std::vector< std::vector<float> > srt_curvature; //@ep: TODO should be int
  
  std::vector<float> p_adaptive_cosThrAngleNC;                          ///< adaptive angle
  std::vector<float> p_adaptive_inlDist;                                ///< adaptive inlier distance
  
  bool pixel_check;
  int max_neighbours;              //< Maximum pixel neighbors for pixel check
  int max_nneighbours;             //< Maximum neighbouring neighbors for pixel check
  cv::Mat_<int> patches;           //< Patch indices(+1) on 2D image grid
  std::vector<bool> mask;           //< Points that are used
  
  std::vector<SurfaceModel::Ptr> surfaces; //< Created surfaces

  // calculates parameters of angles between normals and distances depending on z value
  void calculateCloudAdaptiveParameters();
  // creates patch image TODO: should be transfered
  void createPatchImage();
  // @ep: do not understand this function
  void countNeighbours(std::vector< std::vector<int> > &reassign_idxs, int nb, int nnb, int inc);
  // assigns points to the closest patch
  bool reasignPoints(std::vector< std::vector<int> > &reassign_idxs);
  // deletes planes with no points and put -1 as a type in planes where there is les than minPoints
  void deleteEmptyPlanes();
  // @ep: something with line ends???
  void singlePixelCheck();
  // @ep: something with removeing and reasigning pixels???
  void pixelCheck();
  // cluster not clustered points yet
  //void clusterRest(int idx, std::vector<int> &pts, pcl::Normal &normal);
  // cluster normals
  void clusterNormals();
  // cluster normals from point
  void clusterNormals(int idx, std::vector<int> &pts, pcl::Normal &normal, bool rest = false);
  // least square plane and updated normals???
  void computeLeastSquarePlanes();
  // adds normals to each point of segmented patches
  void addNormals();
  void pruneSurfaces();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ClusterNormalsToPlanes(Parameter p=Parameter());
  ~ClusterNormalsToPlanes();

  /** Set parameters for plane estimation **/
  void setParameter(Parameter p);

  /** Set surface check: try to reasign single pixels to planes **/
  void setPixelCheck(bool check, int neighbors);

  /** Compute planes by surface normal grouping **/
  virtual void compute();
  
  /** Compute planes by surface normal grouping **/
  inline std::vector<SurfaceModel::Ptr> getSurfaces();
  
  /** Prints created pathc image **/
  void printClusters(std::string file_name);
  void printCloudAdaptiveParams(std::string file_name);
  void printMask(std::string file_name);
  void printSrtCurvature(std::string file_name);
  void printClusteredIndices(std::string file_name, int idx, std::vector<int> pts);
  void print(std::string file_name);
  
  void showSurfaces(cv::Mat &kImage);

};

inline std::vector<SurfaceModel::Ptr> ClusterNormalsToPlanes::getSurfaces()
{
  return surfaces;
}

}

#endif

