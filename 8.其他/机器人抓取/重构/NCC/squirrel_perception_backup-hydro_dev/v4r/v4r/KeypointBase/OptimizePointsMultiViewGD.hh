/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_OPTIMIZE_POINTS_MULTIVIEW_GD_HH
#define KP_OPTIMIZE_POINTS_MULTIVIEW_GD_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include <Eigen/Dense>
#include "v4r/KeypointCameraTracker/Scene.hh"
#include "v4r/KeypointCameraTracker/View.hh"
#include "v4r/KeypointTools/SmartPtr.hpp"
#include "MultiViewPatchError.hpp"




namespace kp
{

/**
 * OptimizePointsMultiViewGD
 */
class OptimizePointsMultiViewGD
{
public:
  class Parameter
  {
  public:
    float init_delta_rot;     //0.35 rad
    float init_delta_trans;   //0.001 m
    float convergence_rot;    //0.017 rad
    float convergence_trans;  //0.0001 m
    int max_iter;
    float view_ray_normal_offset;
    float max_residual;       // 0.6
    float max_residual_optimized; //0.3
    cv::Size patch_size;
    Parameter(float _init_delta_rot=0.35, float _init_delta_trans=0.002,
      float _convergence_rot=0.017, float _convergence_trans=0.0001, 
      int _max_iter=50, float _view_ray_normal_offset=85, 
      float _max_residual=0.6, float _max_residual_optimized=0.3,
      const cv::Size &_patch_size=cv::Size(11,11))
    : init_delta_rot(_init_delta_rot), init_delta_trans(_init_delta_trans),
      convergence_rot(_convergence_rot), convergence_trans(_convergence_trans), 
      max_iter(_max_iter), view_ray_normal_offset(_view_ray_normal_offset),
      max_residual(_max_residual), max_residual_optimized(_max_residual_optimized),
      patch_size(_patch_size) {}
  };

private:

  float cos_view_ray_normal_offset;
  
  Eigen::Matrix3d C, invC;         // camera parameter

  std::vector<MVView> mv_views;
  std::vector<Eigen::Matrix4f> poses;
  
  // data structure
  bool have_im_pts, have_depth, have_points, have_normals, have_im_indices;
  std::vector<cv::Point2f> im_pts;
  std::vector<double> depth;
  std::vector<Eigen::Vector3f> points;
  std::vector<Eigen::Vector3f> normals;
  std::vector<bool> converged;
  std::vector< std::vector<int> > im_indices;
  std::vector<Eigen::Vector3f> view_rays;

  bool optimizeSequentialDim(MultiViewPatchError &mpe, Eigen::Vector3d &rot, double &ddepth);

  void initData(const Eigen::Matrix4f &pose);
  void initPoses(std::vector<MVView> &mv_views, std::vector<Eigen::Matrix4f> &poses);


  inline void getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R);
  inline void getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t);


public:
  Parameter param;

  OptimizePointsMultiViewGD(const Parameter &p=Parameter());
  ~OptimizePointsMultiViewGD();


  /** clear the data structure **/
  void clearFrames();

  /** frames and the corresponding camera pose with respect to the target frame **/
  int addFrame(const cv::Mat_<unsigned char> &image, const Eigen::Matrix4f &pose);

  /** Set image points of the target frame to refine. 
1f image points are notavailable 3d points are projected to the target frame **/
  void setImagePoints(const std::vector<cv::Point2f> &_im_pts);

  /** Set depth for each image point.
      If depth is not available it is computed from the 3d points **/
  void setDepth(const std::vector<double> &_depth);

  /** Set 3d points in target frame coordinates. 
      If 3d points are not available they are computed from image points and depth.**/
  void setPoints(const std::vector<Eigen::Vector3f> &_points);

  /** Set normals hypotheses. If not available view rays are used as normals hypotheses **/
  void setNormals(const std::vector<Eigen::Vector3f> &_normals);

  /** Set index to images for each point. If empty occlusions are estimated. **/
  void setImageIndices(const std::vector< std::vector<int> > &_idx);

  /** target frame to optimize **/
  void optimize(const cv::Mat_<unsigned char> &image, const Eigen::Matrix4f &pose=Eigen::Matrix4f::Identity());

  inline std::vector<cv::Point2f> &getImagePoints() {return im_pts;}
  inline std::vector<double> &getDepth() {return depth;}
  inline std::vector<Eigen::Vector3f> &getPoints() {return points;}
  inline std::vector<Eigen::Vector3f> &getNormals() {return normals;}
  inline std::vector<bool> &getStatus() {return converged;}
  inline std::vector< std::vector<int> > &getImageIndices() {return im_indices;}

  void setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs=cv::Mat());

  typedef kp::SmartPtr< ::kp::OptimizePointsMultiViewGD> Ptr;
  typedef kp::SmartPtr< ::kp::OptimizePointsMultiViewGD const> ConstPtr;

};




/*********************** INLINE METHODES **************************/

inline void OptimizePointsMultiViewGD::getR(const Eigen::Matrix4f &pose, Eigen::Matrix3d &R)
{
  R = pose.topLeftCorner<3,3>().cast<double>();
}

inline void OptimizePointsMultiViewGD::getT(const Eigen::Matrix4f &pose, Eigen::Vector3d &t)
{
  t = pose.block<3,1>(0,3).cast<double>();
}



}

#endif

