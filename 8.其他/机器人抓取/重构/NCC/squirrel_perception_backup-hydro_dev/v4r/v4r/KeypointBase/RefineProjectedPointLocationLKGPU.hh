/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_REFINE_PROJECTED_POINT_LOCATION_LK_GPU_HH
#define KP_REFINE_PROJECTED_POINT_LOCATION_LK_GPU_HH

#include <vector>
#include <iostream>
#include <stdexcept>
#include "RefineProjectedPointLocationLK.hh"
#include "RefineProjectedPointLocationLKbase.hh"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "v4r/KeypointTools/SmartPtr.hpp"

#include "v4r/TomGine5/GLContext.h"
#include "v4r/TomGine5/GLTexture.h"
#include "v4r/TomGine5/GLSLProgram.h"
#include "v4r/TomGine5/Pyramid.h"
#include <glm/glm.hpp>




namespace kp
{
/**
 * RefineProjectedPointLocationLKGPU
 */
class RefineProjectedPointLocationLKGPU : public RefineProjectedPointLocationLKbase
{
public:
  /*class Parameter
  {
  public:
    float step_factor;        // 2.???
    float min_determinant;
    float min_displacement;   //0.1
    int max_iterations;       //10
    int max_residual;
    float ncc_residual;
    bool use_ncc;
    cv::Size patch_size;
    Parameter(float _step_factor=10., float _min_determinant=0.01, float _min_displacement=0.1, 
      int _max_iterations=10, int _max_residual=15., float _ncc_residual=0.3, 
      bool _use_ncc=true, const cv::Size &_patch_size=cv::Size(16,16))
    : step_factor(_step_factor), min_determinant(_min_determinant), min_displacement(_min_displacement),
      max_iterations(_max_iterations), max_residual(_max_residual), ncc_residual(_ncc_residual),
      use_ncc(_use_ncc), patch_size(_patch_size) {}
  };*/

private:
  RefineProjectedPointLocationLK::Parameter param;



  //Simon magic:
  unsigned int maxPatchCount;
  unsigned int patchSize;
  unsigned int pyrDepth;//TODO: get these parameters inside the parameter structure
  //GL magic:
  tg::GLContext* context;
  tg::GLTexture2D* srcTexture;
  tg::Pyramid* srcPyramid;
  tg::GLTexture2D* tgtTexture;
  tg::Pyramid* tgtPyramid;
  tg::GLTexture2D* srcITexture;

  tg::GLTexture2D* tgtITexture;
  tg::GLTexture2D* gxxxyyyTexture;
  tg::GLTexture2D* errxyTexture;
  tg::GLTexture2D* s1s2distTexture;

  GLuint homographicReadVAO;
  GLuint homographicReadFBO;
  tg::GLSLProgram homographicReadProgram;
  GLuint readIndexVBO;
  glm::mat3* homographicReadMatData;
  GLuint homographicReadMatVBO;
  GLuint quadVBO;

  GLuint tgtReadVAOs[2];
  GLuint tgtReadFBO;
  tg::GLSLProgram tgtReadProgram;
  glm::vec3* tgtReadPointData;


  GLuint gxyerrVAO;
  GLuint gxyerrFBO;
  tg::GLSLProgram gxyerrProgram;
  GLuint gxyerrPointCountUniform;

  //normalized cross correlation
  GLuint nccVAO;
  GLuint nccFBO;
  tg::GLSLProgram nccProgram;

  //Simple sobel filter:
  /*GLuint sobelVAO;
  GLuint sobelFBO;
  tg::GLSLProgram sobelProgram;*/

  //solve Transform feedback TODOOOO
  GLuint solveTFVAOs[2];
  tg::GLSLProgram solveTFProgram;
  GLuint positionTFVBOs[2];
  GLuint positionTFBOs[2];


  cv::Mat_<double> src_intrinsic, tgt_intrinsic;
  cv::Mat_<double> src_dist_coeffs, tgt_dist_coeffs;
  Eigen::Matrix3f src_C, tgt_C;


  Eigen::Matrix4f pose_src, pose_tgt;
  Eigen::Matrix4f inv_pose_tgt, delta_pose;
  Eigen::Matrix3f R_tgt, delta_R;
  Eigen::Vector3f t_tgt, delta_t;

  bool solve(const cv::Point2f &err, float gxx, float gxy, float gyy, cv::Point2f &delta);



public:
  RefineProjectedPointLocationLKGPU(const RefineProjectedPointLocationLK::Parameter &p=RefineProjectedPointLocationLK::Parameter());
  virtual ~RefineProjectedPointLocationLKGPU();

  virtual void setSourceImage(const cv::Mat_<unsigned char> &_im_src, const Eigen::Matrix4f &_pose_src);
  virtual void setTargetImage(const cv::Mat_<unsigned char> &_im_tgt, const Eigen::Matrix4f &_pose_tgt);
  virtual void refineImagePoints(const std::vector<Eigen::Vector3f> &pts, 
        const std::vector<Eigen::Vector3f> &normals, 
        std::vector<cv::Point2f> &im_pts_tgt, std::vector<int> &converged);

  virtual void setSourceCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);
  virtual void setTargetCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs);

  typedef SmartPtr< ::kp::RefineProjectedPointLocationLKGPU> Ptr;
  typedef SmartPtr< ::kp::RefineProjectedPointLocationLKGPU const> ConstPtr;
};






}

#endif

