/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "ProjBundleAdjuster.hh"
#include "v4r/KeypointTools/invPose.hpp"


namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
ProjBundleAdjuster::ProjBundleAdjuster(const Parameter &p)
 : param(p)
{ 
}

ProjBundleAdjuster::~ProjBundleAdjuster()
{
}

/**
 * getDataToBundle
 */
/*void ProjBundleAdjuster::getDataToBundle(const std::vector<View::Ptr> &views, std::vector<Camera> &cams, std::vector<Point3D> &pts, std::vector<ImagePoint> &im_pts, std::vector<View::Ptr> &kfs)
{
  for (unsigned i=0; i<views.size(); i++)
    if (views[i]->is_keyframe)
      kfs.push_back(views[i]);      

  // get cameras
  cams.resize(kfs.size());
  for (unsigned i=0; i<kfs.size(); i++)
  {
    View &kf = *kfs[i];
    Camera &cam = cams[i];
    cam.image = i;
    getR(kf.pose, cam.R);
    getT(kf.pose, cam.t);
  }

  // get 3d points and image points
  unsigned z;
  Eigen::Vector3d pt;
  std::vector<ImagePoint> tmp_im;
  pts.clear(); 
  im_pts.clear();
  LinkedKeypoint::nb_cnt++;

  for (int i=kfs.size()-1; i>=int(kfs.size())-param.inc_bundle_frames && i>=0; i--)
  {
    View &kf = *kfs[i];
    for (int j=0; j<int(kf.keys.size()); j++)
    {
      LinkedKeypoint &key = kf.keys[j];
      z=0;
      pt = Eigen::Vector3d::Zero();
      tmp_im.clear();
      if (key.bw!=-1 && key.nb != LinkedKeypoint::nb_cnt)
      {
        int bw = key.bw;
        tmp_im.push_back(ImagePoint(i, pts.size(), key.pt.x, key.pt.y));
        key.nb = LinkedKeypoint::nb_cnt;
        if (!isnan(key.pt3[0])){
          pt += Eigen::Vector3d(key.pt3[0],key.pt3[1],key.pt3[2]);
          z++;
        }
        for (int k=i-1; k>=int(kfs.size())-param.inc_bundle_frames && k>=0; k--) 
        {
          LinkedKeypoint &key = kfs[k]->keys[bw];
          tmp_im.push_back(ImagePoint(k, pts.size(), key.pt.x, key.pt.y));
          key.nb = LinkedKeypoint::nb_cnt;
          if (!isnan(key.pt3[0])){
            pt += Eigen::Vector3d(key.pt3[0],key.pt3[1],key.pt3[2]);
            z++;
          }
          if (key.bw!=-1)
            bw = key.bw;
          else break;
        }
        if (z>0)
        {
          pt /= double(z);
          pts.push_back(Point3D(pts.size(), pt));
          im_pts.insert(im_pts.end(), tmp_im.begin(), tmp_im.end()); 
        }
      }
    }
  }
}*/

/**
 * getDataToBundle
 */
/**void ProjBundleAdjuster::copyBackBundledData(const std::vector<Camera> &cams, const std::vector<Point3D> &pts, std::vector<View::Ptr> &kfs)
{
  //kfs.clear();  //HMMMMMM???????

  // get cameras
  for (unsigned i=0; i<kfs.size(); i++)
  {
    View &kf = *kfs[i];
    const Camera &cam = cams[i];
    setPose(cam.R, cam.t, kf.pose);
  }

  // get 3d points and image points
  unsigned z, z1=0;
  LinkedKeypoint::nb_cnt++;

  for (int i=kfs.size()-1; i>=int(kfs.size())-param.inc_bundle_frames && i>=0; i--)
  {
    View &kf = *kfs[i];
    for (int j=0; j<int(kf.keys.size()); j++)
    {
      LinkedKeypoint &key = kf.keys[j];
      z=0;
      if (key.bw!=-1 && key.nb != LinkedKeypoint::nb_cnt)
      {
        int bw = key.bw;
        key.nb = LinkedKeypoint::nb_cnt;
        if (!isnan(key.pt3[0]))
          z++;
        for (int k=i-1; k>=int(kfs.size())-param.inc_bundle_frames && k>=0; k--) 
        {
          LinkedKeypoint &key = kfs[k]->keys[bw];
          key.nb = LinkedKeypoint::nb_cnt;
          if (!isnan(key.pt3[0])){
            z++;
          }
          if (key.bw!=-1)
            bw = key.bw;
          else break;
        }
        if (z>0)
        {
          const Eigen::Vector3d &pt = pts[z1].X; 
          key.pt3 = Eigen::Vector3f(pt[0],pt[1],pt[2]);
          for (int k=i-1; k>=int(kfs.size())-param.inc_bundle_frames && k>=0; k--)
          {
            LinkedKeypoint &key = kfs[k]->keys[bw];
            if (!isnan(key.pt3[0]))
              key.pt3 = Eigen::Vector3f(pt[0],pt[1],pt[2]);
          }
          z1++;
        }
      }
    }
  }
}*/

/**
 * getDataToBundle
 */
void ProjBundleAdjuster::getDataToBundle(const Scene &scene, std::vector<Camera> &cameras)
{
  cameras.clear();
  int z=0, num_frames=1; 
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  if (!dbg.empty()) cout<<"[ProjBundleAdjuster::getDataToBundle] "<<param.inc_bundle_frames<<endl;

  for (int i=scene.views.size()-2; i>=0 && num_frames<param.inc_bundle_frames; i--)
    if (scene.views[i]->is_keyframe) num_frames++;

  if (num_frames<2) return;

  cameras.resize(num_frames);

  for (int i=scene.views.size()-1; i>=0 && z<num_frames; i--)
  {
    if (scene.views[i]->is_keyframe || i==(int)scene.views.size()-1)
    {
      const View &view = *scene.views[i];
      getR(view.pose, R);
      getT(view.pose, t);
      ceres::RotationMatrixToAngleAxis(&R(0,0), &cameras[z].pose_Rt(0));                
      cameras[z].pose_Rt.tail<3>() = t;
      cameras[z].idx = i;
      z++;
    }
  } 
}

/**
 * setBundledData
 */
void ProjBundleAdjuster::setBundledData(const std::vector<Camera> &cameras, Scene &scene)
{
  Eigen::Matrix4f inv_pose, pose, delta_pose;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  for (int i=cameras.size()-1; i>=0; i--)
  {
    const Camera &cam = cameras[i];

    ceres::AngleAxisToRotationMatrix(&cam.pose_Rt(0), &R(0,0));
    t = cam.pose_Rt.tail<3>();

    setPose(R,t, pose);
    
    invPose(scene.views[cam.idx]->pose,inv_pose);
    delta_pose = pose*inv_pose;
    scene.views[cam.idx]->pose = pose;

    // update/ move un-bundled cameras
    for (int j=cam.idx+1; j<(int)scene.views.size(); j++)
    {
      View &view = *scene.views[j];
      if (view.is_keyframe) break;
      view.pose = delta_pose*view.pose;
    }
  }
}

/*void ProjBundleAdjuster::packCamerasRotationAndTranslation(const vector<Camera> &all_cameras, std::vector<Eigen::Matrix<double, 6, 1> > &all_cameras_R_t)
{
  all_cameras_R_t.resize(all_cameras.size());

  for (unsigned i = 0; i<all_cameras_R_t.size(); i++) 
  {
    ceres::RotationMatrixToAngleAxis(&all_cameras[i].R(0, 0), &all_cameras_R_t[i](0));
    all_cameras_R_t[i].tail<3>() = all_cameras[i].t;
  }
}
*/

/**
 * unpackCamerasRotationAndTranslation
 */
/*void ProjBundleAdjuster::unpackCamerasRotationAndTranslation(const std::vector<Eigen::Matrix<double, 6, 1> > &all_cameras_R_t, std::vector<Camera> &all_cameras) 
{
  for (unsigned i = 0; i<all_cameras_R_t.size(); i++) 
  {
    ceres::AngleAxisToRotationMatrix(&all_cameras_R_t[i](0), &all_cameras[i].R(0, 0));
    all_cameras[i].t = all_cameras_R_t[i].tail<3>();
  }
}*/


// Apply camera intrinsics to the normalized point to get image coordinates.
// This applies the radial lens distortion to a point which is in normalized
// camera coordinates (i.e. the principal point is at (0, 0)) to get image
// coordinates in pixels. Templated for use with autodifferentiation.
template <typename T>
inline void ApplyRadialDistortionCameraIntrinsics(const T &focal_length_x,
                                                  const T &focal_length_y,
                                                  const T &principal_point_x,
                                                  const T &principal_point_y,
                                                  const T &k1,
                                                  const T &k2,
                                                  const T &k3,
                                                  const T &p1,
                                                  const T &p2,
                                                  const T &normalized_x,
                                                  const T &normalized_y,
                                                  T *image_x,
                                                  T *image_y) {
  T x = normalized_x;
  T y = normalized_y;

  // Apply distortion to the normalized points to get (xd, yd).
  T r2 = x*x + y*y;
  T r4 = r2 * r2;
  T r6 = r4 * r2;
  T r_coeff = (T(1) + k1*r2 + k2*r4 + k3*r6);
  T xd = x * r_coeff + T(2)*p1*x*y + p2*(r2 + T(2)*x*x);
  T yd = y * r_coeff + T(2)*p2*x*y + p1*(r2 + T(2)*y*y);

  // Apply focal length and principal point to get the final image coordinates.
  *image_x = focal_length_x * xd + principal_point_x;
  *image_y = focal_length_y * yd + principal_point_y;
}

// Cost functor which computes reprojection error of 3D point X
// on camera defined by angle-axis rotation and it's translation
// (which are in the same block due to optimization reasons).
//
// This functor uses a radial distortion model.
struct OpenCVReprojectionError {
  OpenCVReprojectionError(const double &_observed_x, const double &_observed_y)
      : observed_x(_observed_x), observed_y(_observed_y) {}

  template <typename T>
  bool operator()(const T* const intrinsics,
                  const T* const R_t,  // Rotation denoted by angle axis
                                       // followed with translation
                  const T* const X,    // Point coordinates 3x1.
                  T* residuals) const {
    // Unpack the intrinsics.
    const T& focal_length_x    = intrinsics[0];
    const T& focal_length_y    = intrinsics[1];
    const T& principal_point_x = intrinsics[2];
    const T& principal_point_y = intrinsics[3];
    const T& k1                = intrinsics[4];
    const T& k2                = intrinsics[5];
    const T& k3                = intrinsics[6];
    const T& p1                = intrinsics[7];
    const T& p2                = intrinsics[8];

    // Compute projective coordinates: x = RX + t.
    T x[3];

    ceres::AngleAxisRotatePoint(R_t, X, x);
    x[0] += R_t[3];
    x[1] += R_t[4];
    x[2] += R_t[5];

    // Compute normalized coordinates: x /= x[2].
    T xn = x[0] / x[2];
    T yn = x[1] / x[2];

    T predicted_x, predicted_y;

    // Apply distortion to the normalized points to get (xd, yd).
    ApplyRadialDistortionCameraIntrinsics(focal_length_x,
                                          focal_length_y,
                                          principal_point_x,
                                          principal_point_y,
                                          k1, k2, k3,
                                          p1, p2,
                                          xn, yn,
                                          &predicted_x,
                                          &predicted_y);

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - T(observed_x);
    residuals[1] = predicted_y - T(observed_y);

    return true;
  }

  const double observed_x;
  const double observed_y;
};


/**
 * bundle
 */
void ProjBundleAdjuster::bundle(Scene &scene, std::vector<Camera> &cameras)
{
  if (cameras.size()<2)
    return;


  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  double *intrinsics = &camera_intrinsics[0];  
  int num_residuals = 0;

  for (unsigned i=0; i<cameras.size(); i++)
  {
    View &view = *scene.views[cameras[i].idx];
    double *pose_Rt = &cameras[i].pose_Rt[0];

    for (unsigned j=0; j<view.keys.size(); j++)
    {
      LinkedKeypoint &key = view.keys[j];
      if (key.pt3_glob>=0)
      {
//double res[2];
//OpenCVReprojectionError t(key.pt.x, key.pt.y );
//t(intrinsics, pose_Rt, &scene.points[key.pt3_glob][0], &res[0]);
//cout<<res[0]<<" "<<res[1]<<endl;

        problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction< OpenCVReprojectionError, 2, 9, 6, 3 >( 
          new OpenCVReprojectionError( key.pt.x, key.pt.y )), NULL, intrinsics,
                                       pose_Rt, &scene.points[key.pt3_glob][0] );
        num_residuals++;
      }
    }
    
    for (unsigned j=0; j<view.marker.size(); j++)
    {
      Marker &m = view.marker[j];
      for (unsigned k=0; k<m.pts.size(); k++)
      {
        if (m.glob_idx[k]>=0)
        {
//double res[2];
//OpenCVReprojectionError t( m.pts[k].x, m.pts[k].y );
//t(intrinsics, pose_Rt, &scene.points[m.glob_idx[k]][0], &res[0]);
//cout<<res[0]<<" "<<res[1]<<endl;
          problem.AddResidualBlock(
          new ceres::AutoDiffCostFunction< OpenCVReprojectionError, 2, 9, 6, 3 >(
          new OpenCVReprojectionError( m.pts[k].x, m.pts[k].y )), NULL, intrinsics,
                                       pose_Rt, &scene.points[m.glob_idx[k]][0] );
          num_residuals++;
        }
      }
    }
  }

  /*if (cameras.back().idx==0)
  {
    cout<<"[ProjBundleAdjuster::bundle] Set first marker points constant!"<<endl;
    for (unsigned i=0; i<4 && i<scene.points.size(); i++)
      problem.SetParameterBlockConstant(&scene.points[i][0]);
  }
  else*/
/*  if (param.optimize_struct_only)  // set all cams const
  {
    for (unsigned i=0; i<cameras.size(); i++)
      problem.SetParameterBlockConstant(&cameras[i].pose_Rt[0]);
  }
  else                             // set only first cam const
*/
  {
    /*problem.SetParameterBlockConstant(&cameras.back().pose_Rt[0]);
    View &view = *scene.views[cameras.back().idx];
    
    for (unsigned i=0; i<view.keys.size(); i++)
      if ( view.keys[i].pt3_glob >= 0 )
        problem.SetParameterBlockConstant(&scene.points[view.keys[i].pt3_glob][0]);

    problem.SetParameterBlockConstant(&cameras[0].pose_Rt[0]);
    View &view = *scene.views[cameras[0].idx];
    unsigned z=0;
    for (unsigned i=0; i<view.keys.size(); i++) {
      if (view.keys[i].pt3_glob!=-1) {
        problem.SetParameterBlockConstant(&scene.points[view.keys[i].pt3_glob][0]);
        z++;
        if (z==3)break;
      }
    }*/
  }

  if (param.optimize_camera)
  {
    std::vector<int> constant_intrinsics;
    constant_intrinsics.push_back(4);
    constant_intrinsics.push_back(5);
    constant_intrinsics.push_back(6);
    constant_intrinsics.push_back(7);
    constant_intrinsics.push_back(8);

    ceres::SubsetParameterization *subset_parameterization =
        new ceres::SubsetParameterization(9, constant_intrinsics);

    problem.SetParameterization(intrinsics, subset_parameterization);
  }
  else
  {
    problem.SetParameterBlockConstant(intrinsics);
  }
 
  // Configure the solver.
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.use_inner_iterations = true;
  options.max_num_iterations = 100;

  if (!dbg.empty()) 
    options.minimizer_progress_to_stdout = true;
  else options.minimizer_progress_to_stdout = false;

  // Solve!
  ceres::Solver::Summary summary;

  ceres::Solve(options, &problem, &summary);

  if (!dbg.empty()) {
    std::cout << "Final report:\n" << summary.FullReport();
    cout<<"intrinsics:"<<endl;
    for (unsigned i=0; i<9; i++)
      cout<<camera_intrinsics[i]<<" ";
    cout<<endl;
  }
}



/***************************************************************************************/

/**
 * optimize
 */
void ProjBundleAdjuster::optimize()
{
  if (!dbg.empty()) cout<<"-- [ProjBundleAdjuster::bundle] debug out --"<<endl;

  if (camera_intrinsics.size()!=9)
    throw std::runtime_error("[ProjBundleAdjuster::optimize] Camera parameter not set!");
  if (scene.get()==0)
    throw std::runtime_error("[ProjBundleAdjuster::optimize] No data available!");


  getDataToBundle(*scene, cameras);

  if (cameras.size()<2) return;

  bundle(*scene, cameras);

  setBundledData(cameras, *scene); 

  if (!dbg.empty()) cout<<"[ProjBundleAdjuster::optimize] Number of cameras to bundle: "<<cameras.size()<<endl;

  /*bundle(all_image_points, &camera_intrinsics[0], &all_cameras, &all_points);

  copyBackBundledData(all_cameras, all_points, kfs);

  
  #ifdef KT_DEBUG
  cout<<"Number of cameras: "<<all_cameras.size()<<endl;
  cout<<"Number of tracks: "<<all_points.size()<<endl;
  cout<<"Number of image points: "<<all_image_points.size()<<endl;
  cout<<"--"<<endl;
  #endif
  */
}


/**
 * setCameraParameter
 */
void ProjBundleAdjuster::setCameraParameter(const cv::Mat &_intrinsic, const cv::Mat &_dist_coeffs) 
{
  dist_coeffs = cv::Mat_<double>();
  if (_intrinsic.type() != CV_64F)
    _intrinsic.convertTo(intrinsic, CV_64F);
  else intrinsic = _intrinsic;
  if (!_dist_coeffs.empty())
  {
    dist_coeffs = cv::Mat_<double>::zeros(1,8);
    for (int i=0; i<_dist_coeffs.cols*_dist_coeffs.rows; i++)
      dist_coeffs(0,i) = _dist_coeffs.at<double>(0,i);
  }

  camera_intrinsics.clear();
  camera_intrinsics.resize(9,0.);

  camera_intrinsics[0] = intrinsic(0,0);
  camera_intrinsics[1] = intrinsic(1,1);
  camera_intrinsics[2] = intrinsic(0,2);
  camera_intrinsics[3] = intrinsic(1,2);
  if (!dist_coeffs.empty())
  {
    camera_intrinsics[4] = dist_coeffs(0,0);
    camera_intrinsics[5] = dist_coeffs(0,1);
    camera_intrinsics[6] = dist_coeffs(0,5);
    camera_intrinsics[7] = dist_coeffs(0,2);
    camera_intrinsics[8] = dist_coeffs(0,3);
  }
}




}












