/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "BundleAdjusterRT.hh"



namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
BundleAdjusterRT::BundleAdjusterRT(const Parameter &p)
{ 
}

BundleAdjusterRT::~BundleAdjusterRT()
{
}

/**
 * getDataToBundle
 */
void BundleAdjusterRT::getDataToBundle(const std::vector<View::Ptr> &views, std::vector<View::Ptr> &kfs, std::vector<Eigen::Matrix<double, 6, 1> > &cameras, std::vector<unsigned> &lt)
{
  Eigen::Matrix4f inv_pose;
  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  lt.resize(views.size(), UINT_MAX);

  for (unsigned i=0; i<views.size(); i++)
    if (views[i]->is_keyframe)
    {
      lt[i] = kfs.size();
      kfs.push_back(views[i]);      
    }

  cameras.resize(kfs.size()); 

  for (unsigned i=0; i<kfs.size(); i++)
  {
    View &view = *kfs[i];
    invPose(view.pose,inv_pose);
    getR(inv_pose, R);
    getT(inv_pose, t);

    ceres::RotationMatrixToAngleAxis(&R(0,0), &cameras[i](0));
    cameras[i].tail<3>() = t;
  }
}

/**
 * copyBackoBundledData
 */
void BundleAdjusterRT::copyBackBundledData(const std::vector<Eigen::Matrix<double, 6, 1> > &cameras, std::vector<View::Ptr> &kfs)
{
  Eigen::Matrix4f inv_pose;

  Eigen::Matrix3d R;
  Eigen::Vector3d t;

  for (unsigned i=0; i<kfs.size(); i++)
  {
    View &kf = *kfs[i];
    
    ceres::AngleAxisToRotationMatrix(&cameras[i](0), &R(0, 0));
    t = cameras[i].tail<3>();

    setPose(R, t, inv_pose);
    invPose(inv_pose,kf.pose);
  }
}


struct RigidTransformationError {
  RigidTransformationError(const double _x1, const double _y1, const double _z1, const double _x2, const double _y2, const double _z2)
      : pt1(Eigen::Vector3d(_x1, _y1, _z1)), pt2(Eigen::Vector3d(_x2, _y2, _z2)) {}

  template <typename T>
  bool operator()(const T* const pose1,
                  const T* const pose2,
                  T* residuals) const {

    // Compute global coordinates: x = RX + t.
    T glob_pt1[3];
    T glob_pt2[3];

    ceres::AngleAxisRotatePoint(pose1,(T*)&pt1[0], glob_pt1);
    glob_pt1[0] += pose1[3];
    glob_pt1[1] += pose1[4];
    glob_pt1[2] += pose1[5];

    ceres::AngleAxisRotatePoint(pose2, (T*)&pt2[0], glob_pt2);
    glob_pt2[0] += pose2[3];
    glob_pt2[1] += pose2[4];
    glob_pt2[2] += pose2[5];

    // The error is the difference between the predicted and observed position.
    residuals[0] = glob_pt1[0] - glob_pt2[0];
    residuals[1] = glob_pt1[1] - glob_pt2[1]; 
    residuals[2] = glob_pt1[2] - glob_pt2[2];

    return true;
  }

  const Eigen::Vector3d pt1;
  const Eigen::Vector3d pt2;
};


/**
 * bundle
 */
void BundleAdjusterRT::bundle(std::vector<View::Ptr> &kfs, std::vector<Eigen::Matrix<double, 6, 1> > &cameras, std::vector<unsigned> &lt)
{
  if (cameras.size()<2)
    return;

  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);

  for (int i=kfs.size()-1; i>0; i--)
  {
    View &view = *kfs[i];
    double *pose2 = &cameras[i][0];

    for (unsigned j=0; j<view.keys.size(); j++)
    {
      LinkedKeypoint &key = view.keys[j];
      for (unsigned k=0; k<key.links.size(); k++)
      {
        LinkedKeypoint &kf_key = kfs[lt[key.links[k].first]]->keys[key.links[k].second];
        if (!isnan(kf_key.pt3[0]))
        {
          Eigen::Vector3f &pt1 = kf_key.pt3;
          Eigen::Vector3f &pt2 = key.pt3;

          if (isnan(pt2[0]))
            continue;

          double *pose1 = &cameras[lt[key.links[k].first]][0];

          problem.AddResidualBlock(
            new ceres::NumericDiffCostFunction< RigidTransformationError, ceres::CENTRAL, 3, 6, 6 >( 
            new RigidTransformationError( pt1[0],pt1[1],pt1[2], pt2[0],pt2[1],pt2[2])), 
                                     NULL, pose1, pose2);
        }
      }
    }
  }

  // first camera is static
  problem.SetParameterBlockConstant(&cameras[0][0]);
 
  // Configure the solver.
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.preconditioner_type = ceres::SCHUR_JACOBI;
  options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  options.use_inner_iterations = true;
  options.max_num_iterations = 100;
  //options.linear_solver_type = ceres::DENSE_SCHUR;
  //options.linear_solver_type = ceres::DENSE_QR;
  #ifdef KT_DEBUG
  options.minimizer_progress_to_stdout = true;
  #else
  options.minimizer_progress_to_stdout = false;
  #endif

  // Solve!
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (!dbg.empty()) std::cout << "Final report:\n" << summary.FullReport();
}



/***************************************************************************************/

/**
 * setSharedData
 */
void BundleAdjusterRT::setSharedData(const Scene::Ptr &_scene) 
{
  scene = _scene;

  getDataToBundle(scene->views, kfs, cameras, lt_camera); 
}

/**
 * optimize
 */
void BundleAdjusterRT::optimize()
{
  if (scene.get()==0)
    throw std::runtime_error("[BundleAdjusterRT::optimize] No data available!");


  bundle(kfs, cameras, lt_camera);

  copyBackBundledData(cameras, kfs);

  if (!dbg.empty()) cout<<"--"<<endl;
}


}












