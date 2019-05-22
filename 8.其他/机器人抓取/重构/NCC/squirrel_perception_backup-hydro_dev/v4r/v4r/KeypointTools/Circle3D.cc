/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "Circle3D.hh"
#include "RandomNumbers.hpp"


namespace kp 
{

using namespace std;


/********************** Circle3D ************************
 * Constructor/Destructor
 */
Circle3D::Circle3D(const Parameter &p)
 : param(p)
{
}

Circle3D::~Circle3D()
{
}




/************************** PRIVATE ************************/

/**
 * countInliers
 */
unsigned Circle3D::countInliers(std::vector<float> &dists)
{
  unsigned cnt=0;

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < param.inl_dist)
      cnt++;
  }

  return cnt;
}

/**
 * GetInliers
 */
void Circle3D::getInliers(std::vector<float> &dists, std::vector<int> &inliers)
{
  inliers.clear();

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < param.inl_dist)
      inliers.push_back(i);
  }
}

/**
 * getDistances
 * hmmm, for now we only check the radius
 */
void Circle3D::getDistances(const std::vector<Eigen::Vector3f> &pts, Eigen::Vector3f &center, Eigen::Vector3f &normal, float &radius, std::vector<float> &dists)
{
  dists.resize(pts.size());

  for (unsigned i=0; i<pts.size(); i++)
  {
    dists[i] = fabs(radius-(center-pts[i]).norm());
  }
}






/************************** PUBLIC *************************/

/**
 * computeCircle
 */
int Circle3D::computeCircle(const Eigen::Vector3f &pt1, const Eigen::Vector3f &pt2, const Eigen::Vector3f &pt3, Eigen::Vector3f &center, Eigen::Vector3f &normal, float &radius)
{
  Eigen::Vector3f v1 = pt2-pt1;
  Eigen::Vector3f v2 = pt3-pt1;
  float sqr_l1 = v1.squaredNorm();
  
  if (sqr_l1 <= std::numeric_limits<float>::epsilon() || v2.squaredNorm() <= std::numeric_limits<float>::epsilon())
    return -1;    // identical points

  Eigen::Vector3f v1n = v1.normalized();
  Eigen::Vector3f v2n = v2.normalized();

  // normal of the plane
  normal = v1n.cross(v2n);

  if (normal[0]+normal[1]+normal[2] < 1e-10)
    return -2;    // colinear points

  // orthogonalization
  v2n = v2n - v2n.dot(v1n)*v1n;
  v2n.normalize();

  // prepare 2d points
  Eigen::Vector2f p1_2d, p2_2d, p3_2d(Eigen::Vector2f::Zero());
  for (unsigned i=0; i<3; i++)
  {
    p3_2d[0] = p3_2d[0] + v2[i]*v1n[i];
    p3_2d[1] = p3_2d[1] + v2[i]*v2n[i];
  }

  // .. and fit circle
  float a = sqrt(sqr_l1);
  float b = p3_2d[0];
  float c = p3_2d[1];
  float t = 0.5*(a-b)/c;
  float scale1 = b/2. + c*t;
  float scale2 = c/2. -b*t;

  for (unsigned i=0; i<3; i++)
    center[i] = pt1[i] + scale1*v1n[i] + scale2*v2n[i];

  radius = (center-pt1).norm();
  normal.normalize();

  return 0;
}



/**
 * ransacCircle
 */
void Circle3D::ransacCircle(const std::vector<Eigen::Vector3f> &pts, Eigen::Vector3f &center, Eigen::Vector3f &normal, float &radius, std::vector<int> &inliers)
{
  inliers.clear();
  if (pts.size()<3) return;

  // int ransac
  int k=0;
  float sig=3., sv_sig=0.;
  float tmp_r, eps = sig/(float)pts.size();
  std::vector<int> idx_keys;
  Eigen::Vector3f tmp_center, tmp_n;
  std::vector<float> dists(pts.size());

  //ransac pose
  while (pow(1. - pow(eps,3), k) >= param.eta_ransac && k < (int)param.max_rand_trials)
  {
    getRandIdx(pts.size(), 3, idx_keys);

    if(computeCircle(pts[idx_keys[0]], pts[idx_keys[1]], pts[idx_keys[2]], tmp_center, tmp_n, tmp_r)==0)
    {
      getDistances(pts, tmp_center, tmp_n, tmp_r, dists);
      sig = countInliers(dists);

      if (sig > sv_sig)
      {
        sv_sig = sig;
        center = tmp_center;
        normal = tmp_n;
        radius = tmp_r;
        eps = sv_sig / (float)pts.size();
      }
    }
    k++;
  }

  getDistances(pts, center, normal, radius, dists);
  getInliers(dists, inliers);

  cout<<"Number of ransac trials: "<<k<<", inl="<<sv_sig<<"("<<inliers.size()<<")/"<<pts.size()<<endl;
}


} //-- THE END --






