/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "RigidTransformationRANSAC.hh"


namespace kp 
{

using namespace std;


/********************** RigidTransformationRANSAC ************************
 * Constructor/Destructor
 */
RigidTransformationRANSAC::RigidTransformationRANSAC(Parameter p)
 : param(p)
{
}

RigidTransformationRANSAC::~RigidTransformationRANSAC()
{
}




/************************** PRIVATE ************************/



/**
 * GetRandIdx
 */
void RigidTransformationRANSAC::GetRandIdx(int size, int num, std::vector<int> &idx)
{
  int temp;
  idx.clear();
  for (int i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(Contains(idx,temp));
    idx.push_back(temp);
  }
}

/**
 * GetDistances
 */
void RigidTransformationRANSAC::GetDistances(
      const std::vector<Eigen::Vector3f > &srcPts,
      const std::vector<Eigen::Vector3f > &tgtPts,
      const Eigen::Matrix4f &transform,
      std::vector<float> &dists)
{
  dists.resize(srcPts.size());
  Eigen::Vector3f pt;
  Eigen::Matrix3f R = transform.topLeftCorner<3,3>();
  Eigen::Vector3f t = transform.block<3,1>(0,3);

  for (unsigned i = 0; i < srcPts.size (); i++)
  {
    pt  = R * srcPts[i] + t;
    dists[i] = (pt - tgtPts[i]).squaredNorm ();
  }
}

/**
 * CountInliers
 */
unsigned RigidTransformationRANSAC::CountInliers(std::vector<float> &dists)
{
  unsigned cnt=0;
  float sqrDist = (float)param.inl_dist*param.inl_dist;

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < sqrDist)
      cnt++;
  }

  return cnt;
}

/**
 * GetInliers
 */
void RigidTransformationRANSAC::GetInliers(std::vector<float> &dists, std::vector<int> &inliers)
{
  inliers.clear();

  float sqrDist = (float)param.inl_dist*param.inl_dist;

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < sqrDist)
      inliers.push_back(i);
  }
}

/**
 * Robust estimation of object poses
 */
void RigidTransformationRANSAC::Ransac(
      const std::vector<Eigen::Vector3f > &srcPts,
      const std::vector<Eigen::Vector3f > &tgtPts,
      Eigen::Matrix4f &transform,
      std::vector<int> &inliers)
{
  // int ransac
  int k=0;
  float sig=3., svSig=0.;
  float eps = sig/(float)srcPts.size();
  std::vector<int> idxKeys;
  Eigen::Matrix4f tmpRT;
  std::vector<float> dists(srcPts.size());

  //ransac pose
  while (pow(1. - pow(eps,4), k) >= param.eta_ransac && k < (int)param.max_rand_trials)
  {

    GetRandIdx(srcPts.size(), 4, idxKeys);

    estimateRigidTransformationSVD(srcPts, idxKeys, tgtPts, idxKeys, tmpRT);

    GetDistances(srcPts, tgtPts, tmpRT, dists);
    sig = CountInliers(dists);

    if (sig > svSig)
    {
      svSig = sig;
      transform = tmpRT;
      
      eps = svSig / (float)srcPts.size(); 
    }

    k++;
  }

  inliers.clear();
  if (svSig>3.1)
  {
    GetDistances(srcPts, tgtPts, transform, dists);
    GetInliers(dists, inliers);

    estimateRigidTransformationSVD(srcPts, inliers, tgtPts, inliers, transform);

    GetDistances(srcPts, tgtPts, transform, dists);
    GetInliers(dists, inliers);
  }

  //cout<<"Number of ransac trials: "<<k<<", inl="<<svSig<<"("<<inliers.size()<<")/"<<srcPts.size()<<endl;
}


/**
 * ComputeCentroid
 */
void RigidTransformationRANSAC::ComputeCentroid(
      const std::vector<Eigen::Vector3f > &pts, 
      const std::vector<int> &indices,
      Eigen::Vector3f &centroid)
{
  // Initialize to 0
  centroid.setZero ();
  if (pts.size()==0 || indices.size()==0)
    return;

  for (unsigned i = 0; i < indices.size (); i++)
    centroid += pts[indices[i]];

  centroid /= indices.size ();
}

/**
 * DemeanPoints
 */
void RigidTransformationRANSAC::DemeanPoints (
      const std::vector<Eigen::Vector3f > &inPts, 
      const std::vector<int> &indices,
      const Eigen::Vector3f &centroid,
      Eigen::MatrixXf &outPts)
{
  if (inPts.size()==0 || indices.size()==0)
  {
    return;
  }

  outPts = Eigen::MatrixXf(4, indices.size());

  // Subtract the centroid from points
  for (unsigned i = 0; i < indices.size(); i++)
    outPts.block<3,1>(0,i) = inPts[indices[i]]-centroid;

  outPts.block(3, 0, 1, indices.size()).setZero();
}





/************************** PUBLIC *************************/

/**
 * estimateRigidTransformationSVD
 */
void RigidTransformationRANSAC::estimateRigidTransformationSVD(
      const std::vector<Eigen::Vector3f > &srcPts,
      const std::vector<int> &srcIndices,
      const std::vector<Eigen::Vector3f > &tgtPts,
      const std::vector<int> &tgtIndices,
      Eigen::Matrix4f &transform)
{
  Eigen::Vector3f srcCentroid, tgtCentroid;

  // compute the centroids of source, target
  ComputeCentroid (srcPts, srcIndices, srcCentroid);
  ComputeCentroid (tgtPts, tgtIndices, tgtCentroid);

  // Subtract the centroids from source, target
  Eigen::MatrixXf srcPtsDemean;
  DemeanPoints(srcPts, srcIndices, srcCentroid, srcPtsDemean);

  Eigen::MatrixXf tgtPtsDemean;
  DemeanPoints(tgtPts, tgtIndices, tgtCentroid, tgtPtsDemean);

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = (srcPtsDemean * tgtPtsDemean.transpose ()).topLeftCorner<3, 3>();

  // Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU ();
  Eigen::Matrix3f v = svd.matrixV ();

  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }

  // Return the transformation
  Eigen::Matrix3f R = v * u.transpose ();
  Eigen::Vector3f t = tgtCentroid - R * srcCentroid;

  // set rotation
  transform.block(0,0,3,3) = R;
  // set translation
  transform.block(0,3,3,1) = t;
  transform.block(3, 0, 1, 3).setZero();  
  transform(3,3) = 1.;
}

/**
 * compute
 */
void RigidTransformationRANSAC::compute(
      const std::vector<Eigen::Vector3f> &srcPts,
      const std::vector<Eigen::Vector3f> &tgtPts,
      Eigen::Matrix4f &transform,
      std::vector<int> &inliers)
{
  if (srcPts.size()<4 || srcPts.size()!=tgtPts.size())
    throw std::runtime_error("[RigidTransformationRANSAC::estimateRigidTransformationSVD] Invalide points!");

  Ransac(srcPts, tgtPts, transform, inliers);
}


} //-- THE END --






