/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "ImageTransformRANSAC.hh"

//#define HOMEST_CV

namespace kp 
{

using namespace std;

double ImageTransformRANSAC::SQRT2 = sqrt(2.);

/********************** ImageTransformRANSAC ************************
 * Constructor/Destructor
 */
ImageTransformRANSAC::ImageTransformRANSAC(Parameter p)
 : param(p)
{
}

ImageTransformRANSAC::~ImageTransformRANSAC()
{
}




/************************** PRIVATE ************************/



/**
 * GetRandIdx
 */
void ImageTransformRANSAC::getRandIdx(int size, int num, std::vector<int> &idx)
{
  int temp;
  idx.clear();
  for (int i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(contains(idx,temp));
    idx.push_back(temp);
  }
}

/**
 * GetDistances
 */
void ImageTransformRANSAC::getDistances(
      const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
      const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
      const Eigen::Matrix3f &transform,
      std::vector<float> &dists)
{
  Eigen::Vector2f pt;
  dists.resize(src_pts.size());

  Eigen::Matrix2f R = transform.topLeftCorner<2, 2> ();
  Eigen::Vector2f t = transform.block<2,1> (0, 2);

  for (unsigned i = 0; i < src_pts.size (); i++)
  {
    pt  = R * src_pts[i] + t;
    dists[i] = (pt - tgt_pts[i]).squaredNorm ();
  }
}

/**
 * CountInliers
 */
unsigned ImageTransformRANSAC::countInliers(std::vector<float> &dists)
{
  unsigned cnt=0;
  float sqr_dist = (float)param.inl_dist*param.inl_dist;

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < sqr_dist)
      cnt++;
  }

  return cnt;
}

/**
 * GetInliers
 */
void ImageTransformRANSAC::getInliers(std::vector<float> &dists, std::vector<int> &inliers)
{
  inliers.clear();

  float sqr_dist = (float)param.inl_dist*param.inl_dist;

  for (unsigned i = 0; i < dists.size(); i++)
  {
    if (dists[i] < sqr_dist)
      inliers.push_back(i);
  }
}

/**
 * getMean
 */
void  ImageTransformRANSAC::normalizePoints(const std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > &pts_in, std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > &pts_out, Eigen::Matrix3d &T)
{
  float dist=0., scale;
  Eigen::Vector2d mean(0.,0.);

  for (unsigned i=0; i<pts_in.size(); i++)
    mean += pts_in[i];

  mean /= float(pts_in.size());

  pts_out.resize(pts_in.size());

  for (unsigned i=0; i<pts_in.size(); i++)
    pts_out[i] = pts_in[i]-mean;

  for (unsigned i=0; i<pts_out.size(); i++)
    dist += pts_out[i].norm();

  dist /= float(pts_out.size());
  scale = SQRT2/dist; 

  for (unsigned i=0; i<pts_out.size(); i++)
    pts_out[i] *= scale;

  T.setIdentity();
  T(0,0) = T(1,1) = scale;
  T(0,2) = -mean[0]*scale;
  T(1,2) = -mean[1]*scale;
}



/************************** PUBLIC *************************/
/**
 * @brief ImageTransformRANSAC::estimateSimilarityLS
 * @param src_pts
 * @param tgt_pts
 * @param transform
 */
void ImageTransformRANSAC::estimateSimilarityLS(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<int> &src_indices,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        const std::vector<int> &tgt_indices,
        Eigen::Matrix3f &transform)
{
  if (src_indices.size()<2 || src_indices.size()!=tgt_indices.size())
    throw std::runtime_error("[ImageTransformRANSAC::estimateSimilarityLS] Invalide points!");

  Eigen::MatrixXf A(src_indices.size()*2, 4);
  Eigen::VectorXf b(src_indices.size()*2);
  Eigen::VectorXf x;

  for( unsigned i = 0; i < src_indices.size(); i++ )
  {
    unsigned j = i*2;
    unsigned j1 = j+1;
    const Eigen::Vector2f &pt1 = src_pts[src_indices[i]];
    const Eigen::Vector2f &pt2 = tgt_pts[tgt_indices[i]];

    A(j,0) = A(j1,1) = pt1[0];
    A(j,1) = -pt1[1];
    A(j1,0) = pt1[1];
    A(j,3) = A(j1,2) = 0.;
    A(j,2) = A(j1,3) = 1.;
    b(j) = pt2[0];
    b(j1) = pt2[1];
  }

  x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  transform.setIdentity();
  transform(0,0) = x[0];
  transform(0,1) = -x[1];
  transform(1,0) = x[1];
  transform(1,1) = x[0];
  transform(0,2) = x[2];
  transform(1,2) = x[3];

}

/**
 * @brief ImageTransformRANSAC::estimateSimilarityLS
 * @param src_pts
 * @param tgt_pts
 * @param transform
 */
void ImageTransformRANSAC::estimateSimilarityLS(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        Eigen::Matrix3f &transform)
{
  if (src_pts.size()<2 || src_pts.size()!=tgt_pts.size())
    throw std::runtime_error("[ImageTransformRANSAC::estimateSimilarityLS] Invalide points!");

  Eigen::MatrixXf A(src_pts.size()*2, 4);
  Eigen::VectorXf b(src_pts.size()*2);
  Eigen::VectorXf x;

  for( unsigned i = 0; i < src_pts.size(); i++ )
  {
    unsigned j = i*2;
    unsigned j1 = j+1;
    const Eigen::Vector2f &pt1 = src_pts[i];
    const Eigen::Vector2f &pt2 = tgt_pts[i];

    A(j,0) = A(j1,1) = pt1[0];
    A(j,1) = -pt1[1];
    A(j1,0) = pt1[1];
    A(j,3) = A(j1,2) = 0.;
    A(j,2) = A(j1,3) = 1.;
    b(j) = pt2[0];
    b(j1) = pt2[1];
  }

  x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  transform.setIdentity();
  transform(0,0) = x[0];
  transform(0,1) = -x[1];
  transform(1,0) = x[1];
  transform(1,1) = x[0];
  transform(0,2) = x[2];
  transform(1,2) = x[3];
}


/**
 * @brief ImageTransformRANSAC::estimateAffineLS
 * @param src_pts
 * @param tgt_pts
 * @param transform
 */
void ImageTransformRANSAC::estimateAffineLS(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<int> &src_indices,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        const std::vector<int> &tgt_indices,
        Eigen::Matrix3f &transform)
{
  if (src_indices.size()<3 || src_indices.size()!=tgt_indices.size())
    throw std::runtime_error("[ImageTransformRANSAC::estimateAffineLS] Invalide points!");

  Eigen::MatrixXf A(src_indices.size()*2, 6);
  Eigen::VectorXf b(src_indices.size()*2);
  Eigen::VectorXf x;

  for( unsigned i = 0; i < src_indices.size(); i++ )
  {
    unsigned j = i*2;
    unsigned j1 = j+1;
    const Eigen::Vector2f &pt1 = src_pts[src_indices[i]];
    const Eigen::Vector2f &pt2 = tgt_pts[tgt_indices[i]];

    A(j,0) = A(j1,2) = pt1[0];
    A(j,1) = A(j1,3) = pt1[1];
    A(j,2) = A(j,3) = A(j,5) = A(j1,0) = A(j1,1) = A(j1,4) = 0.;
    A(j,4) = A(j1,5) = 1.;
    b(j) = pt2[0];
    b(j1) = pt2[1];
  }

  x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  transform.setIdentity();
  transform(0,0) = x[0];
  transform(0,1) = x[1];
  transform(1,0) = x[2];
  transform(1,1) = x[3];
  transform(0,2) = x[4];
  transform(1,2) = x[5];
}

/**
 * @brief ImageTransformRANSAC::estimateAffineLS
 * @param src_pts
 * @param tgt_pts
 * @param transform
 */
void ImageTransformRANSAC::estimateAffineLS(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        Eigen::Matrix3f &transform)
{
  if (src_pts.size()<3 || src_pts.size()!=tgt_pts.size())
    throw std::runtime_error("[ImageTransformRANSAC::estimateAffineLS] Invalide points!");

  Eigen::MatrixXf A(src_pts.size()*2, 6);
  Eigen::VectorXf b(src_pts.size()*2);
  Eigen::VectorXf x;

  for( unsigned i = 0; i < src_pts.size(); i++ )
  {
    unsigned j = i*2;
    unsigned j1 = j+1;
    const Eigen::Vector2f &pt1 = src_pts[i];
    const Eigen::Vector2f &pt2 = tgt_pts[i];

    A(j,0) = A(j1,2) = pt1[0];
    A(j,1) = A(j1,3) = pt1[1];
    A(j,2) = A(j,3) = A(j,5) = A(j1,0) = A(j1,1) = A(j1,4) = 0.;
    A(j,4) = A(j1,5) = 1.;
    b(j) = pt2[0];
    b(j1) = pt2[1];
  }

  x = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(b);

  transform.setIdentity();
  transform(0,0) = x[0];
  transform(0,1) = x[1];
  transform(1,0) = x[2];
  transform(1,1) = x[3];
  transform(0,2) = x[4];
  transform(1,2) = x[5];
}


/**
 * @brief ImageTransformRANSAC::estimateHomographyLS
 * @param src_pts
 * @param tgt_pts
 * @param transform
 */
void ImageTransformRANSAC::estimateHomographyLS(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<int> &src_indices,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        const std::vector<int> &tgt_indices,
        Eigen::Matrix3f &transform)
{
  if (src_indices.size()<4 || src_indices.size()!=tgt_indices.size())
    throw std::runtime_error("[ImageTransformRANSAC::estimateHomographyLS] Invalide points!");

  transform.setIdentity();

#ifdef HOMEST_CV
  Eigen::Vector2d pn1, pn2, center1(0.,0.), center2(0.,0.), scale1(0.,0.), scale2(0.,0.); 
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > pts1(src_indices.size());
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > pts2(tgt_indices.size());
  Eigen::Matrix3d T, T1, invT2;
  Eigen::MatrixXd A(Eigen::MatrixXd::Zero(9,9));
  Eigen::VectorXd Ax(Eigen::VectorXd::Zero(9)), Ay(Eigen::VectorXd::Zero(9));

  // convert to double
  for (unsigned i=0; i<src_indices.size(); i++)
  {
    const Eigen::Vector2f &pt = src_pts[src_indices[i]];
    pts1[i] = Eigen::Vector2d(pt[0],pt[1]);
  }
  for (unsigned i=0; i<tgt_indices.size(); i++)
  {
    const Eigen::Vector2f &pt = tgt_pts[tgt_indices[i]];
    pts2[i] = Eigen::Vector2d(pt[0],pt[1]);
  }

  // compute scale
  for( unsigned i = 0; i < pts1.size(); i++ )
  {
    center1 += pts1[i];
    center2 += pts2[i];
  }

  center1 /= double(pts1.size());
  center2 /= double(pts2.size());

  for( unsigned i = 0; i < pts1.size(); i++ )
  {
    Eigen::Vector2d &pt1 = pts1[i];
    Eigen::Vector2d &pt2 = pts2[i];

    scale1[0] += fabs(pt1[0] - center1[0]);
    scale1[1] += fabs(pt1[1] - center1[1]);
    scale2[0] += fabs(pt2[0] - center2[0]);
    scale2[1] += fabs(pt2[1] - center2[1]);
  }

  if( fabs(scale1[0]) <= numeric_limits<double>::epsilon() || fabs(scale1[1]) <= numeric_limits<double>::epsilon() ||
      fabs(scale2[0]) <= numeric_limits<double>::epsilon() || fabs(scale2[1]) <= numeric_limits<double>::epsilon() )
    return;

  scale1[0] = double(pts1.size())/scale1[0]; 
  scale1[1] = double(pts1.size())/scale1[1];
  scale2[0] = double(pts2.size())/scale2[0]; 
  scale2[1] = double(pts2.size())/scale2[1];

  invT2 << 1./scale2[0], 0, center2[0], 0, 1./scale2[1], center2[1], 0, 0, 1;
  T1 << scale1[0], 0, -center1[0]*scale1[0], 0, scale1[1], -center1[1]*scale1[1], 0, 0, 1;

  // set data matrix
  int j, k;
  for( unsigned i = 0; i <  pts1.size(); i++ )
  {
    pn1 = pts1[i]-center1;
    pn2 = pts2[i]-center2;
    pn1[0] *= scale1[0], pn1[1] *= scale1[1];
    pn2[0] *= scale2[0], pn2[1] *= scale2[1];

    Ax[0]=pn1[0], Ax[1]=pn1[1], Ax[2]=1., Ax[6]=-pn2[0]*pn1[0], Ax[7]=-pn2[0]*pn1[1], Ax[8]=-pn2[0]; 
    Ay[3]=pn1[0], Ay[4]=pn1[1], Ay[5]=1., Ay[6]=-pn2[1]*pn1[0], Ay[7]=-pn2[1]*pn1[1], Ay[8]=-pn2[1]; 

    for( j = 0; j < 9; j++ )
      for( k = j; k < 9; k++ )
        A(j,k) += Ax[j]*Ax[k] + Ay[j]*Ay[k];
  }

  // add symetric part
  for( j = 0; j < 9; j++ )
    for( k = 0; k < j; k++ )
      A(j,k) = A(k,j);

  //solve
  Eigen::JacobiSVD<Eigen::MatrixXd> svd (A, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd V = svd.matrixV();

  unsigned z=0;
  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++,z++)
      T(i,j) = V(z,8);

  //denormalize
  T = invT2 * T * T1;
  T /= T(2,2);

  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++)
      transform(i,j) = T(i,j);
#else
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp_pts1(src_indices.size());
  std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > tmp_pts2(tgt_indices.size());
  Eigen::MatrixXd A(2*src_indices.size(), 9);
  Eigen::Matrix3d T1, T2;
  Eigen::Matrix3d T;

  for (unsigned i=0; i<src_indices.size(); i++)
  {
    const Eigen::Vector2f &pt = src_pts[src_indices[i]];
    tmp_pts1[i] = Eigen::Vector2d(pt[0],pt[1]);
  }

  for (unsigned i=0; i<tgt_indices.size(); i++)
  {
    const Eigen::Vector2f &pt = tgt_pts[tgt_indices[i]];
    tmp_pts2[i] = Eigen::Vector2d(pt[0],pt[1]);
  }

  // normalize points
  normalizePoints(tmp_pts1, tmp_pts1, T1);
  normalizePoints(tmp_pts2, tmp_pts2, T2);

  // assemble data matrix
  for (unsigned i=0; i<tmp_pts1.size(); i++)
  {
    unsigned j = i*2;
    unsigned j1 = j+1;
    const Eigen::Vector2d &pt1 = tmp_pts1[i];
    const Eigen::Vector2d &pt2 = tmp_pts2[i];

    A(j,0) = -pt1[0];
    A(j,1) = -pt1[1];
    A(j,2) = -1.0;
    A(j,3) = 0.0;
    A(j,4) = 0.0;
    A(j,5) = 0.0;
    A(j,6) = pt2[0]*pt1[0];
    A(j,7) = pt1[1]*pt2[0];
    A(j,8) = pt2[0];
    A(j1,0) = 0.0;
    A(j1,1) = 0.0;
    A(j1,2) = 0.0;
    A(j1,3) = A(j,0);
    A(j1,4) = A(j,1);
    A(j1,5) = -1.0;
    A(j1,6) = pt2[1]*pt1[0];
    A(j1,7) = pt2[1]*pt1[1];
    A(j1,8) = pt2[1];
  }

  // Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::MatrixXd> svd (A, Eigen::ComputeFullU | Eigen::ComputeFullV);

  Eigen::MatrixXd V = svd.matrixV();
  Eigen::VectorXd W = svd.singularValues();

  unsigned z=0;
  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++,z++)
      T(i,j) = V(z,8);

  //denormalize
  T = T2.inverse() * T * T1;
  T /= T(2,2);

  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++)
      transform(i,j) = T(i,j);
#endif
}

/**
 * @brief ImageTransformRANSAC::ransacSimilarity
 * @param src_pts
 * @param tgt_pts
 * @param transform
 * @param inliers
 */
void ImageTransformRANSAC::ransacSimilarity(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        Eigen::Matrix3f &transform, std::vector<int> &inliers)
{
  if (src_pts.size()<3 || src_pts.size()!=tgt_pts.size())
    throw std::runtime_error("[ImageTransformRANSAC::ransacSimilarity] Invalide points!");

  // int ransac
  int k=0;
  float sig=2., sv_sig=0.;
  float eps = sig/(float)src_pts.size();
  std::vector<int> idx_keys;
  Eigen::Matrix3f tmpT;
  std::vector<float> dists(src_pts.size());

  //ransac pose
  while (pow(1. - pow(eps,2), k) >= param.eta_ransac && k < (int)param.max_rand_trials)
  {

    getRandIdx(src_pts.size(), 2, idx_keys);

    estimateSimilarityLS(src_pts, idx_keys, tgt_pts, idx_keys, tmpT);

    getDistances(src_pts, tgt_pts, tmpT, dists);
    sig = countInliers(dists);

    if (sig > sv_sig)
    {
      sv_sig = sig;
      transform = tmpT;

      eps = sv_sig / (float)src_pts.size();
    }

    k++;
  }

  inliers.clear();
  if (sv_sig>2.1)
  {
    getDistances(src_pts, tgt_pts, transform, dists);
    getInliers(dists, inliers);

    estimateSimilarityLS(src_pts, inliers, tgt_pts, inliers, transform);

    getDistances(src_pts, tgt_pts, transform, dists);
    getInliers(dists, inliers);
  }

  #ifdef DEBUG
  //cout<<"Number of ransac trials: "<<k<<", inl="<<sv_sig<<"("<<inliers.size()<<")/"<<src_pts.size()<<endl;
  #endif
}

/**
 * @brief ImageTransformRANSAC::ransacAffine
 * @param src_pts
 * @param tgt_pts
 * @param transform
 * @param inliers
 */
void ImageTransformRANSAC::ransacAffine(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        Eigen::Matrix3f &transform, std::vector<int> &inliers)
{
  if (src_pts.size()<4 || src_pts.size()!=tgt_pts.size())
    throw std::runtime_error("[ImageTransformRANSAC::ransacAffine] Invalide points!");

  // int ransac
  int k=0;
  float sig=3., sv_sig=0.;
  float eps = sig/(float)src_pts.size();
  std::vector<int> idx_keys;
  Eigen::Matrix3f tmpT;
  std::vector<float> dists(src_pts.size());

  //ransac pose
  while (pow(1. - pow(eps,3), k) >= param.eta_ransac && k < (int)param.max_rand_trials)
  {

    getRandIdx(src_pts.size(), 3, idx_keys);

    estimateAffineLS(src_pts, idx_keys, tgt_pts, idx_keys, tmpT);

    getDistances(src_pts, tgt_pts, tmpT, dists);
    sig = countInliers(dists);

    if (sig > sv_sig)
    {
      sv_sig = sig;
      transform = tmpT;

      eps = sv_sig / (float)src_pts.size();
    }

    k++;
  }

  inliers.clear();
  if (sv_sig>3.1)
  {
    getDistances(src_pts, tgt_pts, transform, dists);
    getInliers(dists, inliers);

    estimateAffineLS(src_pts, inliers, tgt_pts, inliers, transform);

    getDistances(src_pts, tgt_pts, transform, dists);
    getInliers(dists, inliers);
  }

  #ifdef DEBUG
  //cout<<"Number of ransac trials: "<<k<<", inl="<<sv_sig<<"("<<inliers.size()<<")/"<<src_pts.size()<<endl;
  #endif
}

/**
 * @brief ImageTransformRANSAC::ransacHomography
 * @param src_pts
 * @param tgt_pts
 * @param transform
 * @param inliers
 */
void ImageTransformRANSAC::ransacHomography(
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &src_pts,
        const std::vector<Eigen::Vector2f,Eigen::aligned_allocator<Eigen::Vector2f> > &tgt_pts,
        Eigen::Matrix3f &transform, std::vector<int> &inliers)
{
  if (src_pts.size()<5 || src_pts.size()!=tgt_pts.size())
    throw std::runtime_error("[ImageTransformRANSAC::ransacHomography] Invalide points!");

  // int ransac
  int k=0;
  float sig=4., sv_sig=0.;
  float eps = sig/(float)src_pts.size();
  std::vector<int> idx_keys;
  Eigen::Matrix3f tmpT;
  std::vector<float> dists(src_pts.size());

  //ransac pose
  while (pow(1. - pow(eps,4), k) >= param.eta_ransac && k < (int)param.max_rand_trials)
  {

    getRandIdx(src_pts.size(), 4, idx_keys);

    estimateHomographyLS(src_pts, idx_keys, tgt_pts, idx_keys, tmpT);

    getDistances(src_pts, tgt_pts, tmpT, dists);
    sig = countInliers(dists);

    if (sig > sv_sig)
    {
      sv_sig = sig;
      transform = tmpT;

      eps = sv_sig / (float)src_pts.size();
    }

    k++;
  }

  inliers.clear();
  if (sv_sig>4.1)
  {
    getDistances(src_pts, tgt_pts, transform, dists);
    getInliers(dists, inliers);

    estimateHomographyLS(src_pts, inliers, tgt_pts, inliers, transform);

    getDistances(src_pts, tgt_pts, transform, dists);
    getInliers(dists, inliers);
  }

  #ifdef DEBUG
  cout<<"Number of ransac trials: "<<k<<", inl="<<sv_sig<<"("<<inliers.size()<<")/"<<src_pts.size()<<endl;
  #endif
}


} //-- THE END --






