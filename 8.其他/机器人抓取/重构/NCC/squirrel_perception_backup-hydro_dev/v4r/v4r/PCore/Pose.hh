/**
 * $Id$
 * Johann Prankl, 2010-01-27 
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_POSE_HH
#define P_POSE_HH

#include <limits.h>
#include <map>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <stdexcept>
#include "PMath.hh"
#include "PMatrix.hh"


namespace P
{


class Pose
{
private:    
  

public:
  cv::Mat R;
  cv::Mat t;

  Pose();
  Pose(bool init);
  Pose(const cv::Mat& matR, const cv::Mat& matt);
  Pose(const Pose &pose);
  ~Pose();
  inline void GetHom(double pose[16]);
  inline void SetHom(double pose[16]);
  inline bool empty();
  inline void create();
  inline void release();
  inline void copyTo(Pose &dst);
  inline void invTo(Pose &dst);
  inline void init();
  Pose& operator=(const Pose &pose);

  static void Pose2ProjMat(double R[9], double t[3], double C[9], double P[12]);
};

inline void InitializePose(Pose &pose);

inline void CopyPose(Pose &src, Pose &dst);

inline void Rot2Quat(double R[9], double q[4]);
inline void Rot2Vec3(double R[9], double d[3]);
inline void Quat2Rot(double q[4], double R[9]);
inline void Vec32Rot(double d[3], double R[9]);

inline void InvPose(Pose &src, Pose &dst);
inline void MulPose(Pose &in1, Pose &in2, Pose &out);

template<typename T1,typename T2, typename T3, typename T4>
inline void ProjectPoint2Image(const T1 p[3], const T2 C[9], const T3 D[8], T4 i[2]);
inline void ProjectPoint2Image(double pin[3], double R[9], double t[3], double C[9], double i[2]);

std::ostream& operator<<(std::ostream &os, const Pose &pose);
std::istream& operator>>(std::istream &is, Pose &pose);




/*********************************** INLINE *********************************/
inline void Pose::init()
{
  InitializePose(*this);
}

inline void Pose::invTo(Pose &dst)
{
 InvPose(*this, dst);
}

inline void Pose::copyTo(Pose &dst)
{
  R.copyTo(dst.R);
  t.copyTo(dst.t); 
}

inline void Pose::create()
{
  R = cv::Mat(3,3,CV_64F);
  t = cv::Mat(3,1,CV_64F);
}

inline void Pose::release()
{
  R.release();
  t.release();
}

inline bool Pose::empty()
{
  if (R.empty() || t.empty())
    return true;
  return false;
}

inline void Pose::GetHom(double pose[16])
{
  //if (empty()) throw Except(__HERE__,"No pose available!");
  if (empty()) throw std::runtime_error ("Pose::GetHom: No pose available!");

  double *ptrR = R.ptr<double>(0);
  double *ptrt = t.ptr<double>(0);

  pose[0] = ptrR[0], pose[1] = ptrR[1], pose[2] = ptrR[2], pose[3] = ptrt[0];
  pose[4] = ptrR[3], pose[5] = ptrR[4], pose[6] = ptrR[5], pose[7] = ptrt[1];
  pose[8] = ptrR[6], pose[9] = ptrR[7], pose[10]= ptrR[8], pose[11]= ptrt[2];
  pose[12]= 0.,   pose[13] = 0.,  pose[14]= 0.,   pose[15]= 1.;
}

inline void Pose::SetHom(double pose[16])
{
  if (R.empty()) R =  cv::Mat(3,3,CV_64F);
  if (t.empty()) t = cv::Mat(3,1,CV_64F);

  double *ptrR = R.ptr<double>(0);
  double *ptrt = t.ptr<double>(0);

  ptrR[0] = pose[0], ptrR[1] = pose[1], ptrR[2] = pose[2], ptrt[0] = pose[3];
  ptrR[3] = pose[4], ptrR[4] = pose[5], ptrR[5] = pose[6], ptrt[1] = pose[7];
  ptrR[6] = pose[8], ptrR[7] = pose[9], ptrR[8] = pose[10],ptrt[2] = pose[11];
}


inline void InitializePose(Pose &pose)
{
  pose.R = cv::Mat(3,3,CV_64F);
  pose.t = cv::Mat(3,1,CV_64F);

  double *R = pose.R.ptr<double>(0);
  double *t = pose.t.ptr<double>(0);

  R[0] = 1, R[1] = 0, R[2] = 0;
  R[3] = 0, R[4] = 1, R[5] = 0;
  R[6] = 0, R[7] = 0, R[8] = 1;

  t[0] = 0, t[1] = 0, t[2] = 0;
}



inline void CopyPose(Pose &src, Pose &dst)
{ 
  src.R.copyTo(dst.R);
  src.t.copyTo(dst.t);
}

inline void Rot2Quat(double R[9], double q[4])
{
  double tmp[4];
  double mag;
  unsigned maxpos=0;
  tmp[0]=1.0 + R[0] + R[4] + R[8];
  tmp[1]=1.0 + R[0] - R[4] - R[8];
  tmp[2]=1.0 - R[0] + R[4] - R[8];
  tmp[3]=1.0 - R[0] - R[4] + R[8];

  mag=-1.0;
  for(unsigned i=0; i<4; i++){
    if(tmp[i]>mag){
      mag=tmp[i];
      maxpos=i;
    }
  }
  if(maxpos==0){
    q[0]=sqrt(tmp[0])*0.5;
    q[1]=(R[7] - R[5])/(4.0*q[0]);
    q[2]=(R[2] - R[6])/(4.0*q[0]);
    q[3]=(R[3] - R[1])/(4.0*q[0]);
  }
  else if(maxpos==1){
    q[1]=sqrt(tmp[1])*0.5;
    q[0]=(R[7] - R[5])/(4.0*q[1]);
    q[2]=(R[3] + R[1])/(4.0*q[1]);
    q[3]=(R[2] + R[6])/(4.0*q[1]);
  }
  else if(maxpos==2){
    q[2]=sqrt(tmp[2])*0.5;
    q[0]=(R[2] - R[6])/(4.0*q[2]);
    q[1]=(R[3] + R[1])/(4.0*q[2]);
    q[3]=(R[7] + R[5])/(4.0*q[2]);
  }
  else if(maxpos==3){
    q[3]=sqrt(tmp[3])*0.5;
    q[0]=(R[3] - R[1])/(4.0*q[3]);
    q[1]=(R[2] + R[6])/(4.0*q[3]);
    q[2]=(R[7] + R[5])/(4.0*q[3]);
  }
  else
  {
    std::cout<<"komisch"<<std::endl;
  }
  // enforce unit length
  mag=q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];

  if(!PMath::IsZero(mag-1.))
  {

    mag=1.0/sqrt(mag);
    q[0]*=mag; q[1]*=mag; q[2]*=mag; q[3]*=mag;
  }
}

inline void Rot2Vec3(double R[9], double d[3])
{
  double t[4];
  Rot2Quat(R,t);

  double mag=sqrt(PMath::Sqr(t[0]) + PMath::Sqr(t[1]) + PMath::Sqr(t[2]) + PMath::Sqr(t[3]));
  double sg=(t[0]>=0.0)? 1.0 : -1.0;
  mag=sg/mag;
  d[0] = t[1]*mag;
  d[1] = t[2]*mag;
  d[2] = t[3]*mag;
}

inline void Quat2Rot(double q[4], double R[9])
{
  // ensure unit length
  double mag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
  if(!PMath::IsZero(mag-1.0))
  {
    mag=1.0/sqrt(mag);
    q[0]*=mag; q[1]*=mag; q[2]*=mag; q[3]*=mag;
  }

  R[0]=q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3];
  R[1]=2*(q[1]*q[2]-q[0]*q[3]);
  R[2]=2*(q[1]*q[3]+q[0]*q[2]);

  R[3]=2*(q[1]*q[2]+q[0]*q[3]);
  R[4]=q[0]*q[0]+q[2]*q[2]-q[1]*q[1]-q[3]*q[3];
  R[5]=2*(q[2]*q[3]-q[0]*q[1]);

  R[6]=2*(q[1]*q[3]-q[0]*q[2]);
  R[7]=2*(q[2]*q[3]+q[0]*q[1]);
  R[8]=q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2];
}

inline void Vec32Rot(double d[3], double R[9])
{
  double q[4];
  q[0] = sqrt(1.0 - PMath::Sqr(d[0]) - PMath::Sqr(d[1])- PMath::Sqr(d[2]));
  q[1] = d[0];
  q[2] = d[1];
  q[3] = d[2];

  Quat2Rot(q,R);  
}

inline void InvPose(Pose &src, Pose &dst)
{
  //if (src.empty()) throw Except(__HERE__,"No pose available!");
  if (src.empty()) throw std::runtime_error ("Pose::InvPose: No pose available!");
  if (dst.empty()) dst.create();

  PMat::Transpose33(src.R.ptr<double>(0), dst.R.ptr<double>(0));
  PMat::Mul3(dst.R.ptr<double>(0),src.t.ptr<double>(0),dst.t.ptr<double>(0));
  PVec::Mul3(dst.t.ptr<double>(0),-1.,dst.t.ptr<double>(0));
}

inline void MulPose(Pose &in1, Pose &in2, Pose &out)
{
  cv::Mat pose1 = cv::Mat(4,4,CV_64F);
  cv::Mat pose2 = cv::Mat(4,4,CV_64F);
  cv::Mat pose3 = cv::Mat(4,4,CV_64F);

  in1.GetHom(pose1.ptr<double>(0));
  in2.GetHom(pose2.ptr<double>(0));

  pose3 = pose1*pose2;

  out.SetHom(pose3.ptr<double>(0));
}

template<typename T1,typename T2, typename T3>
inline void ProjectPoint2Image(T1 p[3], T2 C[9], T3 i[2])
{
  i[0] = C[0] * p[0]/p[2] + C[2];
  i[1] = C[4] * p[1]/p[2] + C[5];
}

template<typename T1,typename T2, typename T3, typename T4>
inline void ProjectPoint2Image(const T1 p[3], const T2 C[9], const T3 D[8], T4 i[2])
{
  double r2, r4, r6, a1, a2, a3, cdist, icdist2;
  double xd, yd;

  double z = p[2] ? 1./p[2] : 1;
  double x = p[0] * z; 
  double y = p[1] * z;

  r2 = x*x + y*y;
  r4 = r2*r2;
  r6 = r4*r2;
  a1 = 2*x*y;
  a2 = r2 + 2*x*x;
  a3 = r2 + 2*y*y;
  cdist = 1 + D[0]*r2 + D[1]*r4 + D[4]*r6;
  icdist2 = 1./(1 + D[5]*r2 + D[6]*r4 + D[7]*r6);
  xd = p[0]*cdist*icdist2 + D[2]*a1 + D[3]*a2;
  yd = p[1]*cdist*icdist2 + D[2]*a3 + D[3]*a1;

  i[0] = xd*C[0] + C[2];
  i[1] = yd*C[4] + C[5];
}

inline void ProjectPoint2Image(double pin[3], double R[9], double t[3], double C[9], double i[2])
{
  double ptmp[3];
  PMat::MulAdd3(R, pin, t, ptmp);
  ProjectPoint2Image(ptmp, C, i);
}


} //--THE END--

#endif

