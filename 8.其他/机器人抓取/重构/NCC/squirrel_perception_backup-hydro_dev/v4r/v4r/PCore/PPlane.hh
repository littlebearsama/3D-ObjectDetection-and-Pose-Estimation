/**
 * $Id$
 */

#ifndef P_PLANE_HH
#define P_PLANE_HH

#include <iostream>
#include <stdexcept>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "PMath.hh"




namespace P 
{

/**
 * some utils for surface modeling
 */
class Plane
{
private:


public:
  Plane(){};
  ~Plane(){};

  template<typename T1,typename T2, typename T3>
  static inline void Exp2Imp(T1 p1[3], T2 p2[3], T3 p3[3], T1 &a, T1 &b, T1 &c, T1 &d);
  template<typename T1,typename T2>
  static inline T1 ImpPointDist(T1 a, T1 b, T1 c, T1 d, T2 p[3]);
  template<typename T1,typename T2, typename T3>
  static inline T1 NormalPointDist(const T1 p[3], const T2 n[3], T3 pd[3]);
  template<typename T1,typename T2, typename T3>
  static inline void Exp2Normal(T1 p1[3], T2 p2[3], T3 p3[3], T1 n[3]);
};




/*********************** INLINE METHODES **************************/

template<typename T1,typename T2, typename T3>
inline void Plane::Exp2Imp(T1 p1[3], T2 p2[3], T3 p3[3], T1 &a, T1 &b, T1 &c, T1 &d)
{
  a = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
    - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  b = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
    - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  c = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
    - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  d = - p2[0] * a - p2[1] * b - p2[2] * c;
}

template<typename T1,typename T2>
inline T1 Plane::ImpPointDist(T1 a, T1 b, T1 c, T1 d, T2 p[3])
{
  return fabs ( a * p[0] + b * p[1] + c * p[2] + d ) /
         sqrt ( a * a + b * b + c * c );
}

template<typename T1,typename T2, typename T3>
inline T1 Plane::NormalPointDist(const T1 p[3], const T2 n[3], T3 pd[3])
{
  T1 t[3];

  t[0] = pd[0] - p[0];
  t[1] = pd[1] - p[1];
  t[2] = pd[2] - p[2];

  return t[0]*n[0] + t[1]*n[1] + t[2]*n[2];
}

template<typename T1,typename T2, typename T3>
inline void Plane::Exp2Normal(T1 p1[3], T2 p2[3], T3 p3[3], T1 n[3])
{
  T1 norm;

  n[0] = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
       - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  n[1] = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
       - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  n[2] = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
       - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  norm = sqrt ( PMath::Sqr(n[0]) + PMath::Sqr(n[1]) + PMath::Sqr(n[2] ) );

  if ( norm == 0.0 )
  {
    //throw std::runtime_error ("[Plane::Exp2Normal] Invalide plane!");
    n[0] = n[1] = n[2] = std::numeric_limits<float>::quiet_NaN();;
  }
  else
  {
    n[0] /= norm;
    n[1] /= norm;
    n[2] /= norm;
  }

}



}

#endif

