/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_MATRIX_HPP
#define KP_MATRIX_HPP

#include "Vector.hpp"


namespace kp 
{

/**
 * mul22
 */
template<typename T1,typename T2, typename T3>
inline void mul22(const T1 m[4], T2 s, T3 r[4])
{
  r[0] = m[0]*s, r[1] = m[1]*s;
  r[2] = m[2]*s, r[3] = m[3]*s;
}

/**
 * mul2
 */
template<typename T1,typename T2, typename T3>
inline void mul2(const T1 R[4], const T2 v[2], T3 r[2])
{
  r[0] = R[0]*v[0] + R[1]*v[1];
  r[1] = R[2]*v[0] + R[3]*v[1];
}

/**
 * mulAdd2
 */
template<typename T1,typename T2, typename T3, typename T4>
inline void mulAdd2( const T1 R[4], const T2 v[2], const T3 t[2], T4 r[2])
{
  mul2(R,v,r);
  add2(r,t,r);
}

template<typename T1, typename T2, typename T3>
inline void add33(const T1 m1[9], const T2 m2[9], T3 r[9])
{
  r[0] = m1[0]+m2[0]; r[1] = m1[1]+m2[1]; r[2] = m1[2]+m2[2];
  r[3] = m1[3]+m2[3]; r[4] = m1[4]+m2[4]; r[5] = m1[5]+m2[5];
  r[6] = m1[6]+m2[6]; r[7] = m1[7]+m2[7]; r[8] = m1[8]+m2[8];
}

/**
 * Matrix multiplication 3x3
 */
template<typename T1, typename T2, typename T3>
inline void mul33m(const T1 m1[9], const T2 m2[9], T3 r[9])
{
  r[0] = m1[0]*m2[0] + m1[1]*m2[3] + m1[2]*m2[6];
  r[1] = m1[0]*m2[1] + m1[1]*m2[4] + m1[2]*m2[7];
  r[2] = m1[0]*m2[2] + m1[1]*m2[5] + m1[2]*m2[8];

  r[3] = m1[3]*m2[0] + m1[4]*m2[3] + m1[5]*m2[6];
  r[4] = m1[3]*m2[1] + m1[4]*m2[4] + m1[5]*m2[7];
  r[5] = m1[3]*m2[2] + m1[4]*m2[5] + m1[5]*m2[8];

  r[6] = m1[6]*m2[0] + m1[7]*m2[3] + m1[8]*m2[6];
  r[7] = m1[6]*m2[1] + m1[7]*m2[4] + m1[8]*m2[7];
  r[8] = m1[6]*m2[2] + m1[7]*m2[5] + m1[8]*m2[8];
}

/**
 * mul33
 */
template<typename T1, typename T2, typename T3>
inline void mul33v(const T1 v1[3], const T2 v2[3], T3 r[9])
{
  r[0]=v1[0]*v2[0]; r[1]=v1[0]*v2[1]; r[2]=v1[0]*v2[2];
  r[3]=v1[1]*v2[0]; r[4]=v1[1]*v2[1]; r[5]=v1[1]*v2[2];
  r[6]=v1[2]*v2[0]; r[7]=v1[2]*v2[1]; r[8]=v1[2]*v2[2];
}

/**
 * Multiplication of a 3x3 matrix with a scalar
 */
template<typename T1, typename T2, typename T3>
inline void mul33(const T1 m[9], T2 s, T3 r[9])
{
  r[0] = m[0]*s; r[1] = m[1]*s; r[2] = m[2]*s;
  r[3] = m[3]*s; r[4] = m[4]*s; r[5] = m[5]*s;
  r[6] = m[6]*s; r[7] = m[7]*s; r[8] = m[8]*s;
}

/**
 * Determinant of a 3x3 matrix
 */
template<typename T1>
inline T1 det33(const T1 m[9])
{
  return m[0]*m[4]*m[8] + m[1]*m[5]*m[6] + m[2]*m[3]*m[7]
        -m[0]*m[5]*m[7] - m[1]*m[3]*m[8] - m[2]*m[4]*m[6];
}

/**
 * inv33
 */
template<typename T1,typename T2>
inline bool inv33(const T1 m[9], T2 r[9])
{
  r[0] = m[4]*m[8] - m[5]*m[7];
  r[1] = m[2]*m[7] - m[1]*m[8];
  r[2] = m[1]*m[5] - m[2]*m[4];

  r[3] = m[5]*m[6] - m[3]*m[8];
  r[4] = m[0]*m[8] - m[2]*m[6];
  r[5] = m[2]*m[3] - m[0]*m[5];

  r[6] = m[3]*m[7] - m[4]*m[6];
  r[7] = m[1]*m[6] - m[0]*m[7];
  r[8] = m[0]*m[4] - m[1]*m[3];

  double det = det33(r);

  if (isZero(det)) return false;

  mul33(r, 1./det, r);

  return true;
}

/**
 * transpose33
 */
template<typename T1,typename T2>
inline void transpose33(const T1 m[9], T2 r[9])
{
  r[0] = m[0]; r[1] = m[3]; r[2] = m[6];
  r[3] = m[1]; r[4] = m[4]; r[5] = m[7];
  r[6] = m[2]; r[7] = m[5]; r[8] = m[8];
}

/**
 * mul3
 */
template<typename T1,typename T2, typename T3>
inline void mul3(const T1 R[9], const T2 v[3], T3 r[3])
{
  r[0] = R[0]*v[0] + R[1]*v[1] + R[2]*v[2];
  r[1] = R[3]*v[0] + R[4]*v[1] + R[5]*v[2];
  r[2] = R[6]*v[0] + R[7]*v[1] + R[8]*v[2];
}

/**
 * mulAdd3
 */
template<typename T1,typename T2, typename T3, typename T4>
inline void mulAdd3( const T1 R[9], const T2 v[3], const T3 t[3], T4 r[3])
{
  mul3(R,v,r);
  add3(r,t,r);
}



} //--END--

#endif




