/**
 * $Id$
 * 
 * Simple matrix operations
 *
 * void Mul22(const T1 m[4], T2 s, T3 r[4]);
 * void Mul2(const T1 R[4], const T2 v[2], T3 r[2]);
 * void MulAdd2(const T1 R[4], const T2 v[2], const T3 t[2], T4 r[2]);
 *
 * void Mul33(const T1 m1[9], const T2 m2[9], T3 r[9]);
 * void Mul33(const T1 m[9], T2 s, T3 r[9]);
 * T1 Det33(const T1 m[9]);
 * bool Inv33(const T1 m[9], T2 r[9]);
 * void Transpose33(const T1 m[9], T2 r[9]);
 * void Mul3(const T1 R[9], const T2 v[3], T3 r[3]);
 * void MulAdd3( const T1 R[9], const T2 v[3], const T3 t[3], T4 r[3]);
 */

#ifndef PMAT_PMATRIX_HH
#define PMAT_PMATRIX_HH

#include "PVector.hh"

namespace PMat
{

template<typename T1,typename T2, typename T3>
extern void Mul22(const T1 m[4], T2 s, T3 r[4]);

template<typename T1,typename T2, typename T3>
extern void Mul2(const T1 R[4], const T2 v[2], T3 r[2]);

template<typename T1,typename T2, typename T3, typename T4>
extern void MulAdd2(const T1 R[4], const T2 v[2], const T3 t[2], T4 r[2]);



template<typename T1>
extern void Mul33(const T1 m1[9], const T1 m2[9], T1 r[9]);

template<typename T1>
extern void Mul33(const T1 m[9], T1 s, T1 r[9]);

template<typename T1>
extern T1 Det33(const T1 m[9]);

template<typename T1,typename T2>
extern bool Inv33(const T1 m[9], T2 r[9]);

template<typename T1,typename T2>
extern void Transpose33(const T1 m[9], T2 r[9]);

template<typename T1,typename T2, typename T3>
extern void Mul3(const T1 R[9], const T2 v[3], T3 r[3]);

template<typename T1,typename T2, typename T3, typename T4>
extern void MulAdd3( const T1 R[9], const T2 v[3], const T3 t[3], T4 r[3]);

}

#include "PMatrix.ic"

#endif

