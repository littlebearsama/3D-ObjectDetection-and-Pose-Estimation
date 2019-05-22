/**
 * $Id$
 *
 * Simple vector operations
 *
 * T1 PolarAngle(const T1 v[2]);
 * void Rotate2(const T1 v[2], const T2 phi, T3 r[2]);
 * T1 Distance2(const T1 d1[2], const T2 d2[2]);
 * T1 DistanceSqr2(const T1 d1[2], const T2 d2[2]);
 * void Normalise2(const T1 s[2], T2 d[2]);
 * void Add2(const T1 v1[2], const T2 v2[2], T3 r[2]);
 * void Sub2(const T1 v1[2], const T2 v2[2], T3 r[2]);
 * void Mul2(const T1 v[2], T2 s, T3 r[2]);
 * void Div2(const T1 v[2], T2 s, T3 r[2]);
 * T1 Norm2(const T1 v[2]);
 * T1 Dot2(const T1 a[2], const T2 b[2]);
 * T1 Cross2(const T1 a[2], const T2 b[2]);
 * void NormalClockwise(const T1 v[2], T2 r[2]);
 * void NormalAntiClockwise(const T1 v[2], T2 r[2]);
 * void MidPoint2(const T1 a[2], const T2 b[2], T3 r[2]);
 * T1 AngleBetween2(const T1 d1[2], const T2 d2[2])
 *
 * T1 Norm3(const T1 v[3]);
 * void Normalise3(const T1 v[3], T2 r[3]);
 * void Mul3(const T1 v[3], T2 s, T3 r[3]);
 * void Div3(const T1 v[3], T2 s, T3 r[3]);
 * void Sub3(const T1 v1[3], const T2 v2[3], T3 r[3]);
 * void Add3(const T1 v1[3], const T2 v2[3], T3 r[3]);
 * T1 Dot3(const T1 v1[3], const T2 v2[3]);
 * T1 Distance3(const T1 d1[3], const T2 d2[3]);
 * T1 DistanceSqr3(const T1 d1[3], const T2 d2[3]);
 * T1 AngleBetween3(const T1 d1[3], const T2 d2[3]);
 *
 * void AddN(const T1 *d1, const T2 *d2, T3 *r);
 * void MulN(const T1 *d1, const T2 *d2, T3 *r, unsigned size);
 * void CopyN(const T1 *d, T2 *r, unsigned size);
 */

#ifndef PVEC_PVECTOR_HH
#define PVEC_PVECTOR_HH

#include <math.h>

namespace PVec
{

extern bool IsZero(double d);

template <typename T>
extern T Sqr(T x);

template<typename T1>
extern T1 PolarAngle(const T1 v[2]);

template<typename T1,typename T2, typename T3>
extern void Rotate2(const T1 v[2], const T2 phi, T3 r[2]);

template<typename T1,typename T2>
extern T1 Distance2(const T1 d1[2], const T2 d2[2]);

template<typename T1,typename T2>
extern T1 DistanceSqr2(const T1 d1[2], const T2 d2[2]);

template<typename T1,typename T2>
extern void Normalise2(const T1 s[2], T2 d[2]);

template<typename T1,typename T2, typename T3>
extern void Add2(const T1 v1[2], const T2 v2[2], T3 r[2]);

template<typename T1,typename T2, typename T3>
extern void Sub2(const T1 v1[2], const T2 v2[2], T3 r[2]);

template<typename T1,typename T2, typename T3>
extern void Mul2(const T1 v[2], T2 s, T3 r[2]);

template<typename T1,typename T2, typename T3>
extern void Div2(const T1 v[2], T2 s, T3 r[2]);

template<typename T1>
extern T1 Norm2(const T1 v[2]);

template<typename T1,typename T2>
extern T1 Dot2(const T1 a[2], const T2 b[2]);

template<typename T1,typename T2>
extern T1 Cross2(const T1 a[2], const T2 b[2]);

template<typename T1,typename T2>
extern void NormalClockwise(const T1 v[2], T2 r[2]);

template<typename T1,typename T2>
extern void NormalAntiClockwise(const T1 v[2], T2 r[2]);

template<typename T1,typename T2, typename T3>
extern void MidPoint2(const T1 a[2], const T2 b[2], T3 r[2]);

template<typename T1,typename T2>
extern T1 AngleBetween2(const T1 d1[2], const T2 d2[2]);

template<typename T1>
extern T1 Norm3(const T1 v[3]);

template<typename T1, typename T2>
extern void Normalise3(const T1 v[3], T2 r[3]);

template<typename T1,typename T2, typename T3>
extern void Mul3(const T1 v[3], T2 s, T3 r[3]);

template<typename T1,typename T2, typename T3>
extern void Div3(const T1 v[3], T2 s, T3 r[3]);

template<typename T1,typename T2, typename T3>
extern void Sub3(const T1 v1[3], const T2 v2[3], T3 r[3]);

template<typename T1,typename T2, typename T3>
extern void Add3(const T1 v1[3], const T2 v2[3], T3 r[3]);

template<typename T1,typename T2>
extern T1 Dot3(const T1 v1[3], const T2 v2[3]);

template<typename T1,typename T2>
extern T1 Distance3(const T1 d1[3], const T2 d2[3]);

template<typename T1,typename T2>
extern T1 DistanceSqr3(const T1 d1[3], const T2 d2[3]);

template<typename T1,typename T2>
extern T1 AngleBetween3(const T1 d1[3], const T2 d2[3]);


template<typename T1,typename T2, typename T3>
extern void AddN(const T1 *d1, const T2 *d2, T3 *r, unsigned size);

template<typename T1,typename T2, typename T3>
extern void MulN(const T1 *d1, const T2 *d2, T3 *r, unsigned size);

template<typename T1,typename T2>
extern void CopyN(const T1 *d, T2 *r, unsigned size);

}

#include "PVector.ic"

#endif

