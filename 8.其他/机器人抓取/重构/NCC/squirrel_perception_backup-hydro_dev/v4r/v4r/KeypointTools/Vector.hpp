/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_VECTOR_HPP
#define KP_VECTOR_HPP

#include <limits>

namespace kp 
{

/**
 * isZero
 */
template<typename T>
inline bool isZero(T d)
{
  return fabs(d) <= std::numeric_limits<T>::epsilon();
}

/**
 * sqr
 */
template<typename T>
inline T sqr(T x)
{
  return x*x;
}

/**
 * normalClockwise
 */
template<typename T1,typename T2>
inline void normalClockwise(const T1 v[2], T2 r[2])
{
  r[0] = v[1];
  r[1] = -v[0];
}

/**
 * normalAntiClockwise
 */
template<typename T1,typename T2>
inline void normalAntiClockwise(const T1 v[2], T2 r[2])
{
  r[0] = -v[1];
  r[1] = v[0];
}

/**
 * add2
 */
template<typename T1,typename T2, typename T3>
inline void add2(const T1 v1[2], const T2 v2[2], T3 r[2])
{
  r[0] = v1[0]+v2[0];
  r[1] = v1[1]+v2[1];
}

/**
 * sub2
 */
template<typename T1,typename T2, typename T3>
inline void sub2(const T1 v1[2], const T2 v2[2], T3 r[2])
{
  r[0] = v1[0]-v2[0];
  r[1] = v1[1]-v2[1];
}

/**
 * dot2
 */
template<typename T1,typename T2>
inline T1 dot2(const T1 a[2], const T2 b[2])
{
  return a[0]*b[0] + a[1]*b[1];
}

/**
 * cross2
 */
template<typename T1,typename T2>
inline T1 cross2(const T1 a[2], const T2 b[2])
{
  return a[0]*b[1] - a[1]*b[0];
}

/**
 * distance2
 */
template<typename T1,typename T2>
inline T1 distance2(const T1 d1[2], const T2 d2[2])
{
  return sqrt(sqr(d1[0]-d2[0])+sqr(d1[1]-d2[1]));
}

/**
 * squaredDistance2
 */
template<typename T1,typename T2>
inline T1 squaredDistance2(const T1 d1[2], const T2 d2[2])
{
  return sqr(d1[0]-d2[0])+sqr(d1[1]-d2[1]);
}

/**
 * norm2
 */
template<typename T1>
inline T1 norm2(const T1 v[2])
{
  return sqrt(sqr(v[0]) + sqr(v[1]));
}

/**
 * squaredNorm2
 */
template<typename T1>
inline T1 squaredNorm2(const T1 v[2])
{
  return sqr(v[0]) + sqr(v[1]);
}

/**
 * mul2
 */
template<typename T1,typename T2, typename T3>
inline void mul2(const T1 v[2], T2 s, T3 r[2])
{
  r[0] = v[0]*s;
  r[1] = v[1]*s;
}

/**
 * div2
 */
template<typename T1,typename T2, typename T3>
inline void div2(const T1 v[2], T2 s, T3 r[2])
{
  s = 1./s;
  r[0] = v[0]*s;
  r[1] = v[1]*s;
}

/**
 * normalise2
 */
template<typename T1,typename T2>
inline void normalise2(const T1 s[2], T2 d[2])
{
  mul2(s,1./norm2(s),d);
}

/**
 * polarAngle
 */
template<typename T1>
inline T1 polarAngle(const T1 v[2])
{
  return atan2(v[1], v[0]);
}

/**
 * rotate2
 */
template<typename T1,typename T2, typename T3>
inline void rotate2(const T1 v[2], const T2 phi, T3 r[2])
{
  T1 si = sin(phi), co = cos(phi);
  r[0] = co*v[0] - si*v[1], r[1] = si*v[0] + co*v[1];
}

/**
 * midPoint2
 */
template<typename T1,typename T2, typename T3>
inline void midPoint2(const T1 a[2], const T2 b[2], T3 r[2])
{
  r[0] = (a[0]+b[0]) / 2.;
  r[1] = (a[1]+b[1]) / 2.;
}

/**
 * angleBetween2
 */
template<typename T1,typename T2>
inline T1 angleBetween2(const T1 d1[2], const T2 d2[2])
{
  T1 cosalpha = dot2(d1,d2)/(norm2(d1)*norm2(d2));
  return (fabs(1-cosalpha) <= std::numeric_limits<T1>::epsilon() ?0:acos(cosalpha));
}

/**
 * mul3
 */
template<typename T1,typename T2, typename T3>
inline void mul3(const T1 v[3], T2 s, T3 r[3])
{
  r[0] = v[0]*s;
  r[1] = v[1]*s;
  r[2] = v[2]*s;
}

/**
 * div3
 */
template<typename T1,typename T2, typename T3>
inline void div3(const T1 v[3], T2 s, T3 r[3])
{
  s = 1./s;
  r[0] = v[0]*s;
  r[1] = v[1]*s;
  r[2] = v[2]*s;
}

/**
 * sub3
 */
template<typename T1,typename T2, typename T3>
inline void sub3(const T1 v1[3], const T2 v2[3], T3 r[3])
{
  r[0] = v1[0]-v2[0];
  r[1] = v1[1]-v2[1];
  r[2] = v1[2]-v2[2];
}

/**
 * add3
 */
template<typename T1,typename T2, typename T3>
inline void add3(const T1 v1[3], const T2 v2[3], T3 r[3])
{
  r[0] = v1[0]+v2[0];
  r[1] = v1[1]+v2[1];
  r[2] = v1[2]+v2[2];
}

/**
 * normalise3
 */
template<typename T1, typename T2>
inline void normalise3(const T1 v[3], T2 r[3])
{
  mul3(v,1./norm3(v),r);
}

/**
 *
 */
template<typename T1>
inline T1 norm3(const T1 v[3])
{
  return sqrt(sqr(v[0]) + sqr(v[1]) + sqr(v[2]));
}

/**
 * squaredNorm3
 */
template<typename T1>
inline T1 squaredNorm3(const T1 v[3])
{
  return sqr(v[0]) + sqr(v[1]) + sqr(v[2]);
}

/**
 * dot3
 */
template<typename T1,typename T2>
inline T1 dot3(const T1 v1[3], const T2 v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

/**
 * distance3
 */
template<typename T1,typename T2>
inline T1 distance3(const T1 d1[3], const T2 d2[3])
{
  return sqrt(sqr(d1[0]-d2[0])+sqr(d1[1]-d2[1])+sqr(d1[2]-d2[2]));
}

/**
 * squaredDistance3
 */
template<typename T1,typename T2>
inline T1 squaredDistance3(const T1 d1[3], const T2 d2[3])
{
  return sqr(d1[0]-d2[0])+sqr(d1[1]-d2[1])+sqr(d1[2]-d2[2]);
}

/**
 * angleBetween3
 */
template<typename T1,typename T2>
inline T1 angleBetween3(const T1 d1[3], const T2 d2[3])
{
  double cosalpha = dot3(d1,d2)/(norm3(d1)*norm3(d2));
  return (fabs(1-cosalpha)<=std::numeric_limits<T1>::epsilon()?0:acos(cosalpha));
}

/**
 * addN
 */
template<typename T1,typename T2, typename T3>
inline void add(const T1 *d1, const T2 *d2, T3 *r, unsigned size)
{
  for (unsigned i=0; i<size; i++)
    r[i] = d1[i]+d2[i];
}

/**
 * mulN
 */
template<typename T1,typename T2, typename T3>
inline void mul(const T1 *d1, const T2 *d2, T3 *r, unsigned size)
{
  for (unsigned i=0; i<size; i++)
    r[i] = d1[i]*d2[i];
}

/**
 * copyN
 */
template<typename T1,typename T2>
inline void copy(const T1 *d, T2 *r, unsigned size)
{
  for (unsigned i=0; i<size; i++)
    r[i] = d[i];
}

/**
 * squaredDistance
 */
template<typename T1,typename T2>
inline T1 squaredDistance(const T1 *d1, const T2 *d2, unsigned size)
{
  T1 dist=0;
  for (unsigned i=0; i<size; i++)
    dist += sqr(d1[i]-d2[i]);
  return dist;
}

/**
 * distanceL1
 */
template<typename T1,typename T2>
inline T1 distanceL1(const T1 *d1, const T2 *d2, unsigned size)
{
  T1 dist=0;
  for (unsigned i=0; i<size; i++)
    dist += fabs(d1[i]-d2[i]);
  return dist;
}

/**
 * distanceL1b
 */
inline int distanceL1b(const unsigned char *d1, const unsigned char *d2, unsigned size)
{
  int dist=0;
  for (unsigned i=0; i<size; i++)
    dist += abs(d1[i]-d2[i]);
  return dist;
}

/**
 * distanceL1b
 */
inline float distanceNCCb(const unsigned char *d1, const unsigned char *d2, unsigned size)
{
  float dist=0, m1=0, m2=0, s1=0, s2=0;

  for (unsigned i=0; i<size; i++)
  {
    m1 += float(d1[i]);
    m2 += float(d2[i]);
  }
  m1 /= float(size);
  m2 /= float(size);
  for (unsigned i=0; i<size; i++)
  {
    s1 += sqr(float(d1[i])-m1);
    s2 += sqr(float(d2[i])-m2);
  }
  s1 /= float(size-1);
  s2 /= float(size-1);

  for (unsigned i=0; i<size; i++)
    dist += (float(d1[i])-m1)*(float(d2[i])-m2);

  dist /= float(size)*sqrt(s1)*sqrt(s2);

  return dist;
}





} //--END--

#endif




