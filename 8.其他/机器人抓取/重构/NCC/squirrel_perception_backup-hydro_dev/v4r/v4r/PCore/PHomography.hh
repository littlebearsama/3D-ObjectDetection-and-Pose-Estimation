/**
 * $Id$
 * Johann Prankl, 2010-03-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_PHOMOGRAPHY_HH
#define P_PHOMOGRAPHY_HH

#include <iostream>
#include <float.h>

#include "PVector.hh"
#include "PMatrix.hh"

namespace P 
{

class Hom 
{
private:

public:
  Hom(){};
  ~Hom(){};

  template<typename TN1, typename TN2, typename TN3>
  static bool ComputeHom(TN1 a1[2], TN1 a2[2], TN1 a3[2], TN1 a4[2], 
                         TN2 b1[2], TN2 b2[2], TN2 b3[2], TN2 b4[2],
                         TN3 H[9]);

  template<typename TN1, typename TN2, typename TN3>
  static bool ComputeAff(TN1 a1[2], TN1 a2[2], TN1 a3[2], 
                         TN2 b1[2], TN2 b2[2], TN2 b3[2], 
                         TN3 H[9]);

  template<typename TN1, typename TN2, typename TN3>
  static inline void MapPoint(TN1 in[2], TN2 H[9], TN3 out[2]);
};




/*************************** INLINE METHODES **********************************/
/*
 * Map a point using an (affine) homography
 * @param H pointer to a 3x3 homography matrix
 */
template<typename TN1, typename TN2, typename TN3>
inline void Hom::MapPoint(TN1 in[2], TN2 H[9], TN3 out[2])
{
  out[0] = H[0]*in[0] + H[1]*in[1] + H[2];
  out[1] = H[3]*in[0] + H[4]*in[1] + H[5];
  TN3 t = H[6]*in[0] + H[7]*in[1] + H[8];
  out[0] /= t;
  out[1] /= t;
}


/**
 * Compute affine mapping of three points
 */
template<typename TN1, typename TN2, typename TN3>
bool Hom::ComputeAff(TN1 a1[2], TN1 a2[2], TN1 a3[2], 
                             TN2 b1[2], TN2 b2[2], TN2 b3[2], 
                             TN3 H[9])
{
    TN3 S;

    S = a1[0]*(a3[1]-a2[1]) + 
        a2[0]*(a1[1]-a3[1]) +
        a3[0]*(a2[1]-a1[1]);

    if (PVec::IsZero(S)) return false;

      S=1/S;

      //calculation of the affine parameter
      H[0] = S * (a1[1]*(b2[0]-b3[0]) + a2[1]*(b3[0]-b1[0])+ a3[1]*(b1[0]-b2[0]));
      H[1] = S * (a1[0]*(b3[0]-b2[0]) + a2[0]*(b1[0]-b3[0])+ a3[0]*(b2[0]-b1[0]));
      H[3] = S * (a1[1]*(b2[1]-b3[1]) + a2[1]*(b3[1]-b1[1])+ a3[1]*(b1[1]-b2[1]));
      H[4] = S * (a1[0]*(b3[1]-b2[1]) + a2[0]*(b1[1]-b3[1])+ a3[0]*(b2[1]-b1[1]));
      H[2] = S * (a1[0]*(a3[1]*b2[0]-a2[1]*b3[0])+
                  a2[0]*(a1[1]*b3[0]-a3[1]*b1[0])+
                  a3[0]*(a2[1]*b1[0]-a1[1]*b2[0]));
      H[5] = S * (a1[0]*(a3[1]*b2[1]-a2[1]*b3[1])+
                  a2[0]*(a1[1]*b3[1]-a3[1]*b1[1])+
                  a3[0]*(a2[1]*b1[1]-a1[1]*b2[1]));
      H[6] = 0;
      H[7] = 0;
      H[8] = 1;

  return true;
}


/**
 * Compute projective mapping of four points
 */
template<typename TN1, typename TN2, typename TN3>
bool Hom::ComputeHom(TN1 a1[2], TN1 a2[2], TN1 a3[2], TN1 a4[2],
                      TN2 b1[2], TN2 b2[2], TN2 b3[2], TN2 b4[2],
                      TN3 H[9])
{
  TN3 T1[9], invT[9];

  // compute mapping of four (a) points to a unit square
  TN3 s = (a2[0]-a3[0]) * (a4[1]-a3[1]) - (a4[0]-a3[0]) * (a2[1]-a3[1]);

  if (PVec::IsZero(s)) return false;

  T1[6] = ((a1[0]-a2[0]+a3[0]-a4[0])*(a4[1]-a3[1]) -
           (a1[1]-a2[1]+a3[1]-a4[1])*(a4[0]-a3[0])) / s;
  T1[7] = ((a1[1]-a2[1]+a3[1]-a4[1])*(a2[0]-a3[0]) -
           (a1[0]-a2[0]+a3[0]-a4[0])*(a2[1]-a3[1])) / s;
  T1[8] = 1.;

  T1[0] = a2[0]-a1[0] + T1[6]*a2[0];  T1[1] = a4[0]-a1[0] + T1[7]*a4[0];  T1[2] = a1[0];
  T1[3] = a2[1]-a1[1] + T1[6]*a2[1];  T1[4] = a4[1]-a1[1] + T1[7]*a4[1];  T1[5] = a1[1];

  if (!PMat::Inv33(T1, invT)) return false;

  // compute mapping of a unit square to four points
  s = (b2[0]-b3[0]) * (b4[1]-b3[1]) - (b4[0]-b3[0]) * (b2[1]-b3[1]);

  if (PVec::IsZero(s)) return false;

  T1[6] = ((b1[0]-b2[0]+b3[0]-b4[0])*(b4[1]-b3[1]) -
           (b1[1]-b2[1]+b3[1]-b4[1])*(b4[0]-b3[0])) / s;
  T1[7] = ((b1[1]-b2[1]+b3[1]-b4[1])*(b2[0]-b3[0]) -
           (b1[0]-b2[0]+b3[0]-b4[0])*(b2[1]-b3[1])) / s;
  T1[8] = 1.;

  T1[0] = b2[0]-b1[0] + T1[6]*b2[0];  T1[1] = b4[0]-b1[0] + T1[7]*b4[0];  T1[2] = b1[0];
  T1[3] = b2[1]-b1[1] + T1[6]*b2[1];  T1[4] = b4[1]-b1[1] + T1[7]*b4[1];  T1[5] = b1[1];

  // compute mapping four points (a) to four points (b)
  PMat::Mul33(T1,invT,H);

  if (PVec::IsZero(H[8]))
    return false;

  PMat::Mul33(H, 1./H[8], H);

  return true;
}









}

#endif

