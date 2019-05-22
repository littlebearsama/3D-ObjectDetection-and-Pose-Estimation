/**
 * $Id$
 *
 * Michael Zillich, 2004-3-04
 * Johann Prankl, 2011-04-26
 */

#ifndef PMATH_HH
#define PMATH_HH

#include <limits.h>
#include <time.h>
#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>

namespace PMath
{

const unsigned UNDEF_ID = UINT_MAX;

extern float FRand();
extern float ExpDev(float lambda);
extern int ExpSelect(int max);

/// Returns true if the value is near zero (+/- epsilon)
extern bool IsZero(double d);
extern bool IsZero(float d);

/// Returns true if the values are equal (+/- epsilon)
extern bool IsEqual(double a, double b);
extern bool IsEqual(float a, float b);

extern int SRound(double d);
template <class T1>
extern int Round(T1 d);

/// Square of given number
template <class Num>
extern Num Sqr(Num x);

template <class Num>
extern Num Max(Num a, Num b);
template <class Num>
extern Num Min(Num a, Num b);

template <class Num>
extern int Sign(Num x);

template <class T>
inline void Swap(T &a, T &b);

template <class Num>
inline bool Between(Num x, Num l, Num u);
template <class Num>
inline bool BetweenEq(Num x, Num l, Num u);

/// Scale angle to [0..2pi[
extern double ScaleAngle_0_2pi(double a);
/// Scale angle to [-pi..pi[
extern double ScaleAngle_mpi_pi(double a);
/// Scale angle to [0..pi[
extern double ScaleAngle_0_pi(double a);
/// Difference of two angles b - a. The result is scaled to [-pi..pi[
extern double DiffAngle_mpi_pi(double b, double a);
/// Difference of two angles b - a. The result is scaled to [0..2pi[
extern double DiffAngle_0_2pi(double b, double a);
/// Angles between (undirected) lines. The result is scaled to [0..pi/2[
extern double AngleBetweenLines(double b, double a);
/// Scale an integer angle to [0..8[
extern int ScaleIntAngle_0_8(int a);

extern int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y);
extern double timespec_diff(struct timespec *x, struct timespec *y);

}

#include "PMath.ic"

#endif

