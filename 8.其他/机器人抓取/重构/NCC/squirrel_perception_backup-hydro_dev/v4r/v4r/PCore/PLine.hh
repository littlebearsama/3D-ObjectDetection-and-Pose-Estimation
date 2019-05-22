/**
 * $Id$
 */

#ifndef P_PLINE2D_HH
#define P_PLINE2D_HH

#include <opencv2/core/core.hpp>
#include "PMath.hh"
#include "PVector.hh"

namespace P
{

class Line
{
public:
  Line(){}
  ~Line(){}

  static double DistPointToLine(const cv::Point2d &q, const cv::Point2d &p, const cv::Point2d &d);
  static double AbsDistPointToLine(const cv::Point2d &q, const cv::Point2d &p, const cv::Point2d &d);
  static cv::Point2d LineIntersection(const cv::Point2d &p1, const cv::Point2d &d1, const cv::Point2d &p2, const cv::Point2d &d2);
  template<typename T1, typename T2>
  static T1 AbsDistPointToLine (const cv::Point_<T1> &p, const cv::Vec<T2,3> &l);
  template<typename T1, typename T2>
  static T1 DistPointToLine(const cv::Point_<T1> &p, const cv::Vec<T2,3> &l);
};


/************************ INLINE METHODES ***************************/

/**
 * Distance from a point to a line
 */
inline double Line::DistPointToLine(const cv::Point2d &q, const cv::Point2d &p, const cv::Point2d &d)
{
  double p_to_q[2];
  PVec::Sub2(&q.x, &p.x, p_to_q);

  return PVec::Cross2(p_to_q, &d.x);
}

/**
 * Absolut distance from a point to a line
 */
inline double Line::AbsDistPointToLine(const cv::Point2d &q, const cv::Point2d &p, const cv::Point2d &d)
{
  return fabs(DistPointToLine(q,p,d));
}

/**
 * Absolut distance from a point to a line (impl.)
 */
template<typename T1, typename T2>
inline T1 Line::AbsDistPointToLine(const cv::Point_<T1> &p, const cv::Vec<T2,3> &l)
{
  if ( l[0] * l[0] + l[1] * l[1] == 0.0 )
    throw std::runtime_error ("Corrupted line parameter!");
  
  return ( fabs ( l[0] * p.x + l[1] * p.y + l[2] ) / sqrt ( l[0]*l[0] + l[1]*l[1] ) );
}

/**
 * distance from a point to a line (impl.)
 */
template<typename T1, typename T2>
inline T1 Line::DistPointToLine(const cv::Point_<T1> &p, const cv::Vec<T2,3> &l)
{
  if ( l[0] * l[0] + l[1] * l[1] == 0.0 )
    throw std::runtime_error ("Corrupted line parameter!");
  
  return - ( l[2]<0.?-1.:1. ) * ( l[0] * p.x + l[1] * p.y + l[2] ) / sqrt ( l[0]*l[0] + l[1]*l[1] );
}


/**
 * Returns intersection of lines defined by point p and direction d (needs not
 * be a unit vector).
 */
inline cv::Point2d Line::LineIntersection(const cv::Point2d &p1, const cv::Point2d &d1, const cv::Point2d &p2, const cv::Point2d &d2)
{
  double d = PVec::Cross2(&d2.x, &d1.x);
  if(d == 0.)
    throw std::runtime_error ("Line::LineIntersection: lines do not intersect!");
  cv::Point2d p1_to_p2 = p2-p1;
  double l = PVec::Cross2(&d2.x, &p1_to_p2.x)/d;
  return cv::Point2d(p1.x + l*d1.x, p1.y + l*d1.y);
}





}


#endif

