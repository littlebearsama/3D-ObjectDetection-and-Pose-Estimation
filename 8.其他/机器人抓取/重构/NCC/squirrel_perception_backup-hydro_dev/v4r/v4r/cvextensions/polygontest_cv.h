/**
 * @file polygontest_cv.h
 * @author Markus Bader
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef POLYGONTEST_CV_H
#define POLYGONTEST_CV_H

#include <opencv/cv.h>
namespace cv
{

/**
 * Checks whether point p is inside closure.
 * Uses the Jordan curve theorem.
 * Note that points on the boundary are undefined.
 * Code thanks to
 *  W Randolph Franklin (WRF)
 * @see http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
 */
template<typename _T1, typename _T2>
inline bool isInsideInPolygon(const std::vector< Point_<_T1> > &hull, Point_<_T2> p){
    unsigned int i, j;
    bool c = false;
    for (i = 0, j = hull.size()-1; i < hull.size(); j = i++) {
        if ( ((hull[i].y > p.y) != (hull[j].y > p.y)) && (p.x < (hull[j].x - hull[i].x) * (p.y - hull[i].y) / (hull[j].y - hull[i].y) + hull[i].x) ) {
            c = !c;
        }
    }
    return c;

}
};

#endif //POLYGONTEST_CV_H
