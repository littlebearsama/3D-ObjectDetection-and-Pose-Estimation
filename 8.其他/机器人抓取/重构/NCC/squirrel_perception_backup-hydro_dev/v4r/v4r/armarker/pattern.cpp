/*
Copyright (C) 2010 Markus Bader

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include "armarker.h"
#include "AR/ar.h"
#include "v4r/cvextensions/print_cv.h"

using namespace V4R;


int MarkerPattern::getCornersAbs(std::vector<cv::Point3d> &points, bool clearFirst) {
    if (clearFirst) points.clear();
    cv::Mat_<double> RT = cv::Mat_<double>::eye(4,4);
    RT(0,3) = mPose[0];
    RT(1,3) = mPose[1];
    RT(2,3) = mPose[2];
    cv::Mat_<double> Rc(RT, cv::Rect(0, 0, 3, 3));
    cv::Rodrigues(cv::Mat_<double>(3,1, &mPose[3]), Rc);
    std::vector<cv::Point3d> cr;
    getCornersRel(cr);
    cv::Mat_<double> p = (cv::Mat_<double>(4,4) <<
                          cr[0].x , cr[1].x, cr[2].x, cr[3].x,
                          cr[0].y , cr[1].y, cr[2].y, cr[3].y,
                                0.,      0.,      0.,      0.,
                                1.,      1.,      1.,      1.);
    cv::Mat_<double> pw = RT *p;
    points.push_back(cv::Point3d(pw(0,0), pw(1,0), pw(2,0)));
    points.push_back(cv::Point3d(pw(0,1), pw(1,1), pw(2,1)));
    points.push_back(cv::Point3d(pw(0,2), pw(1,2), pw(2,2)));
    points.push_back(cv::Point3d(pw(0,3), pw(1,3), pw(2,3)));
    return 0;
}

void MarkerPattern::getCornersRel(std::vector<cv::Point3d> &points, bool clearFirst) {
    if (clearFirst) points.clear();
    double rw = mSize.width/2.;
    double rh = mSize.height/2.;
    points.push_back(cv::Point3d(+rw, +rh, 0.));
    points.push_back(cv::Point3d(+rw, -rh, 0.));
    points.push_back(cv::Point3d(-rw, -rh, 0.));
    points.push_back(cv::Point3d(-rw, +rh, 0.));
}

const std::vector<MarkerPattern> MarkerPattern::loadList(const std::string &patternList) {
    std::vector<MarkerPattern> patternVector;
    cv::FileStorage fs(patternList, cv::FileStorage::READ);
    cv::FileNode fn = fs.root();
    for (cv::FileNodeIterator it = fn["marker"].begin(); it != fn["marker"].end(); ++it) {
        std::string name = ((std::string)*it);
        cv::FileNode fnMaker = fs[name];
        std::string file = fnMaker["file"];
        cv::FileNodeIterator itSize = fnMaker["size"].begin();
        cv::Size_<double> size;
        size.width = (double)*itSize++;
        size.height = (double)*itSize++;
        cv::Vec<double, 6> pose;
        cv::FileNodeIterator itPose = fnMaker["pose"].begin();
        pose[0] = (double)*itPose++;
        pose[1] = (double)*itPose++;
        pose[2] = (double)*itPose++;
        pose[3] = (double)*itPose++;
        pose[4] = (double)*itPose++;
        pose[5] = (double)*itPose++;
        std::string group = fnMaker["group"];
        if(group.empty()) group = name;
        patternVector.push_back(MarkerPattern(name, file, size, pose, group));
    }
    fs.release();
    return patternVector;
}



// kate: indent-mode cstyle; space-indent on; indent-width 0;
