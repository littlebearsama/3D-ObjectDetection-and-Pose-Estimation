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
#include "v4r/cvextensions/print_cv.h"
#include "AR/ar.h"
#include <set>
#include <wordexp.h>
using namespace V4R;

std::string expandName ( const std::string &fileString ) {
  std::stringstream ss;
    wordexp_t p;
    char** w;
    wordexp( fileString.c_str(), &p, 0 );
    w = p.we_wordv;
    for (size_t i=0; i < p.we_wordc; i++ ) {
      ss << w[i];
    }
    wordfree( &p );
    return ss.str();
}

MarkerDetection::MarkerDetection ( )
        : mpARCParam ( NULL )
        , mFrameCount ( 0 ) {
}
MarkerDetection::~MarkerDetection() {
    if ( mpARCParam != NULL ) delete ( ( ARParam* ) mpARCParam );
}

const std::vector<Marker>  &MarkerDetection::detectMarker ( const cv::Mat &img, int thresh ) {
    if ( mpARCParam == NULL )  init(img.cols, img.rows);

    ARUint8         *pSrc;
    ARMarkerInfo    *marker_info;
    int             marker_num;
    MarkerHdl       markerHdl;
    pSrc = ( ARUint8* ) img.data;
    mMarker. clear();
    if ( arDetectMarker ( pSrc, thresh, &marker_info, &marker_num ) < 0 ) {
        exit ( 0 );
    }
    mMarker.resize ( marker_num );
    for ( int i = 0; i < marker_num; i++ ) {
        markerHdl.set(&mMarker[i]);
        markerHdl.setARMarkerInfo (&marker_info[i] );
    }
    mFrameCount++;
    return mMarker;
}

void MarkerDetection::init ( int imgWidht, int imgHeight ) {
    if ( mpARCParam == NULL ) {
        mpARCParam = new ARParam;
    }
    ARParam *pCParam = ( ARParam* ) mpARCParam;
    memset ( pCParam, 0, sizeof ( ARParam ) );
    pCParam->dist_factor[3] = 1;
    pCParam->xsize = imgWidht;
    pCParam->ysize = imgHeight;
    arInitCparam ( pCParam );
    arUtilTimerReset();
    mFrameCount = 0;

}

int MarkerDetection::loadMarker ( const std::string &rFile, double size ) {
    std::string file = expandName(rFile);
    int patt_id;
    MarkerPattern pattern (file, file, cv::Size_<double>(size,size), cv::Vec6d());
    if ( ( patt_id=arLoadPatt ( pattern.patternFile().c_str() ) ) < 0 ) {
            std::cerr << "pattern load error: " << file << std::endl;
        return -1;
    }
    if (mMarkerPatterns.size() != (unsigned int) patt_id) return -1;
    mMarkerPatterns.push_back(pattern);
    return patt_id;
}

int MarkerDetection::loadMarkerList(const std::string &rFile) {
    std::string file = expandName(rFile);
    mMarkerPatterns = MarkerPattern::loadList(file);
    for (int i = 0; i < (int) mMarkerPatterns.size(); i++) {
      std::string patternFile = expandName(mMarkerPatterns[i].patternFile());
        int patt_id = arLoadPatt ( patternFile.c_str() );
        if ((patt_id < 0 ) || (patt_id != i)) {
            std::cerr << "pattern load error: " << patternFile << std::endl;
            return -1;
        }
    }
    return 0;
}
const std::vector<MarkerPattern> &MarkerDetection::getPatterns() const {
    return mMarkerPatterns;
}

const std::vector<V4R::Marker>  &MarkerDetection::computeMarkerLocation (const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs ) {
    if ((cameraMatrix.rows != 3) ||  (cameraMatrix.cols != 3)) {
        CV_Error( CV_StsUnsupportedFormat, "cameraMatrix must be 3x3" );
    }
    for (unsigned int i = 0; i < mMarker.size(); i++) {
        Marker &marker = mMarker[i];
        MarkerPattern &pattern = mMarkerPatterns[marker.id];
        if (marker.id < 0) {
            // Thats not a known marker
            marker.poseR[0] = 0, marker.poseR[1] = 0, marker.poseR[2] = 0;
            marker.poseT[0] = 0, marker.poseT[1] = 0, marker.poseT[2] = 0;
        } else {
            std::vector<cv::Point3d> corners;
            pattern.getCornersRel(corners);
            CvMat objPoints = cvMat( 4, 1, CV_64FC3, (double*) &corners.front() );
            CvMat imgPoints = cvMat( 4, 1, CV_64FC2, (double*) marker.vertex );
            CvMat rotation = cvMat( 3,1, CV_64F, (double*) marker.poseR );
            CvMat translation = cvMat( 3,1, CV_64F, (double*) marker.poseT );
            CvMat _cameraMatrix = cameraMatrix, _distCoeffs = distCoeffs;
            if (distCoeffs.empty()) {
                cvFindExtrinsicCameraParams2(&objPoints, &imgPoints, &_cameraMatrix,  NULL, &rotation, &translation, false );
            } else {
                cvFindExtrinsicCameraParams2(&objPoints, &imgPoints, &_cameraMatrix,  &_distCoeffs, &rotation, &translation,  false);
            }
        }
    }
    return mMarker;
}

int MarkerDetection::computePose(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Mat_<double> &rvec, cv::Mat_<double> &tvec, double &group_conficence, std::string group, double min_conficence) {
    if ((cameraMatrix.rows != 3) ||  (cameraMatrix.cols != 3)) {
        CV_Error( CV_StsUnsupportedFormat, "cameraMatrix must be 3x3" );
    }
    cv::vector<cv::Point3d> corners3D;
    corners3D.reserve(mMarker.size());
    cv::vector<cv::Point2d> corners2D;
    corners2D.reserve(mMarker.size());
    std::set< int> usedMarker;
    group_conficence = 0;
    for (unsigned int i = 0; i < mMarker.size(); i++ ) {
        V4R::Marker &marker = mMarker[i];
        V4R::MarkerPattern &pattern = mMarkerPatterns[mMarker[i].id];

        if (marker.id < 0) continue;
        if (marker.cf < min_conficence) continue;
        if ((group.empty()) || (group.compare(pattern.group()) != 0)) continue;

        if (usedMarker.find(marker.id) != usedMarker.end()) return 1;
        usedMarker.insert(marker.id);

        pattern.getCornersAbs(corners3D, false);
        marker.getCorners(corners2D, false);
        group_conficence += marker.cf;
    }
    if (corners3D.size() < 4) return 1;
    group_conficence = group_conficence/ (double) (corners3D.size()/4);
    CvMat objPoints = cvMat( corners3D.size(), 1, CV_64FC3, &corners3D.front());
    CvMat imgPoints = cvMat( corners2D.size(), 1, CV_64FC2, &corners2D.front());
    CvMat _cameraMatrix = cameraMatrix, _distCoeffs = distCoeffs;

    rvec.create(3, 1);
    tvec.create(3, 1);

    CvMat _rvec = rvec, _tvec = tvec;

    //V4R::print(&objPoints, "objPoints");
    //V4R::print(&imgPoints, "imgPoints");
    if (distCoeffs.empty()) {
        cvFindExtrinsicCameraParams2(&objPoints, &imgPoints, &_cameraMatrix, NULL, &_rvec, &_tvec, false);
    } else {
        cvFindExtrinsicCameraParams2(&objPoints, &imgPoints, &_cameraMatrix, &_distCoeffs, &_rvec, &_tvec, false);
    }
    /*
        cv::vector<cv::Point3f> objVec(corners3D.size());
        cv::vector<cv::Point2f> imgVec(corners2D.size());
        for (unsigned int i = 0; i < corners3D.size(); i++) {
            objVec[i].x = corners3D[i].x;
            objVec[i].y = corners3D[i].y;
            objVec[i].z = corners3D[i].z;
            imgVec[i].x = corners2D[i].x;
            imgVec[i].y = corners2D[i].y;
        }
        cv::Mat obj(corners3D.size(), 1, CV_32FC3, &objVec[0]);
        cv::Mat img(corners2D.size(), 1, CV_32FC2, &imgVec[0]);

        cv::solvePnP(obj, img, cameraMatrix, distCoeffs, rvec, tvec);
    */


    return 0;
}


const std::vector<V4R::Marker>  &MarkerDetection::subPixOptimization(const cv::Mat &img, const cv::Size winSize) {
    if(mMarker.size() == 0) return mMarker;
    std::vector<cv::Point_<float> > corners;
    corners.reserve(mMarker.size()*4);
    for (unsigned int i = 0; i < mMarker.size(); i++) {
        mMarker[i].getCorners(corners, false);
       
    }

    cv::Size zeroZone(-1,-1);
    cv::TermCriteria criteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03);
    cv::cornerSubPix(img, corners, winSize, zeroZone, criteria);

    cv::Point_<float> *pnt = &corners.front();
    for (unsigned int i = 0; i < mMarker.size(); i++) {
        for (unsigned int j = 0; j < 4; j++) {
            mMarker[i].vertex[j][0] = pnt->x,  mMarker[i].vertex[j][1] = pnt->y;
            pnt++;
        }
    }

    return mMarker;
}


// kate: indent-mode cstyle; space-indent on; indent-width 0;
