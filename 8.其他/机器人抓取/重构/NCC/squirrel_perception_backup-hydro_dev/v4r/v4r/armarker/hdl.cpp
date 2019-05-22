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

using namespace V4R;

MarkerHdl::MarkerHdl()
        : mpMarker(NULL)
        , mImg(cv::Mat()) {
}
MarkerHdl::MarkerHdl(Marker *pMarker)
        : mpMarker(pMarker)
        , mImg(cv::Mat()) {
}

void MarkerHdl::set(const std::vector< MarkerPattern > &patterns) {
    mPatterns = patterns;
}
void MarkerHdl::set(Marker *pMarker) {
    mpMarker = pMarker;
}
void MarkerHdl::set(IplImage *pImg) {
    mImg = cv::Mat(pImg);
}
void MarkerHdl::set(cv::Mat &img) {
    mImg = img;
}
std::string MarkerHdl::name() {
    if (mpMarker->id < 0) return "NA";
    if (mpMarker->id > (int) mPatterns.size()-1) return "NA";
    return mPatterns[mpMarker->id].name();
}
std::string MarkerHdl::group(){
    if (mpMarker->id < 0) return "NA";
    if (mpMarker->id > (int) mPatterns.size()-1) return "NA";
    return mPatterns[mpMarker->id].group();
}

Marker *MarkerHdl::get() {
    return mpMarker;
}
const Marker &MarkerHdl::operator()() {
    return *mpMarker;
}

void MarkerHdl::setARMarkerInfo(void* pARMarkerInfo) {
    ARMarkerInfo *pARMarker = ( ARMarkerInfo* ) pARMarkerInfo;
    mpMarker->area = pARMarker->area;
    mpMarker->id = pARMarker->id;
    mpMarker->cf = pARMarker->cf;
    mpMarker->pos[0] = pARMarker->pos[0];
    mpMarker->pos[1] = pARMarker->pos[1];
    mpMarker->line[0][0] = pARMarker->line[0][0], mpMarker->line[0][1] = pARMarker->line[0][1], mpMarker->line[0][2] = pARMarker->line[0][2];
    mpMarker->line[1][0] = pARMarker->line[1][0], mpMarker->line[1][1] = pARMarker->line[1][1], mpMarker->line[1][2] = pARMarker->line[1][2];
    mpMarker->line[2][0] = pARMarker->line[2][0], mpMarker->line[2][1] = pARMarker->line[2][1], mpMarker->line[2][2] = pARMarker->line[2][2];
    mpMarker->line[3][0] = pARMarker->line[3][0], mpMarker->line[3][1] = pARMarker->line[3][1], mpMarker->line[3][2] = pARMarker->line[3][2];

    int dir = pARMarker->dir;
    mpMarker->vertex[0][0] = pARMarker->vertex[(4 - dir) % 4][0];
    mpMarker->vertex[0][1] = pARMarker->vertex[(4 - dir) % 4][1];
    mpMarker->vertex[1][0] = pARMarker->vertex[(5 - dir) % 4][0];
    mpMarker->vertex[1][1] = pARMarker->vertex[(5 - dir) % 4][1];
    mpMarker->vertex[2][0] = pARMarker->vertex[(6 - dir) % 4][0];
    mpMarker->vertex[2][1] = pARMarker->vertex[(6 - dir) % 4][1];
    mpMarker->vertex[3][0] = pARMarker->vertex[(7 - dir) % 4][0];
    mpMarker->vertex[3][1] = pARMarker->vertex[(7 - dir) % 4][1];


    mpMarker->line[4][0] = mpMarker->vertex[0][1] - mpMarker->vertex[2][1];
    mpMarker->line[4][1] = mpMarker->vertex[2][0] - mpMarker->vertex[0][0];
    mpMarker->line[4][2] = mpMarker->vertex[0][0] * mpMarker->vertex[2][1] - mpMarker->vertex[0][1]*mpMarker->vertex[2][0];


    mpMarker->line[5][0] = mpMarker->vertex[1][1] - mpMarker->vertex[3][1];
    mpMarker->line[5][1] = mpMarker->vertex[3][0] - mpMarker->vertex[1][0];
    mpMarker->line[5][2] = mpMarker->vertex[1][0] * mpMarker->vertex[3][1] - mpMarker->vertex[1][1]*mpMarker->vertex[3][0];

    double cen[3];
    cen[0] = mpMarker->line[4][1] * mpMarker->line[5][2] - mpMarker->line[4][2] * mpMarker->line[5][1];
    cen[1] = mpMarker->line[4][2] * mpMarker->line[5][0] - mpMarker->line[4][0] * mpMarker->line[5][2];
    cen[2] = mpMarker->line[4][0] * mpMarker->line[5][1] - mpMarker->line[4][1] * mpMarker->line[5][0];
    mpMarker->pos[0] = cen[0] / cen[2];
    mpMarker->pos[1] = cen[1] / cen[2];
}

void MarkerHdl::drawCenter (const CvScalar &color, int size ) {
    if (mImg.empty()) return;
    cv::Point p( mpMarker->pos[0], mpMarker->pos[1] );
    if ( ( p.x + size >= mImg.cols ) || ( p.x - size < 0 ) ||
            ( p.y + size >= mImg.rows ) || ( p.y - size < 0 ) ) {
        return;
    }
    cv::Point pt1( p.x + size, p.y + size );
    cv::Point pt2( p.x - size, p.y - size );
    cv::Point pt3( p.x - size, p.y + size );
    cv::Point pt4( p.x + size, p.y - size );
    cv::line ( mImg, pt1, pt2, color );
    cv::line ( mImg, pt3, pt4, color );
}

void MarkerHdl::drawVertexId (const CvScalar &color, double scale, int fontFace ) {
    if (mImg.empty()) return;
    char pText[16];
    for ( int i = 0; i < 4; i++ ) {
        sprintf ( pText, "%i", i );
        cv::Point p( mpMarker->vertex[i][0], mpMarker->vertex[i][1] );
        if ( ( p.x >= mImg.cols ) || ( p.x < 0 ) ||
                ( p.y >= mImg.rows ) || ( p.y < 0 ) ) {
            return;
        }
        cv::putText ( mImg, pText, p, fontFace, scale, color );
    }
}

void MarkerHdl::drawVertex (const CvScalar &color, int size ) {
    if (mImg.empty()) return;
    for ( int i = 0; i < 4; i++ ) {
        cv::Point p( mpMarker->vertex[i][0], mpMarker->vertex[i][1] );
        if ( ( p.x + size >= mImg.cols ) || ( p.x - size < 0 ) ||
                ( p.y + size >= mImg.rows ) || ( p.y - size < 0 ) ) {
            return;
        }
        cv::Point pt1( p.x + size, p.y + size );
        cv::Point pt2( p.x - size, p.y - size );
        cv::Point pt3( p.x - size, p.y + size );
        cv::Point pt4( p.x + size, p.y - size );
        cv::line ( mImg, pt1, pt2, color );
        cv::line ( mImg, pt3, pt4, color );
    }
}
void MarkerHdl::drawLines (const CvScalar &color, bool bSegmentsOnly ) {
    if (mImg.empty()) return;
    if ( bSegmentsOnly ) {
        cv::line ( mImg, cv::Point ( mpMarker->vertex[0][0], mpMarker->vertex[0][1] ), cv::Point ( mpMarker->vertex[1][0], mpMarker->vertex[1][1] ), color );
        cv::line ( mImg, cv::Point ( mpMarker->vertex[1][0], mpMarker->vertex[1][1] ), cv::Point ( mpMarker->vertex[2][0], mpMarker->vertex[2][1] ), color );
        cv::line ( mImg, cv::Point ( mpMarker->vertex[2][0], mpMarker->vertex[2][1] ), cv::Point ( mpMarker->vertex[3][0], mpMarker->vertex[3][1] ), color );
        cv::line ( mImg, cv::Point ( mpMarker->vertex[3][0], mpMarker->vertex[3][1] ), cv::Point ( mpMarker->vertex[0][0], mpMarker->vertex[0][1] ), color );
        cv::line ( mImg, cv::Point ( mpMarker->vertex[0][0], mpMarker->vertex[0][1] ), cv::Point ( mpMarker->vertex[2][0], mpMarker->vertex[2][1] ), color );
        cv::line ( mImg, cv::Point ( mpMarker->vertex[1][0], mpMarker->vertex[1][1] ), cv::Point ( mpMarker->vertex[3][0], mpMarker->vertex[3][1] ), color );
    } else {
        for ( int i = 0; i < 6; i ++ ) {
            double x1 = 0;
            double x2 = mImg.cols-1;
            double y1 = - ( mpMarker->line[i][0] * x1 + mpMarker->line[i][2] ) / mpMarker->line[i][1];
            double y2 = - ( mpMarker->line[i][0] * x2 + mpMarker->line[i][2] ) / mpMarker->line[i][1];
            cv::line ( mImg, cv::Point ( x1, y1 ), cv::Point ( x2,y2 ), color );
        }
    }
}

void MarkerHdl::drawId (const CvScalar &color, double scale, int fontFace ) {
    if (mImg.empty()) return;
    char pText[16];
    sprintf ( pText, "%i", mpMarker->id );
    cv::Point p( mpMarker->pos[0]+2, mpMarker->pos[1] );
    if ( ( p.x >= mImg.cols ) || ( p.x < 0 ) ||
            ( p.y >= mImg.rows ) || ( p.y < 0 ) ) {
        return;
    }
    cv::putText ( mImg, pText, p, fontFace, scale, color );
}
void MarkerHdl::drawGroup (const CvScalar &color, double scale, int fontFace ) {
    if (mImg.empty()) return;
    cv::Point p( mpMarker->pos[0]-20, mpMarker->pos[1] );
    if ( ( p.x >= mImg.cols ) || ( p.x < 0 ) ||
            ( p.y >= mImg.rows ) || ( p.y < 0 ) ) {
        return;
    }
    cv::putText ( mImg, group(), p, fontFace, scale, color );
}
void MarkerHdl::drawName (const CvScalar &color, double scale, int fontFace ) {
    if (mImg.empty()) return;
    cv::Point p( mpMarker->pos[0]+2, mpMarker->pos[1]-10 );
    if ( ( p.x >= mImg.cols ) || ( p.x < 0 ) ||
            ( p.y >= mImg.rows ) || ( p.y < 0 ) ) {
        return;
    }
    cv::putText ( mImg, name(), p, fontFace, scale, color );
}

void MarkerHdl::drawPosT (const CvScalar &color, double scale, int fontFace ) {
    if (mImg.empty()) return;
    char pText[32];
    sprintf ( pText, "%3.3f,%3.3f,%3.3f", mpMarker->poseT[0], mpMarker->poseT[1], mpMarker->poseT[2] );
    cv::Point p( mpMarker->pos[0]-10, mpMarker->pos[1]+10 );
    if ( ( p.x >= mImg.cols ) || ( p.x < 0 ) ||
            ( p.y >= mImg.rows ) || ( p.y < 0 ) ) {
        return;
    }
    cv::putText ( mImg, pText, p, fontFace, scale, color );
}
void MarkerHdl::drawConficence (const CvScalar &color, double scale, int fontFace ) {
    if (mImg.empty()) return;
    char pText[32];
    sprintf ( pText, "%3.2f", mpMarker->cf );
    cv::Point p( mpMarker->pos[0]-10, mpMarker->pos[1]+20 );
    if ( ( p.x >= mImg.cols ) || ( p.x < 0 ) ||
            ( p.y >= mImg.rows ) || ( p.y < 0 ) ) {
        return;
    }
    cv::putText ( mImg, pText, p, fontFace, scale, color );
}


void MarkerHdl::setUndistortParameter(const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) {
    cameraMatrix.assignTo(mCameraMatrix, CV_64F);
    distCoeffs.assignTo(mDistCoeffs, CV_64F);
}

void MarkerHdl::undistor() {
    CvMat  *src = cvCreateMat(5,1, CV_64FC2);
    CvMat  *des = cvCreateMat(5,1, CV_64FC2);
    src->data.db[0] = mpMarker->pos[0],src->data.db[1] = mpMarker->pos[1];
    src->data.db[2] = mpMarker->vertex[0][0],src->data.db[3] = mpMarker->vertex[0][1];
    src->data.db[4] = mpMarker->vertex[1][0],src->data.db[5] = mpMarker->vertex[1][1];
    src->data.db[6] = mpMarker->vertex[2][0],src->data.db[7] = mpMarker->vertex[2][1];
    src->data.db[8] = mpMarker->vertex[3][0],src->data.db[9] = mpMarker->vertex[3][1];
    CvMat cameraMatrix = cvMat(3,3, CV_64FC1, mCameraMatrix.data);
    CvMat distCoeffs = cvMat(1,4, CV_64FC1, mDistCoeffs.data);
    cvUndistortPoints(src, des, &cameraMatrix, &distCoeffs);
    mpMarker->pos[0] = des->data.db[0] * mCameraMatrix(0,0) + mCameraMatrix(0,2), mpMarker->pos[1] = des->data.db[1]* mCameraMatrix(1,1) + mCameraMatrix(1,2);
    mpMarker->vertex[0][0] = des->data.db[2]* mCameraMatrix(0,0) + mCameraMatrix(0,2), mpMarker->vertex[0][1] = des->data.db[3]* mCameraMatrix(1,1) + mCameraMatrix(1,2);
    mpMarker->vertex[1][0] = des->data.db[4]* mCameraMatrix(0,0) + mCameraMatrix(0,2), mpMarker->vertex[1][1] = des->data.db[5]* mCameraMatrix(1,1) + mCameraMatrix(1,2);
    mpMarker->vertex[2][0] = des->data.db[6]* mCameraMatrix(0,0) + mCameraMatrix(0,2), mpMarker->vertex[2][1] = des->data.db[7]* mCameraMatrix(1,1) + mCameraMatrix(1,2);
    mpMarker->vertex[3][0] = des->data.db[8]* mCameraMatrix(0,0) + mCameraMatrix(0,2), mpMarker->vertex[3][1] = des->data.db[9]* mCameraMatrix(1,1) + mCameraMatrix(1,2);
    cvReleaseMat(&src), cvReleaseMat(&des);

}


void MarkerHdl::findPose(double size, const cv::Mat_<double> &M) {
    double pWP[4][3];
    CvMat imagePoints = cvMat(4, 2, CV_64F, mpMarker->vertex);
    CvMat worldPoints = cvMat(4, 3, CV_64F, pWP);
    double r = size/2;
    pWP[0][0] = -r, pWP[0][1]  =  r, pWP[0][2] = 0;
    pWP[1][0] =  r, pWP[1][1]  =  r, pWP[1][2] = 0;
    pWP[2][0] =  r, pWP[2][1]  = -r, pWP[2][2] = 0;
    pWP[3][0] = -r, pWP[3][1]  = -r, pWP[3][2] = 0;
    CvMat rotation = cvMat ( 3,1, CV_64FC1, mpMarker->poseR );
    CvMat translation = cvMat ( 3,1, CV_64FC1, mpMarker->poseT );
    CvMat cameraMatrix = cvMat(3,3, CV_64FC1, mCameraMatrix.data);
    CvMat distCoeffs = cvMat(1,4, CV_64FC1, mDistCoeffs.data);
    cvFindExtrinsicCameraParams2(&worldPoints, &imagePoints, &cameraMatrix, &distCoeffs, &rotation, &translation);
    if (M.rows > 0) {
        cv::Mat_<double> S = cv::Mat_<double>::eye(4,4);
        cv::Mat_<double> D( 4, 4);
        cv::Mat_<double> r( 3, 1, mpMarker->poseR);
        cv::Mat_<double> R( 3, 3);
        cv::Rodrigues(r, R);
        S(0,3) = mpMarker->poseT[0], S(1,3) = mpMarker->poseT[1], S(2,3) = mpMarker->poseT[2];
        S(0,0) = R(0,0), S(0,1) = R(0,1), S(0,2) = R(0,2);
        S(1,0) = R(1,0), S(1,1) = R(1,1), S(1,2) = R(1,2);
        S(2,0) = R(2,0), S(2,1) = R(2,1), S(2,2) = R(2,2);
        D = M * S;
        R(0,0) = D(0,0), R(0,1) = D(0,1), R(0,2) = D(0,2);
        R(1,0) = D(1,0), R(1,1) = D(1,1), R(1,2) = D(1,2);
        R(2,0) = D(2,0), R(2,1) = D(2,1), R(2,2) = D(2,2);
        cv::Rodrigues(R, r);
        mpMarker->poseT[0] = D(0,3) , mpMarker->poseT[1] = D(1,3) , mpMarker->poseT[2] = D(2,3);
    }
}

void MarkerHdl::undistor(std::vector<Marker> &rMarker, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs) {
    MarkerHdl hdl;
    hdl.setUndistortParameter(cameraMatrix, distCoeffs);
    for (unsigned int i = 0; i < rMarker.size(); i++) {
        hdl.set(&rMarker[i]);
        hdl.undistor();
    }
}


// kate: indent-mode cstyle; space-indent on; indent-width 0;
