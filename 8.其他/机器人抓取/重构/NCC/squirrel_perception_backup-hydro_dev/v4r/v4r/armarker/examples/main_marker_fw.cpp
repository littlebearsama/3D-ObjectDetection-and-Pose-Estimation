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


#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <stdio.h>
#include <iostream>
#include <armarker.h>
#include <fw/fw.h>

int main ( int argc, char** argv ) {

    if ( ( argc != 1 ) && ( argc < 3 ) ) {
        fprintf( stderr, "usage: %s <CameraMatrix.xml> <DistCoeffs.xml> <pattern> <pattern> <pattern>\n", argv[0] );
        return 1;
    }

    int mode = 1; //0=YUV422, 1=MONO8/BAYER
    int fps = 2; //0=3.75Hz, 1=7.5Hz, 2=15Hz, 3=30Hz
    bool printModeSelection = false;
    V4R::FW fw;
    std::vector<uint64_t> camGuids = fw.readCameraGuids();
    std::vector<uint64_t> useGuid;
    useGuid.push_back(camGuids[0]);
    fw.init( mode, fps, useGuid, printModeSelection );
    std::vector<IplImage *> imagesBy8;
    imagesBy8.push_back(fw.createImageHeader());
    IplImage *pImgSrc = cvCreateImage(cvGetSize(imagesBy8[0]), IPL_DEPTH_8U, 3);
    cvNamedWindow( "pImgSrc", CV_WINDOW_AUTOSIZE );
    cvNamedWindow( "pImgUnDist", CV_WINDOW_AUTOSIZE );


    V4R::ARMarkerDetection md(fw.getWidth(), fw.getHeight());
    printf("Image size: %i x %i\n",fw.getWidth(), fw.getHeight());

    int key = 0;
    if ( argc != 1 ) {
        std::string fileCameraMatrix = argv[1];
        std::string fileDistCoeffs = argv[2];
        printf("CameraMatrix: %s\n",  fileCameraMatrix.c_str());
        printf("DistCoeffs  : %s\n",  fileDistCoeffs.c_str());
        md.loadCameraParameter(fileCameraMatrix, fileDistCoeffs, false);
    }

    if (argc > 2) {
        for (int i = 3; i < argc; i++) {
            md.loadMarker(argv[i], 200.0);
        }
    }

    while (key != 27) {
        fw.dequeue(imagesBy8);
        fw.bayerTo(imagesBy8[0], pImgSrc, 1, 514 );
        fw.enqueue();
        std::vector<V4R::Marker> marker = md.detectMarker(pImgSrc, 100);
        IplImage* pImgUnDist = md.getImgSrc();
        for (unsigned int i = 0; i < marker.size(); i ++) {
            V4R::MarkerHdl markerHdl(&marker[i]);
            markerHdl.set(pImgUnDist);
            markerHdl.drawCenter(cvScalar(0,0,255,0), 10);
            markerHdl.drawVertex(cvScalar(255,0,0,0));
            markerHdl.drawId(cvScalar(255,255,255,0));
            markerHdl.set(pImgSrc);
            markerHdl.drawCenter(cvScalar(0,0,255,0), 10);
            markerHdl.drawVertex( cvScalar(255,0,0,0));
            markerHdl.drawId(cvScalar(255,255,255,0));
            if ( marker[i].id < 0) {
                markerHdl.set(pImgUnDist);
                markerHdl.drawLines(cvScalar(255,255,255,0));
                markerHdl.set(pImgSrc);
                markerHdl.drawLines(cvScalar(255,255,255,0));
            } else {
                markerHdl.set(pImgUnDist);
                markerHdl.drawLines(cvScalar(0,255,0,0));
                markerHdl.drawPosT(cvScalar(255,255,255,0));
                markerHdl.set(pImgSrc);
                markerHdl.drawLines(cvScalar(0,255,0,0));
                markerHdl.drawPosT(cvScalar(255,255,255,0));
            }
        }
        cvShowImage( "pImgSrc", pImgSrc );
        cvShowImage( "pImgUnDist", pImgUnDist );
        key = cvWaitKey(10);
        //cvResize(pImgCam, pImgSrc);
        fflush (stdout);
    }
    cvReleaseImageHeader(&imagesBy8[0]);
    cvReleaseImage ( &pImgSrc );
    cvDestroyWindow ( "pImgSrc" );

    return 0;
}
