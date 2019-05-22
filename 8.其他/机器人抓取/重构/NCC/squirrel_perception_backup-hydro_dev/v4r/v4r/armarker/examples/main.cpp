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

int main ( int argc, char** argv ) {
    printf ( "usage: %s <patternList> [imagefile]\n", argv[0] );

    IplImage* pImgSrc = NULL;
    CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
    if ( !capture ) {
        fprintf( stderr, "ERROR: capture is NULL using image file\n" );
        capture = NULL;

    }

    cvNamedWindow( "pImgSrc", CV_WINDOW_AUTOSIZE );

    if (capture) {
        pImgSrc = cvQueryFrame( capture );
    } else {
        if (argc < 2) {
            fprintf( stderr, "ERROR: no image imagefile\n" );
            return(-1);
        }
        const char *pFile = argv[argc-1];
        fprintf( stdout, "Using image %s\n",  pFile);
        pImgSrc = cvLoadImage ( pFile, CV_LOAD_IMAGE_UNCHANGED );
    }
    //IplImage *pImgSrc = cvCreateImage(cvSize(640,480), IPL_DEPTH_8U, 3);
    //cvResize(pImgCam, pImgSrc);

    V4R::MarkerDetection md;
    printf("Image size: %i x %i\n",pImgSrc->width, pImgSrc->height);

    int key = 0;

    if (argc > 1) {
        if (capture) {
            for (int i = 1; i < argc; i++) {
                md.loadMarker(argv[i]);
            }
        } else {
            for (int i = 1; (i < argc-1); i++) {
                md.loadMarker(argv[i]);
            }
        }
    }

    while (key != 27) {

        cv::Mat imgSrc(pImgSrc);
        std::vector<V4R::Marker> marker = md.detectMarker(imgSrc);
        for (unsigned int i = 0; i < marker.size(); i ++) {
            V4R::MarkerHdl markerHdl(&marker[i]);
            markerHdl.set(pImgSrc);
            markerHdl.drawCenter(cvScalar(0,0,255,0), 10);
            markerHdl.drawVertex(cvScalar(255,0,0,0));
            markerHdl.drawId(cvScalar(255,255,255,0));
            if ( marker[i].id < 0) {
                markerHdl.drawLines(cvScalar(255,255,255,0));
            } else {
                markerHdl.drawLines(cvScalar(0,255,0,0));
            }
        }
        cvShowImage( "pImgSrc", pImgSrc );
        key = cvWaitKey(10);
	if(capture){
	  pImgSrc = cvQueryFrame( capture );
	}
        //cvResize(pImgCam, pImgSrc);
        fflush (stdout);
    }
    cvReleaseCapture( &capture );

    cvReleaseImage ( &pImgSrc );
    cvDestroyWindow ( "pImgSrc" );

    return 0;
}
