/***************************************************************************
 *   Copyright (C) 2006 by Mian Zhou   *
 *   M.Zhou@reading.ac.uk   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef CVGABOR_H
#define CVGABOR_H

#include <iostream>
#include <stdio.h>

#include <opencv2/opencv.hpp>

namespace surface
{

#define PI 3.1415926534
#define CV_GABOR_REAL 1
#define CV_GABOR_IMAG 2
#define CV_GABOR_MAG  3
#define CV_GABOR_PHASE 4


/**
@author Mian Zhou
*/
class CvGabor{
public:
    CvGabor();
    CvGabor(int iMu, int iNu);
    ~CvGabor();
  
    CvGabor(int iMu, int iNu, double dSigma);
    CvGabor(int iMu, int iNu, double dSigma, double dF);
    CvGabor(double dPhi, int iNu);
    CvGabor(double dPhi, int iNu, double dSigma);
    CvGabor(double dPhi, int iNu, double dSigma, double dF);
    bool IsInit();
    long mask_width();
    IplImage* get_image(int Type);
    bool IsKernelCreate();
    long get_mask_width();
    void Init(int iMu, int iNu, double dSigma, double dF);
    void Init(double dPhi, int iNu, double dSigma, double dF);
    void output_file(const char *filename, int Type);
    CvMat* get_matrix(int Type);
    void show(int Type);
    void normalize( const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask );

    void conv_img_a(IplImage *src, IplImage *dst, int Type);
    void conv_img(IplImage *src, IplImage *dst, int Type);
//     void conv_img(cv::Mat_<cv::Vec3b> &_src, cv::Mat_<cv::Vec3b> &_dst, int type);

protected:
                              /// iMu ... The orientations which is iMu*PI.8
                              /// iNu ... The scale can be from -5 to infinit

    double Sigma;             /// dSigma ... The Sigma value of gabor, normally set to 2*PI 
    double F;                 /// The spatial frequence , normally is sqrt(2)
    double Kmax;              /// Kmax = PI/2.
    double K;                 /// K = Kmax / pow(F, (double)iNu);
    double Phi;               /// Phi = PI*iMu/8;
    bool bInitialised;
    bool bKernel;
    long Width;               /// The kernel with (Width x Width)
    CvMat *Imag;
    CvMat *Real;
  
private:
    void creat_kernel();
    

};

}

#endif
