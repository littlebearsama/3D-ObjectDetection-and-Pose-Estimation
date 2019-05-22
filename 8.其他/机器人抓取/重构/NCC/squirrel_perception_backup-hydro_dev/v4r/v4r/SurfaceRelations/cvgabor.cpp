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
#include "cvgabor.h"

namespace surface
{

CvGabor::CvGabor()    //constructor
{}


CvGabor::~CvGabor()   //Deconstructor
{
  cvReleaseMat( &Real );
  cvReleaseMat( &Imag );
}


/**
 * @brief Construct a gabor
 * Create a gabor with a orientation iMu*PI/8, a scale iNu, and a sigma value dSigma. 
 * The spatial frequence (F) is set to sqrt(2) defaultly. It calls Init() to generate
 * parameters and kernels.
 * @param iMu The orientation iMu*PI/8,     
 * @param iNu The scale                     
 * @param dSigma The sigma value of Gabor,
 */
CvGabor::CvGabor(int iMu, int iNu, double dSigma)
{ 
    F = sqrt(2.0);    
    Init(iMu, iNu, dSigma, F);   
}


/**
 * @brief Construct a gabor
 * Create a gabor with a orientation iMu*PI/8, a scale iNu, a sigma value dSigma, 
 * and a spatial frequence dF. It calls Init() to generate parameters and kernels.
 * @param iMu The orientation iMu*PI/8,    
 * @param iNu The scale                    
 * @param dSigma The sigma value of Gabor, 
 * @param dF The spatial frequency         
 */
CvGabor::CvGabor(int iMu, int iNu, double dSigma, double dF)   
{
    Init(iMu, iNu, dSigma, dF);
}


/**
 * @brief Construct a gabor
 * Create a gabor with a orientation dPhi, and with a scale iNu. The sigma (Sigma)
 * and the spatial frequence (F) are set to 2*PI and sqrt(2) defaultly. It calls 
 * Init() to generate parameters and kernels.
 * @param dPhi The orientation in arc
 * @param iNu The scale                    
 */
CvGabor::CvGabor(double dPhi, int iNu)   
{
    Sigma = 2*PI;
    F = sqrt(2.0);
    Init(dPhi, iNu, Sigma, F);
}


/**
 * @brief Construct a gabor
 * Create a gabor with a orientation dPhi, a scale iNu, and a sigma value dSigma.
 * The spatial frequence (F) is set to sqrt(2) defaultly. It calls Init() to 
 * generate parameters and kernels.
 * @param dPhi The orientation in arc
 * @param iNu The scale                     
 * @param dSigma The sigma value of Gabor
 */
CvGabor::CvGabor(double dPhi, int iNu, double dSigma) 
{
    F = sqrt(2.0);
    Init(dPhi, iNu, dSigma, F);
}


/**
 * @brief Construct a gabor
 * Create a gabor with a orientation dPhi, a scale iNu, a sigma value dSigma,
 * and a spatial frequence dF. It calls Init() to generate parameters and kernels.
 * @param dPhi The orientation in arc
 * @param iNu The scale                     
 * @param dSigma The sigma value of Gabor
 * @param dF The spatial frequency
 */
CvGabor::CvGabor(double dPhi, int iNu, double dSigma, double dF)  
{
   Init(dPhi, iNu, dSigma,dF);
}


/**
 * @brief Determine whether the gabor has been initlized.
 * Variables F, K, Kmax, Phi, Sigma are filled.
 * @return A boolean value, TRUE is initilised or FALSE is non-initilised.
 */
bool CvGabor::IsInit()
{
    return bInitialised;
}


/**
 * @brief Return the width of mask (should be NxN) by the value of Sigma and iNu.
 * @return The long type show the width.
 */
long CvGabor::mask_width()  
{
    long lWidth;
    if (IsInit() == false)  {
       perror ("Error: The Object has not been initilised in mask_width()!\n");
       return 0;
    }
    else {
       //determine the width of Mask
      double dModSigma = Sigma/K;
      double dWidth = (int)(dModSigma*6 + 1);
      //test whether dWidth is an odd.
      if (fmod(dWidth, 2.0)==0.0) dWidth++;
      lWidth = (long)dWidth;
      //printf("[CvGabor::mask_width] Gabor mask with: %lu\n", lWidth);
      return lWidth;
    }
}


/**
 * @brief Create 2 gabor kernels - REAL and IMAG, with an orientation and a scale 
 */
void CvGabor::creat_kernel()   //创建gabor核
{
  if (IsInit() == false) {perror("Error: The Object has not been initilised in creat_kernel()!\n");}
  else {
    CvMat *mReal, *mImag;
    mReal = cvCreateMat( Width, Width, CV_32FC1);  //实部窗口框的大小
    mImag = cvCreateMat( Width, Width, CV_32FC1);  //虚部窗口框的大小
    
    /**************************** Gabor Function ****************************/ 
    int x, y;
    double dReal;
    double dImag;
    double dTemp1, dTemp2, dTemp3;

    for (int i = 0; i < Width; i++)
    {
        for (int j = 0; j < Width; j++)
        {
          x = i-(Width-1)/2;
          y = j-(Width-1)/2;
          dTemp1 = (pow(K,2)/pow(Sigma,2))*exp(-(pow((double)x,2)+pow((double)y,2))*pow(K,2)/(2*pow(Sigma,2)));//高斯窗口函数
          dTemp2 = cos(K*cos(Phi)*x + K*sin(Phi)*y) - exp(-(pow(Sigma,2)/2));  //实部，去噪
          dTemp3 = sin(K*cos(Phi)*x + K*sin(Phi)*y);  //虚部
          dReal = dTemp1*dTemp2;    //求得的实部
          dImag = dTemp1*dTemp3;    //求得的虚部
                    //gan_mat_set_el(pmReal, i, j, dReal);
                    //cvmSet( (CvMat*)mReal, i, j, dReal );
          cvSetReal2D((CvMat*)mReal, i, j, dReal ); //存储求得的结果
                    //gan_mat_set_el(pmImag, i, j, dImag);
                    //cvmSet( (CvMat*)mImag, i, j, dImag );
          cvSetReal2D((CvMat*)mImag, i, j, dImag );
        } 
    }
    /**************************** Gabor Function ****************************/
    bKernel = true;
    cvCopy(mReal, Real, NULL);  //拷贝求得的结果
    cvCopy(mImag, Imag, NULL);
// printf("[CvGabor::creat_kernel] Message: A %d x %d Gabor kernel with %f PI in arc is created.\n", Width, Width, Phi/PI);
    cvReleaseMat( &mReal );
    cvReleaseMat( &mImag );
  }
}


/**
 * @brief Return an Image (gandalf image class) with a specific Type
 * @param Type The Type of gabor kernel, e.g. REAL, IMAG, MAG, PHASE   
 * @return Pointer to image structure, or NULL on failure
 */
IplImage* CvGabor::get_image(int Type)
{
    if(IsKernelCreate() == false) { 
      perror("Error: the Gabor kernel has not been created in get_image()!\n");
      return NULL;
    }
    else
    {  
    IplImage* pImage;
    IplImage *newimage;
    newimage = cvCreateImage(cvSize(Width,Width), IPL_DEPTH_8U, 1 );
    //printf("Width is %d.\n",(int)Width);
    //printf("Sigma is %f.\n", Sigma);
    //printf("F is %f.\n", F);
    //printf("Phi is %f.\n", Phi);
    
    //pImage = gan_image_alloc_gl_d(Width, Width);
    pImage = cvCreateImage( cvSize(Width,Width), IPL_DEPTH_32F, 1 );
    
    CvMat* kernel = cvCreateMat(Width, Width, CV_32FC1);
    double ve;
//     CvScalar S;
    CvSize size = cvGetSize( kernel );
    int rows = size.height;
    int cols = size.width;
    switch(Type)
    {
        case 1:  //Real
           cvCopy( (CvMat*)Real, (CvMat*)kernel, NULL );
            //pImage = cvGetImage( (CvMat*)kernel, pImageGL );
          for (int i = 0; i < rows; i++)
          {
            for (int j = 0; j < cols; j++)
            {
              ve = cvGetReal2D((CvMat*)kernel, i, j);
              cvSetReal2D( (IplImage*)pImage, j, i, ve );
            }
          }
          break;
        case 2:  //Imag
           cvCopy( (CvMat*)Imag, (CvMat*)kernel, NULL );
           //pImage = cvGetImage( (CvMat*)kernel, pImageGL );
          for (int i = 0; i < rows; i++)
          {
            for (int j = 0; j < cols; j++)
            {
              ve = cvGetReal2D((CvMat*)kernel, i, j);
              cvSetReal2D( (IplImage*)pImage, j, i, ve );
            }
          }
          break; 
        case 3:  //Magnitude
           ///@todo  
           printf("[CvGabor::get_image] Error: No magnitude available.\n");
           break;
        case 4:  //Phase
          ///@todo
           printf("[CvGabor::get_image] Error: No phase available.\n");
           break;
    }
   
    cvNormalize((IplImage*)pImage, (IplImage*)pImage, 0, 255, CV_MINMAX, NULL );
    cvConvertScaleAbs( (IplImage*)pImage, (IplImage*)newimage, 1, 0 );

    cvReleaseMat(&kernel);
    cvReleaseImage(&pImage);
    return newimage;
  }
}


/**
 * @brief Determine the gabor kernel is created or not
 * @return A boolean value, TRUE is created or FALSE is non-created.
 */
bool CvGabor::IsKernelCreate()
{
    return bKernel;
}


/**
 * @brief Reads the width of Mask
 * @return Pointer to long type width of mask.
 */
long CvGabor::get_mask_width()
{
  return Width;
}


/**
 * @brief Initilize the.gabor with the orientation iMu, the scale iNu, 
 * the sigma dSigma, the frequency dF, it will call the function 
 * creat_kernel(); So a gabor is created.
 * @param iMu   The orientations which is iMu*PI.8
 * @param iNu   The scale can be from -5 to infinit
 * @param dSigma  The Sigma value of gabor, Normally set to 2*PI
 * @param dF  The spatial frequence , normally is sqrt(2)
 */
void CvGabor::Init(int iMu, int iNu, double dSigma, double dF)
{
// printf("CvGabor::Init: start\n");
  //Initilise the parameters 
  bInitialised = false;
  bKernel = false;

  Sigma = dSigma;
  F = dF;

  Kmax = PI/2;

  // Absolute value of K
  K = Kmax / pow(F, (double)iNu);
  Phi = PI*iMu/8;
  bInitialised = true;
  Width = mask_width();
  Real = cvCreateMat( Width, Width, CV_32FC1);
  Imag = cvCreateMat( Width, Width, CV_32FC1);
  creat_kernel();  
// printf("CvGabor::Init: done\n");
}


/**
 * @brief Initilize the.gabor with the orientation dPhi, the scale iNu, the sigma dSigma, 
 * the frequency dF, it will call the function creat_kernel(); So a gabor is created.filename  
 * The name of the image file
      file_format   The format of the file, e.g. GAN_PNG_FORMAT
      image   The image structure to be written to the file
      octrlstr  Format-dependent control structure
 * @param dPhi  The orientations 
 * @param iNu   The scale can be from -5 to infinit
 * @param dSigma  The Sigma value of gabor, Normally set to 2*PI
 * @param dF  The spatial frequence , normally is sqrt(2)
 */
void CvGabor::Init(double dPhi, int iNu, double dSigma, double dF)
{
// printf("CvGabor::Init2: start\n");
  bInitialised = false;
  bKernel = false;
  Sigma = dSigma;
  F = dF;
  
  Kmax = PI/2;
  
  // Absolute value of K
  K = Kmax / pow(F, (double)iNu);
  Phi = dPhi;
  bInitialised = true;
  Width = mask_width();
// printf("CvGabor::Init2: 1\n");
  Real = cvCreateMat( Width, Width, CV_32FC1);
// printf("CvGabor::Init2: 2\n");
  Imag = cvCreateMat( Width, Width, CV_32FC1);
// printf("CvGabor::Init2: 3\n");
  creat_kernel();  
// printf("CvGabor::Init2: done\n");
}


/**
 * @brief Return the gabor kernel.
 * @param Type    The type of kernel, e.g. REAL, IMAG, MAG, PHASE
 * @return Pointer to matrix structure, or NULL on failure.
 */
CvMat* CvGabor::get_matrix(int Type)
{
  if (!IsKernelCreate()) {perror("Error: the gabor kernel has not been created!\n"); return NULL;}
  switch (Type)
  {
    case CV_GABOR_REAL:
      return Real;
      break;
    case CV_GABOR_IMAG:
      return Imag;
      break;
    case CV_GABOR_MAG:
      printf("[CvGabor::get_matrix] Error: No gabor magnitude available.\n");
      return NULL;
      break;
    case CV_GABOR_PHASE:
      printf("[CvGabor::get_matrix] Error: No gabor phase available.\n");
      return NULL;
      break;
  }
  return NULL;
}


/**
 * @brief Writes an image from the provided image structure into the 
 * given file and the type of gabor kernel.
 * @param filename  The name of the image file
 * @param file_format   The format of the file, e.g. GAN_PNG_FORMAT
 * @param Type    The Type of gabor kernel, e.g. REAL, IMAG, MAG, PHASE   
 * @return Pointer to matrix structure, or NULL on failure.
 */
void CvGabor::output_file(const char *filename, int Type)
{
  IplImage *pImage;
  pImage = get_image(Type);
  if(pImage != NULL)
  {
    if( cvSaveImage(filename, pImage )) printf("%s has been written successfully!\n", filename);
    else printf("Error: writting %s has failed!\n", filename);
  }
  else 
    perror("Error: the image is empty in output_file()!\n"); 

  cvReleaseImage(&pImage);
}



/**
 * @brief CvGabor::show(int Type)
 */
void CvGabor::show(int Type)
{
    if(!IsInit()) {
        perror("Error: the gabor kernel has not been created!\n");
    }
    else {
      //    IplImage *pImage;
      //pImage = get_image(Type);
      //cvNamedWindow("Testing",1);
      //cvShowImage("Testing",pImage);
      //cvWaitKey(0);
      //cvDestroyWindow("Testing");
      //cvReleaseImage(&pImage);
    }
}


/**
 * @brief CvGabor::conv_img_a(IplImage *src, IplImage *dst, int Type)
 */
void CvGabor::conv_img_a(IplImage *src, IplImage *dst, int Type)   //图像做gabor卷积  函数名：conv_img_a
{
    double ve, re,im;
  
    int width = src->width;
    int height = src->height;
    CvMat *mat = cvCreateMat(src->width, src->height, CV_32FC1);
    
    for (int i = 0; i < width; i++)  //对整幅图像进行图像坐标转换
    {
       for (int j = 0; j < height; j++)
       {
              ve = cvGetReal2D((IplImage*)src, j, i);
              cvSetReal2D( (CvMat*)mat, i, j, ve );
       }
    }

    CvMat *rmat = cvCreateMat(width, height, CV_32FC1);  //存实部
    CvMat *imat = cvCreateMat(width, height, CV_32FC1);  //存虚部

    CvMat *kernel = cvCreateMat( Width, Width, CV_32FC1 ); //创建核函数窗口

    switch (Type)
    {
      case CV_GABOR_REAL:   //实部卷积
        cvCopy( (CvMat*)Real, (CvMat*)kernel, NULL );
        cvFilter2D( (CvMat*)mat, (CvMat*)mat, (CvMat*)kernel, cvPoint( (Width-1)/2, (Width-1)/2));
        break;
      case CV_GABOR_IMAG:      //虚部卷积
        cvCopy( (CvMat*)Imag, (CvMat*)kernel, NULL );
        cvFilter2D( (CvMat*)mat, (CvMat*)mat, (CvMat*)kernel, cvPoint( (Width-1)/2, (Width-1)/2));
        break;
      case CV_GABOR_MAG:   //实部与虚部卷积
        /* Real Response */
        cvCopy( (CvMat*)Real, (CvMat*)kernel, NULL );
        cvFilter2D( (CvMat*)mat, (CvMat*)rmat, (CvMat*)kernel, cvPoint( (Width-1)/2, (Width-1)/2));
        /* Imag Response */
        cvCopy( (CvMat*)Imag, (CvMat*)kernel, NULL );
        cvFilter2D( (CvMat*)mat, (CvMat*)imat, (CvMat*)kernel, cvPoint( (Width-1)/2, (Width-1)/2));
        /* Magnitude response is the square root of the sum of the square of real response and imaginary response */
        for (int i = 0; i < width; i++)
        {
           for (int j = 0; j < height; j++)
           {
               re = cvGetReal2D((CvMat*)rmat, i, j);
               im = cvGetReal2D((CvMat*)imat, i, j);
               ve = sqrt(re*re + im*im);
               cvSetReal2D( (CvMat*)mat, i, j, ve );
           }
        }       
        break;
      case CV_GABOR_PHASE:
        break;
    }
    
    if (dst->depth == IPL_DEPTH_8U)  //归一化
    {
        cvNormalize((CvMat*)mat, (CvMat*)mat, 0, 255, CV_MINMAX, NULL);
      for (int i = 0; i < width; i++)
      {
            for (int j = 0; j < height; j++)
            {
                ve = cvGetReal2D((CvMat*)mat, i, j);
                ve = cvRound(ve);
                cvSetReal2D( (IplImage*)dst, j, i, ve );
            }
        }
     }

     if (dst->depth == IPL_DEPTH_32F)
     {
         for (int i = 0; i < width; i++)
       {
            for (int j = 0; j < height; j++)
            {
                ve = cvGetReal2D((CvMat*)mat, i, j);
                cvSetReal2D( (IplImage*)dst, j, i, ve );
            }
         }
     } 

    cvReleaseMat(&kernel);
    cvReleaseMat(&imat);
    cvReleaseMat(&rmat);
    cvReleaseMat(&mat);
}


/**
 * @brief CvGabor::CvGabor(int iMu, int iNu)
 */
 CvGabor::CvGabor(int iMu, int iNu)
{
  double dSigma = 2*PI; 
  F = sqrt(2.0);
  Init(iMu, iNu, dSigma, F);
}


/**
 * @brief CvGabor::normalize( const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask )
 * @param src
 * @param dst
 * @param a
 * @param b
 * @param norm_type
 * @param mask
 */
void CvGabor::normalize( const CvArr* src, CvArr* dst, double a, double b, int norm_type, const CvArr* mask )
{
    CvMat* tmp = 0;
//     __BEGIN__;

    double scale, shift;
    if( norm_type == CV_MINMAX )
    {
        double smin = 0, smax = 0;
        double dmin = MIN( a, b ), dmax = MAX( a, b );
        cvMinMaxLoc( src, &smin, &smax, 0, 0, mask );
        scale = (dmax - dmin)*(smax - smin > DBL_EPSILON ? 1./(smax - smin) : 0);
        shift = dmin - smin*scale;
    }
    else if( norm_type == CV_L2 || norm_type == CV_L1 || norm_type == CV_C )
    {
//         CvMat *s = (CvMat*)src, *d = (CvMat*)dst;
        scale = cvNorm( src, 0, norm_type, mask );
        scale = scale > DBL_EPSILON ? 1./scale : 0.;
        shift = 0;
    }
    else {}
    
    if( !mask )
        cvConvertScale( src, dst, scale, shift );
    else
    {
//         CvMat stub, *dmat;
        cvConvertScale( src, tmp, scale, shift );
        cvCopy( tmp, dst, mask );
    }

//    __END__;
    if( tmp )
        cvReleaseMat( &tmp );
}


/**
 * @brief CvGabor::conv_img(IplImage *src, IplImage *dst, int Type)
 * @param src
 * @param dst
 * @param Type
 */
void CvGabor::conv_img(IplImage *src, IplImage *dst, int Type)   //函数名：conv_img
{
// printf("CvGabor::conv_img 1\n");
  double ve; //, re,im;
  
  CvMat *mat = cvCreateMat(src->width, src->height, CV_32FC1);
  for (int i = 0; i < src->width; i++) {
    for (int j = 0; j < src->height; j++) {
      ve = CV_IMAGE_ELEM(src, uchar, j, i);   //CV_IMAGE_ELEM 是取图像（j，i）位置的像素值
      CV_MAT_ELEM(*mat, float, i, j) = (float)ve;  //转化成float 类型
    }
  }
  
// printf("CvGabor::conv_img 2\n");
  CvMat *rmat = cvCreateMat(src->width, src->height, CV_32FC1);
  CvMat *imat = cvCreateMat(src->width, src->height, CV_32FC1);
  
  switch (Type)
  {
    case CV_GABOR_REAL:
      cvFilter2D( (CvMat*)mat, (CvMat*)mat, (CvMat*)Real, cvPoint( (Width-1)/2, (Width-1)/2));
      break;
    case CV_GABOR_IMAG:
      cvFilter2D( (CvMat*)mat, (CvMat*)mat, (CvMat*)Imag, cvPoint( (Width-1)/2, (Width-1)/2));
      break;
    case CV_GABOR_MAG:
      cvFilter2D( (CvMat*)mat, (CvMat*)rmat, (CvMat*)Real, cvPoint( (Width-1)/2, (Width-1)/2));
      cvFilter2D( (CvMat*)mat, (CvMat*)imat, (CvMat*)Imag, cvPoint( (Width-1)/2, (Width-1)/2));
      
      cvPow(rmat,rmat,2); 
      cvPow(imat,imat,2);
      cvAdd(imat,rmat,mat); 
      cvPow(mat,mat,0.5); 
      break;
    case CV_GABOR_PHASE:
      break;
  }
  
// printf("CvGabor::conv_img 3\n");
  if (dst->depth == IPL_DEPTH_8U)
  {
    cvNormalize((CvMat*)mat, (CvMat*)mat, 0, 255, CV_MINMAX);
    for (int i = 0; i < mat->rows; i++)
    {
      for (int j = 0; j < mat->cols; j++)
      {
        ve = CV_MAT_ELEM(*mat, float, i, j);
        CV_IMAGE_ELEM(dst, uchar, j, i) = (uchar)cvRound(ve);
      }
    }
  }
  
// printf("CvGabor::conv_img 4\n");
  if (dst->depth == IPL_DEPTH_32F)
  {
    for (int i = 0; i < mat->rows; i++)
    {
      for (int j = 0; j < mat->cols; j++)
      {
        ve = cvGetReal2D((CvMat*)mat, i, j);
        cvSetReal2D( (IplImage*)dst, j, i, ve );
      }
    }
  }
// printf("CvGabor::conv_img 5\n");
  cvReleaseMat(&imat);
  cvReleaseMat(&rmat);
  cvReleaseMat(&mat);
// printf("CvGabor::conv_img 6\n");
}

// void CvGabor::conv_img(cv::Mat_<cv::Vec3b> &_src, cv::Mat_<cv::Vec3b> &_dst, int type)   //函数名：conv_img
// {
// }

}
