/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "ImGradientDescriptor.hh"

//#define IMGD_INTERPOLATED


namespace kp 
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
ImGradientDescriptor::ImGradientDescriptor(const Parameter &p)
 : param(p)
{ 
  //param.computeRootGD = false;
}

ImGradientDescriptor::~ImGradientDescriptor()
{
}

/**
 * ComputeGradients
 */
void ImGradientDescriptor::ComputeGradients(const cv::Mat_<unsigned char> &im)
{
  if (param.smooth)
    cv::blur(im, im_smooth, cv::Size(3,3));
  else im_smooth = im;

  cv::Sobel(im_smooth, im_dx, CV_16S, 1, 0, 3, 1, 0, cv::BORDER_DEFAULT );
  cv::Sobel(im_smooth, im_dy, CV_16S, 0, 1, 3, 1, 0, cv::BORDER_DEFAULT );
}

/**
 * ComputeDescriptor
 */
void ImGradientDescriptor::ComputeDescriptor(std::vector<float> &desc, const cv::Mat_<float> &weight)
{
  int h;
  short *ptr_dx, *ptr_dy;

  desc.clear();
  desc.resize(128,0);

  int dv = (im_dx.rows-2)/4;
  int du = (im_dx.cols-2)/4;

  for (int v=1; v<im_dx.rows-1; v++)
  {
    ptr_dx = &im_dx(v,1);
    ptr_dy = &im_dy(v,1);

    for (int u=1; u<im_dx.cols-1; u++, ptr_dx++, ptr_dy++)
    {
      if (*ptr_dx>0 && *ptr_dy>0){
        if (*ptr_dx>*ptr_dy) h=0;
        else h=1;
      }else if (*ptr_dx<0 && *ptr_dy>0){
        if ((-*ptr_dx)<*ptr_dy) h=2;
        else h=3;
      }else if (*ptr_dx<0 && *ptr_dy<0){
        if ((*ptr_dx)<*ptr_dy) h=4;
        else h=5;
      }else{
        if ((*ptr_dx)<(-*ptr_dy)) h=6;
        else h=7;
      }

      //desc[(((v-1)/dv*4 + (u-1)/du))*8 + h] += float(sqrt(*ptr_dx * *ptr_dx + *ptr_dy * *ptr_dy)) * weight(v,u);
      desc[(((v-1)/dv*4 + (u-1)/du))*8 + h] += float(fabs(*ptr_dx)+fabs(*ptr_dy)) * weight(v,u);
      //desc[(v-1)/dv*4*8 + (u-1)/du*8 + h] += float(fabs(*ptr_dx)+fabs(*ptr_dy)) * weight(v,u);

      //cout<<sqrt(*ptr_dx * *ptr_dx + *ptr_dy * *ptr_dy)<<" ";
    }
  } 
}

/**
 * ComputeDescriptorInterpolate
 */
void ImGradientDescriptor::ComputeDescriptorInterpolate(std::vector<float> &desc, const cv::Mat_<float> &weight)
{
  int h;
  //short *ptr_dx, *ptr_dy;

  desc.clear();
  desc.resize(128,0);

  int dv = (im_dx.rows-2)/4;
  int du = (im_dx.cols-2)/4;
  float du_2 = float(du)/2.;
  float dv_2 = float(dv)/2.;
  float wx,wy, wx2,wy2;

  for (int v=0; v<4; v++)
  {
    for (int u=0; u<4; u++)
    {
      for (int y=0; y<dv; y++)
      {
        for (int x=0; x<du; x++)
        {
          int ug = u*du+x+1;
          int vg = v*dv+y+1;
          const float &dx = im_dx(vg,ug);
          const float &dy = im_dy(vg,ug);

          if (dx>0 && dy>0){
            if (dx>dy) h=0;
            else h=1;
          }else if (dx<0 && dy>0){
            if ((-dx)<dy) h=2;
            else h=3;
          }else if (dx<0 && dy<0){
            if (dx<dy) h=4;
            else h=5;
          }else{
            if (dx<-dy) h=6;
            else h=7;
          }
          
          wx = 0.5 + .5 * (1. - (float(x)-du_2) / du_2);
          wy = 0.5 + .5 * (1. - (float(y)-dv_2) / dv_2);
          
          desc[(v*4 + u)*8 + h] += float(fabs(dx)+fabs(dy)) * weight(vg,ug) * fabs(wx)*fabs(wy);

          wx2 = 1.-fabs(wx);
          wy2 = 1.-fabs(wy);
          int u2 = u+sign(wx);
          int v2 = v+sign(wy);

          if (u2>=0 && u2<4) {
            desc[(v*4+u2)*8+h]+=float(fabs(dx)+fabs(dy))*weight(vg,ug)*fabs(wx2)*fabs(wy);
          }
          if (v2>=0 && v2<4) {
            desc[(v2*4+u)*8+h]+=float(fabs(dx)+fabs(dy))*weight(vg,ug)*fabs(wx)*fabs(wy2);
          }
          if (u2>=0 && u2<4 && v2>=0 && v2<4) {
            desc[(v*4+u2)*8+h]+=float(fabs(dx)+fabs(dy))*weight(vg,ug)*fabs(wx2)*fabs(wy2);
          }

        }
      }
    }
  }
}


/**
 * ComputeLTGauss
 */
void ImGradientDescriptor::ComputeLTGauss(const cv::Mat_<unsigned char> &im)
{
  if (lt_gauss.rows!=im.rows || lt_gauss.cols!=im.cols)
  {
    if (param.gauss_lin)
      ComputeLTGaussLin(im);
    else ComputeLTGaussCirc(im);
  }
}

/**
 * ComputeLTGaussCirc
 */
void ImGradientDescriptor::ComputeLTGaussCirc(const cv::Mat_<unsigned char> &im)
{  
  if (im.rows!=im.cols || (im.rows-2)%4 != 0)
    throw std::runtime_error("[ImGradientDescriptor::ComputeLTGaussCirc] Invalid patch size!");

  lt_gauss=cv::Mat_<float>(im.rows,im.cols);

  float h_size = im.rows/2;
  float invSqrSigma;

  invSqrSigma = param.sigma*(float)(h_size-1);
  invSqrSigma = -1./(invSqrSigma*invSqrSigma);
  
  for (int v=-h_size; v<h_size; v++)
  {
    for (int u=-h_size; u<h_size; u++)
    {
      lt_gauss(v+h_size,u+h_size) = exp(invSqrSigma*(u*u+v*v));
    }
  }

  /*cv::Mat_<unsigned char> tmp(im.rows,im.cols);
  for (int v=0; v<im.rows; v++)
    for (int u=0; u<im.cols; u++)
      tmp(v,u) = lt_gauss(v,u)*255;
  cv::imshow("image",tmp);
  cv::waitKey(0);*/
}

/**
 * ComputeLTGaussLin
 */
void ImGradientDescriptor::ComputeLTGaussLin(const cv::Mat_<unsigned char> &im)
{  
  if ((im.cols-2)%4 != 0 || (im.rows-2)%4 != 0)
    throw std::runtime_error("[ImGradientDescriptor::ComputeLTGaussLin] Invalid patch size!");

  lt_gauss.resize(im.rows,im.cols);

  float h_size = im.rows/2;
  float invSqrSigma;

  invSqrSigma = param.sigma*(float)(h_size-1);
  invSqrSigma = -1./(invSqrSigma*invSqrSigma);
  
  for (int v=-h_size; v<h_size; v++)
  {
    for (int u=0; u<im.cols; u++)
    {
      lt_gauss(v+h_size,u) = exp(invSqrSigma*(u*u));
    }
  }
}


/**
 * Normalize
 */
void ImGradientDescriptor::Normalize(std::vector<float> &desc)
{
  float norm=0;

  float *ptr = &desc[0];
  for (unsigned i=0; i<desc.size(); i++, ptr++)
    norm += (*ptr * *ptr);

  if (norm > numeric_limits<float>::epsilon( ))
  {
    norm = 1./sqrt(norm);

    for (unsigned i=0; i<desc.size(); i++)
      desc[i] *= norm;
  }
}

/**
 * Normalize
 */
void ImGradientDescriptor::Cut(std::vector<float> &desc)
{
  float *ptr = &desc[0];
  float *ptr_end = &desc.back();
  ptr_end++;

  for (; ptr!=ptr_end; ptr++)
    if (*ptr > param.thrCutDesc)
      *ptr = param.thrCutDesc;
}



/***************************************************************************************/

/**
 * compute gradient descriptor of center point (sift like)
 * we use a 3x3 sobel kernel
 * @param im image patch to compute the descriptor (min. 18x18 = 16x16 + 1px boarder)
 */
void ImGradientDescriptor::compute(const cv::Mat_<unsigned char> &im, std::vector<float> &desc)
{
  ComputeLTGauss(im);

  ComputeGradients(im);

  #ifdef IMGD_INTERPOLATED
  ComputeDescriptorInterpolate(desc, lt_gauss);
  #else
  ComputeDescriptor(desc, lt_gauss);
  #endif

  if (param.normalize)
  {
    Normalize(desc);   // to 1
    Cut(desc);         // cut 0.2
    Normalize(desc);   // renormalize to 1
  }

  if (param.computeRootGD)
  {
    Eigen::Map<Eigen::VectorXf> eig_desc(&desc[0], desc.size());
    float norm = eig_desc.lpNorm<1>();
    eig_desc.array() /= norm;
    eig_desc.array() = eig_desc.array().sqrt();
  }
}

/**
 * compute gradient descriptor of center point (sift like)
 * we use a 3x3 sobel kernel
 * @param im image patch to compute the descriptor (min. 18x18 = 16x16 + 1px boarder)
 */
void ImGradientDescriptor::compute(const cv::Mat_<unsigned char> &im, const cv::Mat_<float> &weight, std::vector<float> &desc)
{
  ComputeGradients(im);

  #ifdef IMGD_INTERPOLATED
  ComputeDescriptorInterpolate(desc, weight);
  #else
  ComputeDescriptor(desc, weight);
  #endif

  if (param.normalize)
  {
    Normalize(desc);   // to 1
    Cut(desc);         // cut 0.2
    Normalize(desc);   // renormalize to 1
  }

  if (param.computeRootGD)
  {
    Eigen::Map<Eigen::VectorXf> eig_desc(&desc[0], desc.size());
    float norm = eig_desc.lpNorm<1>();
    eig_desc.array() /= norm;
    eig_desc.array() = eig_desc.array().sqrt();
  }
}





}












