/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file Fourier.cpp
 * @author Richtsfeld
 * @date December 2011
 * @version 0.1
 * @brief Use fourier filter to compare surface texture.
 */

#include "Fourier.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

Fourier::Fourier()
{
  N = 8;
  kmax = 5;
  nbins = 8;
  binWidth = 32;        /// TODO 8x binWidth is higher than max-range of values
  binStretch = 2.;      // optimal = 8 => is now 6*k+binStretch
  
  have_image = false;
  computed = false;
  have_indices = false;
  
  bins = new double[kmax*nbins];
}

Fourier::~Fourier()
{
  delete dft;
  delete used;
  if(bins)
      delete[] bins;
}

void Fourier::setInputImage(cv::Mat &_image)
{
  if ( (_image.cols<=0) || (_image.rows<=0) )
    throw std::runtime_error("[Fourier::setInputImage] Invalid image (height|width must be > 1)");
  
  if ( _image.type() != CV_8UC1 )
    throw std::runtime_error("[Fourier::setInputImage] Invalid image type (must be 8UC1)");

  image = _image;
  width = image.cols;
  height = image.rows;
  
  have_image = true;
  computed = false;
  
  indices.reset(new pcl::PointIndices);
  for(int i = 0; i < width*height; i++)
  {
    indices->indices.push_back(i);
  }
  
  used = new bool[width*height*kmax];
  for(int i = 0; i < width*height*kmax; i++)
    used[i] = false;
  
  dft = new uchar[width*height*kmax];
  for(int i = 0; i < width*height*kmax; i++)
    dft[i] = 0;
  
}

void Fourier::setIndices(pcl::PointIndices::Ptr _indices)
{
  if(!have_image) {
    printf("[Fourier::setIndices]: Error: No image available.\n");
    return;
  }
    
  indices = _indices;
  have_indices = true;
  
  delete used;
  used = new bool[(indices->indices.size())*kmax];
  for(unsigned int i = 0; i < (indices->indices.size())*kmax; i++)
    used[i] = false;
  
  delete dft;
  dft = new uchar[(indices->indices.size())*kmax];
  for(unsigned int i = 0; i < (indices->indices.size())*kmax; i++)
    dft[i] = 0;
}

void Fourier::setIndices(std::vector<int> &_indices)
{
  indices.reset(new pcl::PointIndices);
  indices->indices = _indices;
  
  have_indices = true;
  
  delete used;
  used = new bool[(indices->indices.size())*kmax];
  for(unsigned int i = 0; i < (indices->indices.size())*kmax; i++)
    used[i] = false;
  
  delete dft;
  dft = new uchar[(indices->indices.size())*kmax];
  for(unsigned int i = 0; i < (indices->indices.size())*kmax; i++)
    dft[i] = 0;
}

void Fourier::setIndices(cv::Rect _rect)
{
  if(!have_image) {
    printf("[Fourier::setIndices]: Error: No image available.\n");
    return;
  }
  
  if(_rect.y >= height)
  {
    _rect.y = height-1;
  }
  
  if( (_rect.y + _rect.height) >= height)
  {
    _rect.height = height-_rect.y;
  }
  
  if(_rect.x >= width)
  {
    _rect.x = width-1;
  }
  
  if( (_rect.x + _rect.width) >= width)
  {
    _rect.width = width-_rect.x;
  }
  
  printf("[Fourier] _rect = %d,%d,%d,%d.\n",_rect.x,_rect.y,_rect.x+_rect.width,_rect.y+_rect.height);
  
  indices.reset(new pcl::PointIndices);
  for(int r = _rect.y; r < (_rect.y+_rect.height); r++)
  {
    for(int c = _rect.x; c < (_rect.x+_rect.width); c++)
    {
      indices->indices.push_back(r*width+c);
    }
  }
  
  have_indices = true;
  
  delete used;
  used = new bool[(indices->indices.size())*kmax];
  for(unsigned int i = 0; i < (indices->indices.size())*kmax; i++)
    used[i] = false;
  
  delete dft;
  dft = new uchar[(indices->indices.size())*kmax];
  for(unsigned int i = 0; i < (indices->indices.size())*kmax; i++)
    dft[i] = 0;
}

void Fourier::compute()
{
  if(!have_image) {
    printf("[Fourier::compute] Error: No image set.\n");
    return;
  }
  
  double SX_r[kmax];
  double SX_i[kmax];
  
  // neigbouring values N = 8
  // p3 p2 p1
  // p4 *  p0
  // p5 p6 p7
  double x[N];

  // real an imaginary part
  double en_r[kmax][N];
  double en_i[kmax][N];  
  
  for(int k = 0; k < kmax; k++) 
  {
    for(int n = 0; n < N; n++) 
    {
      en_r[k][n] = cos(-2*M_PI/N * k*n);     // magnitude: x*e^(j*phi) = x*(cos(phi) + j*sin(phi))
      en_i[k][n] = sin(-2*M_PI/N * k*n);
    }
  }
   
   for(unsigned int idx=0; idx<indices->indices.size(); idx++)
  {
    int i = indices->indices.at(idx)/width;
    int j = indices->indices.at(idx)%width;
    
    if( ((i-1) < 0) || ((i+1) >= height) || ((j-1) < 0) || ((j+1) >= width) )
    {
      for(int k = 0; k < kmax; k++)
      {
        dft[(indices->indices.size())*k + idx] = 0;
        used[(indices->indices.size())*k + idx] = true;
      }
      continue;
    }
    
    x[0] = (double) image.at<uchar>(i,  j+1);
    x[1] = (double) image.at<uchar>(i-1,j+1);
    x[2] = (double) image.at<uchar>(i-1,j);
    x[3] = (double) image.at<uchar>(i-1,j-1);
    x[4] = (double) image.at<uchar>(i,  j-1);
    x[5] = (double) image.at<uchar>(i+1,j-1);
    x[6] = (double) image.at<uchar>(i+1,j);
    x[7] = (double) image.at<uchar>(i+1,j+1);
      
    for(int k = 0; k < kmax; k++)
    {
      SX_r[k] = 0.;
      SX_i[k] = 0.;
      
      for(int n=0; n<N; n++) 
      {
        SX_r[k] += x[n]*en_r[k][n];
        SX_i[k] += x[n]*en_i[k][n];
      }
      double mag = sqrt(SX_r[k]*SX_r[k] + SX_i[k]*SX_i[k]) / ((double) N);
        
      //@ep: why double -> int -> char and not double -> char or double
      int Xk = (int) mag;
      dft[(indices->indices.size())*k + idx] = (uchar) Xk;
      used[(indices->indices.size())*k + idx] = true;
      
    }
  }
  
  for(int k = 0; k < kmax; k++) 
  {
    for(int n = 0; n < N; n++) 
    {
      bins[k*nbins+n] = 0.;
    }
  }
  
  for(unsigned int idx=0; idx<indices->indices.size(); idx++)
  {
    //int i = indices->indices.at(idx)/width;
    //int j = indices->indices.at(idx)%width;

    for(int k=0; k<kmax; k++) 
    {  
      if(!(used[(indices->indices.size())*k + idx]))
	continue;
      
      // 8 bins
      if(k == 0) 
      { 
        int bin = (int) (dft[(indices->indices.size())*k + idx]/((double)binWidth));
        bins[k*nbins+bin]+=1;
      }
      else 
      {
        //@ep: it is written differently in the paper
	int bin = (int) (dft[(indices->indices.size())*k + idx]/((double) binWidth/(binStretch+(6*k))));
        if(bin < nbins)
          bins[k*nbins+bin]+=1;
        else 
	{
          bin = nbins-1;
          bins[k*nbins+bin]+=1;
        }
      } 
    }
  }
  
  // TODO normalize from k=1 @ep: WHY???
  for(int k = 0; k < kmax; k++)
  {
    for(int n = 0; n < N; n++) 
    {
      bins[k*nbins+n] /= (indices->indices.size()*(kmax-1));
    }
  }
  
  computed = true;
}
  
// void Fourier::check()
// {
//   if(!computed) {
//     printf("[Fourier::check] Error: DFT not computed. Abort.\n");
//     return;
//   }
// 
//   cv::Mat_<uchar> X_image[5];
//   X_image[0] = cv::Mat_<uchar>(height,width);
//   X_image[1] = cv::Mat_<uchar>(height,width);
//   X_image[2] = cv::Mat_<uchar>(height,width);
//   X_image[3] = cv::Mat_<uchar>(height,width);
//   X_image[4] = cv::Mat_<uchar>(height,width);
// 
//   int bins[kmax][nbins];
//   for(int k=0; k<kmax; k++)
//     for(int n=0; n<N; n++)
//       bins[k][n] = 0;
//     
//   for(int row = 1; row<gray_image.size().height-1; row++) {
//     for(int col = 1; col<gray_image.size().width-1; col++) {
//       for(int k=0; k<kmax; k++) {  
//         X_image[k].at<uchar>(row, col) = dft->dft[col][row][k];
//         if(k == 0) { // 8 bins
//           int bin = (int) (dft->dft[col][row][k]/binWidth);
//           bins[k][bin]++;
//         }
//         else {
//           int bin = (int) (dft->dft[col][row][k]/binWidth*binStretch);
//           if(bin < nbins)
//             bins[k][bin]++;
//           else
//             printf("Fourier: Warning: bin size too big.\n");
//         } 
//       }
//     }
//   }
// 
//   cv::imshow("X_image[0]", X_image[0]);
//   cv::imshow("X_image[1]", X_image[1]);
//   cv::imshow("X_image[2]", X_image[2]);
//   cv::imshow("X_image[3]", X_image[3]);
//   cv::imshow("X_image[4]", X_image[4]);
//   
//   for(int k=0; k<kmax; k++)
//     for(int n=0; n<N; n++)
//       printf("bin[%u][%u]: %u\n", k, n, bins[k][n]);
// }
 

double Fourier::compare(Fourier::Ptr f)
{
  if(!computed) {
    printf("[Fourier::compare] Error: DFT not computed. Abort.\n");
    return 0.;
  }

  double fidelity = 0.;
  for(int k=1; k < kmax; k++) {
    for(int n = 0; n < N; n++) {
      fidelity += sqrt(bins[k*nbins+n] * f->bins[k*nbins+n]);
    }
  }  

//   // Compare histogram with euclidean distance
//   double euclidean = 0.;
//   for(int k=0; k<kmax; k++) {
//     for(int n=0; n<N; n++) {
//       euclidean += fabs(bins_0[k][n] - bins_1[k][n]);
//     }
//   }
//   euclidean = sqrt(euclidean);
//   
//   // print results
//   for(int k=0; k<kmax; k++)
//     for(int n=0; n<N; n++)
//       printf("bin[%u][%u]: %3.3f-%3.3f => %3.3f\n", k, n, bins_0[k][n], bins_1[k][n], sqrt(bins_0[k][n] * bins_1[k][n]));  
// 
//   printf("[Fourier::compare] fidelity: %4.3f / euclidean: %4.3f\n", fidelity, euclidean);

  return fidelity;
}

} // end surface














