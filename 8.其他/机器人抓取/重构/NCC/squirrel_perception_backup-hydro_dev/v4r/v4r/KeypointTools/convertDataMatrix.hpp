/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CONVERT_DATAMATRIX_2D_HPP
#define KP_CONVERT_DATAMATRIX_2D_HPP

#include <limits>
#include <opencv2/core/core.hpp>
#include "DataMatrix2D.hpp"


namespace kp
{

template<typename T>
void copyDataMatrix(const cv::Mat_<T> &src, DataMatrix2D<T> &dst)
{
  dst.clear();
  dst.reserve(src.rows, src.cols);

  for (int v=0; v<src.rows; v++)
    dst.push_back(&src(v,0), src.cols);
}

template<typename T1, typename T2>
void convertDataMatrix(const cv::Mat_<T1> &src, DataMatrix2D<T2> &dst)
{
  dst.resize(src.rows, src.cols);

  T2 *ptr = &dst[0];

  for (int v=0; v<src.rows; v++)
    for (int u=0; u<src.cols; u++, ptr++)
      *ptr = static_cast<T2>(src(v,u));
}

template<typename T>
void getMinMax(const DataMatrix2D<T> &src, T &min, T &max)
{
  min = std::numeric_limits<T>::max();
  max = -std::numeric_limits<T>::max();

  T *ptr = (T*)&src.data[0];
  T *end = (T*)&src.data.back();
  end++;

  for (; ptr!=end; ptr++)
  {
    if (*ptr>max) max = *ptr;
    if (*ptr<min) min = *ptr;
  }
}

template<typename T>
void getMinMaxXY(const DataMatrix2D<T> &src, T &min, T &max, int &xmin, int &ymin, int &xmax, int &ymax)
{
  min = std::numeric_limits<T>::max();
  max = -std::numeric_limits<T>::max();

  for (int v=0; v<src.rows; v++)
  {
    for (int u=0; u<src.cols; u++)
    {
      const T &d = src(v,u);
      if (d>max) { max = d; xmax = u; ymax = v; }
      if (d<min) { min = d; xmin = u; ymin = v; }
    }
  }
}

template<typename T1, typename T2>
void convertScaleMinMaxAbs(const DataMatrix2D<T1> &src, cv::Mat_<T2> &dst)
{
  dst = cv::Mat_<T2>(src.rows, src.cols);

  T1 min, max;
  getMinMax(src, min, max);

  T1 scale = std::numeric_limits<T2>::max()/(max-min);

  T1 *ptr = (T1*)&src[0];

  for (int v=0; v<src.rows; v++)
    for (int u=0; u<src.cols; u++, ptr++)
      dst(v,u) = static_cast<T2>( (*ptr-min) * scale);
}

template<typename T1, typename T2>
void convertScaleMinMaxAbs(const DataMatrix2D<T1> &src, cv::Mat_<T2> &dst, const T1 &min, const T1 &max)
{
  dst = cv::Mat_<T2>(src.rows, src.cols);

  T1 scale = std::numeric_limits<T2>::max()/(max-min);

  T1 *ptr = (T1*)&src[0];

  for (int v=0; v<src.rows; v++)
    for (int u=0; u<src.cols; u++, ptr++)
      dst(v,u) = static_cast<T2>( (*ptr-min) * scale);
}

template<typename T1, typename T2>
void convertScaleMinMaxAbs(const DataMatrix2D<T1> &src, DataMatrix2D<T2> &dst)
{
  dst.resize(src.rows, src.cols);

  T1 min, max;
  getMinMax(src, min, max);

  T1 scale = std::numeric_limits<T2>::max()/(max-min);

  T1 *ptr = (T1*)&src[0];
  T2 *ptr_dst = (T2*)&dst.data[0];
  T2 *end_dst = (T2*)&dst.data.back();
  end_dst++;

  for (; ptr_dst!=end_dst; ptr++, ptr_dst++)
    *ptr_dst = static_cast<T2>( (*ptr-min) * scale);
}

} //--END--

#endif




