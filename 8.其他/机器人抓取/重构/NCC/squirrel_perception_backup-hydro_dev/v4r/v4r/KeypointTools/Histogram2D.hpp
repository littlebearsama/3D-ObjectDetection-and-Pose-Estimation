/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_HISTOGRAM2D_HPP
#define KP_HISTOGRAM2D_HPP

#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <stdexcept>
#include "SmartPtr.hpp"

namespace kp 
{

template <class T=unsigned>
class Histogram2D
{
public:
  int rows, cols;

private:
  double rmin, rmax, cmin, cmax;
  double roffs, coffs;
  double rscale, cscale;

  int num_samples;

  std::vector<T> data;

public:
 
  Histogram2D(const int &_rows=100, const int &_cols=100, 
              const double &_rmin=0., const double &_rmax=1., 
              const double &_cmin=0., const double &_cmax=1.);
  ~Histogram2D();

  void insert(const std::vector<std::pair<double,double> > &samples);
  inline bool insert(const double &data_row, const double &data_col);

  inline int numSamples() {return num_samples;}
  inline int GetIdx(const short row, const short col) const;
  inline short Row(const int idx) const;
  inline short Col(const int idx) const;
  inline void setTo(const T &d);

  T& operator[](unsigned i);
  const T& operator[](unsigned i) const;
  T& operator[](int i);
  const T& operator[](int i) const;
  T& operator()(unsigned row, unsigned col);
  const T& operator()(unsigned row, unsigned col) const;
  T& operator()(int row, int col);
  const T& operator()(int row, int col) const;
  T& operator()(double data_row, double data_col);
  const T& operator()(double data_row, double data_col) const;

  static void write(const std::string &filename, const Histogram2D<T> &hist);
  static void write(std::ofstream &out, const Histogram2D<T> &hist);
  static void read(const std::string &filename, Histogram2D<T> &hist);
  static void read(std::ifstream &in, Histogram2D<T> &hist);

  typedef SmartPtr< ::kp::Histogram2D<T> > Ptr;
  typedef SmartPtr< ::kp::Histogram2D<T>  const> ConstPtr;
};


/**
 * Histogram2Di
 */
class Histogram2Di : public Histogram2D<int>
{
public:
  Histogram2Di(const int &_rows=100, const int &_cols=100, 
               const double &_rmin=0., const double &_rmax=1., 
                const double &_cmin=0., const double &_cmax=1.)
  : Histogram2D<int>(_rows, _cols, _rmin, _rmax, _cmin, _cmax)
  {
  }
};

/**
 * Histogram2Df
 */
class Histogram2Df : public Histogram2D<float>
{
public:
  Histogram2Df(const int &_rows=100, const int &_cols=100,
               const double &_rmin=0., const double &_rmax=1.,
               const double &_cmin=0., const double &_cmax=1.)
  : Histogram2D<float>(_rows, _cols, _rmin, _rmax, _cmin, _cmax)
  {
  }
};

/**
 * Histogram2Dd
 */
class Histogram2Dd : public Histogram2D<double>
{
public:
  Histogram2Dd(const int &_rows=100, const int &_cols=100,
               const double &_rmin=0., const double &_rmax=1.,
               const double &_cmin=0., const double &_cmax=1.)
  : Histogram2D<double>(_rows, _cols, _rmin, _rmax, _cmin, _cmax)
  {
  }
};

/************************************************************************************
 * Constructor/Destructor
 */
template <class T>
Histogram2D<T>::Histogram2D(const int &_rows, const int &_cols,
                            const double &_rmin, const double &_rmax,
                            const double &_cmin, const double &_cmax)
 : rows(_rows), cols(_cols), rmin(_rmin), rmax(_rmax), cmin(_cmin), cmax(_cmax), 
   num_samples(0)
{
  roffs = -_rmin;
  coffs = -_cmin;
  rscale = double(_rows)/(_rmax-_rmin);
  cscale = double(_cols)/(_cmax-_cmin);

  data.resize(rows*cols,T(0));
}

template <class T>
Histogram2D<T>::~Histogram2D()
{
}

/**
 * insert
 */
template <class T>
void Histogram2D<T>::insert(const std::vector<std::pair<double,double> > &samples)
{
  int cidx, ridx;

  for (unsigned i=0; i<samples.size(); i++)
  {
    const std::pair<double,double> &sample = samples[i];
    cidx = int( (sample.second + coffs) * cscale );
    
    if (cidx>=0 && cidx < cols)
    {
      ridx = int( (sample.first+roffs) * rscale );
      if (ridx>=0 && ridx < rows )
      {
        data[ridx*rows+cidx]++;
        num_samples++;
      }
    }
  }
}



/*************************** INLINE METHODES **************************/
template <class T>
inline void Histogram2D<T>::setTo(const T &d)
{
  data.clear();
  data.resize(rows*cols, d);
}

template <class T>
inline short Histogram2D<T>::Row(const int idx) const
{
  return idx/cols;
}

template <class T>
inline short Histogram2D<T>::Col(const int idx) const
{
  return idx%cols;
}

template <class T>
inline int Histogram2D<T>::GetIdx(const short row, const short col) const
{
  return row*cols + col;
}

template <class T>
inline T& Histogram2D<T>::operator[](unsigned i)
{
  return data[i];
}

template <class T>
inline const T& Histogram2D<T>::operator[](unsigned i) const
{
  return data[i];
}

template <class T>
inline T& Histogram2D<T>::operator[](int i)
{
  return data[i];
}

template <class T>
inline const T& Histogram2D<T>::operator[](int i) const
{
  return data[i];
}

template <class T>
inline T& Histogram2D<T>::operator()(unsigned row, unsigned col)
{
  return data[row*rows+col];
}

template <class T>
inline const T& Histogram2D<T>::operator()(unsigned row, unsigned col) const
{
  return data[row*rows+col];
}

template <class T>
inline T& Histogram2D<T>::operator()(int row, int col)
{
  return data[row*rows+col];
}

template <class T>
inline const T& Histogram2D<T>::operator()(int row, int col) const
{
  return data[row*rows+col];
}

template <class T>
inline T& Histogram2D<T>::operator()(double data_row, double data_col)
{
  return data[ GetIdx(int((data_row+roffs)*rscale), int((data_col+coffs)*cscale)) ];
}

template <class T>
inline const T& Histogram2D<T>::operator()(double data_row, double data_col) const
{
  return data[ GetIdx(int((data_row+roffs)*rscale), int((data_col+coffs)*cscale)) ];
}

template <class T>
inline bool Histogram2D<T>::insert(const double &data_row, const double &data_col)
{
  int ridx = int( (data_row+roffs) * rscale );

  if (ridx>=0 && ridx<rows)
  {
    int cidx = int( (data_col+coffs) * cscale );

    if (cidx>=0 && cidx<cols)
    {
      data[ridx*rows+cidx]++;
      num_samples++;
      return true;
    }
  }
  return false;
}

/****************************** io ***************************/
template <class T>
void Histogram2D<T>::write(const std::string &filename, const Histogram2D<T> &hist)
{
  std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
  write(out, hist);
  out.close();
}

template <class T>
void Histogram2D<T>::write(std::ofstream &out, const Histogram2D<T> &hist)
{
  out.write((char*)&hist.rmin, sizeof(double));
  out.write((char*)&hist.rmax, sizeof(double));
  out.write((char*)&hist.cmin, sizeof(double));
  out.write((char*)&hist.cmax, sizeof(double));
  out.write((char*)&hist.roffs, sizeof(double));
  out.write((char*)&hist.coffs, sizeof(double));
  out.write((char*)&hist.rscale, sizeof(double));
  out.write((char*)&hist.cscale, sizeof(double));
  out.write((char*)&hist.num_samples, sizeof(int));
  
  int tmp = sizeof(T);
  out.write((char*)&tmp, sizeof(int));
  out.write((char*)&hist.rows, sizeof(int));
  out.write((char*)&hist.cols, sizeof(int));
  out.write((char*)&hist.data[0], sizeof(T)*hist.data.size());
}

template <class T>
void Histogram2D<T>::read(const std::string &filename, Histogram2D<T> &hist)
{

  std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);

  if (in.is_open())
  {
    read(in, hist);
    in.close();
  }
  else throw std::runtime_error("[Histogram2D::read] File not found!");
}

template <class T>
void Histogram2D<T>::read(std::ifstream &in, Histogram2D<T> &hist)
{
  in.read((char*)&hist.rmin, sizeof(double));
  in.read((char*)&hist.rmax, sizeof(double));
  in.read((char*)&hist.cmin, sizeof(double));
  in.read((char*)&hist.cmax, sizeof(double));
  in.read((char*)&hist.roffs, sizeof(double));
  in.read((char*)&hist.coffs, sizeof(double));
  in.read((char*)&hist.rscale, sizeof(double));
  in.read((char*)&hist.cscale, sizeof(double));
  in.read((char*)&hist.num_samples, sizeof(int));

  int tmp;
  in.read((char*)&tmp, sizeof(int));

  if (tmp != sizeof(T))
    throw std::runtime_error("[Histogram2D::read] Wrong data type!");

  in.read((char*)&hist.rows, sizeof(int));
  in.read((char*)&hist.cols, sizeof(int));

  hist.data.resize(hist.rows*hist.cols);

  in.read((char*)&hist.data[0], sizeof(T)*hist.data.size());
}

} //--END--


#endif

