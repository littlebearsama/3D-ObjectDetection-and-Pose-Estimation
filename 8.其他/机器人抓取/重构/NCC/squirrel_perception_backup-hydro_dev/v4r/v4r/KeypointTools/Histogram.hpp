/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_HISTOGRAM_HH
#define KP_HISTOGRAM_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <stdexcept>

namespace kp 
{

template <class T=unsigned>
class Histogram
{
private:
  double min, max;
  double offs;
  double scale;

  T num_samples;

  std::vector<T> bins;


public:
  
  Histogram(const int &_nb_bins=100, const double &_min=0., const double &_max=1.);
  ~Histogram();

  void insert(const std::vector<double> &data);
  inline bool insert(const double &data);
  void normalize(const T &value=100);

  inline int size() {return bins.size(); }
  inline int numBins() {return bins.size();}
  inline T numSamples() {return num_samples;}
  inline double getMin() {return min;}
  inline double getMax() {return max;}
  inline void countSamples();

  static void getMinMax(const std::vector<double> &data, double &min, double &max);
  static void getMinMax(const std::vector<double> &d1, const std::vector<double> &d2, double &min, double &max);


  T& operator[](unsigned i);
  const T& operator[](unsigned i) const;
  T& operator[](int i);
  const T& operator[](int i) const;
  T& operator[](double d);
  const T& operator[](double d) const;

  static void write(const std::string &filename, const Histogram<T> &hist);
  static void write(std::ofstream &out, const Histogram<T> &hist);
  static void read(const std::string &filename, Histogram<T> &hist);
  static void read(std::ifstream &in, Histogram<T> &hist);

  typedef SmartPtr< ::kp::Histogram<T> > Ptr;
  typedef SmartPtr< ::kp::Histogram<T>  const> ConstPtr;
};


/**
 * Histogram1Di
 */
class Histogram1Di : public Histogram<int>
{
public:
  Histogram1Di(const int &_nb_bins=100, const double &_min=0., const double &_max=1.)
  : Histogram<int>(_nb_bins, _min, _max)
  {
  }
};

/**
 * Histogram1Df
 */
class Histogram1Df : public Histogram<float>
{
public:
  Histogram1Df(const int &_nb_bins=100, const double &_min=0., const double &_max=1.)
  : Histogram<float>(_nb_bins, _min, _max)
  {
  }
};

/**
 * Histogram1Dd
 */
class Histogram1Dd : public Histogram<double>
{
public:
  Histogram1Dd(const int &_nb_bins=100, const double &_min=0., const double &_max=1.)
  : Histogram<double>(_nb_bins, _min, _max)
  {
  }
};

/************************************************************************************
 * Constructor/Destructor
 */
template <class T>
Histogram<T>::Histogram(const int &_nb_bins, const double &_min, const double &_max)
 : min(_min), max(_max), num_samples(0)
{
  offs = -_min;
  scale = double(_nb_bins)/(_max-_min);

  bins.resize(_nb_bins,0);
}

template <class T>
Histogram<T>::~Histogram()
{
}

/**
 * insert
 */
template <class T>
void Histogram<T>::insert(const std::vector<double> &data)
{
  int idx;

  for (unsigned i=0; i<data.size(); i++)
  {
    idx = int( (data[i]+offs) * scale );

    if (idx>=0 && idx<(int)bins.size())
    {
      bins[idx]++;
      num_samples+=1;
    }
  }
}

/**
 * normalize
 */
template <class T>
void Histogram<T>::normalize(const T &value)
{
  T max=0;

  for (unsigned i=0; i<bins.size(); i++)
    if (max<bins[i])
      max = bins[i];

  if (max==0)
    return;

  float norm = float(value)/float(max);

  for (unsigned i=0; i<bins.size(); i++)
    bins[i] *= norm;
}


/******************************** STATIC METHODES ***********************
 * getMinMax
 */
template <class T>
void Histogram<T>::getMinMax(const std::vector<double> &data, double &min, double &max)
{
  min = DBL_MAX;
  max = -DBL_MAX;

  for (unsigned i=0; i<data.size(); i++)
  {
    const double &d = data[i];
    if (d<min) min = d;
    if (d>max) max = d;
  }
}

/**
 * getMinMax
 */
template <class T>
void Histogram<T>::getMinMax(const std::vector<double> &d1, const std::vector<double> &d2, double &min, double &max)
{
  min = DBL_MAX;
  max = -DBL_MAX;

  for (unsigned i=0; i<d1.size(); i++)
  {
    const double &d = d1[i];
    if (d<min) min = d;
    if (d>max) max = d;
  }
  for (unsigned i=0; i<d2.size(); i++)
  {
    const double &d = d2[i];
    if (d<min) min = d;
    if (d>max) max = d;
  }
}


/*************************** INLINE METHODES **************************/
template <class T>
inline T& Histogram<T>::operator[](unsigned i)
{
  return bins[i];
}

template <class T>
inline const T& Histogram<T>::operator[](unsigned i) const
{
  return bins[i];
}

template <class T>
inline T& Histogram<T>::operator[](int i)
{
  return bins[i];
}

template <class T>
inline const T& Histogram<T>::operator[](int i) const
{
  return bins[i];
}

template <class T>
inline T& Histogram<T>::operator[](double d)
{
  return bins[int( (d+offs) * scale )];
}

template <class T>
inline const T& Histogram<T>::operator[](double d) const
{
  return bins[int( (d+offs) * scale )];
}

template <class T>
inline bool Histogram<T>::insert(const double &data)
{
  int idx = int( (data+offs) * scale );

  if (idx>=0 && idx<(int)bins.size())
  {
    bins[idx]++;
    num_samples+=1;
    return true;
  }
  return false;
}

template <class T>
inline void Histogram<T>::countSamples() 
{
  num_samples=0; 
  for (unsigned i=0; i<bins.size(); i++)
    num_samples += bins[i];
}


/******************** IO ***************************/
template <class T>
void Histogram<T>::write(const std::string &filename, const Histogram<T> &hist)
{
  std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
  write(out, hist);
  out.close();
}

template <class T>
void Histogram<T>::write(std::ofstream &out, const Histogram<T> &hist)
{
  int tmp = sizeof(T);
  out.write((char*)&tmp, sizeof(int));
  out.write((char*)&hist.min, sizeof(double));
  out.write((char*)&hist.max, sizeof(double));
  out.write((char*)&hist.offs, sizeof(double));
  out.write((char*)&hist.scale, sizeof(double));
  tmp = hist.bins.size();
  out.write((char*)&tmp, sizeof(int));
  out.write((char*)&hist.bins[0], sizeof(T)*hist.bins.size());
}

template <class T>
void Histogram<T>::read(const std::string &filename, Histogram<T> &hist)
{

  std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);

  if (in.is_open())
  {
    read(in, hist);
    in.close();
  }
  else throw std::runtime_error("[Histogram::read] File not found!");

}

template <class T>
void Histogram<T>::read(std::ifstream &in, Histogram<T> &hist)
{
  int tmp;
  in.read((char*)&tmp, sizeof(int));

  if (tmp != sizeof(T))
    throw std::runtime_error("[Histogram::read] Wrong data type!");

  in.read((char*)&hist.min, sizeof(double));
  in.read((char*)&hist.max, sizeof(double));
  in.read((char*)&hist.offs, sizeof(double));
  in.read((char*)&hist.scale, sizeof(double));
  in.read((char*)&tmp, sizeof(int));

  hist.bins.resize(tmp);

  in.read((char*)&hist.bins[0], sizeof(T)*hist.bins.size());

  hist.countSamples();
}


} //--END--


#endif

