/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_DATA_MATRIX_2D_HPP
#define KP_DATA_MATRIX_2D_HPP

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <string.h>
#include "DataContainer.hh"
#include "SmartPtr.hpp"

namespace kp 
{

/**
 * DataMatrix2D
 */
template <class T>
class DataMatrix2D : public DataContainer
{
public:
  int rows, cols;
  std::vector<T> data;

  DataMatrix2D(const DataContainer::Type &_type=UNDEF);
  DataMatrix2D(const int _rows, const int _cols, const DataContainer::Type &_type=UNDEF);
  ~DataMatrix2D(){};

  T& operator[](unsigned i);
  const T& operator[](unsigned i) const;
  T& operator[](int i);
  const T& operator[](int i) const;
  T& operator()(int row, int col);
  const T& operator()(int row, int col) const;

  inline void clear();
  inline void resize(const int _rows, const int _cols);
  inline void reserve(const int _rows, const int _cols);
  virtual inline void setTo(const T &d);
  virtual inline void setZero(){std::cout<<"[DataMatrix2D::setZero] Not available!"<<std::endl;}
  virtual inline void push_back(const std::vector<T> &datarow);
  virtual inline void push_back(const T *d, const int _cols);
  virtual inline void setRow(const int row, const std::vector<T> &datarow);
  virtual inline void getRow(const int row, std::vector<T> &datarow) const;

  inline int GetIdx(const int row, const int col) const;
  inline int Row(const int idx) const;
  inline int Col(const int idx) const;

  static void write(const std::string &filename, const DataMatrix2D<T> &mat);
  static void write(std::ofstream &out, const DataMatrix2D<T> &mat);
  static void read(const std::string &filename, DataMatrix2D<T> &mat);
  static void read(std::ifstream &in, DataMatrix2D<T> &mat);

  typedef SmartPtr< ::kp::DataMatrix2D<T> > Ptr;
  typedef SmartPtr< ::kp::DataMatrix2D<T> const> ConstPtr;
};


/**
 * DataMatrix2Df
 */
class DataMatrix2Df : public DataMatrix2D<float>
{
public:
  DataMatrix2Df() : DataMatrix2D(FLOAT_2D){}
  DataMatrix2Df(const int _rows, const int _cols) : DataMatrix2D<float>(_rows,_cols, FLOAT_2D) {}
  virtual inline void push_back(const std::vector<float> &datarow);
  virtual inline void push_back(const float *d, const int _cols);
  virtual inline void setRow(const int row, const std::vector<float> &datarow);
  virtual inline void getRow(const int row, std::vector<float> &datarow) const;
  virtual inline void setTo(const float &d);
  virtual inline void setZero();

  typedef SmartPtr< ::kp::DataMatrix2Df > Ptr;
  typedef SmartPtr< ::kp::DataMatrix2Df const> ConstPtr;
};

/**
 * DataMatrix2Db
 */
class DataMatrix2Db : public DataMatrix2D<unsigned char>
{
public:
  DataMatrix2Db() : DataMatrix2D(UCHAR_2D) {}
  DataMatrix2Db(const int _rows, const int _cols) : DataMatrix2D<unsigned char>(_rows,_cols,UCHAR_2D) {}
  virtual inline void push_back(const std::vector<unsigned char> &datarow);
  virtual inline void push_back(const unsigned char *d, const int _cols);
  virtual inline void setRow(const int row, const std::vector<unsigned char> &datarow);
  virtual inline void getRow(const int row, std::vector<unsigned char> &datarow) const;
  virtual inline void setTo(const unsigned char &d);
  virtual inline void setZero();

  typedef SmartPtr< ::kp::DataMatrix2Db > Ptr;
  typedef SmartPtr< ::kp::DataMatrix2Db const> ConstPtr;
};

/**
 * DataMatrix2Di
 */
class DataMatrix2Di : public DataMatrix2D<int>
{
public:
  DataMatrix2Di() : DataMatrix2D(INT_2D) {}
  DataMatrix2Di(const int _rows, const int _cols) : DataMatrix2D<int>(_rows,_cols, INT_2D) {}
  virtual inline void push_back(const std::vector<int> &datarow);
  virtual inline void push_back(const int *d, const int _cols);
  virtual inline void setRow(const int row, const std::vector<int> &datarow);
  virtual inline void getRow(const int row, std::vector<int> &datarow) const;
  virtual inline void setTo(const int &d);
  virtual inline void setZero();

  typedef SmartPtr< ::kp::DataMatrix2Di > Ptr;
  typedef SmartPtr< ::kp::DataMatrix2Di const> ConstPtr;
};

/*************************** INLINE METHODES **************************/

/********************** DataMatrix2D ***********************************
 * constructor
 */
template <class T>
DataMatrix2D<T>::DataMatrix2D(const DataContainer::Type &_type)
 :  DataContainer(_type), rows(0), cols(0)
{
}

template <class T>
DataMatrix2D<T>::DataMatrix2D(const int _rows, const int _cols, const DataContainer::Type &_type)
 :  DataContainer(_type), rows(_rows), cols(_cols)
{
  data.resize(rows*cols);
}

/************************* DataMatrix2D operators ***************************/

template <class T>
inline T& DataMatrix2D<T>::operator()(int row, int col)
{
  return data[row*cols+col];
}

template <class T>
inline const T& DataMatrix2D<T>::operator()(int row, int col) const
{
  return data[row*cols+col];
}

template <class T>
inline T& DataMatrix2D<T>::operator[](unsigned i)
{
  return data[i];
}

template <class T>
inline const T& DataMatrix2D<T>::operator[](unsigned i) const
{
  return data[i];
}

template <class T>
inline T& DataMatrix2D<T>::operator[](int i)
{
  return data[i];
}

template <class T>
inline const T& DataMatrix2D<T>::operator[](int i) const
{
  return data[i];
}

/************************ DataMatrix2D data handling ******************************/

template <class T>
inline void DataMatrix2D<T>::clear()
{
  data.clear();
  rows=cols=0;
}

template <class T>
inline void DataMatrix2D<T>::resize(const int _rows, const int _cols)
{
  rows = _rows;
  cols = _cols;
  data.resize(rows*cols); 
}

template <class T>
inline void DataMatrix2D<T>::reserve(const int _rows, const int _cols)
{
  cols = _cols;
  data.reserve(_rows*_cols);
}

template <class T>
inline void DataMatrix2D<T>::push_back(const std::vector<T> &datarow)
{
  data.resize(data.size()+datarow.size());

  for (unsigned i=0, z=rows*cols; i<datarow.size(); i++, z++)
    data[z] = datarow[i];

  rows++;
}

template <class T>
inline void DataMatrix2D<T>::push_back(const T *d, const int _cols)
{
  data.resize(data.size()+_cols);

  T* ptr = (T*)d;

  for (unsigned i=rows*cols; i<data.size(); i++, ptr++)
    data[i] = *ptr;

  rows++;
}

template <class T>
inline void DataMatrix2D<T>::setRow(const int row, const std::vector<T> &datarow)
{
  for (unsigned i=0, z=row*cols; i<datarow.size(); i++, z++)
    data[z] = datarow[i];
}

template <class T>
inline void DataMatrix2D<T>::getRow(const int row, std::vector<T> &datarow) const
{
  datarow.resize(cols);
  for (int i=0, z=row*cols; i<cols; i++, z++)
    datarow[i] = data[z];
}


template <class T>
inline void DataMatrix2D<T>::setTo(const T &d)
{
  data.clear();
  data.resize(rows*cols, d);
}

template <class T>
inline int DataMatrix2D<T>::Row(const int idx) const
{
  return idx/cols;
}

template <class T>
inline int DataMatrix2D<T>::Col(const int idx) const
{
  return idx%cols;
}

template <class T>
inline int DataMatrix2D<T>::GetIdx(const int row, const int col) const
{
  return row*cols + col;
}


/*********************** DataMatrix2D IO *****************************/

template <class T>
void DataMatrix2D<T>::write(const std::string &filename, const DataMatrix2D<T> &mat)
{
  std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary);
  write(out, mat);
  out.close();
}

template <class T>
void DataMatrix2D<T>::write(std::ofstream &out, const DataMatrix2D<T> &mat)
{
  int tmp = sizeof(T);
  out.write((char*)&tmp, sizeof(int)); 
  out.write((char*)&mat.rows, sizeof(int));
  out.write((char*)&mat.cols, sizeof(int));
  out.write((char*)&mat.data[0], sizeof(T)*mat.data.size());
}

template <class T>
void DataMatrix2D<T>::read(const std::string &filename, DataMatrix2D<T> &mat)
{

  std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);

  if (in.is_open())
  {
    read(in, mat);
    in.close();
  }
  else throw std::runtime_error("[DataMatrix2D::read] File not found!");

}

template <class T>
void DataMatrix2D<T>::read(std::ifstream &in, DataMatrix2D<T> &mat)
{
  int tmp;
  in.read((char*)&tmp, sizeof(int));

  if (tmp != sizeof(T))
    throw std::runtime_error("[DataMatrix2D::read] Wrong data type!");

  in.read((char*)&mat.rows, sizeof(int));
  in.read((char*)&mat.cols, sizeof(int));
  
  mat.data.resize(mat.rows*mat.cols);

  in.read((char*)&mat.data[0], sizeof(T)*mat.data.size());
}



/*********************************** DataMatrix2Df *************************/
inline void DataMatrix2Df::push_back(const std::vector<float> &datarow)
{
  data.resize(data.size()+datarow.size());
  memcpy ( &data[rows*cols], &datarow[0], sizeof(float)*datarow.size() );  
  rows++;
}

inline void DataMatrix2Df::push_back(const float *d, const int _cols)
{
  data.resize(data.size()+_cols);
  memcpy ( &data[rows*cols], d, sizeof(float)*_cols );
  rows++;
}

inline void DataMatrix2Df::setRow(const int row, const std::vector<float> &datarow)
{
  memcpy ( &data[row*cols], &datarow[0], sizeof(float)*datarow.size() );  
}

inline void DataMatrix2Df::getRow(const int row, std::vector<float> &datarow) const
{
  datarow.resize(cols);
  memcpy ( &datarow[0], &data[row*cols], sizeof(float)*cols );  
}

inline void DataMatrix2Df::setTo(const float &d)
{
  for (int i=0; i<cols; i++)
    data[i] = d;
  for (int i=1; i<rows; i++)
    memcpy ( &data[i*cols], &data[0], sizeof(float)*cols );
}

inline void DataMatrix2Df::setZero()
{
  memset ( &data[0], 0, sizeof(float)*data.size() );
}


/*********************************** DataMatrix2Db *************************/
inline void DataMatrix2Db::push_back(const std::vector<unsigned char> &datarow)
{
  data.resize(data.size()+datarow.size());
  memcpy ( &data[rows*cols], &datarow[0], sizeof(unsigned char)*datarow.size() );  
  rows++;
}

inline void DataMatrix2Db::push_back(const unsigned char *d, const int _cols)
{
  data.resize(data.size()+_cols);
  memcpy ( &data[rows*cols], d, sizeof(unsigned char)*_cols );
  rows++;
}

inline void DataMatrix2Db::setRow(const int row, const std::vector<unsigned char> &datarow)
{
  memcpy ( &data[row*cols], &datarow[0], sizeof(unsigned char)*datarow.size() );  
}

inline void DataMatrix2Db::getRow(const int row, std::vector<unsigned char> &datarow) const
{
  datarow.resize(cols);
  memcpy ( &datarow[0], &data[row*cols], sizeof(unsigned char)*cols );  
}

inline void DataMatrix2Db::setTo(const unsigned char &d)
{
  for (int i=0; i<cols; i++)
    data[i] = d;
  for (int i=1; i<rows; i++)
    memcpy ( &data[i*cols], &data[0], sizeof(unsigned char)*cols );
}

inline void DataMatrix2Db::setZero()
{
  memset ( &data[0], 0, sizeof(unsigned char)*data.size() );
}


/*********************************** DataMatrix2Di *************************/
inline void DataMatrix2Di::push_back(const std::vector<int> &datarow)
{
  data.resize(data.size()+datarow.size());
  memcpy ( &data[rows*cols], &datarow[0], sizeof(int)*datarow.size() );  
  rows++;
}

inline void DataMatrix2Di::push_back(const int *d, const int _cols)
{
  data.resize(data.size()+_cols);
  memcpy ( &data[rows*cols], d, sizeof(int)*_cols );
  rows++;
}

inline void DataMatrix2Di::setRow(const int row, const std::vector<int> &datarow)
{
  memcpy ( &data[row*cols], &datarow[0], sizeof(int)*datarow.size() );  
}

inline void DataMatrix2Di::getRow(const int row, std::vector<int> &datarow) const
{
  datarow.resize(cols);
  memcpy (&datarow[0], &data[row*cols], sizeof(int)*cols );  
}


inline void DataMatrix2Di::setTo(const int &d)
{
  for (int i=0; i<cols; i++)
    data[i] = d;
  for (int i=1; i<rows; i++)
    memcpy ( &data[i*cols], &data[0], sizeof(int)*cols );
}

inline void DataMatrix2Di::setZero()
{
  memset ( &data[0], 0, sizeof(int)*data.size() );
}



} //--END--

#endif

