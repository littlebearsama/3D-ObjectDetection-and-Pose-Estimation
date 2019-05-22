/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_DATACONTAINER_HH
#define KP_DATACONTAINER_HH

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <string.h>
#include "SmartPtr.hpp"

namespace kp 
{

/**
 * DataContainer
 */
class DataContainer
{
public:
  enum Type
  {
    FLOAT_2D,
    FLOAT3_2D,
    FLOAT4_2D,
    UCHAR_2D,
    INT_2D,
    UINT_2D,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

public:
  Type type;

  DataContainer(const Type &_type=UNDEF) {type=_type;};
  virtual ~DataContainer(){};

  typedef SmartPtr< ::kp::DataContainer > Ptr;
  typedef SmartPtr< ::kp::DataContainer const> ConstPtr;
};


} //--END--

#endif

