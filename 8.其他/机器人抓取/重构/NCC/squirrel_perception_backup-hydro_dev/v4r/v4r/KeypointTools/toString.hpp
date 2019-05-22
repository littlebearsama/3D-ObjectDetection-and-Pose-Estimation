/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_TO_STRING_HH
#define KP_TO_STRING_HH

#include <iostream>
#include <iomanip>

namespace kp
{

template <class T>
inline std::string toString (const T& t, unsigned precision=2)
{
  std::stringstream ss;
  ss << std::setprecision(precision) << t;
  return ss.str();
}


}

#endif
