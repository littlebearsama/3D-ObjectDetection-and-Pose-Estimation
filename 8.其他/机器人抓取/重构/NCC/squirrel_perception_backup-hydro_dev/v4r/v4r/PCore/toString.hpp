
#ifndef P_TO_STRING_HH
#define P_TO_STRING_HH

#include <iostream>
#include <iomanip>

namespace P
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
