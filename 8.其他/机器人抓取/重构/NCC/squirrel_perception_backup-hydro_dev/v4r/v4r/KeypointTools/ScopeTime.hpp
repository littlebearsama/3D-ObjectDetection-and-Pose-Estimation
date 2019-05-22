/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_SCOPE_TIME_HPP
#define KP_SCOPE_TIME_HPP

#include <cmath>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace kp
{

class ScopeTime
{
public:
  std::string title;
  boost::posix_time::ptime start;

  inline ScopeTime (const char* _title) : title(std::string(_title))
  {
    start = boost::posix_time::microsec_clock::local_time ();
  }

  inline ScopeTime () : title(std::string(""))
  {
    start = boost::posix_time::microsec_clock::local_time ();
  }

  inline ~ScopeTime ()
  {
    double val = this->getTime ();
    std::cout << title << ": " << val << "ms\n";
  }

  inline double getTime ()
  {
    boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time ();
    return (static_cast<double> (((end - start).total_milliseconds ())));
  }

  inline void reset ()
  {
    start = boost::posix_time::microsec_clock::local_time ();
  }
};

} //--END--

#endif




