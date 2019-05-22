/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_TRIPLE_SERI_HPP
#define KP_TRIPLE_SERI_HPP


#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

#include "triple.hpp"

namespace boost {
    namespace serialization {

    template<class Archive, typename F, typename S, typename T>
    void serialize(Archive & ar, kp::triple<F,S,T> &t, const unsigned int version)
    {
      ar & t.first;
      ar & t.second;
      ar & t.third;
    }

  }
}

#endif
