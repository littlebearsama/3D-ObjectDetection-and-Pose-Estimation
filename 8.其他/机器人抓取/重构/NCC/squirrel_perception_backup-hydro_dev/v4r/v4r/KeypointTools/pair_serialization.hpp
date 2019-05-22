/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_PAIR_SERI_HPP
#define KP_PAIR_SERI_HPP


#include <opencv2/opencv.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>


namespace boost {
    namespace serialization {

    template<class Archive, typename F, typename S>
    void serialize(Archive & ar, std::pair<F,S> &t, const unsigned int version)
    {
      ar & t.first;
      ar & t.second;
    }

  }
}

#endif
