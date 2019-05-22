/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2014 by Markus Bader <markus.bader@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/


#ifndef KP_SERIALIZATION_IO_FILE_H
#define KP_SERIALIZATION_IO_FILE_H


#include <boost/algorithm/string.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

namespace kp {


enum SerializeFormat{
  FORMAT_NA  = 0,
  FORMAT_XML = 1,
  FORMAT_BIN = 2,
  FORMAT_TXT = 3
};


inline SerializeFormat file_postix(const std::string &filename){
  size_t i = filename.find_last_of ( "." );
    if ( i == std::string::npos ) return FORMAT_NA;
    if ( filename.length() - i < 2) return FORMAT_NA;
    std::string postfix = filename.substr ( i+1, filename.length() );
    if ( boost::iequals ( postfix, "xml" ) )return FORMAT_XML;
    if ( boost::iequals ( postfix, "bin" ) )return FORMAT_BIN;
    if ( boost::iequals ( postfix, "txt" ) )return FORMAT_TXT;
    else return FORMAT_NA;    
}  

template<class T>
inline void write ( const std::string &filename, const T &src, SerializeFormat format) {
    std::ofstream ofs ( filename.c_str() );
    assert ( ofs.good() );
    if ( format == FORMAT_XML ) {
        boost::archive::xml_oarchive oa ( ofs );
        oa << boost::serialization::make_nvp ( "io", src );
    } else if ( format == FORMAT_TXT ) {
        boost::archive::text_oarchive oa ( ofs );
        oa << src;
    } else if ( format == FORMAT_BIN ) {
        boost::archive::binary_oarchive oa ( ofs );
        oa << src;
    }
}

template<class T>
inline T &read ( const std::string &filename, T &des, SerializeFormat format ) {
    std::ifstream ifs ( filename.c_str() );
    assert ( ifs.good() );
    if ( format == FORMAT_XML ) {
        boost::archive::xml_iarchive ia ( ifs );
        ia >> boost::serialization::make_nvp ( "io", des );
    } else if ( format == FORMAT_TXT ) {
        boost::archive::text_iarchive ia ( ifs );
        ia >> des;
    } else if ( format == FORMAT_BIN ) {
        boost::archive::binary_iarchive ia ( ifs );
        ia >> des;
    }
    return des;
}

  
template<class T>
inline void write ( const std::string &filename, const T &src ) {
    SerializeFormat format = file_postix(filename);
    write (filename, src, format);
}

template<class T>
inline T read ( const std::string &filename, T &des ) {
    SerializeFormat format = file_postix(filename);
    return read (filename, des, format);
}

};

#endif // KP_SERIALIZATION_IO_FILE_H
