/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */


#ifndef PYRAMID_SIMPLE_HPP
#define PYRAMID_SIMPLE_HPP

#include "pyramidBase.hpp"

namespace AttentionModule
{

class SimplePyramid: public BasePyramid
{
public:
  SimplePyramid();
  virtual ~SimplePyramid();
  typedef boost::shared_ptr<SimplePyramid> Ptr;

  virtual void reset();
  virtual void combinePyramid(bool standard = false);

private:

};

}
#endif //PYRAMID_SIMPLE_HPP