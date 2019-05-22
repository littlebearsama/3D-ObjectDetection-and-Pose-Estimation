/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
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

/**
 * @file BoundaryRelationsMeanCurvature.hpp
 * @author Potapova
 * @date August 2013
 * @version 0.1
 * @brief Class to calculate boundary curvature and standart deviation.
 */

#ifndef BOUNDARY_RELATIONS_MEANCURVATURE_H
#define BOUNDARY_RELATIONS_MEANCURVATURE_H

#include "BoundaryRelationsBase.hpp"

namespace surface
{

class BoundaryRelationsMeanCurvature : public BoundaryRelationsBase
{
public:

protected:

private:

public:

  typedef boost::shared_ptr<BoundaryRelationsMeanCurvature> Ptr;

  BoundaryRelationsMeanCurvature();
  ~BoundaryRelationsMeanCurvature();

  //@ep: not so sure that the type of the return argument is the best
  virtual surface::meanVal compute();
};

}

#endif //BOUNDARY_RELATIONS_MEANCURVATURE_H

