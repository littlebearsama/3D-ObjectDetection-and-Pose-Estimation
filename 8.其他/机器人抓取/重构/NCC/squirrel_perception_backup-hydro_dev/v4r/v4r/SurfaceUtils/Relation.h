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
 * @file Relation.h
 * @author Richtsfeld
 * @date November 2011
 * @version 0.1
 * @brief Struct relation
 */

#ifndef SURFACE_RELATION_H
#define SURFACE_RELATION_H

#include <vector>

namespace surface
{

enum RelationModel {
  ALL_RELATIONS        = 0,
  STRUCTURAL_RELATIONS,
  ASSEMBLY_RELATIONS,
};
  
struct Relation
{
  int type;                                ///< Type of relation (structural level = 1 / assembly level = 2)
  int id_0;                                ///< ID of first surface 
  int id_1;                                ///< ID of second surface
  std::vector<double> rel_value;           ///< relation values (feature vector)
  std::vector<double> rel_probability;     ///< probabilities of correct prediction (two class)
  int groundTruth;                         ///< 0=false / 1=true
  int prediction;                          ///< 0=false / 1=true
  bool valid;                              ///< validity flag
};

}

#endif
