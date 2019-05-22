/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
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
 * @file Edge.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Edge definitions for the graph.
 */

#ifndef GC_EDGE_H
#define GC_EDGE_H

namespace gc
{
 
struct Edge
{
  int type;   /// type of node
  float w;    /// weightning factor
  float w2;   /// optional second weightning factor
  int a, b;   /// node numbers
};

}

#endif

