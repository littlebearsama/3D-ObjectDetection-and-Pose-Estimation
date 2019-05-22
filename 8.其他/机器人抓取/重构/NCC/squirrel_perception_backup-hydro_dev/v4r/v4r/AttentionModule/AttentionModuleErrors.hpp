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


#ifndef AM_ERRORS_HPP
#define AM_ERRORS_HPP

namespace AttentionModule
{

enum AttentionModuleErrors
{
  AM_OK                  = 0,
  AM_POINTCLOUD          = 1,
  AM_IMAGE,
  AM_PLANECOEFFICIENTS,
  AM_CAMERAPARAMETRS,
  AM_NORMALCLOUD,
  AM_DIFFERENTSIZES,
  AM_NORMALCOEFFICIENTS,
  AM_CURVATURECLOUD,
  AM_ZEROSIZE,
  AM_PARAMETERS,
  AM_DEPTH,
  AM_CUSTOM,
};

enum CombinationTypeEnum
{
  AM_COMB_SUM = 0,
  AM_COMB_MUL = 1,
  AM_COMB_MAX,
};

} //namespace AttentionModule

#endif // AM_ERRORS_HPP
