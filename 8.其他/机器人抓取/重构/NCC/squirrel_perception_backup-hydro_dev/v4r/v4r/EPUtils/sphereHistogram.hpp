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

#ifndef EPSPHEREHISTOGRAM_HPP
#define EPSPHEREHISTOGRAM_HPP

#include "headers.hpp"
#include "convertions.hpp"

namespace EPUtils
{

struct v1v2new_v{
  unsigned int v1, v2, new_v;
};
  
class FacePatch
{
public:
  unsigned int vs[3];         // vertices
  
  cv::Point3d norm; // normal
  float weight; // what ever you want to accumulate
  
  FacePatch() : weight(0.) {};
};

class SphereHistogram
{
public:
  std::vector<cv::Point3d> vertices;
  std::vector<FacePatch> faces; // 20 icosahedron faces
  
  SphereHistogram();
  void Subdevide();
  void ComputeNormals();
  int FindMatch(cv::Point3d &n);
  
private:
  void InitIcosahedron();
  unsigned int AddMidpoint(unsigned v1, unsigned v2);
  void SubdevideFace(FacePatch &face, std::vector<FacePatch> &newFaces);
  bool findEdge(unsigned int v1,unsigned int v2,unsigned int &new_v);
  
  std::vector<v1v2new_v> checkedVertices;
};

} //namespace EPUtils

#endif //EPSPHEREHISTOGRAM_HPP