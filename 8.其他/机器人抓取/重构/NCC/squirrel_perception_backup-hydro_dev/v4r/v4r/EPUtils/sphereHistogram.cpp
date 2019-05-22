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

#include "sphereHistogram.hpp"


namespace EPUtils 
{
/********************** SphereHistogram ************************
 * Constructor/Destructor
 */
SphereHistogram::SphereHistogram()
{
  InitIcosahedron();
}

/**
 * Init icosahedron
 */
void SphereHistogram::InitIcosahedron()
{
  double t = (1.+sqrt(5.))/2.;
  double tau = t/sqrt(1.+t*t);
  double one = 1./sqrt(1.+t*t);

  vertices.resize(12);

  vertices[0] = cv::Point3d(tau, one, 0.0);
  vertices[1] = cv::Point3d(-tau, one, 0.0);
  vertices[2] = cv::Point3d(-tau, -one, 0.0);
  vertices[3] = cv::Point3d(tau, -one, 0.0);
  vertices[4] = cv::Point3d(one, 0.0 ,  tau);
  vertices[5] = cv::Point3d(one, 0.0 , -tau);
  vertices[6] = cv::Point3d(-one, 0.0 , -tau);
  vertices[7] = cv::Point3d(-one, 0.0 , tau);
  vertices[8] = cv::Point3d(0.0 , tau, one);
  vertices[9] = cv::Point3d(0.0 , -tau, one);
  vertices[10] = cv::Point3d(0.0 , -tau, -one);
  vertices[11] = cv::Point3d(0.0 , tau, -one);

  unsigned int icosahedron_faces[] = {
              4, 8, 7,
              4, 7, 9,
              5, 6, 11,
              5, 10, 6,
              0, 4, 3,
              0, 3, 5,
              2, 7, 1,
              2, 1, 6,
              8, 0, 11,
              8, 11, 1,
              9, 10, 3,
              9, 2, 10,
              8, 4, 0,
              11, 0, 5,
              4, 9, 3,
              5, 3, 10,
              7, 8, 1,
              6, 1, 11,
              7, 2, 9,
              6, 10, 2};

  faces.clear();
  faces.resize(20);
  for (unsigned int i=0; i<20; i++)
  {
    FacePatch face;
    face.vs[0] = icosahedron_faces[i*3 + 0];
    face.vs[1] = icosahedron_faces[i*3 + 1];
    face.vs[2] = icosahedron_faces[i*3 + 2];
    
    faces.at(i) = face;
  }
}

/**
 * SearchMidpoint
 */
unsigned int SphereHistogram::AddMidpoint (unsigned v1, unsigned v2)
{
  unsigned int t1, t2;
  t1 = v1;
  t2 = v2;
  v1 = std::min(t1,t2);
  v2 = std::max(t1,t2);
  
  unsigned int new_v;
  if(findEdge(v1,v2,new_v))
  {
    return(new_v);
  }
  
  cv::Point3d vertice1 = vertices.at(v1);
  cv::Point3d vertice2 = vertices.at(v2);
  float x = (vertice1.x + vertice2.x)/2;
  float y = (vertice1.y + vertice2.y)/2;
  float z = (vertice1.z + vertice2.z)/2;
  cv::Point3d new_vertice(x,y,z);
  // normalize vertix to 1
  normalizeVector(new_vertice);

  // create new vertex
  vertices.push_back(new_vertice);

  v1v2new_v temp;
  temp.v1 = v1;
  temp.v2 = v2;
  temp.new_v = vertices.size()-1;
  checkedVertices.push_back(temp);
  
  return(temp.new_v);
}

bool SphereHistogram::findEdge(unsigned int v1,unsigned int v2,unsigned int &new_v)
{
  for(unsigned int i = 0; i < checkedVertices.size(); ++i)
  {
    if((checkedVertices.at(i).v1 == v1) && (checkedVertices.at(i).v2 == v2))
    {
      new_v = checkedVertices.at(i).new_v;
      return(true);
    }
  }
  return(false);
}

/**
 * Subdevide face
 */
void SphereHistogram::SubdevideFace(FacePatch &face, std::vector<FacePatch> &newFaces)
{
  unsigned int v1 = face.vs[0];
  unsigned int v2 = face.vs[1];
  unsigned int v3 = face.vs[2];
  
  unsigned v1v2_midpoint = AddMidpoint(v1, v2);
  unsigned v2v3_midpoint = AddMidpoint(v2, v3);
  unsigned v1v3_midpoint = AddMidpoint(v1, v3);
  
  newFaces.clear();
  newFaces.resize(4);
  
  newFaces.at(0).vs[0] = v1;
  newFaces.at(0).vs[1] = v1v2_midpoint;
  newFaces.at(0).vs[2] = v1v3_midpoint;
  
  newFaces.at(1).vs[0] = v2;
  newFaces.at(1).vs[1] = v1v2_midpoint;
  newFaces.at(1).vs[2] = v2v3_midpoint;
  
  newFaces.at(2).vs[0] = v1v2_midpoint;
  newFaces.at(2).vs[1] = v2v3_midpoint;
  newFaces.at(2).vs[2] = v1v3_midpoint;
  
  newFaces.at(3).vs[0] = v3;
  newFaces.at(3).vs[1] = v2v3_midpoint;
  newFaces.at(3).vs[2] = v1v3_midpoint;
}

void SphereHistogram::Subdevide()
{
  checkedVertices.clear();
  
  std::vector<FacePatch> new_faces;
  new_faces.clear();
  new_faces.resize(faces.size()*4);
  
  for(unsigned int i = 0; i < faces.size(); ++i)
  {
    std::vector<FacePatch> tempFaces;
    SubdevideFace(faces.at(i),tempFaces);
    
    new_faces.at(4*i+0) = tempFaces.at(0);
    new_faces.at(4*i+1) = tempFaces.at(1);
    new_faces.at(4*i+2) = tempFaces.at(2);
    new_faces.at(4*i+3) = tempFaces.at(3);
  }
  
  faces.clear();
  faces = new_faces;
  
}

/**
 * Compute normals of the current subdevided faces
 */
void SphereHistogram::ComputeNormals()
{
  for (unsigned int i = 0; i < faces.size(); i++)
  {
    cv::Point3d N;
    unsigned int v1 = faces.at(i).vs[0];
    unsigned int v2 = faces.at(i).vs[1];
    unsigned int v3 = faces.at(i).vs[2];
    calculatePlaneNormal(vertices.at(v1),vertices.at(v2),vertices.at(v3),faces.at(i).norm);
    
    //orient normal
    cv::Point3d cen;
    cen.x = (vertices.at(v1).x + vertices.at(v2).x + vertices.at(v3).x)/3;
    cen.y = (vertices.at(v1).y + vertices.at(v2).y + vertices.at(v3).y)/3;
    cen.z = (vertices.at(v1).z + vertices.at(v2).z + vertices.at(v3).z)/3;
    if(calculateCosine2(cen,faces.at(i).norm) < 0)
    {
      faces.at(i).norm.x = -faces.at(i).norm.x;
      faces.at(i).norm.y = -faces.at(i).norm.y;
      faces.at(i).norm.z = -faces.at(i).norm.z;
    }
  }
}



/**
 * find best match
 */
int SphereHistogram::FindMatch(cv::Point3d &n)
{
  float max_cosine = 0;
  int index = -1;
  
  for (unsigned int i=0; i<faces.size(); i++)
  {
    float cosine = calculateCosine2(n,faces.at(i).norm);
    if(cosine > max_cosine)
    {
      max_cosine = cosine;
      index = i;
    }
  }

  return(index);
}


} //namespace EPUtils

