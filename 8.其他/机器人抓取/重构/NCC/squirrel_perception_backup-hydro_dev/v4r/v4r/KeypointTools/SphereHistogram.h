/**
 * $Id$
 */

#ifndef KP_SPHERE_HISTOGRAM_HH
#define KP_SPHERE_HISTOGRAM_HH

#include <set>
#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <stdexcept>


namespace kp
{

template <class T>
class SFace
{
public:
  unsigned vs[3];                       // vertices
  std::vector<SFace<T>*> sub_faces;     // links to 4 subdivided faces
  SFace<T>* neighbours[3];

  Eigen::Vector3f n;                  // normal

  T data;

  SFace() {};
  ~SFace(){
    for (unsigned i=0; i<sub_faces.size(); i++)
      delete sub_faces[i];
  }
};


template <class T>
class SphereHistogram
{
protected:
  std::vector<Eigen::Vector3f> vertices;
  std::vector<SFace<T> *> start_faces;     // 20 icosahedron faces
  std::vector<SFace<T> *> subdiv_faces;    // subdevided faces depending on init

  unsigned edge_walk, num_vertices;
  std::vector<unsigned> start, end, midpoint; //just temp container 

  void clear();
  void init(unsigned num_subdiv);
  void initIcosahedron();
  void subdevide();
  unsigned searchMidpoint (unsigned idx_start, unsigned idx_end);
  void computeNormals();
  const SFace<T>& findFace(const Eigen::Vector3f &n, const std::vector<SFace<T>*> &faces) const;
  void setNeighbours();
  void copyFace(SFace<T> *src, SFace<T> *dst, std::vector<SFace<T>*> &subdiv);
  void deepCopy(const SphereHistogram<T> *src, SphereHistogram<T> *dst);

  inline void planeExp2Normal(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, Eigen::Vector3f &n);

public:

  SphereHistogram(unsigned num_subdiv=1);
  ~SphereHistogram();

  T& get(const Eigen::Vector3f &n);
  const T& get(const Eigen::Vector3f &n) const;
  const SFace<T> &getFace(const Eigen::Vector3f &n) const;
 
  inline unsigned size(){return subdiv_faces.size();}
  inline std::vector<Eigen::Vector3f> &getVertices() {return vertices;}
  inline T& get(unsigned i);
  inline const T& get(unsigned i) const;
  inline const SFace<T> &getFace(unsigned i)const;

  SphereHistogram& operator=(const SphereHistogram &sphere);
};



/*********************** INLINE METHODES **************************/

template <class T>
inline void SphereHistogram<T>::planeExp2Normal(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2, const Eigen::Vector3f &p3, Eigen::Vector3f &n)
{
  n[0] = ( p2[1] - p1[1] ) * ( p3[2] - p1[2] )
       - ( p2[2] - p1[2] ) * ( p3[1] - p1[1] );

  n[1] = ( p2[2] - p1[2] ) * ( p3[0] - p1[0] )
       - ( p2[0] - p1[0] ) * ( p3[2] - p1[2] );

  n[2] = ( p2[0] - p1[0] ) * ( p3[1] - p1[1] )
       - ( p2[1] - p1[1] ) * ( p3[0] - p1[0] );

  n.normalize();
}

template <class T>
inline T& SphereHistogram<T>::get(unsigned i)
{
  return subdiv_faces[i]->data;
}

template <class T>
inline const T& SphereHistogram<T>::get(unsigned i) const
{
  return subdiv_faces[i]->data;
}

template <class T>
inline const SFace<T> & SphereHistogram<T>::getFace(unsigned i) const
{
  return *subdiv_faces[i]; 
}



}

#include "SphereHistogram.hpp"

#endif

