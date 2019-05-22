/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


namespace kp 
{




/********************** SphereHistogram ************************
 * Constructor/Destructor
 */
template <class T>
SphereHistogram<T>::SphereHistogram(unsigned num_subdiv)
{
  init(num_subdiv);
}

template <class T>
SphereHistogram<T>::~SphereHistogram()
{
  clear();
}

/**
 * Init icosahedron
 */
template <class T>
void SphereHistogram<T>::initIcosahedron()
{
  double t = (1.+sqrt(5.))/2.;
  double tau = t/sqrt(1.+t*t);
  double one = 1./sqrt(1.+t*t);

  vertices.resize(12);

  vertices[0] = Eigen::Vector3f(tau, one, 0.0);
  vertices[1] = Eigen::Vector3f(-tau, one, 0.0);
  vertices[2] = Eigen::Vector3f(-tau, -one, 0.0);
  vertices[3] = Eigen::Vector3f(tau, -one, 0.0);
  vertices[4] = Eigen::Vector3f(one, 0.0 ,  tau);
  vertices[5] = Eigen::Vector3f(one, 0.0 , -tau);
  vertices[6] = Eigen::Vector3f(-one, 0.0 , -tau);
  vertices[7] = Eigen::Vector3f(-one, 0.0 , tau);
  vertices[8] = Eigen::Vector3f(0.0 , tau, one);
  vertices[9] = Eigen::Vector3f(0.0 , -tau, one);
  vertices[10] = Eigen::Vector3f(0.0 , -tau, -one);
  vertices[11] = Eigen::Vector3f(0.0 , tau, -one);

  unsigned icosahedron_faces[] = {
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

  start_faces.resize(20);
  for (unsigned i=0; i<20; i++)
  {
    start_faces[i] = new SFace<T>();
    memcpy ((void*)&start_faces[i]->vs[0], (void*)&icosahedron_faces[i*3], 3*sizeof(unsigned));
  }

  subdiv_faces = start_faces;
}


/**
 * SearchMidpoint
 */
template <class T>
unsigned SphereHistogram<T>::searchMidpoint (unsigned idx_start, unsigned idx_end)
{
  for (unsigned i=0; i<edge_walk; i++)
    if ((start[i] == idx_start && end[i] == idx_end) ||
        (start[i] == idx_end && end[i] == idx_start))
    {
      unsigned res = midpoint[i];

      // update the arrays
      start[i]    = start[edge_walk-1];
      end[i]      = end[edge_walk-1];
      midpoint[i] = midpoint[edge_walk-1];
      edge_walk--;

      return res;
    }

  // vertex not in the list, so we add it
  start[edge_walk] = idx_start;
  end[edge_walk] = idx_end;
  midpoint[edge_walk] = vertices.size();

  // create new vertex
  vertices.push_back( vertices[idx_start]+vertices[idx_end] );

  // normalize the new vertex
  vertices.back().normalize();
  edge_walk++;
  return midpoint[edge_walk-1];
}


/**
 * Subdevide face
 */
template <class T>
void SphereHistogram<T>::subdevide()
{
  edge_walk = 0;
  unsigned num_edges = 2*vertices.size() + 3*subdiv_faces.size();

  start.resize(num_edges);
  end.resize(num_edges);
  midpoint.resize(num_edges);

  std::vector<SFace<T>*> old_faces = subdiv_faces;
  subdiv_faces.resize(4*old_faces.size());

  unsigned z=0;
  for (unsigned i=0; i<old_faces.size(); i++)
  {
    unsigned a = old_faces[i]->vs[0];
    unsigned b = old_faces[i]->vs[1];
    unsigned c = old_faces[i]->vs[2];

    unsigned ab_midpoint = searchMidpoint(b, a);
    unsigned bc_midpoint = searchMidpoint(c, b);
    unsigned ca_midpoint = searchMidpoint(a, c);

    subdiv_faces[z] = new SFace<T>();
    subdiv_faces[z]->vs[0] = a;
    subdiv_faces[z]->vs[1] = ab_midpoint;
    subdiv_faces[z]->vs[2] = ca_midpoint;
    old_faces[i]->sub_faces.push_back(subdiv_faces[z]);
    z++;
    subdiv_faces[z] = new SFace<T>();
    subdiv_faces[z]->vs[0] = ca_midpoint;
    subdiv_faces[z]->vs[1] = ab_midpoint;
    subdiv_faces[z]->vs[2] = bc_midpoint;
    old_faces[i]->sub_faces.push_back(subdiv_faces[z]);
    z++;
    subdiv_faces[z] = new SFace<T>();
    subdiv_faces[z]->vs[0] = ca_midpoint;
    subdiv_faces[z]->vs[1] = bc_midpoint;
    subdiv_faces[z]->vs[2] = c;
    old_faces[i]->sub_faces.push_back(subdiv_faces[z]);
    z++;
    subdiv_faces[z] = new SFace<T>();
    subdiv_faces[z]->vs[0] = ab_midpoint;
    subdiv_faces[z]->vs[1] = b;
    subdiv_faces[z]->vs[2] = bc_midpoint;
    old_faces[i]->sub_faces.push_back(subdiv_faces[z]);
    z++;
  }

  start.clear();
  end.clear();
  midpoint.clear();
}

/**
 * Compute normals of the current subdevided faces
 */
template <class T>
void SphereHistogram<T>::computeNormals()
{
  for (unsigned i=0; i<subdiv_faces.size(); i++)
  {
    planeExp2Normal(vertices[subdiv_faces[i]->vs[0]],
                    vertices[subdiv_faces[i]->vs[1]],
                    vertices[subdiv_faces[i]->vs[2]],
                    subdiv_faces[i]->n);
  }
}

/**
 * Copy a face
 */
template <class T>
void SphereHistogram<T>::copyFace(SFace<T> *src, SFace<T> *dst, std::vector<SFace<T>*> &subdiv)
{
  dst->vs[0]=src->vs[0], dst->vs[1]=src->vs[1], dst->vs[2]=src->vs[2];
  dst->n = src->n;

  if (src->sub_faces.size()==0)
  {
    subdiv.push_back(dst);
  }
  else
  {
    for (unsigned i=0; i<src->sub_faces.size(); i++)
    {
      dst->sub_faces.push_back(new SFace<T>());
      copyFace(src->sub_faces[i], dst->sub_faces.back(), subdiv);
    }
  }
}

/**
 * deep copy of a SphereHistogram
 */
template <class T>
void SphereHistogram<T>::deepCopy(const SphereHistogram<T> *src, SphereHistogram<T> *dst)
{
  dst->clear();

  dst->vertices = src->vertices;
  for (unsigned i=0; i<src->start_faces.size(); i++)
  {
    dst->start_faces.push_back(new SFace<T>());
    copyFace(src->start_faces[i], dst->start_faces.back(), dst->subdiv_faces);
  }
}

/**
 * find match
 */
template <class T>
const SFace<T>& SphereHistogram<T>::findFace(const Eigen::Vector3f &n, const std::vector<SFace<T>*> &faces) const
{
  float tmp, min=-1;
  int idx=-1;
  
  for (unsigned i=0; i<faces.size(); i++)
  {
    tmp = n.dot(faces[i]->n);
    if (tmp>min)
    {
      min=tmp;
      idx=i;
    }
  }

  return *faces[idx];
}

/**
 * SetNeighbours
 */
template <class T>
void SphereHistogram<T>::setNeighbours()
{
  SFace<T> *face;
  double dist, dist1, dist2, dist3;

  for (unsigned i=0; i<subdiv_faces.size(); i++)
  {
    face = subdiv_faces[i];
    dist1 = dist2 = dist3 = 0.;
    for (unsigned j=0; j<subdiv_faces.size(); j++)
    {
      if (i!=j)
      {
        dist = subdiv_faces[i]->n.dot(subdiv_faces[j]->n);
        if (dist > dist1)
        {
          dist3 = dist2;
          dist2 = dist1;
          dist1 = dist;
          face->neighbours[2] = face->neighbours[1];
          face->neighbours[1] = face->neighbours[0];
          face->neighbours[0] = subdiv_faces[j];
        }
        else
        {
          if (dist > dist2)
          {
            dist3 = dist2;
            dist2 = dist;
            face->neighbours[2] = face->neighbours[1];
            face->neighbours[1] = subdiv_faces[j];
          }
          else
          {
            if (dist > dist3)
            {
              dist3 = dist;
              face->neighbours[2] = subdiv_faces[j];
            }
          }
        }
      }
    }
  }
}





/**
 * clear()
 */
template <class T>
void SphereHistogram<T>::clear()
{
  for (unsigned i=0; i<start_faces.size(); i++)
    delete start_faces[i];

  start_faces.clear();
  subdiv_faces.clear();
  vertices.clear();
}

/**
 * Init indexing sphere
 */
template <class T>
void SphereHistogram<T>::init(unsigned num_subdiv)
{
  clear();
  initIcosahedron();  

  computeNormals();
  
  for (unsigned i=0; i<num_subdiv; i++) {
    subdevide();
    computeNormals();
  }

  computeNormals();
  setNeighbours();
}




/************************* PUBLIC **********************************/

template <class T>
const SFace<T> & SphereHistogram<T>::getFace(const Eigen::Vector3f &n) const
{
  if (start_faces.size()==0)
    throw std::runtime_error("[SphereHistogram::get] Invalid SphereHistogram!");;

  SFace<T> *face = (SFace<T>*)&findFace(n,start_faces);

  while (face->sub_faces.size()!=0)
  {
    face = (SFace<T>*)&findFace(n,face->sub_faces);
  }

  return *face;
}

template <class T>
T& SphereHistogram<T>::get(const Eigen::Vector3f &n)
{
  if (start_faces.size()==0)
    throw std::runtime_error("[SphereHistogram::get] Invalid SphereHistogram!");;

  SFace<T> *face = (SFace<T>*)&findFace(n,start_faces);

  while (face->sub_faces.size()!=0)
  {
    face = (SFace<T>*)&findFace(n,face->sub_faces);
  }

  return face->data;
}

template <class T>
const T& SphereHistogram<T>::get(const Eigen::Vector3f &n) const
{
  if (start_faces.size()==0)
    throw std::runtime_error("[SphereHistogram::get] Invalid SphereHistogram!");;

  SFace<T> *face = (SFace<T>*)&findFace(n,start_faces);

  while (face->sub_faces.size()!=0)
  {
    face = (SFace<T>*)&findFace(n,face->sub_faces);
  }

  return face->data;
}


}

