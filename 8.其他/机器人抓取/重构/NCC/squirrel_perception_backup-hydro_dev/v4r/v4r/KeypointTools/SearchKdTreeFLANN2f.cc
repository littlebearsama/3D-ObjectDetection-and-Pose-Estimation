/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "SearchKdTreeFLANN2f.hh"


namespace kp 
{

using namespace std;


/********************** SearchKdTreeFLANN2f ************************
 * Constructor/Destructor
 */
SearchKdTreeFLANN2f::SearchKdTreeFLANN2f(const Parameter &p)
 : param(p), flann_index(NULL), data(NULL)
{
  param_k = ::flann::SearchParams (-1 , param.epsilon);
  param_radius = ::flann::SearchParams (-1 , param.epsilon, param.sorted);
}

SearchKdTreeFLANN2f::~SearchKdTreeFLANN2f()
{
  clean();
}




/************************** PRIVATE ************************/

/**
 * convertPointsToArray
 */
void SearchKdTreeFLANN2f::convertPointsToArray(const std::vector<Eigen::Vector2f> &points)
{
  if (points.size()==0)
    return;

  data = static_cast<float*>(malloc(points.size() * dim * sizeof(float)));
  float *ptr = data;
  index_mapping.reserve(points.size());
  identity_mapping = true;

  for (unsigned i = 0; i < points.size(); i++)
  {
    const Eigen::Vector2f &pt = points[i];
    if (isnan(pt[0]) || isnan(pt[1]) || isnan(pt[2]))
    {
      identity_mapping = false;
      continue;
    }

    index_mapping.push_back(i);

    *ptr = pt[0]; ptr++;
    *ptr = pt[1]; ptr++;
    *ptr = pt[2]; ptr++;
  }
}





/************************** PUBLIC *************************/

/**
 * setInputPoints
 */
void SearchKdTreeFLANN2f::setInputPoints(const std::vector<Eigen::Vector2f> &points)
{
  clean();
  dim = 2;

  convertPointsToArray(points);
  
  total_nr_points = static_cast<int>(index_mapping.size());

  flann_index = new FLANNIndex( flann::Matrix<float>(data, index_mapping.size(), dim),
                                flann::KDTreeSingleIndexParams(15) );
  flann_index->buildIndex();  
}

/**
 * nearestKSearch
 */
void SearchKdTreeFLANN2f::nearestKSearch(const Eigen::Vector2f &pt, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
{
  if (!data || isnan(pt[0]) || isnan(pt[1]) || isnan(pt[2]))
  {
    k_indices.clear();
    k_distances.clear();
    return;
  }

  if (k > total_nr_points)
    k = total_nr_points;

  k_indices.resize(k);
  k_distances.resize(k);

  std::vector<float> query(dim);
  for (int i=0; i<dim; i++)
    query[i] = pt[i];

  ::flann::Matrix<int> k_indices_mat(&k_indices[0], 1, k);
  ::flann::Matrix<float> k_distances_mat(&k_distances[0], 1, k);

  flann_index->knnSearch(flann::Matrix<float>(&query[0], 1, dim),
                         k_indices_mat, k_distances_mat, k, param_k);

  if (!identity_mapping)
  {
    for (int i=0; i < k; i++)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping[neighbor_index];
    }
  }

  k_indices.resize(k);
  k_distances.resize(k);
}

/**
 * radiusSearch
 */
void SearchKdTreeFLANN2f::radiusSearch(const Eigen::Vector2f &pt, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_dists, unsigned int max_nn)
{
  if (!data || isnan(pt[0]) || isnan(pt[1]) || isnan(pt[2]))
  {
    k_indices.clear();
    k_sqr_dists.clear();
    return;
  }

  std::vector<float> query(dim);
  for (int i=0; i<dim; i++)
    query[i] = pt[i];
  
  if (max_nn == 0 || max_nn > static_cast<unsigned int> (total_nr_points))
    max_nn = total_nr_points;

  std::vector<std::vector<int> > indices(1);
  std::vector<std::vector<float> > dists(1);

  ::flann::SearchParams params(param_radius);
  if (max_nn == static_cast<unsigned int>(total_nr_points))
    params.max_neighbors = -1;
  else
    params.max_neighbors = max_nn;

  int neighbors_in_radius = flann_index->radiusSearch (::flann::Matrix<float>(&query[0], 1, dim),
      indices, dists, static_cast<float>(radius*radius), params);

  k_indices = indices[0];
  k_sqr_dists = dists[0];

  if (!identity_mapping)
  {
    for (int i = 0; i < neighbors_in_radius; ++i)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping[neighbor_index];
    }
  }

  k_indices.resize(neighbors_in_radius);
  k_sqr_dists.resize(neighbors_in_radius);
}

/**
 * nearestKSearch
 */
void SearchKdTreeFLANN2f::nearestKSearch(int idx, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)
{
  if (idx>=total_nr_points || !data)
  {
    k_indices.clear();
    k_distances.clear();
    return;
  }

  if (k > total_nr_points)
    k = total_nr_points;

  k_indices.resize(k);
  k_distances.resize(k);

  ::flann::Matrix<int> k_indices_mat(&k_indices[0], 1, k);
  ::flann::Matrix<float> k_distances_mat(&k_distances[0], 1, k);

  flann_index->knnSearch(flann::Matrix<float>(&data[idx*dim], 1, dim),
                         k_indices_mat, k_distances_mat, k, param_k);

  if (!identity_mapping)
  {
    for (int i=0; i < k; i++)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping[neighbor_index];
    }
  }

  k_indices.resize(k);
  k_distances.resize(k);
}

/**
 * radiusSearch
 */
void SearchKdTreeFLANN2f::radiusSearch(int idx, double radius, std::vector<int> &k_indices, std::vector<float> &k_sqr_dists, unsigned int max_nn)
{
  if (idx>=total_nr_points || !data)
  {
    k_indices.clear();
    k_sqr_dists.clear();
    return;
  }
  
  if (max_nn == 0 || max_nn > static_cast<unsigned int> (total_nr_points))
    max_nn = total_nr_points;

  std::vector<std::vector<int> > indices(1);
  std::vector<std::vector<float> > dists(1);

  ::flann::SearchParams params(param_radius);
  if (max_nn == static_cast<unsigned int>(total_nr_points))
    params.max_neighbors = -1;
  else
    params.max_neighbors = max_nn;

  int neighbors_in_radius = flann_index->radiusSearch (::flann::Matrix<float>(&data[idx*dim], 1, dim),
      indices, dists, static_cast<float>(radius*radius), params);

  k_indices = indices[0];
  k_sqr_dists = dists[0];

  if (!identity_mapping)
  {
    for (int i = 0; i < neighbors_in_radius; ++i)
    {
      int& neighbor_index = k_indices[i];
      neighbor_index = index_mapping[neighbor_index];
    }
  }

  k_indices.resize(neighbors_in_radius);
  k_sqr_dists.resize(neighbors_in_radius);
}


/**
 * clean
 */
void SearchKdTreeFLANN2f::clean()
{
  if (flann_index)
    delete flann_index;

  if (data)
  {
    free(data);
    data = NULL;
  }

  index_mapping.clear();
}


} //-- THE END --






