/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "EuclideanClustering3f.hh"


namespace kp 
{

using namespace std;


/********************** EuclideanClustering3f ************************
 * Constructor/Destructor
 */
EuclideanClustering3f::EuclideanClustering3f(const Parameter &p)
 : nb_points(0), param(p)
{
  tree.reset(new SearchKdTreeFLANN3f(p.flannParam));
}

EuclideanClustering3f::~EuclideanClustering3f()
{
}




/************************** PRIVATE ************************/




/************************** PUBLIC *************************/

/**
 * setInputPoints
 */
void EuclideanClustering3f::setInputPoints(const std::vector<Eigen::Vector3f> &points)
{
  tree->setInputPoints(points);
  nb_points = points.size();
}

/**
 * extract
 */
void EuclideanClustering3f::extract(std::vector< std::vector<int> > &clusters)
{
  clusters.clear();

  if (nb_points==0)
    return;

  std::vector<int> seed_queue;
  int sq_idx;

  std::vector<bool> processed(nb_points, false);

  std::vector<int> nn_indices;
  std::vector<float> nn_distances;

  for (unsigned i = 0; i < nb_points; ++i)
  {
    if (processed[i])
      continue;

    seed_queue.clear();
    seed_queue.push_back(i);
    sq_idx = 0;
    processed[i] = true;

    while ( sq_idx < static_cast<int>(seed_queue.size ()) )
    {
      tree->radiusSearch(seed_queue[sq_idx], param.max_distance, nn_indices, nn_distances);

      for (size_t j = 1; j < nn_indices.size (); ++j)
      {
        if (nn_indices[j] == -1 || processed[nn_indices[j]])
          continue;

        seed_queue.push_back (nn_indices[j]);
        processed[nn_indices[j]] = true;
      }

      sq_idx++;
    }

    // finished -> add to the clusters
    if (seed_queue.size() >= param.min_points_per_cluster && seed_queue.size() <= param.max_points_per_cluster)
    {
      clusters.push_back(seed_queue);
    }
  }
}


} //-- THE END --






