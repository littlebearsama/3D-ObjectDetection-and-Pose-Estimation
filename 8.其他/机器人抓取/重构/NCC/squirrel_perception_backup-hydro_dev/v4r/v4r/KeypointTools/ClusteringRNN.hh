/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CLUSTERING_RNN_HH
#define KP_CLUSTERING_RNN_HH

#include <vector>
#include <string>
#include <stdexcept>
#include <float.h>
#include "DataMatrix2D.hpp"
#include "Clustering.hh"




namespace kp 
{

/**
 * ClusteringRNN
 */
class ClusteringRNN : public Clustering
{
public:
  class Parameter
  {
  public:
    float dist_thr;

    Parameter(float _dist_thr=0.4)
     : dist_thr(_dist_thr) {}
  };

private:
  std::vector< Cluster::Ptr > clusters;

  int getNearestNeighbour(const Cluster &cluster, const std::vector<Cluster::Ptr> &clusters, float &sim);
  void agglomerate(const Cluster &src, Cluster &dst);

  void initDataStructure(const DataMatrix2Df &samples, std::vector< Cluster::Ptr > &data);


 
public:
  Parameter param;
  bool dbg;

  ClusteringRNN(const Parameter &_param = Parameter(), bool _dbg=true);
  ~ClusteringRNN();

  virtual void cluster(const DataMatrix2Df &samples); 
  virtual void getClusters(std::vector<std::vector<int> > &_clusters);
  virtual void getCenters(DataMatrix2Df &_centers);
};





/************************** INLINE METHODES ******************************/



}

#endif

