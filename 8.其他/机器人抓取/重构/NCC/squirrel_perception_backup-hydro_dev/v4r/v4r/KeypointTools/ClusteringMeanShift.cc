/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "ClusteringMeanShift.hh"


namespace kp 
{

using namespace std;

/********************** ClusteringMeanShift ************************
 * Constructor/Destructor
 */
ClusteringMeanShift::ClusteringMeanShift(const Parameter &p)
 : sqr_lambda(p.lambda*p.lambda), sqr_thr_pruning(p.thr_prune*p.thr_prune),
   init_pcent(p.init_p), max_iter(p.max_iter), max_dist(p.eps_converge)
{
  inv_sqr_sigma = 1. / (p.sigma*p.sigma);
}

ClusteringMeanShift::~ClusteringMeanShift()
{
}

/**
 * Compute new sample mean
 * return true if converged
 */
int ClusteringMeanShift::updateMean(const Eigen::MatrixXf &data, Eigen::VectorXf &center)
{
  Eigen::VectorXf mean = center;
  float weight, sum_weight = 0.;

  center = Eigen::VectorXf::Zero(mean.size());

  for (unsigned i=0; i<data.rows(); i++)
  {
    weight = truncatedGaussian( (mean-(data.row(i).transpose())).squaredNorm() );
    center += weight*data.row(i);
    sum_weight += weight;
  }

  if (sum_weight > std::numeric_limits<float>::epsilon()) 
    center /= sum_weight;
  else
    return -1;

  if ((mean-center).squaredNorm() < max_dist)
    return 1;

  return 0;
}

void ClusteringMeanShift::initClusters(const Eigen::MatrixXf &data, std::vector< Cluster::Ptr > &clusters)
{
  if (fabs(init_pcent-1.) < 0.001)
  {
    clusters.resize(data.rows());
    for (unsigned i=0; i<data.rows(); i++)
    {
      clusters[i].reset( new Cluster(data.row(i)) );
    }
  }
  else
  {
    unsigned num = (unsigned)(init_pcent*(float)data.rows());
    num = (num < 1 ? 1 : (num > data.rows() ? data.rows() : num) );

    clusters.resize(num);
    for (unsigned i=0; i<num; i++)
    {
      clusters[i].reset( new Cluster( data.row(rand()%data.rows()) ) );
    }
  }
}






/****************************** PUBLIC *************************/
/**
 * Mean shift clustering
 */
void ClusteringMeanShift::cluster(const DataMatrix2Df &samples)
{
  clusters.clear();

  if (samples.rows==0)
    return;

  Eigen::Map<const Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > data(&samples(0,0), samples.rows, samples.cols);

  initClusters(data, clusters);

  std::vector<bool> converged(clusters.size(),false);

  int it=0;
  bool  changed = true;
  int status;
  while(changed && it<max_iter)
  {
    changed = false;

    for (unsigned i=0; i<clusters.size(); i++)
    {
      if (!converged[i])
      {
        status = updateMean(data, clusters[i]->data);

        if (status == 1)
        {
          converged[i]=true;
        }
        else
        {
          if (status == 0)
          {
            changed |= true;
          }
          else
          {
            clusters.erase(clusters.begin()+i);
            converged.erase(converged.begin()+i);
            i--;
          }
        }
      }
    }

    //pruning of clusters
    for (unsigned i=0; i<clusters.size(); i++)
    {
      for (unsigned j=i+1; j<clusters.size(); j++)
      {
        if ( (clusters[i]->data-clusters[j]->data).squaredNorm() < sqr_thr_pruning)
        {
          clusters.erase(clusters.begin()+j);
          converged.erase(converged.begin()+j);
          j--;
        }
      }
    }

    it++;
  }

  if (clusters.size()==0) return;

  //Assign samples to nearest cluster
  unsigned idx;
  double dist, minDist;
  for (unsigned i=0; i<data.rows(); i++)
  {
    idx=UINT_MAX, minDist=DBL_MAX;
    for (unsigned j=0; j<clusters.size(); j++)
    {
      dist = (clusters[j]->data-data.row(i).transpose()).squaredNorm();
      if (dist < minDist)
      {
        minDist = dist;
        idx=j;
      }
    }

    if (idx!=UINT_MAX) clusters[idx]->indices.push_back(i);
  }
}

/**
 * getClusters
 */
void ClusteringMeanShift::getClusters(std::vector<std::vector<int> > &_clusters)
{
  _clusters.resize(clusters.size());

  for (unsigned i=0; i<clusters.size(); i++)
    _clusters[i] = clusters[i]->indices;
}

/**
 * getCenters
 */
void ClusteringMeanShift::getCenters(DataMatrix2Df &_centers)
{
  _centers.clear();

  if (clusters.size()==0)
    return;

  int cols = clusters[0]->data.size();
  _centers.reserve(clusters.size(), cols);

  for (unsigned i=0; i<clusters.size(); i++)
    _centers.push_back(&clusters[i]->data[0], cols);
}


}






