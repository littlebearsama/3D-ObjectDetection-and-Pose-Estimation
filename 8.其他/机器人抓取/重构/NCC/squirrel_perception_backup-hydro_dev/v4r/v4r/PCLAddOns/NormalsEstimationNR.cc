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
 * $Id$
 * Johann Prankl, 18.2.2012
 */


#include "NormalsEstimationNR.hh"

namespace pclA 
{

using namespace std;

float NormalsEstimationNR::NaN  = std::numeric_limits<float>::quiet_NaN(); 

template<typename T1,typename T2, typename T3>
inline void Div3(const T1 v[3], T2 s, T3 r[3])
{
  s = 1./s;
  r[0] = v[0]*s;
  r[1] = v[1]*s;
  r[2] = v[2]*s;
}

template<typename T1,typename T2, typename T3>
inline void Add3(const T1 v1[3], const T2 v2[3], T3 r[3])
{
  r[0] = v1[0]+v2[0];
  r[1] = v1[1]+v2[1];
  r[2] = v1[2]+v2[2];
}

template<typename T1,typename T2>
inline T1 Dot3(const T1 v1[3], const T2 v2[3])
{
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

template<typename T1>
inline T1 Norm3(const T1 v[3])
{
  return sqrt(pow(v[0], 2) + pow(v[1], 2) + pow(v[2], 2));
}

template<typename T1, typename T2>
inline void Normalise3(const T1 v[3], T2 r[3])
{
  Div3(v,Norm3(v),r);
}

template<typename T1,typename T2, typename T3>
inline void Mul3(const T1 v[3], T2 s, T3 r[3])
{
  r[0] = v[0]*s;
  r[1] = v[1]*s;
  r[2] = v[2]*s;
}

template<typename T1,typename T2>
inline T1 DistanceSqr3(const T1 d1[3], const T2 d2[3])
{
  return pow((d1[0]-d2[0]),2) + pow((d1[1]-d2[1]),2) + pow((d1[2]-d2[2]),2);
}


/********************** NormalsEstimationNR ************************
 * Constructor/Destructor
 */
NormalsEstimationNR::NormalsEstimationNR(Parameter p)
{
  setParameter(p);
}

NormalsEstimationNR::~NormalsEstimationNR()
{
}


/************************** PRIVATE ************************/

/**
 * Init
 */
void NormalsEstimationNR::Init()
{
  normals.reset(new pcl::PointCloud<pcl::Normal>());

  if (m0.get()==0)
    m0.reset(new pcl::PointCloud<pcl::Normal>());
  if (m1.get()==0)
    m1.reset(new pcl::PointCloud<pcl::Normal>());

  normals->header   = cloud->header;
  normals->width    = width;
  normals->height   = height;
  normals->is_dense = cloud->is_dense;
  normals->resize(width*height);

  m0->header   = cloud->header;
  m0->width    = width;
  m0->height   = height;
  m0->is_dense = cloud->is_dense;
  m0->resize(width*height);

  m1->header   = cloud->header;
  m1->width    = width;
  m1->height   = height;
  m1->is_dense = cloud->is_dense;
  m1->resize(width*height);

  converged.clear();
  converged.resize(width*height,false);
}

/**
 * InitNeighbours
 */
void NormalsEstimationNR::InitNeighbours()
{
  int numPoints = Sqr(2*param.radius+1)+1;
  neighbours.clear();
  neighbours.resize(cloud->points.size());   // the first index of each vector is the point itself
  pcl::PointCloud<pcl::PointXYZRGB> &refCloud = *cloud;
  
  // if the pointcloud is organized use the grid structure to detect neighbours
  if (!param.useOctree && refCloud.isOrganized())
  {
    #pragma omp parallel for
    for (int v=0; v<height; v++)
    {
      int idx, idx0;
      for (int u=0; u<width; u++)
      {
        idx0 = GetIdx(u,v);
        pcl::PointXYZRGB &pt0 = refCloud.points[idx0];

        if (!IsNaN(pt0))
        {
          std::vector<int> &indices = neighbours[idx0];
          indices.reserve(numPoints);
          indices.push_back(idx0);

          for (int y = v-param.radius; y<=v+param.radius; y++)
          {
            for (int x = u-param.radius; x<=u+param.radius; x++)
            {
              if (x>=0 && x<width && y>=0 && y<height)
              {
                idx = GetIdx(x,y);
                pcl::PointXYZRGB &pt = refCloud.points[idx];

                if (!IsNaN(pt) && SqrDistance(pt0, pt) < sqrMaxDist)
                  indices.push_back(idx);
              }
            }
          }

          if (indices.size()<5) 
            indices.clear();
        }
      }
    }
  }
  else   // use an octree...
  {
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(param.treeResolution);
    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

    std::vector<float> sqrDists;
    std::vector<int> indices;

    #pragma omp parallel for private(indices, sqrDists)
    for (unsigned i=0; i<refCloud.points.size(); i++)
    {
      if (!IsNaN(refCloud.points[i]))
      {
        pcl::PointXYZRGB &pt0 = refCloud.points[i];
        indices.resize(numPoints);
        sqrDists.resize(numPoints);

        int cnt = octree.nearestKSearch(i,numPoints, indices, sqrDists);

        if (cnt>3)
        {
          std::vector<int> &nbs = neighbours[i];
          nbs.reserve(cnt+1);
          nbs.push_back(i);

          for (int j=0; j<cnt; j++)
            if (SqrDistance(pt0,refCloud.points[indices[j]]) < sqrMaxDist)
              nbs.push_back(indices[j]);
        }
      }
    }
  }
}

/**
 * InitNeighbours
 */
void NormalsEstimationNR::InitNeighbours(const std::vector<int> &mask)
{
  int numPoints = Sqr(2*param.radius+1)+1;
  neighbours.clear();
  neighbours.reserve(mask.size());   // the first index of each vector is the point itself
  pcl::PointCloud<pcl::PointXYZRGB> &refCloud = *cloud;
  
  // if the pointcloud is organized use the grid structure to detect neighbours
  if (refCloud.isOrganized())
  {
    std::vector<int> *indices;

    #pragma omp parallel for private(indices)
    for (int i=0; i<(int)mask.size(); i++)
    {
      int u,v, idx;
      int idx0 = mask[i];
      pcl::PointXYZRGB &pt0 = refCloud.points[idx0];

      if (!IsNaN(pt0))
      {
        u = X(idx0);
        v = Y(idx0);

        #pragma omp critical
        {
          neighbours.push_back(std::vector<int>());
          indices = &neighbours.back();
        }
        indices->reserve(numPoints);
        indices->push_back(idx0);

        for (int y = v-param.radius; y<=v+param.radius; y++)
        {
          for (int x = u-param.radius; x<=u+param.radius; x++)
          {
            if (x>=0 && x<width && y>=0 && y<height)
            {
              idx = GetIdx(x,y);
              pcl::PointXYZRGB &pt = refCloud.points[idx];

              if (!IsNaN(pt) && SqrDistance(pt0, pt) < sqrMaxDist)
                indices->push_back(idx);
            }
          }
        }

        if (indices->size()<5) 
          indices->clear();
      }
    }
  }
  else   // use an octree...
  {
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree(param.treeResolution);
    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();

    std::vector<float> sqrDists;
    std::vector<int> indices;
    std::vector<int> *nbs;

    #pragma omp parallel for private(indices, sqrDists, nbs)
    for (unsigned i=0; i<mask.size(); i++)
    {
      int idx0 = mask[i];
      if (!IsNaN(refCloud.points[idx0]))
      {
        pcl::PointXYZRGB &pt0 = refCloud.points[idx0];

        indices.resize(numPoints);
        sqrDists.resize(numPoints);

        int cnt = octree.nearestKSearch(idx0,numPoints, indices, sqrDists);

        if (cnt>3)
        {
          #pragma omp critical
          {
            neighbours.push_back(std::vector<int>());
            nbs = &neighbours.back();
          }
          nbs->reserve(cnt+1);
          nbs->push_back(idx0);

          for (int j=0; j<cnt; j++)
            if (SqrDistance(pt0,refCloud.points[indices[j]]) < sqrMaxDist)
              nbs->push_back(indices[j]);
        }
      }
    }
  }
}

/**
 * CCFilterNeighbours
 */
unsigned mcnt, ucnt;
#pragma omp threadprivate(mcnt,ucnt)
void NormalsEstimationNR::CCFilterNeighbours()
{
  pcl::PointCloud<pcl::PointXYZRGB> &refCloud = *cloud;

  if (refCloud.isOrganized())
  {
    std::vector<int> queue;
    std::vector<unsigned> used;
    std::vector<unsigned> mask;
    unsigned idx;
    short x,y;

    mcnt=0, ucnt=0;

    #pragma omp parallel for private(queue,used,mask,idx,x,y) copyin(mcnt,ucnt)
    for (unsigned i=0; i<neighbours.size(); i++)
    {
      used.resize(refCloud.points.size(),0);
      mask.resize(refCloud.points.size(),0);
      std::vector<int> &nbs = neighbours[i];

      if (nbs.size()>0)
      {
        mcnt++;
        ucnt++;
        idx = nbs[0];
        queue.clear();
        queue.push_back(idx);
        used[idx] = ucnt;

        //set mask
        for (unsigned j=0; j<nbs.size(); j++)
          mask[nbs[j]]=mcnt;

        nbs.clear();
        nbs.push_back(idx);

        //cluster
        while (queue.size()>0)
        {
          idx = queue.back();
          queue.pop_back();
          x = X(idx);
          y = Y(idx);

          pcl::PointXYZRGB &pt0 = refCloud.points[idx];

          if (pt0.x==pt0.x)
          {
            for (int v=y-1; v<=y+1; v++)
            {
              for (int u=x-1; u<=x+1; u++)
              {
                if (v>0 && u>0 && v<height && u<width)
                {
                  idx = GetIdx(u,v);
                  if (mask[idx] == mcnt && used[idx]!=ucnt)
                  {
                    pcl::PointXYZRGB &pt = refCloud.points[idx];
                    if (pt.x==pt.x && SqrDistance(pt0,pt) < sqrMaxPointDist)
                    {
                      nbs.push_back(idx);
                      queue.push_back(idx);
                    }
                    used[idx] = ucnt;
                  }
                }
              }
            }
          }
        }

        if (nbs.size()<4)
          nbs.clear();
      }
    }
  }
  else
  {
    cout<<"[NormalsEstimationNR::CCFilterNeighbours] Connected component filter is not implemented for unorganized point clouds!"<<endl;
  }
}

/**
 * ComputeCovarianceMatrix
 */
void NormalsEstimationNR::ComputeCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
      const std::vector<int> &indices, const Eigen::Vector4f &mean, Eigen::Matrix3f &cov)
{
  cov.setZero ();

  for (unsigned i = 0; i < indices.size (); ++i)
  {
    Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - mean;

    cov(1,1) += pt.y () * pt.y ();
    cov(1,2) += pt.y () * pt.z ();

    cov(2,2) += pt.z () * pt.z ();

    pt *= pt.x ();
    cov(0,0) += pt.x ();
    cov(0,1) += pt.y ();
    cov(0,2) += pt.z ();
  }

  cov(1,0) = cov(0,1);
  cov(2,0) = cov(0,2);
  cov(2,1) = cov(1,2);
}

/**
 * ComputeWeightedCovarianceMatrix
 */
void NormalsEstimationNR::ComputeWeightedCovarianceMatrix (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
      const std::vector<int> &indices, const std::vector<float> &weights, 
      const Eigen::Vector4f &weightedMean, Eigen::Matrix3f &cov)
{
  float w;
  cov.setZero ();

  for (unsigned i = 0; i < indices.size (); ++i)
  {
    w = 1. - weights[i];
    Eigen::Vector4f pt = cloud.points[indices[i]].getVector4fMap () - weightedMean;

    cov(1,1) += w * pt.y () * pt.y ();
    cov(1,2) += w * pt.y () * pt.z ();

    cov(2,2) += w * pt.z () * pt.z ();

    pt *= w * pt.x ();
    cov(0,0) += pt.x ();
    cov(0,1) += pt.y ();
    cov(0,2) += pt.z ();
  }

  cov(1,0) = cov(0,1);
  cov(2,0) = cov(0,2);
  cov(2,1) = cov(1,2);
}

/**
 * ComputeWeightedMean
 */
void NormalsEstimationNR::ComputeWeightedMean (const pcl::PointCloud<pcl::PointXYZRGB> &cloud,
      const std::vector<int> &indices, const std::vector<float> &weights, Eigen::Vector4f &weightedMean)
{
  float sum=0, w;
  weightedMean.setZero ();

  for (unsigned i = 0; i < indices.size (); ++i)
  {
    w = 1. - weights[i];
    weightedMean += w*cloud.points[indices[i]].getVector4fMap();
    sum += w;
  }

  weightedMean /= sum;
}



/**
 * ComputeNormalsLS
 */
void NormalsEstimationNR::ComputeNormalsLS(pcl::PointCloud<pcl::PointXYZRGB> &cloud, pcl::PointCloud<pcl::Normal> &normals)
{
  Eigen::Vector4f n0(0.,0.,1.,0.);
  Eigen::Vector4f mean;
  EIGEN_ALIGN16 Eigen::Matrix3f cov;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  float eigsum;

  #pragma omp parallel for private(mean,cov,eigen_values,eigen_vectors,eigsum)
  for (unsigned i=0; i<neighbours.size(); i++)
  {
    std::vector<int> &indices = neighbours[i];

    if (indices.size()<4)
      continue;

    mean.setZero();

    for (unsigned j=0; j<indices.size(); j++)
      mean += cloud[indices[j]].getVector4fMap();

    pcl::Normal &n = normals.points[indices[0]];
    mean /= (float)indices.size();
    mean[3]=0.;
    ComputeCovarianceMatrix (cloud, indices, mean, cov);

    pcl::eigen33 (cov, eigen_vectors, eigen_values);
    n.normal[0] = eigen_vectors (0,0);
    n.normal[1] = eigen_vectors (1,0);
    n.normal[2] = eigen_vectors (2,0);
    eigsum = eigen_values.sum();

    if (eigsum != 0)
      n.curvature = fabs (eigen_values[0] / eigsum );
    else
      n.curvature = 0;

    if ( pclA::Dot3(&n.normal[0], &n0[0]) > 0)
         pclA::Mul3(&n.normal[0],-1,&n.normal[0]);
  }
}

/**
 * ComputeNormalsWeightedLS
 */
void NormalsEstimationNR::ComputeNormalsWeightedLS(pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
       pcl::PointCloud<pcl::Normal> &normals_m, pcl::PointCloud<pcl::Normal> &normals)
{
  Eigen::Vector4f n0(0.,0.,1.,0.);
  float w;
  vector<float> weights;        // l
  Eigen::Vector4f weightedMean;
  EIGEN_ALIGN16 Eigen::Matrix3f cov;
  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
  //float eigsum;

  #pragma omp parallel for private(w,weights,weightedMean,cov,eigen_values,eigen_vectors)//,eigsum)
  for (unsigned i=0; i<neighbours.size(); i++)
  {
    std::vector<int> &indices = neighbours[i];

    if (indices.size()<4)
      continue;

    weights.clear();
    pcl::Normal &mi = normals_m.points[indices[0]];

    for (unsigned j=0; j<indices.size(); j++)
    { 
      pcl::Normal &mj = normals_m.points[indices[j]];
      w = pclA::DistanceSqr3(&mi.normal[0], &mj.normal[0]);
      weights.push_back(w/(w+param.beta));
    }

    pcl::Normal &n = normals.points[indices[0]];
    ComputeWeightedMean(cloud, indices, weights, weightedMean);
    weightedMean[3]=0.;
    ComputeWeightedCovarianceMatrix(cloud, indices, weights, weightedMean, cov);

    pcl::eigen33 (cov, eigen_vectors, eigen_values);
    n.normal[0] = eigen_vectors (0,0);
    n.normal[1] = eigen_vectors (1,0);
    n.normal[2] = eigen_vectors (2,0);
    /*eigsum = eigen_values.sum();   // compute weighted curvature does not make sense!

    if (eigsum != 0)
      n.curvature = fabs ( eigen_values[0] / eigsum );
    else
      n.curvature = 0;*/

    if ( pclA::Dot3(&n.normal[0], &n0[0]) > 0)
         pclA::Mul3(&n.normal[0],-1,&n.normal[0]);
  }
}

/**
 * ReorganizeNeighborhood
 */
bool NormalsEstimationNR::ReorganizeNeighborhood(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
      pcl::PointCloud<pcl::Normal> &normals_n, pcl::PointCloud<pcl::Normal> &normals_m0, 
      pcl::PointCloud<pcl::Normal> &normals_m1)
{
  int idx, idx_uv;
  float w;
  vector<int> indices;
  vector<float> weights;        // l
  float tmp_n[3];

  #pragma omp parallel for private(idx,idx_uv,w,indices,weights,tmp_n)
  for (unsigned i=0; i<neighbours.size(); i++)
  {
    std::vector<int> &indices = neighbours[i];

    if (indices.size()<4)
      continue;

    idx_uv = indices[0];

    if (converged[idx_uv])
      continue;

    weights.clear();
    pcl::Normal &m0 = normals_m0.points[idx_uv];

    for (unsigned j=0; j<indices.size(); j++)
    {
      idx = indices[j];
      pcl::Normal &m0j = normals_m0.points[idx];
      w = pclA::DistanceSqr3(&m0.normal[0], &m0j.normal[0]);
      weights.push_back(w/(w+param.beta));
    }

    pcl::Normal &m1 = normals_m1.points[idx_uv];
    m1.normal[0] = m1.normal[1] = m1.normal[2] = 0.;

    for (unsigned j=0; j<indices.size(); j++) {
      pclA::Mul3(normals_m0.points[indices[j]].normal, Sqr(1.-weights[j]), tmp_n);
      pclA::Add3(m1.normal, tmp_n ,m1.normal);
    }

    pclA::Mul3(m1.normal, param.alpha, m1.normal);
    pclA::Add3(m1.normal, normals_n.points[idx_uv].normal, m1.normal);
    pclA::Normalise3(m1.normal,m1.normal);

    if (pclA::DistanceSqr3(&m1.normal[0], &m0.normal[0]) < sqrEpsConverge) {
      converged[idx_uv]=true;
    }
  }

  return false;
}






/************************** PUBLIC *************************/

/**
 * setInputCloud
 */
void NormalsEstimationNR::setInputCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &_cloud)
{
  cloud = _cloud;

  width = cloud->width;
  height = cloud->height;
}

/**
 * compute the normals
 */
void NormalsEstimationNR::compute()
{
  int z=0;
  bool converged=false;

  if (cloud.get() == 0)
    throw std::runtime_error ("[NormalsEstimationNR::compute] No point cloud available!");

  Init();

  InitNeighbours();  
  if (param.filter) CCFilterNeighbours(); 
  


  ComputeNormalsLS(*cloud, *normals);

  if (param.maxIter>0)
  {
    pcl::copyPointCloud(*normals, *m1);

    while(!converged && z<param.maxIter)
    {
      std::swap(m0,m1);
      converged = ReorganizeNeighborhood(*cloud, *normals, *m0, *m1);
      z++;
    }

    ComputeNormalsWeightedLS(*cloud, *m1, *normals);
  }
}

/**
 * compute the normals
 */
void NormalsEstimationNR::compute(const vector<int> &mask)
{
  int z=0;
  bool converged=false;

  if (cloud.get() == 0)
    throw std::runtime_error ("[NormalsEstimationNR::compute] No point cloud available!");

  Init();
  InitNeighbours(mask);  

  if (param.filter) CCFilterNeighbours(); 

  ComputeNormalsLS(*cloud, *normals);

  if (param.maxIter > 0)
  {
    pcl::copyPointCloud(*normals, *m1);

    while(!converged && z<param.maxIter)
    {
      std::swap(m0,m1);
      converged = ReorganizeNeighborhood(*cloud, *normals, *m0, *m1);
      z++;
    }

    ComputeNormalsWeightedLS(*cloud, *m1, *normals);
  }
}


/**
 * getNormals
 */
void NormalsEstimationNR::getNormals(pcl::PointCloud<pcl::Normal>::Ptr &_normals)
{
  _normals = normals;
}

/**
 * setParameter
 */
void NormalsEstimationNR::setParameter(Parameter p)
{
  param = p;
  sqrMaxDist = Sqr(param.maxDist);
  sqrEpsConverge = Sqr(param.epsConverge);
  sqrMaxPointDist = Sqr(param.maxPointDist);
}


} //-- THE END --

