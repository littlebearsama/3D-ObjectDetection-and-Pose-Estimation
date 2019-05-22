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
 * @file PlanePopout.cc
 * @author Prankl, Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Extract a dominant plane from kinect data and estimate popouts.
 * 
 * TODO SOI and ROI labels are not equal.
 */


#include "PlanePopout.hh"
#include "PCLUtils.h"
#include "PCLFunctions.h"

namespace pclA
{

using namespace std;
  
/**************************************  SOI **********************************************/
PlanePopout::SOI::SOI(unsigned l, 
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr &s)
{
  label = l;
  soi = s;

  convex_hull.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(unsigned i=0; i<soi->points.size()/2; i++)
    convex_hull->push_back(soi->points[i]);
}
  
unsigned PlanePopout::SOI::IsInSOI(const pcl::PointXYZRGB &p)
{
  if(pcl::isPointIn2DPolygon(p, *convex_hull))
    return label;
  return 0;
}

/************************************************************************************
 * Constructor/Destructor
 */

PlanePopout::PlanePopout(Parameter _param)
 : param(_param)
{
  zFilter.setFilterFieldName ("z"); 
  zFilter.setFilterLimits (param.minZ, param.maxZ);
  zFilter.setKeepOrganized(true);

  normalsTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB> > ();
  clustersTree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB> > ();

  grid.setLeafSize(param.downsampleLeaf, param.downsampleLeaf, param.downsampleLeaf);
  gridObjects.setLeafSize(param.downsampleLeafObjects, param.downsampleLeafObjects, param.downsampleLeafObjects);
  grid.setFilterFieldName ("z");
  grid.setFilterLimits (param.minZ, param.maxZ);
  grid.setDownsampleAllData (false);
  gridObjects.setDownsampleAllData (false);

  n3d.setKSearch (param.nbNeighbours);
  n3d.setSearchMethod (normalsTree);

  seg.setDistanceThreshold (param.thrSacDistance);
  seg.setMaxIterations (2000);                          // 2000
  seg.setNormalDistanceWeight (param.normalDistanceWeight);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);        //pcl::SACMODEL_NORMAL_PLANE
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setProbability (0.99);

  proj.setModelType (pcl::SACMODEL_NORMAL_PLANE);       //pcl::SACMODEL_NORMAL_PLANE

  prism.setHeightLimits (param.minObjectHeight, param.maxObjectHeight);

  ccLabeling = new pclA::CCLabeling(pclA::CCLabeling::Parameter(param.thr,param.minClusterSize));

  
  // initialize variables
  valid_computation = false;
  input_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  cloudDownsampled.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  cloudDownsampledNormalFiltered.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  cloudNormals.reset (new pcl::PointCloud<pcl::Normal> ());
  tableInliers.reset (new pcl::PointIndices());
  tableCoefficients.reset (new pcl::ModelCoefficients());
  tableProjected.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
  tableHull.reset (new pcl::PointCloud<pcl::PointXYZRGB>());
}

PlanePopout::~PlanePopout()
{
}

// ================================= Public functions ================================= //
/**
 * Calculate Space of Interest (SOI) prisms as convex hulls. Processing:
 * - First filter incoming cloud in z-direction
 * - Detect the popout points of the point cloud after removing the dominant plane.
 * - Do euclidean clustering
 * - Project the clustered popouts to the table plane and estimate the convex hull
 * - Build from the convex hull (perpendicular) a soi prism with bottom/top points of 
 *   the convex hull.
 */
bool PlanePopout::CalculateSOIs(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
  bool have_done_something_not_entirely_wrong = false;
  valid_computation = false;    
  sois.clear();

  pclA::CopyPointCloud(*cloud, *input_cloud);
 
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > clusters_projected;
  std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > cluster_hulls;

  if(popoutsCloud.get() == 0)
    popoutsCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  if(DetectPopout(cloud, popouts))
  {
    // something just worked, so we are by and large happy
    have_done_something_not_entirely_wrong = true;

    pcl::copyPointCloud(*cloud, popouts, *popoutsCloud);

    // NOTE: with a small point cloud (e.g. size 5 or 7) euclidian clustering
    // still segfaults!
    // So do not proceed here if there are fewer than minClusterSize points in
    // the cloud.
    if(popoutsCloud->points.size() >= param.minClusterSize)
    {
      pclA::EuclideanClustering(popoutsCloud, clusters);                           // Do euclidean clustering of the popout-cloud
      for(unsigned i=0; i<clusters.size(); i++)
      {
        if(clusters[i]->points.size() > param.minClusterSize)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_hull;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr soi;

          double max_dist = 0;
          pclA::MaxDistanceToPlane(clusters[i], tableCoefficients, max_dist);     // Calculate the maximum distance to table plane
          pclA::GetConvexHull(clusters[i], tableCoefficients, cluster_hull);      // Project points on plane and get convex hull
          cluster_hulls.push_back(cluster_hull);

          pclA::CreateSOI(cluster_hull, tableCoefficients, max_dist, soi);        // Create SOI prism from convex hull and height
          SOI s(i+1, soi);
          sois.push_back(s);
        }
      }
 
      valid_computation = true;
    }
  }
  return have_done_something_not_entirely_wrong;
}



/**
 * Calculate the ROI mask with labels, starting with 1. 0 means no ROI.
 */
bool PlanePopout::CalculateROIMask()
{
  if(!valid_computation) return false;
  cv::Mat_<cv::Vec4f> matCloud;
  std::vector<unsigned> sizeClusters;
  
  ConvertPopout2Mat(*input_cloud, popouts, matCloud);
  LabelClusters(matCloud, roi_label_mask, sizeClusters);

  // Filter ROI label mask, concerning the minimum cluster size
  cv::Mat_<ushort> new_roi_label_mask;
  new_roi_label_mask = cv::Mat::zeros(roi_label_mask.rows, roi_label_mask.cols, CV_8U);
  std::vector<unsigned> new_sizeClusters;
  new_sizeClusters.push_back(0);
  ushort cluster_id = 1;
  
  for (int i = 0; i < (int)sizeClusters.size(); i++)
  {
    if(sizeClusters[i] > param.minClusterSize)
    {
      new_sizeClusters.push_back(sizeClusters[i]);
      for (int v = 0; v < roi_label_mask.rows; ++v)
      {
        for (int u = 0; u < roi_label_mask.cols; ++u)
        {
          const ushort &la = roi_label_mask(v,u);
          if (la == i)
            new_roi_label_mask(v,u) = cluster_id;
        }
      }
      cluster_id++;
    }
  }
  roi_label_mask = new_roi_label_mask;
  sizeClusters = new_sizeClusters;
  return true;
}


/**
 * Check, if image coordinate point is in a ROI.
 */
ushort PlanePopout::IsInROI(int x, int y)
{
  return roi_label_mask(y, x);
}

void PlanePopout::GetSOIs(std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &_sois,
                          std::vector<unsigned> &labels)
{
  for(unsigned i=0; i<sois.size(); i++)
  {
    _sois.push_back(sois[i].GetSoi());
    labels.push_back(sois[i].GetLabel());
  }
}

/**
 * Check, if point is in SOI. Returns the cluster label for that point.
 * Returns zero, if point is "under" the dominant plane (view dependent!)
 */ 
unsigned PlanePopout::IsInSOI(float x, float y, float z)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointXYZRGB p, pn, pp;
  p.x = x;
  p.y = y;
  p.z = z;
  pcl_cloud->push_back(p);
  pclA::GetProjectedPoints(pcl_cloud, tableCoefficients, pcl_cloud_projected);
  pn = pcl_cloud_projected->points[0];
  
  unsigned label = 0;
  for(unsigned i=0; i<sois.size(); i++)
  {
    unsigned newLabel = sois[i].IsInSOI(p);
    if(newLabel > 0) 
    {
      // check, if point is "under" table (depends on view!!!)
      pp.x = pn.x - p.x;
      pp.y = pn.y - p.y;
      pp.z = pn.z - p.z;
      double norm_p = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      double norm_pp = sqrt(pp.x*pp.x + pp.y*pp.y + pp.z*pp.z);
      double dot_p_pp = p.x*pp.x + p.y*pp.y + p.z*pp.z;
      double angle = acos(dot_p_pp/(norm_p*norm_pp));
      if(angle < 1.57)  // Pi/2
        label = newLabel;
    }
  }
  return label;
}

/**
 * Detect table plane and objects which pop out
 */
bool PlanePopout::CollectTableInliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                                      pcl::ModelCoefficients::Ptr dpc)
{
  tableInliers.reset (new pcl::PointIndices());
  // check if we have a valid plane
  if(dpc && dpc->values.size() == 4)
  {
    for (unsigned i=0; i<cloud->points.size(); i++)
    {
      float dist=abs(dpc->values[0]*cloud->points[i].x
        +dpc->values[1]*cloud->points[i].y
        +dpc->values[2]*cloud->points[i].z
        +dpc->values[3])
      /sqrt(dpc->values[0]*dpc->values[0]
           +dpc->values[1]*dpc->values[1]
           +dpc->values[2]*dpc->values[2]);
      if (dist < param.thr)
      {//--------------------------------added by Kai Zhou, for collecting points which can be projected inside of the tablehull-------------------- 
	  if (pcl::isPointIn2DPolygon(cloud->points[i],*tableHull))
	      (*tableInliers).indices.push_back(i);
      }
    }
  }
  return true;
}

/**
 * Ensure that normal vector is unit vector.
 * And we need tableCoefficients->values[3] as positive distance
 * to estimate the maximum distance to plane.
 */
bool PlanePopout::NormalisePlane(pcl::ModelCoefficients::Ptr &plane)
{
  double s = sqrt(plane->values[0]*plane->values[0] +
		  plane->values[1]*plane->values[1] +
		  plane->values[2]*plane->values[2]);
  if(std::fpclassify(s) == FP_ZERO) {
    printf("[PCLAddOns::PlanePopout::NormalisePlane] Error: invalid plane - zero normal vector. Return.\n");
    return false;
  }
  plane->values[0] /= s;
  plane->values[1] /= s;
  plane->values[2] /= s;
  plane->values[3] /= s;
  if(plane->values[3] < 0.) {
    plane->values[0] = -plane->values[0];
    plane->values[1] = -plane->values[1];
    plane->values[2] = -plane->values[2];
    plane->values[3] = -plane->values[3];
  }
  return true;
}

/**
 * Detect table plane and objects which pop out
 */
bool PlanePopout::DetectPopout(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                               pcl::PointIndices &popout)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Filter the PointCloud in z-coordinate
  FilterZ(cloud, *cloudFiltered);
  if ((int)cloudFiltered->points.size() < param.nbNeighbours) {
    cout << "[PCLAddOns::PlanePopout::DetectPopout] Only " << (int)cloudFiltered->points.size() << " are available!"<<endl;
    return false;
  }

  grid.setInputCloud (cloudFiltered);
  grid.filter (*cloudDownsampled);
  if(cloudDownsampled->points.size() < param.minSACPoints) {
    printf("[PCLAddOns::PlanePopout::DetectPopout] Warning: Downsampled cloud is empty. Return.\n");
    return false;
  }

  if (tableCoefficients.get()==0)
    tableCoefficients.reset(new pcl::ModelCoefficients());
  bool have_plane = false;
  while(!have_plane)
  {
    n3d.setInputCloud (cloudDownsampled);
    n3d.compute (*cloudNormals);
    
    seg.setInputCloud (cloudDownsampled);
    seg.setInputNormals (cloudNormals);
    seg.segment (*tableInliers, *tableCoefficients);

    if (tableInliers->indices.size () == 0) {
      cout<<"PlanePopout::DetectPopout: No Plane Inliers points!"<<endl;
      return false;
    }

    if(!NormalisePlane(tableCoefficients))
      return false;

    // only accept horizontal planes
    // NOTE: assumes that z points up!
    if(fabs(tableCoefficients->values[2]) > cos(param.delta)) {
      have_plane = true;
    }
    else {
      pcl::ExtractIndices<pcl::PointXYZRGB> extract_inliers;
      extract_inliers.setNegative(true);
      extract_inliers.setInputCloud(cloudDownsampled);
      extract_inliers.setIndices(tableInliers);
      extract_inliers.filter(*cloudDownsampledNormalFiltered);
      pcl::copyPointCloud(*cloudDownsampledNormalFiltered, *cloudDownsampled);
    }
  }
  
  //------------------------------added by kai, get the biggest cluster from the dominant plane
  //------------------------------remove the "ghost" points as well as the non-connected plane points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr planepointsCloud;
  planepointsCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloudDownsampled, *tableInliers, *planepointsCloud);
  
  double cluster_tolerance = 0.05;  // make it bigger for the clustering since we are dealing with the downsampled point cloud
  double min_cluster_size = 5;
  double max_cluster_size = 1000000;
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
  tree->setInputCloud(planepointsCloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  std::vector<pcl::PointIndices> inliers_vector;
  ec.setClusterTolerance (cluster_tolerance);
  ec.setMinClusterSize (min_cluster_size);
  ec.setMaxClusterSize (max_cluster_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud(planepointsCloud);
  ec.extract(inliers_vector);
  
  int index_cluster=0;
  int MaxNumberOfPointsInOneCluster=0;
  //cout<<"There are "<<inliers_vector.size()<<" clusters"<<endl;
  for(unsigned i=0; i<inliers_vector.size(); i++)
  {
    //cout<<"In No. "<<i+1<<" cluster, there are "<<inliers_vector.at(i).indices.size()<<" points"<<endl;
    if((int) inliers_vector.at(i).indices.size() > MaxNumberOfPointsInOneCluster)
    {
      index_cluster=i;
      MaxNumberOfPointsInOneCluster=inliers_vector.at(i).indices.size();
    }
  }
      
  pcl::PointIndices::Ptr planePointsIndices;
  planePointsIndices.reset (new pcl::PointIndices());
  (*planePointsIndices)=inliers_vector[index_cluster];

  proj.setInputCloud (planepointsCloud);
  proj.setIndices (planePointsIndices);
  proj.setModelCoefficients (tableCoefficients);
  proj.filter (*tableProjected); 
      

//   proj.setInputCloud (cloudDownsampled);
//   proj.setIndices (tableInliers);
//   proj.setModelCoefficients (tableCoefficients);
//   proj.filter (*tableProjected); 
  
//--------------------------------------------------------------------ends here-----------------------
//  hull.setDimension(2);
  
  hull.setInputCloud (tableProjected);
  hull.reconstruct (*tableHull);

  prism.setInputCloud (cloud);
  prism.setInputPlanarHull (tableHull);
  prism.segment (popout); 
  return true;
}

/**
 * Filter point cloud depending on z-value
 * (PassThroughFilter)
 */
void PlanePopout::FilterZ(const cv::Mat_<cv::Vec4f> &cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB> &filtered)
{
  pclA::ConvertCvMat2PCLCloud(cloud, pclCloud);
  zFilter.setInputCloud (pclCloud);
  zFilter.filter(filtered);
}


/**
 * Filter point cloud depending on z-value
 * (PassThroughFilter)
 */
void PlanePopout::FilterZ(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB> &filtered)
{
  zFilter.setInputCloud (cloud);
  zFilter.filter(filtered);
}

/**
 * Filter point cloud depending on z-value
 * (PassThroughFilter)
 */
void PlanePopout::FilterZ(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, 
                          pcl::PointCloud<pcl::PointXYZRGB> &filtered)
{
  zFilter.setInputCloud (cloud);
  zFilter.filter(filtered);
}

/**
 * Convert point indices from plane popout to a cv::Mat
 */
void PlanePopout::ConvertPopout2Mat(const pcl::PointCloud<pcl::PointXYZRGB> &cloud, 
                                    const pcl::PointIndices &popout, 
                                    cv::Mat_<cv::Vec4f> &matCloud)
{
  matCloud = cv::Mat_<cv::Vec4f>(cloud.height, cloud.width);
  float bad_point =  std::numeric_limits<float>::quiet_NaN ();

  for (int v = 0; v < matCloud.rows; ++v)
  {
    for (int u = 0; u < matCloud.cols; ++u)
    {
      cv::Vec4f &matpt = matCloud(v,u);
      matpt[0] = bad_point;
      matpt[1] = bad_point;
      matpt[2] = bad_point;
      matpt[3] = bad_point;
    }
  }

  unsigned u, v;
  for (size_t i = 0; i < popout.indices.size(); ++i)
  {
    u = popout.indices[i]%cloud.width;
    v = popout.indices[i]/cloud.width;

    const pcl::PointXYZRGB &pt = cloud(u, v);
    cv::Vec4f &matpt = matCloud(v,u);

    matpt[0] = pt.x;
    matpt[1] = pt.y;
    matpt[2] = pt.z;
    matpt[3] = pt.rgb;
  }
}

/**
 * Euclidean neighbourhood clustering
 * using a connected component analysis of the point cloud grid
 */
void PlanePopout::LabelClusters(const cv::Mat_<cv::Vec4f> &cloud, 
                                cv::Mat_<ushort> &labels, 
                                vector<unsigned> &sizeClusters)
{
  ccLabeling->Operate(cloud, labels, sizeClusters);
}

/**
 * Create a mask from all selected labels
 */
void PlanePopout::CreateMaskAll(const cv::Mat_<ushort> &labels, 
                                const vector<unsigned> &sizeClusters, 
                                cv::Mat_<uchar> &mask)
{
  ccLabeling->CreateMask(labels, sizeClusters, mask);
}


/**
 * Create mask from clusters with id
 */
void PlanePopout::CreateMaskId(const cv::Mat_<ushort> &labels, const ushort id, cv::Mat_<uchar> &mask)
{
  mask = cv::Mat::zeros(labels.rows, labels.cols, CV_8U);
  
  //filter largest cluster
  for (int v = 0; v < labels.rows; ++v)
  {
    for (int u = 0; u < labels.cols; ++u)
    {
      if (labels(v,u)==id)
      {
        mask(v,u) = 255;
      }
    }
  }
}

/**
 * Create mask from largest cluster
 */
void PlanePopout::CreateMaskLargest(const cv::Mat_<ushort> &labels, 
           const vector<unsigned> &sizeClusters, cv::Mat_<uchar> &mask)
{
  ushort maxLabel = 0;
  unsigned max = 0;

  for (unsigned i=1; i<sizeClusters.size(); i++)
  {
    if (max<sizeClusters[i])
    {
      max=sizeClusters[i];
      maxLabel=i;
    }
  }

  if (max>0)
    CreateMaskId(labels, maxLabel, mask);
  else
    mask = cv::Mat::zeros(labels.rows, labels.cols, CV_8U);
}

// ================================= Private functions ================================= //


}












