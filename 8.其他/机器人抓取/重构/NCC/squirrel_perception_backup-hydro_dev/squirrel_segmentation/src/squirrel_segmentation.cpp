/*
 * squirrel_segmentation.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_segmentation/squirrel_segmentation.hpp>
  
bool
SegmenterComplete::segment (squirrel_object_perception_msgs::Segment::Request & req, squirrel_object_perception_msgs::Segment::Response & response)
{
  //get point cloud
  pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);
  ROS_INFO ("Number of points in the scene: %ld", scene->points.size());
    
  segmenter_->setPointCloud(scene);
  
  ROS_INFO ("Going to segment the scene.");
  
  segmenter_->segment();
  
  std::vector<std::vector<int> > clusters = segmenter_->getSegmentedObjectsIndices();
    
  ROS_INFO ("Number of segmented objects: %ld",clusters.size());
  
  for(size_t i=0; i < clusters.size(); i++)
  {
    std_msgs::Int32MultiArray indx;
    for(size_t k=0; k < clusters[i].size(); k++)
    {
      indx.data.push_back(clusters[i][k]);
    }
    response.clusters_indices.push_back(indx);
  }
    
  return true;
}

SegmenterComplete::SegmenterComplete ()
{
  //default values
}

SegmenterComplete::~SegmenterComplete ()
{
  if(n_)
    delete n_;
}

void
SegmenterComplete::initialize (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_segmentation_server");
  n_ = new ros::NodeHandle ("~");
  n_->getParam ( "model_filename", model_filename_);
  n_->getParam ( "scaling_filename", scaling_filename_);
  
  if (model_filename_.compare ("") == 0)
  {
    ROS_ERROR ("Set -model_filename option in the command line, ABORTING");
    return;
  }
    
  if (scaling_filename_.compare ("") == 0)
  {
    ROS_ERROR ("Set -scaling_filename option in the command line, ABORTING");
    return;
  }
    
  segmenter_.reset(new segmentation::Segmenter);
  segmenter_->setModelFilename(model_filename_);
  segmenter_->setScaling(scaling_filename_);
    
  Segment_ = n_->advertiseService ("/squirrel_segmentation", &SegmenterComplete::segment, this);
  ROS_INFO ("Ready to get service calls...");
  ros::spin ();
}


int
main (int argc, char ** argv)
{
  SegmenterComplete m;
  m.initialize (argc, argv);

  return 0;
}
