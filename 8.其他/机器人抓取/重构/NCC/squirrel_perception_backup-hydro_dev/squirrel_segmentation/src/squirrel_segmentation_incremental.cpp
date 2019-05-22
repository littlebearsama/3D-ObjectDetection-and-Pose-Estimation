/*
 * squirrel_segmentation_incremental.cpp
 *
 *  Created on: Nov 7, 2014
 *      Author: Ekaterina Potapova
 */

#include <squirrel_segmentation/squirrel_segmentation_incremental.hpp>

bool
SegmenterIncremental::segmentInit (squirrel_object_perception_msgs::SegmentInit::Request & req, squirrel_object_perception_msgs::SegmentInit::Response & response)
{
  //get point cloud
  pcl::PointCloud<PointT>::Ptr scene (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg (req.cloud, *scene);
  ROS_INFO ("Number of points in the scene: %ld", scene->points.size());

  //get saliency map
  std::vector<cv::Mat> saliencyMaps;
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(req.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat salMap = cv_ptr->image;
  salMap.convertTo(salMap,CV_32F,1.0/255);

//   cv::imshow("salMap",salMap);
//   cv::waitKey(-1);

  saliencyMaps.resize(1);
  salMap.copyTo(saliencyMaps.at(0));

  segmenter_->setPointCloud(scene);
  segmenter_->setSaliencyMaps(saliencyMaps);

  segmenter_->attentionSegmentInit();

  ROS_INFO("Finished initialization.");
  return true;
}

bool
SegmenterIncremental::segmentOnce (squirrel_object_perception_msgs::SegmentOnce::Request & req, squirrel_object_perception_msgs::SegmentOnce::Response & response)
{
  ROS_INFO ("Going to segment an object.");

  if(!(segmenter_->attentionSegmentNext()))
  {
    ROS_INFO("The object that is going to be returned is emty. There can be two reasons for that: 1) the whole scene was segmented; 2) the object you are trying segment was already segmented from this view.");
  }

  std::vector<std::vector<int> > clusters = segmenter_->getSegmentedObjectsIndices();

  ROS_INFO ("Number of segmented objects: %ld",clusters.size());
  assert(clusters.size() == 1);

  std_msgs::Int32MultiArray indx;
  for(size_t k=0; k < clusters[0].size(); k++)
  {
    indx.data.push_back(clusters[0][k]);
  }
  response.clusters_indices.clear();
  response.clusters_indices.push_back(indx);

  return true;
}

SegmenterIncremental::SegmenterIncremental ()
{
  //default values
}

SegmenterIncremental::~SegmenterIncremental ()
{
  if(n_)
    delete n_;
}

void
SegmenterIncremental::initialize (int argc, char ** argv)
{
  ros::init (argc, argv, "squirrel_segmentation_incremental_server");
  n_ = new ros::NodeHandle ("~");
  n_->getParam ( "model_filename", model_filename_);
  n_->getParam ( "scaling_filename", scaling_filename_);

  //model_filename_ = "/home/ekaterina/work/catkin_ws/squirrel_ros/object_perception/squirrel_segmentation/data/ST-TrainAll.txt.model";
  //scaling_filename_ = "/home/ekaterina/work/catkin_ws/squirrel_ros/object_perception/squirrel_segmentation/data/ST-TrainAll.txt.scalingparams";

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

  SegmentInit_ = n_->advertiseService ("/squirrel_segmentation_incremental_init", &SegmenterIncremental::segmentInit, this);
  SegmentOnce_ = n_->advertiseService ("/squirrel_segmentation_incremental_once", &SegmenterIncremental::segmentOnce, this);
  ROS_INFO ("Ready to get service calls...");
  ros::spin ();
}


int
main (int argc, char ** argv)
{
  SegmenterIncremental m;
  m.initialize (argc, argv);

  return 0;
}
