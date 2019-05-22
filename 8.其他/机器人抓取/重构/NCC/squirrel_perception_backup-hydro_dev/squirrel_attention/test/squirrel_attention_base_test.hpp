#ifndef SQUIRREL_ATTENTION_BASE_TEST_HPP_
#define SQUIRREL_ATTENTION_BASE_TEST_HPP_

// Bring in gtest
#include <gtest/gtest.h>
#include <squirrel_attention/squirrel_attention_base.hpp>

class ClientAttentionBase : public testing::Test 
{
  
protected:
  // Remember that SetUp() is run immediately before a test starts.
  virtual void SetUp() 
  {
    n_ = new ros::NodeHandle ("~");
    n_->getParam ( "cloud_name", cloud_name_);
    readCloud ();
  }

  // TearDown() is invoked immediately after a test finishes.
  virtual void TearDown() 
  {
    if(n_)
      delete n_;
  }

  //memebers
  typedef pcl::PointXYZRGB PointT;
  std::string cloud_name_;
  pcl::PointCloud<PointT>::Ptr cloud_;
  sensor_msgs::PointCloud2 msg;
  
  std::string ground_truth_;
  cv::Mat groundTruth;
  
  //int service_calls_;
  ros::NodeHandle *n_;
  
  void readCloud ()
  {
    cloud_.reset(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT> (cloud_name_.c_str(), *cloud_) == -1) //* load the file
    {
      ROS_ERROR ("Couldn't read file!\n");
      return;
    }
    
    pcl::toROSMsg (*cloud_,msg);
    ROS_INFO("Cloud is up and running!");
  }
  
  void readGroundTruth ()
  {
    groundTruth= cv::imread(ground_truth_,-1);
    ROS_INFO("Ground Truth is up and running!");
  }

public:

};

#endif //SQUIRREL_ATTENTION_BASE_TEST_HPP_