// Bring in my package's API, which is what I'm testing
#include <squirrel_segmentation/squirrel_segmentation.hpp>
// Bring in gtest
#include <gtest/gtest.h>

class ClientSegmenter : public testing::Test 
{
  
protected:
  // Remember that SetUp() is run immediately before a test starts.
  virtual void SetUp() 
  {
    n_ = new ros::NodeHandle ("~");
    n_->getParam ( "cloud_name", cloud_name_);
    readCloud ();
    
    n_->getParam ( "ground_truth", ground_truth_);
    readGroundTruth();
    
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
    
//     cv::imshow("saliency",saliency);
//     cv::waitKey();
    
    ROS_INFO("Saliency is up and running!");
  }

public:

};

TEST_F(ClientSegmenter, testClientSegmenter_1) 
{ 
  std::cout << "going to call service..." << std::endl;
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::Segment>("/squirrel_segmentation");
  squirrel_object_perception_msgs::Segment srv;
  srv.request.cloud = msg;
  
  EXPECT_TRUE(client.call(srv));

//   cv::Mat segmented_object = cv::Mat_<uchar>::zeros(groundTruth.rows,groundTruth.cols);
  for(size_t k=0; k < srv.response.clusters_indices.size(); k++)
  {
    for(size_t i=0; i < srv.response.clusters_indices[k].data.size(); i++)
    { 
      int idx = srv.response.clusters_indices[k].data[i];
    
//       segmented_object.at<uchar>(idx/groundTruth.cols,idx%groundTruth.cols) = k+1;
      EXPECT_EQ(k+1,groundTruth.at<uchar>(idx/groundTruth.cols,idx%groundTruth.cols));
    }
  }
  
//   cv::imshow("segmented_object",segmented_object);
//   cv::imshow("groundTruth",groundTruth);
//   cv::waitKey();
  
}

int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "squirrel_segmentation_test");
  return RUN_ALL_TESTS();
}
