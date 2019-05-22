// Bring in my package's API, which is what I'm testing
#include <squirrel_segmentation/squirrel_segmentation_incremental.hpp>
// Bring in gtest
#include <gtest/gtest.h>

class ClientSegmenterIncremental : public testing::Test 
{
  
protected:
  // Remember that SetUp() is run immediately before a test starts.
  virtual void SetUp() 
  {
    n_ = new ros::NodeHandle ("~");
    n_->getParam ( "cloud_name", cloud_name_);
    readCloud ();
    
    n_->getParam ( "saliency_name", saliency_name_);
    readSaliency();
    
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
  
  std::string saliency_name_;
  cv::Mat saliency;
  sensor_msgs::Image im;
  
  std::string ground_truth_;
  cv::Mat groundTruth;
  
  //int service_calls_;
  //ros::NodeHandle n_;
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
  
  void readSaliency ()
  {
    saliency = cv::imread(saliency_name_,-1);
    
//     cv::imshow("saliency",saliency);
//     cv::waitKey();
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    // convert OpenCV image to ROS message
    cv_ptr->header.stamp = time;
    cv_ptr->header.frame_id = "saliency_map";
    cv_ptr->encoding = "mono8";
    cv_ptr->image = saliency;
    cv_ptr->toImageMsg(im);
    ROS_INFO("Ground Truth is up and running!");
  }

public:

};

TEST_F(ClientSegmenterIncremental, testClientSegmenterIncremental_1) 
{ 
  std::cout << "going to call service..." << std::endl;
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::SegmentInit>("/squirrel_segmentation_incremental_init");
  squirrel_object_perception_msgs::SegmentInit srv;
  srv.request.cloud = msg;
  srv.request.saliency_map = im;
  
  EXPECT_TRUE(client.call(srv));
}

TEST_F(ClientSegmenterIncremental, testClientSegmenterIncremental_2) 
{
  ros::ServiceClient client2 = n_->serviceClient<squirrel_object_perception_msgs::SegmentOnce>("/squirrel_segmentation_incremental_once");
  squirrel_object_perception_msgs::SegmentOnce srv2;
  
  EXPECT_TRUE(client2.call(srv2));
  
  EXPECT_TRUE(srv2.response.clusters_indices.size() == 1);

//   cv::Mat segmented_object = cv::Mat_<uchar>::zeros(groundTruth.rows,groundTruth.cols);
  for(size_t k=0; k < srv2.response.clusters_indices[0].data.size(); k++)
  {
    int idx = srv2.response.clusters_indices[0].data[k];
    
    //segmented_object.at<uchar>(idx/groundTruth.cols,idx%groundTruth.cols) = 255;
    EXPECT_EQ(255,groundTruth.at<uchar>(idx/groundTruth.cols,idx%groundTruth.cols));
  }
  
  //cv::imshow("segmented_object",segmented_object);
  //cv::waitKey();
  
}

int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "squirrel_segmentation_incremental_test");
  return RUN_ALL_TESTS();
}
