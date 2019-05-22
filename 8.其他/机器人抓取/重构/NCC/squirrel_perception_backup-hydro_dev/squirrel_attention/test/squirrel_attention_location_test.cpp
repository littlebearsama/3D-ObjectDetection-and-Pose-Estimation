#include "squirrel_attention_base_test.hpp"
// Bring in my package's API, which is what I'm testing
#include <squirrel_attention/squirrel_attention_location.hpp>

TEST_F(ClientAttentionBase, testClientAttentionLocation_1) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  ground_truth_ = ground_truth_ + "_0_0_0.png";
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencyLocation>("/squirrel_attention_location");
  squirrel_object_perception_msgs::GetSaliencyLocation srv;
  srv.request.cloud = msg;
  srv.request.location.data = 0;
  
  EXPECT_TRUE(client.call(srv));
  
  //msg->cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat saliency_map = cv_ptr->image;
  
  for(int i = 0; i < saliency_map.rows; ++i)
  {
    for(int j = 0; j < saliency_map.cols; ++j)
    {
      EXPECT_EQ(saliency_map.at<uchar>(i,j),groundTruth.at<uchar>(i,j));
    }
  }
}

TEST_F(ClientAttentionBase, testClientAttentionLocation_2) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  ground_truth_ = ground_truth_ + "_9_50_50.png";
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencyLocation>("/squirrel_attention_location");
  squirrel_object_perception_msgs::GetSaliencyLocation srv;
  srv.request.cloud = msg;
  srv.request.location.data = 9;
  srv.request.center.x = 50;
  srv.request.center.y = 50;
  
  EXPECT_TRUE(client.call(srv));
  
  //msg->cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat saliency_map = cv_ptr->image;
  
  for(int i = 0; i < saliency_map.rows; ++i)
  {
    for(int j = 0; j < saliency_map.cols; ++j)
    {
      EXPECT_EQ(saliency_map.at<uchar>(i,j),groundTruth.at<uchar>(i,j));
    }
  }
}

int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "squirrel_attention_location_test");
  return RUN_ALL_TESTS();
}