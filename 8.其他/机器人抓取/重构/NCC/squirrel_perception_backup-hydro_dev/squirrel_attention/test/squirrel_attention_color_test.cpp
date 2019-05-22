#include "squirrel_attention_base_test.hpp"
// Bring in my package's API, which is what I'm testing
#include <squirrel_attention/squirrel_attention_color.hpp>

TEST_F(ClientAttentionBase, testClientAttentionColor_1) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  ground_truth_ = ground_truth_ + "_0_0_255.png";
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencyColor>("/squirrel_attention_color");
  squirrel_object_perception_msgs::GetSaliencyColor srv;
  srv.request.cloud = msg;
  srv.request.color.x = 0;
  srv.request.color.y = 0;
  srv.request.color.z = 255;
  
  EXPECT_TRUE(client.call(srv));
  
  //msg->cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat saliency_map = cv_ptr->image;
  
//   cv::imshow("groundTruth",groundTruth);
//   cv::waitKey();
  
  for(int i = 0; i < saliency_map.rows; ++i)
  {
    for(int j = 0; j < saliency_map.cols; ++j)
    {
      EXPECT_EQ(saliency_map.at<uchar>(i,j),groundTruth.at<uchar>(i,j));
    }
  }
}

TEST_F(ClientAttentionBase, testClientAttentionColor_2) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  ground_truth_ = ground_truth_ + "_255_0_0.png";
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencyColor>("/squirrel_attention_color");
  squirrel_object_perception_msgs::GetSaliencyColor srv;
  srv.request.cloud = msg;
  srv.request.color.x = 255;
  srv.request.color.y = 0;
  srv.request.color.z = 0;
  
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
  ros::init(argc, argv, "squirrel_attention_color_test");
  return RUN_ALL_TESTS();
}