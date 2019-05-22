#include "squirrel_attention_base_test.hpp"
// Bring in my package's API, which is what I'm testing
#include <squirrel_attention/squirrel_attention_surfaceorientation.hpp>

TEST_F(ClientAttentionBase, testClientAttentionHeight_1) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  ground_truth_ = ground_truth_ + "_0.png";
//   std::cerr << ground_truth_ << std::endl;
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencySurfaceOrientation>("/squirrel_attention_surfaceorientation");
  squirrel_object_perception_msgs::GetSaliencySurfaceOrientation srv;
  srv.request.cloud = msg;
  srv.request.orientation_type.data = 0;
  
  EXPECT_TRUE(client.call(srv));
  
  //msg->cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat saliency_map = cv_ptr->image;
  
//   cv::imshow("groundTruth",groundTruth);
//   cv::imshow("saliency_map",saliency_map);
//   cv::waitKey();
  
  for(int i = 0; i < saliency_map.rows; ++i)
  {
    for(int j = 0; j < saliency_map.cols; ++j)
    {
      EXPECT_EQ(saliency_map.at<uchar>(i,j),groundTruth.at<uchar>(i,j));
    }
  }
}

TEST_F(ClientAttentionBase, testClientAttentionHeight_2) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  ground_truth_ = ground_truth_ + "_1.png";
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencySurfaceOrientation>("/squirrel_attention_surfaceorientation");
  squirrel_object_perception_msgs::GetSaliencySurfaceOrientation srv;
  srv.request.cloud = msg;
  srv.request.orientation_type.data = 1;
  
  EXPECT_TRUE(client.call(srv));
  
  //msg->cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(srv.response.saliency_map, sensor_msgs::image_encodings::MONO8);
  cv::Mat saliency_map = cv_ptr->image;
  
//   cv::imshow("groundTruth",groundTruth);
//   cv::imshow("saliency_map",saliency_map);
//   cv::waitKey();
  
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
  ros::init(argc, argv, "squirrel_attention_surfaceorientation_test");
  return RUN_ALL_TESTS();
}