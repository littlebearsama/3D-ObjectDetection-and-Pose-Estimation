#include "squirrel_attention_base_test.hpp"
// Bring in my package's API, which is what I'm testing
#include <squirrel_attention/squirrel_attention_itti.hpp>

TEST_F(ClientAttentionBase, testClientAttentionItti_1) 
{
  n_->getParam ( "ground_truth", ground_truth_);
  readGroundTruth();
  
  ros::ServiceClient client = n_->serviceClient<squirrel_object_perception_msgs::GetSaliencyItti>("/squirrel_attention_itti");
  squirrel_object_perception_msgs::GetSaliencyItti srv;
  srv.request.cloud = msg;
  
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
  ros::init(argc, argv, "squirrel_attention_itti_test");
  return RUN_ALL_TESTS();
}