/*
 * shape_simple_classifier_node.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: Aitor Aldoma, aldoma@acin.tuwien.ac.at
 *      Author: Michael Zillich, zillich@acin.tuwien.ac.at
 */

#include <sstream>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/lexical_cast.hpp>
#include <Eigen/Eigenvalues>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <v4r/ORFramework/mesh_source.h>
#include <v4r/ORFramework/vfh_estimator.h>
#include <v4r/ORFramework/esf_estimator.h>
#include <v4r/ORFramework/cvfh_estimator.h>
#include <v4r/ORFramework/metrics.h>
#include <v4r/ORFramework/global_nn_classifier.h>

#include <squirrel_object_perception_msgs/Classify.h>
#include <squirrel_object_perception_msgs/Classification.h>
#include <squirrel_classification/pcl_conversions.h>

class ShapeClassifier
{
  private:
    typedef pcl::PointXYZ PointT;
    std::string models_dir_;
    std::string training_dir_;
    std::string desc_name_;
    int NN_;
    pcl::PointCloud<PointT>::Ptr frame_;
    std::vector<pcl::PointIndices> cluster_indices_;
    std::vector < std::string > categories_;
    std::vector<float> conf_;

    boost::shared_ptr<faat_pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640> > classifier_;
    ros::ServiceServer segment_and_classify_service_;
    ros::ServiceServer classify_service_;
    ros::NodeHandle *n_;
    ros::Publisher vis_pub_, vis_pc_pub_;
    visualization_msgs::MarkerArray markerArray_;
    std::string camera_frame_;

    /**
     * The actual classification method, calling the v4r classification library.
     * TODO: separate visualisation code from work code
     * TODO: clean up comments
     * TODO: clean up formatting
     */
    bool classify(squirrel_object_perception_msgs::Classify::Request & req,
                  squirrel_object_perception_msgs::Classify::Response & response)
    {
      ROS_INFO("Classifying %d objects\n", (int)req.clusters_indices.size());
      pcl::fromROSMsg(req.cloud, *frame_);
      classifier_->setInputCloud(frame_);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pClusteredPCl (new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::copyPointCloud(*frame_, *pClusteredPCl);

      //clear all data from previous visualization steps and publish empty markers/point cloud
      for (size_t i=0; i < markerArray_.markers.size(); i++)
      {
        markerArray_.markers[i].action = visualization_msgs::Marker::DELETE;
      }
      vis_pub_.publish( markerArray_ );
      sensor_msgs::PointCloud2 scenePc2;
      vis_pc_pub_.publish(scenePc2);
      markerArray_.markers.clear();

      for(size_t i=0; i < req.clusters_indices.size(); i++)
      {
        std::vector<int> cluster_indices_int;
        float r = std::rand() % 255;
        float g = std::rand() % 255;
        float b = std::rand() % 255;

        for(size_t kk=0; kk < req.clusters_indices[i].data.size(); kk++)
        {
          cluster_indices_int.push_back(static_cast<int>(req.clusters_indices[i].data[kk]));
          pClusteredPCl->at(req.clusters_indices[i].data[kk]).r = r;
          pClusteredPCl->at(req.clusters_indices[i].data[kk]).g = g;
          pClusteredPCl->at(req.clusters_indices[i].data[kk]).b = b;
        }

        classifier_->setIndices(cluster_indices_int);
        classifier_->classify ();
        classifier_->getCategory (categories_);
        classifier_->getConfidence (conf_);

        std::cout << "for cluster " << i << " with size " << cluster_indices_int.size() << ", I have following hypotheses: " << std::endl;

        squirrel_object_perception_msgs::Classification class_tmp;
        for(size_t kk=0; kk < categories_.size(); kk++)
        {
          std::cout << categories_[kk] << " with confidence " << conf_[kk] << std::endl;
          std_msgs::String str_tmp;
          str_tmp.data = categories_[kk];
          class_tmp.class_type.push_back(str_tmp);
          class_tmp.confidence.push_back(conf_[kk]);
        }
        response.class_results.push_back(class_tmp);

        // visualize the result as ROS topic
        
        // get the centroid
        Eigen::Vector4f centroid;
        Eigen::Matrix3f covariance_matrix;
        pcl::computeMeanAndCovarianceMatrix(*frame_, cluster_indices_int, covariance_matrix, centroid);
        geometry_msgs::Point32 centroid_ros;
        centroid_ros.x = centroid[0];
        centroid_ros.y = centroid[1];
        centroid_ros.z = centroid[2];

        visualization_msgs::Marker marker;
        marker.header.frame_id = camera_frame_;
        marker.header.stamp = ros::Time::now();
        //marker.header.seq = ++marker_seq_;
        marker.ns = "object_classification";
        marker.id = i;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = centroid_ros.x;
        marker.pose.position.y = centroid_ros.y;
        marker.pose.position.z = centroid_ros.z;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.8*r/255.f;
        marker.color.g = 0.8*g/255.f;
        marker.color.b = 0.8*b/255.f;
        std::stringstream marker_text;
        marker_text << categories_[0] << conf_[0];
        marker.text = marker_text.str();
        markerArray_.markers.push_back(marker);

        /*boost::shared_ptr<pcl::visualization::PCLVisualizer> vis;
        vis.reset(new pcl::visualization::PCLVisualizer("cluster visualization"));
        vis->addCoordinateSystem(0.2f);
        vis->addPointCloud(pClusterPCl_transformed, "scene_cloud_eigen_transformed");
        vis->spin();*/

//          if(categories_.size() == 1)
//          {
//            std_msgs::String ss;
//            ss.data = categories_[0];
//            response.categories.push_back(ss);
//            response.confidence.push_back(conf_[0]);
//          }
//          else if(categories_.size() == 0)
//          {
//            //weird case, do nothing...
//          }
//          else
//          {
//            //at least 2 categories
//            std::vector< std::pair<float, std::string> > conf_categories_map_;
//            for (size_t kk = 0; kk < categories_.size (); kk++)
//            {
//              conf_categories_map_.push_back(std::make_pair(conf_[kk], categories_[kk]));
//            }

//            std::sort (conf_categories_map_.begin (), conf_categories_map_.end (),
//                       boost::bind (&std::pair<float, std::string>::first, _1) > boost::bind (&std::pair<float, std::string>::first, _2));

//            /*for (size_t kk = 0; kk < categories.size (); kk++)
//            {
//              std::cout << conf_categories_map_[kk].first << std::endl;
//            }*/

//            if( (conf_categories_map_[1].first / conf_categories_map_[0].first) < 0.85f)
//            {


//              if (!boost::starts_with(conf_categories_map_[0].second, "unknown"))
//              {
//                std_msgs::String ss;
//                ss.data = conf_categories_map_[0].second;
//                response.categories.push_back(ss);
//                response.confidence.push_back(conf_categories_map_[0].first);
//              }
//            }
//          }
      }

      pcl::toROSMsg (*pClusteredPCl, scenePc2);
      vis_pc_pub_.publish(scenePc2);
      vis_pub_.publish( markerArray_ );

      return true;
    }

  public:
    ShapeClassifier()
    {
      //default values
      desc_name_ = "esf";
      NN_ = 50;
      frame_.reset(new pcl::PointCloud<PointT>());
    }

    void initialize(int argc, char ** argv)
    {
      ros::init (argc, argv, "classifier_service");
      n_ = new ros::NodeHandle ( "~" );
      n_->getParam ( "models_dir", models_dir_ );
      n_->getParam ( "training_dir", training_dir_ );
      n_->getParam ( "descriptor_name", desc_name_ );
      n_->getParam ( "nn", NN_ );

      if(!n_->getParam ( "camera_frame", camera_frame_ ))
	camera_frame_ = "/head_xtion_depth_optical_frame";

      ROS_INFO("models_dir, training dir, desc, camera_frame:  %s, %s, %s, %s",  models_dir_.c_str(), training_dir_.c_str(), desc_name_.c_str(), camera_frame_.c_str());

      if(models_dir_.compare("") == 0)
      {
        PCL_ERROR("Set -models_dir option in the command line, ABORTING");
        return;
      }

      if(training_dir_.compare("") == 0)
      {
        PCL_ERROR("Set -training_dir option in the command line, ABORTING");
        return;
      }

      boost::shared_ptr<faat_pcl::rec_3d_framework::MeshSource<PointT> > mesh_source (new faat_pcl::rec_3d_framework::MeshSource<PointT>);
      mesh_source->setPath (models_dir_);
      mesh_source->setResolution (150);
      mesh_source->setTesselationLevel (0);
      mesh_source->setViewAngle (57.f);
      mesh_source->setRadiusSphere (1.f);
      mesh_source->setModelScale (1.f);
      mesh_source->generate (training_dir_);

      boost::shared_ptr<faat_pcl::rec_3d_framework::Source<PointT> > cast_source;
      cast_source = boost::static_pointer_cast<faat_pcl::rec_3d_framework::MeshSource<PointT> > (mesh_source);

      boost::shared_ptr<faat_pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640> > estimator;
      estimator.reset (new faat_pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640>);

      boost::shared_ptr<faat_pcl::rec_3d_framework::GlobalEstimator<PointT, pcl::ESFSignature640> > cast_estimator;
      cast_estimator = boost::dynamic_pointer_cast<faat_pcl::rec_3d_framework::ESFEstimation<PointT, pcl::ESFSignature640> > (estimator);

      classifier_.reset(new faat_pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, PointT, pcl::ESFSignature640>);
      classifier_->setDataSource (cast_source);
      classifier_->setTrainingDir (training_dir_);
      classifier_->setDescriptorName (desc_name_);
      classifier_->setFeatureEstimator (cast_estimator);
      classifier_->setNN (NN_);
      classifier_->initialize (false);

      classify_service_ = n_->advertiseService("/squirrel_classify", &ShapeClassifier::classify, this);
      vis_pub_ = n_->advertise<visualization_msgs::MarkerArray>( "visualization_marker", 0 );
      vis_pc_pub_ = n_->advertise<sensor_msgs::PointCloud2>( "clusters", 1 );
      ros::spin();
    }
};

int main (int argc, char ** argv)
{
  ShapeClassifier m;
  m.initialize (argc, argv);

  return 0;
}
