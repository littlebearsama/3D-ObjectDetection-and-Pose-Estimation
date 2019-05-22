/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "CameraTrackerRGBDPCL.hh"
//#include "v4r/KeypointTools/getImageKPtoCV.hpp"
#include "v4r/KeypointConversions/convertImage.hpp"
#include "v4r/KeypointConversions/convertCloud.hpp"
#include <opencv2/highgui/highgui.hpp>




namespace kp
{


using namespace std;


/************************************************************************************
 * Constructor/Destructor
 */
CameraTrackerRGBDPCL::CameraTrackerRGBDPCL(const CameraTrackerRGBD::Parameter &p)
 : CameraTrackerRGBD(p)
{ 
}

CameraTrackerRGBDPCL::~CameraTrackerRGBDPCL()
{
}

/**
 * track
 */
bool CameraTrackerRGBDPCL::trackPCL(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Matrix4f &pose, const cv::Mat_<unsigned char> &mask)
{
  kp::convertCloud(cloud, kp_cloud);

  return track(kp_cloud, pose, mask);
}

/**
 * operate
 * needs an ordered set of pointclouds
 */
void CameraTrackerRGBDPCL::operate(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &clouds, std::vector<Eigen::Matrix4f> &poses, std::vector<bool> &is_keyframe, bool do_bundle_adjustment)
{
  // set new data container
  Scene::Ptr tmp_scene( new Scene() );
  setSharedData( tmp_scene );

  poses.resize(clouds.size());
  is_keyframe.resize(clouds.size());

  // process/ track pointclouds
  for (unsigned i=0; i<clouds.size(); i++)
  {
    if (!dbg.empty()) {
       kp::convertImage(*clouds[i], dbg);
    }

    trackPCL(*clouds[i], poses[i]);
    is_keyframe[i] = isKeyframe();

    if (!dbg.empty()) {
      cv::imshow("image", dbg);
      cv::waitKey(10);
    }
  }

  // do a full bundle adjustment
  if (do_bundle_adjustment)
  {
    doFullBundleAdjustment();
    
    Scene &ref = *getSharedData();

    for (unsigned i=0; i<ref.views.size(); i++)
      poses[i] = ref.views[i]->pose;
  }

  // clean up data container
  tmp_scene.reset( new Scene() );
  setSharedData( tmp_scene );
}
  



}












