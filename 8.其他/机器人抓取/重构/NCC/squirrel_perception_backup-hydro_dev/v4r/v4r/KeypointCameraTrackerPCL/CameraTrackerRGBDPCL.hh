/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_CAMERA_TRACKER_PCL_HH
#define KP_CAMERA_TRACKER_PCL_HH

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "v4r/KeypointCameraTracker/CameraTrackerRGBD.hh"
#include "v4r/KeypointTools/DataMatrix2D.hpp"
#include "v4r/KeypointTools/PointTypes.hpp"


namespace kp
{

class CameraTrackerRGBDPCL : public CameraTrackerRGBD 
{
private:
  kp::DataMatrix2D<kp::PointXYZRGB> kp_cloud;
 
public:

  CameraTrackerRGBDPCL(const CameraTrackerRGBD::Parameter &p=Parameter());
  ~CameraTrackerRGBDPCL();

  bool trackPCL(pcl::PointCloud<pcl::PointXYZRGB> &cloud, Eigen::Matrix4f &pose, 
        const cv::Mat_<unsigned char> &mask=cv::Mat());

  void operate(const std::vector< pcl::PointCloud<pcl::PointXYZRGB>::Ptr > &clouds, 
        std::vector<Eigen::Matrix4f> &poses, std::vector<bool> &is_keyframe, 
        bool do_bundle_adjustment=true);
  

  typedef SmartPtr< ::kp::CameraTrackerRGBDPCL> Ptr;
  typedef SmartPtr< ::kp::CameraTrackerRGBDPCL const> ConstPtr;
};



/*************************** INLINE METHODES **************************/

} //--END--

#endif

