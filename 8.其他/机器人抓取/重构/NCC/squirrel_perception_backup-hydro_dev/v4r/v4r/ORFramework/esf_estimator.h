/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_ESF_ESTIMATOR_H_
#define REC_FRAMEWORK_ESF_ESTIMATOR_H_

#include "faat_3d_rec_framework_defines.h"
#include "global_estimator.h"
#include <pcl/features/esf.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    template<typename PointInT, typename FeatureT>
      class FAAT_3D_FRAMEWORK_API ESFEstimation : public GlobalEstimator<PointInT, FeatureT>
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;

      public:
        bool
        estimate (const PointInTPtr & in, PointInTPtr & processed,
                  typename pcl::PointCloud<FeatureT>::CloudVectorType & signatures,
                  std::vector<Eigen::Vector3f> & centroids)
        {

          if(!in)
          {
              PCL_ERROR("ESFEstimation, input is empty!");
              return false;
          }

          if(in->points.size() == 0)
          {
              PCL_ERROR("ESFEstimation, input has no points!");
              return false;
          }

          typedef typename pcl::ESFEstimation<PointInT, FeatureT> ESFEstimation;
          pcl::PointCloud<FeatureT> ESF_signature;

          ESFEstimation esf;
          esf.setInputCloud (in);
          esf.compute (ESF_signature);

          signatures.resize (1);
          centroids.resize (1);

          signatures[0] = ESF_signature;

          Eigen::Vector4f centroid4f;
          pcl::compute3DCentroid (*in, centroid4f);
          centroids[0] = Eigen::Vector3f (centroid4f[0], centroid4f[1], centroid4f[2]);

          pcl::copyPointCloud(*in, *processed);

          return true;
        }

        bool
        computedNormals ()
        {
          return false;
        }
      };
  }
}

#endif /* REC_FRAMEWORK_ESF_ESTIMATOR_H_ */
