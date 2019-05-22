/*
 * noise_models.h
 *
 *  Created on: Oct 28, 2013
 *      Author: aitor
 */

#ifndef NOISE_MODELS_H_
#define NOISE_MODELS_H_

#include <pcl/common/common.h>
#include <pcl/common/io.h>

namespace faat_pcl
{
  namespace utils
  {
    namespace noise_models
    {
      template<class PointT>
        class NguyenNoiseModel
        {
        private:
          typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;
          typedef typename pcl::PointCloud<pcl::Normal>::Ptr PointNormalTPtr;
          float lateral_sigma_;
          float max_angle_;
          PointTPtr input_;
          PointNormalTPtr normals_;
          std::vector<float> weights_;
          pcl::PointIndices discontinuity_edges_;
          bool use_depth_edges_;
        public:
          NguyenNoiseModel ();

          void
          setInputCloud (PointTPtr & input)
          {
            input_ = input;
          }

          void
          setMaxAngle(float f)
          {
            max_angle_ = f;
          }

          void
          setUseDepthEdges(bool b)
          {
            use_depth_edges_ = b;
          }

          void
          getDiscontinuityEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr & disc)
          {
            disc.reset(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*input_, discontinuity_edges_, *disc);
          }

          //in meters, lateral sigma (3*s used to downweight points close to depth discontinuities)
          void
          setLateralSigma(float s)
          {
            lateral_sigma_ = s;
          }

          void
          setInputNormals (PointNormalTPtr & normals)
          {
            normals_ = normals;
          }

          void
          compute ();

          void
          getWeights (std::vector<float> & weights)
          {
            weights = weights_;
          }

          //void getFilteredCloud(PointTPtr & filtered, float w_t);

          void getFilteredCloudRemovingPoints(PointTPtr & filtered, float w_t);

          void getFilteredCloudRemovingPoints(PointTPtr & filtered, float w_t, std::vector<int> & kept);
        };
    }
  }
}

#endif /* NOISE_MODELS_H_ */
