/*
 * estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_GLOBAL_ESTIMATOR_H_
#define FAAT_PCL_REC_FRAMEWORK_GLOBAL_ESTIMATOR_H_

#include "faat_3d_rec_framework_defines.h"
#include "normal_estimator.h"

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    template <typename PointInT, typename FeatureT>
    class FAAT_3D_FRAMEWORK_API GlobalEstimator {
      protected:
        bool computed_normals_;
        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<FeatureT>::Ptr FeatureTPtr;

        typename boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > normal_estimator_;

        pcl::PointCloud<pcl::Normal>::Ptr normals_;

      public:
        virtual bool
        estimate (const PointInTPtr & in, PointInTPtr & processed, std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<
            pcl::PointCloud<FeatureT> > > & signatures, std::vector<Eigen::Vector3f> & centroids)=0;

        virtual bool computedNormals() = 0;

        void setNormalEstimator(boost::shared_ptr<PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > & ne) {
          normal_estimator_ = ne;
        }

        void getNormals(pcl::PointCloud<pcl::Normal>::Ptr & normals) {
          normals = normals_;
        }

        virtual bool acceptsIndices()
        {
            return false;
        }

        virtual void
        setIndices(pcl::PointIndices & p_indices)
        {

        }

        virtual void
        setIndices(std::vector<int> & p_indices)
        {

        }

    };
  }
}


#endif /* REC_FRAMEWORK_ESTIMATOR_H_ */
