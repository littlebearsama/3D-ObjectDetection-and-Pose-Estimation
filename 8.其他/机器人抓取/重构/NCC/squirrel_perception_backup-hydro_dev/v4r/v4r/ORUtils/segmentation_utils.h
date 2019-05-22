#ifndef SEGMENTATION_UTILS_H_
#define SEGMENTATION_UTILS_H_

#include <vector>
#include <pcl/features/integral_image_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/plane_refinement_comparator.h>

namespace faat_pcl
{
    namespace utils
    {
        template<class PointT>
        void computeTablePlane (typename pcl::PointCloud<PointT>::Ptr & pCloud,
                         Eigen::Vector4f & table_plane,
                         float z_dist = std::numeric_limits<float>::max());
    }
}

#endif /* SEGMENTATION_UTILS_H_ */
