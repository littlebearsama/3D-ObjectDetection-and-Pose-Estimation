#include "segmentation_utils.h"

namespace faat_pcl
{
    namespace utils
    {
        template<class PointT>
        void computeTablePlane (typename pcl::PointCloud<PointT>::Ptr & pCloud,
                         Eigen::Vector4f & table_plane,
                         float z_dist)
      {
            typedef typename pcl::PointCloud<PointT> PointCloudT;
            typedef typename PointCloudT::Ptr PointCloudTPtr;

            pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
            ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
            ne.setMaxDepthChangeFactor (0.02f);
            ne.setNormalSmoothingSize (20.0f);
            ne.setBorderPolicy (pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_IGNORE);
            ne.setInputCloud (pCloud);
            pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
            ne.compute (*normal_cloud);

            int num_plane_inliers = 5000;

            PointCloudTPtr cloud_filtered (new PointCloudT);
            pcl::PassThrough<PointT> pass_;
            pass_.setFilterLimits (0.f, z_dist);
            pass_.setFilterFieldName ("z");
            pass_.setInputCloud (pCloud);
            pass_.setKeepOrganized (true);
            pass_.filter (*cloud_filtered);

            pcl::OrganizedMultiPlaneSegmentation<PointT, pcl::Normal, pcl::Label> mps;
            mps.setMinInliers (num_plane_inliers);
            mps.setAngularThreshold (0.017453 * 1.5f); // 2 degrees
            mps.setDistanceThreshold (0.01); // 1cm
            mps.setInputNormals (normal_cloud);
            mps.setInputCloud (cloud_filtered);

            std::vector<pcl::PlanarRegion<PointT>, Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > > regions;
            std::vector<pcl::ModelCoefficients> model_coefficients;
            std::vector<pcl::PointIndices> inlier_indices;
            pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
            std::vector<pcl::PointIndices> label_indices;
            std::vector<pcl::PointIndices> boundary_indices;

            typename pcl::PlaneRefinementComparator<PointT, pcl::Normal, pcl::Label>::Ptr ref_comp (
                        new pcl::PlaneRefinementComparator<PointT,
                        pcl::Normal, pcl::Label> ());
            ref_comp->setDistanceThreshold (0.01f, true);
            ref_comp->setAngularThreshold (0.017453 * 10);
            mps.setRefinementComparator (ref_comp);
            mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);

            std::cout << "Number of planes found:" << model_coefficients.size () << std::endl;

            int table_plane_selected = 0;
            int max_inliers_found = -1;
            std::vector<int> plane_inliers_counts;
            plane_inliers_counts.resize (model_coefficients.size ());

            for (size_t i = 0; i < model_coefficients.size (); i++)
            {
                Eigen::Vector4f table_plane = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2],
                                                               model_coefficients[i].values[3]);

                std::cout << "Number of inliers for this plane:" << inlier_indices[i].indices.size () << std::endl;
                int remaining_points = 0;
                PointCloudTPtr plane_points (new PointCloudT (*cloud_filtered));
                for (int j = 0; j < plane_points->points.size (); j++)
                {
                    Eigen::Vector3f xyz_p = plane_points->points[j].getVector3fMap ();

                    if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
                        continue;

                    float val = xyz_p[0] * table_plane[0] + xyz_p[1] * table_plane[1] + xyz_p[2] * table_plane[2] + table_plane[3];

                    if (std::abs (val) > 0.01)
                    {
                        plane_points->points[j].x = std::numeric_limits<float>::quiet_NaN ();
                        plane_points->points[j].y = std::numeric_limits<float>::quiet_NaN ();
                        plane_points->points[j].z = std::numeric_limits<float>::quiet_NaN ();
                    }
                    else
                        remaining_points++;
                }

                plane_inliers_counts[i] = remaining_points;

                if (remaining_points > max_inliers_found)
                {
                    table_plane_selected = i;
                    max_inliers_found = remaining_points;
                }
            }

            size_t itt = static_cast<size_t> (table_plane_selected);
            table_plane = Eigen::Vector4f (model_coefficients[itt].values[0], model_coefficients[itt].values[1], model_coefficients[itt].values[2],
                                           model_coefficients[itt].values[3]);

            Eigen::Vector3f normal_table = Eigen::Vector3f (model_coefficients[itt].values[0], model_coefficients[itt].values[1],
                                                            model_coefficients[itt].values[2]);

            int inliers_count_best = plane_inliers_counts[itt];

            //check that the other planes with similar normal are not higher than the table_plane_selected
            for (size_t i = 0; i < model_coefficients.size (); i++)
            {
                Eigen::Vector4f model = Eigen::Vector4f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2],
                                                         model_coefficients[i].values[3]);

                Eigen::Vector3f normal = Eigen::Vector3f (model_coefficients[i].values[0], model_coefficients[i].values[1], model_coefficients[i].values[2]);

                int inliers_count = plane_inliers_counts[i];

                std::cout << "Dot product is:" << normal.dot (normal_table) << std::endl;
                if ((normal.dot (normal_table) > 0.95) && (inliers_count_best * 0.5 <= inliers_count))
                {
                    //check if this plane is higher, projecting a point on the normal direction
                    std::cout << "Check if plane is higher, then change table plane" << std::endl;
                    std::cout << model[3] << " " << table_plane[3] << std::endl;
                    if (model[3] < table_plane[3])
                    {
                        PCL_WARN ("Changing table plane...");
                        table_plane_selected = i;
                        table_plane = model;
                        normal_table = normal;
                        inliers_count_best = inliers_count;
                    }
                }
            }

            /*table_plane = Eigen::Vector4f (model_coefficients[table_plane_selected].values[0], model_coefficients[table_plane_selected].values[1],
           model_coefficients[table_plane_selected].values[2], model_coefficients[table_plane_selected].values[3]);*/

            std::cout << "Table plane computed... " << std::endl;
      }
    }
}
