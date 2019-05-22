/*
 * vfh_estimator.h
 *
 *  Created on: Mar 22, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_ORGANIZED_COLOR_OURCVFH_ESTIMATOR_H_
#define REC_FRAMEWORK_ORGANIZED_COLOR_OURCVFH_ESTIMATOR_H_

#include "faat_3d_rec_framework_defines.h"
#include "ourcvfh_estimator.h"
#include "organized_ourcvfh.h"
#include "normal_estimator.h"
#include <pcl/features/our_cvfh.h>
#include <pcl/surface/mls.h>

namespace faat_pcl
{
    namespace rec_3d_framework
    {

        template<typename PointInT = pcl::PointXYZRGB, typename FeatureT = pcl::Histogram<367> >
        class FAAT_3D_FRAMEWORK_API OrganizedColorOURCVFHEstimator : public OURCVFHEstimator<PointInT, FeatureT>
        {

            typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
            using OURCVFHEstimator<PointInT, FeatureT>::normal_estimator_;
            using OURCVFHEstimator<PointInT, FeatureT>::normals_;
            using OURCVFHEstimator<PointInT, FeatureT>::valid_roll_transforms_;
            using OURCVFHEstimator<PointInT, FeatureT>::cluster_indices_;
            using OURCVFHEstimator<PointInT, FeatureT>::transforms_;
            using OURCVFHEstimator<PointInT, FeatureT>::eps_angle_threshold_;
            using OURCVFHEstimator<PointInT, FeatureT>::curvature_threshold_;
            using OURCVFHEstimator<PointInT, FeatureT>::cluster_tolerance_factor_;
            using OURCVFHEstimator<PointInT, FeatureT>::normalize_bins_;
            using OURCVFHEstimator<PointInT, FeatureT>::adaptative_MLS_;
            using OURCVFHEstimator<PointInT, FeatureT>::refine_factor_;
            using OURCVFHEstimator<PointInT, FeatureT>::eps_angle_threshold_vector_;
            using OURCVFHEstimator<PointInT, FeatureT>::curvature_threshold_vector_;
            using OURCVFHEstimator<PointInT, FeatureT>::cluster_tolerance_vector_;
            using OURCVFHEstimator<PointInT, FeatureT>::organized_data_;
            bool use_RF_;
            float axis_ratio_;
            float min_axis_value_;
            bool normals_set_;
            float normal_radius_;
            float resolution_;
            pcl::PointIndices indices_;

        public:

            OrganizedColorOURCVFHEstimator ()
            {
                eps_angle_threshold_ = 0.13f;
                curvature_threshold_ = 0.035f;
                normalize_bins_ = true;
                cluster_tolerance_factor_ = 3.f;
                use_RF_ = false;
                min_axis_value_ = 0.925f;
                axis_ratio_ = 0.8f;
                adaptative_MLS_ = false;
                normals_set_ = false;
                normal_radius_ = 0.02f;
                resolution_ = 0.003f;
                organized_data_ = true;
            }

            void setAxisRatio(float f) {
                axis_ratio_ = f;
            }

            void setMinAxisValue(float f) {
                min_axis_value_ = f;
            }

            void
            setUseRFForColor (bool b)
            {
                use_RF_ = b;
            }

            void
            setNormals(pcl::PointCloud<pcl::Normal>::Ptr & normals)
            {
                normals_ = normals;
                normals_set_ = true;
                std::cout << "organized color ourcvfh:" << normals_set_ << " " << normals->points.size() << std::endl;
            }

            void setInternalNormalRadiusAndResolution(float r, float res)
            {
                normal_radius_ = r;
                resolution_ = res;
            }

            bool getUsesOrganizedData()
            {
                return organized_data_;
            }

            bool acceptsIndices()
            {
                return true;
            }

            void
            setIndices (pcl::PointIndices & p_indices)
            {
              indices_ = p_indices;
            }

            void
            setIndices(std::vector<int> & p_indices)
            {
              indices_.indices = p_indices;
            }

            bool
            estimate (const PointInTPtr & in, PointInTPtr & processed, typename pcl::PointCloud<FeatureT>::CloudVectorType & signatures,
                      std::vector<Eigen::Vector3f> & centroids)
            {
                pcl::ScopeTime time_estimate("Organized OUR-CVFH color estimate");
                valid_roll_transforms_.clear ();
                transforms_.clear ();
                cluster_indices_.clear ();

                if(!in->isOrganized())
                {
                    PCL_WARN("OrganizedColorOURCVFHEstimator: input cloud not organized?!?");
                }

                if(indices_.indices.size() > 0)
                {
                    //set all points not belonging to indices_ to NaN
                    std::vector<bool> set_to_nan(in->points.size(), true);
                    for(size_t i=0; i < indices_.indices.size(); i++)
                    {
                        set_to_nan[indices_.indices[i]] = false;
                    }

                    for(size_t i=0; i < in->points.size(); i++)
                    {
                        if(set_to_nan[i])
                        {
                            in->points[i].x = in->points[i].y = in->points[i].z = std::numeric_limits<float>::quiet_NaN();
                        }
                    }

                    std::cout << "indices size:" << indices_.indices.size() << std::endl;
                }

                if(!normals_set_)
                {
                    pcl::ScopeTime t("normals were not set, computing...");
                    if (!normal_estimator_)
                    {
                        PCL_ERROR("This feature needs normals... please provide a normal estimator\n");
                        return false;
                    }

                    pcl::MovingLeastSquares<PointInT, PointInT> mls;
                    if (adaptative_MLS_)
                    {
                        typename pcl::search::KdTree<PointInT>::Ptr tree;
                        Eigen::Vector4f centroid_cluster;
                        pcl::compute3DCentroid (*in, centroid_cluster);
                        float dist_to_sensor = centroid_cluster.norm ();
                        float sigma = dist_to_sensor * 0.01f;
                        mls.setSearchMethod (tree);
                        mls.setSearchRadius (sigma);
                        mls.setUpsamplingMethod (mls.SAMPLE_LOCAL_PLANE);
                        mls.setUpsamplingRadius (0.002);
                        mls.setUpsamplingStepSize (0.001);
                    }

                    normals_.reset (new pcl::PointCloud<pcl::Normal>);
                    {
                        pcl::ScopeTime time_estimate("Organized color OUR-CVFH normals");
                        normal_estimator_->estimate_organized(in, processed, normals_);
                    }

                    if (adaptative_MLS_)
                    {
                        mls.setInputCloud (processed);

                        PointInTPtr filtered (new pcl::PointCloud<PointInT>);
                        mls.process (*filtered);

                        processed.reset (new pcl::PointCloud<PointInT>);
                        normals_.reset (new pcl::PointCloud<pcl::Normal>);
                        {
                            filtered->is_dense = false;
                            normal_estimator_->estimate (filtered, processed, normals_);
                        }
                    }
                }
                else
                {
                    processed.reset (new pcl::PointCloud<PointInT>(*in));
                }

                typedef typename faat_pcl::rec_3d_framework::OURCVFHEstimation<PointInT, pcl::Normal, pcl::VFHSignature308> OURCVFHEstimation;
                typename pcl::search::KdTree<PointInT>::Ptr cvfh_tree (new pcl::search::KdTree<PointInT>);

                //if vectors are not set, assume that single values are set...
                if (eps_angle_threshold_vector_.size () == 0)
                    eps_angle_threshold_vector_.push_back (eps_angle_threshold_);

                if (curvature_threshold_vector_.size () == 0)
                    curvature_threshold_vector_.push_back (curvature_threshold_);

                if (cluster_tolerance_vector_.size () == 0)
                    cluster_tolerance_vector_.push_back (cluster_tolerance_factor_);

                //std::cout << eps_angle_threshold_vector_.size() << " " << curvature_threshold_vector_.size() << std::endl;

                //double time_final_copying = 0.f;
                //double time_getting_copying = 0.f;

                OURCVFHEstimation cvfh;
                cvfh.setSearchMethod (cvfh_tree);
                cvfh.setInputCloud (processed);
                cvfh.setInputNormals (normals_);
                cvfh.setNormalizeBins (normalize_bins_);
                cvfh.setRefineClusters (refine_factor_);
                cvfh.setAxisRatio (axis_ratio_);
                cvfh.setMinAxisValue (min_axis_value_);
                cvfh.setMinPoints (50);

                for (size_t ei = 0; ei < eps_angle_threshold_vector_.size (); ei++)
                {
                    for (size_t ci = 0; ci < curvature_threshold_vector_.size (); ci++)
                    {
                        for (size_t ti = 0; ti < cluster_tolerance_vector_.size (); ti++)
                        {
                            pcl::ScopeTime t ("***************************************** Computing OURCVFH + color histogram...");

                            cvfh.setEPSAngleThreshold (eps_angle_threshold_vector_[ei]);
                            cvfh.setCurvatureThreshold (curvature_threshold_vector_[ci]);

                            float radius, cluster_tolerance_radius;

                            if (!normals_set_ && (normal_estimator_->compute_mesh_resolution_))
                            {
                                radius = normal_estimator_->mesh_resolution_ * normal_estimator_->factor_normals_;
                                cluster_tolerance_radius = normal_estimator_->mesh_resolution_ * cluster_tolerance_vector_[ti];

                                if (normal_estimator_->do_voxel_grid_)
                                {
                                    radius *= normal_estimator_->factor_voxel_grid_;
                                    cluster_tolerance_radius *= normal_estimator_->factor_voxel_grid_;
                                }
                            }
                            else
                            {
                                if(normals_set_)
                                {
                                    radius = normal_radius_;
                                    cluster_tolerance_radius = resolution_ * cluster_tolerance_vector_[ti];
                                }
                                else
                                {
                                    radius = normal_estimator_->normal_radius_;
                                    cluster_tolerance_radius = normal_estimator_->grid_resolution_ * cluster_tolerance_vector_[ti];
                                }
                            }

                            //std::cout << cluster_tolerance_radius << " " << radius << std::endl;

                            cvfh.setClusterTolerance (cluster_tolerance_radius);
                            cvfh.setRadiusNormals (radius);

                            pcl::PointCloud<pcl::VFHSignature308> cvfh_signatures;
                            std::vector<Eigen::Vector3f> centroids_in;
                            std::vector<bool> valid_roll_transforms_in;
                            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_in;
                            std::vector<pcl::PointIndices> indices_cluster_used;

                            {
                                pcl::ScopeTime t ("***************************************** Computing OURCVFH...");
                                cvfh.compute (cvfh_signatures);
                            }

                            size_t valid = 0;

                            //{
                            //    pcl::StopWatch t; //("getting and copying...");
                            cvfh.getCentroidClusters (centroids_in);
                            cvfh.getTransforms (transforms_in);
                            cvfh.getValidTransformsVec (valid_roll_transforms_in);
                            cvfh.getClusterIndices(indices_cluster_used);

                            for (size_t i = 0; i < valid_roll_transforms_in.size (); i++)
                            {
                                if (valid_roll_transforms_in[i])
                                {
                                    transforms_in[valid] = transforms_in[i];
                                    centroids_in[valid] = centroids_in[i];
                                    valid_roll_transforms_in[valid] = valid_roll_transforms_in[i];
                                    cvfh_signatures.points[valid] = cvfh_signatures.points[i];
                                    indices_cluster_used[valid] = indices_cluster_used[i];
                                    valid++;
                                }
                            }

                            valid_roll_transforms_in.resize (valid);
                            centroids_in.resize (valid);
                            transforms_in.resize (valid);
                            cvfh_signatures.points.resize (valid);
                            indices_cluster_used.resize(valid);

                            for(size_t kk=0; kk < centroids_in.size(); kk++) {
                                centroids.push_back(centroids_in[kk]);
                                transforms_.push_back(transforms_in[kk]);
                                valid_roll_transforms_.push_back(valid_roll_transforms_in[kk]);
                                cluster_indices_.push_back(indices_cluster_used[kk]);
                            }

                                //time_getting_copying += t.getTime();
                            //}

                            //compute color histogram
                            //1) transform rgb field to YUV
                            std::vector<float> y, u, v;
                            y.resize (processed->points.size ());
                            u.resize (processed->points.size ());
                            v.resize (processed->points.size ());

                            int size_uv = 8;
                            int size_y = 2;
                            float weight_factor = 1.f;
                            int not_nans = 0;

                            for (size_t i = 0; i < processed->points.size (); i++)
                            {
                                Eigen::Vector4f p = processed->points[i].getVector4fMap ();
                                if(pcl_isnan(p[2]))
                                    continue;

                                int r, g, b;
                                r = static_cast<int> (processed->points[i].r);
                                g = static_cast<int> (processed->points[i].g);
                                b = static_cast<int> (processed->points[i].b);

                                y[i] = 0.257f * r + 0.504f * g + 0.098f * b + 16;
                                u[i] = -(0.148 * r) - (0.291 * g) + (0.439 * b) + 128;
                                v[i] = (0.439 * r) - (0.368 * g) - (0.071 * b) + 128;
                                not_nans++;
                            }

                            if (normalize_bins_)
                                weight_factor = 100.f / not_nans * 8;

                            //using the rf
                            int size_hists = size_uv * size_uv * size_y;
                            int num_hists = 8;
                            std::vector < Eigen::VectorXf > octants (8);

                            float * weights = new float[num_hists];
                            float sigma = 0.005f; //1cm
                            float sigma_sq = 2.f * sigma * sigma;

                            for (size_t t = 0; t < cvfh_signatures.points.size (); t++)
                            {
                                //pcl::ScopeTime t_sig("computing color histogram for one signature");
                                for (int k = 0; k < num_hists; k++)
                                    octants[k].setZero (size_hists);

                                for (int k = 0; k < static_cast<int> (processed->points.size ()); k++)
                                {
                                    Eigen::Vector4f p = processed->points[k].getVector4fMap ();
                                    if(pcl_isnan(p[2]))
                                        continue;

                                    p = transforms_[t] * p;

                                    //compute weight for all octants
                                    float wx = 1.f - std::exp (-((p[0] * p[0]) / (sigma_sq))); //how is the weight distributed among two semi-cubes
                                    float wy = 1.f - std::exp (-((p[1] * p[1]) / (sigma_sq)));
                                    float wz = 1.f - std::exp (-((p[2] * p[2]) / (sigma_sq)));

                                    //distribute the weights using the x-coordinate
                                    if (p[0] >= 0)
                                    {
                                        for (size_t ii = 0; ii <= 3; ii++)
                                            weights[ii] = 0.5f - wx * 0.5f;

                                        for (size_t ii = 4; ii <= 7; ii++)
                                            weights[ii] = 0.5f + wx * 0.5f;
                                    }
                                    else
                                    {
                                        for (size_t ii = 0; ii <= 3; ii++)
                                            weights[ii] = 0.5f + wx * 0.5f;

                                        for (size_t ii = 4; ii <= 7; ii++)
                                            weights[ii] = 0.5f - wx * 0.5f;
                                    }

                                    //distribute the weights using the y-coordinate
                                    if (p[1] >= 0)
                                    {
                                        for (size_t ii = 0; ii <= 1; ii++)
                                            weights[ii] *= 0.5f - wy * 0.5f;
                                        for (size_t ii = 4; ii <= 5; ii++)
                                            weights[ii] *= 0.5f - wy * 0.5f;

                                        for (size_t ii = 2; ii <= 3; ii++)
                                            weights[ii] *= 0.5f + wy * 0.5f;

                                        for (size_t ii = 6; ii <= 7; ii++)
                                            weights[ii] *= 0.5f + wy * 0.5f;
                                    }
                                    else
                                    {
                                        for (size_t ii = 0; ii <= 1; ii++)
                                            weights[ii] *= 0.5f + wy * 0.5f;
                                        for (size_t ii = 4; ii <= 5; ii++)
                                            weights[ii] *= 0.5f + wy * 0.5f;

                                        for (size_t ii = 2; ii <= 3; ii++)
                                            weights[ii] *= 0.5f - wy * 0.5f;

                                        for (size_t ii = 6; ii <= 7; ii++)
                                            weights[ii] *= 0.5f - wy * 0.5f;
                                    }

                                    //distribute the weights using the z-coordinate
                                    if (p[2] >= 0)
                                    {
                                        for (size_t ii = 0; ii <= 7; ii += 2)
                                            weights[ii] *= 0.5f - wz * 0.5f;

                                        for (size_t ii = 1; ii <= 7; ii += 2)
                                            weights[ii] *= 0.5f + wz * 0.5f;

                                    }
                                    else
                                    {
                                        for (size_t ii = 0; ii <= 7; ii += 2)
                                            weights[ii] *= 0.5f + wz * 0.5f;

                                        for (size_t ii = 1; ii <= 7; ii += 2)
                                            weights[ii] *= 0.5f - wz * 0.5f;
                                    }

                                    //fill yuv histogram and put it in octants vector
                                    int uu = std::floor (u[k] / 255.f * size_uv);
                                    int vv = std::floor (v[k] / 255.f * size_uv);
                                    int yy = std::floor ((y[k] - 16.f) / 219.f * size_y);
                                    if(yy >= size_y)
                                        yy = size_y - 1;

                                    if(uu >= size_uv)
                                        uu = size_uv - 1;

                                    if(vv >= size_uv)
                                        vv = size_uv - 1;

                                    /*if(!(yy < size_y && uu < size_uv && vv < size_uv))
                                    {
                                        std::cout << yy << " " << uu << " " << vv << std::endl;
                                        std::cout << size_y << " " << size_uv << " " << size_uv << std::endl;
                                        std::cout << (y[k] - 16.f) / 219.f * size_y << std::endl;
                                    }*/

                                    assert (yy < size_y && uu < size_uv && vv < size_uv);

                                    int h_index = yy * size_uv * size_uv + uu * size_uv + vv;
                                    for (int j = 0; j < num_hists; j++)
                                        octants[j][h_index] += weight_factor * weights[j];

                                }

                                //{
                                //pcl::StopWatch tt;
                                pcl::PointCloud<FeatureT> vfh_signature;
                                vfh_signature.points.resize (1);
                                vfh_signature.width = vfh_signature.height = 1;
                                for (int d = 0; d < 303; ++d)
                                    vfh_signature.points[0].histogram[d] = cvfh_signatures.points[t].histogram[d];

                                int pos = 303;
                                for (int k = 0; k < num_hists; k++)
                                {
                                    for (int ii = 0; ii < size_hists; ii++, pos++)
                                    {
                                        vfh_signature.points[0].histogram[pos] = octants[k][ii];
                                    }
                                }
                                signatures.push_back (vfh_signature);
                                    //time_final_copying += tt.getTime();
                                //}
                            }

                            delete[] weights;
                        }
                    }
                }

                //std::cout << "time final copying:" << time_final_copying << std::endl;
                //std::cout << "time getting copying:" << time_getting_copying << std::endl;
                std::cout << "Signatures.size()" << signatures.size() << std::endl;

                indices_.indices.clear ();
                return true;
            }

            bool
            computedNormals ()
            {
                return true;
            }
        };
    }
}

#endif /* REC_FRAMEWORK_COLOR_OURCVFH_ESTIMATOR_H_ */
