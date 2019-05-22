/*
 * recognizer.h
 *
 *  Created on: Feb 24, 2013
 *      Author: aitor
 */

#ifndef RECOGNIZER_H_
#define RECOGNIZER_H_

#include "faat_3d_rec_framework_defines.h"
#include <pcl/common/common.h>
#include "voxel_based_correspondence_estimation.h"
#include "source.h"
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include "v4r/ORRecognition/hypotheses_verification.h"
#include <pcl/common/time.h>
#include <pcl/filters/crop_box.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {

    template<typename PointInT>
    inline void
    getIndicesFromCloud(typename pcl::PointCloud<PointInT>::Ptr & processed,
                          typename pcl::PointCloud<PointInT>::Ptr & keypoints_pointcloud,
                          std::vector<int> & indices)
    {
      pcl::octree::OctreePointCloudSearch<PointInT> octree (0.005);
      octree.setInputCloud (processed);
      octree.addPointsFromInputCloud ();

      std::vector<int> pointIdxNKNSearch;
      std::vector<float> pointNKNSquaredDistance;

      for(size_t j=0; j < keypoints_pointcloud->points.size(); j++)
      {
       if (octree.nearestKSearch (keypoints_pointcloud->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
       {
         indices.push_back(pointIdxNKNSearch[0]);
       }
      }
    }

    template<typename PointInT>
    class ObjectHypothesis
    {
      typedef Model<PointInT> ModelT;
      typedef boost::shared_ptr<ModelT> ModelTPtr;

      public:
        ModelTPtr model_;
        typename pcl::PointCloud<PointInT>::Ptr correspondences_pointcloud; //points in model coordinates
        pcl::PointCloud<pcl::Normal>::Ptr normals_pointcloud; //points in model coordinates
//        boost::shared_ptr<std::vector<float> > feature_distances_;
        pcl::CorrespondencesPtr correspondences_to_inputcloud; //indices between correspondences_pointcloud and scene cloud
//        int num_corr_;
        std::vector<int> indices_to_flann_models_;
    };

    template<typename PointInT>
    class FAAT_3D_FRAMEWORK_API Recognizer
    {
      typedef Model<PointInT> ModelT;
      typedef boost::shared_ptr<ModelT> ModelTPtr;

      typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
      typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;

      protected:
        /** \brief Point cloud to be classified */
        PointInTPtr input_;

        boost::shared_ptr<std::vector<ModelTPtr> > models_;
        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_;

        //before HV
        boost::shared_ptr<std::vector<ModelTPtr> > models_before_hv_;
        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_before_hv_;

        int ICP_iterations_;
        int icp_type_;
        float VOXEL_SIZE_ICP_;
        float max_corr_distance_;
        bool requires_segmentation_;
        std::vector<int> indices_;
        bool recompute_hv_normals_;
        pcl::PointIndicesPtr icp_scene_indices_;

        /** \brief Hypotheses verification algorithm */
        typename boost::shared_ptr<faat_pcl::HypothesisVerification<PointInT, PointInT> > hv_algorithm_;

        void poseRefinement()
        {
          pcl::ScopeTime ticp ("ICP ");
          PointInTPtr cloud_voxelized_icp (new pcl::PointCloud<PointInT> ());
          pcl::VoxelGrid<PointInT> voxel_grid_icp;
          voxel_grid_icp.setInputCloud (input_);
          if(icp_scene_indices_ && icp_scene_indices_->indices.size() > 0)
          {
            voxel_grid_icp.setIndices(icp_scene_indices_);
          }
          voxel_grid_icp.setLeafSize (VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_, VOXEL_SIZE_ICP_);
          voxel_grid_icp.filter (*cloud_voxelized_icp);
          std::cout << "Number of hypotheses to ICP:" << models_->size () << std::endl;
          switch (icp_type_)
          {
            case 0:
            {
    #pragma omp parallel for schedule(dynamic,1) num_threads(omp_get_num_procs())
              for (int i = 0; i < static_cast<int> (models_->size ()); i++)
              {

                ConstPointInTPtr model_cloud;
                PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
                model_cloud = models_->at (i)->getAssembled (VOXEL_SIZE_ICP_);
                pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));

                typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT>::Ptr
                                        rej (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT> ());

                rej->setInputTarget (cloud_voxelized_icp);
                rej->setMaximumIterations (1000);
                rej->setInlierThreshold (0.005f);
                rej->setInputSource (model_aligned);

                pcl::IterativeClosestPoint<PointInT, PointInT> reg;
                reg.addCorrespondenceRejector (rej);
                reg.setInputTarget (cloud_voxelized_icp); //scene
                reg.setInputSource (model_aligned); //model
                reg.setMaximumIterations (ICP_iterations_);
                reg.setMaxCorrespondenceDistance (max_corr_distance_);

                typename pcl::PointCloud<PointInT>::Ptr output_ (new pcl::PointCloud<PointInT> ());
                reg.align (*output_);

                Eigen::Matrix4f icp_trans = reg.getFinalTransformation ();
                transforms_->at (i) = icp_trans * transforms_->at (i);
              }
            }
              break;
            default:
            {

    #pragma omp parallel for schedule(dynamic,1) num_threads(omp_get_num_procs())
              for (int i = 0; i < static_cast<int> (models_->size ()); i++)
              {

                typename VoxelBasedCorrespondenceEstimation<PointInT, PointInT>::Ptr
                            est (new VoxelBasedCorrespondenceEstimation<PointInT, PointInT> ());

                typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT>::Ptr
                            rej (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT> ());

                Eigen::Matrix4f scene_to_model_trans = transforms_->at (i).inverse ();
                //boost::shared_ptr<VoxelGridDistanceTransform<PointInT> > dt;
                boost::shared_ptr<distance_field::PropagationDistanceField<PointInT> > dt;
                models_->at (i)->getVGDT (dt);

                PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
                typename pcl::PointCloud<PointInT>::ConstPtr cloud;
                dt->getInputCloud(cloud);
                model_aligned.reset(new pcl::PointCloud<PointInT>(*cloud));

                PointInTPtr cloud_voxelized_icp_transformed (new pcl::PointCloud<PointInT> ());
                pcl::transformPointCloud (*cloud_voxelized_icp, *cloud_voxelized_icp_transformed, scene_to_model_trans);

                PointInT minPoint, maxPoint;
                pcl::getMinMax3D(*cloud, minPoint, maxPoint);
                minPoint.x -= max_corr_distance_;
                minPoint.y -= max_corr_distance_;
                minPoint.z -= max_corr_distance_;

                maxPoint.x += max_corr_distance_;
                maxPoint.y += max_corr_distance_;
                maxPoint.z += max_corr_distance_;

                pcl::CropBox<PointInT> cropFilter;
                cropFilter.setInputCloud (cloud_voxelized_icp_transformed);
                cropFilter.setMin(minPoint.getVector4fMap());
                cropFilter.setMax(maxPoint.getVector4fMap());

                PointInTPtr cloud_voxelized_icp_cropped (new pcl::PointCloud<PointInT> ());
                cropFilter.filter (*cloud_voxelized_icp_cropped);

                cloud_voxelized_icp_transformed = cloud_voxelized_icp_cropped;
                //PointInTPtr cloud_voxelized_icp_transformed (new pcl::PointCloud<PointInT> ());
                //pcl::transformPointCloud (*cloud_voxelized_icp, *cloud_voxelized_icp_transformed, scene_to_model_trans);

                est->setVoxelRepresentationTarget (dt);
                est->setInputSource (cloud_voxelized_icp_transformed);
                est->setInputTarget (model_aligned);
                est->setMaxCorrespondenceDistance (max_corr_distance_);

                rej->setInputTarget (model_aligned);
                rej->setMaximumIterations (1000);
                rej->setInlierThreshold (0.005f);
                rej->setInputSource (cloud_voxelized_icp_transformed);

                pcl::IterativeClosestPoint<PointInT, PointInT, float> reg;
                reg.setCorrespondenceEstimation (est);
                reg.addCorrespondenceRejector (rej);
                reg.setInputTarget (model_aligned); //model
                reg.setInputSource (cloud_voxelized_icp_transformed); //scene
                reg.setMaximumIterations (ICP_iterations_);
                reg.setEuclideanFitnessEpsilon(1e-5);
                reg.setTransformationEpsilon(0.001f * 0.001f);

                pcl::registration::DefaultConvergenceCriteria<float>::Ptr convergence_criteria;
                convergence_criteria = reg.getConvergeCriteria();
                convergence_criteria->setAbsoluteMSE(1e-12);
                convergence_criteria->setMaximumIterationsSimilarTransforms(15);
                convergence_criteria->setFailureAfterMaximumIterations(false);

                typename pcl::PointCloud<PointInT>::Ptr output_ (new pcl::PointCloud<PointInT> ());
                reg.align (*output_);

                Eigen::Matrix4f icp_trans;
                icp_trans = reg.getFinalTransformation () * scene_to_model_trans;
                transforms_->at (i) = icp_trans.inverse ();

                /*pcl::registration::DefaultConvergenceCriteria<float>::ConvergenceState conv_state;
                conv_state = convergence_criteria->getConvergenceState();

                if(conv_state != pcl::registration::DefaultConvergenceCriteria<float>::CONVERGENCE_CRITERIA_ITERATIONS
                    && conv_state != pcl::registration::DefaultConvergenceCriteria<float>::CONVERGENCE_CRITERIA_NOT_CONVERGED
                    && conv_state != pcl::registration::DefaultConvergenceCriteria<float>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES) {
                  extra_weights_[i] = 1.f;
                } else {
                  extra_weights_[i] = 0.f;
                }
                std::cout << "Converged:" << reg.hasConverged() << " state:" << conv_state << std::endl;*/
              }
            }
          }
        }

        void
        hypothesisVerification ()
        {
          models_before_hv_ = models_;
          transforms_before_hv_ = transforms_;

          pcl::ScopeTime thv ("HV verification");

          std::vector<typename pcl::PointCloud<PointInT>::ConstPtr> aligned_models;
          std::vector<pcl::PointCloud<pcl::Normal>::ConstPtr> aligned_normals;
          aligned_models.resize (models_->size ());
          aligned_normals.resize (models_->size ());

#pragma omp parallel for schedule(dynamic,1) num_threads(omp_get_num_procs())
          for (size_t i = 0; i < models_->size (); i++)
          {
            //we should get the resolution of the hv_algorithm here... then we can avoid to voxel grid again when computing the cues...
            //ConstPointInTPtr model_cloud = models_->at (i)->getAssembled (0.005f);
            ConstPointInTPtr model_cloud = models_->at (i)->getAssembled (VOXEL_SIZE_ICP_);

            PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));
            aligned_models[i] = model_aligned;

            if (hv_algorithm_->getRequiresNormals () && !recompute_hv_normals_)
            {
              pcl::PointCloud<pcl::Normal>::ConstPtr normals_cloud = models_->at (i)->getNormalsAssembled (VOXEL_SIZE_ICP_);
              pcl::PointCloud<pcl::Normal>::Ptr normals_aligned (new pcl::PointCloud<pcl::Normal>);
              normals_aligned->points.resize (normals_cloud->points.size ());
              normals_aligned->width = normals_cloud->width;
              normals_aligned->height = normals_cloud->height;
              for (size_t k = 0; k < normals_cloud->points.size (); k++)
              {
                Eigen::Vector3f nt (normals_cloud->points[k].normal_x, normals_cloud->points[k].normal_y, normals_cloud->points[k].normal_z);
                normals_aligned->points[k].normal_x = static_cast<float> (transforms_->at (i) (0, 0) * nt[0] + transforms_->at (i) (0, 1) * nt[1]
                    + transforms_->at (i) (0, 2) * nt[2]);
                normals_aligned->points[k].normal_y = static_cast<float> (transforms_->at (i) (1, 0) * nt[0] + transforms_->at (i) (1, 1) * nt[1]
                    + transforms_->at (i) (1, 2) * nt[2]);
                normals_aligned->points[k].normal_z = static_cast<float> (transforms_->at (i) (2, 0) * nt[0] + transforms_->at (i) (2, 1) * nt[1]
                    + transforms_->at (i) (2, 2) * nt[2]);

                //flip here based on vp?
                pcl::flipNormalTowardsViewpoint (model_aligned->points[k], 0, 0, 0, normals_aligned->points[k].normal[0],
                                                 normals_aligned->points[k].normal[1], normals_aligned->points[k].normal[2]);
              }

              aligned_normals[i] = normals_aligned;
            }
          }

          std::vector<bool> mask_hv;
          hv_algorithm_->setSceneCloud (input_);
          if (hv_algorithm_->getRequiresNormals () && !recompute_hv_normals_)
          {
            hv_algorithm_->addNormalsClouds (aligned_normals);
          }

          hv_algorithm_->addModels (aligned_models, true);
          hv_algorithm_->verify ();
          hv_algorithm_->getMask (mask_hv);

          boost::shared_ptr<std::vector<ModelTPtr> > models_temp;
          boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_temp;

          models_temp.reset (new std::vector<ModelTPtr>);
          transforms_temp.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

          for (size_t i = 0; i < models_->size (); i++)
          {
            if (!mask_hv[i])
              continue;

            models_temp->push_back (models_->at (i));
            transforms_temp->push_back (transforms_->at (i));
          }

          models_ = models_temp;
          transforms_ = transforms_temp;

        }

      public:

        Recognizer()
        {
          ICP_iterations_ = 30;
          VOXEL_SIZE_ICP_ = 0.0025f;
          icp_type_ = 1;
          max_corr_distance_ = 0.02f;
          requires_segmentation_ = false;
          recompute_hv_normals_ = true;
        }

        virtual size_t getFeatureType() const
        {
            std::cout << "Get feature type is not implemented for this recognizer. " << std::endl;
            return 0;
        }

        /*virtual void setISPK(typename pcl::PointCloud<FeatureT>::Ptr & signatures, PointInTPtr & p, PointInTPtr & keypoints)
        {
          std::cerr << "Set ISPK is not implemented for this type of feature estimator. " << std::endl;
        }*/

        virtual bool acceptsNormals() const
        {
            return false;
        }

        virtual void setSceneNormals(pcl::PointCloud<pcl::Normal>::Ptr & /*normals*/)
        {
            PCL_WARN("Set scene normals is not implemented for this class.");
        }

        virtual void
        setSaveHypotheses(const bool b)
        {
            PCL_WARN("Set save hypotheses is not implemented for this class.");
        }

        virtual
        void
        getSavedHypotheses(std::map<std::string, ObjectHypothesis<PointInT> > & hypotheses) const
        {
            PCL_WARN("Get saved hypotheses is not implemented for this class.");
        }

        virtual
        void
        getKeypointCloud(PointInTPtr & keypoint_cloud) const
        {
            PCL_WARN("Get keypoint cloud is not implemented for this class.");
        }

        virtual
        void
        getKeypointIndices(pcl::PointIndices & indices) const
        {
            PCL_WARN("Get keypoint indices is not implemented for this class.");
        }

        virtual void recognize () = 0;

        virtual typename boost::shared_ptr<Source<PointInT> >
        getDataSource () = 0;

        /*virtual void
        setHVAlgorithm (typename boost::shared_ptr<pcl::HypothesisVerification<PointInT, PointInT> > & alg) = 0;*/

        void
        setHVAlgorithm (const typename boost::shared_ptr<const faat_pcl::HypothesisVerification<PointInT, PointInT> > & alg)
        {
          hv_algorithm_ = alg;
        }

        void
        setInputCloud (PointInTPtr & cloud)
        {
          input_ = cloud;
        }

        boost::shared_ptr<std::vector<ModelTPtr> >
        getModels () const
        {
          return models_;
        }

        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > >
        getTransforms () const
        {
          return transforms_;
        }

        boost::shared_ptr<std::vector<ModelTPtr> >
        getModelsBeforeHV () const
        {
          return models_before_hv_;
        }

        boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > >
        getTransformsBeforeHV () const
        {
          return transforms_before_hv_;
        }

        void
        setICPIterations (const int it)
        {
          ICP_iterations_ = it;
        }

        void setICPType(const int t) {
          icp_type_ = t;
        }

        void setVoxelSizeICP(const float s) {
          VOXEL_SIZE_ICP_ = s;
        }

        virtual bool requiresSegmentation() const
        {
          return requires_segmentation_;
        }

        virtual void
        setIndices (const std::vector<int> & indices) {
          indices_ = indices;
        }
    };
  }
}
#endif /* RECOGNIZER_H_ */
