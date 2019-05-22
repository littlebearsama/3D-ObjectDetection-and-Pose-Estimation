/*
 * local_recognizer.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_LOCAL_RECOGNIZER_H_
#define FAAT_PCL_REC_FRAMEWORK_LOCAL_RECOGNIZER_H_

#include <flann/flann.h>
#include <pcl/common/common.h>
#include "source.h"
#include "local_estimator.h"
#include "faat_3d_rec_framework_defines.h"
#include <v4r/ORRecognition/correspondence_grouping.h>
#include <v4r/ORRecognition/hypotheses_verification.h>
#include "recognizer.h"

inline bool
correspSorter (const pcl::Correspondence & i, const pcl::Correspondence & j)
{
  return (i.distance < j.distance);
}

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    /**
     * \brief Object recognition + 6DOF pose based on local features, GC and HV
     * Contains keypoints/local features computation, matching using FLANN,
     * point-to-point correspondence grouping, pose refinement and hypotheses verification
     * Available features: SHOT, FPFH
     * See apps/3d_rec_framework/tools/apps for usage
     * \author Aitor Aldoma, Federico Tombari
     */

    template<template<class > class Distance, typename PointInT, typename FeatureT>
      class FAAT_3D_FRAMEWORK_API LocalRecognitionPipeline : public Recognizer<PointInT>
      {
        protected:
          typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
          typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;

          typedef Distance<float> DistT;
          typedef Model<PointInT> ModelT;
          typedef boost::shared_ptr<ModelT> ModelTPtr;
          /*struct ObjectHypothesis
          {
            ModelTPtr model_;
            typename pcl::PointCloud<PointInT>::Ptr correspondences_pointcloud; //points in model coordinates
            pcl::PointCloud<pcl::Normal>::Ptr normals_pointcloud; //points in model coordinates
            boost::shared_ptr<std::vector<float> > feature_distances_;
            pcl::CorrespondencesPtr correspondences_to_inputcloud; //indices between correspondences_pointcloud and scene cloud
            int num_corr_;
            std::vector<int> indices_to_flann_models_;
          };*/

          using Recognizer<PointInT>::input_;
          using Recognizer<PointInT>::models_;
          using Recognizer<PointInT>::transforms_;
          using Recognizer<PointInT>::ICP_iterations_;
          using Recognizer<PointInT>::icp_type_;
          using Recognizer<PointInT>::VOXEL_SIZE_ICP_;
          using Recognizer<PointInT>::indices_;
          using Recognizer<PointInT>::hv_algorithm_;
          using Recognizer<PointInT>::poseRefinement;
          using Recognizer<PointInT>::hypothesisVerification;

          class flann_model
          {
          public:
            ModelTPtr model;
            int view_id;
            int keypoint_id;
            std::vector<float> descr;
          };

          class codebook_model
          {
            public:
              int cluster_idx_;
              std::vector<float> descr;
              std::vector<int> clustered_indices_to_flann_models_;
          };

          /** \brief Directory where the trained structure will be saved */
          std::string training_dir_;

          /** \brief Point cloud to be classified */
          //PointInTPtr input_;

          /** \brief Model data source */
          typename boost::shared_ptr<Source<PointInT> > source_;

          /** \brief Computes a feature */
          typename boost::shared_ptr<LocalEstimator<PointInT, FeatureT> > estimator_;

          /** \brief Point-to-point correspondence grouping algorithm */
          typename boost::shared_ptr<faat_pcl::CorrespondenceGrouping<PointInT, PointInT> > cg_algorithm_;

          /** \brief Descriptor name */
          std::string descr_name_;

          /** \brief Id of the model to be used */
          std::string search_model_;

          flann::Matrix<float> flann_data_;
          flann::Index<DistT> * flann_index_;
          //flann::NNIndex<DistT> * flann_index_;

          std::map< std::pair< ModelTPtr, int >, std::vector<int> > model_view_id_to_flann_models_;
          std::vector<flann_model> flann_models_;
          std::vector<codebook_model> codebook_models_;

          bool use_cache_;
          std::map<std::pair<std::string, int>, Eigen::Matrix4f, std::less<std::pair<std::string, int> >, Eigen::aligned_allocator<std::pair<std::pair<
              std::string, int>, Eigen::Matrix4f> > > poses_cache_;
          std::map<std::pair<std::string, int>, typename pcl::PointCloud<PointInT>::Ptr> keypoints_cache_;
          std::map<std::pair<std::string, int>, pcl::PointCloud<pcl::Normal>::Ptr> normals_cache_;
          std::map<std::pair<std::string, int>, pcl::PointCloud<IndexPoint>::Ptr> idxpoint_cache_;

          float threshold_accept_model_hypothesis_;
          int kdtree_splits_;

          PointInTPtr keypoints_input_;
          PointInTPtr processed_;
          typename pcl::PointCloud<FeatureT>::Ptr signatures_;
          pcl::PointIndices keypoint_indices_;

          std::string cb_flann_index_fn_;
          std::string flann_index_fn_;
          std::string flann_data_fn_;
          bool use_codebook_;

          bool save_hypotheses_;
          typename std::map<std::string, ObjectHypothesis<PointInT> > saved_object_hypotheses_;
          PointInTPtr keypoint_cloud_;
          int knn_;
          float distance_same_keypoint_;
          float max_descriptor_distance_;
          float correspondence_distance_constant_weight_;

          //load features from disk and create flann structure
          void
          loadFeaturesAndCreateFLANN ();

          template <typename Type>
          inline void
          convertToFLANN (const std::vector<Type> &models, flann::Matrix<float> &data)
          {
            data.rows = models.size ();
            data.cols = models[0].descr.size (); // number of histogram bins

            flann::Matrix<float> flann_data (new float[models.size () * models[0].descr.size ()], models.size (), models[0].descr.size ());

            for (size_t i = 0; i < data.rows; ++i)
              for (size_t j = 0; j < data.cols; ++j)
              {
                flann_data.ptr ()[i * data.cols + j] = models[i].descr[j];
              }

            data = flann_data;
          }

          /*void
          nearestKSearch (flann::Index<DistT> * index, float * descr, int descr_size, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);*/

          void
          nearestKSearch (flann::Index<DistT> * index, flann::Matrix<float> & p, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

          void
          getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix);

          void
          getNormals (ModelT & model, int view_id, pcl::PointCloud<pcl::Normal>::Ptr & normals_cloud);

          void
          getIndicesToProcessedAndNormals (ModelT & model, int view_id, pcl::PointCloud<IndexPoint>::Ptr & index_cloud);

          void
          getKeypoints (ModelT & model, int view_id, typename pcl::PointCloud<PointInT>::Ptr & keypoints_cloud);

          void
          getView (ModelT & model, int view_id, PointInTPtr & view);

          void
          drawCorrespondences (PointInTPtr & cloud, ObjectHypothesis<PointInT> & oh, PointInTPtr & keypoints_pointcloud, pcl::Correspondences & correspondences)
          {
            pcl::visualization::PCLVisualizer vis_corresp_;
            vis_corresp_.setWindowName("correspondences...");
            pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler (cloud, 255, 0, 0);
            vis_corresp_.addPointCloud<PointInT> (cloud, random_handler, "points");

            typename pcl::PointCloud<PointInT>::ConstPtr cloud_sampled;
            cloud_sampled = oh.model_->getAssembled (0.0025f);

            pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler_sampled (cloud_sampled, 0, 0, 255);
            vis_corresp_.addPointCloud<PointInT> (cloud_sampled, random_handler_sampled, "sampled");

            for (size_t kk = 0; kk < correspondences.size (); kk++)
            {
              pcl::PointXYZ p;
              p.getVector4fMap () = oh.correspondences_pointcloud->points[correspondences[kk].index_query].getVector4fMap ();
              pcl::PointXYZ p_scene;
              p_scene.getVector4fMap () = keypoints_pointcloud->points[correspondences[kk].index_match].getVector4fMap ();

              std::stringstream line_name;
              line_name << "line_" << kk;

              vis_corresp_.addLine<pcl::PointXYZ, pcl::PointXYZ> (p_scene, p, line_name.str ());
            }

            vis_corresp_.spin ();
            vis_corresp_.removeAllPointClouds();
            vis_corresp_.removeAllShapes();
            vis_corresp_.close();
          }

          virtual void specificLoadFeaturesAndCreateFLANN()
          {
            std::cout << "specificLoadFeaturesAndCreateFLANN => this function does nothing..." << std::endl;
          }

          virtual void prepareSpecificCG(PointInTPtr & scene_cloud, PointInTPtr & scene_keypoints)
          {

          }

          virtual void specificCG(PointInTPtr & scene_cloud, PointInTPtr & scene_keypoints, ObjectHypothesis<PointInT> & oh)
          {
            //std::cout << "specificCG => this function does nothing..." << std::endl;
          }

          virtual void clearSpecificCG() {

          }

//          virtual void cgVerificationAndPoseEstimation(
//                    PointInTPtr keypoints_pointcloud,
//                    PointInTPtr processed,
//                    pcl::PointCloud<pcl::Normal>::Ptr scene_normal,
//                    std::map<std::string, ObjectHypothesis<PointInT> > &object_hypotheses);

      public:

        LocalRecognitionPipeline (std::string index_fn=std::string("index_flann.txt"),
                                     std::string cb_index_fn=std::string("index_codebook.txt")) : Recognizer<PointInT>()
        {
          use_cache_ = false;
          threshold_accept_model_hypothesis_ = 0.2f;
          kdtree_splits_ = 512;
          search_model_ = "";
          flann_index_fn_ = index_fn;
          cb_flann_index_fn_ = cb_index_fn;
          use_codebook_ = false;
          save_hypotheses_ = false;
          knn_ = 1;
          distance_same_keypoint_ = 0.001f * 0.001f;
          max_descriptor_distance_ = std::numeric_limits<float>::infinity();
          correspondence_distance_constant_weight_ = 1.f;
        }

        size_t getFeatureType() const
        {
            return estimator_->getFeatureType();
        }

        void setCorrespondenceDistanceConstantWeight(float w)
        {
            correspondence_distance_constant_weight_ = w;
        }

        void setMaxDescriptorDistance(float d)
        {
            max_descriptor_distance_ = d;
        }

        void setDistanceSameKeypoint(float d)
        {
            distance_same_keypoint_ = d*d;
        }

        ~LocalRecognitionPipeline ()
        {

        }

        void setKnn(int k)
        {
          knn_ = k;
        }

        void setSaveHypotheses(const bool set)
        {
          save_hypotheses_ = set;
        }

        virtual
        void
        getSavedHypotheses(std::map<std::string, ObjectHypothesis<PointInT> > & hypotheses) const
        {
          hypotheses = saved_object_hypotheses_;
        }

        void
        getKeypointCloud(PointInTPtr & keypoint_cloud) const
        {
          keypoint_cloud = keypoint_cloud_;
        }

        void getKeypointIndices(pcl::PointIndices & indices) const
        {
            indices.header = keypoint_indices_.header;
            indices.indices = keypoint_indices_.indices;
        }

        void setISPK(typename pcl::PointCloud<FeatureT>::Ptr & signatures, PointInTPtr & p, pcl::PointIndices & keypoint_indices)
        {  
          keypoint_indices_.header = keypoint_indices.header;
          keypoint_indices_.indices = keypoint_indices.indices;
          keypoints_input_.reset(new pcl::PointCloud<PointInT>());
          pcl::copyPointCloud(*p, keypoint_indices.indices, *keypoints_input_);
          //keypoints_input_ = keypoints;
          signatures_ = signatures;
          processed_ = p;

        }

        void setUseCodebook(bool t) {
          use_codebook_ = t;
        }

        void setIndexFN(std::string & in)
        {
          flann_index_fn_ = in;
        }

        void setCodebookFN(std::string & in)
        {
          cb_flann_index_fn_ = in;
        }

        void
        setSearchModel (std::string & id)
        {
          search_model_ = id;
        }

        void
        setThresholdAcceptHyp (float t)
        {
          threshold_accept_model_hypothesis_ = t;
        }

        void
        setKdtreeSplits (int n)
        {
          kdtree_splits_ = n;
        }

        void
        setIndices (std::vector<int> & indices)
        {
          indices_ = indices;
        }

        void
        setUseCache (bool u)
        {
          use_cache_ = u;
        }

        /**
         * \brief Sets the model data source_
         */
        void
        setDataSource (typename boost::shared_ptr<Source<PointInT> > & source)
        {
          source_ = source;
        }

        typename boost::shared_ptr<Source<PointInT> >
        getDataSource ()
        {
          return source_;
        }

        /**
         * \brief Sets the local feature estimator
         */
        void
        setFeatureEstimator (typename boost::shared_ptr<LocalEstimator<PointInT, FeatureT> > & feat)
        {
          estimator_ = feat;
        }

        /**
         * \brief Sets the CG algorithm
         */
        void
        setCGAlgorithm (typename boost::shared_ptr<faat_pcl::CorrespondenceGrouping<PointInT, PointInT> > & alg)
        {
          cg_algorithm_ = alg;
        }

        /**
         * \brief Sets the HV algorithm
         */
        /*void
        setHVAlgorithm (typename boost::shared_ptr<pcl::HypothesisVerification<PointInT, PointInT> > & alg)
        {
          hv_algorithm_ = alg;
        }*/

        /**
         * \brief Sets the input cloud to be classified
         */
        /*void
        setInputCloud (const PointInTPtr & cloud)
        {
          input_ = cloud;
        }*/

        /**
         * \brief Sets the descriptor name
         */
        void
        setDescriptorName (std::string & name)
        {
          descr_name_ = name;
        }

        /**
         * \brief Filesystem dir where to keep the generated training data
         */
        void
        setTrainingDir (const std::string dir)
        {
          training_dir_ = dir;
        }

        void
        getProcessed(PointInTPtr & cloud) {
          cloud = processed_;
        }

        /**
         * \brief Initializes the FLANN structure from the provided source
         * It does training for the models that havent been trained yet
         */

        void
        initialize (bool force_retrain = false);

        /**
         * \brief Performs recognition and pose estimation on the input cloud
         */

        void
        recognize ();
      };
  }
}

#endif /* REC_FRAMEWORK_LOCAL_RECOGNIZER_H_ */
