/*
 * global.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef REC_FRAMEWORK_GLOBAL_RECOGNIZER_CVFH_H_
#define REC_FRAMEWORK_GLOBAL_RECOGNIZER_CVFH_H_


#include <flann/flann.h>
#include <pcl/common/common.h>
#include "source.h"
#include "global_estimator.h"
#include "ourcvfh_estimator.h"
#include <v4r/ORRecognition/hypotheses_verification.h>
#include "recognizer.h"

namespace faat_pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Nearest neighbor search based classification of PCL point type features.
     * Available features: OUR-CVFH
     * \author Aitor Aldoma
     */

    template<template<class > class Distance, typename PointInT, typename FeatureT = pcl::VFHSignature308>
      class PCL_EXPORTS GlobalNNCVFHRecognizer : public Recognizer<PointInT>
      {

      protected:

        struct index_score
        {
          int idx_models_;
          int idx_input_;
          double score_;
        };

        struct sortIndexScores
        {
          bool
          operator() (const index_score& d1, const index_score& d2)
          {
            return d1.score_ < d2.score_;
          }
        } sortIndexScoresOp;

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;

        typedef Distance<float> DistT;
        typedef Model<PointInT> ModelT;
        typedef boost::shared_ptr<ModelT> ModelTPtr;

        using Recognizer<PointInT>::input_;
        using Recognizer<PointInT>::models_;
        using Recognizer<PointInT>::transforms_;
        using Recognizer<PointInT>::ICP_iterations_;
        using Recognizer<PointInT>::icp_type_;
        using Recognizer<PointInT>::VOXEL_SIZE_ICP_;
        using Recognizer<PointInT>::poseRefinement;
        using Recognizer<PointInT>::requires_segmentation_;
        using Recognizer<PointInT>::indices_;
        using Recognizer<PointInT>::hv_algorithm_;

        /** \brief Directory where the trained structure will be saved */
        std::string training_dir_;

        /** \brief Model data source */
        typename boost::shared_ptr<Source<PointInT> > source_;

        /** \brief Computes a feature */
        typename boost::shared_ptr<OURCVFHEstimator<PointInT, FeatureT> > micvfh_estimator_;

        /** \brief Descriptor name */
        std::string descr_name_;

        int OUR_CVFH_MAX_HYP_;

        bool noisify_;
        float noise_;
        int debug_level_;

        class flann_model
        {
        public:
          ModelTPtr model;
          int view_id;
          int descriptor_id;
          std::vector<float> descr;

          bool
          operator< (const flann_model &other) const
          {
            if ((this->model->id_.compare (other.model->id_) < 0))
            {
              return true;
            }
            else
            {

              if (this->model->id_.compare (other.model->id_) == 0)
              {
                //check view id
                if ((this->view_id < other.view_id))
                {
                  return true;
                }
                else
                {
                  if (this->view_id == other.view_id)
                  {
                    if (this->descriptor_id < other.descriptor_id)
                    {
                      return true;
                    }
                  }
                }
              }
            }

            return false;
          }

          bool
          operator== (const flann_model &other) const
          {
            return (model->id_ == other.model->id_) && (view_id == other.view_id) && (descriptor_id == other.descriptor_id);
          }

        };

        pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
        bool normals_set_;

        flann::Matrix<float> flann_data_;
        flann::Index<DistT> * flann_index_;
        std::vector<flann_model> flann_models_;

        std::vector<flann::Matrix<float> > single_categories_data_;
        std::vector<flann::Index<DistT> *> single_categories_index_;
        std::vector<boost::shared_ptr<std::vector<int> > > single_categories_pointers_to_models_;
        std::map<std::string, int> category_to_vectors_indices_;
        std::vector<std::string> categories_to_be_searched_;
        bool use_single_categories_;

        bool use_cache_;
        std::map<std::pair<std::string, int>, Eigen::Matrix4f, std::less<std::pair<std::string, int> >, Eigen::aligned_allocator<std::pair<std::pair<
            std::string, int>, Eigen::Matrix4f> > > poses_cache_;
        std::map<std::pair<std::string, int>, Eigen::Vector3f> centroids_cache_;

        //typedef std::map<std::pair<int, Eigen::Matrix4f>, std::less<int>, Eigen::aligned_allocator<std::pair<int, Eigen::Matrix4f> > > mapRTC;
        //map roll_trans_cache_descriptor_id_;
        //std::map<std::pair<std::string, int>, mapRTC, std::less<std::pair<std::string, int> >, std::pair<std::pair<std::string, int>, mapRTC> > roll_trans_cache_;
        bool compute_scale_;

        //load features from disk and create flann structure
        void
        loadFeaturesAndCreateFLANN ();

        inline void
        convertToFLANN (const std::vector<flann_model> &models, flann::Matrix<float> &data)
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

        inline void
        convertToFLANN (const std::vector<flann_model> &models, boost::shared_ptr<std::vector<int> > & indices, flann::Matrix<float> &data)
        {
          data.rows = indices->size ();
          data.cols = models[0].descr.size (); // number of histogram bins

          flann::Matrix<float> flann_data (new float[indices->size () * models[0].descr.size ()], indices->size (), models[0].descr.size ());

          for (size_t i = 0; i < data.rows; ++i)
            for (size_t j = 0; j < data.cols; ++j)
            {
              flann_data.ptr ()[i * data.cols + j] = models[indices->at (i)].descr[j];
            }

          data = flann_data;
        }

        void
        nearestKSearch (flann::Index<DistT> * index, const flann_model &model, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

        void
        nearestKSearch (flann::Index<DistT> * index, flann::Matrix<float> & p, int k, flann::Matrix<int> &indices, flann::Matrix<float> &distances);

        void
        getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix);

        bool
        getRollPose (ModelT & model, int view_id, int d_id, Eigen::Matrix4f & pose_matrix);

        void
        getCentroid (ModelT & model, int view_id, int d_id, Eigen::Vector3f & centroid);

        void
        getView (ModelT & model, int view_id, PointInTPtr & view);

        size_t NN_;

        std::vector<float> descriptor_distances_;
        float accept_hypotheses_threshold_;
        float max_desc_distance_;

      public:

        GlobalNNCVFHRecognizer () : Recognizer<PointInT>()
        {
          noisify_ = false;
          compute_scale_ = false;
          use_single_categories_ = false;
          accept_hypotheses_threshold_ = 0.8f;
          max_desc_distance_ = 0.65f;
          OUR_CVFH_MAX_HYP_ = std::numeric_limits<int>::max();
          requires_segmentation_ = true;
          debug_level_ = 0;
          normals_set_ = false;
        }

        bool acceptsNormals() const
        {
            return true;
        }

        void setSceneNormals(pcl::PointCloud<pcl::Normal>::Ptr normals)
        {
            scene_normals_ = normals;
            normals_set_ = true;
        }

        void setDebugLevel(int lev)
        {
            debug_level_ = lev;
        }

        ~GlobalNNCVFHRecognizer ()
        {
        }

        void setMaxHyp(const int t) {
          OUR_CVFH_MAX_HYP_ = t;
        }

        void
        setAcceptHypThreshold (const float t)
        {
          accept_hypotheses_threshold_ = t;
        }

        void
        setMaxDescDistance (const float t)
        {
          max_desc_distance_ = t;
        }

        void
        getDescriptorDistances (std::vector<float> & ds)
        {
          ds = descriptor_distances_;
        }

        void
        setComputeScale (const bool d)
        {
          compute_scale_ = d;
        }

        void
        setCategoriesToUseForRecognition (const std::vector<std::string> cats_to_use)
        {
          categories_to_be_searched_.clear ();
          categories_to_be_searched_ = cats_to_use;
        }

        void
        setUseSingleCategories (const bool b)
        {
          use_single_categories_ = b;
        }

        void
        setNoise (const float n)
        {
          noisify_ = true;
          noise_ = n;
        }

        void
        setNN (const size_t nn)
        {
          NN_ = nn;
        }

        /**
         * \brief Initializes the FLANN structure from the provided source
         */

        void
        initialize (bool force_retrain = false);

        /**
         * \brief Sets the model data source_
         */
        void
        setDataSource (typename boost::shared_ptr<Source<PointInT> > source)
        {
          source_ = source;
        }

        typename boost::shared_ptr<Source<PointInT> >
        getDataSource ()
        {
          return source_;
        }

        /**
         * \brief Sets the model data source_
         */

        void
        setFeatureEstimator (typename boost::shared_ptr<OURCVFHEstimator<PointInT, FeatureT> > & feat)
        {
          micvfh_estimator_ = feat;
        }

        void
        setDescriptorName (const std::string name)
        {
          descr_name_ = name;
        }

        void
        setTrainingDir (const std::string dir)
        {
          training_dir_ = dir;
        }

        /**
         * \brief Performs recognition on the input cloud
         */

        void
        recognize ();

        void
        setUseCache (const bool u)
        {
          use_cache_ = u;
        }

      };
  }
}
#endif /* REC_FRAMEWORK_GLOBAL_PIPELINE_H_ */
