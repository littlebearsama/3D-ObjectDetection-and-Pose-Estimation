/*
 * local_recognizer.h
 *
 *  Created on: Mar 24, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_LINEMOD3D_RECOGNIZER_H_
#define FAAT_PCL_REC_FRAMEWORK_LINEMOD3D_RECOGNIZER_H_

#include <pcl/common/common.h>
#include <v4r/ORRecognition/hypotheses_verification.h>
#include "source.h"
#include "faat_3d_rec_framework_defines.h"
#include "recognizer.h"
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <v4r/ORUtils/pcl_opencv.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    /**
     * \brief Object recognition + 6DOF pose based on Linemod
     */

    template<typename PointInT>
      class FAAT_3D_FRAMEWORK_API LineMod3DPipeline : public Recognizer<PointInT>
      {
        protected:
          typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
          typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;

          typedef Model<PointInT> ModelT;
          typedef boost::shared_ptr<ModelT> ModelTPtr;

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

          /** \brief Directory where the trained structure will be saved */
          std::string training_dir_;

          /** \brief Model data source */
          typename boost::shared_ptr<Source<PointInT> > source_;

          /** \brief Descriptor name */
          std::string descr_name_;

          /** \brief Id of the model to be used */
          std::string search_model_;

          bool use_cache_;
          std::map<std::pair<std::string, int>, Eigen::Matrix4f, std::less<std::pair<std::string, int> >, Eigen::aligned_allocator<std::pair<std::pair<
              std::string, int>, Eigen::Matrix4f> > > poses_cache_;

          float threshold_accept_model_hypothesis_;

          void
          getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix);

          void
          getView (ModelT & model, int view_id, PointInTPtr & view);

          struct view_model
          {
            ModelTPtr model_;
            int view_id_;
          };

          std::vector<view_model> template_id_to_model_;
          pcl::LINEMOD linemod_;

          void getMaskFromObjectIndices(pcl::PointIndices & indices,
                                            PointInTPtr & cloud,
                                            pcl::MaskMap & mask, pcl::RegionXY & region)
          {
            pcl::PointCloud<int> mask_cloud;
            mask_cloud.width = cloud->width;
            mask_cloud.height = cloud->height;
            mask_cloud.points.resize(cloud->width * cloud->height);
            for(size_t k=0; k < mask_cloud.points.size(); k++)
              mask_cloud.points[k] = 0;

            for(size_t k=0; k < indices.indices.size(); k++)
              mask_cloud.points[indices.indices[k]] = 1;

            mask.resize(cloud->width, cloud->height);
            size_t min_x (cloud->width), min_y (cloud->height), max_x (0), max_y (0);

            for (size_t j = 0; j < cloud->height; ++j)
            {
              for (size_t i = 0; i < cloud->width; ++i)
              {
                mask (i,j) = mask_cloud.points[j*cloud->width+i];
                if (mask_cloud.points[j*cloud->width+i])
                {
                  min_x = std::min (min_x, i);
                  max_x = std::max (max_x, i);
                  min_y = std::min (min_y, j);
                  max_y = std::max (max_y, j);
                }
              }
            }

            //pcl::RegionXY region;
            region.x = static_cast<int> (min_x);
            region.y = static_cast<int> (min_y);
            region.width = static_cast<int> (max_x - min_x + 1);
            region.height = static_cast<int> (max_y - min_y + 1);
          }

      public:

        LineMod3DPipeline () : Recognizer<PointInT>()
        {
          use_cache_ = false;
          threshold_accept_model_hypothesis_ = 0.2f;
          search_model_ = "";
        }

        ~LineMod3DPipeline ()
        {

        }

        void
        setSearchModel (const std::string & id)
        {
          search_model_ = id;
        }

        void
        setThresholdAcceptHyp (const float t)
        {
          threshold_accept_model_hypothesis_ = t;
        }

        void
        setIndices (const std::vector<int> &indices)
        {
          indices_ = indices;
        }

        void
        setUseCache (const bool u)
        {
          use_cache_ = u;
        }

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
         * \brief Sets the descriptor name
         */
        void
        setDescriptorName (const std::string name)
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

        /**
         * \brief Initializes the recognizers
         * It does training for the models that havent been trained yet, if force_retrain = true, then trains all again
         */

        void
        initialize (const bool force_retrain = false);

        /**
         * \brief Performs recognition and pose estimation on the input cloud
         */

        void
        recognize ();
      };
  }
}

#endif /* REC_FRAMEWORK_LOCAL_RECOGNIZER_H_ */
