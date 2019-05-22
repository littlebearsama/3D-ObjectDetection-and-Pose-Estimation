#ifndef FAAT_PCL_REC_FRAMEWORK_HG_LOCAL_RECOGNIZER_H_
#define FAAT_PCL_REC_FRAMEWORK_HG_LOCAL_RECOGNIZER_H_

#include "local_recognizer.h"
#include <pcl/features/shot_lrf_omp.h>
#include <v4r/ORRecognition/hough_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/board.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    /*
     * This class applies HoughGrouping to the correspondences yielded by a LocalRecognitionPipeline
     * It requires for each keypoint (model and scene) to have an associated LRF.
     * In practice, we have a huge pcl::ReferenceFrame point cloud where each descriptor in flann_models_ (inherited
     * from LocalRecognitionPipeline) is associated with a pcl::ReferenceFrame
     */
    template<template<class > class Distance, typename PointInT, typename FeatureT, typename TypeRF = pcl::ReferenceFrame>
      class FAAT_3D_FRAMEWORK_API LocalRecognitionHoughGroupingPipeline : public LocalRecognitionPipeline<Distance, PointInT, FeatureT>
      {

        typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
        typedef typename pcl::PointCloud<PointInT>::ConstPtr ConstPointInTPtr;
        typedef Distance<float> DistT;
        typedef Model<PointInT> ModelT;
        typedef boost::shared_ptr<ModelT> ModelTPtr;

        using LocalRecognitionPipeline<Distance, PointInT, FeatureT>::flann_models_;
        using LocalRecognitionPipeline<Distance, PointInT, FeatureT>::model_view_id_to_flann_models_;
        using LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getView;
        using LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getKeypoints;
        using LocalRecognitionPipeline<Distance, PointInT, FeatureT>::cg_algorithm_;
        //typedef typename LocalRecognitionPipeline<Distance, PointInT, FeatureT>::ObjectHypothesis ObjectHypothesis;
        typename pcl::PointCloud<TypeRF>::Ptr flann_models_rfs_;
        float support_radius_;
        typename pcl::PointCloud<TypeRF>::Ptr scene_rfs_;
        bool use_board_;

        void
        specificLoadFeaturesAndCreateFLANN ()
        {
          std::cout << "specificLoadFeaturesAndCreateFLANN in LocalRecognitionHoughGroupingPipeline" << std::endl;
          std::cout << "This functions computes LRF for model keypoints and stores them..." << std::endl;

          flann_models_rfs_.reset (new pcl::PointCloud<TypeRF>);
          if (pcl::io::loadPCDFile ("rfs.pcd", *flann_models_rfs_) < 0)
          {
            flann_models_rfs_->points.resize (flann_models_.size ());

            typename std::map<std::pair<ModelTPtr, int>, std::vector<int> >::iterator it;
            for (it = model_view_id_to_flann_models_.begin (); it != model_view_id_to_flann_models_.end (); it++)
            {
              //load view, compute normals...
              PointInTPtr view (new pcl::PointCloud<PointInT>);
              PointInTPtr keypoints (new pcl::PointCloud<PointInT>);
              getView (*(it->first.first), it->first.second, view);
              getKeypoints (*(it->first.first), it->first.second, keypoints);

              //compute lrfs
              typename pcl::PointCloud<TypeRF> rfs;

              if (!use_board_)
              {
                pcl::SHOTLocalReferenceFrameEstimationOMP<PointInT> shot_lrf_est;

                shot_lrf_est.setNumberOfThreads (8);
                shot_lrf_est.setSearchSurface (view);
                shot_lrf_est.setInputCloud (keypoints);
                shot_lrf_est.setRadiusSearch (support_radius_);
                shot_lrf_est.compute (rfs);
              }
              else
              {
                pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
                pcl::NormalEstimationOMP<PointInT, pcl::Normal> norm_est;
                norm_est.setKSearch (10);
                norm_est.setInputCloud (view);
                norm_est.compute (*model_normals);

                pcl::BOARDLocalReferenceFrameEstimation<PointInT, pcl::Normal> board_lrf_est;
                board_lrf_est.setFindHoles (true);
                board_lrf_est.setSearchSurface (view);
                board_lrf_est.setInputNormals (model_normals);
                board_lrf_est.setInputCloud (keypoints);
                board_lrf_est.setRadiusSearch (support_radius_);
                board_lrf_est.compute (rfs);
              }

              //put lrfs in the right position in flann_models_rfs_
              if(!(rfs.points.size() == it->second.size()))
                std::cout << rfs.points.size() << " " << it->second.size() << " " << keypoints->points.size() << " " << (it->first.first)->id_ << std::endl;

              //assert(rfs.points.size() == it->second.size());
              int valid_rf = 0;
              for (size_t i = 0; i < rfs.points.size (); i++)
              {
                bool valid = true;
                for(size_t k=0; k < 9; k++)
                {
                  if(pcl_isnan(rfs.points[i].rf[k]))
                  {
                    PCL_ERROR("rf is nan...\n");
                    valid = false;
                  }
                }

                if(valid) {
                  flann_models_rfs_->points[it->second[valid_rf]] = rfs.points[i];
                  valid_rf++;
                }
              }
            }

            pcl::io::savePCDFileBinary ("rfs.pcd", *flann_models_rfs_);
          }
        }

        virtual void
        prepareSpecificCG (PointInTPtr & scene_cloud, PointInTPtr & scene_keypoints)
        {
          //compute reference frames for scene cloud...
          scene_rfs_.reset (new pcl::PointCloud<TypeRF>);

          //compute lrfs

          if (use_board_)
          {
            pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
            pcl::NormalEstimationOMP<PointInT, pcl::Normal> norm_est;
            norm_est.setKSearch (10);
            norm_est.setInputCloud (scene_cloud);
            norm_est.compute (*model_normals);

            pcl::BOARDLocalReferenceFrameEstimation<PointInT, pcl::Normal> board_lrf_est;
            board_lrf_est.setFindHoles (true);
            board_lrf_est.setSearchSurface (scene_cloud);
            board_lrf_est.setInputNormals (model_normals);
            board_lrf_est.setInputCloud (scene_keypoints);
            board_lrf_est.setRadiusSearch (support_radius_);
            board_lrf_est.compute (*scene_rfs_);
          }
          else
          {
            pcl::SHOTLocalReferenceFrameEstimationOMP<PointInT> shot_lrf_est;
            shot_lrf_est.setNumberOfThreads (8);
            shot_lrf_est.setSearchSurface (scene_cloud);
            shot_lrf_est.setInputCloud (scene_keypoints);
            shot_lrf_est.setRadiusSearch (support_radius_);
            shot_lrf_est.compute (*scene_rfs_);
          }

          boost::shared_ptr<faat_pcl::Hough3DGrouping<PointInT, PointInT, TypeRF, TypeRF> > hough_3d_voting_cg_alg;
          hough_3d_voting_cg_alg = boost::static_pointer_cast<faat_pcl::Hough3DGrouping<PointInT, PointInT, TypeRF, TypeRF> > (cg_algorithm_);
          hough_3d_voting_cg_alg->setSceneRf (scene_rfs_);
        }

        virtual void
        clearSpecificCG ()
        {
          scene_rfs_->clear ();
        }

        void
        specificCG (PointInTPtr & /*scene_clooud*/, PointInTPtr & /*scene_keypoints*/, ObjectHypothesis<PointInT> & oh)
        {
          std::cout << "specificCG => this function should set the reference frames to the cg_algorithm class, after casting" << std::endl;
          typename pcl::PointCloud<TypeRF>::Ptr model_rfs (new pcl::PointCloud<TypeRF>);
          pcl::copyPointCloud (*flann_models_rfs_, oh.indices_to_flann_models_, *model_rfs);

          //downcast...
          boost::shared_ptr<faat_pcl::Hough3DGrouping<PointInT, PointInT, TypeRF, TypeRF> > hough_3d_voting_cg_alg;
          hough_3d_voting_cg_alg = boost::static_pointer_cast<faat_pcl::Hough3DGrouping<PointInT, PointInT, TypeRF, TypeRF> > (cg_algorithm_);
          hough_3d_voting_cg_alg->setInputRf (model_rfs);
          hough_3d_voting_cg_alg->setSceneRf (scene_rfs_);
        }

      public:
        LocalRecognitionHoughGroupingPipeline (std::string index_fn = std::string ("index_flann.txt"),
                                               std::string cb_index_fn = std::string ("index_codebook.txt")) :
          LocalRecognitionPipeline<Distance, PointInT, FeatureT> (index_fn, cb_index_fn)
        {
          support_radius_ = 0.04f;
          use_board_ = false;
        }

        void setUseBoard(bool use)
        {
          use_board_ = use;
        }

        void setRFRadius(float r)
        {
          support_radius_ = r;
        }
      };
  }
}
#endif //FAAT_PCL_REC_FRAMEWORK_HG_LOCAL_RECOGNIZER_H_
