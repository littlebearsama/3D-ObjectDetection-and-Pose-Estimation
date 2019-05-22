/*
 * global_nn_classifier.cpp
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#include "global_nn_recognizer_cvfh.h"
#include <pcl/registration/icp.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <pcl/common/time.h>
#include <pcl/visualization/pcl_visualizer.h>

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix)
  {

    if (use_cache_)
    {
      typedef std::pair<std::string, int> mv_pair;
      mv_pair pair_model_view = std::make_pair (model.id_, view_id);

      std::map<mv_pair, Eigen::Matrix4f, std::less<mv_pair>, Eigen::aligned_allocator<std::pair<mv_pair, Eigen::Matrix4f> > >::iterator it =
          poses_cache_.find (pair_model_view);

      if (it != poses_cache_.end ())
      {
        pose_matrix = it->second;
        return;
      }

    }

    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/pose_" << view_id << ".txt";

    PersistenceUtils::readMatrixFromFile2 (dir.str (), pose_matrix);
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  bool
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::getRollPose (ModelT & model, int view_id, int d_id,
                                                                                            Eigen::Matrix4f & pose_matrix)
  {

    /*if (use_cache_)
    {
        typedef std::pair<std::string, int> mv_pair;
        mv_pair pair_model_view = std::make_pair (model.id_, view_id);

        std::map<mv_pair, Eigen::Matrix4f, std::less<mv_pair>, Eigen::aligned_allocator<std::pair<mv_pair, Eigen::Matrix4f> > >::iterator it =
            roll_trans_cache_.find (pair_model_view);

        if (it != roll_trans_cache_.end ())
        {
          pose_matrix = it->second;
          return;
        }
    }*/

    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);

    dir << path << "/roll_trans_" << view_id << "_" << d_id << ".txt";

    bf::path file_path = dir.str ();
    if (bf::exists (file_path))
    {
      PersistenceUtils::readMatrixFromFile2 (dir.str (), pose_matrix);
      return true;
    }
    else
    {
      return false;
    }
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::getCentroid (ModelT & model, int view_id, int d_id,
                                                                                            Eigen::Vector3f & centroid)
  {
    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/centroid_" << view_id << "_" << d_id << ".txt";

    PersistenceUtils::getCentroidFromFile (dir.str (), centroid);
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::getView (ModelT & model, int view_id, PointInTPtr & view)
  {
    view.reset (new pcl::PointCloud<PointInT>);
    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/view_" << view_id << ".pcd";
    pcl::io::loadPCDFile (dir.str (), *view);

  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::loadFeaturesAndCreateFLANN ()
  {

    boost::shared_ptr < std::vector<ModelTPtr> > models = source_->getModels ();

    std::map < std::string, boost::shared_ptr<std::vector<int> > > single_categories;
    if (use_single_categories_)
    {
      for (size_t i = 0; i < models->size (); i++)
      {
        std::map<std::string, boost::shared_ptr<std::vector<int> > >::iterator it;
        std::string cat_model = models->at (i)->class_;
        it = single_categories.find (cat_model);
        if (it == single_categories.end ())
        {
          boost::shared_ptr < std::vector<int> > v (new std::vector<int>);
          single_categories[cat_model] = v;
        }
      }
    }

    for (size_t i = 0; i < models->size (); i++)
    {
      std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);
      bf::path inside = path;
      bf::directory_iterator end_itr;

      for (bf::directory_iterator itr_in (inside); itr_in != end_itr; ++itr_in)
      {
#if BOOST_FILESYSTEM_VERSION == 3
        std::string file_name = (itr_in->path ().filename ()).string();
#else
        std::string file_name = (itr_in->path ()).filename ();
#endif

        std::vector < std::string > strs;
        boost::split (strs, file_name, boost::is_any_of ("_"));

        if (strs[0] == "descriptor")
        {

          int view_id = atoi (strs[1].c_str ());
          std::vector < std::string > strs1;
          boost::split (strs1, strs[2], boost::is_any_of ("."));
          int descriptor_id = atoi (strs1[0].c_str ());

          std::string full_file_name = itr_in->path ().string ();
          typename pcl::PointCloud<FeatureT>::Ptr signature (new pcl::PointCloud<FeatureT>);
          pcl::io::loadPCDFile (full_file_name, *signature);

          flann_model descr_model;
          descr_model.model = models->at (i);
          descr_model.view_id = view_id;
          descr_model.descriptor_id = descriptor_id;

          int size_feat = sizeof(signature->points[0].histogram) / sizeof(float);
          descr_model.descr.resize (size_feat);
          memcpy (&descr_model.descr[0], &signature->points[0].histogram[0], size_feat * sizeof(float));

          if (use_single_categories_)
          {
            std::map<std::string, boost::shared_ptr<std::vector<int> > >::iterator it;
            std::string cat_model = models->at (i)->class_;
            it = single_categories.find (cat_model);
            if (it == single_categories.end ())
            {
              std::cout << cat_model << std::endl;
              std::cout << "Should not happen..." << std::endl;
            }
            else
            {
              it->second->push_back (static_cast<int> (flann_models_.size ()));
            }
          }

          flann_models_.push_back (descr_model);

          if (use_cache_)
          {

            std::stringstream dir_pose;
            dir_pose << path << "/pose_" << descr_model.view_id << ".txt";

            Eigen::Matrix4f pose_matrix;
            PersistenceUtils::readMatrixFromFile2 (dir_pose.str (), pose_matrix);
            std::pair<std::string, int> pair_model_view = std::make_pair (models->at (i)->id_, descr_model.view_id);
            poses_cache_[pair_model_view] = pose_matrix;

            /*{
                std::stringstream dir;
                std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
                dir << path << "/roll_trans_" << descr_model.view_id << "_" << descr_model.descriptor_id << ".txt";
                Eigen::Matrix4f pose_matrix;
                PersistenceUtils::readMatrixFromFile2 (dir.str (), pose_matrix);
                std::pair<std::string, int> pair_model_view = std::make_pair (models->at (i)->id_, descr_model.view_id);
                roll_trans_cache_[pair_model_view] = pose_matrix;
            }*/
          }
        }
      }
    }

    convertToFLANN (flann_models_, flann_data_);
    flann_index_ = new flann::Index<DistT> (flann_data_, flann::LinearIndexParams ());
    flann_index_->buildIndex ();

    //single categories...
    if (use_single_categories_)
    {
      std::map<std::string, boost::shared_ptr<std::vector<int> > >::iterator it;

      single_categories_data_.resize (single_categories.size ());
      single_categories_index_.resize (single_categories.size ());
      single_categories_pointers_to_models_.resize (single_categories.size ());

      int kk = 0;
      for (it = single_categories.begin (); it != single_categories.end (); it++)
      {
        //create index and flann data
        convertToFLANN (flann_models_, it->second, single_categories_data_[kk]);
        single_categories_index_[kk] = new flann::Index<DistT> (single_categories_data_[kk], flann::LinearIndexParams ());
        single_categories_pointers_to_models_[kk] = it->second;

        category_to_vectors_indices_[it->first] = kk;
        kk++;
      }
    }
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::nearestKSearch (flann::Index<DistT> * index, const flann_model &model,
                                                                                               int k, flann::Matrix<int> &indices,
                                                                                               flann::Matrix<float> &distances)
  {
    flann::Matrix<float> p = flann::Matrix<float> (new float[model.descr.size ()], 1, model.descr.size ());
    memcpy (&p.ptr ()[0], &model.descr[0], p.cols * p.rows * sizeof(float));

    indices = flann::Matrix<int> (new int[k], 1, k);
    distances = flann::Matrix<float> (new float[k], 1, k);
    index->knnSearch (p, indices, distances, k, flann::SearchParams (512));
    delete[] p.ptr ();
  }

  template<template<class > class Distance, typename PointInT, typename FeatureT>
    void
    faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::nearestKSearch (flann::Index<DistT> * index,
                                                                                                      flann::Matrix<float> & p,
                                                                                                      int k, flann::Matrix<int> &indices,
                                                                                                      flann::Matrix<float> &distances)
    {
      index->knnSearch (p, indices, distances, k, flann::SearchParams (512));
    }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::recognize ()
  {

    models_.reset (new std::vector<ModelTPtr>);
    transforms_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

    PointInTPtr processed (new pcl::PointCloud<PointInT>);
    PointInTPtr in (new pcl::PointCloud<PointInT>);

    std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures;
    std::vector < Eigen::Vector3f > centroids;

    if(micvfh_estimator_->getUsesOrganizedData())
    {
        if(indices_.size() > 0)
        {
            pcl::copyPointCloud (*input_, *in);
            std::vector<bool> negative_indices(in->points.size(), true);
            for(size_t i=0; i < indices_.size(); i++)
                negative_indices[indices_[i]] = false;

            for(size_t i=0; i < in->points.size(); i++)
            {
                if(negative_indices[i])
                    in->points[i].x = in->points[i].z = in->points[i].y = std::numeric_limits<float>::quiet_NaN();
            }
        }
        else
            in = input_;
    }
    else
    {
        if (indices_.size () > 0)
          pcl::copyPointCloud (*input_, indices_, *in);
        else
          in = input_;

        /*{
          //pcl::ScopeTime t ("Estimate feature");
          micvfh_estimator_->estimate (in, processed, signatures, centroids);
        }*/
    }

    {
      //pcl::ScopeTime t ("Estimate feature");
      if(normals_set_)
      {
          std::cout << "normals_set OURCVFH:" << normals_set_ << std::endl;
          std::cout << "uses organized data:" << micvfh_estimator_->getUsesOrganizedData() << std::endl;
      }

      if(normals_set_ && micvfh_estimator_->getUsesOrganizedData())
      {
          PCL_WARN("setting normals\n");
          micvfh_estimator_->setNormals(scene_normals_);
      }

      micvfh_estimator_->estimate (in, processed, signatures, centroids);
    }

    std::vector<index_score> indices_scores;
    descriptor_distances_.clear ();

    if (signatures.size () > 0)
    {

      {
        pcl::ScopeTime t_matching ("Matching and roll...");
        if (use_single_categories_ && (categories_to_be_searched_.size () > 0))
        {

          //perform search of the different signatures in the categories_to_be_searched_
          for (size_t c = 0; c < categories_to_be_searched_.size (); c++)
          {
            std::cout << "Using category:" << categories_to_be_searched_[c] << std::endl;
            for (size_t idx = 0; idx < signatures.size (); idx++)
            {
              /*float* hist = signatures[idx].points[0].histogram;
               std::vector<float> std_hist (hist, hist + getHistogramLength (dummy));
               flann_model histogram ("", std_hist);
               flann::Matrix<int> indices;
               flann::Matrix<float> distances;

               std::map<std::string, int>::iterator it;
               it = category_to_vectors_indices_.find (categories_to_be_searched_[c]);

               assert (it != category_to_vectors_indices_.end ());
               nearestKSearch (single_categories_index_[it->second], histogram, nmodels_, indices, distances);*/

              float* hist = signatures[idx].points[0].histogram;
              int size_feat = sizeof(signatures[idx].points[0].histogram) / sizeof(float);
              std::vector<float> std_hist (hist, hist + size_feat);
              //ModelT empty;

              flann_model histogram;
              histogram.descr = std_hist;
              flann::Matrix<int> indices;
              flann::Matrix<float> distances;

              std::map<std::string, int>::iterator it;
              it = category_to_vectors_indices_.find (categories_to_be_searched_[c]);
              assert (it != category_to_vectors_indices_.end ());

              nearestKSearch (single_categories_index_[it->second], histogram, NN_, indices, distances);
              //gather NN-search results
              double score = 0;
              for (size_t i = 0; i < NN_; ++i)
              {
                score = distances[0][i];
                index_score is;
                is.idx_models_ = single_categories_pointers_to_models_[it->second]->at (indices[0][i]);
                is.idx_input_ = static_cast<int> (idx);
                is.score_ = score;
                indices_scores.push_back (is);
              }
            }

            //we cannot add more than nmodels per category, so sort here and remove offending ones...
            std::sort (indices_scores.begin (), indices_scores.end (), sortIndexScoresOp);
            indices_scores.resize ((c + 1) * NN_);
          }
        }
        else
        {
          pcl::ScopeTime t("Matching knn");
          double knn_seconds = 0;
          int size_feat = 0;
          if(signatures.size() > 0)
              size_feat = sizeof(signatures[0].points[0].histogram) / sizeof(float);

          flann::Matrix<int> indices = flann::Matrix<int> (new int[NN_], 1, NN_);
          flann::Matrix<float> distances = flann::Matrix<float> (new float[NN_], 1, NN_);
          flann::Matrix<float> p = flann::Matrix<float> (new float[size_feat], 1, size_feat);

          for (size_t idx = 0; idx < signatures.size (); idx++)
          {

            float* hist = signatures[idx].points[0].histogram;
            memcpy (&p.ptr ()[0], hist, size_feat * sizeof(float));

            pcl::StopWatch stop_watch;
            nearestKSearch (flann_index_, p, NN_, indices, distances);
            knn_seconds += stop_watch.getTimeSeconds();

            //gather NN-search results
            double score = 0;
            for (size_t i = 0; i < NN_; ++i)
            {
              score = distances[0][i];
              index_score is;
              is.idx_models_ = indices[0][i];
              is.idx_input_ = static_cast<int> (idx);
              is.score_ = score;
              indices_scores.push_back (is);
            }
          }

          delete[] indices.ptr();
          delete[] distances.ptr();
          delete[] p.ptr ();

          if(debug_level_ > 1)
            std::cout << "nearest K search:" << knn_seconds << std::endl;
        }

        if(debug_level_ > 0)
            std::cout << "Number of hypotheses for cluster:" << indices_scores.size() << std::endl;

        std::sort (indices_scores.begin (), indices_scores.end (), sortIndexScoresOp);

        std::map< std::string, std::vector<index_score> > model_idx_score_map;
        typename std::map< std::string, std::vector<index_score> >::iterator it_map;
        for (size_t i = 0; i < indices_scores.size (); i++)
        {
          std::string m_id = flann_models_[indices_scores[i].idx_models_].model->id_;
          it_map = model_idx_score_map.find (m_id);
          if(it_map == model_idx_score_map.end())
          {
            std::vector<index_score> tmp;
            tmp.push_back(indices_scores[i]);
            model_idx_score_map[m_id] = tmp;
          }
          else
          {
            it_map->second.push_back(indices_scores[i]);
          }
        }

        indices_scores.clear();

        //take a maximum number of hypotheses per model that represent a substantially different pose
        for(it_map = model_idx_score_map.begin(); it_map != model_idx_score_map.end(); it_map++)
        {
          if(debug_level_ > 0)
            std::cout << it_map->first << " " <<  it_map->second.size() << std::endl;

          int max_hyp_per_model = std::min(10, static_cast<int>(std::ceil( static_cast<int>(it_map->second.size()) * 0.1f))); //10% of the total hypotheses
          it_map->second.resize(std::min(max_hyp_per_model, static_cast<int>(it_map->second.size())));
          for(size_t i=0; i < it_map->second.size(); i++)
          {
            indices_scores.push_back(it_map->second[i]);
          }
        }

        for(it_map = model_idx_score_map.begin(); it_map != model_idx_score_map.end(); it_map++)
        {
          if(debug_level_ > 0)
            std::cout << it_map->first << " " <<  it_map->second.size() << std::endl;
        }

        if(debug_level_ > 0)
            std::cout << "Number of hypotheses for cluster:" << indices_scores.size() << std::endl;

        std::sort (indices_scores.begin (), indices_scores.end (), sortIndexScoresOp);

        int num_n = std::min (OUR_CVFH_MAX_HYP_, static_cast<int> (indices_scores.size ()));
        {
          std::vector < index_score > indices_scores_filtered;
          indices_scores_filtered.resize (num_n);

          int kept = 0;
          for (int i = 0; i < num_n; ++i)
          {
              ModelTPtr m = flann_models_[indices_scores[i].idx_models_].model;
              if(debug_level_ > 0)
              {
                std::cout << m->class_ << "/" << m->id_ << " ==> " << indices_scores[i].score_ << std::endl;
              }

            //std::cout << best_score << indices_scores[i].score_ << (best_score / indices_scores[i].score_) << std::endl;
            if (indices_scores[i].score_ <= max_desc_distance_)
            {
              indices_scores_filtered[i] = indices_scores[i];
              kept++;
            }
          }

          indices_scores_filtered.resize (kept);
          //std::cout << indices_scores_filtered.size () << " § " << num_n << std::endl;

          indices_scores = indices_scores_filtered;
          num_n = static_cast<int> (indices_scores.size ());
        }

        //std::cout << "Number of object hypotheses... " << num_n << std::endl;

        std::vector<bool> valid_trans;
        std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;

        micvfh_estimator_->getValidTransformsVec (valid_trans);
        micvfh_estimator_->getTransformsVec (transformations);

        {
            pcl::ScopeTime time_roll("roll");
            for (int i = 0; i < num_n; ++i)
            {
              ModelTPtr m = flann_models_[indices_scores[i].idx_models_].model;
              int view_id = flann_models_[indices_scores[i].idx_models_].view_id;
              int desc_id = flann_models_[indices_scores[i].idx_models_].descriptor_id;

              int idx_input = indices_scores[i].idx_input_;

              if(debug_level_ > 0)
              {
                std::cout << m->class_ << "/" << m->id_ << " ==> " << indices_scores[i].score_ << std::endl;
              }

              Eigen::Matrix4f roll_view_pose;
              bool roll_pose_found = getRollPose (*m, view_id, desc_id, roll_view_pose);

              if (roll_pose_found && valid_trans[idx_input])
              {
                Eigen::Matrix4f model_view_pose;
                getPose (*m, view_id, model_view_pose);

                Eigen::Matrix4f scale_mat;
                scale_mat.setIdentity (4, 4);

                if (compute_scale_)
                {

                  PointInTPtr view;
                  getView (*m, view_id, view);

                  //compute scale using the whole view
                  PointInTPtr view_transformed (new pcl::PointCloud<PointInT>);
                  Eigen::Matrix4f hom_from_OVC_to_CC;
                  hom_from_OVC_to_CC = transformations[idx_input].inverse () * roll_view_pose;
                  pcl::transformPointCloud (*view, *view_transformed, hom_from_OVC_to_CC);

                  Eigen::Vector3f input_centroid = centroids[indices_scores[i].idx_input_];
                  Eigen::Vector3f view_centroid;
                  getCentroid (*m, view_id, desc_id, view_centroid);

                  Eigen::Vector4f cmatch4f (view_centroid[0], view_centroid[1], view_centroid[2], 0);
                  Eigen::Vector4f cinput4f (input_centroid[0], input_centroid[1], input_centroid[2], 0);

                  Eigen::Vector4f max_pt_input;
                  pcl::getMaxDistance (*processed, cinput4f, max_pt_input);
                  max_pt_input[3] = 0;
                  float max_dist_input = (cinput4f - max_pt_input).norm ();

                  //compute max dist for transformed model_view
                  pcl::getMaxDistance (*view, cmatch4f, max_pt_input);
                  max_pt_input[3] = 0;
                  float max_dist_view = (cmatch4f - max_pt_input).norm ();

                  cmatch4f = hom_from_OVC_to_CC * cmatch4f;
                  //std::cout << max_dist_view << " " << max_dist_input << std::endl;

                  float scale_factor_view = max_dist_input / max_dist_view;
                  //std::cout << "Scale factor:" << scale_factor_view << std::endl;

                  Eigen::Matrix4f center, center_inv;

                  center.setIdentity (4, 4);
                  center (0, 3) = -cinput4f[0];
                  center (1, 3) = -cinput4f[1];
                  center (2, 3) = -cinput4f[2];

                  center_inv.setIdentity (4, 4);
                  center_inv (0, 3) = cinput4f[0];
                  center_inv (1, 3) = cinput4f[1];
                  center_inv (2, 3) = cinput4f[2];

                  scale_mat (0, 0) = scale_factor_view;
                  scale_mat (1, 1) = scale_factor_view;
                  scale_mat (2, 2) = scale_factor_view;

                  scale_mat = center_inv * scale_mat * center;
                }

                //std::cout << roll_view_pose << std::endl;
                //std::cout << model_view_pose << std::endl;
                Eigen::Matrix4f hom_from_OC_to_CC;
                hom_from_OC_to_CC = scale_mat * transformations[idx_input].inverse () * roll_view_pose * model_view_pose;

                models_->push_back (m);
                transforms_->push_back (hom_from_OC_to_CC);
                descriptor_distances_.push_back (static_cast<float> (indices_scores[i].score_));
              }
              else
              {
                PCL_WARN("The roll pose was not found, should use CRH here... \n");
              }
            }
        }
      }

      //std::cout << "Number of object hypotheses:" << models_->size () << std::endl;

      /**
       * POSE REFINEMENT
       **/

      if (ICP_iterations_ > 0)
      {
        poseRefinement();
      }

      /**
       * HYPOTHESES VERIFICATION
       **/

      if (hv_algorithm_ && models_->size() > 0)
      {

        pcl::ScopeTime t ("HYPOTHESES VERIFICATION");

        std::vector<typename pcl::PointCloud<PointInT>::ConstPtr> aligned_models;
        aligned_models.resize (models_->size ());

        for (size_t i = 0; i < models_->size (); i++)
        {
          ConstPointInTPtr model_cloud;
          PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);

          if (compute_scale_)
          {
            model_cloud = models_->at (i)->getAssembled (-1);
            PointInTPtr model_aligned_m (new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud (*model_cloud, *model_aligned_m, transforms_->at (i));
            pcl::VoxelGrid<PointInT> voxel_grid_icp;
            voxel_grid_icp.setInputCloud (model_aligned_m);
            voxel_grid_icp.setLeafSize (0.005f, 0.005f, 0.005f);
            voxel_grid_icp.filter (*model_aligned);
          }
          else
          {
            model_cloud = models_->at (i)->getAssembled (0.005f);
            pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));
          }

          //ConstPointInTPtr model_cloud = models_->at (i).getAssembled (0.005f);
          //PointInTPtr model_aligned (new pcl::PointCloud<PointInT>);
          //pcl::transformPointCloud (*model_cloud, *model_aligned, transforms_->at (i));
          aligned_models[i] = model_aligned;
        }

        std::vector<bool> mask_hv;
        hv_algorithm_->setSceneCloud (input_);
        hv_algorithm_->addModels (aligned_models, true);
        hv_algorithm_->verify ();
        hv_algorithm_->getMask (mask_hv);

        boost::shared_ptr < std::vector<ModelTPtr> > models_temp;
        boost::shared_ptr < std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > transforms_temp;

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

    }
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::GlobalNNCVFHRecognizer<Distance, PointInT, FeatureT>::initialize (bool force_retrain)
  {

    //use the source to know what has to be trained and what not, checking if the descr_name directory exists
    //unless force_retrain is true, then train everything
    boost::shared_ptr < std::vector<ModelTPtr> > models = source_->getModels ();
    std::cout << "Models size:" << models->size () << std::endl;

    if (force_retrain)
    {
      for (size_t i = 0; i < models->size (); i++)
      {
        source_->removeDescDirectory (*models->at (i), training_dir_, descr_name_);
      }
    }

    for (size_t i = 0; i < models->size (); i++)
    {
      if (!source_->modelAlreadyTrained (*models->at (i), training_dir_, descr_name_))
      {
        if(!source_->getLoadIntoMemory())
          source_->loadInMemorySpecificModel(training_dir_, *(models->at (i)));

        std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);

        bf::path desc_dir = path;
        if (!bf::exists (desc_dir)) {
          std::cout << "Creating dir..." << path << std::endl;
          bf::create_directory (desc_dir);
        } else {
          std::cout << "dir alredy exists..." << path << std::endl;
        }

        std::cout << "Number of views..." << models->at (i)->views_->size () << std::endl;
        for (size_t v = 0; v < models->at (i)->views_->size (); v++)
        {
          PointInTPtr processed (new pcl::PointCloud<PointInT>);
          PointInTPtr view = models->at (i)->views_->at (v);

          if (view->points.size () == 0)
            PCL_WARN("View has no points!!!\n");

          if (noisify_)
          {
            double noise_std = noise_;
            boost::posix_time::ptime time = boost::posix_time::microsec_clock::local_time ();
            boost::posix_time::time_duration duration (time.time_of_day ());
            boost::mt19937 rng;
            rng.seed (static_cast<unsigned int> (duration.total_milliseconds ()));
            boost::normal_distribution<> nd (0.0, noise_std);
            boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);
            // Noisify each point in the dataset
            for (size_t cp = 0; cp < view->points.size (); ++cp)
              view->points[cp].z += static_cast<float> (var_nor ());

          }

          //pro view, compute signatures
          std::vector<pcl::PointCloud<FeatureT>, Eigen::aligned_allocator<pcl::PointCloud<FeatureT> > > signatures;
          std::vector < Eigen::Vector3f > centroids;

          if(models->at(i)->indices_ && (models->at(i)->indices_->at (v).indices.size() > 0)
                                     && micvfh_estimator_->acceptsIndices())
          {
            std::cout << "micvfh_estimator accepts indices:" << micvfh_estimator_->acceptsIndices() << " size:" << models->at(i)->indices_->at (v).indices.size() << std::endl;
            micvfh_estimator_->setIndices(models->at(i)->indices_->at (v));
          }

          micvfh_estimator_->estimate (view, processed, signatures, centroids);

          std::vector<bool> valid_trans;
          std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms;

          micvfh_estimator_->getValidTransformsVec (valid_trans);
          micvfh_estimator_->getTransformsVec (transforms);

          std::stringstream path_view;
          path_view << path << "/view_" << v << ".pcd";
          pcl::io::savePCDFileBinary (path_view.str (), *processed);

          std::stringstream path_pose;
          path_pose << path << "/pose_" << v << ".txt";
          PersistenceUtils::writeMatrixToFile (path_pose.str (), models->at (i)->poses_->at (v));

          std::stringstream path_entropy;
          path_entropy << path << "/entropy_" << v << ".txt";
          PersistenceUtils::writeFloatToFile (path_entropy.str (), models->at (i)->self_occlusions_->at (v));

          //save signatures and centroids to disk
          for (size_t j = 0; j < signatures.size (); j++)
          {
            if (valid_trans[j])
            {
              std::stringstream path_centroid;
              path_centroid << path << "/centroid_" << v << "_" << j << ".txt";
              Eigen::Vector3f centroid (centroids[j][0], centroids[j][1], centroids[j][2]);
              PersistenceUtils::writeCentroidToFile (path_centroid.str (), centroid);

              std::stringstream path_descriptor;
              path_descriptor << path << "/descriptor_" << v << "_" << j << ".pcd";
              pcl::io::savePCDFileBinary (path_descriptor.str (), signatures[j]);

              //save roll transform
              std::stringstream path_pose;
              path_pose << path << "/roll_trans_" << v << "_" << j << ".txt";
              PersistenceUtils::writeMatrixToFile (path_pose.str (), transforms[j]);
            }
          }
        }

        if(!source_->getLoadIntoMemory())
          models->at (i)->views_->clear();

      }
      else
      {
        //else skip model
        std::cout << "The model has already been trained..." << std::endl;
        //there is no need to keep the views in memory once the model has been trained
        models->at (i)->views_->clear ();
      }
    }

    //load features from disk
    //initialize FLANN structure
    loadFeaturesAndCreateFLANN ();

    if(ICP_iterations_ > 0)
      source_->createVoxelGridAndDistanceTransform(VOXEL_SIZE_ICP_);
  }
