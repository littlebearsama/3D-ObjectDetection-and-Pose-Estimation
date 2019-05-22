#include "local_recognizer.h"

//#include <pcl/visualization/pcl_visualizer.h>
template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::loadFeaturesAndCreateFLANN ()
  {
    boost::shared_ptr < std::vector<ModelTPtr> > models = source_->getModels ();
    std::cout << "Models size:" << models->size () << std::endl;
    std::cout << "use cache:" << static_cast<int>(use_cache_) << std::endl;
    int idx_flann_models = 0;
    for (size_t i = 0; i < models->size (); i++)
    {
      pcl::ScopeTime t("Model finished");

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
          //std::cout << "Using cache:" << (int)use_cache_ << std::endl;
          std::string full_file_name = itr_in->path ().string ();
          std::string name = file_name.substr (0, file_name.length () - 4);
          std::vector < std::string > strs;
          boost::split (strs, name, boost::is_any_of ("_"));

          flann_model descr_model;
          descr_model.model = models->at (i);
          descr_model.view_id = atoi (strs[1].c_str ());

          if (use_cache_)
          {

            std::stringstream dir_keypoints;
            std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);
            dir_keypoints << path << "/keypoint_indices_" << descr_model.view_id << ".pcd";

            std::stringstream dir_pose;
            dir_pose << path << "/pose_" << descr_model.view_id << ".txt";

            Eigen::Matrix4f pose_matrix;
            PersistenceUtils::readMatrixFromFile2 (dir_pose.str (), pose_matrix);
            std::pair<std::string, int> pair_model_view = std::make_pair (models->at (i)->id_, descr_model.view_id);
            poses_cache_[pair_model_view] = pose_matrix;

            //load keypoints and save them to cache
            typename pcl::PointCloud<PointInT>::Ptr keypoints (new pcl::PointCloud<PointInT> ());
            pcl::io::loadPCDFile (dir_keypoints.str (), *keypoints);
            keypoints_cache_[pair_model_view] = keypoints;

            if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
            {
              std::stringstream dir_normals;
              dir_normals << path << "/normals_" << descr_model.view_id << ".pcd";
              pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());
              pcl::io::loadPCDFile (dir_normals.str (), *normals_cloud);
              normals_cache_[pair_model_view] = normals_cloud;

              std::stringstream dir_idxpoint;
              dir_idxpoint << path << "/keypoints_indices_to_processed_and_normals_" << descr_model.view_id << ".pcd";
              pcl::PointCloud<IndexPoint>::Ptr index_cloud (new pcl::PointCloud<IndexPoint> ());
              pcl::io::loadPCDFile (dir_idxpoint.str (), *index_cloud);
              idxpoint_cache_[pair_model_view] = index_cloud;
            }
          }

          typename pcl::PointCloud<FeatureT>::Ptr signature (new pcl::PointCloud<FeatureT> ());
          pcl::io::loadPCDFile (full_file_name, *signature);

          int size_feat = sizeof(signature->points[0].histogram) / sizeof(float);

          std::vector<int> idx_flann_models_for_this_view;
          idx_flann_models_for_this_view.reserve(signature->points.size ());

          for (size_t dd = 0; dd < signature->points.size (); dd++)
          {
            descr_model.keypoint_id = static_cast<int> (dd);
            descr_model.descr.resize (size_feat);

            memcpy (&descr_model.descr[0], &signature->points[dd].histogram[0], size_feat * sizeof(float));

            flann_models_.push_back (descr_model);
            idx_flann_models_for_this_view.push_back(idx_flann_models);
            idx_flann_models++;
          }

          std::pair< ModelTPtr, int > pp = std::make_pair(descr_model.model,descr_model.view_id);
          model_view_id_to_flann_models_[pp] = idx_flann_models_for_this_view;

        }
      }
    }

    specificLoadFeaturesAndCreateFLANN();
    std::cout << "Number of features:" << flann_models_.size () << std::endl;

    /*if (use_codebook_)
    {
      //testing fastRNN...
      std::string filename_codebook_data;
      std::stringstream f;
      f << cb_flann_index_fn_ << "_data.txt";
      filename_codebook_data = f.str ();

      fast_rnn::fastRNN frnn;
      if (frnn.load_data (filename_codebook_data) > 0)
      {

      }
      else
      {
        std::cout << "Generating codebook... this might take several minutes..." << std::endl;
        std::vector<std::vector<double> > fast_rnn_data;
        for (size_t i = 0; i < flann_models_.size (); i++)
        {
          std::vector<double> newOne = std::vector<double> (flann_models_[i].descr.begin (), flann_models_[i].descr.end ());
          fast_rnn_data.push_back (newOne);
        }
        frnn.setData (fast_rnn_data);

        {
          pcl::ScopeTime t ("fast rnn");
          frnn.do_clustering ();
        }

        frnn.save_data(filename_codebook_data);
      }

      std::vector<std::vector<float> > cluster_centers;
      std::vector<unsigned int> assignments;
      frnn.getCentersAndAssignments (cluster_centers, assignments);

      //take
      codebook_models_.resize (cluster_centers.size ());
      std::cout << "Number of codewords:" << codebook_models_.size () << std::endl;
      for (size_t i = 0; i < codebook_models_.size (); i++)
      {
        codebook_models_[i].descr = cluster_centers[i];
        codebook_models_[i].cluster_idx_ = i;
      }

      int higher_than_thres = 0;
      for (size_t i = 0; i < assignments.size (); i++)
      {
        //boost::shared_ptr<flann_model> model_ptr;
        //model_ptr.reset(&flann_models_[i]);
        assert(assignments[i] < codebook_models_.size());
        codebook_models_[assignments[i]].clustered_indices_to_flann_models_.push_back (i);

        float diff = 0.f;
        for(size_t k=0; k < codebook_models_[assignments[i]].descr.size(); k++)
        {
          diff += std::pow(codebook_models_[assignments[i]].descr[k] - flann_models_[i].descr[k],2);
        }

        diff = sqrt(diff);

        float thres = 0.2f;
        if(diff > thres) {
          PCL_WARN("Descriptor distance to codeword center higher than %f ---- %f\n", thres, diff);
          higher_than_thres++;
        }
      }

//      int valid = 0;
//      for (size_t i = 0; i < codebook_models_.size (); i++) {
//        if(codebook_models_[i].clustered_indices_to_flann_models_.size() < 50) {
//          codebook_models_[valid] = codebook_models_[i];
//          valid++;
//        }
//        else
//        {
//          PCL_WARN("Too many descriptors in codeword... %d\n", codebook_models_[i].clustered_indices_to_flann_models_.size());
//        }
//      }

//      codebook_models_.resize(valid);

      PCL_INFO("Farther away than threshold %d\n", higher_than_thres);
    }*/

    std::string filename;
    if(codebook_models_.size() > 0) {
      convertToFLANN<codebook_model> (codebook_models_, flann_data_);
      filename = cb_flann_index_fn_;
    } else {
      convertToFLANN<flann_model> (flann_models_, flann_data_);
      filename = flann_index_fn_;
    }

#if defined (_WIN32)
    flann_index_ = new flann::Index<DistT> (flann_data_, flann::KDTreeIndexParams (4));
    flann_index_->buildIndex ();
#else
    bf::path idx_file_path = filename;
    if(bf::exists(idx_file_path)) {
      pcl::ScopeTime t("Loading flann index");
      flann_index_ = new flann::Index<DistT> (flann_data_, flann::SavedIndexParams (filename));
    } else {
      pcl::ScopeTime t("Building and saving flann index");
      flann_index_ = new flann::Index<DistT> (flann_data_, flann::KDTreeIndexParams (4));
      flann_index_->buildIndex ();
      flann_index_->save (filename);
    }
#endif

    //once the descriptors in flann_models_ have benn converted to flann_data_, i can delete them
    for(size_t i=0; i < flann_models_.size(); i++)
      flann_models_[i].descr.clear();

    for(size_t i=0; i < codebook_models_.size(); i++)
      codebook_models_[i].descr.clear();

    std::cout << "End load feature and create flann" << std::endl;
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::nearestKSearch (flann::Index<DistT> * index,
                                                                                                     /*float * descr, int descr_size*/
                                                                                                     flann::Matrix<float> & p, int k,
                                                                                                     flann::Matrix<int> &indices,
                                                                                                     flann::Matrix<float> &distances)
  {
    index->knnSearch (p, indices, distances, k, flann::SearchParams (kdtree_splits_));
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::initialize (bool force_retrain)
  {
    boost::shared_ptr < std::vector<ModelTPtr> > models;

    if(search_model_.compare("") == 0) {
      models = source_->getModels ();
    } else {
      models = source_->getModels (search_model_);
      //reset cache and flann structures
      if(flann_index_ != 0)
        delete flann_index_;

      flann_models_.clear();
      poses_cache_.clear();
      keypoints_cache_.clear();
      normals_cache_.clear();
    }

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
      std::cout << models->at (i)->class_ << " " << models->at (i)->id_ << std::endl;

      if (!source_->modelAlreadyTrained (*models->at (i), training_dir_, descr_name_))
      {
        std::cout << "Model not trained..." << models->at (i)->views_->size () << std::endl;
        if(!source_->getLoadIntoMemory())
          source_->loadInMemorySpecificModel(training_dir_, *(models->at (i)));

        std::cout << "number of views:" << models->at (i)->views_->size () << " " << source_->getLoadIntoMemory() << std::endl;
        for (size_t v = 0; v < models->at (i)->views_->size (); v++)
        {
          PointInTPtr processed (new pcl::PointCloud<PointInT>);
          typename pcl::PointCloud<FeatureT>::Ptr signatures (new pcl::PointCloud<FeatureT> ());
          PointInTPtr keypoints_pointcloud;

          if(models->at(i)->indices_)
          {
              std::cout << models->at(i)->indices_->at (v).indices.size() << " " << estimator_->acceptsIndices() << std::endl;
          }
          else
          {
              std::cout << "indices are not defined";
          }

          if(models->at(i)->indices_ && (models->at(i)->indices_->at (v).indices.size() > 0)
                                     && estimator_->acceptsIndices())
          {
            std::cout << "accepts indices:" << estimator_->acceptsIndices() << " size:" << models->at(i)->indices_->at (v).indices.size() << std::endl;
            estimator_->setIndices(models->at(i)->indices_->at (v));
          }
          else
          {
              std::cout << "accepts indices:" << estimator_->acceptsIndices() << std::endl;
          }

          bool success = estimator_->estimate (models->at (i)->views_->at (v), processed, keypoints_pointcloud, signatures);
          std::cout << "success:" << success << std::endl;

          if (success)
          {
            std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);

            bf::path desc_dir = path;
            if (!bf::exists (desc_dir))
              bf::create_directory (desc_dir);

            std::stringstream path_view;
            path_view << path << "/view_" << v << ".pcd";
            pcl::io::savePCDFileBinary (path_view.str (), *processed);

            std::stringstream path_pose;
            path_pose << path << "/pose_" << v << ".txt";
            PersistenceUtils::writeMatrixToFile (path_pose.str (), models->at (i)->poses_->at (v));

            if(v < models->at (i)->self_occlusions_->size()) {
              std::stringstream path_entropy;
              path_entropy << path << "/entropy_" << v << ".txt";
              PersistenceUtils::writeFloatToFile (path_entropy.str (), models->at (i)->self_occlusions_->at (v));
            }

            //save keypoints and signatures to disk
            std::stringstream keypoints_sstr;
            keypoints_sstr << path << "/keypoint_indices_" << v << ".pcd";
            pcl::io::savePCDFileBinary (keypoints_sstr.str (), *keypoints_pointcloud);

            std::stringstream path_descriptor;
            path_descriptor << path << "/descriptor_" << v << ".pcd";
            pcl::io::savePCDFileBinary (path_descriptor.str (), *signatures);

            if(estimator_->needNormals())
            {
              PCL_WARN("estimator needs normals\n");

              //save normals to disk...
              std::stringstream normals_sstr;
              normals_sstr << path << "/normals_" << v << ".pcd";
              pcl::PointCloud<pcl::Normal>::Ptr normals;
              estimator_->getNormals(normals);
              pcl::io::savePCDFileBinary (normals_sstr.str (), *normals);

              //use the keypoints point cloud to find the NN in processed, so we can have indices between keypoints and processed,normals
              pcl::octree::OctreePointCloudSearch<PointInT> octree (0.005);
              octree.setInputCloud (processed);
              octree.addPointsFromInputCloud ();

              std::vector<int> pointIdxNKNSearch;
              std::vector<float> pointNKNSquaredDistance;
              pcl::PointCloud<IndexPoint> keypoints_indices_to_processed;
              for(size_t j=0; j < keypoints_pointcloud->points.size(); j++)
              {
                if (octree.nearestKSearch (keypoints_pointcloud->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                {
                  IndexPoint ip;
                  ip.idx = pointIdxNKNSearch[0];
                  keypoints_indices_to_processed.push_back(ip);
                }
              }

              std::stringstream kitp_sstr;
              kitp_sstr << path << "/keypoints_indices_to_processed_and_normals_" << v << ".pcd";
              pcl::io::savePCDFileBinary (kitp_sstr.str (), keypoints_indices_to_processed);
            }
            else
            {
              PCL_WARN("estimator does not need normals\n");

              //the cg alg require normals but the estimator did not compute them, compute them now
              if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
              {
                PCL_ERROR("Need to compute normals due to the cg algorithm\n");
                boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > normal_estimator;
                normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointInT, pcl::Normal>);
                normal_estimator->setCMR (false);
                normal_estimator->setDoVoxelGrid (false);
                normal_estimator->setRemoveOutliers (false);
                normal_estimator->setValuesForCMRFalse (0.003f, 0.02f);
                normal_estimator->setForceUnorganized(true);

                if(estimator_->acceptsIndices())
                {
                  pcl::PointIndices indices;
                  estimator_->getKeypointIndices(indices);
                  normal_estimator->setIndices(indices.indices);
                }
                PointInTPtr processed (new pcl::PointCloud<PointInT>);
                pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
                normal_estimator->estimate (models->at (i)->views_->at (v), processed, normals);

                std::stringstream normals_sstr;
                normals_sstr << path << "/normals_" << v << ".pcd";
                pcl::io::savePCDFileBinary (normals_sstr.str (), *normals);

                //use the keypoints point cloud to find the NN in processed, so we can have indices between keypoints and processed,normals
                pcl::octree::OctreePointCloudSearch<PointInT> octree (0.005);
                octree.setInputCloud (processed);
                octree.addPointsFromInputCloud ();

                std::vector<int> pointIdxNKNSearch;
                std::vector<float> pointNKNSquaredDistance;
                pcl::PointCloud<IndexPoint> keypoints_indices_to_processed;
                for(size_t j=0; j < keypoints_pointcloud->points.size(); j++)
                {
                  if (octree.nearestKSearch (keypoints_pointcloud->points[j], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                  {
                    IndexPoint ip;
                    ip.idx = pointIdxNKNSearch[0];
                    keypoints_indices_to_processed.push_back(ip);
                  }
                }

                std::stringstream kitp_sstr;
                kitp_sstr << path << "/keypoints_indices_to_processed_and_normals_" << v << ".pcd";
                pcl::io::savePCDFileBinary (kitp_sstr.str (), keypoints_indices_to_processed);

                //should we take the normals from the model?
              }
              else
              {
                  PCL_WARN("gc algorithm does not need normals\n");
              }
            }
          }
        }

        if(!source_->getLoadIntoMemory())
          models->at (i)->views_->clear();

      } else {
        std::cout << "Model already trained..." << std::endl;
        //there is no need to keep the views in memory once the model has been trained
        models->at (i)->views_->clear();
      }
    }

    loadFeaturesAndCreateFLANN ();

   if(ICP_iterations_ > 0 && icp_type_ == 1)
     source_->createVoxelGridAndDistanceTransform(VOXEL_SIZE_ICP_);

  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::recognize ()
  {

    models_.reset (new std::vector<ModelTPtr>);
    transforms_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

    PointInTPtr processed;
    typename pcl::PointCloud<FeatureT>::Ptr signatures (new pcl::PointCloud<FeatureT> ());
    PointInTPtr keypoints_pointcloud;

    {
      pcl::ScopeTime t("Compute keypoints and features");
      if (signatures_ != 0 && processed_ != 0 && (signatures_->size () == keypoints_input_->points.size ()))
      {
        keypoints_pointcloud = keypoints_input_;
        signatures = signatures_;
        processed = processed_;
        std::cout << "Using the ISPK ..." << std::endl;
      }
      else
      {
        processed.reset( (new pcl::PointCloud<PointInT>));
        if (indices_.size () > 0)
        {
          if(estimator_->acceptsIndices())
          {
            estimator_->setIndices(indices_);
            estimator_->estimate (input_, processed, keypoints_pointcloud, signatures);
          }
          else
          {
            PointInTPtr sub_input (new pcl::PointCloud<PointInT>);
            pcl::copyPointCloud (*input_, indices_, *sub_input);
            estimator_->estimate (sub_input, processed, keypoints_pointcloud, signatures);
          }
        }
        else
        {
          estimator_->estimate (input_, processed, keypoints_pointcloud, signatures);
        }

        processed_ = processed;
        estimator_->getKeypointIndices(keypoint_indices_);
      }
      std::cout << "Number of keypoints:" << keypoints_pointcloud->points.size () << std::endl;
    }

    keypoint_cloud_ = keypoints_pointcloud;

    int size_feat = sizeof(signatures->points[0].histogram) / sizeof(float);

    //feature matching and object hypotheses
    typename std::map<std::string, ObjectHypothesis<PointInT> > object_hypotheses;
    {
      //double time_nn = 0;
      double time_pk = 0;

      flann::Matrix<int> indices;
      flann::Matrix<float> distances;
      size_t k = knn_;
      distances = flann::Matrix<float> (new float[k], 1, k);
      indices = flann::Matrix<int> (new int[k], 1, k);

      Eigen::Matrix4f homMatrixPose;
      typename pcl::PointCloud<PointInT>::Ptr keypoints (new pcl::PointCloud<PointInT> ());
      pcl::PointCloud<pcl::Normal>::Ptr normals_model_view_cloud (new pcl::PointCloud<pcl::Normal> ());
      pcl::PointCloud<IndexPoint>::Ptr indices_from_keypoints_to_normals (new pcl::PointCloud<IndexPoint> ());
      PointInT model_keypoint;
      pcl::Normal model_view_normal;

      flann::Matrix<float> p = flann::Matrix<float> (new float[size_feat], 1, size_feat);

      pcl::ScopeTime t("Generating object hypotheses");
      for (size_t idx = 0; idx < signatures->points.size (); idx++)
      {
        {
          //boost::posix_time::ptime start_time (boost::posix_time::microsec_clock::local_time ());
          //nearestKSearch (flann_index_, signatures->points[idx].histogram, size_feat, k, indices, distances);
          memcpy (&p.ptr ()[0], &signatures->points[idx].histogram[0], size_feat * sizeof(float));
          nearestKSearch (flann_index_, p, k, indices, distances);
          //boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
          //time_nn += (end_time - start_time).total_microseconds ();
        }

        int dist = distances[0][0];
        if(dist > max_descriptor_distance_)
            continue;

        std::vector<int> flann_models_indices;
        std::vector<float> model_distances;
        if(use_codebook_) {
          //indices[0][0] points to the codebook entry
          int cb_entry = indices[0][0];
//          int cb_entries = static_cast<int>(codebook_models_[cb_entry].clustered_indices_to_flann_models_.size());
          //std::cout << "Codebook entries:" << cb_entries << " " << cb_entry << " " << codebook_models_.size() << std::endl;
          flann_models_indices = codebook_models_[cb_entry].clustered_indices_to_flann_models_;
          model_distances.reserve(flann_models_indices.size());
          for(size_t ii=0; ii < flann_models_indices.size(); ii++)
          {
              model_distances.push_back(dist);
          }
        } else {
          flann_models_indices.reserve(k);
          model_distances.reserve(k);
          for(size_t ii=0; ii < k; ii++)
          {
            flann_models_indices.push_back(indices[0][ii]);
            model_distances.push_back(distances[0][ii]);
          }
        }

        std::vector<PointInT> model_keypoints_for_scene_keypoint;
        std::vector<std::string> model_id_for_scene_keypoint;

        for (size_t ii = 0; ii < flann_models_indices.size(); ii++)
        {
          {
            boost::posix_time::ptime start_time (boost::posix_time::microsec_clock::local_time ());
            getPose (*(flann_models_.at (flann_models_indices[ii]).model), flann_models_.at (flann_models_indices[ii]).view_id, homMatrixPose);
            getKeypoints (*(flann_models_.at (flann_models_indices[ii]).model), flann_models_.at (flann_models_indices[ii]).view_id, keypoints);

            boost::posix_time::ptime end_time = boost::posix_time::microsec_clock::local_time ();
            time_pk += (end_time - start_time).total_microseconds ();
          }

          //assert(normals_model_view_cloud->points.size() == processed->points.size());
          //homMatrixPose should go from model to view (inverse from view to model)
          model_keypoint.getVector4fMap () = homMatrixPose.inverse () * keypoints->points[flann_models_.at (flann_models_indices[ii]).keypoint_id].getVector4fMap ();
          if(model_keypoints_for_scene_keypoint.size() != 0)
          {
            bool found = false;
            for(size_t kk=0; kk < model_keypoints_for_scene_keypoint.size(); kk++)
            {

              if(model_id_for_scene_keypoint[kk].compare(flann_models_.at (flann_models_indices[kk]).model->id_) == 0)
              {
                if( (model_keypoints_for_scene_keypoint[kk].getVector3fMap() - model_keypoint.getVector3fMap()).squaredNorm() < distance_same_keypoint_)
                {
                  found = true;
                  break;
                }
              }
            }

            if(found)
              continue;
          }
          else
          {
            model_keypoints_for_scene_keypoint.push_back(model_keypoint);
            model_id_for_scene_keypoint.push_back(flann_models_.at (flann_models_indices[ii]).model->id_);
          }

          if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
          {
            getNormals (*(flann_models_.at (flann_models_indices[ii]).model), flann_models_.at (flann_models_indices[ii]).view_id, normals_model_view_cloud);
            getIndicesToProcessedAndNormals (*(flann_models_.at (flann_models_indices[ii]).model), flann_models_.at (flann_models_indices[ii]).view_id, indices_from_keypoints_to_normals);
            int normal_idx = indices_from_keypoints_to_normals->points[flann_models_.at (flann_models_indices[ii]).keypoint_id].idx;
            model_view_normal.getNormalVector3fMap () = homMatrixPose.block<3,3>(0,0).inverse () * normals_model_view_cloud->points[normal_idx].getNormalVector3fMap ();
          }

          float dist = model_distances[ii];

          typename std::map<std::string, ObjectHypothesis<PointInT> >::iterator it_map;
          if ((it_map = object_hypotheses.find (flann_models_.at (flann_models_indices[ii]).model->id_)) != object_hypotheses.end ())
          {
            (*it_map).second.correspondences_pointcloud->points.push_back(model_keypoint);
            //if(estimator_->needNormals())
            if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
            {
              (*it_map).second.normals_pointcloud->points.push_back(model_view_normal);
            }

            (*(*it_map).second.correspondences_to_inputcloud).push_back(pcl::Correspondence ((*it_map).second.correspondences_pointcloud->points.size()-1, static_cast<int> (idx), dist));
//            (*(*it_map).second.feature_distances_).push_back(dist);
            (*it_map).second.indices_to_flann_models_.push_back(flann_models_indices[ii]);
//            (*it_map).second.num_corr_++;
          }
          else
          {
            //create object hypothesis
            ObjectHypothesis<PointInT> oh;
            oh.correspondences_pointcloud.reset (new pcl::PointCloud<PointInT> ());
            //if(estimator_->needNormals())
            if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
            {
              oh.normals_pointcloud.reset (new pcl::PointCloud<pcl::Normal> ());
              oh.normals_pointcloud->points.resize (1);
              oh.normals_pointcloud->points.reserve (signatures->points.size ());
              oh.normals_pointcloud->points[0] = model_view_normal;
            }

//            oh.feature_distances_.reset (new std::vector<float>);
            oh.correspondences_to_inputcloud.reset (new pcl::Correspondences ());

            oh.correspondences_pointcloud->points.resize (1);
            oh.correspondences_to_inputcloud->resize (1);
//            oh.feature_distances_->resize (1);
            oh.indices_to_flann_models_.resize(1);

            oh.correspondences_pointcloud->points.reserve (signatures->points.size ());
            oh.correspondences_to_inputcloud->reserve (signatures->points.size ());
//            oh.feature_distances_->reserve (signatures->points.size ());
            oh.indices_to_flann_models_.reserve(signatures->points.size ());

            oh.correspondences_pointcloud->points[0] = model_keypoint;
            oh.correspondences_to_inputcloud->at (0) = pcl::Correspondence (0, static_cast<int> (idx), dist);
//            oh.feature_distances_->at (0) = dist;
            oh.indices_to_flann_models_[0] = flann_models_indices[ii];
//            oh.num_corr_ = 1;
            oh.model_ = flann_models_.at (flann_models_indices[ii]).model;

            object_hypotheses[oh.model_->id_] = oh;
          }
        }
      }

      delete[] indices.ptr ();
      delete[] distances.ptr ();
      delete[] p.ptr ();

      typename std::map<std::string, ObjectHypothesis<PointInT> >::iterator it_map;
      for (it_map = object_hypotheses.begin(); it_map != object_hypotheses.end (); it_map++) {

//          std::cout << "Showing local recognizer keypoints for " << it_map->first << std::endl;
//                     pcl::visualization::PCLVisualizer viewer("Keypoint Viewer for local recognizer");
//                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//                     ConstPointInTPtr model_cloud_templated = it_map->second.model_->getAssembled (0.003f);
//                     pcl::copyPointCloud(*model_cloud_templated, *model_cloud);
//                     pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler_rgb (model_cloud);
//                     viewer.addPointCloud<pcl::PointXYZRGB>(model_cloud, handler_rgb, "scene");

//                     pcl::PointCloud<pcl::PointXYZRGB>::Ptr pKeypointCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//                     PointInTPtr pKeypointCloudTemplated = it_map->second.correspondences_pointcloud;
//                     pcl::copyPointCloud(*pKeypointCloudTemplated, *pKeypointCloud);
//                     viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pKeypointCloud, it_map->second.normals_pointcloud, 5, 0.04);
//                     viewer.spin();

        ObjectHypothesis<PointInT> oh = (*it_map).second;
        size_t num_corr = oh.correspondences_to_inputcloud->size();
        oh.correspondences_pointcloud->points.resize(num_corr);
        if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
          oh.normals_pointcloud->points.resize(num_corr);

        oh.correspondences_to_inputcloud->resize(num_corr);
//        oh.feature_distances_->resize(num_corr);
      }

      //std::cout << "Time nearest searches:" << time_nn / 1000.f << " ms" << std::endl;
      std::cout << "Time pose/keypoint get:" << time_pk / 1000.f << " ms" << std::endl;
    }

    if(save_hypotheses_)
    {
      pcl::ScopeTime t("Saving hypotheses...");

      if(correspondence_distance_constant_weight_ != 1.f)
      {
          PCL_WARN("correspondence_distance_constant_weight_ activated! %f", correspondence_distance_constant_weight_);
          //go through the object hypotheses and multiply the correspondences distances by the weight
          //this is done to favour correspondences from different pipelines that are more reliable than other (SIFT and SHOT corr. simultaneously fed into CG)
          typename std::map<std::string, ObjectHypothesis<PointInT> >::iterator it_map;
          for (it_map = object_hypotheses.begin (); it_map != object_hypotheses.end (); it_map++)
          {
              for(size_t k=0; k < (*it_map).second.correspondences_to_inputcloud->size(); k++)
              {
                  (*it_map).second.correspondences_to_inputcloud->at(k).distance *= correspondence_distance_constant_weight_;
              }
          }
      }

      saved_object_hypotheses_ = object_hypotheses;
    }
    else
    {
      typename std::map<std::string, ObjectHypothesis<PointInT> >::iterator it_map;

      {
        pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>);
        if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
        {
          pcl::PointCloud<pcl::Normal>::Ptr all_scene_normals;

          if(estimator_->needNormals())
          {
            estimator_->getNormals(all_scene_normals);
          }
          else
          {
            //compute them...
            PCL_ERROR("Need to compute normals due to the cg algorithm\n");
            all_scene_normals.reset(new pcl::PointCloud<pcl::Normal>);
            boost::shared_ptr<faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointInT, pcl::Normal> > normal_estimator;
            normal_estimator.reset (new faat_pcl::rec_3d_framework::PreProcessorAndNormalEstimator<PointInT, pcl::Normal>);
            normal_estimator->setCMR (false);
            normal_estimator->setDoVoxelGrid (false);
            normal_estimator->setRemoveOutliers (false);
            normal_estimator->setValuesForCMRFalse (0.003f, 0.02f);
            normal_estimator->setForceUnorganized(true);
            PointInTPtr processed (new pcl::PointCloud<PointInT>);
            normal_estimator->estimate (input_, processed, all_scene_normals);
          }
          std::vector<int> correct_indices;
          getIndicesFromCloud<PointInT>(processed, keypoints_pointcloud, correct_indices);
          pcl::copyPointCloud(*all_scene_normals, correct_indices, *scene_normals);
        }

        prepareSpecificCG(processed, keypoints_pointcloud);
        pcl::ScopeTime t("Geometric verification, RANSAC and transform estimation");
        for (it_map = object_hypotheses.begin (); it_map != object_hypotheses.end (); it_map++)
        {
          std::vector < pcl::Correspondences > corresp_clusters;
          cg_algorithm_->setSceneCloud (keypoints_pointcloud);
          cg_algorithm_->setInputCloud ((*it_map).second.correspondences_pointcloud);

          if((cg_algorithm_ && cg_algorithm_->getRequiresNormals()) || save_hypotheses_)
          {
            std::cout << "CG alg requires normals..." << ((*it_map).second.normals_pointcloud)->points.size() << " " << (scene_normals)->points.size() << std::endl;
            cg_algorithm_->setInputAndSceneNormals((*it_map).second.normals_pointcloud, scene_normals);
          }
          //we need to pass the keypoints_pointcloud and the specific object hypothesis
          specificCG(processed, keypoints_pointcloud, it_map->second);
          cg_algorithm_->setModelSceneCorrespondences ((*it_map).second.correspondences_to_inputcloud);
          cg_algorithm_->cluster (corresp_clusters);

          std::cout << "Instances:" << corresp_clusters.size () << " Total correspondences:" << (*it_map).second.correspondences_to_inputcloud->size () << " " << it_map->first << std::endl;
          std::vector<bool> good_indices_for_hypothesis (corresp_clusters.size (), true);

          if (threshold_accept_model_hypothesis_ < 1.f)
          {
            //sort the hypotheses for each model according to their correspondences and take those that are threshold_accept_model_hypothesis_ over the max cardinality
            int max_cardinality = -1;
            for (size_t i = 0; i < corresp_clusters.size (); i++)
            {
              //std::cout <<  (corresp_clusters[i]).size() << " -- " << (*(*it_map).second.correspondences_to_inputcloud).size() << std::endl;
              if (max_cardinality < static_cast<int> (corresp_clusters[i].size ()))
              {
                max_cardinality = static_cast<int> (corresp_clusters[i].size ());
              }
            }

            for (size_t i = 0; i < corresp_clusters.size (); i++)
            {
              if (static_cast<float> ((corresp_clusters[i]).size ()) < (threshold_accept_model_hypothesis_ * static_cast<float> (max_cardinality)))
              {
                good_indices_for_hypothesis[i] = false;
              }
            }
          }

          int keeping = 0;
          for (size_t i = 0; i < corresp_clusters.size (); i++)
          {

            if (!good_indices_for_hypothesis[i])
              continue;

            //drawCorrespondences (processed, it_map->second, keypoints_pointcloud, corresp_clusters[i]);

            Eigen::Matrix4f best_trans;
            typename pcl::registration::TransformationEstimationSVD < PointInT, PointInT > t_est;
            t_est.estimateRigidTransformation (*(*it_map).second.correspondences_pointcloud, *keypoints_pointcloud, corresp_clusters[i], best_trans);

            models_->push_back ((*it_map).second.model_);
            transforms_->push_back (best_trans);

            keeping++;
          }

          std::cout << "kept " << keeping << " out of " << corresp_clusters.size () << std::endl;
        }

        clearSpecificCG();
      }

      std::cout << "Number of hypotheses:" << models_->size() << std::endl;

      if (ICP_iterations_ > 0 || hv_algorithm_) {
        //Prepare scene and model clouds for the pose refinement step
        source_->voxelizeAllModels (VOXEL_SIZE_ICP_);
      }

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

      if (hv_algorithm_ && (models_->size () > 0))
      {
        hypothesisVerification();
      }
    }
  }

template<template<class > class Distance, typename PointInT, typename FeatureT>
void
faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getView (ModelT & model, int view_id, PointInTPtr & view)
{
  view.reset (new pcl::PointCloud<PointInT>);
  std::stringstream dir;
  std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
  dir << path << "/view_" << view_id << ".pcd";
  pcl::io::loadPCDFile (dir.str (), *view);

}

template<template<class > class Distance, typename PointInT, typename FeatureT>
void
faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getIndicesToProcessedAndNormals (ModelT & model,
                                                                                                                       int view_id,
                                                                                                                       pcl::PointCloud<IndexPoint>::Ptr & index_cloud)
{

  if (use_cache_)
   {
   typedef std::pair<std::string, int> mv_pair;
   mv_pair pair_model_view = std::make_pair (model.id_, view_id);

   std::map<mv_pair, pcl::PointCloud<IndexPoint>::Ptr, std::less<mv_pair>,
                     Eigen::aligned_allocator<std::pair<mv_pair, pcl::PointCloud<IndexPoint>::Ptr > > >::iterator it =
                         idxpoint_cache_.find (pair_model_view);

   if (it != idxpoint_cache_.end ())
   {
     index_cloud = it->second;
     return;
   }

  }

  std::stringstream dir;
  std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
  dir << path << "/keypoints_indices_to_processed_and_normals_" << view_id << ".pcd";

  pcl::io::loadPCDFile (dir.str (), *index_cloud);
}

template<template<class > class Distance, typename PointInT, typename FeatureT>
void
faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getNormals (ModelT & model, int view_id,
                                                                                                pcl::PointCloud<pcl::Normal>::Ptr & normals_cloud)
{

  if (use_cache_)
   {
   typedef std::pair<std::string, int> mv_pair;
   mv_pair pair_model_view = std::make_pair (model.id_, view_id);

   std::map<mv_pair, pcl::PointCloud<pcl::Normal>::Ptr, std::less<mv_pair>,
                     Eigen::aligned_allocator<std::pair<mv_pair, pcl::PointCloud<pcl::Normal>::Ptr > > >::iterator it =
       normals_cache_.find (pair_model_view);

   if (it != normals_cache_.end ())
   {
     normals_cloud = it->second;
     return;
   }

  }

  std::stringstream dir;
  std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
  dir << path << "/normals_" << view_id << ".pcd";

  pcl::io::loadPCDFile (dir.str (), *normals_cloud);
}


template<template<class > class Distance, typename PointInT, typename FeatureT>
  void
  faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix)
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
  void
  faat_pcl::rec_3d_framework::LocalRecognitionPipeline<Distance, PointInT, FeatureT>::getKeypoints (
                                                                                               ModelT & model,
                                                                                               int view_id,
                                                                                               typename pcl::PointCloud<PointInT>::Ptr & keypoints_cloud)
  {

    if (use_cache_)
    {
      std::pair<std::string, int> pair_model_view = std::make_pair (model.id_, view_id);
      typename std::map<std::pair<std::string, int>, PointInTPtr>::iterator it = keypoints_cache_.find (pair_model_view);

      if (it != keypoints_cache_.end ())
      {
        keypoints_cloud = it->second;
        return;
      }

    }

    std::stringstream dir;
    std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
    dir << path << "/keypoint_indices_" << view_id << ".pcd";

    pcl::io::loadPCDFile (dir.str (), *keypoints_cloud);
  }
