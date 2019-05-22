/*
 * or_evaluator.hpp
 *
 *  Created on: Mar 12, 2013
 *      Author: aitor
 */

#ifndef OR_EVALUATOR_HPP_
#define OR_EVALUATOR_HPP_

#include "or_evaluator.h"
#include "pcl/octree/octree.h"
#include "pcl/registration/icp.h"
#include <pcl/common/angles.h>
#include <v4r/ORUtils/filesystem_utils.h>

template<typename ModelPointT, typename SceneId>
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::OREvaluator ()
{
  gt_data_loaded_ = false;
  scene_file_extension_ = "pcd";
  model_file_extension_ = "ply";
  check_pose_ = false;
  max_centroid_diff_ = 0.01f;
  check_rotation_ = false;
  max_rotation_ = 180;
  replace_model_ext_ = true;
  max_occlusion_ = 0.9f;
  use_max_occlusion_ = false;
}

template<typename ModelPointT, typename SceneId>
void faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::
    computeOcclusionValues(bool only_if_not_exist, bool do_icp_)
{
    std::cout << "computeOcclusionValues" << std::endl;
    if(!checkLoaded())
        return;

    //pcl::visualization::PCLVisualizer vis("occlusion values");

    std::cout << "computeOcclusionValues" << std::endl;

    bool do_icp = do_icp_;
    float model_res = 0.001f;
    float inlier = 0.003f;
    float max_correspondence_distance_ = 0.005f;

    typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
    for(gt_data_main_iterator = gt_data_.begin(); gt_data_main_iterator != gt_data_.end(); gt_data_main_iterator++)
    {
        //iterate over the scene and add models
        typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
        scene_map_iterator = gt_data_main_iterator->second.begin();

        pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
        std::stringstream scene_path;
        scene_path << scenes_dir_ << "/" << gt_data_main_iterator->first << "." << scene_file_extension_;
        std::cout << scene_path.str() << std::endl;
        pcl::io::loadPCDFile(scene_path.str().c_str(), *scene);

        while (scene_map_iterator != gt_data_main_iterator->second.end ())
        {

            std::string model_name = scene_map_iterator->first;
            std::cout << model_name << " " << replace_model_ext_ << std::endl;
            std::stringstream model_ext;
            model_ext << "." << model_file_extension_;
            boost::replace_all (model_name, model_ext.str (), "");
            std::cout << model_name << " after replacement " << model_ext.str() << std::endl;

            for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
            {

                std::stringstream occ_file;
                occ_file << gt_dir_ << "/" << gt_data_main_iterator->first << "_occlusion_" << model_name << "_" << scene_map_iterator->second[i]->inst_  << ".txt";
                bf::path occ_file_path = occ_file.str();
                if(bf::exists(occ_file_path) && only_if_not_exist)
                    continue;

                ConstPointInTPtr model_cloud = scene_map_iterator->second[i]->model_->getAssembled (model_res);
                typename pcl::PointCloud<ModelPointT>::Ptr model_aligned_1 (new pcl::PointCloud<ModelPointT>);
                Eigen::Matrix4f trans = scene_map_iterator->second[i]->transform_;
                pcl::transformPointCloud (*model_cloud, *model_aligned_1, trans);

                pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned (new pcl::PointCloud<pcl::PointXYZ>);
                if(do_icp)
                {
                    pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned_2 (new pcl::PointCloud<pcl::PointXYZ>);
                    pcl::copyPointCloud(*model_aligned_1, *model_aligned_2);
                    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                    icp.setInputSource(model_aligned_2);
                    icp.setInputTarget(scene);
                    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
                    icp.setMaximumIterations(100);
                    icp.setEuclideanFitnessEpsilon(1e-12);
                    icp.align(*model_aligned);
                    Eigen::Matrix4f final_trans;
                    final_trans = icp.getFinalTransformation();
                    final_trans = final_trans * trans;
                }
                else
                {
                    pcl::copyPointCloud(*model_aligned_1, *model_aligned);
                }

                pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.001);
                octree.setInputCloud (scene);
                octree.addPointsFromInputCloud ();

                std::vector<int> pointIdxNKNSearch;
                std::vector<float> pointNKNSquaredDistance;

                int overlap = 0;
                std::vector<int> indices;
                for(size_t kk=0; kk < model_aligned->points.size(); kk++)
                {
                    pcl::PointXYZ p;
                    p.getVector3fMap() = model_aligned->points[kk].getVector3fMap();
                    if (octree.nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
                    {
                        float d = sqrt (pointNKNSquaredDistance[0]);
                        if (d < inlier)
                        {
                            overlap++;
                            indices.push_back(kk);
                        }
                    }
                }

                typename pcl::PointCloud<ModelPointT>::Ptr visible_model(new pcl::PointCloud<ModelPointT>);
                pcl::copyPointCloud(*model_aligned, indices, *visible_model);
                float occ = 1.f - overlap / static_cast<float>(model_aligned->points.size());
                scene_map_iterator->second[i]->occlusion_ = occ;
                std::cout << overlap << " " << model_aligned->points.size() << " " << occ << std::endl;

                //std::stringstream occ_file;
                //occ_file << gt_dir_ << "/" << gt_data_main_iterator->first << "_occlusion_" << model_name << "_" << scene_map_iterator->second[i]->inst_  << ".txt";
                std::cout << occ_file.str() << std::endl;
                writeFloatToFile(occ_file.str(), scene_map_iterator->second[i]->occlusion_);

                /*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_handler (model_aligned, 125, 125, 125);
                vis.addPointCloud(model_aligned, model_handler, "lolo");
                vis.addPointCloud(scene, "scene");
                {
                  pcl::visualization::PointCloudColorHandlerCustom<ModelPointT> model_handler (visible_model, 255, 0, 0);
                  vis.addPointCloud(visible_model, model_handler, "lolo_visible");
                }
                vis.spin();
                vis.removeAllPointClouds();*/

            }
            scene_map_iterator++;
        }

    }
}

template<typename ModelPointT, typename SceneId>
void faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::
    refinePoses()
{
    std::cout << "computeOcclusionValues" << std::endl;
    if(!checkLoaded())
        return;

    //pcl::visualization::PCLVisualizer vis("occlusion values");

    std::cout << "computeOcclusionValues" << std::endl;

    float model_res = 0.003f;
    float max_correspondence_distance_ = 0.02f;

    typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
    for(gt_data_main_iterator = gt_data_.begin(); gt_data_main_iterator != gt_data_.end(); gt_data_main_iterator++)
    {
        //iterate over the scene and add models
        typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
        scene_map_iterator = gt_data_main_iterator->second.begin();

        pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
        std::stringstream scene_path;
        scene_path << scenes_dir_ << "/" << gt_data_main_iterator->first << "." << scene_file_extension_;
        std::cout << scene_path.str() << std::endl;
        pcl::io::loadPCDFile(scene_path.str().c_str(), *scene);

        while (scene_map_iterator != gt_data_main_iterator->second.end ())
        {

            std::string model_name = scene_map_iterator->first;
            std::cout << model_name << " " << replace_model_ext_ << std::endl;
            std::stringstream model_ext;
            model_ext << "." << model_file_extension_;
            boost::replace_all (model_name, model_ext.str (), "");
            std::cout << model_name << " after replacement " << model_ext.str() << std::endl;

            for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
            {
                ConstPointInTPtr model_cloud = scene_map_iterator->second[i]->model_->getAssembled (model_res);
                typename pcl::PointCloud<ModelPointT>::Ptr model_aligned_1 (new pcl::PointCloud<ModelPointT>);
                Eigen::Matrix4f trans = scene_map_iterator->second[i]->transform_;
                pcl::transformPointCloud (*model_cloud, *model_aligned_1, trans);

                pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned_2 (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*model_aligned_1, *model_aligned_2);
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setInputSource(model_aligned_2);
                icp.setInputTarget(scene);
                icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
                icp.setMaximumIterations(100);
                icp.setEuclideanFitnessEpsilon(1e-12);
                icp.align(*model_aligned);
                Eigen::Matrix4f final_trans;
                final_trans = icp.getFinalTransformation();
                final_trans = final_trans * trans;

                scene_map_iterator->second[i]->transform_ = final_trans;
                //writeMatrixToFile(pose_file.str(),model_inst->transform_);
                /*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_handler (model_aligned, 125, 125, 125);
                vis.addPointCloud(model_aligned, model_handler, "lolo");
                vis.addPointCloud(scene, "scene");
                {
                  pcl::visualization::PointCloudColorHandlerCustom<ModelPointT> model_handler (visible_model, 255, 0, 0);
                  vis.addPointCloud(visible_model, model_handler, "lolo_visible");
                }
                vis.spin();
                vis.removeAllPointClouds();*/

            }
            scene_map_iterator++;
        }

    }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::loadGTData ()
{
  //get scenes in scene_dir
  std::vector<std::string> scene_files;
  std::vector<std::string> model_files;
  std::vector<std::string> model_files_wo_extension;

  {
    std::string start = "";
    std::string ext = scene_file_extension_;
    bf::path dir = scenes_dir_;
    getModelsInDirectory (dir, start, scene_files, ext);
    std::cout << "Number of scenes:" << scene_files.size() << std::endl;
    for(size_t i=0; i < scene_files.size(); i++)
    {
      std::cout << scene_files[i] << std::endl;
    }
  }

  //get models in models_dir
  {
    std::string start = "";
    std::string ext = model_file_extension_;
    bf::path dir = models_dir_;
    getModelsInDirectory (dir, start, model_files, ext);

    std::stringstream model_ext;
    model_ext << "." << model_file_extension_;
    model_files_wo_extension.resize(model_files.size());

    for(size_t i=0; i < model_files.size(); i++)
    {
      //Create a gt model...
      if(replace_model_ext_)
      {
        boost::replace_all (model_files[i], model_ext.str(), "");
        model_files_wo_extension[i] = model_files[i];
      }
      else
      {
        model_files_wo_extension[i] = model_files[i];
        boost::replace_all (model_files_wo_extension[i], model_ext.str(), "");
      }
      std::cout << model_files_wo_extension[i] << std::endl;
    }
  }

  std::cout << "Number of models:" << model_files.size() << std::endl;
  //load the necessary files from gt_dir based on the scenes and the models
  //fill gt_data_ appropiately

  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;

  for (size_t s = 0; s < scene_files.size (); s++)
  {


    if(ignore_list_.size() > 0)
    {
        //std::cout << "ignore list is not zero, looking for:" << scene_files[s] << std::endl;
        typename std::map<SceneId, bool>::iterator it_ignore_list;
        it_ignore_list = ignore_list_.find(scene_files[s]);
        if(it_ignore_list != ignore_list_.end())
        {
            continue;
        }
    }

    std::string scene_path (scene_files[s]);

    std::stringstream scene_ext;
    scene_ext << "." << scene_file_extension_;
    boost::replace_all (scene_path, scene_ext.str(), "");
    std::cout << scene_path << std::endl;

    std::string seq_id;
    std::vector < std::string > strs_2;
    boost::split (strs_2, scene_path, boost::is_any_of ("/\\"));
    seq_id = strs_2[strs_2.size() - 1];

    std::string dir_without_scene_name;

    if(strs_2.size() > 1)
    {
        for(size_t j=0; j < (strs_2.size() - 1); j++)
        {
            dir_without_scene_name.append(strs_2[j]);
            dir_without_scene_name.append("/");
        }
    }
    else
    {
        dir_without_scene_name = "";
    }

    for (size_t m = 0; m < model_files.size (); m++)
    {

      //std::stringstream ss;
      //ss << model_files[m] << "." << model_file_extension_;

      std::stringstream ss;
      ss << gt_dir_ << "/" << dir_without_scene_name;
      bf::path model_file_path;
      model_file_path = ss.str();
      std::vector<std::string> paths;
      std::stringstream pattern;
      pattern << seq_id << "_" <<  model_files_wo_extension[m] << "_.*.txt";
      //std::cout << model_file_path.string() << "-------" << pattern.str() << std::endl;
      faat_pcl::utils::getFilesInDirectory(model_file_path, paths, pattern.str());
      size_t max_inst = paths.size();

      for (size_t inst = 0; inst < max_inst; inst++)
      {
        std::stringstream pose_file;
        pose_file << gt_dir_ << "/" << scene_path << "_" << model_files_wo_extension[m] << "_" << inst << ".txt";

        bf::path pose_path = pose_file.str ();
        if (bf::exists (pose_path))
        {
          //std::cout << pose_file.str () << std::endl;
          GTModelTPtr model_inst(new GTModel<ModelPointT>);
          bool found = source_->getModelById(model_files[m], model_inst->model_);
          if(!found)
            std::cout << "Model id not found!!!" << model_files[m] << std::endl;

          readMatrixFromFile2(pose_file.str(), model_inst->transform_);
          //std::cout << model_inst->transform_ << std::endl;
          model_inst->inst_ = inst;

          std::stringstream occ_file;
          occ_file << gt_dir_ << "/" << scene_path << "_occlusion_" << model_files_wo_extension[m] << "_" << inst << ".txt";
          bf::path occ_path = occ_file.str ();
          if (bf::exists (occ_path))
          {
            std::cout << occ_file.str () << std::endl;
            readFloatFromFile(occ_file.str(), model_inst->occlusion_);
          }
          else
          {
            model_inst->occlusion_ = 0;
            //compute occlusion for this instance and save result
            /*ConstPointInTPtr model_cloud = model_inst->model_->getAssembled (0.001f);
            typename pcl::PointCloud<ModelPointT>::Ptr model_aligned_1 (new pcl::PointCloud<ModelPointT>);
            Eigen::Matrix4f trans;
            trans = model_inst->transform_;
            pcl::transformPointCloud (*model_cloud, *model_aligned_1, trans);

            pcl::PointCloud<pcl::PointXYZ>::Ptr scene(new pcl::PointCloud<pcl::PointXYZ>);
            std::stringstream scene_path;
            scene_path << scenes_dir_ << "/" << scene_files[s];
            pcl::io::loadPCDFile(scene_path.str().c_str(), *scene);

            pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned (new pcl::PointCloud<pcl::PointXYZ>);
            bool do_icp = true;
            if(do_icp)
            {
              pcl::PointCloud<pcl::PointXYZ>::Ptr model_aligned_2 (new pcl::PointCloud<pcl::PointXYZ>);
              pcl::copyPointCloud(*model_aligned_1, *model_aligned_2);
              pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
              icp.setInputSource(model_aligned_2);
              icp.setInputTarget(scene);
              icp.setMaxCorrespondenceDistance(0.0025f);
              icp.setMaximumIterations(100);
              icp.setEuclideanFitnessEpsilon(1e-12);
              icp.align(*model_aligned);
              Eigen::Matrix4f final_trans;
              final_trans = icp.getFinalTransformation();
              final_trans = final_trans * trans;
              model_inst->transform_ = final_trans;
            }
            else
            {
              pcl::copyPointCloud(*model_aligned_1, *model_aligned);
            }

            pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (0.001);
            octree.setInputCloud (scene);
            octree.addPointsFromInputCloud ();

            std::vector<int> pointIdxNKNSearch;
            std::vector<float> pointNKNSquaredDistance;

            float inlier = 0.003f;
            int overlap = 0;
            std::vector<int> indices;
            for(size_t kk=0; kk < model_aligned->points.size(); kk++)
            {
              pcl::PointXYZ p;
              p.getVector3fMap() = model_aligned->points[kk].getVector3fMap();
              if (octree.nearestKSearch (p, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
              {
                float d = sqrt (pointNKNSquaredDistance[0]);
                if (d < inlier)
                {
                  overlap++;
                  indices.push_back(kk);
                }
              }
            }

            typename pcl::PointCloud<ModelPointT>::Ptr visible_model(new pcl::PointCloud<ModelPointT>);
            pcl::copyPointCloud(*model_aligned, indices, *visible_model);
            float occ = 1.f - overlap / static_cast<float>(model_aligned->points.size());
            model_inst->occlusion_ = occ;
            std::cout << overlap << " " << model_aligned->points.size() << " " << occ << std::endl;

            //pcl::visualization::PCLVisualizer vis("");
//            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> model_handler (model_aligned, 125, 125, 125);
//            vis.addPointCloud(model_aligned, model_handler, "lolo");
//            vis.addPointCloud(scene, "scene");
//            {
//              pcl::visualization::PointCloudColorHandlerCustom<ModelPointT> model_handler (visible_model, 255, 0, 0);
//              vis.addPointCloud(visible_model, model_handler, "lolo_visible");
//            }
//            vis.spin();

            writeFloatToFile(occ_file.str(), model_inst->occlusion_);
            //writeMatrixToFile(pose_file.str(),model_inst->transform_);*/
          }

          gt_data_main_iterator = gt_data_.find(scene_path);
          if(gt_data_main_iterator == gt_data_.end())
          {
            std::map<std::string, std::vector<GTModelTPtr> > scene_map;
            std::vector<GTModelTPtr> vec;
            vec.push_back(model_inst);
            scene_map.insert(std::pair<std::string, std::vector<GTModelTPtr> >(model_files[m], vec));
            gt_data_.insert(std::pair<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >(scene_path, scene_map));
          }
          else
          {
            //already something in this scene...
            scene_map_iterator = gt_data_main_iterator->second.find(model_files[m]);
            if(scene_map_iterator == gt_data_main_iterator->second.end())
            {
              std::vector<GTModelTPtr> vec;
              vec.push_back(model_inst);
              gt_data_main_iterator->second.insert(std::pair<std::string, std::vector<GTModelTPtr> >(model_files[m], vec));
            }
            else
            {
              scene_map_iterator->second.push_back(model_inst);
            }
          }
        }
      }
    }
  }
  gt_data_loaded_ = true;
}

template<typename ModelPointT, typename SceneId>
int
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::countTotalNumberOfObjectsSequenceWise ()
{
  //get scenes in scene_dir
  std::vector<std::string> scene_files;
  std::vector<std::string> model_files;
  std::vector<std::string> model_files_wo_extension;

  {
    std::string start = "";
    std::string ext = scene_file_extension_;
    bf::path dir = scenes_dir_;
    getModelsInDirectory( dir, start, scene_files, ext);
    std::cout << "Number of scenes:" << scene_files.size() << std::endl;
    for(size_t i=0; i < scene_files.size(); i++)
    {
      std::cout << scene_files[i] << std::endl;
    }
  }

  //get models in models_dir
  {
    std::string start = "";
    std::string ext = model_file_extension_;
    bf::path dir = models_dir_;
    getModelsInDirectory (dir, start, model_files, ext);

    std::stringstream model_ext;
    model_ext << "." << model_file_extension_;
    model_files_wo_extension.resize(model_files.size());

    for(size_t i=0; i < model_files.size(); i++)
    {
      //Create a gt model...
      if(replace_model_ext_)
      {
        boost::replace_all (model_files[i], model_ext.str(), "");
        model_files_wo_extension[i] = model_files[i];
      }
      else
      {
        model_files_wo_extension[i] = model_files[i];
        boost::replace_all (model_files_wo_extension[i], model_ext.str(), "");
      }
      std::cout << model_files_wo_extension[i] << std::endl;
    }
  }

  std::cout << "Number of models:" << model_files.size() << std::endl;
  //load the necessary files from gt_dir based on the scenes and the models
  //fill gt_data_ appropiately

  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;

  std::map<std::string, int> count_per_sequence;
  std::map<std::string, int>::iterator it;
  int num_instances = 0;

  for (size_t s = 0; s < scene_files.size (); s++)
  {
    std::string scene_path (scene_files[s]);

    std::stringstream scene_ext;
    scene_ext << "." << scene_file_extension_;
    boost::replace_all (scene_path, scene_ext.str(), "");
    std::cout << scene_path << std::endl;

    std::string seq_id;
    std::vector < std::string > strs_2;
    boost::split (strs_2, scene_files[s], boost::is_any_of ("/\\"));
    seq_id = strs_2[0];

    for (size_t m = 0; m < model_files.size (); m++)
    {

      //std::stringstream ss;
      //ss << model_files[m] << "." << model_file_extension_;

      for (size_t inst = 0; inst < 10; inst++)
      {
        std::stringstream pose_file;
        pose_file << gt_dir_ << "/" << scene_path << "_" << model_files_wo_extension[m] << "_" << inst << ".txt";

        bf::path pose_path = pose_file.str ();
        if (bf::exists (pose_path))
        {
            std::stringstream idss;
            idss << seq_id << model_files_wo_extension[m] << "_" << inst;
            std::string id = idss.str();
            it = count_per_sequence.find(id);
            if(it == count_per_sequence.end())
            {
                count_per_sequence.insert(std::make_pair(id, 1));
            }

            num_instances++;
        }
      }
    }
  }

  std::cout << "Number of frames:" << scene_files.size() << std::endl;
  std::cout << "Number of instances:" << num_instances << std::endl;

  return (int)(count_per_sequence.size());
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::addRecognitionResults (std::vector<std::string> & model_ids, std::vector<Eigen::Matrix4f> & poses)
{

}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::saveRecognitionResults(std::string & out_dir)
{
    bf::path out_dir_path = out_dir;
    if(!bf::exists(out_dir_path))
    {
        bf::create_directory(out_dir_path);
    }

    std::cout << "number of recognition results:" << recognition_results_.size () << std::endl;
    //iterate over recognition results and compare to the corresponding gt_data_
    typename std::map<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > >::iterator results_it;
    typename std::map<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > >::iterator results_transforms_it;

    results_transforms_it = transforms_.begin();

    for (results_it = recognition_results_.begin (); results_it != recognition_results_.end (); results_it++, results_transforms_it++)
    {
      typename std::pair<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > > rr = *results_it;
      typename std::pair<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > > rtrans = *results_transforms_it;
      typename std::vector<ModelTPtr>::iterator recog_id;

      std::map<std::string, int> instances_per_scene;
      std::map<std::string, int>::iterator instances_per_scene_it;
      std::string scene_name = rr.first;
      int idx = 0;
      for (recog_id = rr.second->begin (); (recog_id != rr.second->end ()); recog_id++, idx++)
      {
        std::string model_id = (*recog_id)->id_;
        std::cout << "scene name: " << scene_name << std::endl;
        std::cout << "model_id: " << model_id << std::endl;
        std::cout << "idx:" << idx << " " << rtrans.second->size() << std::endl;
        std::cout << rtrans.second->at(idx) << std::endl;

        instances_per_scene_it = instances_per_scene.find((*recog_id)->id_);
        int inst = 0;
        if(instances_per_scene_it != instances_per_scene.end())
        {
            inst = ++instances_per_scene_it->second;
        }
        else
        {
            instances_per_scene.insert(std::make_pair(model_id, 0));
        }

        std::string seq_id;
        std::vector < std::string > strs_2;
        boost::split (strs_2, scene_name, boost::is_any_of ("/\\"));
        seq_id = strs_2[0];

        if(strs_2.size() > 1)
        {
            std::string dir_without_scene_name;
            for(size_t j=0; j < (strs_2.size() - 1); j++)
            {
                dir_without_scene_name.append(strs_2[j]);
                dir_without_scene_name.append("/");
            }

            std::stringstream dir;
            dir << out_dir << "/" << dir_without_scene_name;
            bf::path dir_sequence = dir.str();
            bf::create_directories(dir_sequence);
        }

        if(!replace_model_ext_)
        {
            std::stringstream model_ext;
            model_ext << "." << model_file_extension_;
            boost::replace_all (model_id, model_ext.str(), "");
        }

        std::stringstream pose_file_name;
        pose_file_name << out_dir << "/" << scene_name << "_" << model_id << "_" << inst << ".txt";
        std::cout << pose_file_name.str() << std::endl;

        writeMatrixToFile (pose_file_name.str (),  rtrans.second->at(idx));
        //std::cout << model_id << " " << (*recog_id)->id_ << std::endl;
      }
    }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::classify
         (SceneId & id,
          boost::shared_ptr<std::vector<ModelTPtr> > & results,
          boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms,
          std::vector<bool> & correct)
{

    correct.resize(results->size(), false);

    recognition_results_.clear();
    transforms_.clear();

    recognition_results_.insert(std::pair<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > >(id, results));
    transforms_.insert(std::pair<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > >(id, transforms));

    //iterate over recognition results and compare to the corresponding gt_data_
    typename std::map<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > >::iterator results_it;
    typename std::map<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > >::iterator results_transforms_it;
    typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
    results_transforms_it = transforms_.begin();

    typename std::map<SceneId, PoseStatistics>::iterator pose_iterator;

    for (results_it = recognition_results_.begin (); results_it != recognition_results_.end (); results_it++, results_transforms_it++)
    {
      typename std::pair<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > > rr = *results_it;
      typename std::pair<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > > rtrans = *results_transforms_it;

      gt_data_main_iterator = gt_data_.find (rr.first);
      if (gt_data_main_iterator == gt_data_.end ())
      {
        std::cout << "ERROR: Scene id not found..." << rr.first << std::endl; //this should not happen
        continue;
      }

      //iterate over ground truth
      typename std::map<std::string, std::vector<GTModelTPtr> >::iterator gt_scene_map_iterator; //first is model, second are model instances
      gt_scene_map_iterator = gt_data_main_iterator->second.begin ();

      while (gt_scene_map_iterator != gt_data_main_iterator->second.end ())
      {

        for (size_t i = 0; i < gt_scene_map_iterator->second.size (); i++)
        {
          ConstPointInTPtr model_cloud = gt_scene_map_iterator->second[i]->model_->getAssembled (0.001f);
          typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
          Eigen::Matrix4f trans;
          trans = gt_scene_map_iterator->second[i]->transform_;
          pcl::transformPointCloud (*model_cloud, *model_aligned, trans);
          std::string model_id = gt_scene_map_iterator->second[i]->model_->id_;

          Eigen::Vector4f centroid_gt =  gt_scene_map_iterator->second[i]->model_->getCentroid();
          centroid_gt[3] = 1.f;
          centroid_gt = trans * centroid_gt;
          centroid_gt[3] = 0.f;

          //is the model in the recognition results?
          typename std::vector<ModelTPtr>::iterator recog_id;

          //int idx_found = -1;
          int idx = 0;
          //std::cout << "rr size:" << rr.second->size() << std::endl;
          for (recog_id = rr.second->begin (); (recog_id != rr.second->end ()); recog_id++, idx++)
          {

            bool model_found = false;

            //std::cout << model_id << " " << (*recog_id)->id_ << std::endl;
            if (model_id.compare ((*recog_id)->id_) == 0)
            {
              if (!check_pose_)
              {
                model_found = true;
              }
              else
              {
                //is pose correct? compare centroids and rotation
                Eigen::Vector4f centroid_recog = (*recog_id)->getCentroid();
                centroid_recog[3] = 1.f;
                centroid_recog = rtrans.second->at(idx) * centroid_recog;
                centroid_recog[3] = 0.f;
                float diff = ( centroid_recog - centroid_gt).norm();

                if(diff > max_centroid_diff_)
                {
                  std::cout << "Id ok but rejected by centroid distances: " << model_id << " " << diff << " " << max_centroid_diff_ << std::endl;
                  continue;
                }

                //rotional error, how?
                Eigen::Vector4f x,y,z;
                x = y = z = Eigen::Vector4f::Zero();
                x = Eigen::Vector4f::UnitX();
                y = Eigen::Vector4f::UnitY();
                z = Eigen::Vector4f::UnitZ();

                Eigen::Vector4f x_gt,y_gt,z_gt;
                Eigen::Vector4f x_eval,y_eval,z_eval;
                x_gt = trans * x; y_gt = trans * y; z_gt = trans * z;
                x_eval = rtrans.second->at(idx) * x;
                y_eval = rtrans.second->at(idx) * y;
                z_eval = rtrans.second->at(idx) * z;

                float dotx, doty, dotz;
                dotx = x_eval.dot(x_gt);
                doty = y_eval.dot(y_gt);
                dotz = z_eval.dot(z_gt);

                if(dotx >= 1.f)
                    dotx = 0.9999f;

                if(doty >= 1.f)
                    doty = 0.9999f;

                if(dotz >= 1.f)
                    dotz = 0.9999f;

                if(dotx <= -1.f)
                    dotx = -0.9999f;

                if(doty <= -1.f)
                    doty = -0.9999f;

                if(dotz <= -1.f)
                    dotz = -0.9999f;

                float angle_x, angle_y, angle_z;
                angle_x = std::abs(pcl::rad2deg(acos(dotx)));
                angle_y = std::abs(pcl::rad2deg(acos(doty)));
                angle_z = std::abs(pcl::rad2deg(acos(dotz)));

                if(check_rotation_)
                {
                  float max_angle = std::max(std::max(angle_x, angle_y), angle_z);

                  if(max_angle > max_rotation_)
                  {
                      std::cout << "Id and centroid ok but rejected by rotation: " << model_id << " " << max_angle << " " << max_rotation_ << std::endl;
                      continue;
                  }
                }

                model_found = true;
              }

              if (model_found)
              {
                  correct[idx] = true;
              }
            }
          }
        }
        gt_scene_map_iterator++;
      }
    }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::addRecognitionResults
                                    (SceneId & id,
                                    boost::shared_ptr<std::vector<ModelTPtr> > & results,
                                    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms)
{
  recognition_results_.insert(std::pair<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > >(id, results));
  transforms_.insert(std::pair<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > >(id, transforms));
  std::cout << "addRecognitionResults:" << results->size() << std::endl;
}

template<typename ModelPointT, typename SceneId>
bool
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::checkLoaded()
{
  return gt_data_loaded_;
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::computeStatistics ()
{
  if (!checkLoaded ())
    return;

  scene_statistics_.clear ();
  pose_statistics_.clear ();
  occlusion_results_.clear ();

  std::cout << "number of recognition results:" << recognition_results_.size () << std::endl;
  //iterate over recognition results and compare to the corresponding gt_data_
  typename std::map<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > >::iterator results_it;
  typename std::map<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > >::iterator results_transforms_it;
  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  results_transforms_it = transforms_.begin();

  typename std::map<SceneId, PoseStatistics>::iterator pose_iterator;

  for (results_it = recognition_results_.begin (); results_it != recognition_results_.end (); results_it++, results_transforms_it++)
  {
    typename std::pair<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > > rr = *results_it;
    typename std::pair<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > > rtrans = *results_transforms_it;
    std::cout << rr.first << " " << rr.second->size () << std::endl;
    gt_data_main_iterator = gt_data_.find (rr.first);
    if (gt_data_main_iterator == gt_data_.end ())
    {
      std::cout << "ERROR: Scene id not found..." << rr.first << std::endl; //this should not happen
      continue;
    }

    RecognitionStatistics rs;
    rs.FN_ = rs.FP_ = rs.TP_ = 0;

    //iterate over ground truth
    typename std::map<std::string, std::vector<GTModelTPtr> >::iterator gt_scene_map_iterator; //first is model, second are model instances
    gt_scene_map_iterator = gt_data_main_iterator->second.begin ();

    std::vector<bool> rr_second_used (rr.second->size (), false);
    while (gt_scene_map_iterator != gt_data_main_iterator->second.end ())
    {
      for (size_t i = 0; i < gt_scene_map_iterator->second.size (); i++)
      {
        ConstPointInTPtr model_cloud = gt_scene_map_iterator->second[i]->model_->getAssembled (0.001f);
        typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
        Eigen::Matrix4f trans;
        trans = gt_scene_map_iterator->second[i]->transform_;
        pcl::transformPointCloud (*model_cloud, *model_aligned, trans);
        std::string model_id = gt_scene_map_iterator->second[i]->model_->id_;
        float occlusion = gt_scene_map_iterator->second[i]->occlusion_;
        Eigen::Vector4f centroid_gt =  gt_scene_map_iterator->second[i]->model_->getCentroid();
        centroid_gt[3] = 1.f;
        centroid_gt = trans * centroid_gt;
        centroid_gt[3] = 0.f;
        //is the model in the recognition results?
        typename std::vector<ModelTPtr>::iterator recog_id;
        bool model_found = false;
        int idx_found = -1;
        int idx = 0;
        //std::cout << "rr size:" << rr.second->size() << std::endl;
        for (recog_id = rr.second->begin (); (recog_id != rr.second->end () && !model_found); recog_id++, idx++)
        {
          if (rr_second_used[idx])
            continue;

          //std::cout << model_id << " " << (*recog_id)->id_ << std::endl;
          if (model_id.compare ((*recog_id)->id_) == 0)
          {


            if (!check_pose_)
            {
              model_found = true;
              idx_found = idx;
            }
            else
            {
              //is pose correct? compare centroids and rotation
              Eigen::Vector4f centroid_recog = (*recog_id)->getCentroid();
              centroid_recog[3] = 1.f;
              centroid_recog = rtrans.second->at(idx) * centroid_recog;
              centroid_recog[3] = 0.f;
              float diff = ( centroid_recog - centroid_gt).norm();

              if(diff > max_centroid_diff_)
              {
                std::cout << "Id ok but rejected by centroid distances: " << model_id << " " << diff << " " << max_centroid_diff_ << std::endl;
                continue;
              }

              //rotional error, how?
              Eigen::Vector4f x,y,z;
              x = y = z = Eigen::Vector4f::Zero();
              x = Eigen::Vector4f::UnitX();
              y = Eigen::Vector4f::UnitY();
              z = Eigen::Vector4f::UnitZ();

              Eigen::Vector4f x_gt,y_gt,z_gt;
              Eigen::Vector4f x_eval,y_eval,z_eval;
              x_gt = trans * x; y_gt = trans * y; z_gt = trans * z;
              x_eval = rtrans.second->at(idx) * x;
              y_eval = rtrans.second->at(idx) * y;
              z_eval = rtrans.second->at(idx) * z;

              float dotx, doty, dotz;
              dotx = x_eval.dot(x_gt);
              doty = y_eval.dot(y_gt);
              dotz = z_eval.dot(z_gt);

              if(dotx >= 1.f)
                  dotx = 0.9999f;

              if(doty >= 1.f)
                  doty = 0.9999f;

              if(dotz >= 1.f)
                  dotz = 0.9999f;

              if(dotx <= -1.f)
                  dotx = -0.9999f;

              if(doty <= -1.f)
                  doty = -0.9999f;

              if(dotz <= -1.f)
                  dotz = -0.9999f;

              float angle_x, angle_y, angle_z;
              angle_x = std::abs(pcl::rad2deg(acos(dotx)));
              angle_y = std::abs(pcl::rad2deg(acos(doty)));
              angle_z = std::abs(pcl::rad2deg(acos(dotz)));

              if(check_rotation_)
              {
                float avg_angle = (angle_x + angle_y + angle_z) / 3.f;
                std::cout << "angles:" << angle_x << " " << angle_y << " " << angle_z << "avg:" << avg_angle << std::endl;
                std::cout << "max rotation:" << max_rotation_ << std::endl;

                if(avg_angle > max_rotation_)
                {
                    std::cout << "Id and centroid ok but rejected by rotation: " << model_id << " " << avg_angle << " " << max_rotation_ << std::endl;
                    continue;
                }
              }

              float angle = std::max(angle_x, std::max(angle_y, angle_z));

              model_found = true;
              idx_found = idx;

              pose_iterator = pose_statistics_.find(rr.first);
              if(pose_iterator == pose_statistics_.end())
              {
                  PoseStatistics p;
                  p.centroid_distances_.push_back(diff);
                  p.rotation_error_.push_back(angle);
                  pose_statistics_.insert (std::pair<SceneId, PoseStatistics>(rr.first, p));
              }
              else
              {
                pose_iterator->second.centroid_distances_.push_back(diff);
                pose_iterator->second.rotation_error_.push_back(angle);
              }

            }

            if(model_found)
            {
              occ_tp ot;
              ot.first = occlusion;
              ot.second = true;
              occlusion_results_.push_back(ot);
            }
          }
        }

        if (model_found)
        {
          rs.TP_++;
          rr_second_used[idx_found] = true;
        }
        else
        {
          if(use_max_occlusion_ && (occlusion > max_occlusion_))
            continue;

          occ_tp ot;
          ot.first = occlusion;
          ot.second = false;
          occlusion_results_.push_back(ot);
          rs.FN_++;
        }
      }
      gt_scene_map_iterator++;
    }
    rs.FP_ = rr.second->size () - rs.TP_;
    std::cout << rs.TP_ << " " << rs.FP_ << " " << rs.FN_ << std::endl;
    scene_statistics_.insert (std::pair<SceneId, RecognitionStatistics>(rr.first, rs));
  }

  std::cout << pose_statistics_.size() << " " << scene_statistics_.size() << std::endl;

  int TP, FP, FN;
  TP = FP = FN = 0;
  typename std::map<SceneId, RecognitionStatistics>::iterator it_stats;
  for(it_stats = scene_statistics_.begin(); it_stats != scene_statistics_.end(); it_stats++)
  {
    TP += it_stats->second.TP_;
    FP += it_stats->second.FP_;
    FN += it_stats->second.FN_;
  }

  float avg_diff = 0.f;
  float max_diff = -1.f;
  float min_diff = std::numeric_limits<float>::max();
  int instances = 0;
  SceneId max_diff_scene_id, min_diff_scene_id;

  for(pose_iterator = pose_statistics_.begin(); pose_iterator != pose_statistics_.end(); pose_iterator++)
  {
      for(size_t i=0; i < pose_iterator->second.centroid_distances_.size(); i++, instances++)
      {
          avg_diff += pose_iterator->second.centroid_distances_[i];

          if(max_diff < pose_iterator->second.centroid_distances_[i])
          {
            max_diff_scene_id = pose_iterator->first;
          }

          max_diff = std::max(max_diff, pose_iterator->second.centroid_distances_[i]);

          min_diff = std::min(min_diff, pose_iterator->second.centroid_distances_[i]);
          if(min_diff > pose_iterator->second.centroid_distances_[i])
          {
            min_diff_scene_id = pose_iterator->first;
          }

      }
  }

  float precision = static_cast<float>(TP) / static_cast<float>(TP + FP);
  float recall = static_cast<float>(TP) / static_cast<float>(TP + FN);
  float fscore = 2 * (precision * recall) / (precision + recall);
  std::cout << "instances wiht pose information:" << instances << std::endl;
  std::cout << "tp:" << TP << " fp:" << FP << " fn:" << FN << std::endl;
  std::cout << "precision:" << precision << std::endl;
  std::cout << "recall:" << recall  << std::endl;
  std::cout << "fscore:" << fscore << std::endl;
  std::cout << "Average centroid diff:" << avg_diff / static_cast<float>(instances) << " min,max:" << min_diff << "," << max_diff << "   " << min_diff_scene_id << "," << max_diff_scene_id << std::endl;

  rsr_.precision_ = precision;
  rsr_.recall_ = recall;
  rsr_.fscore_ = fscore;
  rsr_.rs_.TP_ = TP;
  rsr_.rs_.FP_ = FP;
  rsr_.rs_.FN_ = FN;
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::computeStatisticsUpperBound ()
{
  if (!checkLoaded ())
    return;

  if(!check_pose_)
  {
      computeStatistics();
      return;
  }

  scene_statistics_.clear ();
  pose_statistics_.clear ();
  occlusion_results_.clear ();

  std::cout << "number of recognition results:" << recognition_results_.size () << std::endl;
  //iterate over recognition results and compare to the corresponding gt_data_
  typename std::map<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > >::iterator results_it;
  typename std::map<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > >::iterator results_transforms_it;
  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  results_transforms_it = transforms_.begin();

  typename std::map<SceneId, PoseStatistics>::iterator pose_iterator;

  pcl::visualization::PCLVisualizer vis("upper bound visualizer");
  int v1,v2;
  vis.createViewPort(0,0,0.5,1,v1);
  vis.createViewPort(0.5,0,1,1,v2);

  for (results_it = recognition_results_.begin (); results_it != recognition_results_.end (); results_it++, results_transforms_it++)
  {
    typename std::pair<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > > rr = *results_it;
    typename std::pair<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > > rtrans = *results_transforms_it;
    std::cout << rr.first << " " << rr.second->size () << std::endl;
    gt_data_main_iterator = gt_data_.find (rr.first);
    if (gt_data_main_iterator == gt_data_.end ())
    {
      std::cout << "ERROR: Scene id not found..." << rr.first << std::endl; //this should not happen
      continue;
    }

    RecognitionStatistics rs;
    rs.FN_ = rs.FP_ = rs.TP_ = 0;

    //iterate over ground truth
    typename std::map<std::string, std::vector<GTModelTPtr> >::iterator gt_scene_map_iterator; //first is model, second are model instances
    gt_scene_map_iterator = gt_data_main_iterator->second.begin ();
    std::vector<bool> rr_second_used (rr.second->size (), false);

    vis.removeAllPointClouds();
    int added_to_vis = 0;

    while (gt_scene_map_iterator != gt_data_main_iterator->second.end ())
    {

      for (size_t i = 0; i < gt_scene_map_iterator->second.size (); i++) //model instances for GT data, select best!
      {
        ConstPointInTPtr model_cloud = gt_scene_map_iterator->second[i]->model_->getAssembled (0.001f);
        typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
        Eigen::Matrix4f trans;
        trans = gt_scene_map_iterator->second[i]->transform_;
        pcl::transformPointCloud (*model_cloud, *model_aligned, trans);
        std::string model_id = gt_scene_map_iterator->second[i]->model_->id_;
        float occlusion = gt_scene_map_iterator->second[i]->occlusion_;
        Eigen::Vector4f centroid_gt =  gt_scene_map_iterator->second[i]->model_->getCentroid();
        centroid_gt[3] = 1.f;
        centroid_gt = trans * centroid_gt;
        centroid_gt[3] = 0.f;

        //is the model in the recognition results?
        typename std::vector<ModelTPtr>::iterator recog_id;
        bool model_found = false;
        int idx_found = -1;
        int idx = 0;
        float best_pose_factor = -1.f;
        float best_translation_diff = -1.f;
        float best_angle_diff = -1.f;

        //std::cout << "rr size:" << rr.second->size() << std::endl;
        for (recog_id = rr.second->begin (); (recog_id != rr.second->end ()); recog_id++, idx++)
        {
          if (rr_second_used[idx])
            continue;

          //std::cout << model_id << " " << (*recog_id)->id_ << std::endl;
          if (model_id.compare ((*recog_id)->id_) == 0)
          {
              //is pose correct? compare centroids and rotation
              Eigen::Vector4f centroid_recog = (*recog_id)->getCentroid();
              centroid_recog[3] = 1.f;
              centroid_recog = rtrans.second->at(idx) * centroid_recog;
              centroid_recog[3] = 0.f;
              float diff = ( centroid_recog - centroid_gt).norm();

              if(diff > max_centroid_diff_)
              {
                std::cout << "Id ok but rejected by centroid distances: " << model_id << " " << diff << " " << max_centroid_diff_ << std::endl;
                continue;
              }

              //rotional error, how?
              Eigen::Vector4f x,y,z;
              x = y = z = Eigen::Vector4f::Zero();
              x = Eigen::Vector4f::UnitX();
              y = Eigen::Vector4f::UnitY();
              z = Eigen::Vector4f::UnitZ();

              Eigen::Vector4f x_gt,y_gt,z_gt;
              Eigen::Vector4f x_eval,y_eval,z_eval;
              x_gt = trans * x; y_gt = trans * y; z_gt = trans * z;
              x_eval = rtrans.second->at(idx) * x;
              y_eval = rtrans.second->at(idx) * y;
              z_eval = rtrans.second->at(idx) * z;

              float dotx, doty, dotz;
              dotx = x_eval.dot(x_gt);
              doty = y_eval.dot(y_gt);
              dotz = z_eval.dot(z_gt);

              if(dotx >= 1.f)
                  dotx = 0.9999f;

              if(doty >= 1.f)
                  doty = 0.9999f;

              if(dotz >= 1.f)
                  dotz = 0.9999f;

              if(dotx <= -1.f)
                  dotx = -0.9999f;

              if(doty <= -1.f)
                  doty = -0.9999f;

              if(dotz <= -1.f)
                  dotz = -0.9999f;

              float angle_x, angle_y, angle_z;
              angle_x = std::abs(pcl::rad2deg(acos(dotx)));
              angle_y = std::abs(pcl::rad2deg(acos(doty)));
              angle_z = std::abs(pcl::rad2deg(acos(dotz)));

              float angle = std::max(angle_x, std::max(angle_y, angle_z));

              float pose_factor = ( 0.75f * ( (max_centroid_diff_ - diff) / max_centroid_diff_) + 0.25f * (std::max(0.f,180.f - angle)) / 180.f);
              if(pose_factor > best_pose_factor)
              {
                  model_found = true;
                  idx_found = idx;
                  best_pose_factor = pose_factor;
                  best_translation_diff = diff;
                  best_angle_diff = angle_x;
              }
          }
        }

        if (model_found)
        {
          rs.TP_++;
          rr_second_used[idx_found] = true;

          occ_tp ot;
          ot.first = occlusion;
          ot.second = true;
          occlusion_results_.push_back(ot);

          //recognition_results_upper_bound_.insert(std::make_pair(gt_scene_map_iterator->first, rr.second->at(idx_found));
          //transforms_upper_bound_.insert(std::make_pair(gt_scene_map_iterator->first, rtrans.second->at(idx_found));

          {
              ConstPointInTPtr model_cloud = rr.second->at(idx_found)->getAssembled (0.001f);
              typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
              Eigen::Matrix4f trans;
              trans = rtrans.second->at(idx_found);
              pcl::transformPointCloud (*model_cloud, *model_aligned, trans);

              std::stringstream name;
              name << "selected_" << added_to_vis;

              pcl::visualization::PointCloudColorHandlerRandom<ModelPointT> scene_handler(model_aligned);

              vis.addPointCloud<ModelPointT>(model_aligned, scene_handler, name.str(), v1);
          }

          std::stringstream name;
          name << "gt_" << added_to_vis;

          pcl::visualization::PointCloudColorHandlerRandom<ModelPointT> scene_handler(model_aligned);
          vis.addPointCloud<ModelPointT>(model_aligned, scene_handler, name.str(), v2);

          added_to_vis++;

          if(check_pose_)
          {
              pose_iterator = pose_statistics_.find(rr.first);
              if(pose_iterator == pose_statistics_.end())
              {
                  PoseStatistics p;
                  p.centroid_distances_.push_back(best_translation_diff);
                  p.rotation_error_.push_back(best_angle_diff);
                  pose_statistics_.insert (std::pair<SceneId, PoseStatistics>(rr.first, p));
              }
              else
              {
                pose_iterator->second.centroid_distances_.push_back(best_translation_diff);
                pose_iterator->second.rotation_error_.push_back(best_angle_diff);
              }
          }

        }
        else
        {
          if(use_max_occlusion_ && (occlusion > max_occlusion_))
            continue;

          occ_tp ot;
          ot.first = occlusion;
          ot.second = false;
          occlusion_results_.push_back(ot);
          rs.FN_++;
        }
      }

      gt_scene_map_iterator++;
    }

    vis.spin();
    rs.FP_ = rr.second->size () - rs.TP_;
    std::cout << rs.TP_ << " " << rs.FP_ << " " << rs.FN_ << std::endl;
    scene_statistics_.insert (std::pair<SceneId, RecognitionStatistics>(rr.first, rs));
  }

  std::cout << pose_statistics_.size() << " " << scene_statistics_.size() << std::endl;

  int TP, FP, FN;
  TP = FP = FN = 0;
  typename std::map<SceneId, RecognitionStatistics>::iterator it_stats;
  for(it_stats = scene_statistics_.begin(); it_stats != scene_statistics_.end(); it_stats++)
  {
    TP += it_stats->second.TP_;
    FP += it_stats->second.FP_;
    FN += it_stats->second.FN_;
  }

  float avg_diff = 0.f;
  float max_diff = -1.f;
  float min_diff = std::numeric_limits<float>::max();
  int instances = 0;
  SceneId max_diff_scene_id, min_diff_scene_id;

  for(pose_iterator = pose_statistics_.begin(); pose_iterator != pose_statistics_.end(); pose_iterator++)
  {
      for(size_t i=0; i < pose_iterator->second.centroid_distances_.size(); i++, instances++)
      {
          avg_diff += pose_iterator->second.centroid_distances_[i];

          if(max_diff < pose_iterator->second.centroid_distances_[i])
          {
            max_diff_scene_id = pose_iterator->first;
          }

          max_diff = std::max(max_diff, pose_iterator->second.centroid_distances_[i]);

          min_diff = std::min(min_diff, pose_iterator->second.centroid_distances_[i]);
          if(min_diff > pose_iterator->second.centroid_distances_[i])
          {
            min_diff_scene_id = pose_iterator->first;
          }

      }
  }

  std::cout << "instances wiht pose information:" << instances << std::endl;
  std::cout << "tp:" << TP << " fp:" << FP << " fn:" << FN << std::endl;
  std::cout << "precision:" << static_cast<float>(TP) / static_cast<float>(TP + FP) << std::endl;
  std::cout << "recall:" << static_cast<float>(TP) / static_cast<float>(TP + FN) << std::endl;
  std::cout << "Average centroid diff:" << avg_diff / static_cast<float>(instances) << " min,max:" << min_diff << "," << max_diff << "   " << min_diff_scene_id << "," << max_diff_scene_id << std::endl;
  /*for(size_t i=0; i < occlusion_results_.size(); i++)
  {
    std::cout << occlusion_results_[i].first << " - " << static_cast<int>(occlusion_results_[i].second) << std::endl;
  }*/
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::saveStatistics(std::string & out_file)
{
    typename std::map<SceneId, RecognitionStatistics>::iterator it;
    std::cout << "Number of scene statistics:" << scene_statistics_.size() << std::endl;

    int total_hypotheses = 0;
    int total_FP = 0;
    for(it = scene_statistics_.begin(); it != scene_statistics_.end(); it++)
    {
        //std::cout << it->second.TP_ << " " << it->second.FP_ << " " << it->second.FN_ << std::endl;
        total_hypotheses += it->second.TP_ + it->second.FP_ + it->second.FN_;
        total_FP += it->second.FP_;
    }

    std::ofstream myfile;
    myfile.open (out_file.c_str());
    myfile << "num_scenes:\t" << scene_statistics_.size() << std::endl;
    myfile << "total_hypotheses:\t" << total_hypotheses << std::endl;
    myfile << "total_FP:\t" << total_FP << std::endl;

    for(size_t i=0; i < occlusion_results_.size(); i++)
    {
      //std::cout << occlusion_results_[i].first << "\t" << static_cast<int>(occlusion_results_[i].second) << std::endl;
      myfile << occlusion_results_[i].first << "\t" << static_cast<int>(occlusion_results_[i].second) << std::endl;
    }

    myfile.close();
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::savePoseStatistics(std::string & out_file)
{
    typename std::map<SceneId, PoseStatistics>::iterator pose_iterator;
    float avg_diff = 0.f;
    float max_diff = -1.f;
    float min_diff = std::numeric_limits<float>::max();
    int instances = 0;
    SceneId max_diff_scene_id, min_diff_scene_id;

    for(pose_iterator = pose_statistics_.begin(); pose_iterator != pose_statistics_.end(); pose_iterator++)
    {
        for(size_t i=0; i < pose_iterator->second.centroid_distances_.size(); i++, instances++)
        {
            avg_diff += pose_iterator->second.centroid_distances_[i];

            if(max_diff < pose_iterator->second.centroid_distances_[i])
            {
              max_diff_scene_id = pose_iterator->first;
            }

            max_diff = std::max(max_diff, pose_iterator->second.centroid_distances_[i]);

            min_diff = std::min(min_diff, pose_iterator->second.centroid_distances_[i]);
            if(min_diff > pose_iterator->second.centroid_distances_[i])
            {
              min_diff_scene_id = pose_iterator->first;
            }

        }
    }

    std::cout << "instances wiht pose information:" << instances << std::endl;
    std::cout << "Average centroid diff:" << avg_diff / static_cast<float>(instances) << " min,max:" << min_diff << "," << max_diff << "   " << min_diff_scene_id << "," << max_diff_scene_id << std::endl;

    std::ofstream myfile;
    myfile.open (out_file.c_str());
    myfile << "num_scenes:\t" << scene_statistics_.size() << std::endl;
    myfile << "total_hypotheses:\t" << instances << std::endl;

    for(pose_iterator = pose_statistics_.begin(); pose_iterator != pose_statistics_.end(); pose_iterator++)
    {
        for(size_t i=0; i < pose_iterator->second.centroid_distances_.size(); i++, instances++)
        {
            myfile << pose_iterator->second.centroid_distances_[i] << std::endl;
        }
    }

    myfile.close();
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::savePoseStatisticsRotation(std::string & out_file)
{
    typename std::map<SceneId, PoseStatistics>::iterator pose_iterator;
    float avg_diff = 0.f;
    float max_diff = -1.f;
    float min_diff = std::numeric_limits<float>::max();
    int instances = 0;
    SceneId max_diff_scene_id, min_diff_scene_id;

    for(pose_iterator = pose_statistics_.begin(); pose_iterator != pose_statistics_.end(); pose_iterator++)
    {
        for(size_t i=0; i < pose_iterator->second.rotation_error_.size(); i++, instances++)
        {
            avg_diff += pose_iterator->second.rotation_error_[i];

            if(max_diff < pose_iterator->second.rotation_error_[i])
            {
              max_diff_scene_id = pose_iterator->first;
            }

            max_diff = std::max(max_diff, pose_iterator->second.rotation_error_[i]);

            min_diff = std::min(min_diff, pose_iterator->second.rotation_error_[i]);
            if(min_diff > pose_iterator->second.rotation_error_[i])
            {
              min_diff_scene_id = pose_iterator->first;
            }

        }
    }

    std::cout << "instances wiht pose information:" << instances << std::endl;
    std::cout << "Average centroid diff:" << avg_diff / static_cast<float>(instances) << " min,max:" << min_diff << "," << max_diff << "   " << min_diff_scene_id << "," << max_diff_scene_id << std::endl;

    std::ofstream myfile;
    myfile.open (out_file.c_str());
    myfile << "num_scenes:\t" << scene_statistics_.size() << std::endl;
    myfile << "total_hypotheses:\t" << instances << std::endl;

    for(pose_iterator = pose_statistics_.begin(); pose_iterator != pose_statistics_.end(); pose_iterator++)
    {
        for(size_t i=0; i < pose_iterator->second.rotation_error_.size(); i++, instances++)
        {
            myfile << pose_iterator->second.rotation_error_[i] << std::endl;
        }
    }

    myfile.close();
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::updateGT
  (std::string & sid, std::string & m_id, Eigen::Matrix4f & pose)
{
  /*std::cout << sid << " " << m_id << std::endl;
  std::string model_path (gt.models_[j].id_);
  boost::replace_all (model_path, ".ply", "");*/

  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  gt_data_main_iterator = gt_data_.find(sid);
  int m_idx = 0;

  if(gt_data_main_iterator == gt_data_.end())
  {
    std::cout << "Scene id not found..." << sid << std::endl;
  }
  else
  {
    //iterate over the scene and add models
    typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
    scene_map_iterator = gt_data_main_iterator->second.find(m_id);
    if(scene_map_iterator == gt_data_main_iterator->second.end())
    {
      std::cout << "Model not found..." << std::endl;
    }
    else
    {
      m_idx = scene_map_iterator->second.size();
    }
  }

  std::stringstream pose_file_out;
  pose_file_out << gt_dir_ << "/" << sid << "_" << m_id << "_" << m_idx << ".txt";
  writeMatrixToFile (pose_file_out.str (), pose);
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::getModelsForScene
    (SceneId & s_id, std::vector< typename pcl::PointCloud<ModelPointT>::Ptr > & model_clouds, float res)
{
  if(!checkLoaded())
      return;

  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  gt_data_main_iterator = gt_data_.find(s_id);
  if(gt_data_main_iterator == gt_data_.end())
  {
    std::cout << "Scene id not found..." << s_id << std::endl;
  }
  else
  {
    //iterate over the scene and add models
    typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
    scene_map_iterator = gt_data_main_iterator->second.begin();
    while (scene_map_iterator != gt_data_main_iterator->second.end ())
    {
      for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
      {
        ConstPointInTPtr model_cloud = scene_map_iterator->second[i]->model_->getAssembled (res);
        typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
        Eigen::Matrix4f trans = scene_map_iterator->second[i]->transform_;
        pcl::transformPointCloud (*model_cloud, *model_aligned, trans);
        model_clouds.push_back(model_aligned);
      }
      scene_map_iterator++;
    }
  }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::
getGroundTruthModelsAndPoses (SceneId & s_id,
                              boost::shared_ptr<std::vector<ModelTPtr> > & results,
                              boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms)
{
    if(!checkLoaded())
        return;

    typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
    gt_data_main_iterator = gt_data_.find(s_id);
    if(gt_data_main_iterator == gt_data_.end())
    {
        std::cout << "Scene id not found... in getGroundTruthModelsAndPoses:" << s_id << " " << gt_data_.size() << std::endl;
        for(gt_data_main_iterator = gt_data_.begin(); gt_data_main_iterator != gt_data_.end(); gt_data_main_iterator++)
        {
            std::cout << "scene id in gt:" << gt_data_main_iterator->first << std::endl;
        }
    }
    else
    {
        //iterate over the scene and add models
        typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
        scene_map_iterator = gt_data_main_iterator->second.begin();

        while (scene_map_iterator != gt_data_main_iterator->second.end ())
        {
            for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
            {
                results->push_back(scene_map_iterator->second[i]->model_);
                transforms->push_back(scene_map_iterator->second[i]->transform_);
            }
            scene_map_iterator++;
        }
    }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::
getGroundTruthPointCloud(SceneId & s_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & gt_cloud, float model_res)
{
    if(!checkLoaded())
        return;

    typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
    gt_data_main_iterator = gt_data_.find(s_id);
    if(gt_data_main_iterator == gt_data_.end())
    {
        std::cout << "Scene id not found..." << s_id << std::endl;
        for(gt_data_main_iterator = gt_data_.begin(); gt_data_main_iterator != gt_data_.end(); gt_data_main_iterator++)
        {
            std::cout << "scene id in gt:" << gt_data_main_iterator->first << std::endl;
        }
    }
    else
    {
        //iterate over the scene and add models
        typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
        scene_map_iterator = gt_data_main_iterator->second.begin();

        int model_added = 0;
        while (scene_map_iterator != gt_data_main_iterator->second.end ())
        {
            for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
            {
                float occlusion = scene_map_iterator->second[i]->occlusion_;
                if(use_max_occlusion_ && (occlusion > max_occlusion_))
                    continue;

                std::stringstream name;
                name << "gt_model_cloud" << model_added;
                ConstPointInTPtr model_cloud = scene_map_iterator->second[i]->model_->getAssembled (model_res);
                typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
                Eigen::Matrix4f trans;
                trans = scene_map_iterator->second[i]->transform_;
                pcl::transformPointCloud (*model_cloud, *model_aligned, trans);

                typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::copyPointCloud(*model_aligned, *model_aligned_rgb);

                *gt_cloud += *model_aligned_rgb;
            }
            scene_map_iterator++;
        }
    }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::visualizeGroundTruth
  (pcl::visualization::PCLVisualizer & vis, SceneId & s_id, int viewport, bool clear, std::string cloud_name)
{
  if(!checkLoaded())
    return;

  if(clear)
    vis.removeAllPointClouds(viewport);

  typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
  gt_data_main_iterator = gt_data_.find(s_id);
  if(gt_data_main_iterator == gt_data_.end())
  {
    std::cout << "Scene id not found..." << s_id << std::endl;
    for(gt_data_main_iterator = gt_data_.begin(); gt_data_main_iterator != gt_data_.end(); gt_data_main_iterator++)
    {
      std::cout << "scene id in gt:" << gt_data_main_iterator->first << std::endl;
    }
  }
  else
  {
    //iterate over the scene and add models
    typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
    scene_map_iterator = gt_data_main_iterator->second.begin();

    int model_added = 0;
    while (scene_map_iterator != gt_data_main_iterator->second.end ())
    {
      for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
      {
        float occlusion = scene_map_iterator->second[i]->occlusion_;
        if(use_max_occlusion_ && (occlusion > max_occlusion_))
          continue;

        std::stringstream name;
        name << cloud_name << model_added;
        ConstPointInTPtr model_cloud = scene_map_iterator->second[i]->model_->getAssembled (0.001f);
        typename pcl::PointCloud<ModelPointT>::Ptr model_aligned (new pcl::PointCloud<ModelPointT>);
        Eigen::Matrix4f trans;
        trans = scene_map_iterator->second[i]->transform_;
        pcl::transformPointCloud (*model_cloud, *model_aligned, trans);

        typedef pcl::PointCloud<ModelPointT> CloudM;
        typedef typename pcl::traits::fieldList<typename CloudM::PointType>::type FieldListM;

        bool exists_m;
        float rgb_m;
        pcl::for_each_type<FieldListM> (pcl::CopyIfFieldExists<typename CloudM::PointType, float> (model_aligned->points[0],"rgb", exists_m, rgb_m));

        if(!exists_m)
        {
          pcl::visualization::PointCloudColorHandlerRandom<ModelPointT> random_handler(model_aligned);
          vis.addPointCloud<ModelPointT>(model_aligned, random_handler, name.str(), viewport);
        }
        else
        {
          typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_aligned_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::copyPointCloud(*model_aligned, *model_aligned_rgb);
          pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> random_handler(model_aligned_rgb);
          vis.addPointCloud<pcl::PointXYZRGB>(model_aligned_rgb, random_handler, name.str(), viewport);
        }

        model_added++;
      }
      scene_map_iterator++;
    }

    std::cout << "added " << model_added << " models..." << std::endl;
  }
}

template<typename ModelPointT, typename SceneId>
void
faat_pcl::rec_3d_framework::or_evaluator::OREvaluator<ModelPointT, SceneId>::copyToDirectory(std::string & out_dir)
{
    typename std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > >::iterator gt_data_main_iterator;
    for(gt_data_main_iterator = gt_data_.begin(); gt_data_main_iterator != gt_data_.end(); gt_data_main_iterator++)
    {
        //check if directory exist, otherwise create it

        typename std::map<std::string, std::vector<GTModelTPtr> >::iterator scene_map_iterator;
        scene_map_iterator = gt_data_main_iterator->second.begin();

        std::cout << gt_data_main_iterator->first << " " << std::endl;
        std::string scene_name = gt_data_main_iterator->first;

        std::string seq_id;
        std::string scene_id;
        std::vector < std::string > strs_2;
        boost::split (strs_2, gt_data_main_iterator->first, boost::is_any_of ("/\\"));

        if(strs_2.size() > 1)
        {
            std::string dir_without_scene_name;
            for(size_t j=0; j < (strs_2.size() - 1); j++)
            {
                dir_without_scene_name.append(strs_2[j]);
                dir_without_scene_name.append("/");
            }

            std::stringstream dir;
            dir << out_dir << "/" << dir_without_scene_name;
            bf::path dir_sequence = dir.str();
            bf::create_directories(dir_sequence);

            seq_id = dir_without_scene_name;
            scene_id = strs_2[strs_2.size() - 1];
        }
        else
        {
            seq_id = "";
            scene_id = strs_2[0];
        }

        while (scene_map_iterator != gt_data_main_iterator->second.end ())
        {

          std::cout << "scene_map_iterator->first:" << scene_map_iterator->first << " " << scene_map_iterator->second.size() << std::endl; //model id
          std::string model_id = scene_map_iterator->first;

          //check in directory the highest instance id for this model
          std::stringstream ss;
          ss << out_dir << "/" << seq_id;
          bf::path model_file_path;
          model_file_path = ss.str();

          if(!replace_model_ext_)
          {
              std::stringstream model_ext;
              model_ext << "." << model_file_extension_;
              boost::replace_all (model_id, model_ext.str(), "");
          }

          std::vector<std::string> paths;
          std::stringstream pattern;
          pattern << scene_id << "_" <<  model_id << "_.*.txt";

          faat_pcl::utils::getFilesInDirectory(model_file_path, paths, pattern.str());
          int inst = static_cast<int>(paths.size());
          std::cout << "current instances:" << inst << std::endl;

          for(size_t i = 0; i < scene_map_iterator->second.size(); i++)
          {

            Eigen::Matrix4f trans;
            trans = scene_map_iterator->second[i]->transform_;

            std::stringstream pose_file_name;
            pose_file_name << out_dir << "/" << scene_name << "_" << model_id << "_" << inst << ".txt";
            //std::cout << pose_file_name.str() << std::endl;

            writeMatrixToFile (pose_file_name.str (), trans);
            inst++;
          }
          scene_map_iterator++;
        }
    }
}

#endif /* OR_EVALUATOR_HPP_ */
