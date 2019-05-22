/*
 * ply_source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_REG_VIEWS_SOURCE_H_
#define FAAT_PCL_REC_FRAMEWORK_REG_VIEWS_SOURCE_H_

#include "source.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "faat_3d_rec_framework_defines.h"

namespace faat_pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Data source class based on partial views from sensor.
     * In this case, the training data is obtained directly from a depth sensor.
     * The filesystem should contain pcd files (representing a view of an object in
     * camera coordinates) and each view needs to be associated with a txt file
     * containing a 4x4 matrix representing the transformation from camera coordinates
     * to a global object coordinates frame.
     * \author Aitor Aldoma
     */

    template<typename Full3DPointT = pcl::PointXYZRGBNormal, typename PointInT = pcl::PointXYZRGB, typename OutModelPointT = pcl::PointXYZRGB>
      class RegisteredViewsSource : public Source<PointInT>
      {
        typedef Source<PointInT> SourceT;
        typedef Model<OutModelPointT> ModelT;
        typedef boost::shared_ptr<ModelT> ModelTPtr;

        using SourceT::path_;
        using SourceT::models_;
        using SourceT::createTrainingDir;
        using SourceT::getModelsInDirectory;
        using SourceT::model_scale_;
        using SourceT::load_into_memory_;
        using SourceT::getViewsFilenames;
        using SourceT::createClassAndModelDirectories;

        std::string model_structure_; //directory with all the views, indices, poses, etc...
        std::string view_prefix_;
        std::string indices_prefix_;
        std::string pose_prefix_;

      public:
        RegisteredViewsSource ()
        {
          view_prefix_ = std::string ("cloud");
          pose_prefix_ = std::string("pose");
          indices_prefix_ = std::string("object_indices");
          load_into_memory_ = false;
        }

        void
        setModelStructureDir(std::string dir)
        {
          model_structure_ = dir;
        }

        void
        setPrefix (std::string & pre)
        {
          view_prefix_ = pre;
        }

        void
        assembleModelFromViewsAndPoses(ModelT & model,
                                           std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & poses,
                                           std::vector<pcl::PointIndices> & indices,
                                           typename pcl::PointCloud<PointInT>::Ptr &model_cloud) {
          for(size_t i=0; i < model.views_->size(); i++) {
            Eigen::Matrix4f inv = poses[i];
            inv = inv.inverse();

            typename pcl::PointCloud<PointInT>::Ptr global_cloud_only_indices(new pcl::PointCloud<PointInT>);
            pcl::copyPointCloud(*(model.views_->at(i)), indices[i], *global_cloud_only_indices);
            typename pcl::PointCloud<PointInT>::Ptr global_cloud(new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud(*global_cloud_only_indices,*global_cloud, inv);
            *(model_cloud) += *global_cloud;
          }
        }

        void
        loadInMemorySpecificModel(std::string & dir, ModelT & model)
        {
          std::stringstream pathmodel;
          pathmodel << dir << "/" << model.class_ << "/" << model.id_;
          bf::path trained_dir = pathmodel.str ();

          if (bf::exists (trained_dir))
          {
            for (size_t i = 0; i < model.view_filenames_.size (); i++)
            {
              std::stringstream view_file;
              view_file << pathmodel.str () << "/" << model.view_filenames_[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);

              std::string file_replaced1 (model.view_filenames_[i]);
              boost::replace_all (file_replaced1, "view", "pose");
              boost::replace_all (file_replaced1, ".pcd", ".txt");

              //read pose as well
              std::stringstream pose_file;
              pose_file << pathmodel.str () << "/" << file_replaced1;
              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile2 (pose_file.str (), pose);

              //the recognizer assumes transformation from M to CC - i think!
              Eigen::Matrix4f pose_inv = pose.inverse();
              model.poses_->push_back (pose_inv);
              model.self_occlusions_->push_back (-1.f);

              std::string file_replaced2 (model.view_filenames_[i]);
              boost::replace_all (file_replaced2, "view", "object_indices");
              pcl::PointCloud<IndexPoint> obj_indices_cloud;

              std::stringstream oi_file;
              oi_file << pathmodel.str () << "/" << file_replaced2;
              pcl::io::loadPCDFile (oi_file.str(), obj_indices_cloud);
              pcl::PointIndices indices;
              indices.indices.resize(obj_indices_cloud.points.size());
              for(size_t kk=0; kk < obj_indices_cloud.points.size(); kk++)
                indices.indices[kk] = obj_indices_cloud.points[kk].idx;

              model.views_->push_back (cloud);
              model.indices_->push_back(indices);
            }
          }
        }

        void
        loadOrGenerate (std::string & dir, std::string & model_path, ModelT & model)
        {

          std::cout << "loadOrGenerate" << std::endl;

          std::stringstream pathmodel;
          pathmodel << dir << "/" << model.class_ << "/" << model.id_;
          bf::path trained_dir = pathmodel.str ();

          std::cout << pathmodel.str () << std::endl;

          model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr>);
          model.indices_.reset (new std::vector<pcl::PointIndices>);
          model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
          model.self_occlusions_.reset (new std::vector<float>);

          if (bf::exists (trained_dir))
          {

            std::stringstream full_model;
            full_model << path_ << "/" << "/" << model.class_ << "/" << model.id_;
            typename pcl::PointCloud<Full3DPointT>::Ptr modell (new pcl::PointCloud<Full3DPointT>);
            typename pcl::PointCloud<Full3DPointT>::Ptr modell_voxelized (new pcl::PointCloud<Full3DPointT>);
            pcl::io::loadPCDFile(full_model.str(), *modell);

            float voxel_grid_size = 0.003f;
            typename pcl::VoxelGrid<Full3DPointT> grid_;
            grid_.setInputCloud (modell);
            grid_.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
            grid_.setDownsampleAllData (true);
            grid_.filter (*modell_voxelized);

            model.normals_assembled_.reset(new pcl::PointCloud<pcl::Normal>);
            model.assembled_.reset (new pcl::PointCloud<PointInT>);

            pcl::copyPointCloud(*modell_voxelized, *model.assembled_);
            pcl::copyPointCloud(*modell_voxelized, *model.normals_assembled_);

            //load views and poses
            getViewsFilenames(trained_dir, model.view_filenames_, "view");
            /*bf::directory_iterator end_itr;
            for (bf::directory_iterator itr (trained_dir); itr != end_itr; ++itr)
            {
              //check if its a directory, then get models in it
              if (!(bf::is_directory (*itr)))
              {
                //check that it is a ply file and then add, otherwise ignore..
                std::vector < std::string > strs;
                std::vector < std::string > strs_;

#if BOOST_FILESYSTEM_VERSION == 3
                std::string file = (itr->path ().filename ()).string();
#else
                std::string file = (itr->path ()).filename ();
#endif

                boost::split (strs, file, boost::is_any_of ("."));
                boost::split (strs_, file, boost::is_any_of ("_"));

                std::string extension = strs[strs.size () - 1];

                if (extension == "pcd" && strs_[0] == "view")
                {
#if BOOST_FILESYSTEM_VERSION == 3
                  model.view_filenames_.push_back ((itr->path ().filename ()).string());
#else
                  model.view_filenames_.push_back ((itr->path ()).filename ());
#endif
                }
              }
            }*/

            if(load_into_memory_)
            {
              loadInMemorySpecificModel(dir, model);
              /*typename pcl::PointCloud<PointInT>::Ptr model_cloud(new pcl::PointCloud<PointInT>);
              assembleModelFromViewsAndPoses(model, *(model.poses_), *(model.indices_), model_cloud);

              pcl::visualization::PCLVisualizer vis ("assembled model...");
              pcl::visualization::PointCloudColorHandlerRGBField<PointInT> random_handler (model_cloud);
              vis.addPointCloud<PointInT> (model_cloud, random_handler, "points");
              vis.addCoordinateSystem(0.1);
              vis.spin ();*/
            }

          }
          else
          {

            //we just need to copy the views to the training directory

            std::cout << "we just need to copy the views to the training directory" << std::endl;

            std::stringstream direc_ms;
            direc_ms << model_structure_ << "/" << model.class_ << "/" << model.id_ << "/";
            createClassAndModelDirectories (dir, model.class_, model.id_);

            std::vector < std::string > view_filenames;

            std::cout << model_structure_ << std::endl;
            std::cout << direc_ms.str() << std::endl;

            bf::path dirr = direc_ms.str();
            getViewsFilenames (dirr, view_filenames, view_prefix_);
            std::cout << view_filenames.size () << std::endl;

            for (size_t i = 0; i < view_filenames.size (); i++)
            {
              std::stringstream direc;
              direc << dir << "/" << model.class_ << "/" << model.id_ << "/";

              std::stringstream view_file;
              view_file << direc_ms.str() << "/" << view_filenames[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);

              std::string file_replaced_indices (view_file.str ());
              boost::replace_last (file_replaced_indices, view_prefix_, indices_prefix_);
              pcl::PointCloud<IndexPoint> obj_indices_cloud;
              pcl::io::loadPCDFile (file_replaced_indices, obj_indices_cloud);

              //cut the cloud and adapt indices before saving,
              //this will speed up the loading afterwards and reduce memory footprint
              pcl::PointIndices indices;
              indices.indices.resize(obj_indices_cloud.points.size());
              for(size_t kk=0; kk < obj_indices_cloud.points.size(); kk++)
                indices.indices[kk] = obj_indices_cloud.points[kk].idx;

              pcl::PointCloud<int> mask_cloud;
              mask_cloud.width = cloud->width;
              mask_cloud.height = cloud->height;
              mask_cloud.points.resize(cloud->width * cloud->height);
              for(size_t k=0; k < mask_cloud.points.size(); k++)
                mask_cloud.points[k] = 0;

              for(size_t k=0; k < indices.indices.size(); k++)
                mask_cloud.points[indices.indices[k]] = 1;

              int min_u, min_v; min_u = min_v = std::numeric_limits<int>::max();
              int max_u, max_v; max_u = max_v = 0;

              for(int u=0; u < mask_cloud.width; u++)
              {
                for(int v=0; v < mask_cloud.height; v++)
                {
                  if(mask_cloud.at(u,v) > 0)
                  {
                    if(u < min_u) min_u = u;
                    if(v < min_v) min_v = v;
                    if(u > max_u) max_u = u;
                    if(v > max_v) max_v = v;
                  }
                }
              }

              //std::cout << min_u << " " << min_v << " " << max_u << " " << max_v << std::endl;
              int allow = 500;
              min_u = std::max(0, min_u - allow);
              min_v = std::max(0, min_v - allow);
              max_u = std::min(static_cast<int>(cloud->width - 1), max_u + allow);
              max_v = std::min(static_cast<int>(cloud->height - 1), max_v + allow);

              int new_width = max_u - min_u + 1;
              int new_height = max_v - min_v + 1;
              typename pcl::PointCloud<PointInT> cloud_cropped;
              cloud_cropped.width = new_width;
              cloud_cropped.height = new_height;
              cloud_cropped.is_dense = cloud->is_dense;
              cloud_cropped.points.resize(new_height * new_width);

              pcl::PointCloud<IndexPoint> obj_indices_cloud_cropped;
              obj_indices_cloud_cropped.width = new_width * new_height;
              obj_indices_cloud_cropped.height = 1;
              obj_indices_cloud_cropped.points.resize(new_height * new_width);

              //std::cout << cloud_cropped.isOrganized() << cloud->isOrganized() << std::endl;

              int kept = 0;
              for (int u = 0; u < mask_cloud.width; u++)
              {
                for (int v = 0; v < mask_cloud.height; v++)
                {
                  if (u >= min_u && u <= (max_u) && v >= min_v && v <= (max_v))
                  {
                    cloud_cropped.at (u - min_u, v - min_v) = cloud->at (u, v);

                    /*bool exists;
                    float rgb;
                    typedef pcl::PointCloud<PointInT> CloudM;
                    typedef typename pcl::traits::fieldList<typename CloudM::PointType>::type FieldListM;

                    pcl::for_each_type<FieldListM> (pcl::CopyIfFieldExists<typename CloudM::PointType, float> (cloud->at (u, v), "rgb", exists, rgb));
                    cloud_cropped.at (u - min_u, v - min_v).getVector3fMap () = cloud->at (u, v).getVector3fMap ();
                    if (exists)
                    {
                      pcl::for_each_type<FieldListM> (pcl::SetIfFieldExists<typename CloudM::PointType, float> (cloud_cropped.at (u - min_u, v - min_v), "rgb", rgb));
                    }
                    else
                    {
                      std::cout << "Does not exist" << std::endl;
                    }*/

                    if (mask_cloud.at (u, v) > 0)
                    {
                      //compute idx and add to obj_indices_cloud_cropped
                      int idx = (v - min_v) * new_width + (u - min_u);
                      obj_indices_cloud_cropped.points[kept].idx = idx;
                      kept++;
                    }
                  }
                }
              }

              obj_indices_cloud_cropped.width = kept;
              obj_indices_cloud_cropped.height = 1;
              obj_indices_cloud_cropped.points.resize(kept);

              //std::cout << kept << " " << obj_indices_cloud.points.size() << std::endl;
              assert(kept == static_cast<int>(obj_indices_cloud.points.size()));

              /*{
                pcl::PointIndices indices;
                indices.indices.resize (obj_indices_cloud_cropped.points.size ());
                for (size_t kk = 0; kk < obj_indices_cloud_cropped.points.size (); kk++)
                  indices.indices[kk] = obj_indices_cloud_cropped.points[kk].idx;

                typename pcl::PointCloud<PointInT>::Ptr cloud_ (new pcl::PointCloud<PointInT> ());
                pcl::copyPointCloud (cloud_cropped, indices, *cloud_);
                pcl::visualization::PCLVisualizer vis ("vis");

                {
                  typename pcl::PointCloud<PointInT>::Ptr cloud_ (new pcl::PointCloud<PointInT> (cloud_cropped));
                  pcl::visualization::PointCloudColorHandlerRGBField<PointInT> random_handler (cloud_);
                  vis.addPointCloud(cloud_, random_handler, "non indices");
                }

                {
                  pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler (cloud_, 255, 0, 0);
                  vis.addPointCloud(cloud_, random_handler, "cloud");
                }
                vis.spin();
            }*/

              std::stringstream path_view;
              path_view << direc.str () << "/view_" << i << ".pcd";
              pcl::io::savePCDFileBinary (path_view.str (), cloud_cropped);

              std::stringstream path_oi;
              path_oi << direc.str () << "/object_indices_" << i << ".pcd";
              pcl::io::savePCDFileBinary(path_oi.str(), obj_indices_cloud_cropped);

              std::string file_replaced1 (view_file.str ());
              boost::replace_last (file_replaced1, view_prefix_, pose_prefix_);
              boost::replace_last (file_replaced1, ".pcd", ".txt");

              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile2 (file_replaced1, pose);
              std::stringstream path_pose;
              path_pose << direc.str () << "/pose_" << i << ".txt";
              faat_pcl::rec_3d_framework::PersistenceUtils::writeMatrixToFile (path_pose.str (), pose);
            }

            loadOrGenerate (dir, model_path, model);

          }
        }

        bool
        isleafDirectory (bf::path & path)
        {
          bf::directory_iterator end_itr;
          bool no_dirs_inside = true;
          for (bf::directory_iterator itr (path); itr != end_itr; ++itr)
          {
            if (bf::is_directory (*itr))
            {
              no_dirs_inside = false;
            }
          }

          return no_dirs_inside;
        }

        void
        getFilesInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
        {
          bf::directory_iterator end_itr;
          for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
          {
            //check if its a directory, then ignore
            if (bf::is_directory (*itr))
            {

            }
            else
            {
              std::vector < std::string > strs;
#if BOOST_FILESYSTEM_VERSION == 3
              std::string file = (itr->path ().filename ()).string();
#else
              std::string file = (itr->path ()).filename ();
#endif

              boost::split (strs, file, boost::is_any_of ("."));
              std::string extension = strs[strs.size () - 1];

              if (extension.compare (ext) == 0)
              {
#if BOOST_FILESYSTEM_VERSION == 3
                std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
                std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif

                relative_paths.push_back (path);
              }
            }
          }
        }

        /**
         * \brief Creates the model representation of the training set, generating views if needed
         */
        void
        generate (std::string & training_dir)
        {

          //create training dir fs if not existent
          createTrainingDir (training_dir);

          //get models in directory
          std::vector < std::string > files;
          std::string start = "";
          bf::path dir = path_;
          std::string ext = "pcd";

          getFilesInDirectory (dir, start, files, ext);
          std::cout << files.size() << std::endl;

          models_.reset (new std::vector<ModelTPtr>);

          for (size_t i = 0; i < files.size (); i++)
          {
            ModelTPtr m(new ModelT());

            std::vector < std::string > strs;
            boost::split (strs, files[i], boost::is_any_of ("/\\"));
            std::string name = strs[strs.size () - 1];

            if (strs.size () == 1)
            {
              m->id_ = strs[0];
            }
            else
            {
              std::stringstream ss;
              for (int i = 0; i < (static_cast<int> (strs.size ()) - 1); i++)
              {
                ss << strs[i];
                if (i != (static_cast<int> (strs.size ()) - 1))
                  ss << "/";
              }

              m->class_ = ss.str ();
              m->id_ = strs[strs.size () - 1];
            }

            std::cout << m->class_ << " . " << m->id_ << std::endl;
            //check which of them have been trained using training_dir and the model_id_
            //load views, poses and self-occlusions for those that exist
            //generate otherwise

            std::stringstream model_path;
            model_path << path_ << "/" << files[i];
            std::string path_model = model_path.str ();
            loadOrGenerate (training_dir, path_model, *m);

            models_->push_back (m);

            //std::cout << files[i] << std::endl;
          }
        }
      };
  }
}

#endif /* REC_FRAMEWORK_MESH_SOURCE_H_ */
