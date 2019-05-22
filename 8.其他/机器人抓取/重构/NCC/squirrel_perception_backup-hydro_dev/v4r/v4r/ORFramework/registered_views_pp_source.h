/*
 * ply_source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_REG_VIEWS_WITH_PP_SOURCE_H_
#define FAAT_PCL_REC_FRAMEWORK_REG_VIEWS_WITH_PP_SOURCE_H_

#include "faat_3d_rec_framework_defines.h"
#include "source.h"
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common.h>
#include <boost/algorithm/string/predicate.hpp>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/lum.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Data source class based on partial views from sensor.
     * In this case, the original training data is obtained directly from a depth sensor.
     * The filesystem should contain pcd files (representing a view of an object in
     * camera coordinates) and each view needs to be associated with a txt file
     * containing a 4x4 matrix representing the transformation from camera coordinates
     * to a global object coordinates frame.
     * Alternatively, the views might be already registered.
     * It includes a post-processing stage to clean up the data (similar to kinfu...)
     * \author Aitor Aldoma
     */

    template<typename PointInT>
      class FAAT_3D_FRAMEWORK_API RegisteredViewsWithPPSource : public Source<PointInT>
      {
        typedef Source<PointInT> SourceT;
        typedef Model<PointInT> ModelT;
        typedef boost::shared_ptr<ModelT> ModelTPtr;

        using SourceT::path_;
        using SourceT::models_;
        using SourceT::createTrainingDir;
        using SourceT::getModelsInDirectory;
        using SourceT::model_scale_;

        std::string view_prefix_;
        int pose_files_order_; //0 is row, 1 is column
        float res_grid_;

        template<typename PointT>
        class FAAT_3D_FRAMEWORK_API Voxel
        {
        public:
          bool label_;
          PointT avg_;
          pcl::Normal normal_avg_;
          pcl::RGB color_avg_;
          int n_;
          std::vector<int> view_idx_;
          float weight_;
        };

        typename std::vector<Voxel<PointInT> > grid_;

      public:
        RegisteredViewsWithPPSource ()
        {
          view_prefix_ = std::string ("view");
          pose_files_order_ = 0;
          res_grid_ = 0.003f;
        }

        void
        setPrefix (std::string & pre)
        {
          view_prefix_ = pre;
        }

        void
        setPoseRowOrder (int o)
        {
          pose_files_order_ = o;
        }

        inline bool
        sortFiles (const std::string & file1, const std::string & file2)
        {
          std::vector < std::string > strs1;
          boost::split (strs1, file1, boost::is_any_of ("/"));

          std::vector < std::string > strs2;
          boost::split (strs2, file2, boost::is_any_of ("/"));

          std::string id_1 = strs1[strs1.size () - 1];
          std::string id_2 = strs2[strs2.size () - 1];

          size_t pos1 = id_1.find (".pcd");
          size_t pos2 = id_2.find (".pcd");

          id_1 = id_1.substr (0, pos1);
          id_2 = id_2.substr (0, pos2);

          id_1 = id_1.substr (2);
          id_2 = id_2.substr (2);

          return atoi (id_1.c_str ()) < atoi (id_2.c_str ());
        }

        void
        getViewsFilenames (bf::path & path_with_views, std::vector<std::string> & view_filenames)
        {
          int number_of_views = 0;
          bf::directory_iterator end_itr;
          for (bf::directory_iterator itr (path_with_views); itr != end_itr; ++itr)
          {
            if (!(bf::is_directory (*itr)))
            {
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

              if (extension == "pcd" && ( boost::algorithm::starts_with(strs_[0], view_prefix_) || (strs_[0].compare (view_prefix_) == 0) ))
              {
#if BOOST_FILESYSTEM_VERSION == 3
                view_filenames.push_back ((itr->path ().filename ()).string());
#else
                view_filenames.push_back ((itr->path ()).filename ());
#endif

                number_of_views++;
              }
            }
          }
        }

        void
        assembleModelFromViewsAndPoses(ModelT & model, std::vector<Eigen::Matrix4f> & poses) {
          for(size_t i=0; i < model.views_->size(); i++) {
            Eigen::Matrix4f inv = poses[i];
            typename pcl::PointCloud<PointInT>::Ptr global_cloud(new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud(*(model.views_->at(i)),*global_cloud, inv);
            *(model.assembled_) += *global_cloud;
          }
        }

        void
        loadOrGenerate (std::string & dir, std::string & model_path, ModelT & model)
        {

          std::stringstream pathmodel;
          pathmodel << dir << "/" << model.class_ << "/" << model.id_;
          bf::path trained_dir = pathmodel.str ();

          model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr>);
          model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
          model.self_occlusions_.reset (new std::vector<float>);

          if (bf::exists (trained_dir))
          {
            //load views and poses
            std::vector < std::string > view_filenames;
            int number_of_views = 0;
            bf::directory_iterator end_itr;
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
                  view_filenames.push_back ((itr->path ().filename ()).string());
#else
                  view_filenames.push_back ((itr->path ()).filename ());
#endif

                  number_of_views++;
                }
              }
            }

            std::vector<Eigen::Matrix4f> poses_to_assemble_;

            for (size_t i = 0; i < view_filenames.size (); i++)
            {
              std::stringstream view_file;
              view_file << pathmodel.str () << "/" << view_filenames[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);

              model.views_->push_back (cloud);

              std::string file_replaced1 (view_filenames[i]);
              boost::replace_all (file_replaced1, "view", "pose");
              boost::replace_all (file_replaced1, ".pcd", ".txt");

              //read pose as well
              std::stringstream pose_file;
              pose_file << pathmodel.str () << "/" << file_replaced1;
              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile (pose_file.str (), pose);

              if(pose_files_order_ != 0) {
                //std::cout << "Transpose..." << std::endl;

                Eigen::Matrix4f pose_trans = pose.transpose();
                poses_to_assemble_.push_back(pose_trans);
                //pose = pose_trans;
                //std::cout << pose << std::endl;
              }

              //std::cout << "pose being push backed to model" << std::endl;
              std::cout << pose << std::endl;

              //the recognizer assumes transformation from M to CC
              Eigen::Matrix4f inv = pose.inverse();
              model.poses_->push_back (inv);

              model.self_occlusions_->push_back (-1.f);

            }

            model.assembled_.reset (new pcl::PointCloud<PointInT>);
            assembleModelFromViewsAndPoses(model, poses_to_assemble_);

            /*pcl::visualization::PCLVisualizer vis ("results");
            pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler (model.assembled_, 255, 0, 0);
            vis.addPointCloud<PointInT> (model.assembled_, random_handler, "points");

            Eigen::Matrix4f view_transformation = model.poses_->at(0).inverse();
            typename pcl::PointCloud<PointInT>::Ptr view_trans(new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud(*(model.views_->at(0)), *view_trans, view_transformation);

            pcl::visualization::PointCloudColorHandlerCustom<PointInT> random_handler2 (view_trans, 0, 255, 0);
            vis.addPointCloud<PointInT> (view_trans, random_handler2, "view");

            vis.addCoordinateSystem(0.1);
            vis.spin ();*/

          }
          else
          {

            grid_.clear();
            //add views to the voxel grid

            //filter out bad matches...

            //we just need to copy the views to the training directory
            std::stringstream direc;
            direc << dir << "/" << model.class_ << "/" << model.id_;
            createClassAndModelDirectories (dir, model.class_, model.id_);

            std::vector < std::string > view_filenames;
            bf::path model_dir = model_path;

            //read all views...compute voxel grid sizes
            getViewsFilenames (model_dir, view_filenames);
            std::cout << view_filenames.size () << " " << model_dir.string() << std::endl;
            std::sort(view_filenames.begin(), view_filenames.end());
            std::vector<typename pcl::PointCloud<PointInT>::Ptr> view_clouds;
            view_clouds.resize(view_filenames.size ());

            for (size_t i = 0; i < view_filenames.size (); i++)
            {
              std::stringstream view_file;
              view_file << model_path << "/" << view_filenames[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);
              //view_clouds[i] = cloud;
              pcl::RadiusOutlierRemoval<PointInT> sor;
              sor.setInputCloud (cloud);
              sor.setRadiusSearch (0.01f);
              sor.setMinNeighborsInRadius (27);
              view_clouds[i].reset(new pcl::PointCloud<PointInT>);
              sor.filter (*(view_clouds[i]));

              std::cout << view_file.str () << std::endl;
            }


            //pairwise pose refinement
            /*for(size_t i = 1; i < view_clouds.size(); i++)
            {
              typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT>::Ptr
                                                  rej (new pcl::registration::CorrespondenceRejectorSampleConsensus<PointInT> ());

              rej->setInputTarget (view_clouds[i - 1]);
              rej->setMaximumIterations (1000);
              rej->setInlierThreshold (0.001f);
              rej->setInputSource (view_clouds[i]);

              pcl::IterativeClosestPoint<PointInT, PointInT> reg;
              reg.addCorrespondenceRejector (rej);
              reg.setInputTarget (view_clouds[i - 1]);
              reg.setInputSource (view_clouds[i]);
              reg.setMaximumIterations (5);
              reg.setMaxCorrespondenceDistance (0.002);

              typename pcl::PointCloud<PointInT>::Ptr output_ (new pcl::PointCloud<PointInT> ());
              reg.align (*output_);

              pcl::copyPointCloud(*output_, *view_clouds[i]);
            }*/

            //lum refinement
            /*pcl::registration::LUM<PointInT> lum;
            lum.addPointCloud (view_clouds[0]);
            std::vector<bool> used_in_lum;
            used_in_lum.resize(view_clouds.size(), true);

            for(size_t i = 1; i < view_clouds.size(); i++)
            {
              if(i < view_clouds.size())
                lum.addPointCloud (view_clouds[i]);

              typename pcl::search::KdTree<PointInT>::Ptr scene_downsampled_tree_;
              scene_downsampled_tree_.reset (new pcl::search::KdTree<PointInT>);
              scene_downsampled_tree_->setInputCloud(view_clouds[i%(view_clouds.size()-1)]);

              std::vector<int> nn_indices;
              std::vector<float> nn_distances;

              //compute correspondences between cloud i-1 and i
              pcl::CorrespondencesPtr corr_iminus1_to_i;
              corr_iminus1_to_i.reset(new pcl::Correspondences);

              for(size_t k=0; k < view_clouds[i-1]->points.size(); k++)
              {
                scene_downsampled_tree_->nearestKSearch(view_clouds[i-1]->points[k], 1, nn_indices, nn_distances);
                if(sqrt(nn_distances[0]) < 0.0025)
                {
                  corr_iminus1_to_i->push_back(pcl::Correspondence (k, nn_indices[0], nn_distances[0]));
                }
              }

              std::cout << "num correspondences:" << corr_iminus1_to_i->size() << " between cloud" << (i-1) << " to cloud" << i%(view_clouds.size()-1) << std::endl;

              if(corr_iminus1_to_i->size() < 500) {
                used_in_lum[i%(view_clouds.size()-1)] = false;
                PCL_WARN("Not enough correspondences..\n");

                pcl::visualization::PCLVisualizer vis_ ("lum cloud");

                {
                  pcl::visualization::PointCloudColorHandlerRGBField<PointInT> handler_outliers (view_clouds[i-1]);
                  vis_.addPointCloud<PointInT> (view_clouds[i-1], handler_outliers, "cloud");
                }

                {
                  pcl::visualization::PointCloudColorHandlerRGBField<PointInT> handler_outliers (view_clouds[i%(view_clouds.size())]);
                  vis_.addPointCloud<PointInT> (view_clouds[i%(view_clouds.size()-1)], handler_outliers, "cloud2");
                }
                vis_.addCoordinateSystem(0.2f);
                vis_.spin();

                continue;
              }

              lum.setCorrespondences (i-1, i%(view_clouds.size()-1), corr_iminus1_to_i);

            }

           lum.setMaxIterations (30);
           lum.setConvergenceThreshold (0.0);
           // Perform the actual LUM computation
           lum.compute ();

           {
             typename pcl::PointCloud<PointInT>::Ptr cloud_out (new pcl::PointCloud<PointInT> ());
             cloud_out = lum.getConcatenatedCloud ();
             pcl::visualization::PCLVisualizer vis_ ("lum cloud");
             pcl::visualization::PointCloudColorHandlerRGBField<PointInT> handler_outliers (cloud_out);
             vis_.addPointCloud<PointInT> (cloud_out, handler_outliers, "cloud");
             vis_.addCoordinateSystem(0.2f);
             vis_.spin();
           }

           for(int i = 0; i < lum.getNumVertices (); i++)
           {
             if(used_in_lum[i]) {
               pcl::transformPointCloud(*view_clouds[i], *view_clouds[i], lum.getTransformation (i));
             }
           }*/

            //compute the bounding boxes for the models
            PointInT min_pt_all, max_pt_all;
            min_pt_all.x = min_pt_all.y = min_pt_all.z = std::numeric_limits<float>::max ();
            max_pt_all.x = max_pt_all.y = max_pt_all.z = (std::numeric_limits<float>::max () - 0.001f) * -1;

            for (size_t i = 0; i < view_clouds.size (); i++)
            {
              PointInT min_pt, max_pt;
              pcl::getMinMax3D (*view_clouds[i], min_pt, max_pt);
              if (min_pt.x < min_pt_all.x)
                min_pt_all.x = min_pt.x;

              if (min_pt.y < min_pt_all.y)
                min_pt_all.y = min_pt.y;

              if (min_pt.z < min_pt_all.z)
                min_pt_all.z = min_pt.z;

              if (max_pt.x > max_pt_all.x)
                max_pt_all.x = max_pt.x;

              if (max_pt.y > max_pt_all.y)
                max_pt_all.y = max_pt.y;

              if (max_pt.z > max_pt_all.z)
                max_pt_all.z = max_pt.z;
            }

            int size_x, size_y, size_z;
            size_x = static_cast<int> (std::ceil (std::abs (max_pt_all.x - min_pt_all.x) / res_grid_)) + 1;
            size_y = static_cast<int> (std::ceil (std::abs (max_pt_all.y - min_pt_all.y) / res_grid_)) + 1;
            size_z = static_cast<int> (std::ceil (std::abs (max_pt_all.z - min_pt_all.z) / res_grid_)) + 1;

            grid_.resize (size_x * size_y * size_z);
            for(size_t i=0; i < grid_.size(); i++)
              grid_[i].n_ = 0;

            bool color_exists = false;
            typedef pcl::PointCloud<PointInT> CloudM;
            typedef typename pcl::traits::fieldList<typename CloudM::PointType>::type FieldListM;

            for (size_t i = 0; i < view_clouds.size (); i++)
            {
              pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
              typedef typename pcl::NormalEstimation<PointInT, pcl::Normal> NormalEstimator_;
              NormalEstimator_ n3d;
              typename pcl::search::KdTree<PointInT>::Ptr normals_tree (new pcl::search::KdTree<PointInT>);
              normals_tree->setInputCloud (view_clouds[i]);
              n3d.setViewPoint(view_clouds[i]->sensor_origin_[0],view_clouds[i]->sensor_origin_[1],view_clouds[i]->sensor_origin_[2]);
              n3d.setKSearch (15);
              n3d.setSearchMethod (normals_tree);
              n3d.setInputCloud (view_clouds[i]);
              n3d.compute (*normals);

              //add view_clouds[i] to the voxel grid
              for(size_t k=0; k < view_clouds[i]->points.size(); k++)
              {

                if (!pcl_isfinite (normals->points[k].normal_x) || !pcl_isfinite (normals->points[k].normal_y)
                    || !pcl_isfinite (normals->points[k].normal_z))
                  continue;

                int pos_x, pos_y, pos_z;
                pos_x = static_cast<int> (pcl_round ((view_clouds[i]->points[k].x - min_pt_all.x) / res_grid_));
                pos_y = static_cast<int> (pcl_round ((view_clouds[i]->points[k].y - min_pt_all.y) / res_grid_));
                pos_z = static_cast<int> (pcl_round ((view_clouds[i]->points[k].z - min_pt_all.z) / res_grid_));

                assert(pos_x >= 0);
                assert(pos_y >= 0);
                assert(pos_z >= 0);

                assert(pos_x < size_x);
                assert(pos_y < size_y);
                assert(pos_z < size_z);

                int idx = pos_z * size_x * size_y + pos_y * size_x + pos_x;

                float rgb_m;
                bool exists_m;

                pcl::for_each_type<FieldListM> (
                                                pcl::CopyIfFieldExists<typename CloudM::PointType, float> (view_clouds[i]->points[k],
                                                                                                           "rgb", exists_m, rgb_m));

                if(grid_[idx].n_ == 0)
                {
                  grid_[idx].avg_ = view_clouds[i]->points[k];
                  grid_[idx].normal_avg_ = normals->points[k];
                  grid_[idx].weight_ = 1.f;

                  if(exists_m)
                  {
                    grid_[idx].color_avg_.rgb = rgb_m;
                    color_exists = true;
                  }

                }
                else
                {
                  grid_[idx].avg_.getVector3fMap() = (grid_[idx].avg_.getVector3fMap() * static_cast<float> (grid_[idx].n_) + view_clouds[i]->points[k].getVector3fMap ())
                          / static_cast<float> (grid_[idx].n_ + 1);

                  grid_[idx].normal_avg_.getNormalVector3fMap() = (grid_[idx].normal_avg_.getNormalVector3fMap() * static_cast<float> (grid_[idx].n_) + normals->points[k].getNormalVector3fMap ())
                                            / static_cast<float> (grid_[idx].n_ + 1);

                  if(exists_m)
                  {
                   uint32_t rgb = *reinterpret_cast<int*> (&rgb_m);
                   int r = (rgb >> 16) & 0x0000ff;
                   int g = (rgb >> 8) & 0x0000ff;
                   int b = (rgb) & 0x0000ff;
                   grid_[idx].color_avg_.r = (grid_[idx].color_avg_.r * static_cast<float> (grid_[idx].n_) + r)
                                              / static_cast<float> (grid_[idx].n_ + 1);

                   grid_[idx].color_avg_.g = (grid_[idx].color_avg_.g * static_cast<float> (grid_[idx].n_) + g)
                                                                 / static_cast<float> (grid_[idx].n_ + 1);

                   grid_[idx].color_avg_.b = (grid_[idx].color_avg_.b * static_cast<float> (grid_[idx].n_) + b)
                                                                 / static_cast<float> (grid_[idx].n_ + 1);

                   //grid_[idx].color_avg_.rgb = rgb_m;
                   color_exists = true;
                  }
                }

                grid_[idx].n_++;
                grid_[idx].view_idx_.push_back(i);
              }
            }

            //extrat cloud from voxel grid and visualize...
            typename pcl::PointCloud<PointInT>::Ptr voxel_cloud (new pcl::PointCloud<PointInT> ());
            pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

            int p=0;
            for(size_t i=0; i < grid_.size(); i++)
            {
              std::vector<int>::iterator it;
              it = std::unique (grid_[i].view_idx_.begin(), grid_[i].view_idx_.end());
              grid_[i].view_idx_.resize( std::distance(grid_[i].view_idx_.begin(),it) );
              //if((grid_[i].n_ > 5) && (grid_[i].weight_ > 0.9f))
              if((grid_[i].view_idx_.size() > 2) && (grid_[i].weight_ > 0.9f))
              {
                voxel_cloud->points.push_back(grid_[i].avg_);
                if(color_exists)
                {
                  float rgb_m;
                  bool exists_m;

                  pcl::SetIfFieldExists<typename CloudM::PointType, float> (voxel_cloud->points[p], "rgb", grid_[i].color_avg_.rgb);
                }
                normals->points.push_back(grid_[i].normal_avg_);
                p++;
              }
            }

            //reconstruct voxel cloud and render views using mesh source...
            /*int default_depth = 8;
            int default_solver_divide = 8;
            int default_iso_divide = 8;
            float default_point_weight = 4.0f;

            pcl::PointCloud<pcl::PointNormal>::Ptr voxel_normal_cloud (new pcl::PointCloud<pcl::PointNormal> ());
            pcl::copyPointCloud(*voxel_cloud, *voxel_normal_cloud);
            pcl::copyPointCloud(*normals, *voxel_normal_cloud);
            pcl::PolygonMesh output;
            pcl::Poisson<pcl::PointNormal> poisson;
            poisson.setDepth (default_depth);
            poisson.setSolverDivide (default_solver_divide);
            poisson.setIsoDivide (default_iso_divide);
            poisson.setPointWeight (default_point_weight);
            poisson.setInputCloud (voxel_normal_cloud);
            poisson.reconstruct (output);*/
            //vis_.addPolygonMesh(output, "reconstructed");

            if(color_exists) {
              pcl::visualization::PCLVisualizer vis_ ("voxel cloud");
              pcl::visualization::PointCloudColorHandlerRGBField<PointInT> handler_outliers (voxel_cloud);
              vis_.addPointCloud<PointInT> (voxel_cloud, handler_outliers, "cloud");
              //vis_.addPointCloudNormals<PointInT, pcl::Normal>(voxel_cloud, normals, 30, 0.01f, "normals");
              vis_.addCoordinateSystem(0.2f);
              vis_.spin();

              voxel_cloud->width = voxel_cloud->points.size();
              voxel_cloud->height = 1;

              std::stringstream name_cloud;
              name_cloud << model.id_ << ".pcd";
              pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr voxel_normal_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
              pcl::copyPointCloud(*voxel_cloud, *voxel_normal_cloud);
              pcl::copyPointCloud(*normals, *voxel_normal_cloud);

              voxel_normal_cloud->width = voxel_cloud->points.size();
              voxel_normal_cloud->height = 1;

              pcl::io::savePCDFile(name_cloud.str(), *voxel_normal_cloud);
            }
            else
            {
              pcl::visualization::PCLVisualizer vis_ ("voxel cloud");
              pcl::visualization::PointCloudColorHandlerCustom<PointInT> handler_outliers (voxel_cloud, 255, 0, 0);
              vis_.addPointCloud<PointInT> (voxel_cloud, handler_outliers, "cloud");
              vis_.addPointCloudNormals<PointInT, pcl::Normal>(voxel_cloud, normals, 30, 0.01f, "normals");
              vis_.addCoordinateSystem(0.2f);
              vis_.spin();

              voxel_cloud->width = voxel_cloud->points.size();
              voxel_cloud->height = 1;

              std::stringstream name_cloud;
              name_cloud << model.id_ << ".pcd";
              pcl::PointCloud<pcl::PointNormal>::Ptr voxel_normal_cloud (new pcl::PointCloud<pcl::PointNormal> ());
              pcl::copyPointCloud(*voxel_cloud, *voxel_normal_cloud);
              pcl::copyPointCloud(*normals, *voxel_normal_cloud);

              voxel_normal_cloud->width = voxel_cloud->points.size();
              voxel_normal_cloud->height = 1;

              pcl::io::savePCDFile(name_cloud.str(), *voxel_normal_cloud);
            }


            /*for (size_t i = 0; i < view_filenames.size (); i++)
            {
              std::stringstream view_file;
              view_file << model_path << "/" << view_filenames[i];
              typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
              pcl::io::loadPCDFile (view_file.str (), *cloud);

              std::cout << view_file.str () << std::endl;

              std::stringstream path_view;
              path_view << direc.str () << "/view_" << i << ".pcd";
              pcl::io::savePCDFileBinary (path_view.str (), *cloud);

              std::string file_replaced1 (view_file.str ());
              boost::replace_all (file_replaced1, view_prefix_, "pose");
              boost::replace_all (file_replaced1, ".pcd", ".txt");

              Eigen::Matrix4f pose;
              PersistenceUtils::readMatrixFromFile (file_replaced1, pose);

              std::cout << pose << std::endl;

              if(pose_files_order_ == 0) {
                std::cout << "Transpose..." << std::endl;
                Eigen::Matrix4f pose_trans = pose.transpose();
                pose = pose_trans;
                std::cout << pose << std::endl;
              }

              std::stringstream path_pose;
              path_pose << direc.str () << "/pose_" << i << ".txt";
              faat_pcl::rec_3d_framework::PersistenceUtils::writeMatrixToFile (path_pose.str (), pose);
            }

            loadOrGenerate (dir, model_path, model);*/

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
        getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths)
        {
          bf::directory_iterator end_itr;
          for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
          {
            //check if its a directory, then get models in it
            if (bf::is_directory (*itr))
            {
#if BOOST_FILESYSTEM_VERSION == 3
              std::string so_far = rel_path_so_far + (itr->path ().filename ()).string() + "/";
#else
              std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
#endif

              bf::path curr_path = itr->path ();

              if (isleafDirectory (curr_path))
              {
#if BOOST_FILESYSTEM_VERSION == 3
                std::string path = rel_path_so_far + (itr->path ().filename ()).string();
#else
                std::string path = rel_path_so_far + (itr->path ()).filename ();
#endif
                relative_paths.push_back (path);

              }
              else
              {
                getModelsInDirectory (curr_path, so_far, relative_paths);
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
          getModelsInDirectory (dir, start, files);

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
            std::cout << "Model path..." << path_model << std::endl;
            loadOrGenerate (training_dir, path_model, *m);

            models_->push_back (m);

            //std::cout << files[i] << std::endl;
          }
        }
      };
  }
}

#endif /* REC_FRAMEWORK_MESH_SOURCE_H_ */
