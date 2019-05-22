/*
 * source.h
 *
 *  Created on: Mar 9, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_REC_FRAMEWORK_VIEWS_SOURCE_H_
#define FAAT_PCL_REC_FRAMEWORK_VIEWS_SOURCE_H_

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>
#include "persistence_utils.h"
#include <pcl/filters/voxel_grid.h>
#include <v4rexternal/EDT/propagation_distance_field.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/regex.hpp>

namespace bf = boost::filesystem;

namespace faat_pcl
{
  namespace rec_3d_framework
  {

    /**
     * \brief Model representation
     * \author Aitor Aldoma
     */

    template<typename PointT>
    class Model
    {
      typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;
      typedef typename pcl::PointCloud<PointT>::ConstPtr PointTPtrConst;
      Eigen::Vector4f centroid_;
      bool centroid_computed_;

    public:
      boost::shared_ptr<std::vector<PointTPtr> > views_;
      boost::shared_ptr< std::vector<pcl::PointIndices> > indices_;
      boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > poses_;
      boost::shared_ptr<std::vector<float> > self_occlusions_;
      std::string id_;
      std::string class_;
      PointTPtr assembled_;
      pcl::PointCloud<pcl::Normal>::Ptr normals_assembled_;
      std::vector<std::string> view_filenames_;
      typename std::map<float, PointTPtrConst> voxelized_assembled_;
      typename std::map<float, pcl::PointCloud<pcl::Normal>::ConstPtr> normals_voxelized_assembled_;
      //typename boost::shared_ptr<VoxelGridDistanceTransform<PointT> > dist_trans_;
      typename boost::shared_ptr<distance_field::PropagationDistanceField<PointT> > dist_trans_;

      pcl::PointCloud<pcl::PointXYZL>::Ptr faces_cloud_labels_;
      typename std::map<float, pcl::PointCloud<pcl::PointXYZL>::Ptr> voxelized_assembled_labels_;
      bool flip_normals_based_on_vp_;

      Model()
      {
        centroid_computed_ = false;
        flip_normals_based_on_vp_ = false;
      }

      bool getFlipNormalsBasedOnVP()
      {
          return flip_normals_based_on_vp_;
      }

      void setFlipNormalsBasedOnVP(bool b)
      {
          flip_normals_based_on_vp_ = b;
      }

      Eigen::Vector4f getCentroid()
      {
        if(centroid_computed_)
        {
          return centroid_;
        }

        //compute
        pcl::compute3DCentroid(*assembled_, centroid_);
        centroid_[3] = 0.f;
        centroid_computed_ = true;
        return centroid_;
      }

      bool
      operator== (const Model &other) const
      {
        return (id_ == other.id_) && (class_ == other.class_);
      }

      void computeNormalsAssembledCloud(float radius_normals) {
        typename pcl::search::KdTree<PointT>::Ptr normals_tree (new pcl::search::KdTree<PointT>);
        typedef typename pcl::NormalEstimationOMP<PointT, pcl::Normal> NormalEstimator_;
        NormalEstimator_ n3d;
        normals_assembled_.reset (new pcl::PointCloud<pcl::Normal> ());
        normals_tree->setInputCloud (assembled_);
        n3d.setRadiusSearch (radius_normals);
        n3d.setSearchMethod (normals_tree);
        n3d.setInputCloud (assembled_);
        n3d.compute (*normals_assembled_);
      }

      pcl::PointCloud<pcl::PointXYZL>::Ptr
      getAssembledSmoothFaces (float resolution)
      {
        if(resolution <= 0)
          return faces_cloud_labels_;

        typename std::map<float, pcl::PointCloud<pcl::PointXYZL>::Ptr>::iterator it = voxelized_assembled_labels_.find (resolution);
        if (it == voxelized_assembled_labels_.end ())
        {
          pcl::PointCloud<pcl::PointXYZL>::Ptr voxelized (new pcl::PointCloud<pcl::PointXYZL>);
          pcl::VoxelGrid<pcl::PointXYZL> grid_;
          grid_.setInputCloud (faces_cloud_labels_);
          grid_.setLeafSize (resolution, resolution, resolution);
          grid_.setDownsampleAllData(true);
          grid_.filter (*voxelized);

          voxelized_assembled_labels_[resolution] = voxelized;
          return voxelized;
        }

        return it->second;
      }

      PointTPtrConst
      getAssembled (float resolution)
      {
        if(resolution <= 0)
          return assembled_;

        typename std::map<float, PointTPtrConst>::iterator it = voxelized_assembled_.find (resolution);
        if (it == voxelized_assembled_.end ())
        {
          PointTPtr voxelized (new pcl::PointCloud<PointT>);
          pcl::VoxelGrid<PointT> grid_;
          grid_.setInputCloud (assembled_);
          grid_.setLeafSize (resolution, resolution, resolution);
          grid_.setDownsampleAllData(true);
          grid_.filter (*voxelized);

          PointTPtrConst voxelized_const (new pcl::PointCloud<PointT> (*voxelized));
          voxelized_assembled_[resolution] = voxelized_const;
          return voxelized_const;
        }

        return it->second;
      }

      pcl::PointCloud<pcl::Normal>::ConstPtr
      getNormalsAssembled (float resolution)
      {
        if(resolution <= 0)
          return normals_assembled_;

        typename std::map<float, pcl::PointCloud<pcl::Normal>::ConstPtr >::iterator it = normals_voxelized_assembled_.find (resolution);
        if (it == normals_voxelized_assembled_.end ())
        {
          pcl::PointCloud<pcl::PointNormal>::Ptr voxelized (new pcl::PointCloud<pcl::PointNormal>);
          pcl::PointCloud<pcl::PointNormal>::Ptr assembled_with_normals (new pcl::PointCloud<pcl::PointNormal>);
          assembled_with_normals->points.resize(assembled_->points.size());
          assembled_with_normals->width = assembled_->width;
          assembled_with_normals->height = assembled_->height;

          for(size_t i=0; i < assembled_->points.size(); i++) {
            assembled_with_normals->points[i].getVector4fMap() = assembled_->points[i].getVector4fMap();
            assembled_with_normals->points[i].getNormalVector4fMap() = normals_assembled_->points[i].getNormalVector4fMap();
          }

          pcl::VoxelGrid<pcl::PointNormal> grid_;
          grid_.setInputCloud (assembled_with_normals);
          grid_.setLeafSize (resolution, resolution, resolution);
          grid_.setDownsampleAllData(true);
          grid_.filter (*voxelized);

          pcl::PointCloud<pcl::Normal>::Ptr voxelized_const (new pcl::PointCloud<pcl::Normal> ());
          voxelized_const->points.resize(voxelized->points.size());
          voxelized_const->width = voxelized->width;
          voxelized_const->height = voxelized->height;

          for(size_t i=0; i < voxelized_const->points.size(); i++) {
            voxelized_const->points[i].getNormalVector4fMap() = voxelized->points[i].getNormalVector4fMap();
          }

          normals_voxelized_assembled_[resolution] = voxelized_const;
          return voxelized_const;
        }

        return it->second;
      }

      void
      createVoxelGridAndDistanceTransform(float res) {
        PointTPtrConst assembled (new pcl::PointCloud<PointT> ());
        assembled = getAssembled(0.001f);
        dist_trans_.reset(new distance_field::PropagationDistanceField<PointT>(res));
        dist_trans_->setInputCloud(assembled);
        dist_trans_->compute();
      }

      void
      getVGDT(boost::shared_ptr<distance_field::PropagationDistanceField<PointT> > & dt) {
        dt = dist_trans_;
      }
    };

    /**
     * \brief Abstract data source class, manages filesystem, incremental training, etc.
     * \author Aitor Aldoma
     */

    template<typename PointInT>
    class Source
    {

    protected:
      typedef Model<PointInT> ModelT;
      typedef boost::shared_ptr<ModelT> ModelTPtr;
      std::string path_;
      boost::shared_ptr<std::vector<ModelTPtr> > models_;
      float model_scale_;
      bool filter_duplicate_views_;
      bool load_views_;
      float radius_normals_;
      bool compute_normals_;
      bool load_into_memory_;

      void
      getIdAndClassFromFilename (std::string & filename, std::string & id, std::string & classname)
      {

        std::vector < std::string > strs;
        boost::split (strs, filename, boost::is_any_of ("/\\"));
        std::string name = strs[strs.size () - 1];

        std::stringstream ss;
        for (int i = 0; i < (static_cast<int> (strs.size ()) - 1); i++)
        {
          ss << strs[i];
          if (i != (static_cast<int> (strs.size ()) - 1))
          ss << "/";
        }

        classname = ss.str ();
        id = name.substr (0, name.length () - 4);
      }

      void
      createTrainingDir (std::string & training_dir)
      {
        bf::path trained_dir = training_dir;
        if (!bf::exists (trained_dir))
        bf::create_directory (trained_dir);
      }

      void
      getViewsFilenames (bf::path & path_with_views,
                           std::vector<std::string> & view_filenames,
                           const std::string & prefix)
      {

        std::stringstream filter_str;
        filter_str << ".*" << prefix  << ".*.pcd";
        const boost::regex my_filter( filter_str.str() );

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

            boost::smatch what;
            if( !boost::regex_match( file, what, my_filter ) ) continue;

            /*boost::split (strs, file, boost::is_any_of ("."));
            boost::split (strs_, file, boost::is_any_of ("_"));

            std::string extension = strs[strs.size () - 1];

            if (extension == "pcd" && (strs_[0].compare (view_prefix_) == 0))
            {*/
#if BOOST_FILESYSTEM_VERSION == 3
              view_filenames.push_back ((itr->path ().filename ()).string());
#else
              view_filenames.push_back ((itr->path ()).filename ());
#endif

              number_of_views++;
            //}
          }
        }
      }

      void
      createClassAndModelDirectories (std::string & training_dir, std::string & class_str, std::string & id_str)
      {
        std::vector < std::string > strs;
        boost::split (strs, class_str, boost::is_any_of ("/\\"));

        std::stringstream ss;
        ss << training_dir << "/";
        for (size_t i = 0; i < strs.size (); i++)
        {
          ss << strs[i] << "/";
          bf::path trained_dir = ss.str ();
          if (!bf::exists (trained_dir))
          bf::create_directory (trained_dir);
        }

        ss << id_str;
        bf::path trained_dir = ss.str ();
        if (!bf::exists (trained_dir))
        bf::create_directory (trained_dir);
      }

    public:

      Source() {
        load_views_ = true;
        compute_normals_ = false;
        load_into_memory_ = true;
      }

      void
      setLoadIntoMemory(bool b)
      {
        load_into_memory_ = b;
      }

      bool
      getLoadIntoMemory()
      {
        return load_into_memory_;
      }

      virtual void
      loadInMemorySpecificModelAndView(std::string & dir, ModelT & model, int view_id)
      {
        PCL_ERROR("This function is not implemented in this Source class\n");
      }

      virtual void
      loadInMemorySpecificModel(std::string & dir, ModelT & model)
      {
        PCL_ERROR("This function is not implemented in this Source class\n");
      }

      float
      getScale ()
      {
        return model_scale_;
      }

      void
      setRadiusNormals(float r) {
        radius_normals_ = r;
        compute_normals_ = true;
      }

      void
      setModelScale (float s)
      {
        model_scale_ = s;
      }

      void setFilterDuplicateViews(bool f) {
        filter_duplicate_views_ = f;
        std::cout << "setting filter duplicate views to " << f << std::endl;
      }

      void
      getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
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
            getModelsInDirectory (curr_path, so_far, relative_paths, ext);
          }
          else
          {
            //check that it is a ply file and then add, otherwise ignore..
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

      void
      voxelizeAllModels (float resolution)
      {
        for (size_t i = 0; i < models_->size (); i++)
        {
          models_->at (i)->getAssembled (resolution);
          if(compute_normals_)
            models_->at (i)->getNormalsAssembled (resolution);
        }
      }

      /**
       * \brief Generate model representation
       */
      virtual void
      generate (std::string & training_dir)=0;

      /**
       * \brief Get the generated model
       */
      boost::shared_ptr<std::vector<ModelTPtr> >
      getModels ()
      {
        return models_;
      }

      bool
      getModelById (std::string & model_id, ModelTPtr & m)
      {

        typename std::vector<ModelTPtr>::iterator it = models_->begin ();
        while (it != models_->end ())
        {
          if (model_id.compare ((*it)->id_) == 0)
          {
            m = *it;
            return true;
          } else
          {
            it++;
          }
        }

        return false;
      }
      boost::shared_ptr<std::vector<ModelTPtr> >
      getModels (std::string & model_id)
      {

        typename std::vector<ModelTPtr>::iterator it = models_->begin ();
        while (it != models_->end ())
        {
          if (model_id.compare ((*it)->id_) != 0)
          {
            it = models_->erase (it);
          }
          else
          {
            it++;
          }
        }

        return models_;
      }

      void getFeaturesFromFile(std::string filename, std::vector<float> feature_vector)
      {
          if (!bf::exists (filename))
          {
              std::cout << "Cannot find a file under " << filename << ". Features cannot be loaded. " << std::endl;
          }
          else
          {
            //boost::numeric::ublas::matrix<double> m;
            //std::ifstream s(filename);

            //if (!s >> m)
            //{
            //    std::cout << "Failed to write to matrix" << std::endl;
            //    return 1;
            //}
          }
      }

      bool
      modelAlreadyTrained (ModelT m, std::string & base_dir, std::string & descr_name)
      {
        std::stringstream dir;
        dir << base_dir << "/" << m.class_ << "/" << m.id_ << "/" << descr_name;
        bf::path desc_dir = dir.str ();
        if (bf::exists (desc_dir))
        {
          std::cout << dir.str () << " exists..." << std::endl;
          return true;
        }

        std::cout << dir.str () << " does not exist..." << std::endl;
        return false;
      }

      std::string
      getModelDescriptorDir (ModelT m, std::string & base_dir, std::string & descr_name)
      {
        std::stringstream dir;
        dir << base_dir << "/" << m.class_ << "/" << m.id_ << "/" << descr_name;
        return dir.str ();
      }

      std::string
      getModelDirectory (ModelT m, std::string & base_dir)
      {
        std::stringstream dir;
        dir << base_dir << "/" << m.class_ << "/" << m.id_;
        return dir.str ();
      }

      std::string
      getModelClassDirectory (ModelT m, std::string & base_dir)
      {
        std::stringstream dir;
        dir << base_dir << "/" << m.class_;
        return dir.str ();
      }

      void
      removeDescDirectory (ModelT m, std::string & base_dir, std::string & descr_name)
      {
        std::string dir = getModelDescriptorDir (m, base_dir, descr_name);

        bf::path desc_dir = dir;
        if (bf::exists (desc_dir))
        bf::remove_all (desc_dir);
      }

      void
      setPath (std::string & path)
      {
        path_ = path;
      }

      void setLoadViews(bool load) {
        load_views_ = load;
      }

      void
      createVoxelGridAndDistanceTransform(float res = 0.001f) {
        for (size_t i = 0; i < models_->size (); i++)
        {
          models_->at (i)->createVoxelGridAndDistanceTransform (res);
        }
      }
    };
  }
}

#endif /* REC_FRAMEWORK_VIEWS_SOURCE_H_ */
