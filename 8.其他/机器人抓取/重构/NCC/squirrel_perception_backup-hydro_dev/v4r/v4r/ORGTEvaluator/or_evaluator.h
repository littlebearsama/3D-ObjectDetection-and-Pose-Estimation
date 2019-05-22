/*
 * or_evaluator.h
 *
 *  Created on: Mar 12, 2013
 *      Author: aitor
 */

#ifndef OR_EVALUATOR_H_
#define OR_EVALUATOR_H_

#include <v4r/ORFramework/source.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    namespace or_evaluator
    {

      inline void
      getModelsInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext, bool recursive = true)
      {
        bf::directory_iterator end_itr;
        for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
        {
          //check if its a directory, then get models in it
          if (bf::is_directory (*itr) && recursive)
          {
      #if BOOST_FILESYSTEM_VERSION == 3
            std::string so_far = rel_path_so_far + (itr->path ().filename ()).string () + "/";
      #else
            std::string so_far = rel_path_so_far + (itr->path ()).filename () + "/";
      #endif

            bf::path curr_path = itr->path ();
            getModelsInDirectory (curr_path, so_far, relative_paths, ext);
          }
          else
          {
            //check that it is a ply file and then add, otherwise ignore..
            std::vector<std::string> strs;
      #if BOOST_FILESYSTEM_VERSION == 3
            std::string file = (itr->path ().filename ()).string ();
      #else
            std::string file = (itr->path ()).filename ();
      #endif

            boost::split (strs, file, boost::is_any_of ("."));
            std::string extension = strs[strs.size () - 1];

            if (extension.compare (ext) == 0)
            {
      #if BOOST_FILESYSTEM_VERSION == 3
              std::string path = rel_path_so_far + (itr->path ().filename ()).string ();
      #else
              std::string path = rel_path_so_far + (itr->path ()).filename ();
      #endif

              relative_paths.push_back (path);
            }
          }
        }
      }

      inline void
      getDirectoriesInDirectory (bf::path & dir, std::string & rel_path_so_far, std::vector<std::string> & relative_paths, std::string & ext)
      {
        bf::directory_iterator end_itr;
        for (bf::directory_iterator itr (dir); itr != end_itr; ++itr)
        {
          //check if its a directory, then get models in it
          if (bf::is_directory (*itr))
          {
            #if BOOST_FILESYSTEM_VERSION == 3
                    std::string path = rel_path_so_far + (itr->path ().filename ()).string ();
            #else
                    std::string path = rel_path_so_far + (itr->path ()).filename ();
            #endif

            relative_paths.push_back (path);
          }
          else
          {

          }
        }
      }

      inline bool
      writeMatrixToFile (std::string file, Eigen::Matrix4f & matrix)
      {
        std::ofstream out (file.c_str ());
        if (!out)
        {
          std::cout << "Cannot open file.\n";
          return false;
        }

        for (size_t i = 0; i < 4; i++)
        {
          for (size_t j = 0; j < 4; j++)
          {
            out << matrix (i, j);
            if (!(i == 3 && j == 3))
              out << " ";
          }
        }
        out.close ();

        return true;
      }

      inline bool
      readMatrixFromFile2 (std::string file, Eigen::Matrix4f & matrix)
      {

        std::ifstream in;
        in.open (file.c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        std::string line (linebuf);
        std::vector < std::string > strs_2;
        boost::split (strs_2, line, boost::is_any_of (" "));

        for (int i = 0; i < 16; i++)
        {
          matrix (i / 4, i % 4) = static_cast<float> (atof (strs_2[i].c_str ()));
        }

        return true;
      }

      inline bool
      readFloatFromFile (std::string file, float& value)
      {

        std::ifstream in;
        in.open (file.c_str (), std::ifstream::in);

        char linebuf[1024];
        in.getline (linebuf, 1024);
        value = static_cast<float> (atof (linebuf));

        return true;
      }

      inline bool
      writeFloatToFile (std::string file, float value)
      {
        std::ofstream out (file.c_str ());
        if (!out)
        {
          std::cout << "Cannot open file.\n";
          return false;
        }

        out << value;
        out.close ();

        return true;
      }

      struct RecognitionStatistics
      {
        int TP_, FP_, FN_;
      };

      class PoseStatistics
      {
        public:
            std::vector<float> centroid_distances_;
            std::vector<float> rotation_error_;
      };

      template<typename ModelPointT>
      struct GTModel
      {
        boost::shared_ptr<faat_pcl::rec_3d_framework::Model<ModelPointT> > model_;
        Eigen::Matrix4f transform_;
        float occlusion_;
        int inst_;
      };

      class RecognitionStatisticsResults
      {
        public:
            float precision_;
            float recall_;
            float fscore_;
            RecognitionStatistics rs_;
      };

      /*
       * Class to perform generic evaluation of recognition algorithms on
       * given datasets following the appropiate conventions (see below).
       */
      template<typename ModelPointT, typename SceneId=std::string>
        class OREvaluator
        {
        private:
          typedef typename pcl::PointCloud<ModelPointT>::ConstPtr ConstPointInTPtr;
          typedef faat_pcl::rec_3d_framework::Model<ModelPointT> ModelT;
          typedef boost::shared_ptr<ModelT> ModelTPtr;
          typedef GTModel<ModelPointT> GTModelT;
          typedef boost::shared_ptr<GTModelT> GTModelTPtr;

          bool gt_data_loaded_;
          std::string models_dir_;
          //gt_dir_ should contain per each scene (scenename.pcd) in the dataset as many files
          //as object instances are to be found in the scene:
          //      - scenename_modelname_instance.txt (i.e, rs10_cheff_0.txt)
          // the presence of a file with modelname indicates the presence of the object in the scene
          // the file contains the pose of the object as a 4x4 matrix (row-wise)
          //additionally, occlusion file can be added for each object instance like this:
          //      - scenename_occlusion_modelname_instance.txt (i.e, rs10_occlusion_cheff_0.txt)
          std::string gt_dir_;
          std::string scenes_dir_;

          typename std::map<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > > recognition_results_;
          typename std::map<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > > transforms_;

          /*typename std::map<SceneId, boost::shared_ptr<std::vector<ModelTPtr> > > recognition_results_upper_bound_;
          typename std::map<SceneId, boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > > transforms_upper_bound_;*/

          std::map<SceneId, std::map<std::string, std::vector<GTModelTPtr> > > gt_data_;
          std::map<SceneId, RecognitionStatistics> scene_statistics_;
          std::map<SceneId, PoseStatistics> pose_statistics_;
          std::map<SceneId, bool> ignore_list_;

          typedef std::pair<float, bool> occ_tp;
          std::vector< occ_tp > occlusion_results_;

          std::string scene_file_extension_;
          std::string model_file_extension_;

          typename boost::shared_ptr<Source<ModelPointT> > source_;
          bool check_pose_;
          bool check_rotation_;
          float max_centroid_diff_;
          float max_rotation_;
          bool replace_model_ext_;
          float max_occlusion_;
          bool use_max_occlusion_;
          bool checkLoaded();

          RecognitionStatisticsResults rsr_;
        public:

          void copyToDirectory(std::string & out_dir);

          void getRecognitionStatisticsResults(RecognitionStatisticsResults & r)
          {
              r = rsr_;
          }

          void setIgnoreList(std::map<SceneId, bool> & list)
          {
              ignore_list_ = list;
          }

          void useMaxOcclusion(bool s)
          {
            use_max_occlusion_ = s;
          }
          void setMaxOcclusion(float m)
          {
            max_occlusion_ = m;
          }

          void setMaxCentroidDistance(float f)
          {
              max_centroid_diff_ = f;
          }

          void setCheckPose(bool f)
          {
              check_pose_ = f;
          };

          void setCheckRotation(bool f)
          {
              check_rotation_ = f;
          }

          //in degrees
          void setMaxRotation(float f)
          {
                max_rotation_ = f;
          }

          OREvaluator ();

          ~OREvaluator ()
          {

          }

          void
          setDataSource (typename boost::shared_ptr<Source<ModelPointT> > source)
          {
            source_ = source;
          }

          void updateGT(std::string & sid, std::string & m_id, Eigen::Matrix4f & pose);

          void refinePoses ();

          void computeOcclusionValues(bool only_if_not_exist=false, bool do_icp_=true);

          void
          loadGTData ();
          void
          addRecognitionResults (std::vector<std::string> & model_ids, std::vector<Eigen::Matrix4f> & poses);

          void
          addRecognitionResults (SceneId & id,
                                    boost::shared_ptr<std::vector<ModelTPtr> > & results,
                                    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms);

          void
          classify (SceneId & id,
                    boost::shared_ptr<std::vector<ModelTPtr> > & results,
                    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms,
                    std::vector<bool> & correct);

          void
          getGroundTruthModelsAndPoses (SceneId & id,
                                    boost::shared_ptr<std::vector<ModelTPtr> > & results,
                                    boost::shared_ptr<std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > > & transforms);

          void
          computeStatistics();

          void
          computeStatisticsUpperBound ();

          void
          visualizeGroundTruth(pcl::visualization::PCLVisualizer & vis, SceneId & s_id, int viewport = -1, bool clear=true, std::string cloud_name="gt_model_cloud");

          void
          getGroundTruthPointCloud(SceneId & s_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & gt_cloud, float model_res = -1.f);


          void
          getModelsForScene(SceneId & s_id,
                              std::vector< typename pcl::PointCloud<ModelPointT>::Ptr > & model_clouds,
                              float res = 0.003f);

          void saveStatistics(std::string & out_file);

          void savePoseStatistics(std::string & out_file);

          void savePoseStatisticsRotation(std::string & out_file);

          void saveRecognitionResults(std::string & out_dir);

          void
          setModelsDir (std::string & dir)
          {
            models_dir_ = dir;
          }

          void
          setGTDir (std::string & dir)
          {
            gt_dir_ = dir;
          }

          void
          setScenesDir (std::string & dir)
          {
            scenes_dir_ = dir;
          }

          void
          setModelFileExtension(std::string aa)
          {
            model_file_extension_ = aa;
          }

          void
          setReplaceModelExtension(bool b)
          {
            replace_model_ext_ = b;
          }

          int countTotalNumberOfObjectsSequenceWise();
        };
    }
  }
}

#endif /* OR_EVALUATOR_H_ */
