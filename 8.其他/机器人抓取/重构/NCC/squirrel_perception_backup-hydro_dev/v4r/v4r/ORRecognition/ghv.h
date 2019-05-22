/*
 * hv_go_1.h
 *
 *  Created on: Feb 27, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_GHV_H_
#define FAAT_PCL_GHV_H_

#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include "hypotheses_verification.h"
//#include "pcl/recognition/3rdparty/metslib/mets.hh"
#include "v4rexternal/metslib/mets.hh"
#include <pcl/features/normal_3d.h>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <map>
#include <iostream>
#include <fstream>
#include "ghv_opt.h"
#include <stack>
#include <pcl/visualization/pcl_visualizer.h>
#include "v4r/ORUtils/common_data_structures.h"

#ifdef _MSC_VER
#ifdef FAAT_REC_EXPORTS
#define FAAT_REC_API __declspec(dllexport)
#else
#define FAAT_REC_API __declspec(dllimport)
#endif
#else
#define FAAT_REC_API
#endif

namespace faat_pcl
{

  /** \brief A hypothesis verification method proposed in
   * "A Global Hypotheses Verification Method for 3D Object Recognition", A. Aldoma and F. Tombari and L. Di Stefano and Markus Vincze, ECCV 2012
   * \author Aitor Aldoma
   * Extended with physical constraints and color information (see ICRA paper)
   */

  template<typename ModelT, typename SceneT>
    class FAAT_REC_API GHV : public faat_pcl::HypothesisVerification<ModelT, SceneT>
    {
      friend class GHVmove_manager<ModelT, SceneT>;
      friend class GHVSAModel<ModelT, SceneT>;

      static float sRGB_LUT[256];
      static float sXYZ_LUT[4000];

      //////////////////////////////////////////////////////////////////////////////////////////////
      void
      RGB2CIELAB (unsigned char R, unsigned char G, unsigned char B, float &L, float &A,float &B2)
      {
        if (sRGB_LUT[0] < 0)
        {
          for (int i = 0; i < 256; i++)
          {
            float f = static_cast<float> (i) / 255.0f;
            if (f > 0.04045)
              sRGB_LUT[i] = powf ((f + 0.055f) / 1.055f, 2.4f);
            else
              sRGB_LUT[i] = f / 12.92f;
          }

          for (int i = 0; i < 4000; i++)
          {
            float f = static_cast<float> (i) / 4000.0f;
            if (f > 0.008856)
              sXYZ_LUT[i] = static_cast<float> (powf (f, 0.3333f));
            else
              sXYZ_LUT[i] = static_cast<float>((7.787 * f) + (16.0 / 116.0));
          }
        }

        float fr = sRGB_LUT[R];
        float fg = sRGB_LUT[G];
        float fb = sRGB_LUT[B];

        // Use white = D65
        const float x = fr * 0.412453f + fg * 0.357580f + fb * 0.180423f;
        const float y = fr * 0.212671f + fg * 0.715160f + fb * 0.072169f;
        const float z = fr * 0.019334f + fg * 0.119193f + fb * 0.950227f;

        float vx = x / 0.95047f;
        float vy = y;
        float vz = z / 1.08883f;

        vx = sXYZ_LUT[int(vx*4000)];
        vy = sXYZ_LUT[int(vy*4000)];
        vz = sXYZ_LUT[int(vz*4000)];

        L = 116.0f * vy - 16.0f;
        if (L > 100)
          L = 100.0f;

        A = 500.0f * (vx - vy);
        if (A > 120)
          A = 120.0f;
        else if (A <- 120)
          A = -120.0f;

        B2 = 200.0f * (vy - vz);
        if (B2 > 120)
          B2 = 120.0f;
        else if (B2<- 120)
          B2 = -120.0f;
      }

    protected:
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::mask_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::scene_cloud_downsampled_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::scene_downsampled_tree_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::visible_models_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::visible_normal_models_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::visible_indices_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::complete_models_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::resolution_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::inliers_threshold_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::normals_set_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::requires_normals_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::occlusion_thres_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::occlusion_cloud_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::zbuffer_self_occlusion_resolution_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::scene_cloud_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::scene_sampled_indices_;
      using faat_pcl::HypothesisVerification<ModelT, SceneT>::zbuffer_scene_resolution_;

      template<typename PointT, typename NormalT>
        inline void
        extractEuclideanClustersSmooth (const typename pcl::PointCloud<PointT> &cloud, const typename pcl::PointCloud<NormalT> &normals, float tolerance,
                                        const typename pcl::search::Search<PointT>::Ptr &tree, std::vector<pcl::PointIndices> &clusters, double eps_angle,
                                        float curvature_threshold, unsigned int min_pts_per_cluster,
                                        unsigned int max_pts_per_cluster = (std::numeric_limits<int>::max) ())
        {

          if (tree->getInputCloud ()->points.size () != cloud.points.size ())
          {
            PCL_ERROR("[pcl::extractEuclideanClusters] Tree built for a different point cloud dataset\n");
            return;
          }
          if (cloud.points.size () != normals.points.size ())
          {
            PCL_ERROR("[pcl::extractEuclideanClusters] Number of points in the input point cloud different than normals!\n");
            return;
          }

          // Create a bool vector of processed point indices, and initialize it to false
          std::vector<bool> processed (cloud.points.size (), false);

          std::vector<int> nn_indices;
          std::vector<float> nn_distances;
          // Process all points in the indices vector
          int size = static_cast<int> (cloud.points.size ());
          for (int i = 0; i < size; ++i)
          {
            if (processed[i])
              continue;

            std::vector<unsigned int> seed_queue;
            int sq_idx = 0;
            seed_queue.push_back (i);

            processed[i] = true;

            while (sq_idx < static_cast<int> (seed_queue.size ()))
            {

              if (normals.points[seed_queue[sq_idx]].curvature > curvature_threshold)
              {
                sq_idx++;
                continue;
              }

              // Search for sq_idx
              if (!tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances))
              {
                sq_idx++;
                continue;
              }

              for (size_t j = 1; j < nn_indices.size (); ++j) // nn_indices[0] should be sq_idx
              {
                if (processed[nn_indices[j]]) // Has this point been processed before ?
                  continue;

                if (normals.points[nn_indices[j]].curvature > curvature_threshold)
                {
                  continue;
                }

                //processed[nn_indices[j]] = true;
                // [-1;1]

                double dot_p = normals.points[seed_queue[sq_idx]].normal[0] * normals.points[nn_indices[j]].normal[0]
                    + normals.points[seed_queue[sq_idx]].normal[1] * normals.points[nn_indices[j]].normal[1] + normals.points[seed_queue[sq_idx]].normal[2]
                    * normals.points[nn_indices[j]].normal[2];

                if (fabs (acos (dot_p)) < eps_angle)
                {
                  processed[nn_indices[j]] = true;
                  seed_queue.push_back (nn_indices[j]);
                }
              }

              sq_idx++;
            }

            // If this queue is satisfactory, add to the clusters
            if (seed_queue.size () >= min_pts_per_cluster && seed_queue.size () <= max_pts_per_cluster)
            {
              pcl::PointIndices r;
              r.indices.resize (seed_queue.size ());
              for (size_t j = 0; j < seed_queue.size (); ++j)
                r.indices[j] = seed_queue[j];

              std::sort (r.indices.begin (), r.indices.end ());
              r.indices.erase (std::unique (r.indices.begin (), r.indices.end ()), r.indices.end ());
              clusters.push_back (r); // We could avoid a copy by working directly in the vector
            }
          }
        }

      void computeClutterCueAtOnce ();

      virtual bool
      handlingNormals (boost::shared_ptr<GHVRecognitionModel<ModelT> > & recog_model, int i, bool is_planar_model, int object_models_size);

      virtual bool
      addModel (int i, boost::shared_ptr<GHVRecognitionModel<ModelT> > & recog_model);

      //Performs smooth segmentation of the scene cloud and compute the model cues
      virtual void
      initialize ();

      float regularizer_;
      pcl::PointCloud<pcl::Normal>::Ptr scene_normals_;
      bool scene_and_normals_set_from_outside_;
      bool ignore_color_even_if_exists_;
      std::vector<std::string> object_ids_;
      float color_sigma_ab_;
      float color_sigma_l_;
      std::vector<float> extra_weights_;

      //class attributes
      bool use_super_voxels_;
      typedef typename pcl::NormalEstimation<SceneT, pcl::Normal> NormalEstimator_;
      pcl::PointCloud<pcl::PointXYZL>::Ptr clusters_cloud_;
      int max_label_clusters_cloud_;
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr clusters_cloud_rgb_;
      pcl::PointCloud<pcl::Normal>::Ptr scene_normals_for_clutter_term_;

      std::vector<int> complete_cloud_occupancy_by_RM_;
      float res_occupancy_grid_;
      float w_occupied_multiple_cm_;

      std::vector<double> duplicates_by_RM_weighted_;
      std::vector<double> duplicates_by_RM_weighted_not_capped;
      std::vector<int> explained_by_RM_; //represents the points of scene_cloud_ that are explained by the recognition models
      std::vector<double> explained_by_RM_distance_weighted; //represents the points of scene_cloud_ that are explained by the recognition models
      std::vector<int> explained_by_RM_model; //id of the model explaining the point
      std::vector< std::stack<std::pair<int, float>, std::vector<std::pair<int, float> > > > previous_explained_by_RM_distance_weighted; //represents the points of scene_cloud_ that are explained by the recognition models
      std::vector<double> unexplained_by_RM_neighboorhods; //represents the points of scene_cloud_ that are not explained by the active hypotheses in the neighboorhod of the recognition models
      std::vector<boost::shared_ptr<GHVRecognitionModel<ModelT> > > recognition_models_;
      //std::vector<size_t> indices_;
      std::vector<bool> valid_model_;

      float duplicy_weight_test_;
      float duplicity_curvature_max_;

      float clutter_regularizer_;
      bool detect_clutter_;
      float radius_neighborhood_GO_;
      float radius_normals_;

      double previous_explained_value;
      double previous_duplicity_;
      int previous_duplicity_complete_models_;
      double previous_bad_info_;
      double previous_unexplained_;

      int max_iterations_; //max iterations without improvement
      GHVSAModel<ModelT, SceneT> best_seen_;
      float initial_temp_;
      bool use_replace_moves_;

      //conflict graph stuff
      int n_cc_;
      std::vector<std::vector<int> > cc_;

      std::vector<std::vector<boost::shared_ptr<GHVRecognitionModel<ModelT> > > > points_explained_by_rm_; //if inner size > 1, conflict

      int opt_type_;
      float active_hyp_penalty_;

      //smooth segmentation parameters
      double eps_angle_threshold_;
      int min_points_;
      float curvature_threshold_;
      float cluster_tolerance_;

      double
      getOccupiedMultipleW ()
      {
        return w_occupied_multiple_cm_;
      }

      void
      setPreviousBadInfo (double f)
      {
        previous_bad_info_ = f;
      }

      double
      getPreviousBadInfo ()
      {
        return previous_bad_info_;
      }

      void
      setPreviousExplainedValue (double v)
      {
        previous_explained_value = v;
      }

      void
      setPreviousDuplicity (double v)
      {
        previous_duplicity_ = v;
      }

      void
      setPreviousDuplicityCM (int v)
      {
        previous_duplicity_complete_models_ = v;
      }

      void
      setPreviousUnexplainedValue (double v)
      {
        previous_unexplained_ = v;
      }

      double
      getPreviousUnexplainedValue ()
      {
        return previous_unexplained_;
      }

      double
      getExplainedValue ()
      {
        return previous_explained_value;
      }

      double
      getDuplicity ()
      {
        return previous_duplicity_;
      }

      int
      getDuplicityCM ()
      {
        return previous_duplicity_complete_models_;
      }

      float
      getHypPenalty ()
      {
        return active_hyp_penalty_;
      }

      double
      getExplainedByIndices (std::vector<int> & indices, std::vector<float> & explained_values, std::vector<double> & explained_by_RM,
                             std::vector<int> & indices_to_update_in_RM_local);

      void
      getExplainedByRM (std::vector<double> & explained_by_rm)
      {
        explained_by_rm = explained_by_RM_distance_weighted;
      }

      void
      getUnexplainedByRM (std::vector<double> & explained_by_rm)
      {
        explained_by_rm = unexplained_by_RM_neighboorhods;
      }

      void
      updateUnexplainedVector (std::vector<int> & unexplained_, std::vector<float> & unexplained_distances, std::vector<double> & unexplained_by_RM,
                               std::vector<int> & explained, std::vector<int> & explained_by_RM, float val)
      {
        {

          double add_to_unexplained = 0.0;

          for (size_t i = 0; i < unexplained_.size (); i++)
          {

            bool prev_unexplained = (unexplained_by_RM[unexplained_[i]] > 0) && (explained_by_RM[unexplained_[i]] == 0);
            unexplained_by_RM[unexplained_[i]] += val * unexplained_distances[i];

            if (val < 0) //the hypothesis is being removed
            {
              if (prev_unexplained)
              {
                //decrease by 1
                add_to_unexplained -= unexplained_distances[i];
              }
            }
            else //the hypothesis is being added and unexplains unexplained_[i], so increase by 1 unless its explained by another hypothesis
            {
              if (explained_by_RM[unexplained_[i]] == 0)
                add_to_unexplained += unexplained_distances[i];
            }
          }

          for (size_t i = 0; i < explained.size (); i++)
          {
            if (val < 0)
            {
              //the hypothesis is being removed, check that there are no points that become unexplained and have clutter unexplained hypotheses
              if ((explained_by_RM[explained[i]] == 0) && (unexplained_by_RM[explained[i]] > 0))
              {
                add_to_unexplained += unexplained_by_RM[explained[i]]; //the points become unexplained
              }
            }
            else
            {
              //std::cout << "being added..." << add_to_unexplained << " " << unexplained_by_RM[explained[i]] << std::endl;
              if ((explained_by_RM[explained[i]] == 1) && (unexplained_by_RM[explained[i]] > 0))
              { //the only hypothesis explaining that point
                add_to_unexplained -= unexplained_by_RM[explained[i]]; //the points are not unexplained any longer because this hypothesis explains them
              }
            }
          }

          //std::cout << add_to_unexplained << std::endl;
          previous_unexplained_ += add_to_unexplained;
        }
      }

      void
      updateExplainedVector (std::vector<int> & vec, std::vector<float> & vec_float, std::vector<int> & explained_,
                             std::vector<double> & explained_by_RM_distance_weighted, float sign, int model_id);

      void
      updateCMDuplicity (std::vector<int> & vec, std::vector<int> & occupancy_vec, float sign);

      double
      getTotalExplainedInformation (std::vector<int> & explained_, std::vector<double> & explained_by_RM_distance_weighted, double * duplicity_);

      double
      getTotalBadInformation (std::vector<boost::shared_ptr<GHVRecognitionModel<ModelT> > > & recog_models)
      {
        double bad_info = 0;
        for (size_t i = 0; i < recog_models.size (); i++)
          bad_info += recog_models[i]->outliers_weight_ * static_cast<double> (recog_models[i]->bad_information_);

        return bad_info;
      }

      double
      getUnexplainedInformationInNeighborhood (std::vector<double> & unexplained, std::vector<int> & explained)
      {
        double unexplained_sum = 0.f;
        for (size_t i = 0; i < unexplained.size (); i++)
        {
          if (unexplained[i] > 0 && explained[i] == 0)
            unexplained_sum += unexplained[i];
        }

        return unexplained_sum;
      }

      mets::gol_type
      evaluateSolution (const std::vector<bool> & active, int changed);

      void
      SAOptimize (std::vector<int> & cc_indices, std::vector<bool> & sub_solution);

      void
      fill_structures (std::vector<int> & cc_indices, std::vector<bool> & sub_solution, GHVSAModel<ModelT, SceneT> & model);

      void
      clear_structures ();

      double
      countActiveHypotheses (const std::vector<bool> & sol);

      double
      countPointsOnDifferentPlaneSides (const std::vector<bool> & sol, bool print=false);

      boost::shared_ptr<GHVCostFunctionLogger<ModelT,SceneT> > cost_logger_;
      bool initial_status_;

      void
      computeRGBHistograms (std::vector<Eigen::Vector3f> & rgb_values, Eigen::MatrixXf & rgb,
                               int dim = 3, float min = 0.f, float max = 255.f, bool soft = false);

      void
      specifyRGBHistograms (Eigen::MatrixXf & src, Eigen::MatrixXf & dst, Eigen::MatrixXf & lookup, int dim = 3);

      void
      computeGSHistogram (std::vector<float> & hsv_values, Eigen::MatrixXf & histogram, int hist_size = 255);

      std::vector<faat_pcl::PlaneModel<ModelT> > planar_models_;
      std::map<int, int> model_to_planar_model_;

      bool use_histogram_specification_;
      typename boost::shared_ptr<pcl::octree::OctreePointCloudSearch<SceneT> > octree_scene_downsampled_;

      int min_contribution_;
      bool LS_short_circuit_;
      std::vector<std::vector<float> > points_one_plane_sides_;
      bool use_points_on_plane_side_;

      boost::function<void (const std::vector<bool> &, float, int)> visualize_cues_during_logger_;
      int visualize_go_cues_;

      void visualizeGOCues(const std::vector<bool> & active_solution, float cost, int times_eval);

      boost::shared_ptr<pcl::visualization::PCLVisualizer> vis_go_cues_;

      std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> models_smooth_faces_;

      void specifyColor(int i, Eigen::MatrixXf & lookup, boost::shared_ptr<GHVRecognitionModel<ModelT> > & recog_model);

      std::vector<float> scene_curvature_;
      std::vector<Eigen::Vector3f> scene_LAB_values_;
      std::vector<Eigen::Vector3f> scene_RGB_values_;
      std::vector<float> scene_GS_values_;
      int color_space_;
      float best_color_weight_;
      bool visualize_accepted_;
      typedef pcl::PointCloud<ModelT> CloudM;
      typedef pcl::PointCloud<SceneT> CloudS;
      typedef typename pcl::traits::fieldList<typename CloudS::PointType>::type FieldListS;
      typedef typename pcl::traits::fieldList<typename CloudM::PointType>::type FieldListM;

      float getCurvWeight(float p_curvature);

      bool use_normals_from_visible_;
      int max_threads_;

      std::vector<std::string> ply_paths_;
      std::vector<vtkSmartPointer <vtkTransform> > poses_ply_;

      float t_cues_, t_opt_;
      int number_of_visible_points_;

      float d_weight_for_bad_normals_;
      bool use_clutter_exp_;
      int multiple_assignment_penalize_by_one_;

    public:
      GHV () :
        faat_pcl::HypothesisVerification<ModelT, SceneT> ()
      {
        multiple_assignment_penalize_by_one_ = 2;
        resolution_ = 0.005f;
        max_iterations_ = 5000;
        regularizer_ = 1.f;
        radius_normals_ = 0.01f;
        initial_temp_ = 1000;
        detect_clutter_ = true;
        radius_neighborhood_GO_ = 0.03f;
        clutter_regularizer_ = 5.f;
        res_occupancy_grid_ = 0.005f;
        w_occupied_multiple_cm_ = 2.f;
        ignore_color_even_if_exists_ = true;
        color_sigma_ab_ = 0.25f;
        color_sigma_l_ = 0.5f;
        opt_type_ = 2;
        use_replace_moves_ = true;
        active_hyp_penalty_ = 0.f;
        requires_normals_ = false;
        initial_status_ = false;

        eps_angle_threshold_ = 0.25;
        min_points_ = 20;
        curvature_threshold_ = 0.04f;
        cluster_tolerance_ = 0.015f;
        use_super_voxels_ = false;
        use_histogram_specification_ = false;
        min_contribution_ = 0;
        LS_short_circuit_ = false;
        visualize_go_cues_ = 0; //0 - No visualization, 1 - accepted moves
        use_points_on_plane_side_ = true;
        color_space_ = 0;
        visualize_accepted_ = false;
        best_color_weight_ = 0.8f;

        duplicy_weight_test_ = 1.f;
        duplicity_curvature_max_ = 0.03f;
        use_normals_from_visible_ = false;
        max_threads_ = 1;
        d_weight_for_bad_normals_ = 0.1f;
        use_clutter_exp_ = false;
        scene_and_normals_set_from_outside_ = false;
      }

      void setSceneAndNormals(typename pcl::PointCloud<SceneT>::Ptr & scene,
                              typename pcl::PointCloud<pcl::Normal>::Ptr & scene_normals)
      {
         scene_cloud_downsampled_ = scene;
         scene_normals_ = scene_normals;
         scene_and_normals_set_from_outside_ = true;
      }

      void setUseClutterExp(bool b)
      {
          use_clutter_exp_ = b;
      }

      void setWeightForBadNormals(float w)
      {
          d_weight_for_bad_normals_ = w;
      }

      int getNumberOfVisiblePoints()
      {
          return number_of_visible_points_;
      }

      float getCuesComputationTime()
      {
          return t_cues_;
      }

      float getOptimizationTime()
      {
          return t_opt_;
      }

      void setPlyPathsAndPoses(std::vector<std::string> & ply_paths_for_go, std::vector<vtkSmartPointer <vtkTransform> > & poses_ply)
      {
          ply_paths_ = ply_paths_for_go;
          poses_ply_ = poses_ply;
      }

      void setMaxThreads(int t)
      {
          max_threads_ = t;
      }

      void setUseNormalsFromVisible(bool b)
      {
          use_normals_from_visible_ = b;
      }

      void setDuplicityWeightTest(float f)
      {
          duplicy_weight_test_ = f;
      }

      void setDuplicityMaxCurvature(float f)
      {
          duplicity_curvature_max_ = f;
      }

      void setBestColorWeight(float bcw)
      {
          best_color_weight_ = bcw;
      }

      void setVisualizeAccepted(bool b)
      {
          visualize_accepted_ = b;
      }

      //0 for LAB (specifying L), 1 for RGB (specifying all)
      void setColorSpace(int cs)
      {
          std::cout << "called color space" << cs << std::endl;
          color_space_ = cs;
      }

      void setUsePointsOnPlaneSides(bool b)
      {
          use_points_on_plane_side_ = b;
      }

      void setSmoothFaces(std::vector<pcl::PointCloud<pcl::PointXYZL>::Ptr> & aligned_smooth_faces)
      {
          models_smooth_faces_ = aligned_smooth_faces;
      }

      void setVisualizeGoCues(int v)
      {
        visualize_go_cues_ = v;
      }

      void setLSShortCircuit(bool b)
      {
          LS_short_circuit_ = b;
      }

      void setDuplicityCMWeight(float w)
      {
          w_occupied_multiple_cm_ = w;
      }

      void setHistogramSpecification(bool b)
      {
          use_histogram_specification_ = b;
      }

      void setNormalsForClutterTerm(pcl::PointCloud<pcl::Normal>::Ptr & normals)
      {
          scene_normals_for_clutter_term_ = normals;
      }

      void setUseSuperVoxels(bool use)
      {
        use_super_voxels_ = use;
      }
      void addPlanarModels(std::vector<faat_pcl::PlaneModel<ModelT> > & models);

      void
      setSmoothSegParameters (float t_eps, float curv_t, float dist_t, int min_points = 20)
      {
        eps_angle_threshold_ = t_eps;
        min_points_ = min_points;
        curvature_threshold_ = curv_t;
        cluster_tolerance_ = dist_t;
      }

      void
      setObjectIds (std::vector<std::string> & ids)
      {
        object_ids_ = ids;
      }

      void
      writeToLog (std::ofstream & of, bool all_costs_ = false)
      {
        cost_logger_->writeToLog (of);
        if (all_costs_)
        {
          cost_logger_->writeEachCostToLog (of);
        }
      }

      void
      setHypPenalty (float p)
      {
        active_hyp_penalty_ = p;
      }

      void setMinContribution(int min)
      {
          min_contribution_ = min;
      }

      void
      setInitialStatus (bool b)
      {
        initial_status_ = b;
      }

      /*void logCosts() {
       cost_logger_.reset(new CostFunctionLogger());
       }*/

      pcl::PointCloud<pcl::PointXYZL>::Ptr
      getSmoothClusters ()
      {
        return clusters_cloud_;
      }

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      getSmoothClustersRGBCloud ()
      {
        return clusters_cloud_rgb_;
      }

      float
      getResolution ()
      {
        return resolution_;
      }

      void
      setRequiresNormals (bool b)
      {
        requires_normals_ = b;
      }

      void
      setUseReplaceMoves (bool u)
      {
        use_replace_moves_ = u;
      }

      void
      setOptimizerType (int t)
      {
        opt_type_ = t;
      }

      void
      verify ();

      void
      setIgnoreColor (bool i)
      {
        ignore_color_even_if_exists_ = i;
      }

      void
      setColorSigma (float s)
      {
        color_sigma_ab_ = s;
        color_sigma_l_ = s;
      }

      void setColorSigma(float s_l, float s_ab)
      {
          color_sigma_ab_ = s_ab;
          color_sigma_l_ = s_l;
      }

      void
      setRadiusNormals (float r)
      {
        radius_normals_ = r;
      }

      void
      setMaxIterations (int i)
      {
        max_iterations_ = i;
      }

      void
      setInitialTemp (float t)
      {
        initial_temp_ = t;
      }

      void
      setRegularizer (float r)
      {
        regularizer_ = r;
        //w_occupied_multiple_cm_ = regularizer_;
      }

      void
      setRadiusClutter (float r)
      {
        radius_neighborhood_GO_ = r;
      }

      void
      setClutterRegularizer (float cr)
      {
        clutter_regularizer_ = cr;
      }

      void
      setDetectClutter (bool d)
      {
        detect_clutter_ = d;
      }

      //Same length as the recognition models
      void
      setExtraWeightVectorForInliers (std::vector<float> & weights)
      {
        extra_weights_.clear ();
        extra_weights_ = weights;
      }

      void
      getOutliersForAcceptedModels(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & outliers_cloud);

      void
      getOutliersForAcceptedModels(std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & outliers_cloud_color,
                                   std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr > & outliers_cloud_3d);

    };
}

#endif //FAAT_PCL_GHV_H_
