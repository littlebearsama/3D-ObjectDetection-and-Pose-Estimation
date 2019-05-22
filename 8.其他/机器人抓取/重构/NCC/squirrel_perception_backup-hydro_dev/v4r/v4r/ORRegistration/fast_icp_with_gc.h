/*
 * fast_icp_with_gc.h
 *
 *  Created on: Sep 8, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_FAST_ICP_WITH_GC_H_
#define FAAT_PCL_FAST_ICP_WITH_GC_H_

#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/registration/correspondence_estimation.h>
#include <boost/unordered_map.hpp>
#include <v4r/ORRegistration/uniform_sampling.h>
#include <v4r/OREdgeDetector/organized_edge_detection.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace faat_pcl
{
  namespace registration
  {
    template <typename PointT>
    class ICPNode
    {
      typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;
      typedef typename faat_pcl::registration::ICPNode<PointT> ICPNodeT;

        public:

        ICPNode(int l, bool root=false)
        {
          converged_ = false;
          is_root_ = root;
          level_ = l;
          color_weight_ = 0.f;
          reg_error_ = 0.f;
          osv_fraction_ = 0.f;
          fsv_fraction_ = 0.f;
          overlap_ = 0;
          incr_transform_.setIdentity();
          accum_transform_.setIdentity();
          childs_.resize(0);
        }

        void
        addChild(boost::shared_ptr<ICPNodeT> & c)
        {
          childs_.push_back(c);
        }

        boost::shared_ptr<ICPNode> parent_;
        bool is_root_;
        int level_; //equivalent to the ICP iteration
        Eigen::Matrix4f incr_transform_; //transform from parent to current node
        Eigen::Matrix4f accum_transform_;
        bool converged_; //wether the alignment path converged or not...
        std::vector< boost::shared_ptr<ICPNodeT> > childs_;
        float reg_error_;
        float color_weight_;
        int overlap_;
        float osv_fraction_;
        float fsv_fraction_;
        pcl::CorrespondencesPtr after_rej_correspondences_;
        PointTPtr src_keypoints_;
        pcl::PointCloud<pcl::Normal>::Ptr normal_src_keypoints_;
    };

    template <typename PointT>
    class FastIterativeClosestPointWithGC
    {

        typedef typename pcl::PointCloud<PointT>::Ptr PointTPtr;
        typedef typename pcl::PointCloud<PointT>::ConstPtr ConstPointTPtr;
      typedef typename faat_pcl::registration::ICPNode<PointT> ICPNodeT;
      void
      visualizeICPNodes(std::vector<boost::shared_ptr<ICPNodeT> > & nodes,
                          pcl::visualization::PCLVisualizer & vis,
                          std::string wname="icp nodes");

      bool
      filterHypothesesByPose(boost::shared_ptr<ICPNodeT> & current,
                                std::vector<boost::shared_ptr<ICPNodeT> > & nodes,
                                float trans_threshold);

      std::vector<float> evaluateHypotheses(PointTPtr & im1,
                                 PointTPtr & im_2,
                                 pcl::PointCloud<pcl::Normal>::Ptr & normals1,
                                 pcl::PointCloud<pcl::Normal>::Ptr & normals_2,
                                 Eigen::Matrix4f pose_2_to_1=Eigen::Matrix4f::Identity());

      //input_ and target_ need to be organized!
      PointTPtr input_, target_;
      pcl::PointCloud<pcl::Normal>::Ptr input_normals_, target_normals_;
      pcl::PointIndices input_indices_, target_indices_;

      int max_iterations_;
      float corr_dist_threshold_;
      float ransac_threshold_;
      int min_number_correspondences_;
      float gc_size_;
      float cx_, cy_, fl_;
      int max_keep_;
      float uniform_sampling_radius_;
      float ov_percentage_;
      std::vector<std::pair<float, Eigen::Matrix4f> > result_;
      bool use_normals_;
      bool standard_cg_;
      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > initial_poses_;
      bool no_cg_;

      inline void
      transformNormals (pcl::PointCloud<pcl::Normal>::Ptr & normals_cloud, Eigen::Matrix4f & transform)
      {
        for (size_t k = 0; k < normals_cloud->points.size (); k++)
        {
          Eigen::Vector3f nt (normals_cloud->points[k].normal_x, normals_cloud->points[k].normal_y, normals_cloud->points[k].normal_z);
          normals_cloud->points[k].normal_x = static_cast<float> (transform (0, 0) * nt[0] + transform (0, 1) * nt[1] + transform (0, 2) * nt[2]);
          normals_cloud->points[k].normal_y = static_cast<float> (transform (1, 0) * nt[0] + transform (1, 1) * nt[1] + transform (1, 2) * nt[2]);
          normals_cloud->points[k].normal_z = static_cast<float> (transform (2, 0) * nt[0] + transform (2, 1) * nt[1] + transform (2, 2) * nt[2]);
        }
      }

      inline void
      transformNormals (pcl::PointCloud<pcl::Normal>::Ptr & normals_cloud, pcl::PointCloud<pcl::Normal>::Ptr & normals_out, Eigen::Matrix4f & transform)
      {
        normals_out->points.resize(normals_cloud->points.size ());
        for (size_t k = 0; k < normals_cloud->points.size (); k++)
        {
          Eigen::Vector3f nt (normals_cloud->points[k].normal_x, normals_cloud->points[k].normal_y, normals_cloud->points[k].normal_z);
          normals_out->points[k].normal_x = static_cast<float> (transform (0, 0) * nt[0] + transform (0, 1) * nt[1] + transform (0, 2) * nt[2]);
          normals_out->points[k].normal_y = static_cast<float> (transform (1, 0) * nt[0] + transform (1, 1) * nt[1] + transform (1, 2) * nt[2]);
          normals_out->points[k].normal_z = static_cast<float> (transform (2, 0) * nt[0] + transform (2, 1) * nt[1] + transform (2, 2) * nt[2]);
        }
      }

      inline void
      uniformSamplingOfKeypoints (typename pcl::PointCloud<PointT>::Ptr & keypoint_cloud, std::vector<int> & indices_keypoints,
                                  std::vector<int> & indices, faat_pcl::registration::UniformSamplingSharedVoxelGrid<PointT> & keypoint_extractor)
      {

        //pcl::UniformSampling<PointT> keypoint_extractor;
        //keypoint_extractor.setRadiusSearch (radius);
        keypoint_extractor.setInputCloud (keypoint_cloud);
        pcl::PointCloud<int> keypoints;
        keypoint_extractor.compute (keypoints);

        indices.resize (keypoints.points.size ());
        for (size_t i = 0; i < keypoints.points.size (); i++)
        {
          indices[i] = indices_keypoints[keypoints[i]];
        }
      }

      inline void
      computeRGBEdges (typename pcl::PointCloud<PointT>::Ptr & cloud, std::vector<int> & indices)
      {
        faat_pcl::OrganizedEdgeFromRGB<PointT, pcl::Label> oed;
        oed.setDepthDisconThreshold (0.03f);
        oed.setRGBCannyLowThreshold (150.f);
        oed.setRGBCannyHighThreshold (200.f);
        oed.setEdgeType (faat_pcl::OrganizedEdgeBase<PointT, pcl::Label>::EDGELABEL_RGB_CANNY);
        oed.setInputCloud (cloud);

        pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
        std::vector<pcl::PointIndices> indices2;
        oed.compute (*labels, indices2);

        for (size_t j = 0; j < indices2.size (); j++)
        {
          if (indices2[j].indices.size () > 0)
          {
            for(size_t k=0; k < indices2[j].indices.size (); k++)
            {
              if(pcl_isfinite(cloud->points[indices2[j].indices[k]].z))
              indices.push_back(indices2[j].indices[k]);
            }
          }
        }
      }

      inline void
      getMask (typename pcl::PointCloud<PointT>::Ptr & cloud,
                std::vector<int> & indices,
                std::vector<bool> & mask)
      {
        mask.resize(cloud->points.size(), false);
        for (size_t j = 0; j < indices.size (); j++)
          mask[indices[j]] = true;
      }

      inline void
      getKeypointsWithMask (typename pcl::PointCloud<PointT>::Ptr & cloud,
                std::vector<int> & indices,
                std::vector<int> & roi_indices,
                std::vector<int> & indices_out)
      {
        std::vector<bool> mask;
        getMask(cloud, roi_indices, mask);
        for (size_t j = 0; j < indices.size (); j++)
        {
          if(mask[indices[j]])
          {
            indices_out.push_back(indices[j]);
          }
        }
      }

      public:
        FastIterativeClosestPointWithGC()
        {
          max_iterations_ = 5;
          corr_dist_threshold_ = 0.05f;
          gc_size_ = ransac_threshold_ = 0.01f;
          min_number_correspondences_ = 5;
          fl_ = 525.f;
          cx_ = 640.f;
          cy_ = 480.f;
          max_keep_ = 7;
          uniform_sampling_radius_ = 0.01f;
          ov_percentage_ = 0.5f;
          use_normals_ = false;
          standard_cg_ = true;
          no_cg_ = false;
        }

        void setNoCG(bool b)
        {
          no_cg_ = b;
        }

        void setGCSize(float t)
        {
          gc_size_ = t;
        }

        void setRansacThreshold(float t)
        {
          ransac_threshold_ = t;
        }

        void setInitialPoses(const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & poses)
        {
          initial_poses_ = poses;
        }

        void setInputAndTargetIndices(const pcl::PointIndices & input,
                                          const pcl::PointIndices & target)
        {
          input_indices_ = input;
          target_indices_ = target;
        }

        void useStandardCG(bool b)
        {
          standard_cg_ = b;
        }

        float getFinalTransformation(Eigen::Matrix4f & matrix) const
        {
          if(result_.size() == 0)
          {
            PCL_WARN("There are no result to be returned\n");
            matrix = Eigen::Matrix4f::Identity();
            return -1.f;
          }

          matrix = result_[0].second;
          return result_[0].first;
        }

        void setMaxCorrespondenceDistance(float f)
        {
          corr_dist_threshold_ = f;
        }

        void setUseNormals(bool b)
        {
          use_normals_ = b;
        }

        void setKeepMaxHypotheses(int k)
        {
          max_keep_ = k;
        }

        void setOverlapPercentage(float f)
        {
          ov_percentage_ = f;
        }

        void setMaximumIterations(int it)
        {
          max_iterations_ = it;
        }

        void setInputSource(PointTPtr cloud)
        {
          input_ = cloud;
        }

        void setInputTarget(PointTPtr cloud)
        {
          target_ = cloud;
        }

        void align(Eigen::Matrix4f initial_guess=Eigen::Matrix4f::Identity());
    };
  }
}
#endif /* FAST_ICP_WITH_GC_H_ */
