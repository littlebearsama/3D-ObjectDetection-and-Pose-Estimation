/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: correspondence_estimation.h 7208 2012-09-20 05:46:54Z rusu $
 *
 */

#ifndef FAAT_PCL_REGISTRATION_VOXELBASED_CORRESPONDENCE_ESTIMATION_H_
#define FAAT_PCL_REGISTRATION_VOXELBASED_CORRESPONDENCE_ESTIMATION_H_

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>
#include <pcl/registration/correspondence_estimation.h>

#include <v4rexternal/EDT/propagation_distance_field.h>
#include "faat_3d_rec_framework_defines.h"

namespace faat_pcl
{
  namespace rec_3d_framework
  {
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class PCL_EXPORTS VoxelBasedCorrespondenceEstimation : public pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<VoxelBasedCorrespondenceEstimation<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const VoxelBasedCorrespondenceEstimation<PointSource, PointTarget, Scalar> > ConstPtr;

        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::point_representation_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_transformed_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::tree_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::corr_name_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::target_indices_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::getClassName;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::initCompute;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::indices_;
        using pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar>::input_fields_;
        using pcl::PCLBase<PointSource>::deinitCompute;

        //typedef typename boost::shared_ptr<VoxelGridDistanceTransform<PointTarget> > VgdtPtr;
        typedef typename boost::shared_ptr<distance_field::PropagationDistanceField<PointTarget> > VgdtPtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        //typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        VgdtPtr vgdt_target_;
        float max_distance_;
        float max_color_distance_;
        float sigma_;

        /** \brief Empty constructor. */
        VoxelBasedCorrespondenceEstimation ()
        {
          corr_name_  = "CorrespondenceEstimation";
          max_color_distance_ = std::numeric_limits<float>::max();
          sigma_ = -1;
        }

        void
        setMaxColorDistance(float max_c, float sigma) {
          max_color_distance_ = max_c;
          sigma_ = sigma;
        }

        void setVoxelRepresentationTarget(VgdtPtr & v) {
          vgdt_target_ = v;
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        void
        determineCorrespondences (pcl::Correspondences &correspondences,
                                     double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        void
        determineReciprocalCorrespondences (pcl::Correspondences & /*correspondences*/,
        double /*max_distance = std::numeric_limits<double>::max ()*/) {
          std::cout << "Going to determine reciprocal correspondences..." << std::endl;
        }

        boost::shared_ptr< pcl::registration::CorrespondenceEstimationBase<PointSource, PointTarget, Scalar> >
        clone () const
        {
          Ptr copy (new VoxelBasedCorrespondenceEstimation<PointSource, PointTarget, Scalar> (*this));
          return (copy);
        }

        void
        setMaxCorrespondenceDistance(float d) {
          max_distance_ = d;
        }

        /** \brief Provide a simple mechanism to update the internal source cloud
          * using a given transformation. Used in registration loops.
          * \param[in] transform the transform to apply over the source cloud
          */
        bool
        updateSource (const Eigen::Matrix<Scalar, 4, 4> &transform)
        {
          if (!input_)
          {
            PCL_ERROR ("[pcl::registration::%s::updateSource] No input dataset given. Please specify the input source cloud using setInputSource.\n", getClassName ().c_str ());
            return (false);
          }

          // Check if XYZ or normal data is available
          int x_idx = -1, nx_idx = -1;

          for (int i = 0; i < int (input_fields_.size ()); ++i)
          {
            if (input_fields_[i].name == "x")
              x_idx = i;
            if (input_fields_[i].name == "normal_x")
              nx_idx = i;
          }

          // If no XYZ available, then return
          if (x_idx == -1)
            return (true);

          input_transformed_.reset (new PointCloudSource (*input_));
         
          int y_idx = x_idx + 1, z_idx = x_idx + 2, ny_idx = nx_idx + 1, nz_idx = nx_idx + 2;
          Eigen::Vector4f pt (0.0f, 0.0f, 0.0f, 1.0f), pt_t;
          Eigen::Matrix4f tr = transform.template cast<float> ();

          if (nx_idx != -1)
          {
            Eigen::Vector3f nt, nt_t;
            Eigen::Matrix3f rot = tr.block<3, 3> (0, 0);

            //pcl::transformPointCloudWithNormals<PointSource, Scalar> (*input_, *input_transformed_, transform);
            for (size_t i = 0; i < input_transformed_->size (); ++i)
            {
              uint8_t* pt_data = reinterpret_cast<uint8_t*> (&input_transformed_->points[i]);
              memcpy (&pt[0], pt_data + input_fields_[x_idx].offset, sizeof (float));
              memcpy (&pt[1], pt_data + input_fields_[y_idx].offset, sizeof (float));
              memcpy (&pt[2], pt_data + input_fields_[z_idx].offset, sizeof (float));

              if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2])) 
                continue;

              pt_t = tr * pt;

              memcpy (pt_data + input_fields_[x_idx].offset, &pt_t[0], sizeof (float));
              memcpy (pt_data + input_fields_[y_idx].offset, &pt_t[1], sizeof (float));
              memcpy (pt_data + input_fields_[z_idx].offset, &pt_t[2], sizeof (float));

              memcpy (&nt[0], pt_data + input_fields_[nx_idx].offset, sizeof (float));
              memcpy (&nt[1], pt_data + input_fields_[ny_idx].offset, sizeof (float));
              memcpy (&nt[2], pt_data + input_fields_[nz_idx].offset, sizeof (float));

              if (!pcl_isfinite (nt[0]) || !pcl_isfinite (nt[1]) || !pcl_isfinite (nt[2])) 
                continue;

              nt_t = rot * nt;

              memcpy (pt_data + input_fields_[nx_idx].offset, &nt_t[0], sizeof (float));
              memcpy (pt_data + input_fields_[ny_idx].offset, &nt_t[1], sizeof (float));
              memcpy (pt_data + input_fields_[nz_idx].offset, &nt_t[2], sizeof (float));
            }
          }
          else
          {
            //pcl::transformPointCloud<PointSource, Scalar> (*input_, *input_transformed_, transform);
            for (size_t i = 0; i < input_transformed_->size (); ++i)
            {
              uint8_t* pt_data = reinterpret_cast<uint8_t*> (&input_transformed_->points[i]);
              memcpy (&pt[0], pt_data + input_fields_[x_idx].offset, sizeof (float));
              memcpy (&pt[1], pt_data + input_fields_[y_idx].offset, sizeof (float));
              memcpy (&pt[2], pt_data + input_fields_[z_idx].offset, sizeof (float));

              if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2])) 
                continue;

              pt_t = tr * pt;

              memcpy (pt_data + input_fields_[x_idx].offset, &pt_t[0], sizeof (float));
              memcpy (pt_data + input_fields_[y_idx].offset, &pt_t[1], sizeof (float));
              memcpy (pt_data + input_fields_[z_idx].offset, &pt_t[2], sizeof (float));
            }
          }
          
          input_ = input_transformed_;
          return (true);
        }
     };
  }
}

#endif /* PCL_REGISTRATION_VOXELBASED_CORRESPONDENCE_ESTIMATION_H_ */
