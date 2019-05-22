/*
 * uniform_sampling.h
 *
 *  Created on: Aug 9, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_UNIFORM_SAMPLING_H_
#define FAAT_PCL_UNIFORM_SAMPLING_H_

#include <pcl/keypoints/keypoint.h>

namespace faat_pcl
{
  namespace registration
  {
    template <typename PointInT>
    class UniformSamplingSharedVoxelGrid: public pcl::Keypoint<PointInT, int>
    {
    protected:
      typedef typename pcl::Keypoint<PointInT, int>::PointCloudIn PointCloudIn;
      typedef typename pcl::Keypoint<PointInT, int>::PointCloudOut PointCloudOut;

      using pcl::Keypoint<PointInT, int>::name_;
      using pcl::Keypoint<PointInT, int>::input_;
      using pcl::Keypoint<PointInT, int>::indices_;
      using pcl::Keypoint<PointInT, int>::search_radius_;
      using pcl::Keypoint<PointInT, int>::getClassName;

      /** \brief Simple structure to hold an nD centroid and the number of points in a leaf. */
      struct Leaf
      {
        Leaf () : idx (-1) { }
        int idx;
      };

      /** \brief The 3D grid leaves. */
      boost::unordered_map<size_t, Leaf> leaves_;

      /** \brief The size of a leaf. */
      Eigen::Vector4f leaf_size_;

      /** \brief Internal leaf sizes stored as 1/leaf_size_ for efficiency reasons. */
      Eigen::Array4f inverse_leaf_size_;

      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;

      /** \brief Downsample a Point Cloud using a voxelized grid approach
        * \param output the resultant point cloud message
        */
      void
      detectKeypoints (PointCloudOut &output)
      {
        // Has the input dataset been set already?
        if (!input_)
        {
          PCL_WARN ("[pcl::%s::detectKeypoints] No input dataset given!\n", getClassName ().c_str ());
          output.width = output.height = 0;
          output.points.clear ();
          return;
        }

        output.height       = 1;                    // downsampling breaks the organized structure
        output.is_dense     = true;                 // we filter out invalid points

        //Eigen::Vector4f min_p, max_p;
        // Get the minimum and maximum dimensions
        pcl::getMinMax3D<PointInT>(*input_, min_p, max_p);

        if(share_voxel_grid_)
        {
          //adapt min and max points
          float val_min_0 = std::abs(min_p_shared_[0] - min_p[0]);
          float val_min_1 =  std::abs(min_p_shared_[1] - min_p[1]);
          float val_min_2 =  std::abs(min_p_shared_[2] - min_p[2]);

          //assert((val_min_0 - static_cast<int> (floor (val_min_0 * inverse_leaf_size_[0]))) < leaf_size_[0]);
          min_p[0] -= val_min_0 - static_cast<int> (floor (val_min_0 * inverse_leaf_size_[0]));
          min_p[1] -= val_min_1 - static_cast<int> (floor (val_min_1 * inverse_leaf_size_[1]));
          min_p[2] -= val_min_2 - static_cast<int> (floor (val_min_2 * inverse_leaf_size_[2]));
        }

        // Compute the minimum and maximum bounding box values
        min_b_[0] = static_cast<int> (floor (min_p[0] * inverse_leaf_size_[0]));
        max_b_[0] = static_cast<int> (floor (max_p[0] * inverse_leaf_size_[0]));
        min_b_[1] = static_cast<int> (floor (min_p[1] * inverse_leaf_size_[1]));
        max_b_[1] = static_cast<int> (floor (max_p[1] * inverse_leaf_size_[1]));
        min_b_[2] = static_cast<int> (floor (min_p[2] * inverse_leaf_size_[2]));
        max_b_[2] = static_cast<int> (floor (max_p[2] * inverse_leaf_size_[2]));

        // Compute the number of divisions needed along all axis
        div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones ();
        div_b_[3] = 0;

        // Clear the leaves
        leaves_.clear ();

        // Set up the division multiplier
        divb_mul_ = Eigen::Vector4i (1, div_b_[0], div_b_[0] * div_b_[1], 0);


        // First pass: build a set of leaves with the point index closest to the leaf center
        for (size_t cp = 0; cp < indices_->size (); ++cp)
        {
          if (!input_->is_dense)
            // Check if the point is invalid
            if (!pcl_isfinite (input_->points[(*indices_)[cp]].x) ||
                !pcl_isfinite (input_->points[(*indices_)[cp]].y) ||
                !pcl_isfinite (input_->points[(*indices_)[cp]].z))
              continue;

          Eigen::Vector4i ijk = Eigen::Vector4i::Zero ();
          ijk[0] = static_cast<int> (floor (input_->points[(*indices_)[cp]].x * inverse_leaf_size_[0]));
          ijk[1] = static_cast<int> (floor (input_->points[(*indices_)[cp]].y * inverse_leaf_size_[1]));
          ijk[2] = static_cast<int> (floor (input_->points[(*indices_)[cp]].z * inverse_leaf_size_[2]));

          // Compute the leaf index
          int idx = (ijk - min_b_).dot (divb_mul_);
          Leaf& leaf = leaves_[idx];
          // First time we initialize the index
          if (leaf.idx == -1)
          {
            leaf.idx = (*indices_)[cp];
            continue;
          }

          // Check to see if this point is closer to the leaf center than the previous one we saved
          float diff_cur   = (input_->points[(*indices_)[cp]].getVector4fMap () - ijk.cast<float> ()).squaredNorm ();
          float diff_prev  = (input_->points[leaf.idx].getVector4fMap ()        - ijk.cast<float> ()).squaredNorm ();

          // If current point is closer, copy its index instead
          if (diff_cur < diff_prev)
            leaf.idx = (*indices_)[cp];
        }

        // Second pass: go over all leaves and copy data
        output.points.resize (leaves_.size ());
        int cp = 0;

        for (typename boost::unordered_map<size_t, Leaf>::const_iterator it = leaves_.begin (); it != leaves_.end (); ++it)
          output.points[cp++] = it->second.idx;
        output.width = static_cast<uint32_t> (output.points.size ());
      }

      bool share_voxel_grid_;
      /** \brief The minimum and maximum bin coordinates, the number of divisions, and the division multiplier. */
      //Eigen::Vector4i min_b_shared_, max_b_shared_, div_b_shared_, divb_mul_shared_;
      Eigen::Vector4f min_p_shared_, max_p_shared_;
      Eigen::Vector4f min_p, max_p;

    public:
      typedef boost::shared_ptr<UniformSamplingSharedVoxelGrid<PointInT> > Ptr;
      typedef boost::shared_ptr<const UniformSamplingSharedVoxelGrid<PointInT> > ConstPtr;

      /** \brief Empty constructor. */
      UniformSamplingSharedVoxelGrid () :
        leaves_ (),
        leaf_size_ (Eigen::Vector4f::Zero ()),
        inverse_leaf_size_ (Eigen::Vector4f::Zero ()),
        min_b_ (Eigen::Vector4i::Zero ()),
        max_b_ (Eigen::Vector4i::Zero ()),
        div_b_ (Eigen::Vector4i::Zero ()),
        divb_mul_ (Eigen::Vector4i::Zero ())
      {
        name_ = "UniformSamplingSharedVoxelGrid";
        share_voxel_grid_ = false;
      }

      /** \brief Destructor. */
      virtual ~UniformSamplingSharedVoxelGrid ()
      {
        leaves_.clear();
      }

      /** \brief Set the 3D grid leaf size.
        * \param radius the 3D grid leaf size
        */
      virtual inline void
      setRadiusSearch (double radius)
      {
        leaf_size_[0] = leaf_size_[1] = leaf_size_[2] = static_cast<float> (radius);
        // Avoid division errors
        if (leaf_size_[3] == 0)
          leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones () / leaf_size_.array ();
        search_radius_ = radius;
      }

      /*void getVoxelGridValues(Eigen::Vector4i & min_b_shared,
                                         Eigen::Vector4i & max_b_shared,
                                         Eigen::Vector4i & div_b_shared,
                                         Eigen::Vector4i & divb_mul_shared)
      {
        min_b_shared = min_b_;
        max_b_shared = max_b_;
        div_b_shared = div_b_;
        divb_mul_shared = divb_mul_;
      }

      void setVoxelGridValues(Eigen::Vector4i & min_b_shared,
                                 Eigen::Vector4i & max_b_shared,
                                 Eigen::Vector4i & div_b_shared,
                                 Eigen::Vector4i & divb_mul_shared)
      {
        min_b_shared_ = min_b_shared;
        max_b_shared_ = max_b_shared;
        div_b_shared_ = div_b_shared;
        divb_mul_shared_ = divb_mul_shared;
        share_voxel_grid_ = true;
      }*/

      void getVoxelGridValues(Eigen::Vector4f & min_p_shared,
                                 Eigen::Vector4f & max_p_shared)
      {
        min_p_shared = min_p;
        max_p_shared = max_p;
      }

      void setVoxelGridValues(Eigen::Vector4f & min_p_shared,
                                 Eigen::Vector4f & max_p_shared)
      {
        min_p_shared_ = min_p_shared;
        max_p_shared_ = max_p_shared;
        share_voxel_grid_ = true;
      }
    };
  }
}


#endif /* FAAT_PCL_UNIFORM_SAMPLING_H_ */
