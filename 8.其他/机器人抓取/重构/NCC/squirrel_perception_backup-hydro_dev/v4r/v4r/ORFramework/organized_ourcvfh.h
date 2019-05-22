#ifndef REC_FRAMEWORK_ORGANIZED_OURCVFH
#define REC_FRAMEWORK_ORGANIZED_OURCVFH

#include <pcl/features/our_cvfh.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh_tools.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>

namespace faat_pcl
{
  namespace rec_3d_framework
  {

template<typename PointInT, typename PointNT, typename PointOutT = pcl::VFHSignature308>
class OURCVFHEstimation : public pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>
{
  public:
    typedef boost::shared_ptr<OURCVFHEstimation<PointInT, PointNT, PointOutT> > Ptr;
    typedef boost::shared_ptr<const OURCVFHEstimation<PointInT, PointNT, PointOutT> > ConstPtr;
    using pcl::Feature<PointInT, PointOutT>::feature_name_;
    using pcl::Feature<PointInT, PointOutT>::getClassName;
    using pcl::Feature<PointInT, PointOutT>::indices_;
    using pcl::Feature<PointInT, PointOutT>::k_;
    using pcl::Feature<PointInT, PointOutT>::search_radius_;
    using pcl::Feature<PointInT, PointOutT>::surface_;
    using pcl::FeatureFromNormals<PointInT, PointNT, PointOutT>::normals_;

    typedef typename pcl::Feature<PointInT, PointOutT>::PointCloudOut PointCloudOut;
    typedef typename pcl::search::Search<pcl::PointNormal>::Ptr KdTreePtr;
    typedef typename pcl::PointCloud<PointInT>::Ptr PointInTPtr;
    /** \brief Empty constructor. */
    OURCVFHEstimation () :
      vpx_ (0), vpy_ (0), vpz_ (0), leaf_size_ (0.005f), normalize_bins_ (false), curv_threshold_ (0.03f), cluster_tolerance_ (leaf_size_ * 3),
          eps_angle_threshold_ (0.125f), min_points_ (50), radius_normals_ (leaf_size_ * 3), centroids_dominant_orientations_ (),
          dominant_normals_ ()
    {
      search_radius_ = 0;
      k_ = 1;
      feature_name_ = "OURCVFHEstimation";
      refine_clusters_ = 1.f;
      min_axis_value_ = 0.925f;
      axis_ratio_ = 0.8f;
    }
    ;

    /** \brief Creates an affine transformation from the RF axes
     * \param[in] evx the x-axis
     * \param[in] evy the z-axis
     * \param[in] evz the z-axis
     * \param[out] transformPC the resulting transformation
     * \param[in] center_mat 4x4 matrix concatenated to the resulting transformation
     */
    inline Eigen::Matrix4f
    createTransFromAxes (Eigen::Vector3f & evx, Eigen::Vector3f & evy, Eigen::Vector3f & evz, Eigen::Affine3f & transformPC,
                         Eigen::Matrix4f & center_mat)
    {
      Eigen::Matrix4f trans;
      trans.setIdentity (4, 4);
      trans (0, 0) = evx (0, 0);
      trans (1, 0) = evx (1, 0);
      trans (2, 0) = evx (2, 0);
      trans (0, 1) = evy (0, 0);
      trans (1, 1) = evy (1, 0);
      trans (2, 1) = evy (2, 0);
      trans (0, 2) = evz (0, 0);
      trans (1, 2) = evz (1, 0);
      trans (2, 2) = evz (2, 0);

      Eigen::Matrix4f homMatrix = Eigen::Matrix4f ();
      homMatrix.setIdentity (4, 4);
      homMatrix = transformPC.matrix ();

      Eigen::Matrix4f trans_copy = trans.inverse ();
      trans = trans_copy * center_mat * homMatrix;
      return trans;
    }

    /** \brief Computes SGURF and the shape distribution based on the selected SGURF
     * \param[in] processed the input cloud
     * \param[out] output the resulting signature
     * \param[in] cluster_indices the indices of the stable cluster
     */
    void
    computeRFAndShapeDistribution (PointInTPtr & processed, PointCloudOut &output, std::vector<pcl::PointIndices> & cluster_indices)
    {
        //this seems to be tricky, i need to update clusters_ to reflect several transformations...
        std::vector<pcl::PointIndices> clusters_extended;
        std::vector<Eigen::Vector3f> centroids_dominant_orientations;
        std::vector<Eigen::Vector3f> dominant_normals;
        PointCloudOut ourcvfh_output;

        //remove nans from the point cloud
        PointInTPtr grid_dense (new pcl::PointCloud<PointInT>);
        grid_dense->points.resize (processed->points.size ());
        int valid = 0;

        std::vector<int> processed_to_grid;
        processed_to_grid.resize(processed->points.size());

        for (size_t k = 0; k < processed->points.size (); k++)
        {
            if(pcl_isnan(processed->points[k].z))
                continue;

            grid_dense->points[valid].getVector4fMap () = processed->points[k].getVector4fMap ();
            processed_to_grid[k] = valid;
            valid++;
        }

        grid_dense->points.resize (valid);
        grid_dense->is_dense = true;

        int num_hists = 8;
        float hist_incr;
        if (normalize_bins_)
        {
          /*int not_nans = 0;
          for (int k = 0; k < static_cast<int> (processed->points.size ()); k++)
          {
            const Eigen::Vector4f & p = processed->points[k].getVector4fMap ();
            if(pcl_isnan(p[2]))
                continue;
            not_nans++;
          }
          hist_incr = 100.f / not_nans * num_hists;*/
          hist_incr = 100.f / static_cast<int>(grid_dense->points.size()) * num_hists;
        }
        else
          hist_incr = 1.0f;

        for (size_t i = 0; i < centroids_dominant_orientations_.size (); i++)
        {

          std::vector < Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
          PointInTPtr grid (new pcl::PointCloud<PointInT>);

          pcl::PointIndices local_indices;
          local_indices.indices.resize(cluster_indices[i].indices.size());
          for (size_t k = 0; k < cluster_indices[i].indices.size (); k++)
              local_indices.indices[k] = processed_to_grid[cluster_indices[i].indices[k]];

          {
            //pcl::ScopeTime t("sgurf");
            //sgurf (centroids_dominant_orientations_[i], dominant_normals_[i], processed, transformations, grid, cluster_indices[i]);
            sgurf (centroids_dominant_orientations_[i], dominant_normals_[i], grid_dense, transformations, grid, local_indices);
          }

          float sigma = 0.01f; //1cm
          float sigma_sq = 2.f * sigma * sigma;
          std::vector < Eigen::VectorXf > quadrants (8);
          int size_hists = 13;

          for (size_t t = 0; t < transformations.size (); t++)
          {
            PointInTPtr grid (new pcl::PointCloud<PointInT>);
            pcl::transformPointCloud (*grid_dense, *grid, transformations[t]);
            transforms_.push_back (transformations[t]);
            valid_transforms_.push_back (true);
            clusters_extended.push_back(cluster_indices[i]);
            centroids_dominant_orientations.push_back(centroids_dominant_orientations_[i]);
            dominant_normals.push_back(dominant_normals_[i]);

            for (int k = 0; k < num_hists; k++)
              quadrants[k].setZero (size_hists);

            Eigen::Vector4f centroid_p = Eigen::Vector4f::Zero();
            Eigen::Vector4f max_pt;
            pcl::getMaxDistance (*grid, centroid_p, max_pt);
            max_pt[3] = 0;
            double distance_normalization_factor = (centroid_p - max_pt).norm ();

            float * weights = new float[num_hists];

            //std::cout << "computeRFAndShapeDistribution:" << grid->points.size() << " - " << distance_normalization_factor << std::endl;
            for (int k = 0; k < static_cast<int> (grid->points.size ()); k++)
            {
              Eigen::Vector4f p = grid->points[k].getVector4fMap ();
              /*if(pcl_isnan(p[0]) || pcl_isnan(p[1]) || pcl_isnan(p[2]))
                  continue;*/
              p[3] = 0.f;
              float d = p.norm ();

              //compute weight for all octants
              float wx = 1.f - std::exp (-((p[0] * p[0]) / (sigma_sq))); //how is the weight distributed among two semi-cubes
              float wy = 1.f - std::exp (-((p[1] * p[1]) / (sigma_sq)));
              float wz = 1.f - std::exp (-((p[2] * p[2]) / (sigma_sq)));

              //distribute the weights using the x-coordinate
              if (p[0] >= 0)
              {
                for (size_t ii = 0; ii <= 3; ii++)
                  weights[ii] = 0.5f - wx * 0.5f;

                for (size_t ii = 4; ii <= 7; ii++)
                  weights[ii] = 0.5f + wx * 0.5f;
              }
              else
              {
                for (size_t ii = 0; ii <= 3; ii++)
                  weights[ii] = 0.5f + wx * 0.5f;

                for (size_t ii = 4; ii <= 7; ii++)
                  weights[ii] = 0.5f - wx * 0.5f;
              }

              //distribute the weights using the y-coordinate
              if (p[1] >= 0)
              {
                for (size_t ii = 0; ii <= 1; ii++)
                  weights[ii] *= 0.5f - wy * 0.5f;
                for (size_t ii = 4; ii <= 5; ii++)
                  weights[ii] *= 0.5f - wy * 0.5f;

                for (size_t ii = 2; ii <= 3; ii++)
                  weights[ii] *= 0.5f + wy * 0.5f;

                for (size_t ii = 6; ii <= 7; ii++)
                  weights[ii] *= 0.5f + wy * 0.5f;
              }
              else
              {
                for (size_t ii = 0; ii <= 1; ii++)
                  weights[ii] *= 0.5f + wy * 0.5f;
                for (size_t ii = 4; ii <= 5; ii++)
                  weights[ii] *= 0.5f + wy * 0.5f;

                for (size_t ii = 2; ii <= 3; ii++)
                  weights[ii] *= 0.5f - wy * 0.5f;

                for (size_t ii = 6; ii <= 7; ii++)
                  weights[ii] *= 0.5f - wy * 0.5f;
              }

              //distribute the weights using the z-coordinate
              if (p[2] >= 0)
              {
                for (size_t ii = 0; ii <= 7; ii += 2)
                  weights[ii] *= 0.5f - wz * 0.5f;

                for (size_t ii = 1; ii <= 7; ii += 2)
                  weights[ii] *= 0.5f + wz * 0.5f;

              }
              else
              {
                for (size_t ii = 0; ii <= 7; ii += 2)
                  weights[ii] *= 0.5f + wz * 0.5f;

                for (size_t ii = 1; ii <= 7; ii += 2)
                  weights[ii] *= 0.5f - wz * 0.5f;
              }

              int h_index = static_cast<int> (std::floor (size_hists * (d / distance_normalization_factor)));
              if(h_index >= size_hists)
                  h_index = (size_hists - 1);

              for (int j = 0; j < num_hists; j++)
                quadrants[j][h_index] += hist_incr * weights[j];

            }

            delete[] weights;

            //copy to the cvfh signature
            PointCloudOut vfh_signature;
            vfh_signature.points.resize (1);
            vfh_signature.width = vfh_signature.height = 1;
            for (int d = 0; d < 308; ++d)
              vfh_signature.points[0].histogram[d] = output.points[i].histogram[d];

            int pos = 45 * 3;
            for (int k = 0; k < num_hists; k++)
            {
              for (int ii = 0; ii < size_hists; ii++, pos++)
              {
                vfh_signature.points[0].histogram[pos] = quadrants[k][ii];
              }
            }

            ourcvfh_output.points.push_back (vfh_signature.points[0]);
          }
        }

        output = ourcvfh_output;
        clusters_ = clusters_extended;
        dominant_normals_ = dominant_normals;
        centroids_dominant_orientations_ = centroids_dominant_orientations;
    }

    /** \brief Computes SGURF
     * \param[in] centroid the centroid of the cluster
     * \param[in] normal_centroid the average of the normals
     * \param[in] processed the input cloud
     * \param[out] transformations the transformations aligning the cloud to the SGURF axes
     * \param[out] grid the cloud transformed internally
     * \param[in] indices the indices of the stable cluster
     */
    bool
    sgurf (Eigen::Vector3f & centroid, Eigen::Vector3f & normal_centroid, PointInTPtr & processed, std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & transformations,
           PointInTPtr & grid, pcl::PointIndices & indices)
    {
        Eigen::Vector3f plane_normal;
        plane_normal[0] = -centroid[0];
        plane_normal[1] = -centroid[1];
        plane_normal[2] = -centroid[2];
        Eigen::Vector3f z_vector = Eigen::Vector3f::UnitZ ();
        plane_normal.normalize ();
        Eigen::Vector3f axis = plane_normal.cross (z_vector);
        double rotation = -asin (axis.norm ());
        axis.normalize ();

        Eigen::Affine3f transformPC (Eigen::AngleAxisf (static_cast<float> (rotation), axis));

        std::vector<int> indices_not_organized;
        /*grid->points.resize (processed->points.size ());
        int valid = 0;
        std::vector<int> processed_to_grid;
        processed_to_grid.resize(processed->points.size());

        for (size_t k = 0; k < processed->points.size (); k++)
        {
            if(pcl_isnan(processed->points[k].z))
                continue;

            grid->points[valid].getVector4fMap () = processed->points[k].getVector4fMap ();
            processed_to_grid[k] = valid;
            valid++;
        }

        grid->points.resize (valid);
        grid->is_dense = true;

        indices_not_organized.resize(indices.indices.size());
        for (size_t k = 0; k < indices.indices.size (); k++)
            indices_not_organized[k] = processed_to_grid[indices.indices[k]];

        pcl::transformPointCloud (*grid, *grid, transformPC);*/
        pcl::transformPointCloud (*processed, *grid, transformPC);
        indices_not_organized = indices.indices;

        Eigen::Vector4f centroid4f (centroid[0], centroid[1], centroid[2], 0);
        Eigen::Vector4f normal_centroid4f (normal_centroid[0], normal_centroid[1], normal_centroid[2], 0);

        centroid4f = transformPC * centroid4f;
        normal_centroid4f = transformPC * normal_centroid4f;

        /*Eigen::Vector4f farthest_away;
        pcl::getMaxDistance (*grid, indices.indices, centroid4f, farthest_away);
        farthest_away[3] = 0;

        float max_dist = (farthest_away - centroid4f).norm ();
        std::cout << "max_dist: " << max_dist << std::endl;*/

        pcl::demeanPointCloud (*grid, centroid4f, *grid);

        Eigen::Matrix4f center_mat;
        center_mat.setIdentity (4, 4);
        center_mat (0, 3) = -centroid4f[0];
        center_mat (1, 3) = -centroid4f[1];
        center_mat (2, 3) = -centroid4f[2];

        Eigen::Matrix3f scatter;
        scatter.setZero ();
        float sum_w = 0.f;
        Eigen::Vector3f evx, evy, evz;
        Eigen::Vector3f evxminus, evyminus, evzminus;

        for (int k = 0; k < static_cast<int> (indices_not_organized.size()); k++)
        {
          Eigen::Vector3f pvector = grid->points[indices_not_organized[k]].getVector3fMap ();
          //float d_k = pvector.norm ();
          //float w = (max_dist - d_k);
          //float w = 1.f;
          //Eigen::Vector3f diff = (pvector);
          Eigen::Matrix3f mat = pvector * pvector.transpose ();
          scatter = scatter + mat; // * w;
          //sum_w += w;
          sum_w += 1.f;
        }

        scatter /= sum_w;

        Eigen::JacobiSVD <Eigen::MatrixXf> svd (scatter, Eigen::ComputeFullV);
        evx = svd.matrixV ().col (0);
        evy = svd.matrixV ().col (1);
        evz = svd.matrixV ().col (2);

        evxminus = evx * -1;
        evyminus = evy * -1;
        evzminus = evz * -1;

        float s_xplus, s_xminus, s_yplus, s_yminus;
        s_xplus = s_xminus = s_yplus = s_yminus = 0.f;

        //disambiguate rf using all points

        Eigen::Vector3f normal3f = Eigen::Vector3f (normal_centroid4f[0], normal_centroid4f[1], normal_centroid4f[2]);

        for (size_t k = 0; k < grid->points.size (); k++)
        {
          Eigen::Vector3f pvector = grid->points[k].getVector3fMap ();
          /*if(pcl_isnan(pvector[0]) || pcl_isnan(pvector[1]) || pcl_isnan(pvector[2]))
              continue;*/

          float dist_x, dist_y;
          dist_x = std::abs (evx.dot (pvector));
          dist_y = std::abs (evy.dot (pvector));

          if ((pvector).dot (evx) >= 0)
            s_xplus += dist_x;
          else
            s_xminus += dist_x;

          if ((pvector).dot (evy) >= 0)
            s_yplus += dist_y;
          else
            s_yminus += dist_y;

        }

        if (s_xplus < s_xminus)
          evx = evxminus;

        if (s_yplus < s_yminus)
          evy = evyminus;

        //select the axis that could be disambiguated more easily
        float fx, fy;
        float max_x = static_cast<float> (std::max (s_xplus, s_xminus));
        float min_x = static_cast<float> (std::min (s_xplus, s_xminus));
        float max_y = static_cast<float> (std::max (s_yplus, s_yminus));
        float min_y = static_cast<float> (std::min (s_yplus, s_yminus));

        fx = (min_x / max_x);
        fy = (min_y / max_y);

        if (normal3f.dot (evz) < 0)
          evz = evzminus;

        //if fx/y close to 1, it was hard to disambiguate
        //what if both are equally easy or difficult to disambiguate, namely fy == fx or very close

        float max_axis = std::max (fx, fy);
        float min_axis = std::min (fx, fy);

        //std::cout << "axis ration:" << (min_axis / max_axis) << "  " << axis_ratio_ << std::endl;
        if ((min_axis / max_axis) > axis_ratio_)
        {
          PCL_WARN("Both axis are equally easy/difficult to disambiguate\n");

          Eigen::Vector3f evy_copy = evy;
          Eigen::Vector3f evxminus = evx * -1;
          Eigen::Vector3f evyminus = evy * -1;

          if (min_axis > min_axis_value_)
          {
            //std::cout << "four cases:" << std::endl;
            //combination of all possibilities
            evy = evz.cross (evx);
            Eigen::Matrix4f trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
            transformations.push_back (trans);

            evx = evxminus;
            evy = evz.cross (evx);
            trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
            transformations.push_back (trans);

            evx = evy_copy;
            evy = evz.cross (evx);
            trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
            transformations.push_back (trans);

            evx = evyminus;
            evy = evz.cross (evx);
            trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
            transformations.push_back (trans);

          }
          else
          {
            //std::cout << "two cases:" << std::endl;
            //1-st case (evx selected)
            evy = evz.cross (evx);
            Eigen::Matrix4f trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
            transformations.push_back (trans);

            //2-nd case (evy selected)
            evx = evy_copy;
            evy = evz.cross (evx);
            trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
            transformations.push_back (trans);
          }
        }
        else
        {
          //std::cout << "completely disambiguable" << std::endl;
          if (fy < fx)
          {
            //std::cout << "switching x and y" << std::endl;
            evx = evy;
            fx = fy;
          }

          evy = evz.cross (evx);
          Eigen::Matrix4f trans = createTransFromAxes (evx, evy, evz, transformPC, center_mat);
          transformations.push_back (trans);

        }

        for(size_t i=0; i < transformations.size(); i++)
        {
            //std::cout << "determinant:" << transformations[i].determinant() << std::endl;
            assert(transformations[i].determinant() > 0);
        }

        return true;
    }

    /** \brief Removes normals with high curvature caused by real edges or noisy data
     * \param[in] cloud pointcloud to be filtered
     * \param[out] indices_out the indices of the points with higher curvature than threshold
     * \param[out] indices_in the indices of the remaining points after filtering
     * \param[in] threshold threshold value for curvature
     */
    void
    filterNormalsWithHighCurvature (const pcl::PointCloud<PointNT> & cloud, std::vector<int> & indices_to_use, std::vector<int> &indices_out,
                                    std::vector<int> &indices_in, float threshold)
    {
        indices_out.resize (cloud.points.size ());
        indices_in.resize (cloud.points.size ());

        size_t in, out;
        in = out = 0;

        for (int i = 0; i < static_cast<int> (indices_to_use.size ()); i++)
        {
          if (cloud.points[indices_to_use[i]].curvature > threshold)
          {
            indices_out[out] = indices_to_use[i];
            out++;
          }
          else
          {
            indices_in[in] = indices_to_use[i];
            in++;
          }
        }

        indices_out.resize (out);
        indices_in.resize (in);
    }

    /** \brief Set the viewpoint.
     * \param[in] vpx the X coordinate of the viewpoint
     * \param[in] vpy the Y coordinate of the viewpoint
     * \param[in] vpz the Z coordinate of the viewpoint
     */
    inline void
    setViewPoint (float vpx, float vpy, float vpz)
    {
      vpx_ = vpx;
      vpy_ = vpy;
      vpz_ = vpz;
    }

    /** \brief Set the radius used to compute normals
     * \param[in] radius_normals the radius
     */
    inline void
    setRadiusNormals (float radius_normals)
    {
      radius_normals_ = radius_normals;
    }

    /** \brief Get the viewpoint.
     * \param[out] vpx the X coordinate of the viewpoint
     * \param[out] vpy the Y coordinate of the viewpoint
     * \param[out] vpz the Z coordinate of the viewpoint
     */
    inline void
    getViewPoint (float &vpx, float &vpy, float &vpz)
    {
      vpx = vpx_;
      vpy = vpy_;
      vpz = vpz_;
    }

    /** \brief Get the centroids used to compute different CVFH descriptors
     * \param[out] centroids vector to hold the centroids
     */
    inline void
    getCentroidClusters (std::vector<Eigen::Vector3f> & centroids)
    {
      for (size_t i = 0; i < centroids_dominant_orientations_.size (); ++i)
        centroids.push_back (centroids_dominant_orientations_[i]);
    }

    /** \brief Get the normal centroids used to compute different CVFH descriptors
     * \param[out] centroids vector to hold the normal centroids
     */
    inline void
    getCentroidNormalClusters (std::vector<Eigen::Vector3f> & centroids)
    {
      for (size_t i = 0; i < dominant_normals_.size (); ++i)
        centroids.push_back (dominant_normals_[i]);
    }

    /** \brief Sets max. Euclidean distance between points to be added to the cluster
     * \param[in] d the maximum Euclidean distance
     */

    inline void
    setClusterTolerance (float d)
    {
      cluster_tolerance_ = d;
    }

    /** \brief Sets max. deviation of the normals between two points so they can be clustered together
     * \param[in] d the maximum deviation
     */
    inline void
    setEPSAngleThreshold (float d)
    {
      eps_angle_threshold_ = d;
    }

    /** \brief Sets curvature threshold for removing normals
     * \param[in] d the curvature threshold
     */
    inline void
    setCurvatureThreshold (float d)
    {
      curv_threshold_ = d;
    }

    /** \brief Set minimum amount of points for a cluster to be considered
     * \param[in] min the minimum amount of points to be set
     */
    inline void
    setMinPoints (size_t min)
    {
      min_points_ = min;
    }

    /** \brief Sets wether if the signatures should be normalized or not
     * \param[in] normalize true if normalization is required, false otherwise
     */
    inline void
    setNormalizeBins (bool normalize)
    {
      normalize_bins_ = normalize;
    }

    /** \brief Gets the indices of the original point cloud used to compute the signatures
     * \param[out] indices vector of point indices
     */
    inline void
    getClusterIndices (std::vector<pcl::PointIndices> & indices)
    {
      indices = clusters_;
    }

    /** \brief Sets the refinement factor for the clusters
     * \param[in] rc the factor used to decide if a point is used to estimate a stable cluster
     */
    void
    setRefineClusters (float rc)
    {
      refine_clusters_ = rc;
    }

    /** \brief Returns the transformations aligning the point cloud to the corresponding SGURF
     * \param[out] trans vector of transformations
     */
    void
    getTransforms (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & trans)
    {
      trans = transforms_;
    }

    /** \brief Returns a boolean vector indicating of the transformation obtained by getTransforms() represents
     * a valid SGURF
     * \param[out] valid vector of booleans
     */
    void
    getValidTransformsVec (std::vector<bool> & valid)
    {
      valid = valid_transforms_;
    }

    /** \brief Sets the min axis ratio between the SGURF axes to decide if disambiguition is feasible
     * \param[in] f the ratio between axes
     */
    void
    setAxisRatio (float f)
    {
      axis_ratio_ = f;
    }

    /** \brief Sets the min disambiguition axis value to generate several SGURFs for the cluster when disambiguition is difficult
     * \param[in] f the min axis value
     */
    void
    setMinAxisValue (float f)
    {
      min_axis_value_ = f;
    }

    /** \brief Overloaded computed method from pcl::Feature.
     * \param[out] output the resultant point cloud model dataset containing the estimated features
     */
    void
    compute (PointCloudOut &output)
    {
        if (!pcl::Feature<PointInT, PointOutT>::initCompute ())
        {
          output.width = output.height = 0;
          output.points.clear ();
          return;
        }
        // Resize the output dataset
        // Important! We should only allocate precisely how many elements we will need, otherwise
        // we risk at pre-allocating too much memory which could lead to bad_alloc
        // (see http://dev.pointclouds.org/issues/657)
        output.width = output.height = 1;
        output.points.resize (1);

        // Perform the actual feature computation
        computeFeature (output);

        pcl::Feature<PointInT, PointOutT>::deinitCompute ();
    }

  private:
    /** \brief Values describing the viewpoint ("pinhole" camera model assumed).
     * By default, the viewpoint is set to 0,0,0.
     */
    float vpx_, vpy_, vpz_;

    /** \brief Size of the voxels after voxel gridding. IMPORTANT: Must match the voxel
     * size of the training data or the normalize_bins_ flag must be set to true.
     */
    float leaf_size_;

    /** \brief Wether to normalize the signatures or not. Default: false. */
    bool normalize_bins_;

    /** \brief Curvature threshold for removing normals. */
    float curv_threshold_;

    /** \brief allowed Euclidean distance between points to be added to the cluster. */
    float cluster_tolerance_;

    /** \brief deviation of the normals between two points so they can be clustered together. */
    float eps_angle_threshold_;

    /** \brief Minimum amount of points in a clustered region to be considered stable for CVFH
     * computation.
     */
    size_t min_points_;

    /** \brief Radius for the normals computation. */
    float radius_normals_;

    /** \brief Factor for the cluster refinement */
    float refine_clusters_;

    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transforms_;
    std::vector<bool> valid_transforms_;

    float axis_ratio_;
    float min_axis_value_;

    /** \brief Estimate the OUR-CVFH descriptors at
     * a set of points given by <setInputCloud (), setIndices ()> using the surface in
     * setSearchSurface ()
     *
     * \param[out] output the resultant point cloud model dataset that contains the OUR-CVFH
     * feature estimates
     */
    void
    computeFeature (PointCloudOut &output)
    {
        if (refine_clusters_ <= 0.f)
          refine_clusters_ = 1.f;

        // Check if input was set
        if (!normals_)
        {
          PCL_ERROR ("[pcl::%s::computeFeature] No input dataset containing normals was given!\n", getClassName ().c_str ());
          output.width = output.height = 0;
          output.points.clear ();
          return;
        }
        if (normals_->points.size () != surface_->points.size ())
        {
          PCL_ERROR ("[pcl::%s::computeFeature] The number of points in the input dataset differs from the number of points in the dataset containing the normals!\n", getClassName ().c_str ());
          output.width = output.height = 0;
          output.points.clear ();
          return;
        }

        centroids_dominant_orientations_.clear ();
        clusters_.clear ();
        transforms_.clear ();
        dominant_normals_.clear ();
        valid_transforms_.clear ();

        std::vector<pcl::PointIndices> clusters;
        std::vector<bool> good_cluster;

        {
            pcl::ScopeTime t("clustering");
            typename pcl::EuclideanClusterComparator<PointInT, pcl::Normal, pcl::Label>::Ptr
                    euclidean_cluster_comparator (new pcl::EuclideanClusterComparator<PointInT, pcl::Normal, pcl::Label> ());

            //create two labels, 1 one for points to be smoothly clustered, another one for the rest
            pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
            labels->points.resize(surface_->points.size());
            labels->width = surface_->width;
            labels->height = surface_->height;
            labels->is_dense = surface_->is_dense;

            for (size_t j = 0; j < surface_->points.size (); j++)
            {
              Eigen::Vector3f xyz_p = surface_->points[j].getVector3fMap ();
              if (!pcl_isfinite (xyz_p[0]) || !pcl_isfinite (xyz_p[1]) || !pcl_isfinite (xyz_p[2]))
              {
                labels->points[j].label = 0;
                continue;
              }

              //check normal
              Eigen::Vector3f normal = normals_->points[j].getNormalVector3fMap ();
              if (!pcl_isfinite (normal[0]) || !pcl_isfinite (normal[1]) || !pcl_isfinite (normal[2]))
              {
                labels->points[j].label = 0;
                continue;
              }

              //check curvature
              float curvature = normals_->points[j].curvature;
              if(curvature > (curv_threshold_)) // * (std::min(1.f,scene_cloud_->points[j].z))))
              {
                  labels->points[j].label = 0;
                  continue;
              }

              labels->points[j].label = 1;
            }

            std::vector<bool> excluded_labels;
            excluded_labels.resize (2, false);
            excluded_labels[0] = true;

            euclidean_cluster_comparator->setInputCloud (surface_);
            euclidean_cluster_comparator->setLabels (labels);
            euclidean_cluster_comparator->setExcludeLabels (excluded_labels);
            euclidean_cluster_comparator->setDistanceThreshold (cluster_tolerance_, true);
            euclidean_cluster_comparator->setAngularThreshold(0.017453 * 5.f); //5 degrees

            pcl::PointCloud<pcl::Label> euclidean_labels;
            pcl::OrganizedConnectedComponentSegmentation<PointInT, pcl::Label> euclidean_segmentation (euclidean_cluster_comparator);
            euclidean_segmentation.setInputCloud (surface_);
            euclidean_segmentation.segment (euclidean_labels, clusters);

            good_cluster.resize(clusters.size(), false);
            for (size_t i = 0; i < clusters.size (); i++)
            {
              if (clusters[i].indices.size () >= min_points_)
                  good_cluster[i] = true;
            }
        }

        {
          //pcl::ScopeTime t("filtering clusters");
          std::vector<pcl::PointIndices> clusters_filtered;
          int cluster_filtered_idx = 0;
          for (size_t i = 0; i < clusters.size (); i++)
          {

              if(!good_cluster[i])
                  continue;

            pcl::PointIndices pi;
            pcl::PointIndices pi_filtered;

            clusters_.push_back (pi);
            clusters_filtered.push_back (pi_filtered);

            Eigen::Vector4f avg_normal = Eigen::Vector4f::Zero ();
            Eigen::Vector4f avg_centroid = Eigen::Vector4f::Zero ();

            for (size_t j = 0; j < clusters[i].indices.size (); j++)
            {
              avg_normal += normals_->points[clusters[i].indices[j]].getNormalVector4fMap ();
              avg_centroid += surface_->points[clusters[i].indices[j]].getVector4fMap ();
            }

            avg_normal /= static_cast<float> (clusters[i].indices.size ());
            avg_centroid /= static_cast<float> (clusters[i].indices.size ());
            avg_normal.normalize ();

            Eigen::Vector3f avg_norm (avg_normal[0], avg_normal[1], avg_normal[2]);
            Eigen::Vector3f avg_dominant_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);

            for (size_t j = 0; j < clusters[i].indices.size (); j++)
            {
              //decide if normal should be added
              double dot_p = avg_normal.dot (normals_->points[clusters[i].indices[j]].getNormalVector4fMap ());
              if (fabs (acos (dot_p)) < (eps_angle_threshold_ * refine_clusters_))
              {
                clusters_[cluster_filtered_idx].indices.push_back (clusters[i].indices[j]);
                clusters_filtered[cluster_filtered_idx].indices.push_back (clusters[i].indices[j]);
              }
            }

            //remove last cluster if no points found...
            if (clusters_[cluster_filtered_idx].indices.size () == 0)
            {
              clusters_.erase (clusters_.end ());
              clusters_filtered.erase (clusters_filtered.end ());
            }
            else
              cluster_filtered_idx++;
          }

          clusters = clusters_filtered;
          //std::cout << "Number of clusters:" << clusters.size() << std::endl;

        }

        boost::shared_ptr< std::vector<int> > indices_for_vfh(new std::vector<int>);
        indices_for_vfh->resize(indices_->size());
        int not_nan = 0;
        for(size_t i=0; i < indices_->size(); i++)
        {
            if(!pcl_isnan(surface_->points[(*indices_)[i]].z))
            {
                (*indices_for_vfh)[not_nan] = (*indices_)[i];
                not_nan++;
            }
        }

        indices_for_vfh->resize(not_nan);

        pcl::VFHEstimation<PointInT, PointNT, pcl::VFHSignature308> vfh;
        vfh.setInputCloud (surface_);
        vfh.setInputNormals (normals_);
        vfh.setIndices (indices_for_vfh);
        vfh.setSearchMethod (this->tree_);
        vfh.setUseGivenNormal (true);
        vfh.setUseGivenCentroid (true);
        vfh.setNormalizeBins (normalize_bins_);
        vfh.setFillSizeComponent(false);
        output.height = 1;

        // ---[ Step 1b : check if any dominant cluster was found
        if (clusters.size () > 0)
        { // ---[ Step 1b.1 : If yes, compute CVFH using the cluster information

          for (size_t i = 0; i < clusters.size (); ++i) //for each cluster
          {

            Eigen::Vector4f avg_normal = Eigen::Vector4f::Zero ();
            Eigen::Vector4f avg_centroid = Eigen::Vector4f::Zero ();

            for (size_t j = 0; j < clusters[i].indices.size (); j++)
            {
              avg_normal += normals_->points[clusters[i].indices[j]].getNormalVector4fMap ();
              avg_centroid += surface_->points[clusters[i].indices[j]].getVector4fMap ();
            }

            avg_normal /= static_cast<float> (clusters[i].indices.size ());
            avg_centroid /= static_cast<float> (clusters[i].indices.size ());
            avg_normal.normalize ();

            Eigen::Vector3f avg_norm (avg_normal[0], avg_normal[1], avg_normal[2]);
            Eigen::Vector3f avg_dominant_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);

            //append normal and centroid for the clusters
            dominant_normals_.push_back (avg_norm);
            centroids_dominant_orientations_.push_back (avg_dominant_centroid);
          }

          //compute modified VFH for all dominant clusters and add them to the list!
          output.points.resize (dominant_normals_.size ());
          output.width = static_cast<uint32_t> (dominant_normals_.size ());

          for (size_t i = 0; i < dominant_normals_.size (); ++i)
          {
            //pcl::ScopeTime kk("processing one cluster... VFH");
            //std::cout << dominant_normals_[i] << std::endl;
            //configure VFH computation for CVFH
            vfh.setNormalToUse (dominant_normals_[i]);
            vfh.setCentroidToUse (centroids_dominant_orientations_[i]);
            pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
            vfh.compute (vfh_signature);
            output.points[i] = vfh_signature.points[0];
          }

          //finish filling the descriptor with the shape distribution
          {
            //pcl::ScopeTime t("computeRFAndShapeDistribution");
            PointInTPtr cloud_input (new pcl::PointCloud<PointInT>);
            pcl::copyPointCloud (*surface_, *indices_, *cloud_input);
            computeRFAndShapeDistribution (cloud_input, output, clusters_); //this will set transforms_
          }
        }
        else
        { // ---[ Step 1b.1 : If no, compute a VFH using all the object points

          PCL_WARN("No clusters were found in the surface... using VFH...\n");
          Eigen::Vector4f avg_centroid;
          pcl::compute3DCentroid (*surface_, avg_centroid);
          Eigen::Vector3f cloud_centroid (avg_centroid[0], avg_centroid[1], avg_centroid[2]);
          centroids_dominant_orientations_.push_back (cloud_centroid);

          //configure VFH computation using all object points
          vfh.setCentroidToUse (cloud_centroid);
          vfh.setUseGivenNormal (false);

          pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
          vfh.compute (vfh_signature);

          output.points.resize (1);
          output.width = 1;

          output.points[0] = vfh_signature.points[0];
          Eigen::Matrix4f id = Eigen::Matrix4f::Identity ();
          transforms_.push_back (id);
          valid_transforms_.push_back (false);
        }
    }

  protected:
    /** \brief Centroids that were used to compute different OUR-CVFH descriptors */
    std::vector<Eigen::Vector3f> centroids_dominant_orientations_;
    /** \brief Normal centroids that were used to compute different OUR-CVFH descriptors */
    std::vector<Eigen::Vector3f> dominant_normals_;
    /** \brief Indices to the points representing the stable clusters */
    std::vector<pcl::PointIndices> clusters_;
};
}
}

#endif
