#include "model_only_source.h"

#include <boost/graph/connected_components.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/undirected_graph.hpp>

template<typename Full3DPointT, typename PointInT>
void
computeFacesImpl (typename faat_pcl::rec_3d_framework::Model<PointInT> & model)
{
    float voxel_resolution = 0.005f;
    float seed_resolution = 0.05f;
    typename pcl::SupervoxelClustering<PointInT> super (voxel_resolution, seed_resolution, false);
    super.setInputCloud (model.assembled_);
    super.setColorImportance (0.f);
    super.setSpatialImportance (1.f);
    super.setNormalImportance (2.f);
    super.setNormalCloud(model.normals_assembled_);
    std::map <uint32_t, typename pcl::Supervoxel<PointInT>::Ptr > supervoxel_clusters;
    pcl::console::print_highlight ("Extracting supervoxels!\n");
    super.extract (supervoxel_clusters);
    //super.refineSupervoxels(5, supervoxel_clusters);
    pcl::console::print_info ("Found %d supervoxels\n", supervoxel_clusters.size ());

    pcl::PointCloud<pcl::PointXYZL>::Ptr supervoxels_labels_cloud = super.getLabeledCloud();
    uint32_t max_label = super.getMaxLabel();

    pcl::PointCloud<pcl::PointNormal>::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);

    std::vector<int> label_to_idx;
    label_to_idx.resize(max_label + 1, -1);
    typename std::map <uint32_t, typename pcl::Supervoxel<PointInT>::Ptr>::iterator sv_itr,sv_itr_end;
    sv_itr = supervoxel_clusters.begin ();
    sv_itr_end = supervoxel_clusters.end ();
    int i=0;
    for ( ; sv_itr != sv_itr_end; ++sv_itr, i++)
    {
      label_to_idx[sv_itr->first] = i;
    }

    std::vector< std::vector<bool> > adjacent;
    adjacent.resize(supervoxel_clusters.size());
    for(size_t i=0; i < (supervoxel_clusters.size()); i++)
        adjacent[i].resize(supervoxel_clusters.size(), false);

    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    super.getSupervoxelAdjacency (supervoxel_adjacency);
    //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
    std::multimap<uint32_t,uint32_t>::iterator label_itr = supervoxel_adjacency.begin ();
    std::cout << "super voxel adjacency size:" << supervoxel_adjacency.size() << std::endl;
    for ( ; label_itr != supervoxel_adjacency.end (); )
    {
      //First get the label
      uint32_t supervoxel_label = label_itr->first;
      Eigen::Vector3f normal_super_voxel = sv_normal_cloud->points[label_to_idx[supervoxel_label]].getNormalVector3fMap();
      normal_super_voxel.normalize();
      //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
      std::multimap<uint32_t,uint32_t>::iterator adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first;
      for ( ; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
      {
        Eigen::Vector3f normal_neighbor_supervoxel = sv_normal_cloud->points[label_to_idx[adjacent_itr->second]].getNormalVector3fMap();
        normal_neighbor_supervoxel.normalize();

        if(normal_super_voxel.dot(normal_neighbor_supervoxel) > 0.9f)
        {
            adjacent[label_to_idx[supervoxel_label]][label_to_idx[adjacent_itr->second]] = true;
        }
      }

      //Move iterator forward to next label
      label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
    }

    typedef boost::adjacency_matrix<boost::undirectedS, int> Graph;
    Graph G(supervoxel_clusters.size());
    for(size_t i=0; i < supervoxel_clusters.size(); i++)
    {
        for(size_t j=(i+1); j < supervoxel_clusters.size(); j++)
        {
            if(adjacent[i][j])
                boost::add_edge(i, j, G);
        }
    }

    std::vector<int> components (boost::num_vertices (G));
    int n_cc = static_cast<int> (boost::connected_components (G, &components[0]));
    std::cout << "Number of connected components..." << n_cc << std::endl;

    std::vector<int> cc_sizes;
    std::vector<std::vector<int> > ccs;
    std::vector<uint32_t> original_labels_to_merged;
    original_labels_to_merged.resize(supervoxel_clusters.size());

    ccs.resize(n_cc);
    cc_sizes.resize (n_cc, 0);
    typename boost::graph_traits<Graph>::vertex_iterator vertexIt, vertexEnd;
    boost::tie (vertexIt, vertexEnd) = vertices (G);
    for (; vertexIt != vertexEnd; ++vertexIt)
    {
      int c = components[*vertexIt];
      cc_sizes[c]++;
      ccs[c].push_back(*vertexIt);
      original_labels_to_merged[*vertexIt] = c;
    }

    for(size_t i=0; i < supervoxels_labels_cloud->points.size(); i++)
    {
        //std::cout << supervoxels_labels_cloud->points[i].label << " " << label_to_idx.size() << " " << original_labels_to_merged.size() << " " << label_to_idx[supervoxels_labels_cloud->points[i].label] << std::endl;
        if(label_to_idx[supervoxels_labels_cloud->points[i].label] < 0)
            continue;

        supervoxels_labels_cloud->points[i].label = original_labels_to_merged[label_to_idx[supervoxels_labels_cloud->points[i].label]];
    }

    model.faces_cloud_labels_ = supervoxels_labels_cloud;

    //    for(size_t i=0; i < cc_sizes.size(); i++)
    //    {
    //        std::cout << "cc size:" << cc_sizes[i] << std::endl;
    //    }

    //    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr supervoxels_rgb(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //    supervoxels_rgb = super.getColoredCloud();

    //    pcl::visualization::PCLVisualizer vis("model smooth surfaces");
    //    int v1,v2, v3;
    //    vis.createViewPort(0,0,0.33,1.0,v1);
    //    vis.createViewPort(0.33, 0, 0.66, 1, v2);
    //    vis.createViewPort(0.66, 0, 1, 1, v3);
    //    vis.addPointCloud<PointInT>(model.assembled_, "model", v1);
    //    vis.addPointCloudNormals<pcl::PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals", v2);
    //    vis.addPointCloud(supervoxels_rgb, "labels", v2);

    //    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZL> handler_labels(supervoxels_labels_cloud, "label");
    //    vis.addPointCloud(supervoxels_labels_cloud, handler_labels, "labels_", v3);
    //    vis.spin();
}

template< >
void
computeFacesImpl <pcl::PointXYZ, pcl::PointXYZ>
    (faat_pcl::rec_3d_framework::Model<pcl::PointXYZ> & model)
{
    PCL_WARN("Not implemented for pcl::PointXYZ... this function would be available for PCL1.7.2 or higher\n");
}

template<typename Full3DPointT, typename PointInT>
void
faat_pcl::rec_3d_framework::ModelOnlySource<Full3DPointT, PointInT>::computeFaces (ModelT & model)
{
    //boost::shared_ptr< typename faat_pcl::rec_3d_framework::Model<PointInT> > m = model;
    computeFacesImpl<Full3DPointT, PointInT>(model);
}

template class faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZRGBNormal, pcl::PointXYZRGB>;
template class faat_pcl::rec_3d_framework::ModelOnlySource<pcl::PointXYZ, pcl::PointXYZ>;
