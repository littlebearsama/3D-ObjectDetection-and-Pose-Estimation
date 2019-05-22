/*
 * partial_pcd_source.h
 *
 *  Created on: Jul 3, 2012
 *      Author: aitor
 */

#ifndef FAAT_PCL_PARTIAL_PCD_SOURCE_HPP_
#define FAAT_PCL_PARTIAL_PCD_SOURCE_HPP_

#include "partial_pcd_source.h"
#include <pcl/common/angles.h>

template<typename Full3DPointT, typename PointInT, typename OutModelPointT>
void
faat_pcl::rec_3d_framework::PartialPCDSource<Full3DPointT, PointInT, OutModelPointT>::loadOrGenerate (std::string & dir, std::string & model_path, ModelT & model)
{
  std::stringstream pathmodel;
  pathmodel << dir << "/" << model.class_ << "/" << model.id_;
  bf::path trained_dir = pathmodel.str ();

  model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr>);
  model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
  model.self_occlusions_.reset (new std::vector<float>);
  if(gen_organized_)
  {
    model.indices_.reset(new std::vector<pcl::PointIndices>);
  }

  model.assembled_.reset (new pcl::PointCloud<OutModelPointT>);
  model.normals_assembled_.reset (new pcl::PointCloud<pcl::Normal>);

  typename pcl::PointCloud<Full3DPointT>::Ptr model_cloud (new pcl::PointCloud<Full3DPointT>);
  pcl::io::loadPCDFile (model_path, *model_cloud);

  pcl::copyPointCloud (*model_cloud, *model.assembled_);
  pcl::copyPointCloud (*model_cloud, *model.normals_assembled_);

  if (bf::exists (trained_dir))
  {
    //load views, poses and self-occlusions

    getViewsFilenames(trained_dir, model.view_filenames_, "view");
    std::sort(model.view_filenames_.begin(), model.view_filenames_.end());
    if(load_into_memory_)
    {
      loadInMemorySpecificModel(dir, model);
    }
  }
  else
  {
    std::cout << "We need to generate views..." << std::endl;

    std::stringstream direc;
    direc << dir << "/" << model.class_ << "/" << model.id_;
    this->createClassAndModelDirectories (dir, model.class_, model.id_);

    //create camera positions taking into account constraints
    vtkSmartPointer < vtkPlatonicSolidSource > ico = vtkSmartPointer<vtkPlatonicSolidSource>::New ();
    ico->SetSolidTypeToIcosahedron ();
    ico->Update ();

    //tesselate cells from icosahedron
    vtkSmartPointer < vtkLoopSubdivisionFilter > subdivide = vtkSmartPointer<vtkLoopSubdivisionFilter>::New ();
    subdivide->SetNumberOfSubdivisions (tes_level_);
    subdivide->SetInputConnection (ico->GetOutputPort ());

    // Get camera positions
    vtkPolyData *sphere = subdivide->GetOutput ();
    sphere->Update ();

    std::vector<Eigen::Vector3f> cam_positions;
    if (!use_vertices_)
    {
      vtkSmartPointer < vtkCellArray > cells_sphere = sphere->GetPolys ();
      cam_positions.resize (sphere->GetNumberOfPolys ());

      double center[3], p1_com[3], p2_com[3], p3_com[3];
      vtkIdType npts_com = 0, *ptIds_com = NULL;
      size_t i = 0;
      for (cells_sphere->InitTraversal (); cells_sphere->GetNextCell (npts_com, ptIds_com);)
      {
        sphere->GetPoint (ptIds_com[0], p1_com);
        sphere->GetPoint (ptIds_com[1], p2_com);
        sphere->GetPoint (ptIds_com[2], p3_com);
        vtkTriangle::TriangleCenter (p1_com, p2_com, p3_com, center);
        cam_positions[i] = Eigen::Vector3f (float (center[0]), float (center[1]), float (center[2]));
        i++;
      }

    }
    else
    {
      cam_positions.resize (sphere->GetNumberOfPoints ());
      for (int i = 0; i < sphere->GetNumberOfPoints (); i++)
      {
        double cam_pos[3];
        sphere->GetPoint (i, cam_pos);
        cam_positions[i] = Eigen::Vector3f (float (cam_pos[0]), float (cam_pos[1]), float (cam_pos[2]));
      }
    }

    //filter camera positions according to the constraints

    if(campos_constraints_func_) {
      int valid = 0;
      for (size_t i = 0; i < cam_positions.size (); i++)
      {
        if (campos_constraints_func_ (cam_positions[i]))
        {
          cam_positions[valid] = cam_positions[i];
          valid++;
        }
      }

      cam_positions.resize (valid);
    }

    for (size_t i = 0; i < cam_positions.size (); i++)
    {
      cam_positions[i] = cam_positions[i] * radius_sphere_;
    }
    std::cout << "camera positions:" << cam_positions.size () << std::endl;

    std::vector<float> inplane_rotations_for_cams;
    if(gen_inplane_rotations_)
    {
      std::vector<Eigen::Vector3f> cam_positions_with_inplane;
      for (size_t i = 0; i < cam_positions.size (); i++)
      {
        float angle = 0.f;
        while(angle < 360.f)
        {
          cam_positions_with_inplane.push_back(cam_positions[i]);
          inplane_rotations_for_cams.push_back(angle);
          angle += angle_incr_inplane_;
        }
      }

      cam_positions = cam_positions_with_inplane;
      assert(cam_positions.size() == inplane_rotations_for_cams.size());
      std::cout << cam_positions.size() << std::endl;
    }

    Eigen::Vector4f obj_centroid;
    pcl::compute3DCentroid (*model.assembled_, obj_centroid);
    Eigen::Vector3f obj_centroid3f = obj_centroid.head<3> ();

    std::vector<typename pcl::PointCloud<PointInT>::Ptr> views_orig;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    std::vector<pcl::PointIndices> indices;

    /*model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr> ());
    model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > ());
    model.self_occlusions_.reset (new std::vector<float> ());
    if(gen_organized_)
    {
      model.indices_.reset(new std::vector<pcl::PointIndices>);
    }*/

    //pcl::visualization::PCLVisualizer vis_test("vis");
    //vis_test.addCoordinateSystem(0.1f);

    for (size_t i = 0; i < cam_positions.size (); i++)
    {
      //obtain partial views and views poses
      typename pcl::PointCloud<Full3DPointT>::Ptr model_cloud_trans (new pcl::PointCloud<Full3DPointT> ());
      //typename pcl::PointCloud<PointInT>::Ptr model_cloud_trans (new pcl::PointCloud<PointInT> ());
      Eigen::Matrix4f flip_y;
      flip_y.setIdentity (4,4);
      flip_y (1, 1) = -1.f;

      pcl::transformPointCloudWithNormals (*model_cloud, *model_cloud_trans, flip_y);

      /*{
        pcl::visualization::PointCloudColorHandlerRGBField<Full3DPointT> handler(model_cloud_trans);
        vis_test.addPointCloud<Full3DPointT>(model_cloud_trans, handler, "transformed");
      }*/

      pcl::demeanPointCloud (*model_cloud_trans, obj_centroid, *model_cloud_trans); //center object

      /*{
        pcl::visualization::PointCloudColorHandlerRGBField<Full3DPointT> handler(model_cloud_trans);
        vis_test.addPointCloud<Full3DPointT>(model_cloud_trans, handler, "transformed2");
      }*/

      Eigen::Vector3f zp = cam_positions[i] * -1.f;
      zp.normalize ();
      Eigen::Vector3f yp = (Eigen::Vector3f::UnitY ()).cross (zp);
      yp.normalize ();
      Eigen::Vector3f xp = zp.cross (yp);
      xp.normalize ();

      Eigen::Matrix4f rot_matrix;
      rot_matrix.setIdentity ();
      rot_matrix.block<3, 1> (0, 0) = xp;
      rot_matrix.block<3, 1> (0, 1) = yp;
      rot_matrix.block<3, 1> (0, 2) = zp;

      Eigen::Matrix4f inv;
      inv = rot_matrix.inverse ();

      pcl::transformPointCloudWithNormals (*model_cloud_trans, *model_cloud_trans, inv); //transform to CC
      /*{
        pcl::visualization::PointCloudColorHandlerRGBField<Full3DPointT> handler(model_cloud_trans);
        vis_test.addPointCloud<Full3DPointT>(model_cloud_trans, handler, "transformed3");
      }*/

      Eigen::Vector4f bb;
      bb.setZero ();
      bb.head<3> () = cam_positions[i];
      bb = inv * bb;

      /*std::cout << inv << std::endl;
      std::cout << bb << std::endl;
      std::cout << cam_positions[i] << std::endl;*/

      pcl::demeanPointCloud (*model_cloud_trans, bb, *model_cloud_trans); //move the object so that camera is at 0,0,0

      /*{
        pcl::visualization::PointCloudColorHandlerRGBField<Full3DPointT> handler(model_cloud_trans);
        vis_test.addPointCloud<Full3DPointT>(model_cloud_trans, handler, "transformed4");
      }

      vis_test.spin();
      vis_test.removeAllPointClouds();*/

      //here i could apply the roll rotation, unfortunately too much memory needed...
      if(gen_inplane_rotations_)
      {
        Eigen::Matrix3f m;
        m = Eigen::AngleAxisf(pcl::deg2rad(inplane_rotations_for_cams[i]), Eigen::Vector3f::UnitZ());
        Eigen::Matrix4f roll_trans;
        roll_trans.setIdentity();
        roll_trans.block<3,3>(0,0) = m;
        pcl::transformPointCloud(*model_cloud_trans, *model_cloud_trans, roll_trans);

      }

      //reason about occlusions, TODO: Check this crap!
      int cx_, cy_;
      cx_ = cx_size;
      cy_ = cy_size;
      int width_ = cx_;
      int height_ = cy_;

      std::cout << "(cx,cy)" << cx_ << "," << cy_ << std::endl;
      std::cout << "(width_,height_)" << width_ << "," << height_ << std::endl;

      typename pcl::PointCloud<Full3DPointT>::Ptr filtered (new pcl::PointCloud<Full3DPointT> ());
      typename pcl::PointCloud<Full3DPointT>::ConstPtr model_cloud_trans_const (new pcl::PointCloud<Full3DPointT> (*model_cloud_trans));

      //typename pcl::occlusion_reasoning::ZBuffering<Full3DPointT, Full3DPointT> zbuffer_self_occlusion (cx_, cy_, 525.f);
      //zbuffer_self_occlusion.computeDepthMap (model_cloud_trans_const, true);

      {
          float * depth_ = new float[cx_ * cy_];
          uint8_t * r = new uint8_t[cx_ * cy_];
          uint8_t * g = new uint8_t[cx_ * cy_];
          uint8_t * b = new uint8_t[cx_ * cy_];
          Eigen::Vector3f * normals = new Eigen::Vector3f[cx_ * cy_];

          typename pcl::PointCloud<Full3DPointT>::Ptr depth_map (new pcl::PointCloud<Full3DPointT>);
          depth_map->width = cx_;
          depth_map->height = cy_;
          depth_map->points.resize (cx_ * cy_);
          depth_map->is_dense = false;

          {
            float cx2, cy2;
            cx2 = static_cast<float> (cx_) / 2.f - 0.5f;
            cy2 = static_cast<float> (cy_) / 2.f - 0.5f;

            /*float max_u, max_v, min_u, min_v;
            max_u = max_v = std::numeric_limits<float>::max () * -1;
            min_u = min_v = std::numeric_limits<float>::max ();

            for (size_t i = 0; i < model_cloud_trans_const->points.size (); i++)
            {
              float b_x = model_cloud_trans_const->points[i].x / model_cloud_trans_const->points[i].z;
              if (b_x > max_u)
                max_u = b_x;
              if (b_x < min_u)
                min_u = b_x;

              float b_y = model_cloud_trans_const->points[i].y / model_cloud_trans_const->points[i].z;
              if (b_y > max_v)
                max_v = b_y;
              if (b_y < min_v)
                min_v = b_y;
            }

            float maxC = std::max (std::max (std::abs (max_u), std::abs (max_v)), std::max (std::abs (min_u), std::abs (min_v)));*/
            //float f_ = (cx) / maxC;
            //f_ /= 1.5;

            for (int i = 0; i < (cx_ * cy_); i++)
              depth_[i] = std::numeric_limits<float>::quiet_NaN ();

            for (size_t i = 0; i < model_cloud_trans_const->points.size (); i++)
            {
              float x = model_cloud_trans_const->points[i].x;
              float y = model_cloud_trans_const->points[i].y;
              float z = model_cloud_trans_const->points[i].z;
              int u = static_cast<int> (f_ * x / z + cx2);
              int v = static_cast<int> (f_ * y / z + cy2);

              if (u >= width_ || v >= height_ || u < 0 || v < 0)
                continue;

              int idx = v * width_ + u;
              if(idx >= (cx_ * cy_))
              {
                  std::cout << "(u,v)" << u << "," << v << std::endl;
                  std::cout << "(cx,cy)" << cx_ << "," << cy_ << std::endl;
                  std::cout << "(width_,height_)" << width_ << "," << height_ << std::endl;
                  std::cout << idx << " " << cx_ * cy_ << std::endl;
              }

              assert(idx < cx_ * cy_);
              if ((z < depth_[idx]) || (!pcl_isfinite(depth_[idx])))
              {
                depth_[idx] = z;
                normals[idx] = model_cloud_trans_const->points[i].getNormalVector3fMap();
                r[idx] = model_cloud_trans_const->points[i].r;
                g[idx] = model_cloud_trans_const->points[i].g;
                b[idx] = model_cloud_trans_const->points[i].b;
              }

            }

            {
              int ws = 3;
              int ws2 = std::floor (static_cast<float> (ws) / 2.f);
              float * depth_smooth = new float[cx_ * cy_];
              uint8_t * r_s = new uint8_t[cx_ * cy_];
              uint8_t * g_s = new uint8_t[cx_ * cy_];
              uint8_t * b_s = new uint8_t[cx_ * cy_];
              Eigen::Vector3f * normals_s = new Eigen::Vector3f[cx_ * cy_];

              memcpy (depth_smooth, depth_, sizeof(float) * cx_ * cy_);
              memcpy (normals_s, normals, sizeof(Eigen::Vector3f) * cx_ * cy_);

              memcpy (r_s, r, sizeof(uint8_t) * cx_ * cy_);
              memcpy (g_s, g, sizeof(uint8_t) * cx_ * cy_);
              memcpy (b_s, b, sizeof(uint8_t) * cx_ * cy_);

              for (int u = ws2; u < (cx_ - ws2); u++)
              {
                for (int v = ws2; v < (cy_ - ws2); v++)
                {
                  //if (!pcl_isfinite(depth_[u * cx_ + v]))
                  // continue;

                  float min = std::numeric_limits<float>::max ();
                  uint8_t min_r;
                  uint8_t min_g;
                  uint8_t min_b;
                  Eigen::Vector3f min_normal;
                  for (int j = (u - ws2); j <= (u + ws2); j++)
                  {
                    for (int i = (v - ws2); i <= (v + ws2); i++)
                    {
                      int idx = i * width_ + j;
                      assert(i < cy_ && j < cx_);
                      assert(idx < cx_ * cy_);

                      if (pcl_isfinite(depth_[idx]) && (depth_[idx] < min))
                      {
                        min = depth_[idx];
                        min_r = r[idx];
                        min_g = g[idx];
                        min_b = b[idx];
                        min_normal = normals[idx];
                      }
                    }
                  }

                  int idx = v * width_ + u;
                  assert(idx < cx_ * cy_);

                  if (min < (std::numeric_limits<float>::max () - 0.1)) //check that min is not inf...
                  {
                    if (((depth_[idx] - min) > 0.01) || !pcl_isfinite(depth_[idx])) //check that the pixel is far away or nan in order to replace it
                    {
                      float new_d = 0.f;
                      int new_r, new_g, new_b;
                      new_r = new_g = new_b = 0;
                      Eigen::Vector3f new_normal;
                      int num = 0;
                      int ws3 = 2;
                      for (int j = (u - ws3); j <= (u + ws3); j++)
                      {
                        for (int i = (v - ws3); i <= (v + ws3); i++)
                        {
                          int idx = i * width_ + j;
                          if(idx >= cx_ * cy_)
                              continue;

                          if (pcl_isfinite(depth_[idx]) && (std::abs (depth_[idx] - min) <= 0.005f))
                          {
                            new_d += depth_[idx];
                            new_r += static_cast<int> (r[idx]);
                            new_g += static_cast<int> (g[idx]);
                            new_b += static_cast<int> (b[idx]);
                            num++;
                            new_normal += normals[idx];
                          }
                        }
                      }

                      uint8_t new_rr;
                      uint8_t new_gg;
                      uint8_t new_bb;

                      if (num != 0)
                      {
                        new_d /= static_cast<float> (num);
                        new_r /= static_cast<float> (num);
                        new_g /= static_cast<float> (num);
                        new_b /= static_cast<float> (num);
                        new_rr = static_cast<uint8_t> (new_r);
                        new_gg = static_cast<uint8_t> (new_g);
                        new_bb = static_cast<uint8_t> (new_b);
                        new_normal /= static_cast<float>(num);
                      }
                      else
                      {
                        new_d = min;
                        new_rr = min_r;
                        new_gg = min_g;
                        new_bb = min_b;
                        new_normal = min_normal;
                      }

                      depth_smooth[idx] = new_d;
                      r_s[idx] = new_rr;
                      g_s[idx] = new_gg;
                      b_s[idx] = new_bb;
                      normals_s[idx] = new_normal;
                    }
                  }
                }
              }

              memcpy (depth_, depth_smooth, sizeof(float) * cx_ * cy_);
              memcpy (normals, normals_s, sizeof(Eigen::Vector3f) * cx_ * cy_);

              memcpy (r, r_s, sizeof(uint8_t) * cx_ * cy_);
              memcpy (g, g_s, sizeof(uint8_t) * cx_ * cy_);
              memcpy (b, b_s, sizeof(uint8_t) * cx_ * cy_);
              delete[] r_s;
              delete[] g_s;
              delete[] b_s;
              delete[] normals_s;

              //erode
              int times_erode = ws2;
              ws2 = 1;
              while (times_erode > 0)
              {
                for (int u = ws2; u < (cx_ - ws2); u++)
                {
                  for (int v = ws2; v < (cy_ - ws2); v++)
                  {

                    int idx = v * width_ + u;

                    bool to_erode = false;
                    int nans = 0;
                    for (int j = (u - ws2); j <= (u + ws2); j++)
                    {
                      for (int i = (v - ws2); i <= (v + ws2); i++)
                      {

                        int idx = i * width_ + j;
                        if (!pcl_isfinite(depth_smooth[idx]))
                        {
                          to_erode = true;
                          nans++;
                        }
                      }
                    }

                    if (to_erode && ((pow ((ws2 + 1) * 2, 2) - 1) >= 3))
                    {
                      depth_[idx] = std::numeric_limits<float>::quiet_NaN ();
                    }
                  }
                }
                times_erode--;
              }

              delete[] depth_smooth;

            }

            for (int cx = 0; cx < cx_; cx++)
            {
              for (int cy = 0; cy < cy_; cy++)
              {
                if (pcl_isfinite(depth_[cy * cx_ + cx]))
                {
                  depth_map->at (cx, cy).x = ((cx - (cx_ / 2)) * depth_[cy * cx_ + cx]) / f_;
                  depth_map->at (cx, cy).y = ((cy - (cy_ / 2)) * depth_[cy * cx_ + cx]) / f_;
                  depth_map->at (cx, cy).z = depth_[cy * cx_ + cx];
                  depth_map->at (cx, cy).getNormalVector3fMap() = normals[cy * cx_ + cx];

                  depth_map->at (cx, cy).r = r[cy * cx_ + cx];
                  depth_map->at (cx, cy).g = g[cy * cx_ + cx];
                  depth_map->at (cx, cy).b = b[cy * cx_ + cx];
                }
                else
                {
                  depth_map->at (cx, cy).x = std::numeric_limits<float>::quiet_NaN ();
                  depth_map->at (cx, cy).y = std::numeric_limits<float>::quiet_NaN ();
                  depth_map->at (cx, cy).z = std::numeric_limits<float>::quiet_NaN ();
                  depth_map->at (cx, cy).r = 255;
                  depth_map->at (cx, cy).g = 255;
                  depth_map->at (cx, cy).b = 0;
                }
              }
            }

            pcl::copyPointCloud (*depth_map, *filtered);
          }

          delete[] normals;
          delete[] depth_;
          delete[] r;
          delete[] g;
          delete[] b;
        }

      std::vector<int> keep;
      std::vector<int> set_to_nan;

      for (size_t k = 0; k < filtered->points.size (); k++)
      {
        Eigen::Vector3f normal_p = filtered->points[k].getNormalVector3fMap ();
        //Eigen::Vector3f normal_vp = filtered->points[k].getVector3fMap () * -1.f;
        Eigen::Vector3f normal_vp = Eigen::Vector3f::UnitZ() * -1.f;

        normal_p.normalize ();
        normal_vp.normalize ();

        if (normal_p.dot (normal_vp) > dot_normal_)
          keep.push_back (static_cast<int> (k));
        else
          set_to_nan.push_back (static_cast<int> (k));
      }

      typename pcl::PointCloud<PointInT>::Ptr filtered2 (new pcl::PointCloud<PointInT> ());
      if(!gen_organized_)
      {
        pcl::copyPointCloud (*filtered, keep, *filtered2);
      }
      else
      {
        pcl::copyPointCloud (*filtered, *filtered2);
        float bad_point = std::numeric_limits<float>::quiet_NaN();
        for(size_t k=0; k < set_to_nan.size(); k++)
        {
          filtered2->points[set_to_nan[k]].x = filtered2->points[set_to_nan[k]].y = filtered2->points[set_to_nan[k]].z = bad_point;
          filtered2->points[set_to_nan[k]].r = 255;
          filtered2->points[set_to_nan[k]].g = 255;
          filtered2->points[set_to_nan[k]].b = 0;
        }
      }

      /*{
        pcl::visualization::PointCloudColorHandlerRGBField<Full3DPointT> handler(model_cloud_trans);
        vis_test.addPointCloud<Full3DPointT>(model_cloud_trans, handler, "transformed5");
      }

      {
        pcl::visualization::PointCloudColorHandlerRGBField<PointInT> handler(filtered2);
        vis_test.addPointCloud<PointInT>(filtered2, handler, "transformed6");
      }

      vis_test.spin();
      vis_test.removeAllPointClouds();*/

      //save transform from OC to CC
      Eigen::Matrix4f final_mat, trans1, trans2;
      final_mat.setIdentity (4,4);
      trans1.setIdentity (4,4);
      trans2.setIdentity (4,4);
      trans1.block<3, 1> (0, 3) = obj_centroid3f * -1.f;
      trans2.block<3, 1> (0, 3) = bb.head<3> () * -1.f;
      //trans2.block<3, 1> (0, 3) = cam_positions[i] * -1.f;
      final_mat = trans2 * inv * trans1 * flip_y;
      //std::cout << final_mat << std::endl;
      /*poses.push_back (final_mat);

      if(gen_organized_)
      {
        pcl::PointIndices indices_cloud;
        indices_cloud.indices = keep;
        indices.push_back(indices_cloud);
      }

      views_orig.push_back (filtered2);*/

      std::stringstream path_view;
      path_view << direc.str () << "/view_" << std::setfill ('0') << std::setw (8) << i << ".pcd";
      std::cout << filtered2->points.size() << std::endl;
      pcl::io::savePCDFileBinary (path_view.str (), *filtered2);

      std::stringstream path_pose;
      path_pose << direc.str () << "/pose_" << std::setfill ('0') << std::setw (8) << i << ".txt";
      PersistenceUtils::writeMatrixToFile (path_pose.str (), final_mat);

      /*std::stringstream path_entropy;
      path_entropy << direc.str () << "/entropy_" << i << ".txt";
      PersistenceUtils::writeFloatToFile (path_entropy.str (), model.self_occlusions_->at (i));*/

      if(gen_organized_)
      {
        pcl::PointIndices indices_cloud;
        indices_cloud.indices = keep;
        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        obj_indices_cloud.resize(indices_cloud.indices.size());
        obj_indices_cloud.width = indices_cloud.indices.size();
        obj_indices_cloud.height = 1;

        for(size_t k=0; k <indices_cloud.indices.size(); k++)
        {
          obj_indices_cloud.points[k].idx = indices_cloud.indices[k];
        }

        std::stringstream path_oi;
        path_oi << direc.str () << "/object_indices_" << std::setfill ('0') << std::setw (8) << i << ".pcd";
        pcl::io::savePCDFileBinary(path_oi.str(), obj_indices_cloud);
        //model.indices_->push_back (indices_cloud);
      }

      //model.views_->push_back (filtered2);
      //model.poses_->push_back (final_mat);
    }

    /*if(gen_organized_)
    {
        typename pcl::PointCloud<PointInT>::Ptr model_cloud(new pcl::PointCloud<PointInT>);
        assembleModelFromViewsAndPoses(model, *(model.poses_), *(model.indices_), model_cloud);

        pcl::visualization::PCLVisualizer vis ("assembled model...");
        int v1,v2;
        vis.createViewPort(0,0,0.5,1,v1);
        vis.createViewPort(0.5,0,1,1,v2);

        {
            pcl::visualization::PointCloudColorHandlerRGBField<PointInT> random_handler (model_cloud);
            vis.addPointCloud<PointInT> (model_cloud, random_handler, "points", v1);
        }

        {
            pcl::visualization::PointCloudColorHandlerRGBField<PointInT> random_handler (model.assembled_);
            vis.addPointCloud<PointInT> (model.assembled_, random_handler, "points_original", v2);
        }
        vis.addCoordinateSystem(0.1);
        vis.spin ();
    }*/

    /*model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr> ());
    model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > ());
    model.self_occlusions_.reset (new std::vector<float> ());
    if(gen_organized_)
    {
      model.indices_.reset(new std::vector<pcl::PointIndices>);
    }

    for (size_t i = 0; i < views_orig.size (); i++)
    {

      if(views_orig[i]->points.size() > 0)
      {
        model.views_->push_back (views_orig[i]);
        model.poses_->push_back (poses[i]);
        model.self_occlusions_->push_back (0.f);
        if(gen_organized_)
        {
          model.indices_->push_back (indices[i]);
        }
      }
    }*/

    /*std::stringstream direc;
    direc << dir << "/" << model.class_ << "/" << model.id_;
    createClassAndModelDirectories (dir, model.class_, model.id_);

    for (size_t i = 0; i < model.views_->size (); i++)
    {
      //save generated model for future use
      std::stringstream path_view;
      path_view << direc.str () << "/view_" << i << ".pcd";
      std::cout << model.views_->at (i)->points.size() << std::endl;
      pcl::io::savePCDFileBinary (path_view.str (), *(model.views_->at (i)));

      std::stringstream path_pose;
      path_pose << direc.str () << "/pose_" << i << ".txt";

      PersistenceUtils::writeMatrixToFile (path_pose.str (), model.poses_->at (i));

      std::stringstream path_entropy;
      path_entropy << direc.str () << "/entropy_" << i << ".txt";
      PersistenceUtils::writeFloatToFile (path_entropy.str (), model.self_occlusions_->at (i));

      if(gen_organized_)
      {

        pcl::PointCloud<IndexPoint> obj_indices_cloud;
        obj_indices_cloud.resize(model.indices_->at(i).indices.size());
        obj_indices_cloud.width = model.indices_->at(i).indices.size();
        obj_indices_cloud.height = 1;

        for(size_t k=0; k < model.indices_->at(i).indices.size(); k++)
        {
          obj_indices_cloud.points[k].idx = model.indices_->at(i).indices[k];
        }

        std::stringstream path_oi;
        path_oi << direc.str () << "/object_indices_" << i << ".pcd";
        pcl::io::savePCDFileBinary(path_oi.str(), obj_indices_cloud);
      }
    }*/

    loadOrGenerate (dir, model_path, model);
  }
}

template<typename Full3DPointT, typename PointInT, typename OutModelPointT>
void
faat_pcl::rec_3d_framework::PartialPCDSource<Full3DPointT, PointInT, OutModelPointT>::loadInMemorySpecificModelAndView(std::string & dir, ModelT & model, int view_id)
{
  std::stringstream pathmodel;
  pathmodel << dir << "/" << model.class_ << "/" << model.id_;
  bf::path trained_dir = pathmodel.str ();

  model.views_->clear();
  model.poses_->clear();
  if(gen_organized_)
   model.indices_->clear();

  int i = view_id;
  std::stringstream view_file;
  view_file << pathmodel.str () << "/" << model.view_filenames_[i];
  std::cout << view_file.str() << std::endl;
  typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
  pcl::io::loadPCDFile (view_file.str (), *cloud);

  model.views_->push_back (cloud);

  std::string file_replaced1 (model.view_filenames_[i]);
  boost::replace_all (file_replaced1, "view", "pose");
  boost::replace_all (file_replaced1, ".pcd", ".txt");

  std::string file_replaced2 (model.view_filenames_[i]);
  boost::replace_all (file_replaced2, "view", "entropy");
  boost::replace_all (file_replaced2, ".pcd", ".txt");

  //read pose as well
  std::stringstream pose_file;
  pose_file << pathmodel.str () << "/" << file_replaced1;

  Eigen::Matrix4f pose;
  PersistenceUtils::readMatrixFromFile (pose_file.str (), pose);

  model.poses_->push_back (pose);

  if(gen_organized_)
  {
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

    model.indices_->push_back(indices);
  }
}

template<typename Full3DPointT, typename PointInT, typename OutModelPointT>
void
faat_pcl::rec_3d_framework::PartialPCDSource<Full3DPointT, PointInT, OutModelPointT>::assembleModelFromViewsAndPoses(ModelT & model,
                                   std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > & poses,
                                   std::vector<pcl::PointIndices> & indices,
                                   typename pcl::PointCloud<PointInT>::Ptr &model_cloud) {
  for(size_t i=0; i < model.views_->size(); i++) {
    //Eigen::Matrix4f inv = poses[i].transpose();
    Eigen::Matrix4f inv = poses[i].inverse();

    typename pcl::PointCloud<PointInT>::Ptr global_cloud_only_indices(new pcl::PointCloud<PointInT>);
    pcl::copyPointCloud(*(model.views_->at(i)), indices[i], *global_cloud_only_indices);
    typename pcl::PointCloud<PointInT>::Ptr global_cloud(new pcl::PointCloud<PointInT>);
    pcl::transformPointCloud(*global_cloud_only_indices,*global_cloud, inv);
    *(model_cloud) += *global_cloud;
  }
}

template<typename Full3DPointT, typename PointInT, typename OutModelPointT>
void
faat_pcl::rec_3d_framework::PartialPCDSource<Full3DPointT, PointInT, OutModelPointT>::loadInMemorySpecificModel(std::string & dir, ModelT & model)
{
  PCL_WARN("Loading into memory %d views \n", static_cast<int>(model.view_filenames_.size ()));
  std::stringstream pathmodel;
  pathmodel << dir << "/" << model.class_ << "/" << model.id_;
  bf::path trained_dir = pathmodel.str ();

  model.views_.reset (new std::vector<typename pcl::PointCloud<PointInT>::Ptr>);
  model.poses_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);
  model.self_occlusions_.reset (new std::vector<float>);
  if(gen_organized_)
  {
    model.indices_.reset(new std::vector<pcl::PointIndices>);
  }

  for (size_t i = 0; i < model.view_filenames_.size (); i++)
  {
    std::stringstream view_file;
    view_file << pathmodel.str () << "/" << model.view_filenames_[i];
    std::cout << view_file.str() << std::endl;
    typename pcl::PointCloud<PointInT>::Ptr cloud (new pcl::PointCloud<PointInT> ());
    pcl::io::loadPCDFile (view_file.str (), *cloud);

    model.views_->push_back (cloud);

    std::string file_replaced1 (model.view_filenames_[i]);
    boost::replace_all (file_replaced1, "view", "pose");
    boost::replace_all (file_replaced1, ".pcd", ".txt");

    std::string file_replaced2 (model.view_filenames_[i]);
    boost::replace_all (file_replaced2, "view", "entropy");
    boost::replace_all (file_replaced2, ".pcd", ".txt");

    //read pose as well
    std::stringstream pose_file;
    pose_file << pathmodel.str () << "/" << file_replaced1;

    std::cout << pose_file.str() << std::endl;

    Eigen::Matrix4f pose;
    PersistenceUtils::readMatrixFromFile2 (pose_file.str (), pose);

    std::cout << pose << std::endl;
    model.poses_->push_back (pose);

    //read entropy as well
    std::stringstream entropy_file;
    entropy_file << pathmodel.str () << "/" << file_replaced2;
    float entropy = 0;
    PersistenceUtils::readFloatFromFile (entropy_file.str (), entropy);
    model.self_occlusions_->push_back (entropy);

    if(gen_organized_)
    {
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

      model.indices_->push_back(indices);
    }
  }

/*if(gen_organized_)
{
    typename pcl::PointCloud<PointInT>::Ptr model_cloud(new pcl::PointCloud<PointInT>);
    assembleModelFromViewsAndPoses(model, *(model.poses_), *(model.indices_), model_cloud);

    pcl::visualization::PCLVisualizer vis ("assembled model...");
    pcl::visualization::PointCloudColorHandlerRGBField<PointInT> random_handler (model_cloud);
    vis.addPointCloud<PointInT> (model_cloud, random_handler, "points");
    vis.addCoordinateSystem(0.1);
    vis.spin ();
}*/

}

#endif /* PARTIAL_PCD_SOURCE_H_ */
