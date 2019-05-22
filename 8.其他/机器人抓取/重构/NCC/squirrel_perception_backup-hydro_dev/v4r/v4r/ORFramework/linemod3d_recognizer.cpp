#include "linemod3d_recognizer.h"
#include "faat_3d_rec_framework_defines.h"
#include <v4r/ORUtils/pcl_opencv.h>
#include <pcl/common/angles.h>
//add functions

inline std::vector<int> getViewIdsFromTemplateViewFile(std::string file)
{
    std::vector<int> view_ids;
    std::ifstream in;
    in.open (file.c_str (), std::ifstream::in);

    while (!in.eof())
    {
      char linebuf[1024];
      in.getline (linebuf, 1024);
      std::string line (linebuf);
      boost::replace_all (line, "view_", "");
      boost::replace_all (line, ".pcd", "");
      view_ids.push_back(atoi(line.c_str()));
    }

    view_ids.resize(view_ids.size() - 1);
    return view_ids;
}

template<typename PointInT>
inline
void rotateOrganizedPointCloud(typename pcl::PointCloud<PointInT>::Ptr & cloud,
                                   pcl::PointIndices & indices,
                                   float angle,
                                   typename pcl::PointCloud<PointInT>::Ptr & rotated,
                                   pcl::PointIndices & rotated_indices)
{
  /*if(angle == 0.f)
  {
    rotated_indices = indices;
    rotated = cloud;
    return;
  }*/

  rotated.reset(new pcl::PointCloud<PointInT>);
  Eigen::Matrix3f m;
  m = Eigen::AngleAxisf(pcl::deg2rad(angle), Eigen::Vector3f::UnitZ());
  Eigen::Matrix4f roll_trans;
  roll_trans.setIdentity();
  roll_trans.block<3,3>(0,0) = m;

  std::cout << roll_trans << std::endl;
  rotated->width = cloud->width;
  rotated->height = cloud->height;
  rotated->points.resize(cloud->points.size());

  float bad_point = std::numeric_limits<float>::quiet_NaN();
  for(size_t i=0; i < rotated->points.size(); i++)
  {
    rotated->points[i].x = rotated->points[i].y = rotated->points[i].z = bad_point;
    rotated->points[i].r = 255;
    rotated->points[i].g = 255;
    rotated->points[i].b = 0;
  }

  float rad_angle = pcl::deg2rad(angle);
  int anchor_i, anchor_j;
  anchor_i = rotated->height / 2;
  anchor_j = rotated->width / 2;
  for(size_t k=0; k < indices.indices.size(); k++)
  {
    int i,j;
    i = indices.indices[k] / rotated->width;
    j = indices.indices[k] % rotated->width;
    PointInT p;
    p.getVector4fMap() = roll_trans * cloud->points[indices.indices[k]].getVector4fMap();
    p.rgb = cloud->points[indices.indices[k]].rgb;

    //new i,j within rotated
    int new_i, new_j;
    new_i = anchor_i + ( (j - anchor_j) * cos(rad_angle) - (i - anchor_i) * sin(rad_angle));
    new_j = anchor_j + ( (j - anchor_j) * sin(rad_angle) + (i - anchor_i) * cos(rad_angle));
    //std::cout << new_i << " " << new_j << std::endl;
    rotated->at(new_j, new_i) = p;

    int idx = new_i * rotated->width + new_j;
    rotated_indices.indices.push_back(idx);
  }
}

template<typename PointInT>
void
faat_pcl::rec_3d_framework::LineMod3DPipeline<PointInT>::initialize (bool force_retrain)
  {
    boost::shared_ptr < std::vector<ModelTPtr> > models;

    if(search_model_.compare("") == 0) {
      models = source_->getModels ();
    } else {
      models = source_->getModels (search_model_);
      poses_cache_.clear();
    }

    std::cout << "Models size:" << models->size () << std::endl;

    if (force_retrain)
    {
      for (size_t i = 0; i < models->size (); i++)
      {
        source_->removeDescDirectory (*models->at (i), training_dir_, descr_name_);
      }
    }

    for (size_t i = 0; i < models->size (); i++)
    {
      std::cout << models->at (i)->class_ << " " << models->at (i)->id_ << std::endl;

      if (!source_->modelAlreadyTrained (*models->at (i), training_dir_, descr_name_))
      {
        std::cout << "Model not trained..." << models->at (i)->views_->size () << std::endl;
        std::cout << "Model not trained... view filenames size:" << models->at (i)->view_filenames_.size () << std::endl;
        /*if(!source_->getLoadIntoMemory())
          source_->loadInMemorySpecificModel(training_dir_, *(models->at (i)));*/

        std::cout << "Model not trained... views in memory:" << models->at (i)->views_->size () << std::endl;

        pcl::LINEMOD linemod;

        std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);
        bf::path desc_dir = path;
        if (!bf::exists (desc_dir))
          bf::create_directory (desc_dir);

        //for (size_t v = 0; v < models->at (i)->views_->size (); v++)
        for (size_t kk = 0; kk < models->at (i)->view_filenames_.size (); kk++)
        {
          source_->loadInMemorySpecificModelAndView(training_dir_, *(models->at (i)), (int)kk);
          int v = 0;

          /*cv::Mat_ < cv::Vec3b > colorImage;
          PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGBA> (models->at (i)->views_->at (v), colorImage);
          cv::namedWindow("test");
          cv::imshow("test", colorImage);
          cv::waitKey(0);*/

          //use models->at(i)->indices_->at (v).indices to define the mask to compute the linemod template
          //input cloud is models->at (i)->views_->at (v)

          pcl::MaskMap mask;
          pcl::RegionXY region;
          getMaskFromObjectIndices(models->at(i)->indices_->at (v), models->at (i)->views_->at (v), mask, region);

          typename pcl::ColorGradientModality<PointInT> color_grad_mod;
          color_grad_mod.setInputCloud (models->at (i)->views_->at (v));
          color_grad_mod.processInputData ();

          typename pcl::SurfaceNormalModality<PointInT> surface_norm_mod;
          surface_norm_mod.setInputCloud (models->at (i)->views_->at (v));
          surface_norm_mod.processInputData ();

          std::vector<pcl::QuantizableModality*> modalities (2);
          modalities[0] = &color_grad_mod;
          modalities[1] = &surface_norm_mod;

          std::vector<pcl::MaskMap*> masks (2);
          masks[0] = &mask;
          masks[1] = &mask;

          linemod.createAndAddTemplate(modalities, masks, region);

          /*std::stringstream path_view;
          path_view << path << "/view_" << v << ".pcd";
          pcl::io::savePCDFileBinary (path_view.str (), *models->at (i)->views_->at (v));

          std::stringstream path_pose;
          path_pose << path << "/pose_" << v << ".txt";
          PersistenceUtils::writeMatrixToFile (path_pose.str (), models->at (i)->poses_->at (v));*/
        }

        //save all templates in file
        std::cout << "Number of templates:" << linemod.getNumOfTemplates() << std::endl;

        std::stringstream path_descriptor;
        path_descriptor << path << "/linemod_templates.lmt";
        linemod.saveTemplates(path_descriptor.str().c_str());

        //save a text file explaining how the template id is related to the view_id
        std::stringstream file;
        file << path << "/template_to_view_id.txt";
        std::ofstream out (file.str().c_str ());
        for (size_t v = 0; v < models->at (i)->view_filenames_.size (); v++)
        {
          out << models->at (i)->view_filenames_[v] << std::endl;
        }

        out.close();

        if(!source_->getLoadIntoMemory())
          models->at (i)->views_->clear();

      } else {
        std::cout << "Model already trained..." << std::endl;
        //there is no need to keep the views in memory once the model has been trained
        models->at (i)->views_->clear();
      }
    }

    //once we are here, its time to load templates
    std::vector<std::string> lmt_filenames;
    for (size_t i = 0; i < models->size (); i++)
    {
      pcl::ScopeTime t("Model finished");

      std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);
      std::stringstream path_descriptor;
      path_descriptor << path << "/linemod_templates.lmt";
      lmt_filenames.push_back(path_descriptor.str());
      std::cout << lmt_filenames[i] << std::endl;
    }

    //TODO: Load poses... and manage structure to handle linemod detections...
    //might be tricky to handle things in the right order... template_id and views should be accurately matched
    //load templates loa
    linemod_.loadTemplates(lmt_filenames);

    template_id_to_model_.resize(linemod_.getNumOfTemplates());
    int templates_per_model = linemod_.getNumOfTemplates() / models->size();
    std::cout << "Number of templates loaded from multiple files:" << linemod_.getNumOfTemplates()  << " x model:" << templates_per_model << std::endl;

    int t = 0;
    for (size_t i = 0; i < models->size (); i++)
    {
      std::string path = source_->getModelDescriptorDir (*models->at (i), training_dir_, descr_name_);
      std::stringstream file;
      file << path << "/template_to_view_id.txt";
      std::vector<int> view_ids = getViewIdsFromTemplateViewFile(file.str());
      for(size_t j=0; j < view_ids.size(); j++, t++)
      {
        template_id_to_model_[t].model_ = models->at(i) ;
        template_id_to_model_[t].view_id_ = view_ids[j];
      }
    }

    std::cout << t << " " << linemod_.getNumOfTemplates() << std::endl;

    if(ICP_iterations_ > 0 && icp_type_ == 1)
      source_->createVoxelGridAndDistanceTransform(VOXEL_SIZE_ICP_);

  }

template<typename PointInT>
void
faat_pcl::rec_3d_framework::LineMod3DPipeline<PointInT>::recognize ()
  {

    models_.reset (new std::vector<ModelTPtr>);
    transforms_.reset (new std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >);

    std::cout << "Number of hypotheses:" << models_->size() << std::endl;

    if (ICP_iterations_ > 0 || hv_algorithm_) {
      //Prepare scene and model clouds for the pose refinement step
      source_->voxelizeAllModels (VOXEL_SIZE_ICP_);
    }

    typename pcl::ColorGradientModality<PointInT> color_grad_mod;
    color_grad_mod.setInputCloud (input_);
    color_grad_mod.processInputData ();

    pcl::SurfaceNormalModality<PointInT> surface_norm_mod;
    surface_norm_mod.setInputCloud (input_);
    surface_norm_mod.processInputData ();

    std::vector<pcl::QuantizableModality*> modalities (2);
    modalities[0] = &color_grad_mod;
    modalities[1] = &surface_norm_mod;

    std::cout << "Number of templates loaded from multiple files:" << linemod_.getNumOfTemplates() << std::endl;

    std::vector<pcl::LINEMODDetection> detections;
    linemod_.detectTemplatesSemiScaleInvariant (modalities, detections);
    //linemod_.detectTemplates (modalities, detections);
    std::cout << detections.size() << std::endl;

    std::vector< std::pair<size_t, float> > template_id_response;
    for(size_t d=0; d < detections.size(); d++)
    {
      //std::cout << detections[d].template_id << " " << detections[d].response << " " << linemod_id_to_template[detections[d].object_id].template_id << std::endl;
      template_id_response.push_back(std::make_pair(d, detections[d].score));
    }

    std::sort(template_id_response.begin(), template_id_response.end(),
              boost::bind(&std::pair<size_t, float>::second, _1) >
              boost::bind(&std::pair<size_t, float>::second, _2));

    cv::Mat_ < cv::Vec3b > colorImage;
    PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGBA> (input_, colorImage);
    cv::Mat collage(colorImage.rows, colorImage.cols*2, CV_8UC3);
    int n_detections_to_show_ = 100;

    for(size_t d=0; d < std::min(n_detections_to_show_, (int)detections.size()); d++)
    {
      std::cout << detections[template_id_response[d].first].template_id << " " << template_id_response[d].second << std::endl;

      const pcl::SparseQuantizedMultiModTemplate & linemod_template =
        linemod_.getTemplate (detections[template_id_response[d].first].template_id);

      cv::Point tl = cv::Point(detections[template_id_response[d].first].x, detections[template_id_response[d].first].y);
      cv::Point br = cv::Point(detections[template_id_response[d].first].x + linemod_template.region.width * detections[template_id_response[d].first].scale,
                               detections[template_id_response[d].first].y + linemod_template.region.height * detections[template_id_response[d].first].scale);
      /*cv::Point br = cv::Point(detections[template_id_response[d].first].x + linemod_template.region.width,
                               detections[template_id_response[d].first].y + linemod_template.region.height);*/

      cv::Scalar color = cv::Scalar( 255, 0, 0);
      cv::rectangle( colorImage, tl, br, color, 2, 8, 0 );

      collage(cv::Range(0, collage.rows), cv::Range(0, colorImage.cols)) = colorImage +cv::Scalar(0);

      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene_match(new pcl::PointCloud<pcl::PointXYZRGBA>);
      ModelTPtr model = template_id_to_model_[detections[template_id_response[d].first].template_id].model_;
      int view_id = template_id_to_model_[detections[template_id_response[d].first].template_id].view_id_;
      std::cout << model->id_ << " " << view_id << std::endl;
      getView(*model, view_id, scene_match);

      cv::Mat_ < cv::Vec3b > colorImage_match;
      PCLOpenCV::ConvertPCLCloud2Image<pcl::PointXYZRGBA> (scene_match, colorImage_match);
      collage(cv::Range(0, collage.rows), cv::Range(collage.cols/2, collage.cols)) = colorImage_match +cv::Scalar(0);

      cv::namedWindow("test");
      cv::imshow("test", collage);
      cv::waitKey(0);
    }

    /**
     * POSE REFINEMENT
     **/

    if (ICP_iterations_ > 0)
    {
      poseRefinement();
    }

    /**
     * HYPOTHESES VERIFICATION
     **/

    if (hv_algorithm_ && (models_->size () > 0))
    {
      hypothesisVerification();
    }
  }

template<typename PointInT>
void
faat_pcl::rec_3d_framework::LineMod3DPipeline<PointInT>::getView (ModelT & model, int view_id, PointInTPtr & view)
{
  view.reset (new pcl::PointCloud<PointInT>);
  std::stringstream dir;
  //std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
  std::string path = source_->getModelDirectory (model, training_dir_);
  dir << path << "/view_" << std::setfill ('0') << std::setw (8) << view_id << ".pcd";
  pcl::io::loadPCDFile (dir.str (), *view);

}

template<typename PointInT>
void
faat_pcl::rec_3d_framework::LineMod3DPipeline<PointInT>::getPose (ModelT & model, int view_id, Eigen::Matrix4f & pose_matrix)
{

  if (use_cache_)
  {
    typedef std::pair<std::string, int> mv_pair;
    mv_pair pair_model_view = std::make_pair (model.id_, view_id);

    std::map<mv_pair, Eigen::Matrix4f, std::less<mv_pair>, Eigen::aligned_allocator<std::pair<mv_pair, Eigen::Matrix4f> > >::iterator it =
        poses_cache_.find (pair_model_view);

    if (it != poses_cache_.end ())
    {
      pose_matrix = it->second;
      return;
    }

  }

  std::stringstream dir;
  //std::string path = source_->getModelDescriptorDir (model, training_dir_, descr_name_);
  std::string path = source_->getModelDirectory (model, training_dir_);
  dir << path << "/pose_" << std::setfill ('0') << std::setw (8) << view_id << ".txt";

  PersistenceUtils::readMatrixFromFile (dir.str (), pose_matrix);
}


template class PCL_EXPORTS faat_pcl::rec_3d_framework::LineMod3DPipeline<pcl::PointXYZRGBA>;
