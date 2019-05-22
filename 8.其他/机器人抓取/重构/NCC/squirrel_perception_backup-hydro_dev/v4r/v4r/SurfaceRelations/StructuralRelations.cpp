/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova, Andreas Richtsfeld, Johann Prankl, Thomas Mörwald, Michael Zillich
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienna, Austria
 *    ari(at)acin.tuwien.ac.at
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see http://www.gnu.org/licenses/
 */

/**
 * @file StructuralRelations.cpp
 * @author Richtsfeld
 * @date December 2012
 * @version 0.1
 * @brief Calculate patch relations for structural level: Efficient version without fourier and gabor filter.
 */

#include "StructuralRelations.h"

namespace surface
{


void StructuralRelations::init()
{
  if((!have_cloud) || (!have_surfaces) || (!have_normals) || (!have_neighbours2D) || (!have_neighbours3D)) //|| (!have_relations))
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud, surfaces, and neighbours.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  std::vector<std::vector<Relation> > relations;
//   surface::Relation rel;
//   rel.valid = false;
  relations.resize(surfaces.size());
  
  #pragma omp parallel for
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    relations.at(i).resize(surfaces.size());
    for(unsigned int j = i+1; j < surfaces.size(); j++)
    {
      Relation r;
      r.groundTruth = -1;
      r.prediction = -1;
      r.type = STRUCTURAL_RELATIONS;                                       // structural level = 1
      r.id_0 = i;
      r.id_1 = j;
      
      r.valid = false;
//       surfaceRelations.push_back(r);
      relations.at(i).at(j) = r;
    }
  }
  
  // copy relations to view
  surfaceRelations.clear();
  for(unsigned int i=0; i<surfaces.size(); i++)
  {
    for(unsigned int j=i+1; j<surfaces.size(); j++)
    {
      surfaceRelations.push_back(relations.at(i).at(j));
//       printf("r_st_l: [%u][%u]: ", relations.at(i).at(j).id_0, relations.at(i).at(j).id_1);
//       printf("\n");
    }
  }

  cloud_model.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*cloud, *cloud_model);

  // for the texture
  //EPUtils::PointCloudXYZRGB2RGB(cloud,matImage);
  EPUtils::pointCloud_2_rgb(matImage,cloud,cloud->width,cloud->height);
  
  double lowThreshold = 5.;
  double highThreshold = 140.;
  int kernel_size = 3;
  
  cv::cvtColor(matImage, gray_image, CV_BGR2GRAY );
  //
  cv::cvtColor(matImage, gray_image2, CV_RGB2GRAY );
  cv::blur(gray_image, edges, cv::Size(3,3));
  cv::Canny(edges, edges, lowThreshold, highThreshold, kernel_size);

  if(usedRelations & R_COS)
  {
    hist.resize(surfaces.size());
  }
  
  if(usedRelations & R_TR)
  {
    text.resize(surfaces.size());
  }
  
  if(usedRelations & R_FS)
  {
    fourier.resize(surfaces.size());
  }
  
  if(usedRelations & R_GS)
  {
    permanentGabor.reset( new Gabor() );
    permanentGabor->setInputImage(gray_image2); //gray_image
    permanentGabor->computeGaborFilters();
    
    gabor.resize(surfaces.size());
  }

  initialized = true;
  
}
  
/************************************************************************************
 * Constructor/Destructor
 */

StructuralRelations::StructuralRelations()
: EPBase()
{
  have_surfaces = false;
//   have_relations = false;
  have_neighbours2D = false;
  have_neighbours3D = false;
  usedRelations = 0x7FF;
  initialized = false;
  
  trainMode = false;
  
  ClassName = "StructuralRelations";
}

StructuralRelations::~StructuralRelations()
{
}

/**Set training mode **/
void StructuralRelations::setTrainingMode(bool _trainMode)
{
  trainMode = _trainMode;
}

/**Set used relations **/
void StructuralRelations::setUsedRelations(int _usedRelations)
{
  usedRelations = _usedRelations;
}

//
void StructuralRelations::setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
}

// void StructuralRelations::setRelations(const std::vector<Relation> _surfaceRelations)
// {
//   surfaceRelations = _surfaceRelations;
//   have_relations = true;
// }

void StructuralRelations::setNeighbours2D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr2D_map)
{
  ngbr2D_map = _ngbr2D_map;
  have_neighbours2D = true;
}

void StructuralRelations::setNeighbours3D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr3D_map)
{
  ngbr3D_map = _ngbr3D_map;
  have_neighbours3D = true;
}

void StructuralRelations::projectPts2Model()
{  
 
  #pragma omp parallel for
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {

    if(!(surfaces.at(i)->isNew))
      continue;

    if((surfaces.at(i)->type == pcl::SACMODEL_PLANE) && (surfaces.at(i)->coeffs.size() == 4))
    {
      pcl::PointIndices::Ptr surface_indices(new pcl::PointIndices);
      surface_indices->indices = surfaces.at(i)->indices;
      pcl::ModelCoefficients::Ptr mc (new pcl::ModelCoefficients);
      mc->values = surfaces.at(i)->coeffs;
	
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(cloud);
      proj.setIndices(surface_indices);
      proj.setModelCoefficients(mc);
      proj.filter(*new_cloud);
      for (unsigned int j = 0; j < new_cloud->points.size(); j++)
        (*cloud_model).points.at(surface_indices->indices.at(j)) = new_cloud->points.at(j);
    }
  }
}

void StructuralRelations::compute()
{
  if(!initialized)
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first call init() function.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  projectPts2Model();
  
  if(usedRelations & R_COS)
  {
  
    #pragma omp parallel for
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      int nrHistBins = 4;
      double uvThreshold = 0.0f;

      if( (!(surfaces.at(i)->selected)) || (!(surfaces.at(i)->isNew)) )
      {
        continue;
      }
      
      hist.at(i).reset( new ColorHistogram(nrHistBins,uvThreshold) );

      hist.at(i)->setInputCloud(cloud);
      hist.at(i)->setIndices(surfaces.at(i)->indices);
      hist.at(i)->compute();
    }
  }

  if(usedRelations & R_TR)
  {
    #pragma omp parallel for
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      if( (!(surfaces.at(i)->selected)) || (!(surfaces.at(i)->isNew)) )
      {
        continue;
      }
      
      text.at(i).reset( new Texture() );

      text.at(i)->setInputEdges(edges);
      text.at(i)->setIndices(surfaces.at(i)->indices);

      text.at(i)->compute();
    }
  }

  if(usedRelations & R_FS)
  {
    #pragma omp parallel for
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      if( (!(surfaces.at(i)->selected)) || (!(surfaces.at(i)->isNew)) )
      {
        continue;
      }
      
      fourier.at(i).reset( new Fourier() );

      fourier.at(i)->setInputImage(gray_image2); //gray_image
      fourier.at(i)->setIndices(surfaces.at(i)->indices);

      fourier.at(i)->compute();
    }
  }

  if(usedRelations & R_GS)
  {

    Gabor::Ptr permanentGabor2 = permanentGabor;
    
//     Gabor::Ptr permanentGabor;
//     permanentGabor.reset( new Gabor() );
//     permanentGabor->setInputImage(gray_image2); //gray_image
//     permanentGabor->computeGaborFilters();

    #pragma omp parallel for shared(permanentGabor2)
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      if( (!(surfaces.at(i)->selected)) || (!(surfaces.at(i)->isNew)) )
      {
        continue;
      }
      
      gabor.at(i).reset( new Gabor() );
      *(gabor.at(i)) = *permanentGabor2;

      gabor.at(i)->setIndices(surfaces.at(i)->indices);

      gabor.at(i)->compute();
    }
  }

  validRelations.clear();
  #pragma omp parallel for
  for(unsigned int i = 0; i < surfaceRelations.size(); ++i)
  {   
    int p0 = surfaceRelations.at(i).id_0;
    int p1 = surfaceRelations.at(i).id_1;

    if( (!(surfaces.at(p0)->selected)) || (!(surfaces.at(p1)->selected)) )
    {
      surfaceRelations.at(i).valid = false;
      continue;
    }

    if(p0 >= p1)
    {
      surfaceRelations.at(i).valid = false;
      continue;
    }
    
    if((surfaceRelations.at(i).groundTruth == -1) && (trainMode))
    {
      surfaceRelations.at(i).valid = false;
      continue;
    }
    
    if(surfaceRelations.at(i).type != STRUCTURAL_RELATIONS)
    {
      surfaceRelations.at(i).valid = false;
      continue;
    }
    
    if( (!(surfaces.at(p0)->isNew)) && (!(surfaces.at(p1)->isNew)) )
    {
//       std::cerr << "Attention! " << p0 << " and " << p1 << std::endl;
      if(surfaceRelations.at(i).valid)
      {
#pragma omp critical
{
	validRelations.push_back(surfaceRelations.at(i));
}
        continue;
      }
    }
    
    // current neighbours border
    std::vector<neighboringPair> currentNeigboringBoundary3D;// = ngbr3D.at(p0).at(p1);

    borderIdentification borderId;
    // p1 < p2 ALWAYS!!!
    borderId.p1 = (p0 < p1 ? p0 : p1);
    borderId.p2 = (p0 < p1 ? p1 : p0);

    std::map<borderIdentification,std::vector<neighboringPair> >::iterator it3D; //,borderCompare
    it3D = ngbr3D_map.find(borderId);
    if(it3D != ngbr3D_map.end())
      currentNeigboringBoundary3D = it3D->second;
  
    std::vector<neighboringPair> currentNeigboringBoundary2D;// = ngbr2D.at(p0).at(p1);

    std::map<borderIdentification,std::vector<neighboringPair> >::iterator it2D; //,borderCompare
    it2D = ngbr2D_map.find(borderId);
    if(it2D != ngbr2D_map.end())
      currentNeigboringBoundary2D = it2D->second;
      
      //@ep: if there are no border in 3D it means there is no real connection between pixels, right?
      if(currentNeigboringBoundary3D.size() > 0)
      {
        
        Relation r = surfaceRelations.at(i);
        r.rel_value.clear();
        
        if(usedRelations & R_COS)
        {
          double colorSimilarity = hist.at(p0)->compare(hist.at(p1));
          r.rel_value.push_back(colorSimilarity);           // r_co ... color similarity (histogram) of the patch
        }

        if(usedRelations & R_TR)
        {
          double textureRate = text.at(p0)->compare(text.at(p1));
          r.rel_value.push_back(textureRate);               // r_tr ... difference of texture rate
        }

        if(usedRelations & R_GS)
        {
          double gaborRate = gabor.at(p0)->compare(gabor.at(p1));
          r.rel_value.push_back(gaborRate);                 // r_ga ... Gabor similarity of patch texture
        }

        if(usedRelations & R_FS)
        {
          double fourierRate = fourier.at(p0)->compare(fourier.at(p1));
          r.rel_value.push_back(fourierRate);               // r_fo ... Fourier similarity of patch texture
        }
        
        if(usedRelations & R_RS)
        {
	  double relSize = std::min((double)surfaces.at(p0)->indices.size()/(double)surfaces.at(p1)->indices.size(), 
                                    (double)surfaces.at(p1)->indices.size()/(double)surfaces.at(p0)->indices.size());
          r.rel_value.push_back(relSize);                   // r_rs ... relative patch size difference
        }

        if(usedRelations & R_COS3)
        {
	  BoundaryRelationsMeanColor::Ptr meanColor( new BoundaryRelationsMeanColor() );
	  meanColor->setInputCloud(cloud_model);
          meanColor->setBoundary(currentNeigboringBoundary3D/*ngbr3D.at(p0).at(p1)*/);
	  surface::meanVal meanColorVal = meanColor->compute();
          r.rel_value.push_back(1.-meanColorVal.mean);      // r_co3 ... color similarity on 3D border
        }
	
        surface::meanVal meanCurvatureVal;
	if((usedRelations & R_CUM3) || (usedRelations & R_CUV3))
        {
          BoundaryRelationsMeanCurvature::Ptr meanCurvature( new BoundaryRelationsMeanCurvature() );
          meanCurvature->setInputCloud(cloud_model);
          meanCurvature->setBoundary(currentNeigboringBoundary3D/*ngbr3D.at(p0).at(p1)*/);
	  meanCurvature->setNormals(normals);
	  meanCurvatureVal = meanCurvature->compute();
        }

        if(usedRelations & R_CUM3)
        {
          r.rel_value.push_back(meanCurvatureVal.mean);     // r_cu3 ... mean curvature of 3D neighboring points
        }

        if((usedRelations & R_DM2) || (usedRelations & R_DV2))
        {
          BoundaryRelationsMeanDepth::Ptr meanDepth( new BoundaryRelationsMeanDepth() );
          meanDepth->setInputCloud(cloud_model);
          meanDepth->setBoundary(currentNeigboringBoundary2D/*ngbr2D.at(p0).at(p1)*/);
          surface::meanVal meanDepthVal = meanDepth->compute();

          if(usedRelations & R_DM2)
          {
            r.rel_value.push_back(meanDepthVal.mean);         // r_di2 ... depth mean value between border points (2D)
          }
          if(usedRelations & R_DV2)
          {
            r.rel_value.push_back(meanDepthVal.stddev);       // r_vd2 ... depth variance value
          }
        }

        if(usedRelations & R_CUV3)
        {
          //@ep: BUG this is done to be consistent with the old code
          meanCurvatureVal.stddev /= currentNeigboringBoundary2D.size();//ngbr2D.at(p0).at(p1).size();
	  r.rel_value.push_back(meanCurvatureVal.stddev);   // r_cu3 ... curvature variance of 3D neighboring points
        }
	
	if(usedRelations & R_3D2)
        {
	  double ratio3Dboundary2Dboundary = ((double)currentNeigboringBoundary3D.size())/((double)currentNeigboringBoundary2D.size());
          r.rel_value.push_back(ratio3Dboundary2Dboundary); // r_3d2 ... relation 3D neighbors / 2D neighbors
        }
 
        r.valid = true;
        surfaceRelations.at(i) = r;
#pragma omp critical
{
        validRelations.push_back(r);
}
      }
  }
  
//   std::vector<Relation> newSurfaceRelations;
//   for(int i = 0; i < surfaceRelations.size(); ++i)
//   {
//     if(surfaceRelations.at(i).valid)
//     {
//       newSurfaceRelations.push_back(surfaceRelations.at(i));
//     }
//   }
// 
//   surfaceRelations = newSurfaceRelations;
  
  // copy relations to view
  for(unsigned int i=0; i<validRelations.size(); i++)
  {
    if(validRelations.at(i).valid) 
    {
      printf("r_st_l: [%u][%u]: ", validRelations.at(i).id_0, validRelations.at(i).id_1);
      for(unsigned int ridx = 0; ridx < validRelations.at(i).rel_value.size(); ridx++)
        printf("%4.3f ", validRelations.at(i).rel_value[ridx]);
      printf("\n");
    }
  }
  
}

} // end surface models





