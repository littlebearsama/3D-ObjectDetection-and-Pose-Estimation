/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
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
 * @file AssemblyRelations.cpp
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Calculate patch relations for assembly level.
 */

#include "AssemblyRelations.h"
#include "v4r/SurfaceUtils/Utils.hh"

namespace surface
{

  
/************************************************************************************
 * Constructor/Destructor
 */

AssemblyRelations::AssemblyRelations()
:EPBase()
{
  z_max = 0.01;
//   bdry_vs3 = new boundary::VisionCore();

  have_surfaces = false;
  have_relations = false;
  have_neighbours2D = false;
  have_neighbours3D = false;
//   have_edges = false;
//   have_edgels = false;
//   have_patch_image = false;
  usedRelations = 0x07FF;//0xFFFF;
  
  ClassName = "AssemblyRelations";
}

AssemblyRelations::~AssemblyRelations()
{
//   delete bdry_vs3;
}

/**Set used relations **/
void AssemblyRelations::setUsedRelations(int _usedRelations)
{
  usedRelations = _usedRelations;
}

void AssemblyRelations::setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
  
  for(unsigned int i = 0; i < surfaces.size(); i++)
    if(surfaces.at(i)->contours.size() == 0)
      printf("[AssemblyRelations::setSurfaces] Error: No contour given for surface %u.\n", i);
}

void AssemblyRelations::setRelations(const std::vector<Relation> _surfaceRelations)
{
  surfaceRelations = _surfaceRelations;
  have_relations = true;
}

void AssemblyRelations::setNeighbours2D(const std::vector< std::vector< std::vector<neighboringPair> > > _ngbr2D)
{
  ngbr2D = _ngbr2D;
  have_neighbours2D = true;
}

void AssemblyRelations::setNeighbours3D(const std::vector< std::vector< std::vector<neighboringPair> > > _ngbr3D)
{
  ngbr3D = _ngbr3D;
  have_neighbours3D = true;
}

// void AssemblyRelations::setEdges(const std::vector<surface::Edge> _edges)
// {
//   edges = _edges;
//   have_edges = true;
// }
// 
// void AssemblyRelations::setEdgels(const std::vector<surface::Edgel> _edgels)
// {
//   edgels = _edgels;
//   have_edgels = true;
// }
// 
// void AssemblyRelations::setPatchImage(cv::Mat_<int> &_patchImage)
// {
//   _patchImage.copyTo(patchImage);
//   have_patch_image = true;
// }

// ================================= Private functions ================================= //

// calculate mean and variance of surface normals
void AssemblyRelations::precalculateNormalRelations()
{
  normalsMean.clear();
  normalsVar.clear();
  //@ep: TODO: make parallelization
//   #pragma omp parallel for
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    Eigen::Vector3d mean;
    mean[0]=0.;
    mean[1]=0.;
    mean[2]=0.;
    double var = 0;
    for(unsigned int j = 0; j < surfaces.at(i)->normals.size(); j++)
    {
      mean += surfaces.at(i)->normals.at(j);
    }
    mean /= surfaces.at(i)->normals.size();
    normalsMean.push_back(mean);

    // calculate variance
    for(unsigned int j = 0; j < surfaces.at(i)->normals.size(); j++)
    {
      //@ep: we assume that all normals are oriented in the same direction
      double x = surfaces.at(i)->normals.at(j).dot(mean) / (surfaces.at(i)->normals.at(j).norm() * mean.norm());
      if(x > 1.0)
      {
//           printf("[PatchRelations::preprocess] Warning: Value too high (%8.8f).\n", x);
        x = 1.0;
      }
      var += acos(x);
    }
    var /= surfaces.at(i)->normals.size();
    normalsVar.push_back(var);
    // printf("Normals sum & var of patch %u: %4.3f (size: %lu)\n", i, var, surfaces[i]->normals.size());
  }
}


bool AssemblyRelations::calculateNormalRelations(int i, int j, double &_nor_mean, double &_nor_var)
{
  _nor_mean = acos(normalsMean[i].dot(normalsMean[j]) / (normalsMean[i].norm() * normalsMean[j].norm()));
  _nor_var = fabs(normalsVar[i] - normalsVar[j]);
  return true;
}



// bool AssemblyRelations::calculateBoundaryRelations()
// {
// printf("AssemblyRelations::calculateBoundaryRelations. process \n");
//   bdry_vs3->setInputCloud(cloud);
//   bdry_vs3->setView(edges,edgels,patchImage,surfaces);
//   bdry_vs3->processBoundary(4000);
// printf("AssemblyRelations::calculateBoundaryRelations. done\n");
//   return true;
// }

// ================================= Public functions ================================= //

void AssemblyRelations::compute()
{
  if((!have_cloud) || (!have_surfaces) || (!have_normals) || (!have_neighbours2D) || (!have_neighbours3D) || (!have_relations)/* ||
     (!have_edges) || (!have_edgels) || (!have_patch_image)*/)
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud, surfaces, and neighbours.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  // for the texture
  cv::Mat_<cv::Vec3b> matImage;
  //EPUtils::PointCloudXYZRGB2RGB(cloud, matImage);
  EPUtils::pointCloud_2_rgb(matImage,cloud,cloud->width,cloud->height);
  double lowThreshold = 5.;
  double highThreshold = 140.;
  int kernel_size = 3;
  cv::Mat gray_image;
  cv::Mat edges_image;
  cv::cvtColor(matImage, gray_image, CV_BGR2GRAY );
  cv::blur(gray_image, edges_image, cv::Size(3,3));
  cv::Canny(edges_image, edges_image, lowThreshold, highThreshold, kernel_size);
  
  std::vector<std::vector<Relation> > relations;
  std::vector<ColorHistogram::Ptr> hist;
  std::vector<Texture::Ptr> text;
  std::vector<Fourier::Ptr> fourier;
  std::vector<Gabor::Ptr> gabor;

  ContourNormalsDistance cnd;
//   surface::Vs3ArcRelations vs3ArcRel;

  if(usedRelations & R_COS)
  {

    hist.resize(surfaces.size());
    #pragma omp parallel for
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      int nrHistBins = 4;
      double uvThreshold = 0.0f;

      hist.at(i).reset( new ColorHistogram(nrHistBins,uvThreshold) );

      hist.at(i)->setInputCloud(cloud);
      hist.at(i)->setIndices(surfaces.at(i)->indices);

      if(!(surfaces.at(i)->selected))
      {
        continue;
      }
      hist.at(i)->compute();
    }
  }

  if(usedRelations & R_TR)
  {
    text.resize(surfaces.size());
    #pragma omp parallel for
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      text.at(i).reset( new Texture() );

      text.at(i)->setInputEdges(edges_image);
      text.at(i)->setIndices(surfaces.at(i)->indices);

      if(!(surfaces.at(i)->selected))
      {
        continue;
      }

      text.at(i)->compute();
    }
  }

  if(usedRelations & R_FS)
  {
    fourier.resize(surfaces.size());
    #pragma omp parallel for
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      fourier.at(i).reset( new Fourier() );

      fourier.at(i)->setInputImage(gray_image);
      fourier.at(i)->setIndices(surfaces.at(i)->indices);

      if(!(surfaces.at(i)->selected))
      {
        continue;
      }

      fourier.at(i)->compute();
    }
  }

  if(usedRelations & R_GS)
  {
    Gabor::Ptr permanentGabor;
    permanentGabor.reset( new Gabor() );
    permanentGabor->setInputImage(gray_image);
    permanentGabor->computeGaborFilters();

    gabor.resize(surfaces.size());
    #pragma omp parallel for shared(permanentGabor)
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      gabor.at(i).reset( new Gabor() );
      *(gabor.at(i)) = *permanentGabor;

      gabor.at(i)->setIndices(surfaces.at(i)->indices);

      if(!(surfaces.at(i)->selected))
      {
        continue;
      }

      gabor.at(i)->compute();
    }
  }

  ContourNormalsDistance::Parameter cndParam;
  cndParam.pcntContourPoints = 0.2;
  cnd.setParameter(cndParam);
  cnd.setInputCloud(cloud);
  cnd.setNormals(normals);

  precalculateNormalRelations();

// printf("AssemblyRelations::computeRelations 3.3\n");

//   calculateBoundaryRelations();         // vs3-boundary relations

/// TODO TODO TODO TODO TODO TODO Arc relations
// #pragma omp section
// {
//   vs3ArcRel.setView(view);
//   vs3ArcRel.setInputImage(matImage);
//   vs3ArcRel.preprocess();               // TODO Muss mir contour image errechnen => Alle contours in gemeinsames Bild (wie patch image)
// }

  surface::Relation rel;
  rel.valid = false;
  relations.clear();
  relations.resize(surfaces.size());
  #pragma omp parallel for
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    for(unsigned int j = 0; j < surfaces.size(); j++)
    {
      relations.at(i).push_back(rel);
    }
  }

//   #pragma omp parallel for
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    if(!(surfaces.at(i)->selected))
      continue;

    for(unsigned int j=i+1; j < surfaces.size(); j++)
    {
      if(!(surfaces.at(j)->selected))
        continue;

      int p0 = i;
      int p1 = j;
      if(p0 > p1)
        continue;

      if(is3DNeighbor(p0,p1))
        continue;

      Relation r;
      r.groundTruth = -1;
      r.prediction = -1;
      r.type = ASSEMBLY_RELATIONS;                                     // assembly level svm type
      r.id_0 = p0;
      r.id_1 = p1;

      if(usedRelations & R_COS)
      {
        double colorSimilarity = hist.at(p0)->compare(hist.at(p1));
        r.rel_value.push_back(colorSimilarity);         // r_co ... color similarity (histogram) of the patch
      }

      if(usedRelations & R_TR)
      {
        double textureRate = text.at(p0)->compare(text.at(p1));
        r.rel_value.push_back(textureRate);             // r_tr ... difference of texture rate
      }

      if(usedRelations & R_GS)
      {
        double gaborRate = gabor.at(p0)->compare(gabor.at(p1));
        r.rel_value.push_back(gaborRate);           // r_ga ... Gabor similarity of patch texture
      }

      if(usedRelations & R_FS)
      {
        double fourierRelation = fourier.at(p0)->compare(fourier.at(p1));
        r.rel_value.push_back(fourierRelation);         // r_fo ... Fourier similarity of patch texture
      }

      if(usedRelations & R_RS)
      {
        double relSize = std::min((double)surfaces.at(p0)->indices.size()/(double)surfaces.at(p1)->indices.size(),
                                  (double)surfaces.at(p1)->indices.size()/(double)surfaces.at(p0)->indices.size());
        r.rel_value.push_back(relSize);                 // r_rs ... relative patch size difference
      }

      float cosDeltaAngle = 0.0;
      float distNormal = 0.0;
      float minDist = 0.0;
      float occlusion = 0.0;

      std::cerr << "before" << std::endl;
      if( (usedRelations & R_MD) || (usedRelations & R_ANG) || (usedRelations & R_DN) || (usedRelations & R_OC) )
      {
        if( !(cnd.compute(surfaces.at(p0),surfaces.at(p1),cosDeltaAngle,distNormal,minDist,occlusion)) )
          continue;
      }
      std::cerr << "after" << std::endl;

      /// TODO HACK: make it faster and connect only patches within 15cm!
      if(minDist > 0.15)
        continue;

      double nor_mean, nor_var;
      if( (usedRelations & R_NM) || (usedRelations & R_NV) )
      {
        if( !(calculateNormalRelations(p0,p1,nor_mean,nor_var)) )
          continue;
      }

      if(usedRelations & R_MD)
      {
        r.rel_value.push_back(minDist);                 // r_md ... proximity: minimum distance between two surfaces
      }

      if(usedRelations & R_NM)
      {
        r.rel_value.push_back(nor_mean);                // r_mn ... Mean value of the normals
      }

      if(usedRelations & R_NV)
      {
        r.rel_value.push_back(nor_var);                 // r_nv ... Variance of the normals
      }

      if(usedRelations & R_ANG)
      {
        r.rel_value.push_back(cosDeltaAngle);           // r_ac ... Angle between normals of contour-neighbors
      }

      if(usedRelations & R_DN)
      {
        r.rel_value.push_back(distNormal);              // r_dn ... distance in normal direction of surface-neighbors
      }

      if(usedRelations & R_OC)
      {
        r.rel_value.push_back(occlusion);               // r_oc ... occlusion value between minimum distance
      }

//       /// TODO Relation 2D contour boundary / contour size
// //       for(unsigned k=0; k<view->surfaces[p0]->neighbors2D.size(); k++)
// //         if(view->surfaces[p0]->neighbors2D[k] == p1)
// //           printf("AssemblyRelations: %u-%u We have here %u 2D-neighboring points\n", p0, p1, view->surfaces[p0]->neighbors2DNrPixel[k]);                /// TODO Wieso hier 2 mal?
// 
// //       if( (usedRelations & R_BR0) || (usedRelations & R_BR1) || (usedRelations & R_BR2) || (usedRelations & R_BR3) || (usedRelations & R_BR4) )
// //       {
// //         std::vector<double> boundaryRelations;
// //         if(is2DNeigbor(p0,p1))
// //         {                                                                           /// TODO Wieso keine 2D Nachbarn? => kein Häferl usw. ....
// //           boundaryRelations.push_back(1.0);       // set to 1 if 2D neighbor
// //           boundaryRelations.push_back(-1.0);
// //           boundaryRelations.push_back(0.0);
// //           boundaryRelations.push_back(1.0);
// //           boundaryRelations.push_back(0.0);
// //         }
// //         else {
// //           boundaryRelations.push_back(2.0);       // set to 2 if not 2D nor 3D neighbor
// //           boundaryRelations.push_back(-1.0);
// //           boundaryRelations.push_back(0.0);
// //           boundaryRelations.push_back(1.0);
// //           boundaryRelations.push_back(0.0);
// //           bdry_vs3->getResult(p0,p1,boundaryRelations);                                                 /// TODO Nochmal aus 3D nachbarn herausnehmen
// //         }
// //
// //         if(usedRelations & R_BR0)
// //         {
// //           r.rel_value.push_back(boundaryRelations[0]);    // r_cs ... Colliniarity: Minimum distance*angle measurement
// //         }
// //         if(usedRelations & R_BR1)
// //         {
// //           r.rel_value.push_back(boundaryRelations[1]);    // r_oc ... Collinearity: Mean depth (occlusion) value
// //         }
// //         if(usedRelations & R_BR2)
// //         {
// //           r.rel_value.push_back(boundaryRelations[2]);    // r_ls ... Closure: line support
// //         }
// //         if(usedRelations & R_BR3)
// //         {
// //           r.rel_value.push_back(boundaryRelations[3]);    // r_gl ... Closure gap-line max
// //         }
// //         if(usedRelations & R_BR4)
// //         {
// //           r.rel_value.push_back(boundaryRelations[4]);    // r_as ... Closure: area support
// //         }
// //
// //       }
// 
      r.valid = true;
      relations.at(p0).at(p1) = r;

    }
  }

  // copy relations to view
  for(unsigned int i=0; i<surfaces.size(); i++)
  {
    for(unsigned int j=i+1; j<surfaces.size(); j++)
    {
      if(relations.at(i).at(j).valid)
      {
        surfaceRelations.push_back(relations.at(i).at(j));
        printf("r_as_l: [%u][%u]: ", relations.at(i).at(j).id_0, relations.at(i).at(j).id_1);
        for(unsigned int ridx = 0; ridx < relations.at(i).at(j).rel_value.size(); ridx++)
          printf("%4.3f ", relations.at(i).at(j).rel_value[ridx]);
        printf("\n");
      }
    }
  }
      
//   bdry_vs3->ShowBoundaries();                                   /// TODO Show boundarie-information from vs3 for debugging

// printf("computeRelations 11\n");
}


// void AssemblyRelations::getView(View &_view)
// {
//   _view->relations.clear();
//   for (unsigned i = 0; i < view->relations.size(); i++)
//     _view.relations.push_back(view->relations[i]);
// }

} // end surface models





