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
 * @file SurfaceModeling.cc
 * @author Richtsfeld
 * @date October 2012
 * @version 0.1
 * @brief Models patches using NURBS.
 */


#include <cstdio>
#include "SurfaceModeling.hh"

namespace surface {

#define COSTS_NURBS_PARAMS      3.0     // Parameter costs: for each control point of a b_spline (3*9 = 27)
#define COSTS_PLANE_PARAMS     12.0     // Parameter costs for a plane (3*4pts = 12)

  
using namespace std;

/**
 * ComputePointProbs
 */
void computePointProbs(std::vector<double> &errs, std::vector<double> &probs, double sigmaError)
{
  double invSqrSigmaError = 1.0 / (sigmaError*sigmaError);
  probs.resize(errs.size());
  for (unsigned int i = 0; i < errs.size(); i++)
  {
    probs.at(i) = std::exp(-( (errs.at(i)*errs.at(i) ) * invSqrSigmaError));
  }
}

double computeSavingsNormalized(int numParams, std::vector<double> &probs, double norm,
                                double kappa1, double kappa2)
{
  norm = 1. / norm;
  double savings = norm * (double) probs.size() - kappa1 * numParams;
  double err = 0.;
  
  for (unsigned int i = 0; i < probs.size(); i++)
  {
    err += (1. - probs.at(i));
  }
  
  savings -= norm * kappa2 * err;
  
  return savings;
  //@ep: so, what is the correct form???
  //return (savings > 0 ? savings : 0);
}

double computeSavings(int numParams, std::vector<double> &probs, double kappa1, double kappa2)
{
  double savings = probs.size() - kappa1 * (double) numParams;
  double err = 0.;
  
  for (unsigned int i = 0; i < probs.size(); i++)
    err += (1. - probs[i]);
  
  savings -= kappa2 * err;
  
  return (savings > 0 ? savings : 0);
}

double computePlaneSavingsNormalized(int numParams, std::vector<double> &probs, double norm, double kappa1, double kappa2)
{
  norm = 1. / norm;
  double savings = /*norm * (double) probs.size() 1.0 */ - kappa1 * numParams;
  double err = 0.;
  
  for (unsigned int i = 0; i < probs.size(); i++)
    err += (1. - probs.at(i));
  
  double probs_size = 1./(double) probs.size();
  savings -= probs_size * kappa2 * err;
  
  //   return (savings > 0 ? savings : 0);
  return savings;
}

/********************** SurfaceModeling ************************
 * Constructor/Destructor
 */
SurfaceModeling::SurfaceModeling(Parameter p) :
  EPBase(), tryMergePlanes(false), tryMergeNurbs(true), msCheck(true), haveIntr(false), haveExtr(false), param(p)
{
  have_surfaces = false;
  filter_by_size = false;
  minSurfaceSize = 0;
  ClassName = "SurfaceModeling";
}

SurfaceModeling::~SurfaceModeling()
{
}

/************************** PRIVATE ************************/

void SurfaceModeling::computeLeastSquarePlane(SurfaceModel::Ptr plane)
{
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> lsPlane(cloud);
  Eigen::VectorXf coeffs(4);
  Eigen::Vector3d n0(0., 0., 1.);

  if( (plane->type == pcl::SACMODEL_PLANE) && (plane->indices.size() > 4) ) 
  {
    lsPlane.optimizeModelCoefficients(plane->indices,coeffs,coeffs);
    //dot product
    //if( (coeffs[0]*n0[0] + coeffs[1]*n0[1] + coeffs[2]*n0[2]) > 0 )
    if( (coeffs[0]*n0[0] + coeffs[1]*n0[1] + coeffs[2]*n0[2]) > 0 )
    {
      coeffs *= -1.;
    }
    
    // ep: why projection of the new coefficiend on the old coefficient should be more than 0.6???
    // jp: that's a plausibility check. 
    // for instable degenerated planes (pixel chains) the old plane (computed from normals) is inaccurate but correct
    // if the new plane parameter are not similag to the old ones let's keep the old parameter
    // dot product
    if( (coeffs[0]*plane->coeffs[0] + coeffs[1]*plane->coeffs[1] + coeffs[2]*plane->coeffs[2]) > 0.6 )
    {
      plane->coeffs.resize(4);
      plane->coeffs[0] = coeffs[0];
      plane->coeffs[1] = coeffs[1];
      plane->coeffs[2] = coeffs[2];
      plane->coeffs[3] = coeffs[3];
    }
    else
    {
      printf("[SurfaceModeling::computeLeastSquarePlane] Warning: Problematic plane found.\n");
    }

    // check orientation of normals and calculate probabilities
    for (unsigned int i = 0; i < plane->indices.size(); i++)
    {

      int idx = plane->indices.at(i);

      Eigen::Vector3f curPoint = cloud->points.at(idx).getVector3fMap();
      Eigen::Vector3d curNormal;
      curNormal[0] = normals->points.at(idx).normal_x;
      curNormal[1] = normals->points.at(idx).normal_y;
      curNormal[2] = normals->points.at(idx).normal_z;
      plane->normals.at(i) = curNormal;
      
      if( (curNormal[0]*curPoint[0] + curNormal[1]*curPoint[1] + curNormal[2]*curPoint[2]) > 0 )
      {
        plane->normals.at(i) *= -1.;
      }
    }
    
  }
}

/**
 * FitPlane
 */
void SurfaceModeling::fitPlane(SurfaceModel::Ptr plane)
{
  computeLeastSquarePlane(plane);
  computePointError(plane);
  computePointProbs(plane->error,plane->probs,param.sigmaError);
}  
  
/**
 * FitNurbs
 */
void SurfaceModeling::fitNurbs(SurfaceModel::Ptr surface)
{
  pcl::PointIndices::Ptr points(new pcl::PointIndices());
  points->indices = surface->indices;

  cv::Ptr<pcl::on_nurbs::SequentialFitter> nurbsFitter;
  nurbsFitter = new pcl::on_nurbs::SequentialFitter(param.nurbsParams);
  if (haveIntr && haveExtr) 
  {
    nurbsFitter->setProjectionMatrix(camIntr,camExtr);
  } else 
  {
    printf("[SurfaceModeling::fitNurbs] Warning, projection matrix is not set!\n");
  }

  nurbsFitter->setInputCloud(cloud);
  nurbsFitter->setInterior(points);
  nurbsFitter->compute();
  nurbsFitter->getInteriorError(surface->error);
  surface->nurbs = nurbsFitter->getNurbs();
  nurbsFitter->getInteriorNormals(surface->normals);
  nurbsFitter->getInteriorParams(surface->nurbs_params);

  // check orientation of normals and calculate probabilities
  for (unsigned int i = 0; i < surface->normals.size(); i++)
  {
    Eigen::Vector3d n = surface->normals.at(i);
    Eigen::Vector3f curPoint = cloud->points.at(surface->indices.at(surface->indices.size()/2)).getVector3fMap();
    if( (n[0]*curPoint[0] + n[1]*curPoint[1] + n[2]*curPoint[2]) > 0 )
    {
      surface->normals.at(i) *= -1.;
    }
  }

  computePointProbs(surface->error,surface->probs,param.sigmaError);
}

/**
 * Check if the given plane SurfaceModel can be replaced by a better NURBS.
 * @param surf the surface to check, will be replaced with a fitted NURBS if that is better (in/out)
 * @param newIdx in case the surface is replaced (i.e. a new NURBS surface created), it will get this index
 * @return true, if plane was replaced with NURBS, otherwise false
 */
bool SurfaceModeling::replacePlaneWithBetterNurbs(SurfaceModel::Ptr &surf)
{
  bool replaced = false;

  // HACK: if plane is not too big (otherwise it is probably a wall or a table)
  if( ((int)surf->indices.size()) < param.planePointsFixation ) 
  {
    SurfaceModel::Ptr model(new SurfaceModel());
      
    model->indices = surf->indices;
    model->type = surf->type;
    model->idx = surf->idx;
    model->selected = surf->selected;
    model->valid = surf->valid;
    model->neighbors3D = surf->neighbors3D;
    model->neighbors2D = surf->neighbors2D;

    if(model->indices.size() <= 3)
      return replaced;

    surf->savings = computeSavingsNormalized(COSTS_PLANE_PARAMS,surf->probs,surf->indices.size(), param.kappa1, param.kappa2);
    fitNurbs(model);
    //@ep: why is it surf->indices.size() and not model->indices.size()
    model->savings = computeSavingsNormalized(model->nurbs.m_cv_count[0]*model->nurbs.m_cv_count[1]*COSTS_NURBS_PARAMS,model->probs,surf->indices.size(), param.kappa1, param.kappa2);

    cout << "Savings plane/NURBS id=" << surf->idx  << ": " << surf->savings << "/" << model->savings;//[SurfaceModeling::modelSelection] 
    cout << " -> " << (model->savings > surf->savings ? "NURBS" : "PLANE") << endl;
      
    // => create NURBS
    if ( model->savings > surf->savings ) 
    { 
      model->type = MODEL_NURBS;
      surf = model;
      replaced = true;
    }
      
  }

  return replaced;
}

      

//       if(model->indices.size() > 3)
//         FitNurbs(*model);
// 
//       view->surfaces[i]->savings = ComputeSavingsNormalized(COSTS_PLANE_PARAMS, view->surfaces[i]->probs, view->surfaces[i]->indices.size());
//       model->savings = ComputeSavingsNormalized(model->nurbs.m_cv_count[0] * model->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS, model->probs, view->surfaces[i]->indices.size());
// 
// #ifdef DEBUG
//       cout << "[SurfaceModeling::ModelSelectionParallel] Savings plane/NURBS id=" << i << ": " << view->surfaces[i]->savings << "/" << model->savings;
// #endif
//       
//       if (model->savings > view->surfaces[i]->savings) { // => create NURBS
//         model->type = MODEL_NURBS;
//         model->savings = view->surfaces[i]->savings;
//         view->surfaces[i] = model;
//       }
// #ifdef DEBUG
//       cout << " -> " << (model->type == MODEL_NURBS ? "NURBS" : "PLANE") << endl;
// #endif

/**
 * See if merging two surfaces gives a better merged surface in terms of savings.
 * @return smart pointer to merged surface, or null pointer
 */
bool SurfaceModeling::tryMergeSurfaces(SurfaceModel::Ptr surf1, SurfaceModel::Ptr surf2, SurfaceModel::Ptr &mergedSurf)
{
  if( (surf1->indices.size() + surf2->indices.size()) > 3 )
  {
    mergedSurf.reset(new SurfaceModel());
    *mergedSurf = *surf1;
    surf2->addTo(*mergedSurf);
    
    mergedSurf->type = MODEL_NURBS;
    fitNurbs(mergedSurf);
    mergedSurf->savings = computeSavingsNormalized(mergedSurf->nurbs.m_cv_count[0] * mergedSurf->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS,
                                                   mergedSurf->probs, mergedSurf->indices.size(), param.kappa1, param.kappa2);
    //@ep: why mergedSurf->indices.size() as the last argument and not surf1->indices.size()???
    //jp: that's the common normalization factor for individual planes and merged nurbs
    surf1->savings = computeSavingsNormalized(
            (surf1->type == MODEL_NURBS ? surf1->nurbs.m_cv_count[0] * surf1->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS),
            surf1->probs, mergedSurf->indices.size(), param.kappa1, param.kappa2);
    
    surf2->savings = computeSavingsNormalized(
            (surf2->type == MODEL_NURBS ? surf2->nurbs.m_cv_count[0] * surf2->nurbs.m_cv_count[1] * COSTS_NURBS_PARAMS : COSTS_PLANE_PARAMS),
            surf2->probs, mergedSurf->indices.size(), param.kappa1, param.kappa2);

    if( mergedSurf->savings > (surf1->savings + surf2->savings) )
    {
      return true;
    }
    else
    {
      return false;
    }
  }
  else
  {
    return false;
  }
}

/**
 * See if merging two planes gives a better merged plane in terms of savings.
 * @return smart pointer to merged surface, or null pointer
 */
bool SurfaceModeling::tryMergeSurfacesWithPlanes(SurfaceModel::Ptr surf1, SurfaceModel::Ptr surf2, SurfaceModel::Ptr &mergedSurf)
{
  mergedSurf.reset(new SurfaceModel());
  *mergedSurf = *surf1;
  surf2->addTo(*mergedSurf);

  if(mergedSurf->indices.size() <= 3)
    return false;
  
  mergedSurf->type = pcl::SACMODEL_PLANE;
  fitPlane(mergedSurf);
  mergedSurf->savings = computePlaneSavingsNormalized(0., mergedSurf->probs, mergedSurf->indices.size(), 0.003, 0.9);

  //@ep: why mergedSurf->indices.size() as the last argument and not surf1->indices.size()???
  //jp: that's the common normalization factor for individual planes and merged nurbs
  surf1->savings = computePlaneSavingsNormalized(0., surf1->probs, mergedSurf->indices.size(), 0.003, 0.9)/2.;
    
  surf2->savings = computePlaneSavingsNormalized(0., surf2->probs, mergedSurf->indices.size(), 0.003, 0.9)/2.;
    
  if( mergedSurf->savings > (surf1->savings + surf2->savings) )
  {
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * ModelSelection with omp parallel
 */
void SurfaceModeling::modelSelection()
{
  // pairs of patches to be merged
  std::vector<MergedPair> mergePairs;
  
  if(tryMergeNurbs)
  {
    #pragma omp parallel for
    for (unsigned int i = 0; i < surfaces.size(); i++)
    {
      if((!(surfaces.at(i)->isNew)) || (!(surfaces.at(i)->selected)) || (!(surfaces.at(i)->valid)) || (surfaces.at(i)->type != pcl::SACMODEL_PLANE))//@ep: TODO back??? (surfaces.at(i)->type == MODEL_NURBS))
        continue;
      
      replacePlaneWithBetterNurbs(surfaces.at(i));
    }

//     exit(0);
    
    mergeWithNurbs(mergePairs);
    
//     exit(0);
  }
  else if (tryMergePlanes)
  {
    mergeWithPlanes(mergePairs);
  }

  addedTo.resize(surfaces.size(),-1);
  
  // merge the surfaces from the best to the weakest connection
  std::sort(mergePairs.begin(),mergePairs.end(),cmpSavings);
  
  for(unsigned int i = 0; i < mergePairs.size(); i++)
  {
    // if we are neigbors of ourselves skip
    if(mergePairs.at(i).id1 == mergePairs.at(i).id2)
      continue;
    
    printf("Try merging %u and %u\n", mergePairs.at(i).id1, mergePairs.at(i).id2);//[SurfaceModeling::modelSelection] 
    
    SurfaceModel::Ptr mergedModel;

    bool toBeMerged = false;

    if(tryMergeNurbs)
    {
      toBeMerged = tryMergeSurfaces(surfaces.at(mergePairs.at(i).id1),surfaces.at(mergePairs.at(i).id2),mergedModel);
    }
    else if(tryMergePlanes)
    {
      toBeMerged = tryMergeSurfacesWithPlanes(surfaces.at(mergePairs.at(i).id1),surfaces.at(mergePairs.at(i).id2),mergedModel);
    }
    
    if(toBeMerged)
    {
      
      printf("MERGED: %u-%u (%1.5f > %1.5f)\n", mergePairs.at(i).id1, mergePairs.at(i).id2,
             mergedModel->savings, surfaces.at(mergePairs.at(i).id1)->savings + surfaces.at(mergePairs.at(i).id2)->savings);//[SurfaceModeling::modelSelection]  => 

//       std::cerr << "surfaces.at(mergePairs.at(i).id1).indices = " << surfaces.at(mergePairs.at(i).id1)->indices.size() << std::endl;
//       std::cerr << "surfaces.at(mergePairs.at(i).id2).indices = " << surfaces.at(mergePairs.at(i).id2)->indices.size() << std::endl;
//       std::cerr << "mergedModel.indices = " << mergedModel->indices.size() << std::endl;
      
      surfaces.at(mergePairs.at(i).id1) = mergedModel;
      surfaces.at(mergePairs.at(i).id2)->selected = false;
      surfaces.at(mergePairs.at(i).id2)->valid = false;
      surfaces.at(mergePairs.at(i).id2)->isNew = false;
      surfaces.at(mergePairs.at(i).id1)->isNew = true;

      addedTo.at(mergePairs.at(i).id2) = mergePairs.at(i).id1;

      modifyNeighbours(mergePairs.at(i).id2,mergePairs.at(i).id1);
      modifyBoundary(mergePairs.at(i).id2,mergePairs.at(i).id1);

      for(unsigned int j = i+1; j < mergePairs.size(); j++)
      {

        if(mergePairs.at(j).id1 == mergePairs.at(i).id2)
          mergePairs.at(j).id1 = mergePairs.at(i).id1;
      
        if(mergePairs.at(j).id2 == mergePairs.at(i).id2)
          mergePairs.at(j).id2 = mergePairs.at(i).id1;
      }
    }
    
  }
//   exit(0);
  
}

void SurfaceModeling::mergeWithPlanes(std::vector<MergedPair> &mergePairs)
{
  // Merge first planes
  #pragma omp parallel for shared(mergePairs)
  for (unsigned int i = 0; i < surfaces.size(); i++)
  {
    // if we are interested in the plane and it is still not used
    if( (surfaces.at(i)->isNew) &&  (surfaces.at(i)->selected) && (surfaces.at(i)->valid) )
    {
      if(surfaces.at(i)->type != pcl::SACMODEL_PLANE)
        continue;

      // go over all 3D neighbours
      for(std::set<unsigned>::iterator itr = surfaces.at(i)->neighbors3D.begin(); itr != surfaces.at(i)->neighbors3D.end(); itr++)
      {
        if( (!(surfaces.at(*itr)->selected)) || (!(surfaces.at(*itr)->valid)) )
          continue;
        
        // select the neigbour
        if( surfaces.at(*itr)->type != pcl::SACMODEL_PLANE )
	  continue;

        // pair (i,j) where i < j ALWAYS!
        if( (*itr) > i )
	{
          SurfaceModel::Ptr mergedModel;
          if( tryMergeSurfacesWithPlanes(surfaces.at(i),surfaces.at(*itr),mergedModel) )
          {
            MergedPair m;
            m.id1 = i;
            m.id2 = *itr;
            m.savings = mergedModel->savings;
            
            #pragma omp critical
            {
              mergePairs.push_back(m);
              cout << "Merge candidates: " << m.id1 << "-" << m.id2 << endl;
            }
          }
        }
      }
    }
  }
}

void SurfaceModeling::mergeWithNurbs(std::vector<MergedPair> &mergePairs)
{
  // go in a parallel fashion over all neighboring pairs and put them into the stack to merge
  #pragma omp parallel for shared(mergePairs)
  for(unsigned int i = 0; i < surfaces.size(); i++) 
  {
    // if surface is valid and selected
    if( (surfaces.at(i)->isNew) && (surfaces.at(i)->selected) && (surfaces.at(i)->valid) )
    {
      // if patch is not too big
      if(surfaces.at(i)->indices.size() < (unsigned)param.planePointsFixation) 
      {
        // go over all neighbors and merge if necessary
	for(std::set<unsigned>::iterator itr = surfaces.at(i)->neighbors3D.begin(); itr != surfaces.at(i)->neighbors3D.end(); itr++) 
	{
          if( (!(surfaces.at(*itr)->selected)) || (!(surfaces.at(*itr)->valid)) )
	    continue;
	    
	  // pair (i,j) where i < j ALWAYS!
	  if( (*itr) > i ) 
	  {
            SurfaceModel::Ptr mergedModel;
            if( tryMergeSurfaces(surfaces.at(i),surfaces.at(*itr),mergedModel) )
	    {
              MergedPair m;
              m.id1 = i;
              m.id2 = *itr;
              m.savings = mergedModel->savings;

              #pragma omp critical
              {
                mergePairs.push_back(m);
		cout << "Merge candidates: " << m.id1 << "-" << m.id2 << endl;
              }
            }
          }
        }
      }
    }
  }
}

void SurfaceModeling::modifyNeighbours(int oldIdx, int newIdx)
{

  //add all neigbours from oldIdx to newIdx
  //3D
  std::set<unsigned>::iterator itr_toremove3D = surfaces.at(newIdx)->neighbors3D.find(oldIdx);
  surfaces.at(newIdx)->neighbors3D.erase(itr_toremove3D);
  for(std::set<unsigned>::iterator itr = surfaces.at(oldIdx)->neighbors3D.begin(); itr != surfaces.at(oldIdx)->neighbors3D.end(); itr++)
  {
    if((*itr) != ((unsigned int)newIdx))
    {
      surfaces.at(newIdx)->neighbors3D.insert(*itr);
      std::set<unsigned>::iterator itr_tochange3D = surfaces.at(*itr)->neighbors3D.find(oldIdx);
      surfaces.at(*itr)->neighbors3D.erase(itr_tochange3D);
      surfaces.at(*itr)->neighbors3D.insert(newIdx);
    }
  }
  //2D
  std::set<unsigned>::iterator itr_toremove2D = surfaces.at(newIdx)->neighbors2D.find(oldIdx);
  surfaces.at(newIdx)->neighbors2D.erase(itr_toremove2D);
  for(std::set<unsigned>::iterator itr = surfaces.at(oldIdx)->neighbors2D.begin(); itr != surfaces.at(oldIdx)->neighbors2D.end(); itr++)
  {
    if((*itr) != ((unsigned int)newIdx))
    {
      surfaces.at(newIdx)->neighbors2D.insert(*itr);
      std::set<unsigned>::iterator itr_tochange2D = surfaces.at(*itr)->neighbors2D.find(oldIdx);
      surfaces.at(*itr)->neighbors2D.erase(itr_tochange2D);
      surfaces.at(*itr)->neighbors2D.insert(newIdx);
    }
  }

//   //then go over all neighbours of oldIdx and change oldIdx to newIdx
//   for(int i = 0; i < surfaces.size(); ++i)
//   {
//     if(neigbouring_matrix2D.at<bool>(i,oldIdx) || neigbouring_matrix2D.at<bool>(oldIdx,i))
//     {
//       neigbouring_matrix2D.at<bool>(i,oldIdx) = false;
//       neigbouring_matrix2D.at<bool>(oldIdx,i) = false;
//       
//       if(i != newIdx)
//       {
//         neigbouring_matrix2D.at<bool>(i,newIdx) = true;
//         neigbouring_matrix2D.at<bool>(newIdx,i) = true;
//       }
//     }
//     
//     if(neigbouring_matrix3D.at<bool>(i,oldIdx) || neigbouring_matrix3D.at<bool>(oldIdx,i))
//     {
//       neigbouring_matrix3D.at<bool>(i,oldIdx) = false;
//       neigbouring_matrix3D.at<bool>(oldIdx,i) = false;
//       
//       if(i != newIdx)
//       {
//         neigbouring_matrix3D.at<bool>(i,newIdx) = true;
//         neigbouring_matrix3D.at<bool>(newIdx,i) = true;
//       }
//     }
//   }
}

void SurfaceModeling::modifyBoundary(unsigned int oldIdx, unsigned int newIdx)
{
  //1. Erase boundary between merged surfaces
  // p1 < p2 ALWAYS!!!
  borderIdentification borderId;
  borderId.p1 = (oldIdx < newIdx ? oldIdx : newIdx);
  borderId.p2 = (oldIdx < newIdx ? newIdx : oldIdx);
  
  //2D
  std::map<borderIdentification,std::vector<neighboringPair> >::iterator it2D = ngbr2D_map.find(borderId);
  if(it2D != ngbr2D_map.end())
    ngbr2D_map.erase(it2D);                   // erasing by iterator
  //3D
  std::map<borderIdentification,std::vector<neighboringPair> >::iterator it3D = ngbr3D_map.find(borderId);
  if(it3D != ngbr3D_map.end())
    ngbr3D_map.erase(it3D);                   // erasing by iterator
  
  //2. Go over all surfaces and check if there is a boundary between the surface and deleted segment
  // if yes, then add this boundary to the new surface
  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    if( (i == oldIdx) || (i == newIdx) )
      continue;

    borderId.p1 = (oldIdx < i ? oldIdx : i);
    borderId.p2 = (oldIdx < i ? i : oldIdx);
    //2D
    std::map<borderIdentification,std::vector<neighboringPair> >::iterator it2D_current = ngbr2D_map.find(borderId);
    //if we have found the boundary between segments i and oldIdx
    if(it2D_current != ngbr2D_map.end())
    {
      // try to find the boundary between i and newIdx
      borderIdentification borderId_temp;
      borderId_temp.p1 = (newIdx < i ? newIdx : i);
      borderId_temp.p2 = (newIdx < i ? i : newIdx);
      std::map<borderIdentification,std::vector<neighboringPair> >::iterator it2D_temp = ngbr2D_map.find(borderId_temp);
      //if there is a boundary
      if(it2D_temp != ngbr2D_map.end())
      {
        (it2D_temp->second).insert((it2D_temp->second).end(),(it2D_current->second).begin(),(it2D_current->second).end());
      }
      else
      {
        std::pair<borderIdentification,std::vector<neighboringPair> > new_border;
        new_border.first = borderId_temp;
        new_border.second = it2D_current->second;
        ngbr2D_map.insert(new_border);
        ngbr2D_map.erase(it2D_current); 
      }
    }
    //3D
    std::map<borderIdentification,std::vector<neighboringPair> >::iterator it3D_current = ngbr3D_map.find(borderId);
    //if we have found the boundary between segments i and oldIdx
    if(it3D_current != ngbr3D_map.end())
    {
      // try to find the boundary between i and newIdx
      borderIdentification borderId_temp;
      borderId_temp.p1 = (newIdx < i ? newIdx : i);
      borderId_temp.p2 = (newIdx < i ? i : newIdx);
      std::map<borderIdentification,std::vector<neighboringPair> >::iterator it3D_temp = ngbr3D_map.find(borderId_temp);
      //if there is a boundary
      if(it3D_temp != ngbr3D_map.end())
      {
        (it3D_temp->second).insert((it3D_temp->second).end(),(it3D_current->second).begin(),(it3D_current->second).end());
      }
      else
      {
        std::pair<borderIdentification,std::vector<neighboringPair> > new_border;
        new_border.first = borderId_temp;
        new_border.second = it3D_current->second;
        ngbr3D_map.insert(new_border);
        ngbr3D_map.erase(it3D_current);
      }
    }
  }
}

void SurfaceModeling::createNeighbours()
{
  neigbouring_matrix2D = cv::Mat_<bool>(surfaces.size(),surfaces.size());
  neigbouring_matrix2D.setTo(false);
  
  neigbouring_matrix3D = cv::Mat_<bool>(surfaces.size(),surfaces.size());
  neigbouring_matrix3D.setTo(false);
  
  for(unsigned int i = 0; i < surfaces.size(); i++) 
  {
    // go over all neighbors
    for(std::set<unsigned>::iterator itr = surfaces.at(i)->neighbors3D.begin(); itr != surfaces.at(i)->neighbors3D.end(); itr++) 
    {
      neigbouring_matrix3D.at<bool>(i,*itr) = true;
      neigbouring_matrix3D.at<bool>(*itr,i) = true;
    }
    // go over all neighbors
    for(std::set<unsigned>::iterator itr = surfaces.at(i)->neighbors2D.begin(); itr != surfaces.at(i)->neighbors2D.end(); itr++) 
    {
      neigbouring_matrix2D.at<bool>(i,*itr) = true;
      neigbouring_matrix2D.at<bool>(*itr,i) = true;
    }
  }
  
}

/**
 * ComputePointError
 */
void SurfaceModeling::computePointError(SurfaceModel::Ptr surf)
{
  surf->error.clear();
  surf->error.resize(surf->indices.size());

  if(surf->type == pcl::SACMODEL_PLANE) 
  {
    float a=surf->coeffs[0];
    float b=surf->coeffs[1];
    float c=surf->coeffs[2];
    float d=surf->coeffs[3];

    for (unsigned int i = 0; i < surf->indices.size(); i++)
    {
      // error is the distance to the plane
      Eigen::Vector3f curPoint = cloud->points.at(surf->indices.at(i)).getVector3fMap();
      surf->error.at(i) =  fabs (a*curPoint[0] + b*curPoint[1] + c*curPoint[2] + d) / std::sqrt (a*a + b*b + c*c);
    }
  }
  else 
  {
    // no error, if we do not have the model
    for(unsigned int i = 0; i < surf->indices.size(); i++)
    {
      surf->error.at(i) = 0.0f;
    }
  }
}

void SurfaceModeling::initSurface(SurfaceModel::Ptr surface)
{
  computePointError(surface);
  computePointProbs(surface->error, surface->probs, param.sigmaError);
}

/**
 * init
 */
void SurfaceModeling::init()
{
  for (unsigned int i = 0; i < surfaces.size(); i++)
  {
    if(surfaces.at(i)->initialized)
      continue;

    if((surfaces.at(i)->selected) && (surfaces.at(i)->valid))
    {
      initSurface(surfaces.at(i));
      surfaces.at(i)->initialized = true;
    }
  }
}

/**
 * computeNeighbors
 */
// void SurfaceModeling::computeNeighbors()
// {
//   create a patch image
//   cv::Mat_<int> patches = cv::Mat_<int>(height,width);
//   patches.setTo(-1);
//   
//   int nr_patches = surfaces.size();
//   
//   fill in patch image and clear neighbors lists
//   for(int i = 0; i < nr_patches; i++) 
//   {
//     if(!(surfaces.at(i)->selected))
//       continue;
//     
//     surfaces.at(i)->neighbors2D.clear();
//     surfaces.at(i)->neighbors3D.clear();
//     surfaces.at(i)->neighbors2DNrPixel.clear();
//    
//     for(int j = 0; j < surfaces.at(i)->indices.size(); j++) 
//     {
//       int r = Y(surfaces.at(i)->indices.at(j));
//       int c = X(surfaces.at(i)->indices.at(j));
//       patches.at<int>(r,c) = i;
//     }
//   }
//   
//   @ep: Why in the original version we've been using only 3 directions out of 4 (no use of +1,-1 shift)
//   cv::Mat neighbors2D, neighbors3D;
//   EPUtils::get2DNeighbors(patches,neighbors2D,nr_patches);
//   EPUtils::get3DNeighbors<pcl::PointXYZRGB>(patches,neighbors3D,nr_patches,cloud,param.z_max);
//   
//   because neigbors are symmetrcal it is enough to go only through the half
//   for(unsigned i = 0; i < nr_patches; i++)
//   {
//     if(!(surfaces.at(i)->selected))
//       continue;
//     
//     for(unsigned j = 0; j < i; j++)
//     {
//       if(neighbors2D.at<bool>(i,j))
//       {
//         surfaces.at(i)->neighbors2D.insert(j);
// 	surfaces.at(j)->neighbors2D.insert(i);
//       }
//       
//       if(neighbors3D.at<bool>(i,j))
//       {
//         surfaces.at(i)->neighbors3D.insert(j);
// 	surfaces.at(j)->neighbors3D.insert(i);
//       }
//     }
//   }
// }

/**
 * getSurfaceModels
 */
void SurfaceModeling::copySurfaces()
{
//   for (int i = 0; i < surfaces.size(); i++) 
//   {
//     surfaces.at(i)->neighbors2D.clear();
//     surfaces.at(i)->neighbors3D.clear();
//   }
  
/*//   if(filter_by_size)
//   {
//     for(int s = 0; s < surfaces.size(); s++) 
//     {
//       if( (surfaces.at(s)->indices.size()) < minSurfaceSize )
//       {
// 	surfaces.at(s)->valid = false;
// 	surfaces.at(s)->selected = false;
//       }
//     }
//   }
  
//   for(int s = 0; s < surfaces.size(); s++) 
//   {
//     if( (surfaces.at(s)->selected) && (surfaces.at(s)->type != pcl::SACMODEL_PLANE) && (surfaces.at(s)->type != MODEL_NURBS) )
//     {
//       surfaces.at(s)->valid = false;
//       surfaces.at(s)->selected = false;
//     }
//   }*/
  
//   for(int i = 0; i < surfaces.size(); i++) 
//   {
//     for(int j = 0; j < i; j++)
//     {
//       if((!(surfaces.at(i)->valid)) || (!(surfaces.at(j)->valid)))
// 	continue;
//       
//       if(neigbouring_matrix2D.at<bool>(i,j) || neigbouring_matrix2D.at<bool>(j,i))
//       {
// 	surfaces.at(i)->neighbors2D.insert(j);
// 	surfaces.at(j)->neighbors2D.insert(i);
//       }
//       
//       if(neigbouring_matrix3D.at<bool>(i,j) || neigbouring_matrix3D.at<bool>(j,i))
//       {
// 	surfaces.at(i)->neighbors3D.insert(j);
// 	surfaces.at(j)->neighbors3D.insert(i);
//       }
//     }
//   }

  // copy surface normals to view
  for(unsigned int s = 0; s < surfaces.size(); s++) 
  {
    if( (!(surfaces.at(s)->valid)) || (!(surfaces.at(s)->selected)) )
      continue;
    
    for(unsigned int i = 0; i < surfaces.at(s)->indices.size(); i++)
    {
      pcl::Normal n;
      n.normal_x = surfaces.at(s)->normals.at(i)[0];
      n.normal_y = surfaces.at(s)->normals.at(i)[1];
      n.normal_z = surfaces.at(s)->normals.at(i)[2];
      normals->points.at(surfaces.at(s)->indices.at(i)) = n;
    }
  }
  
}

void SurfaceModeling::pruneSurfaces()
{
  std::vector<SurfaceModel::Ptr>::iterator surf_itr = surfaces.begin();
//   int curent_surface_number = 0;
  while(surf_itr != surfaces.end())
  {
    //if the surface is valid
    if((*surf_itr)->valid)
    {
      surf_itr++;
//       curent_surface_number++;
      continue;
    }
    
    surf_itr = surfaces.erase(surf_itr,surf_itr+1);
  }
    
//   for (int i = 0; i < surfaces.size(); i++) 
//   {
//     surfaces.at(i)->neighbors2D.clear();
//     surfaces.at(i)->neighbors3D.clear();
//   }
    
}

/************************** PUBLIC *************************/

/**
 * setSurfaces
 */
void SurfaceModeling::setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
}

/** Set boundary 2D **/
void SurfaceModeling::setBoundary2D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr2D_map)
{
  ngbr2D_map = _ngbr2D_map;
  have_boundary2D = true;
}

/** Set boundary 3D **/
void SurfaceModeling::setBoundary3D(const std::map<borderIdentification,std::vector<neighboringPair> > _ngbr3D_map)
{
  ngbr3D_map = _ngbr3D_map;
  have_boundary3D = true;
}

/**
 * setIntrinsic
 */
void SurfaceModeling::setIntrinsic(double fx, double fy, double cx, double cy)
{
  camIntr = Eigen::Matrix4d::Zero();
  camIntr(0, 0) = fx;
  camIntr(1, 1) = fy;
  camIntr(0, 2) = cx;
  camIntr(1, 2) = cy;
  camIntr(2, 2) = 1.0;
  haveIntr = true;
}

/**
 * setExtrinsic
 */
void SurfaceModeling::setExtrinsic(Eigen::Matrix4d &pose)
{
  camExtr = pose;
  haveExtr = true;
}

/** 
 *set minimum number of points in the surface for it to be valid 
 **/
void SurfaceModeling::setMinSurfaceSize(int _minSurfaceSize)
{
  if(_minSurfaceSize > 0)
  {
    filter_by_size = true;
    minSurfaceSize = _minSurfaceSize;
  }
  else
  {
    filter_by_size = true;
    minSurfaceSize = _minSurfaceSize;
  }
}

void SurfaceModeling::printErrorsAndProbs(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    for(unsigned int j = 0; j < surfaces.at(i)->error.size(); ++j)
    {
      fprintf(f,"%f ",surfaces.at(i)->error.at(j));
    }
    
    fprintf(f,"\n");
    
    for(unsigned int j = 0; j < surfaces.at(i)->probs.size(); ++j)
    {
      fprintf(f,"%f ",surfaces.at(i)->probs.at(j));
    }
    
    fprintf(f,"\n");
  }
  fclose(f);
}

void SurfaceModeling::printNeigbours(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(int i = 0; i < neigbouring_matrix2D.rows; ++i)
  {
    for(int j = 0; j < neigbouring_matrix2D.cols; ++j)
    {
      fprintf(f,"%d ",(neigbouring_matrix2D.at<bool>(i,j) ? 1 : 0));
    }
    fprintf(f,"\n");
  }
  
  fprintf(f,"\n");
  
  for(int i = 0; i < neigbouring_matrix3D.rows; ++i)
  {
    for(int j = 0; j < neigbouring_matrix3D.cols; ++j)
    {
      fprintf(f,"%d ",(neigbouring_matrix3D.at<bool>(i,j) ? 1 : 0));
    }
    fprintf(f,"\n");
  }
  
  fclose(f);
}

void SurfaceModeling::printSurfaces(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(unsigned int s = 0; s < surfaces.size(); ++s)
  {
    if(!(surfaces.at(s)->valid))
      continue;
    
    for(unsigned int i = 0; i < surfaces.at(s)->indices.size(); ++i)
    {
      fprintf(f,"%d %f %f %f\n",surfaces.at(s)->indices.at(i),surfaces.at(s)->normals.at(i)[0],surfaces.at(s)->normals.at(i)[1],surfaces.at(s)->normals.at(i)[2]);
    }
    fprintf(f,"\n");
  }
  
  fclose(f);
}

/**
 * Computes model for each patch (NURBS or simple plane)
 */
void SurfaceModeling::compute()
{  
  if((!have_surfaces) || (!have_cloud) || (!have_normals) || (!have_boundary2D) || (!have_boundary3D))
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud, normals, boundary and surfaces.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

//   createNeighbours();

  init();
//   printErrorsAndProbs("probs.txt");
//   exit(0);
//   computeNeighbors();
  
//   printNeigbours("neighbours.txt");
//   exit(0);

  modelSelection();

  copySurfaces();
  
//   printSurfaces("surfaces.txt");
//   exit(0);

}

} //namespace surface

