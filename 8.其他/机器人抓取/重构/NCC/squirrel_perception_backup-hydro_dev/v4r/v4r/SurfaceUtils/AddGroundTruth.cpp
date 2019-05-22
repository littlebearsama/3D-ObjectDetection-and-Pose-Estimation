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
 * @file AddGroundTruth.cpp
 * @author Richtsfeld
 * @date Dezember 2012
 * @version 0.1
 * @brief Add ground truth to relations.
 */

#include "AddGroundTruth.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

AddGroundTruth::AddGroundTruth()
{
  have_cloud = false;
  have_surfaces = false;
  have_relations = false;
  computed = false;
}

AddGroundTruth::~AddGroundTruth()
{}

// ================================= Private functions ================================= //


// ================================= Public functions ================================= //

void AddGroundTruth::setInputCloud(pcl::PointCloud<pcl::PointXYZRGBL>::Ptr _cloud)
{
  if ( (_cloud->height <= 1) || (_cloud->width <= 1) || (!_cloud->isOrganized()) )
    throw std::runtime_error("[AddGroundTruth::setInputCloud] Invalid point cloud (height must be > 1)");

  //@ep: check type here!!!
  
  cloud = _cloud;
  width = cloud->width;
  height = cloud->height;

  have_cloud = true;
  have_surfaces = false;
  have_relations = false;
  computed = false;
}

void AddGroundTruth::setSurfaces(std::vector<SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
}

void AddGroundTruth::setRelations(std::vector<surface::Relation> _relations)
{
  relations = _relations;
  have_relations = true;
}

void AddGroundTruth::compute(int type)
{
  if((!have_cloud) || (!have_relations) || (!have_surfaces))
  {
    printf("[AddGroundTruth::compute]: Error: No input cloud and/or relations and/or surfaces available.\n");
    return;
  }
  
  // Label the surface patches according to labels of the point cloud
  unsigned int maxObjects = 0;
  for(unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if(cloud->points.at(i).label > maxObjects)
    {
      maxObjects = cloud->points.at(i).label;
    }
  }
  
  //std::cerr << "1:" << maxObjects << std::endl; 
  
  // go over all surfaces
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    //std::cerr << "2.1" << std::endl;
    if(!(surfaces.at(i)->selected))
      continue;
    
    std::vector<int> nrPoints;
    nrPoints.resize(maxObjects+1);
    
    for(unsigned int j = 0; j <= maxObjects; j++)
    {
      nrPoints.at(j) = 0;
    }
    
    //std::cerr << "2.2" << std::endl;
    
    for(unsigned int j=0; j<surfaces.at(i)->indices.size(); j++)
    {
      pcl::PointXYZRGBL p = cloud->points.at(surfaces.at(i)->indices.at(j));
      if(p.label != 0)
      {
        nrPoints.at(p.label) += 1;
      }
      else 
      {
        nrPoints.at(0) += 1;
      }
    }  
      
    //std::cerr << "2.3" << std::endl;
      
    int maxNrPoints = 0;
    int objectID = 0;
    for(unsigned int j = 0; j <= maxObjects; j++) 
    {
      if(nrPoints.at(j) > maxNrPoints) 
      {
        maxNrPoints = nrPoints.at(j);
        objectID = j;
      }
    }

    //std::cerr << "2.4" << std::endl;
    
    if( (type == ALL_RELATIONS) || (type == STRUCTURAL_RELATIONS) ) 
      surfaces.at(i)->label = objectID;
      
    if( (type == ALL_RELATIONS) || (type == ASSEMBLY_RELATIONS) ) 
      surfaces.at(i)->label_ass = objectID;
  }
  
  //std::cerr << "2" << std::endl;
    
  // Add ground truth to the relations
  for(unsigned int i = 0; i < relations.size(); i++)
  {
    //std::cerr << "11" << std::endl;
    if( (relations.at(i).type == STRUCTURAL_RELATIONS) && ( (type == ALL_RELATIONS) || (type == STRUCTURAL_RELATIONS) ) )
    { 
      int id_0_label = surfaces.at(relations.at(i).id_0)->label;
      int id_1_label = surfaces.at(relations.at(i).id_1)->label;
      
      if( (id_0_label == id_1_label) && (id_0_label != 0) ) 
      {
        relations.at(i).groundTruth = 1;
        printf(" AddGT: positive structural example (type: %u): %u-%u\n", 1, relations.at(i).id_0, relations.at(i).id_1);
      }
      
      if( (id_0_label != id_1_label) && ((id_0_label != 0) || (id_1_label != 0)) ) //(id_0_label != 0) && (id_1_label != 0) )//
      {
        relations.at(i).groundTruth = 0;
        printf(" AddGT: negative structural example (type: %u): %u-%u\n", 1, relations.at(i).id_0, relations.at(i).id_1);
      }
    }
    else if( (relations.at(i).type == ASSEMBLY_RELATIONS) && ( (type == ALL_RELATIONS) || (type == ASSEMBLY_RELATIONS) ) )
    {
//       int st_0_label = surfaces.at(relations.at(i).id_0)->label;
//       int st_1_label = surfaces.at(relations.at(i).id_1)->label;
      int as_0_label = surfaces.at(relations.at(i).id_0)->label_ass;
      int as_1_label = surfaces.at(relations.at(i).id_1)->label_ass;
        
      if( (as_0_label == as_1_label) && (as_0_label != 0) ) 
      {
        relations.at(i).groundTruth = 1;
        printf(" AddGT: positive assembly example (type: %u): %u-%u\n", 2, relations.at(i).id_0, relations.at(i).id_1);
      }
      if( (as_0_label != as_1_label) && ((as_0_label != 0) || (as_1_label != 0)) ) //(as_0_label != 0) && (as_1_label != 0) ) // 
      {
        relations.at(i).groundTruth = 0;
        printf(" AddGT: negative assembly example (type: %u): %u-%u\n", 2, relations.at(i).id_0, relations.at(i).id_1);
      }
    }
  }
}


}


