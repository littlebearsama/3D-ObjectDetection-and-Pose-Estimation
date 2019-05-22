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
 * @file GraphCut.cpp
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief GraphCut algorithm to cut graphs into pieces for segmentation.
 */


#include "GraphCut.h"

#ifndef GC_DEBUG
#define GC_DEBUG false
#endif

namespace gc
{

/**
 * @brief Constructor of GraphCut
 */
GraphCut::GraphCut()
{
  initialized = false;
  processed = false;
  createAllRelations = false;     // create fully connected graph to avoid segmentation fault
  print = false;
  have_surfaces = false;
  have_relations = false;
  
  ClassName = "GraphCut";
}


/**
 * @brief Destructor of GraphCut
 */
GraphCut::~GraphCut()
{}


void GraphCut::setSurfaces(const std::vector<surface::SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
}

void GraphCut::setRelations(std::vector<surface::Relation> _relations)
{
  relations = _relations;
  have_relations = true;
}

bool GraphCut::init()
{
  if((!have_surfaces) || (!have_relations)) 
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud and surfaces.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  
  if(relations.size() < 1)
  {
    printf("[GraphCut::init] Warning: No relations available!\n");
    //return false;
  }
  
  if(createAllRelations) 
  {
    int maxID = 0;
    for(unsigned int i = 0; i < relations.size(); i++) 
    {
      if(relations.at(i).id_0 > maxID)
        maxID = relations.at(i).id_0;
      if(relations.at(i).id_1 > maxID)
        maxID = relations.at(i).id_1;
    }
    for(int i=0; i<maxID; i++) 
    {
      for(int j=i+1; j<=maxID; j++) 
      {
        bool foundRelation = false;
        for(int k=0; k<(int)relations.size(); k++) 
	{
          if((relations.at(k).id_0 == i && relations.at(k).id_1 == j) ||
             (relations.at(k).id_1 == i && relations.at(k).id_0 == j))
	  {
            foundRelation = true;
	  }
        }
        if(!foundRelation) 
	{
          surface::Relation r;
          r.id_0 = i; 
          r.id_1 = j;
          // means that edges are disconnected
	  r.rel_probability.push_back(1.0f);
          r.rel_probability.push_back(0.0f);
          r.groundTruth = -1;
          relations.push_back(r);
        }
      }
    }
  }
  
  std::vector<gc::Edge> e;
  
  Graph graph(surfaces, relations);
  
  std::cerr << "graph initialized" << std::endl;
  
  graph.BuildFromSVM(e, num_edges, surfaces_reindex2);
  
  std::cerr << "graph created" << std::endl;

  edges = new gc::Edge[num_edges];
  for(unsigned i=0; i<num_edges; i++)
    edges[i] = e[i];

  if(num_edges == 0) 
  { 
    initialized = false;
    return false;
  } 
  else {
    u = new universe(num_edges);
    initialized = true;
    if(GC_DEBUG) 
      printf("[GraphCut::Initialize] num_edges: %u\n", num_edges);
  }
  return true;
}

/**
 * @brief Compare edges for sort function.
 * @param a Edge a
 * @param b Edge b
 * @return Returns true if a.w < b.w
 */
bool smallerEdge(const gc::Edge &a, const gc::Edge &b)
{
  return a.w < b.w;
}

bool smallerRelations(const surface::Relation &a, const surface::Relation &b)
{
  return a.rel_probability[0] < b.rel_probability[0];
}

/**
 * @brief Graph Cutting
 */
void GraphCut::process()
{
  if(!initialized) {
    printf("[GraphCut::process] Error: Graph is not initialized!\n");
    return;
  }

  if(GC_DEBUG) printf("[GraphCut::process] Start processing.\n");

  // sort edges by weight  
  std::cerr << "before sorting" << std::endl;
  std::sort(edges, edges + num_edges, smallerEdge);
  std::cerr << "after sorting" << std::endl;

  // init thresholds
  float *threshold = new float[num_edges];
  for (unsigned i = 0; i < num_edges; i++)
    threshold[i] = THRESHOLD(1, THRESHOLD_CONSTANT);
  
  std::cerr << "after threshold initialization" << std::endl;
  
  if(GC_DEBUG) printf("THRESHOLD: %4.3f\n", threshold[0]);
  
  // for each edge, in non-decreasing weight order...
  for (unsigned i = 0; i < num_edges; i++)
  {
    if(GC_DEBUG)
    {
      u->printAll();
      for (unsigned j = 0; j < num_edges; j++)
      {
        printf("get edge %i\n", j);
        gc::Edge *pedge = &edges[j];
        printf("  all edges: %u:", j);
        printf("  %u-%u with thrd: %4.3f", pedge->a, pedge->b, threshold[j]);
        printf("  => universe: %u-%u\n", u->find(pedge->a), u->find(pedge->b));
      }
    }
    
    if(GC_DEBUG) printf("\nstart with edge %u: ", i);

    gc::Edge *pedge = &edges[i];
    int a = u->find(pedge->a);    // components conected by this edge
    int b = u->find(pedge->b);
    if(GC_DEBUG) printf("node: %u-%u / universe: %u-%u: ", pedge->a, pedge->b, a, b);
    
    if (a != b)
    {
      if(GC_DEBUG)
        printf("weight: %4.3f and thds: %4.3f-%4.3f\n", pedge->w, threshold[a], threshold[b]);
      
      if ((pedge->w <= threshold[a]) && (pedge->w <= threshold[b])) 
      {
        u->join(a, b);
        a = u->find(a);
        if(GC_DEBUG)
          printf("  => join: threshold[%u] = w(%4.3f) + %4.3f => ", a, pedge->w, threshold[a]);
        threshold[a] = pedge->w + THRESHOLD(u->size(a), THRESHOLD_CONSTANT);
        if(GC_DEBUG)
          printf("%4.3f (size: %u)\n", threshold[a], u->size(a));
      }
    }
  }

  int num_components = u->num_sets();
  if(GC_DEBUG) printf("[GraphCut::process] Number of components: %u => What does that mean?\n", num_components);

  // copy graph cut groups
  std::vector<std::vector<int> > graphCutGroups;
  graphCutGroups.clear();
  int cut_labels[surfaces.size()];             // cut-ids for all models
  std::set<int> graphCutLabels;                      // all graph cut labels
  for(unsigned int i = 0; i < surfaces.size(); i++) {
    
    cut_labels[i] = -1;
    
    if(!(surfaces.at(i)->selected)) 
	continue;
    
    int cut_id = u->find(surfaces_reindex2.at(i));
    cut_labels[i] = cut_id;
    if(graphCutLabels.find(cut_id) == graphCutLabels.end())
      graphCutLabels.insert(cut_id);
  }
  
  std::set<int>::iterator it;
  for(it = graphCutLabels.begin(); it != graphCutLabels.end(); it++) {
    std::vector<int> cluster;
    for(unsigned int i = 0; i< surfaces.size(); i++)
      if((*it == cut_labels[i]) && (cut_labels[i] >= 0))
        cluster.push_back(i);
    graphCutGroups.push_back(cluster);
  }
  
  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    surfaces.at(i)->label = -1;
  }
  
  for(unsigned int i = 0; i < graphCutGroups.size(); i++)
    for(unsigned int j = 0; j < graphCutGroups[i].size(); j++)
    {
      printf("[GraphCut::process] Surface %u is in groud %u:\n",graphCutGroups[i][j],i);
      surfaces[graphCutGroups[i][j]]->label = i;
    }
  
  if(print) {
    printf("[GraphCut::process] Resulting groups:\n");
    for(unsigned int i = 0; i < relations.size(); i++) {
      if(relations[i].rel_probability[1] != 0.0)
        printf("  p(%u, %u) = %4.3f\n", relations[i].id_0, relations[i].id_1, relations[i].rel_probability[1]);
    }
    
    printf("[GraphCut::process] Resulting groups:\n");
    for(unsigned int i = 0; i < graphCutGroups.size(); i++) {
      printf("  Group %u: ", i);
      for(unsigned int j = 0; j < graphCutGroups[i].size(); j++)
        printf("%u ", graphCutGroups[i][j]);
      printf("\n");
    }
  }
      
  delete []threshold;
  delete []edges;
  initialized = false;
  processed = true;
}

void GraphCut::process2()
{
  if((!have_surfaces) || (!have_relations))
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud and surfaces.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

//   std::vector<surface::Relation>::iterator itr = relations.begin();
//   while(itr != relations.end())
//   {
//     if(itr->valid)
//     {
//       itr++;
//     }
//     else
//     {
//       itr = relations.erase(itr,itr+1);
//     }
//   }

  if(relations.size() < 1)
  {
    printf("[GraphCut::init] Warning: No relations available!\n");
    //return false;
  }
  
  // sort edges by weight
  std::sort(relations.begin(),relations.end(),smallerRelations);
  std::vector<bool> used_relations(relations.size(),false);

  for(unsigned int i = 0; i < relations.size(); ++i)
  {
//     printf("%d--%d\n",relations.at(i).id_0,relations.at(i).id_1);
  }
  
  std::vector<std::vector<int> > universe(surfaces.size());
  std::vector<float> threshold(surfaces.size());

  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    threshold.at(i) = THRESHOLD_CONSTANT;
    if(surfaces.at(i)->selected)
      universe.at(i).push_back(i);
  }
  
  // for each edge, in non-decreasing weight order...
  for(unsigned int r_idx = 0; r_idx < relations.size(); r_idx++)
  {
    if(used_relations.at(r_idx))
      continue;

//     printf("Processing relation %d\n",r_idx);
    
    //if probability that patches are disconnected by relation is lower than constant, than connect pathes
    if( ((relations.at(r_idx).rel_probability[0]) <= (threshold.at(relations.at(r_idx).id_0))) && ((relations.at(r_idx).rel_probability[0]) <= (threshold.at(relations.at(r_idx).id_1))) )
    {
//       printf("Relation will be connected\n");

      int id_0 = relations.at(r_idx).id_0;
      int id_1 = relations.at(r_idx).id_1;

//       printf("id0 = %d; id1 = %d\n",id_0,id_1);

      //calculate the size of each universe
      int uni0_size = 0;
      for(unsigned int i = 0; i < universe.at(id_0).size(); ++i)
      {
        uni0_size += surfaces.at(universe.at(id_0).at(i))->indices.size();
      }
      int uni1_size = 0;
      for(unsigned int i = 0; i < universe.at(id_1).size(); ++i)
      {
        uni1_size += surfaces.at(universe.at(id_1).at(i))->indices.size();
      }

//       printf("Size of universe 0 -- %d\n",uni0_size);
//       printf("Size of universe 1 -- %d\n",uni1_size);
      
      float w_uni0 = ((float)uni0_size)/((float)uni0_size + (float)uni1_size);
      float w_uni1 = ((float)uni1_size)/((float)uni0_size + (float)uni1_size);
      
      //add one part of the universe to another
      if( uni0_size > uni1_size )
      {
//         printf("Universe 1 will be added to universe 0\n");

        //id_1 --> id_0
        for(unsigned int i = 0; i < universe.at(id_1).size(); ++i)
        {
          universe.at(id_0).push_back(universe.at(id_1).at(i));
//           printf("adding surface %d\n",universe.at(id_1).at(i));
        }
        universe.at(id_1).clear();
        threshold.at(id_0) = relations.at(r_idx).rel_probability[0] + THRESHOLD(universe.at(id_0).size(), THRESHOLD_CONSTANT);
        used_relations.at(r_idx) = true;

        for(unsigned int i = 0; i < relations.size(); ++i)
        {
          if(used_relations.at(i))
            continue;

//           printf("looking at relation %d (%d--%d)\n",i,relations.at(i).id_0,relations.at(i).id_1);
          
          if( (relations.at(i).id_0 == id_0) && (relations.at(i).id_1 != id_1) )
          {
            int new_id = relations.at(i).id_1;
            for(unsigned int j = 0; j < relations.size(); ++j)
            {
              if(used_relations.at(j))
                continue;

              if( ( (relations.at(j).id_0 == new_id) && (relations.at(j).id_1 == id_1) ) || ( (relations.at(j).id_0 == id_1) && (relations.at(j).id_1 == new_id) ) )
              {
//                 relations.at(i).id_1 = id_0;
                relations.at(i).rel_probability[0] = w_uni1*relations.at(j).rel_probability[0] + w_uni0*relations.at(i).rel_probability[0];
                relations.at(i).rel_probability[1] = 1 - relations.at(j).rel_probability[0];
                used_relations.at(j) = true;

//                 printf("relation %d (%d--%d) is eliminated and will be added to relation %d (%d--%d)\n",j,relations.at(j).id_0,relations.at(j).id_1,i,relations.at(i).id_0,relations.at(i).id_1);
                break;
              }
            }
          }
          else if( (relations.at(i).id_1 == id_0) && (relations.at(i).id_0 != id_1) )
          {
            int new_id = relations.at(i).id_0;
            for(unsigned int j = 0; j < relations.size(); ++j)
            {
              if(used_relations.at(j))
                continue;
              
              if( ( (relations.at(j).id_0 == new_id) && (relations.at(j).id_1 == id_1) ) || ( (relations.at(j).id_0 == id_1) && (relations.at(j).id_1 == new_id) ) )
              {
                //                 relations.at(i).id_1 = id_0;
                relations.at(i).rel_probability[0] = w_uni1*relations.at(j).rel_probability[0] + w_uni0*relations.at(i).rel_probability[0];
                relations.at(i).rel_probability[1] = 1 - relations.at(j).rel_probability[0];
                used_relations.at(j) = true;

//                 printf("relation %d (%d--%d) is eliminated and will be added to relation %d (%d--%d)\n",j,relations.at(j).id_0,relations.at(j).id_1,i,relations.at(i).id_0,relations.at(i).id_1);
                break;
              }
            }
          }
        }

        for(unsigned int i = 0; i < relations.size(); ++i)
        {
          if(used_relations.at(i))
            continue;
          
          if(relations.at(i).id_0 == id_1)
          {
//             printf("relation %d (%d--%d) was moved to relation ",i,relations.at(i).id_0,relations.at(i).id_1);
            relations.at(i).id_0 = id_0;
//             printf("(%d--%d)\n",i,relations.at(i).id_0,relations.at(i).id_1);
          }
          else if(relations.at(i).id_1 == id_1)
          {
//             printf("relation %d (%d--%d) was moved to relation ",i,relations.at(i).id_0,relations.at(i).id_1);
            relations.at(i).id_1 = id_0;
//             printf("(%d--%d)\n",i,relations.at(i).id_0,relations.at(i).id_1);
          }
          
        }
        
      }
      else
      {
//         printf("Universe 0 will be added to universe 1\n");

        //id_0 --> id_1
        for(unsigned int i = 0; i < universe.at(id_0).size(); ++i)
        {
          universe.at(id_1).push_back(universe.at(id_0).at(i));
//           printf("adding surface %d\n",universe.at(id_0).at(i));
        }
        universe.at(id_0).clear();
        threshold.at(id_1) = relations.at(r_idx).rel_probability[0] + THRESHOLD(universe.at(id_1).size(), THRESHOLD_CONSTANT);
        used_relations.at(r_idx) = true;

        for(unsigned int i = 0; i < relations.size(); ++i)
        {
          if(used_relations.at(i))
            continue;

//           printf("looking at relation %d (%d--%d)\n",i,relations.at(i).id_0,relations.at(i).id_1);
          
          if( (relations.at(i).id_0 == id_1) && (relations.at(i).id_1 != id_0) )
          {
            int new_id = relations.at(i).id_1;
            for(unsigned int j = 0; j < relations.size(); ++j)
            {
              if(used_relations.at(j))
                continue;
              
              if( ( (relations.at(j).id_0 == new_id) && (relations.at(j).id_1 == id_0) ) || ( (relations.at(j).id_0 == id_0) && (relations.at(j).id_1 == new_id) ) )
              {
                //                 relations.at(i).id_1 = id_0;
                relations.at(i).rel_probability[0] = w_uni1*relations.at(j).rel_probability[0] + w_uni0*relations.at(i).rel_probability[0];
                relations.at(i).rel_probability[1] = 1 - relations.at(j).rel_probability[0];
                used_relations.at(j) = true;

//                 printf("relation %d (%d--%d) is eliminated and will be added to relation %d (%d--%d)\n",j,relations.at(j).id_0,relations.at(j).id_1,i,relations.at(i).id_0,relations.at(i).id_1);
                break;
              }
            }
          }
          else if( (relations.at(i).id_1 == id_1) && (relations.at(i).id_0 != id_0) )
          {
            int new_id = relations.at(i).id_0;
            for(unsigned int j = 0; j < relations.size(); ++j)
            {
              if(used_relations.at(j))
                continue;
              
              if( ( (relations.at(j).id_0 == new_id) && (relations.at(j).id_1 == id_0) ) || ( (relations.at(j).id_0 == id_0) && (relations.at(j).id_1 == new_id) ) )
              {
                //                 relations.at(i).id_1 = id_0;
                relations.at(i).rel_probability[0] = w_uni1*relations.at(j).rel_probability[0] + w_uni0*relations.at(i).rel_probability[0];
                relations.at(i).rel_probability[1] = 1 - relations.at(j).rel_probability[0];
                used_relations.at(j) = true;

//                 printf("relation %d (%d--%d) is eliminated and will be added to relation %d (%d--%d)\n",j,relations.at(j).id_0,relations.at(j).id_1,i,relations.at(i).id_0,relations.at(i).id_1);
                break;
              }
            }
          }
        }

        for(unsigned int i = 0; i < relations.size(); ++i)
        {
          if(used_relations.at(i))
            continue;
          
          if(relations.at(i).id_0 == id_0)
          {
//             printf("relation %d (%d--%d) was moved to relation ",i,relations.at(i).id_0,relations.at(i).id_1);
            relations.at(i).id_0 = id_1;
//             printf("(%d--%d)\n",i,relations.at(i).id_0,relations.at(i).id_1);
          }
          else if(relations.at(i).id_1 == id_0)
          {
//             printf("relation %d (%d--%d) was moved to relation ",i,relations.at(i).id_0,relations.at(i).id_1);
            relations.at(i).id_1 = id_1;
//             printf("(%d--%d)\n",i,relations.at(i).id_0,relations.at(i).id_1);
          }

        }
      }
    }

//     printf("Finished relation!\n");
  }

  printf("Wow, we are here!\n");
  
  std::vector<int> universe_number(universe.size(),-1);
  int current_uni_number = 0;
  for(unsigned int i = 0; i < universe.size(); ++i)
  {
    if(universe.at(i).size() > 0)
    {
      universe_number.at(i) = current_uni_number;
      current_uni_number++;
    }
  }

//   printf("And here!\n");
  
  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    surfaces.at(i)->label = -1;
  }

//   printf("And here too!\n");
  
  for(unsigned int i = 0; i < universe.size(); ++i)
  {
    for(unsigned int j = 0; j < universe.at(i).size(); ++j)
    {
      int idx = universe.at(i).at(j);
      surfaces.at(idx)->label = universe_number.at(i);
    }
  }
}

} 











