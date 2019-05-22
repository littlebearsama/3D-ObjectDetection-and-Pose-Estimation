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

#include "ClusterNormalsToPlanes.hh"

namespace surface 
{
  
/********************** ClusterNormalsToPlanes ************************
 * Constructor/Destructor
 */
ClusterNormalsToPlanes::ClusterNormalsToPlanes(Parameter p):
EPBase()
{
  setParameter(p);
  pixel_check = false;
  max_neighbours = 4;
  max_nneighbours = 2*max_neighbours;
  
  srt_curvature.resize(20);
  
  ClassName = "ClusterNormalsToPlanes";
}

ClusterNormalsToPlanes::~ClusterNormalsToPlanes()
{
}

/************************** PRIVATE ************************/

void ClusterNormalsToPlanes::createPatchImage()
{
  patches = cv::Mat_<int>::zeros(height,width);
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    for(unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++)
    {
      int row = Y(surfaces.at(i)->indices.at(j));
      int col = X(surfaces.at(i)->indices.at(j));
      patches(row, col) = i+1;   // plane 1,2,...,n
    }
  }
}

/**
 * Count number of neighbouring pixels (8-neighbourhood)
 * @param nb Maximum neighbouring pixels
 * @param nnb Maximum neighbouring pixels with neighbouring neighbors
 * @param nnb_inc Increment value for neighbouring neighbors
 */
void ClusterNormalsToPlanes::countNeighbours(std::vector< std::vector<int> > &reassign_idxs, int nb, int nnb, int nnb_inc)
{
  reassign_idxs.clear();
  reassign_idxs.resize(surfaces.size());
  
  #pragma omp parallel for
  for(int row = 1; row < patches.rows-1; row++)
  {
    for(int col = 1; col < patches.cols-1; col++)
    {
      int nb_counter = 0;
      int nnb_counter = 0;
      bool neighbors[8] = {false, false, false, false, false, false, false, false};
      int dr[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
      int dc[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
      // no patch at this point
      if(patches(row,col) == 0)
        continue;

      for(int i = 0; i < 8; ++i)
      {
        if( patches(row,col) == patches(row+dr[i],col+dc[i]) )
        {
          nb_counter++;
          neighbors[i] = true;
        }
      }
        
      for(int i = 0; i < 8; ++i)
      {
        int j = (i+1) % 8;
//         int j = i+1;
//         if(j > 7) j=0;
        if( neighbors[i] && neighbors[j] )
        {
          nnb_counter += nnb_inc;
        }
      }

      if( (nb_counter <= nb) && ((nb_counter+nnb_counter) <= nnb) )
      {
        int idx = getIdx(col,row);
        if(mask.at(idx))
        {
          #pragma omp critical 
          {
            mask.at(idx) = false;
            reassign_idxs.at(patches(row, col) - 1).push_back(idx);
          }
        }
      }
    }
  }

}

/**
 * Reasign points to neighbouring patches
 * We allow the inlier distance to assign points to other patches
 */
bool ClusterNormalsToPlanes::reasignPoints(std::vector< std::vector<int> > &reassign_idxs)
{
  bool ready = false;
  bool assigned = false;
  bool have_assigned = false;
  
  while(!ready)
  {
    assigned = false;
    
    for(int i = (int)(reassign_idxs.size()-1); i >= 0 ; i--)
    {
      std::vector<int> not_assigned_indexes;
      
      for(int j = 0; j < (int)(reassign_idxs.at(i).size()); j++)
      {
        int idx = reassign_idxs.at(i).at(j);
        int row = Y(idx);
        int col = X(idx);

        if( (row < 0) || (row >= height) || (col < 0) || (col >= width) )
          continue;
        
        int surounding[surfaces.size()];
        for(unsigned int s = 0; s < surfaces.size(); s++)
        {
          surounding[s] = 0;
        }
        
        for(int v = row-1; v <= row+1; v++)
        {
          for(int u = col-1; u <= col+1; u++)
          {
            if( (v < 0) || (v >= height) || (u < 0) || (u >= width) )
              continue;

            if( ((v == row) && (u == col)) || (patches(v,u) == 0) || (patches(v,u) == patches(row,col)) )
              continue;
              
            int a_idx = getIdx(u,v);

            if( isNaN(cloud->points.at(a_idx)) )
              continue;
            
            float dist =(cloud->points.at(idx).getVector3fMap() - cloud->points.at(a_idx).getVector3fMap()).norm();
                
            if( dist < param.ra_dist )
            {
              surounding[patches(v,u)-1]++;
            }
          }
        }
        
        //BUG: what if we assign to the patch, that should be reassigned???
        
        // select the patch that is the closest to the point
        int max_neighbors = 0;
        int most_id = 0;
        for(unsigned int nr = 0; nr < surfaces.size(); nr++)
        {
          if(max_neighbors < surounding[nr])
          {
            max_neighbors = surounding[nr];
            most_id = nr;
          }
        }

        // we have found the patch
        if(max_neighbors > 0)
        {
          have_assigned = true;
          assigned = true;
          // add point to the indices of the new patch
          surfaces.at(most_id)->indices.push_back(idx);

          // remove point from the old patch
          std::vector<int> surfaces_indices_copy;
          for(unsigned int su = 0; su < surfaces.at(i)->indices.size(); su++)
          {
            if(surfaces.at(i)->indices.at(su) != idx)
            {
              surfaces_indices_copy.push_back(surfaces.at(i)->indices.at(su));
            }
          }
          surfaces.at(i)->indices = surfaces_indices_copy;
          // change patch image
          patches(row,col) = most_id+1;
        }
        else // there is no patch that is suitable
        {
          not_assigned_indexes.push_back(idx);
        }
      }

      // assign new points
      reassign_idxs.at(i) = not_assigned_indexes;
    }

    // no points were assigned, means that there are only isolated points, time to stop
    if( !assigned )
    {
      ready = true;
    }
  }

  // return true if we reasigned at least anything
  return have_assigned;
}


void ClusterNormalsToPlanes::deleteEmptyPlanes()
{ 
  std::vector<surface::SurfaceModel::Ptr>::iterator itr = surfaces.begin();
  while( itr != surfaces.end() )
  {
    if((int)((*itr)->indices.size()) <= 0)
    {
      itr = surfaces.erase(itr,itr+1);
    }
    else
    {
      if((int)((*itr)->indices.size()) < param.minPoints)
        (*itr)->type = -1;
      itr++;
    }
  }
}

/* Reasign single pixels (line-ends) */
void ClusterNormalsToPlanes::singlePixelCheck()
{
  int max_nb = 2;
  int max_nnb = 1;
  int nnb_inc = -1;
  bool assigned = true;
  std::vector< std::vector<int> > reassign_idxs;
  while(assigned) {
    countNeighbours(reassign_idxs, max_nb, max_nnb, nnb_inc);
    assigned = reasignPoints(reassign_idxs);
  }
}


void ClusterNormalsToPlanes::pixelCheck()
{ 
  printf("[ClusterNormalsToPlanes::pixelCheck()]: Surfaces before deletion: %lu\n", surfaces.size());

  std::vector< std::vector<int> > reassign_idxs;
  mask.clear(); // move pixel only once
  mask.resize(width*height,true);

  // Reassign patches which have less neighbouring points than minPoints
  createPatchImage();
  countNeighbours(reassign_idxs,max_neighbours,max_nneighbours,1);

  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    if( (surfaces.at(i)->indices.size() - reassign_idxs.at(i).size()) < ((unsigned int)param.minPoints) )
      reassign_idxs.at(i) = surfaces.at(i)->indices;
    else
      reassign_idxs.at(i).clear();
  }
  reasignPoints(reassign_idxs);
  
  // Reassign single-neighboured points of patches
  mask.clear();
  mask.resize(width*height,true);
  singlePixelCheck();
  deleteEmptyPlanes();
  
  printf("[ClusterNormalsToPlanes::pixelCheck()] Surfaces after deletion: %lu\n", surfaces.size());
}


/**
 * Cluster rest of the points
 */
// void ClusterNormalsToPlanes::clusterRest(int idx, std::vector<int> &pts, pcl::Normal &normal)
// {
//   pts.clear();
//   
//   normal = view->normals->points.at(idx);
// 
//   if(isNaN(normal))
//   {
//     printf("[ClusterNormalsToPlanes::clusterRest] Error: NAN found.\n");
//     //@ep: why there is no return???
//   }
// 
//   // point is not useful anymore
//   mask.at(idx) = false;
//   pts.push_back(idx);
//   
//   int queue_idx = 0;
//   std::vector<int> queue;
//   queue.reserve(width*height);
//   queue.push_back(idx);       
// 
//   while ((int)queue.size() > queue_idx)
//   {
//     idx = queue.at(queue_idx);
//     queue_idx++;
//     
//     int x = X(idx);
//     int y = Y(idx);
// 
//     for (int v = y-1; v <= y+1; v++)
//     {
//       for (int u = x-1; u <= x+1; u++)
//       {
//         if ( (v < 0) || (u < 0) || (v >= height) || (u >= width) )
//           continue;
//         
//         idx = getIdx(u,v);
//         // valid point
//         if (!(mask.at(idx)))
//           continue;
//         
//         mask.at(idx) = false;
// 
//         // update normal
//         normal.getNormalVector3fMap() = ((pts.size())*normal.getNormalVector3fMap() + view->normals->points.at(idx).getNormalVector3fMap()) / ((float)pts.size() + 1.0f);
//         normal.getNormalVector3fMap().normalize();
// 
//         pts.push_back(idx);
//         queue.push_back(idx);
//       }
//     }
//   }
// }

void ClusterNormalsToPlanes::calculateCloudAdaptiveParameters()
{
//   param.print();
  
  p_adaptive_cosThrAngleNC.resize(width*height,0);
  p_adaptive_inlDist.resize(width*height,0);

//   FILE *f = fopen("adaptive.txt", "w");
  
  #pragma omp parallel for
  for(unsigned int i=0; i < cloud->points.size(); i++)
  {
    if(isNaN(cloud->points.at(i)))
    {
      continue;
    }
      Eigen::Vector3f curPt = cloud->points.at(i).getVector3fMap();
      // @ep: why do we divide in two subcases?
      // z coordinate
      
      p_adaptive_inlDist.at(i) = param.omega_c + param.omega_g*curPt[2];
      
      if(curPt[2] <= param.d_c)
      {
        p_adaptive_cosThrAngleNC.at(i) = std::cos(param.epsilon_c);
// 	fprintf(f,"* %d: %f,%f,%f -- %f %f (%f,%f,%f,%f,%f) \n",i,curPt[0],curPt[1],curPt[2],p_adaptive_cosThrAngleNC[i],p_adaptive_inlDist[i],
// 	                                      param.epsilon_c, param.epsilon_g,param.omega_c,param.omega_g,param.d_c);
      }
      else
      {
        float temp1 = (curPt[2]-param.d_c);
        float temp2 = param.epsilon_g*temp1;
        float temp3 = param.epsilon_c + temp2;
        p_adaptive_cosThrAngleNC[i] = std::cos(temp3);
//         fprintf(f,"# %d: %f,%f,%f -- %f (%f,%f,%f) %f (%f,%f,%f,%f,%f) \n",i,curPt[0],curPt[1],curPt[2],p_adaptive_cosThrAngleNC[i],
// 	                                      temp1, temp2, temp3, p_adaptive_inlDist[i],
// 	                                      param.epsilon_c, param.epsilon_g,param.omega_c,param.omega_g,param.d_c);
      }
    
  }
  
//   fclose(f);
  
//   printCloudAdaptiveParams("adaptive.txt");
}

void ClusterNormalsToPlanes::printCloudAdaptiveParams(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");

  for(unsigned int i=0; i<p_adaptive_cosThrAngleNC.size(); i++) 
  {
    Eigen::Vector3f curPt = cloud->points.at(i).getVector3fMap();
    fprintf(f,"(%d,%d): %f,%f,%f -- %f %f \n",i%width,i/width,curPt[0],curPt[1],curPt[2],p_adaptive_cosThrAngleNC[i],p_adaptive_inlDist[i]);
  }
  fclose(f);
}

/**
 * ClusterNormals
 */
void ClusterNormalsToPlanes::clusterNormals(int idx, std::vector<int> &pts, pcl::Normal &normal, bool rest)
{
//   FILE *f = fopen("clusterNormals_fromIdx.txt", "w");
  
  pts.clear();

  normal = normals->points.at(idx);

  if(isNaN(normal))
    return;

  // point is not useful anymore
  mask.at(idx) = false;
  pts.push_back(idx);
  
//   fprintf(f,"start from index %d\n", idx);
  
  int queue_idx = 0;
  std::vector<int> queue;
  queue.reserve(width*height);
  queue.push_back(idx);
  
  EIGEN_ALIGN16 Eigen::Vector3f pt = cloud->points.at(idx).getVector3fMap();

  while (((int)queue.size()) > queue_idx)
  {
    // extract current index
    idx = queue.at(queue_idx);
    queue_idx++;
    
//     fprintf(f,"current index %d\n",idx);

//     int x = X(idx);
//     int y = Y(idx);

//     int dx[4] = {-1,1,0,0};
//     int dy[4] = {0,0,1,-1};
    
//     for (int v = y-1; v <= y+1; v++)
//     {
//       for (int u = x-1; u <= x+1; u++)
//       {
//       for (int j = 0; j < 4; j++)
//       {
//         int v = y + dy[j];
// 	int u = x + dx[j];
    
      std::vector<int> n4ind;  
      if(rest)
      {
	int x = X(idx);
        int y = Y(idx);
	for (int v=y-1; v<=y+1; v++) 
	{
          for (int u=x-1; u<=x+1; u++) 
	  {
            if (v>=0 && u>=0 && v<height && u<width) 
	    {
              int temp_idx = getIdx(u,v);
	      n4ind.push_back(temp_idx);
	    }
	  }
	}
      }
      else
      {
        n4ind.push_back(idx-1);
        n4ind.push_back(idx+1);
        n4ind.push_back(idx+width);
        n4ind.push_back(idx-width);
      }
      for(unsigned i=0; i<n4ind.size(); i++) 
      {
        int u = n4ind[i] % width;
        int v = n4ind[i] / width;
	
	if ( (v < 0) || (u < 0) || (v >= height) || (u >= width) )
          continue;

        idx = getIdx(u,v);
	
// 	fprintf(f,"current sub index %d\n",idx);

        // not valid or not used point
        if (!(mask.at(idx)))
          continue;

        EIGEN_ALIGN16 Eigen::Vector3f n = normals->points.at(idx).getNormalVector3fMap();
	if(isNaN(n))
          continue;

        if(!rest)
        {
  
          float newCosThrAngleNC = cosThrAngleNC;
          float newInlDist = param.inlDist;
          
          if(param.adaptive)
          {
            newCosThrAngleNC = p_adaptive_cosThrAngleNC.at(idx);
            newInlDist = p_adaptive_inlDist.at(idx);
          }

//           fprintf(f,"newCosThrAngleNC = %f,newInlDist = %f\n",newCosThrAngleNC,newInlDist);
          
          float currentCos = normal.normal[0]*n[0] + normal.normal[1]*n[1] + normal.normal[2]*n[2];//normal.getNormalVector3fMap().dot(n);
// 	  
//           Dot3(&normal.normal[0], n)
//          inline T1 Dot3(const T1 v1[3], const T2 v2[3])
//          {
//          return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
//          }
	  
	  
          EIGEN_ALIGN16 Eigen::Vector3f currentPoint = cloud->points.at(idx).getVector3fMap();
	  float pd_p0 = currentPoint[0] - pt[0];
	  float pd_p1 = currentPoint[1] - pt[1];
	  float pd_p2 = currentPoint[2] - pt[2];
          float currentDist = fabs(pd_p0 * normal.normal[0] + pd_p1 * normal.normal[1] + pd_p2 * normal.normal[2]);//fabs(normal.getNormalVector3fMap().dot(currentPoint - pt));
	  
// 	  fprintf(f,"currentCos = %f,currentDist = %f\n",currentCos,currentDist);
	  
// 	  fabs(Plane::NormalPointDist(&pt[0], &normal.normal[0], &cloud.points[idx].x))
// 	  inline T1 Plane::NormalPointDist(const T1 p[3], const T2 n[3], T3 pd[3])
// 	  {
//  
// 	    T1 t[3];
// 
// 	    t[0] = pd[0] - p[0];
// 	    t[1] = pd[1] - p[1];
// 	    t[2] = pd[2] - p[2];
// 
// 	    return t[0]*n[0] + t[1]*n[1] + t[2]*n[2];
// 	    
// 	  }

          // we can add this point to the plane
          if ( (currentCos > newCosThrAngleNC) && (currentDist < newInlDist) )
          {
            mask.at(idx) = false;
            
            // update average normal
            // all points already in the plane have equal value into the plane
            // new normal = (number of points * current average normal + normal from the new point)/(number of points + 1)
	    
	    normal.normal[0] = normal.normal[0] * pts.size();
	    normal.normal[1] = normal.normal[1] * pts.size();
	    normal.normal[2] = normal.normal[2] * pts.size();
	    
// 	    Mul3(&normal.normal[0], pts.size(), &normal.normal[0]);
// 	    inline void Mul3(const T1 v[3], T2 s, T3 r[3])
// 	    {
// 	      r[0] = v[0]*s;
// 	      r[1] = v[1]*s;
// 	      r[2] = v[2]*s;
// 	    }

	    normal.normal[0] = normal.normal[0] + n[0];
	    normal.normal[1] = normal.normal[1] + n[1];
	    normal.normal[2] = normal.normal[2] + n[2];
	    
// 	    Add3(&normal.normal[0], &normals.points[idx].normal[0], &normal.normal[0]);
// 	    inline void Add3(const T1 v1[3], const T2 v2[3], T3 r[3])
// 	    {
// 	      r[0] = v1[0]+v2[0];
// 	      r[1] = v1[1]+v2[1];
// 	      r[2] = v1[2]+v2[2];
// 	    }
	    
// 	    normal.getNormalVector3fMap() = ((pts.size())*normal.getNormalVector3fMap() + n) / ((float)pts.size() + 1.0f);
//             normal.getNormalVector3fMap().normalize();
	    
            pt = pt*pts.size();
            pt += cloud->points[idx].getVector3fMap();

            // update average point of the plane
            // @ep: assuming all planes are concave?
//             pt = ((pts.size())*pt + currentPoint) / ((float)pts.size() + 1.0f);

            pts.push_back(idx);
            queue.push_back(idx);
	    
// 	    Mul3(&normal.normal[0], 1./(float)pts.size(), &normal.normal[0]);
	    normal.normal[0] = normal.normal[0] / ((float)pts.size());
	    normal.normal[1] = normal.normal[1] / ((float)pts.size());
	    normal.normal[2] = normal.normal[2] / ((float)pts.size());
	    
            pt /= ((float)pts.size());
            normal.getNormalVector3fMap().normalize();
	    
// 	    fprintf(f,"normal = %f,%f,%f pt = %f,%f,%f\n",normal.normal[0],normal.normal[1],normal.normal[2],pt[0],pt[1],pt[2]);
          }
        }
        else
        {
          mask.at(idx) = false;

          // update normal
// 	  Mul3(&normal.normal[0], pts.size(), &normal.normal[0]);
	  //normal.getNormalVector3fMap() = ((pts.size())*normal.getNormalVector3fMap() + n) / ((float)pts.size() + 1.0f);
	  //normal.getNormalVector3fMap().normalize();
	  normal.normal[0] = normal.normal[0] * pts.size();
	  normal.normal[1] = normal.normal[1] * pts.size();
	  normal.normal[2] = normal.normal[2] * pts.size();
//        Add3(&normal.normal[0], &normals.points[idx].normal[0], &normal.normal[0]);
	  normal.normal[0] = normal.normal[0] + normals->points[idx].normal[0];
	  normal.normal[1] = normal.normal[1] + normals->points[idx].normal[1];
	  normal.normal[2] = normal.normal[2] + normals->points[idx].normal[2];

          pts.push_back(idx);
          queue.push_back(idx);
              
          //Mul3(&normal.normal[0], 1./(float)pts.size(), &normal.normal[0]);
	  normal.normal[0] = normal.normal[0] / pts.size();
	  normal.normal[1] = normal.normal[1] / pts.size();
	  normal.normal[2] = normal.normal[2] / pts.size();
          normal.getNormalVector3fMap().normalize();
        }
      }  
//     }
  }
  
//   fclose(f);

  if(rest)
    return;
  
  if( (int) pts.size() >= param.minPoints)
  {
    std::vector<int>::iterator itr = pts.begin();
    while( itr != pts.end() )
    {
      EIGEN_ALIGN16 Eigen::Vector3f n = normals->points.at(*itr).getNormalVector3fMap();
      
      float newCosThrAngleNC = cosThrAngleNC;
      float newInlDist = param.inlDist;
      
      if(param.adaptive)
      {
        //@ep: it seems to be a bug -- idx --> *itr
	newCosThrAngleNC = p_adaptive_cosThrAngleNC.at(idx);
        newInlDist = p_adaptive_inlDist.at(idx);
      }          

      float currentCos = normal.normal[0]*n[0] + normal.normal[1]*n[1] + normal.normal[2]*n[2];//normal.getNormalVector3fMap().dot(n);
// 	  
//    Dot3(&normal.normal[0], n)
//    inline T1 Dot3(const T1 v1[3], const T2 v2[3])
//    {
//    return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
//    }

      EIGEN_ALIGN16 Eigen::Vector3f currentPoint = cloud->points.at(*itr).getVector3fMap();
      float pd_p0 = currentPoint[0] - pt[0];
      float pd_p1 = currentPoint[1] - pt[1];
      float pd_p2 = currentPoint[2] - pt[2];
      float currentDist = fabs(pd_p0 * normal.normal[0] + pd_p1 * normal.normal[1] + pd_p2 * normal.normal[2]);//fabs(normal.getNormalVector3fMap().dot(currentPoint - pt));
      //fabs(Plane::NormalPointDist(&pt[0], &normal.normal[0], &cloud.points[pts[i]].x)) > newInlDist)
      
// 	  fprintf(f,"currentCos = %f,currentDist = %f\n",currentCos,currentDist);
	  
// 	  fabs(Plane::NormalPointDist(&pt[0], &normal.normal[0], &cloud.points[idx].x))
// 	  inline T1 Plane::NormalPointDist(const T1 p[3], const T2 n[3], T3 pd[3])
// 	  {
//  
// 	    T1 t[3];
// 
// 	    t[0] = pd[0] - p[0];
// 	    t[1] = pd[1] - p[1];
// 	    t[2] = pd[2] - p[2];
// 
// 	    return t[0]*n[0] + t[1]*n[1] + t[2]*n[2];
// 	    
// 	  }
      
      // @ep: why is it && and not || ???
      if ( (currentCos < newCosThrAngleNC) && (currentDist > newInlDist) )
      {
        // check point as not used
        mask.at(*itr) = true;
        itr = pts.erase(itr, itr+1);
      }
      else
      {
        itr++;
      }
    }

    // re-calculate plane normal
    if(pts.size() > 0)
    {
      EIGEN_ALIGN16 Eigen::Vector3f new_n = normals->points.at(pts.at(0)).getNormalVector3fMap();
      
      for(unsigned int i = 1; i < pts.size(); i++)
        new_n += normals->points.at(pts.at(i)).getNormalVector3fMap();
      
      new_n.normalize();
      normal.normal[0] = new_n[0];
      normal.normal[1] = new_n[1];
      normal.normal[2] = new_n[2];
    }
  }
}

void ClusterNormalsToPlanes::printMask(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");

  for(unsigned int i = 0; i < mask.size(); i++)
  {
    fprintf(f,"(%d,%d): %d \n",i%width,i/width,(mask.at(i) ? 0 : 1));
  }
  fclose(f);
}

void ClusterNormalsToPlanes::printSrtCurvature(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");

  for(unsigned i = 0; i < srt_curvature.size(); i++)
  {
    for(unsigned int j = 0; j < srt_curvature.at(i).size(); ++j)
    {
      fprintf(f,"%f ",srt_curvature[i][j]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
}

void ClusterNormalsToPlanes::printClusteredIndices(std::string file_name, int idx, std::vector<int> pts)
{
//   std::sort(pts.begin(),pts.end());
  
  FILE *f = fopen(file_name.c_str(), "a");
  
  fprintf(f,"%d: ",idx);
  
  for(unsigned int i = 0; i < pts.size(); i++)
  {
    fprintf(f,"%d \n",pts.at(i));
  }
  
  fprintf(f,"\n");
  
  fclose(f);
}

/**
 * ClusterNormals
 */
void ClusterNormalsToPlanes::clusterNormals()
{
  // init mask (without nans); true - useful value, true - not useful value
  mask.clear();
  if (!have_indices)
  {
    mask.resize(width*height,true);
    for (int i = 0; i < width*height; i++)
    {
      if(isNaN(cloud->points.at(i)))
      {
        mask.at(i) = false;
      }
    }
  }
  else
  {
    mask.resize(width*height,false);
    for (unsigned int i = 0; i < indices.size(); i++)
    {
      int idx = indices.at(i);
      if(!isNaN(cloud->points.at(idx)))
      {
        mask.at(idx) = true;
      }
    }
  }
  
//   printMask("mask.txt");
  
  // @ep: why 20? what does it mean?
  for(unsigned int i = 0; i < srt_curvature.size(); i++)
  {
    srt_curvature.at(i).clear();
  }

  // @ep: during this step we cluster curvature values  
  for(int idx = 0; idx < (int)(normals->points.size()); idx++)
  {
    // @ep: not valid point
    if (!(mask.at(idx))) // == false
    {
      continue;
    }
    // @ep: here we ASSUME that curvature can't be negative
    int curv = (int) (normals->points.at(idx).curvature*1000);
    if(curv > 19)
      curv = 19;

    srt_curvature.at(curv).push_back(idx);

  }

// //   FILE *f = fopen("srt_curvature.txt", "w");
// 
//   for(unsigned i=0; i< srt_curvature.size(); i++)
//   {
// //     fprintf(f,"cleaning %d...\n",i);
//     srt_curvature[i].clear();
//   }
//   for(int idx=0; idx< (int) normals->points.size(); idx++) {
// //     fprintf(f,"index %d...\n",idx);
//     if(normals->points[idx].curvature == 0.0) {
// //       fprintf(f,"curvature 0.0...\n");
//       if (mask[idx])
//       {
//         srt_curvature[0].push_back(idx);
// // 	fprintf(f,"valid point, adding index to group 0...\n");
//       }
//     }
//     else {
//       double curvature = normals->points[idx].curvature*1000;
//       unsigned curv = (unsigned) curvature;
//       if(curv > 19)
//         curv = 19;
// //       fprintf(f,"curvature group %d...\n",curvature);
//       if (mask[idx])
//       {
//         srt_curvature[curv].push_back(idx);
// // 	fprintf(f,"valid point, adding index to group %d...\n",curv);
//       }
//     }
//   }
// 
// //   fclose(f);
  
//   printSrtCurvature("srt_curvature.txt");
  
  pcl::Normal normal;
  SurfaceModel::Ptr plane;
  
  // cluster points
//   int plane_count = 0;
  bool morePlanes = true;
  while(morePlanes)
  {
    morePlanes = false;
    // we start from plane points
    for(unsigned int i=0; i<srt_curvature.size(); i++)
    {
      // for all points with the same curvature
      for(unsigned int j=0; j<srt_curvature.at(i).size(); j++)
      {
        int idx = srt_curvature.at(i).at(j);
        // not valid point -- > skip
        if(!(mask.at(idx)))
          continue;

        plane.reset(new SurfaceModel());
        plane->type = pcl::SACMODEL_PLANE;

        // collect all points into plane->indices from the point with index idx, normal is the plane normal
        clusterNormals(idx,plane->indices,normal);
	
// 	printClusteredIndices("clustered_indices.txt",idx,plane->indices);
// 	plane_count++;
// 	if(plane_count >= 4)
// 	  exit(0);

        // plane has more than minimum required number of points
        if (((int)plane->indices.size()) >= param.minPoints)
        {
          //@ep: this assignment is very stragnge, what if we have finished clustered one curvature, but there is still something left?
	  morePlanes = true;
          surfaces.push_back(plane);
          plane->coeffs.resize(3);
          plane->coeffs[0] = normal.normal[0];
          plane->coeffs[1] = normal.normal[1];
          plane->coeffs[2] = normal.normal[2];
        }
        else
        {
          std::vector<int> &pts = plane->indices;
          for (unsigned i=0; i<pts.size(); i++)
            mask.at(pts.at(i)) = true;
        }
      }
    }
  }
  
//   exit(0);
  
  // cluster rest of unclustered point cloud in 2D
  for (int v = 0; v < (int)height; v++)
  {
    for (int u = 0; u < (int)width; u++)
    {
      int idx = getIdx(u,v);
      // valid point
      if (mask.at(idx))
      {
        plane.reset(new SurfaceModel());
        plane->type = -1; // No model
        clusterNormals(idx,plane->indices,normal,true);
// 	printClusteredIndices("clustered_indices.txt",idx,plane->indices);
        plane->coeffs.resize(3);
        plane->coeffs[0]=normal.normal[0];
        plane->coeffs[1]=normal.normal[1];
        plane->coeffs[2]=normal.normal[2];
        if (((int)plane->indices.size()) > 0)
	{
	  surfaces.push_back(plane);
	}
      }
    }
  }
  
//   exit(0);

}

/**
 * computeLeastSquarePlanes to check plane models
 */
void ClusterNormalsToPlanes::computeLeastSquarePlanes()
{
  pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> lsPlane(cloud);
  Eigen::VectorXf coeffs(4);

  for (unsigned int i = 0; i < surfaces.size(); i++)
  {
    if( (surfaces.at(i)->type == pcl::SACMODEL_PLANE) && (surfaces.at(i)->indices.size() > 4) )
    {
      lsPlane.optimizeModelCoefficients(surfaces.at(i)->indices, coeffs, coeffs);

      EIGEN_ALIGN16 Eigen::Vector3f currentPoint = cloud->points.at(surfaces.at(i)->indices.at(0)).getVector3fMap();

      if( (coeffs[0]*currentPoint[0] + coeffs[1]*currentPoint[1] + coeffs[2]*currentPoint[2]) > 0 )
      {
        coeffs*=-1.;
      }

      std::vector<float> currentCoeffs = surfaces.at(i)->coeffs;
      
      //@ep: why do we do this???
      if ( (coeffs[0]*currentCoeffs[0] + coeffs[1]*currentCoeffs[1] + coeffs[2]*currentCoeffs[2]) > cosThrAngleNC)
      {
        surfaces.at(i)->coeffs.resize(4);
        surfaces.at(i)->coeffs[0] = coeffs[0];
        surfaces.at(i)->coeffs[1] = coeffs[1];
        surfaces.at(i)->coeffs[2] = coeffs[2];
        surfaces.at(i)->coeffs[3] = coeffs[3];
      }
      else
      {
        if(surfaces.at(i)->indices.size() < 50)      /// HACK !!!
          surfaces.at(i)->type = -1;            // TODO Disable invalid planes

        printf("[ClusterNormalsToPlanes::computeLeastSquarePlanes()][Warning]: Problematic plane found: %u\n", i);//,ClassName.c_str());

      }
    }
  }
}

/**
 * Add normals to surface patches. Add directed plane normals (to camera view).
 */
void ClusterNormalsToPlanes::addNormals()
{
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    //@ep: TODO: this shouldn't be here!!!
    surfaces.at(i)->idx = i;
    
    surfaces.at(i)->normals.resize(surfaces.at(i)->indices.size());
    
    // no PLANE model
    if(surfaces.at(i)->type != pcl::SACMODEL_PLANE )
    {
      for(unsigned int ni = 0; ni < surfaces.at(i)->indices.size(); ni++)
      {
        Eigen::Vector3f nor = normals->points[surfaces[i]->indices[ni]].getNormalVector3fMap();
        surfaces.at(i)->normals.at(ni)[0] = nor[0];
        surfaces.at(i)->normals.at(ni)[1] = nor[1];
        surfaces.at(i)->normals.at(ni)[2] = nor[2];
      }
    }
    else
    {
      float coeffs[3];  // kann man das mit float * machen?
      coeffs[0] = surfaces.at(i)->coeffs[0];
      coeffs[1] = surfaces.at(i)->coeffs[1];
      coeffs[2] = surfaces.at(i)->coeffs[2];
      
      for (unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++)
      {
        surfaces.at(i)->normals.at(j)[0] = coeffs[0];
        surfaces.at(i)->normals.at(j)[1] = coeffs[1];
        surfaces.at(i)->normals.at(j)[2] = coeffs[2];
        // copy model normals to view
        normals->points.at(surfaces.at(i)->indices.at(j)).normal_x = coeffs[0];
        normals->points.at(surfaces.at(i)->indices.at(j)).normal_y = coeffs[1];
        normals->points.at(surfaces.at(i)->indices.at(j)).normal_z = coeffs[2];
      }
    }
  }

}


/************************** PUBLIC *************************/

/**
 * setParameter
 */
void ClusterNormalsToPlanes::setParameter(Parameter p)
{
  param = p;
  cosThrAngleNC = std::cos(param.thrAngle);
}

/**
 * @brief Check if there are patch models with "line"-style (not more than n neighbors)
 * @param check True to check
 * @param neighbors Threshold for line_check neighbors
 */
void ClusterNormalsToPlanes::setPixelCheck(bool check, int neighbors)
{
  pixel_check = check;
  max_neighbours = neighbors;
  max_nneighbours = neighbors*2;
}

void ClusterNormalsToPlanes::pruneSurfaces()
{
  std::vector<SurfaceModel::Ptr>::iterator itr = surfaces.begin();
  while(itr != surfaces.end())
  {
    if((*itr)->type != pcl::SACMODEL_PLANE)
    {
      itr = surfaces.erase(itr,itr+1);
    }
    else
      itr++;
  }
}

/**
 * Compute
 */
void ClusterNormalsToPlanes::compute()
{
  if((!have_cloud) || (!have_normals))
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud and normals.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }
  
  surfaces.clear();

  if(param.adaptive)
    calculateCloudAdaptiveParameters();
  
  clusterNormals();
  
  if(pixel_check)
    pixelCheck();

//   printClusters("clusters.txt");
//   exit(0);

  pruneSurfaces();
  
  computeLeastSquarePlanes();
  
  addNormals();
  
//   print("clusters.txt");
//   exit(0);
}

void ClusterNormalsToPlanes::printClusters(std::string file_name)
{
  FILE *f = std::fopen(file_name.c_str(), "w");
  createPatchImage();
  for(int i = 0; i < patches.rows; ++i)
  {
    for(int j = 0; j < patches.cols; ++j)
    {
      fprintf(f,"%d ",patches.at<int>(i,j));
    }
    fprintf(f,"\n");
  }
  std::fclose(f);
}

void ClusterNormalsToPlanes::print(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    fprintf(f,"%d ",surfaces.at(i)->type);
    if(surfaces.at(i)->type != -1)
      fprintf(f,"%f %f %f ",surfaces.at(i)->coeffs[0],surfaces.at(i)->coeffs[1],surfaces.at(i)->coeffs[2]);
    fprintf(f,"\n");
    
    for(unsigned int j = 0; j < surfaces.at(i)->indices.size(); ++j)
    {
      fprintf(f,"%d %f %f %f\n",surfaces.at(i)->indices.at(j),surfaces.at(i)->normals.at(j)[0],surfaces.at(i)->normals.at(j)[1],surfaces.at(i)->normals.at(j)[2]);
    }
    fprintf(f,"\n");
  }
  fclose(f);
}

void ClusterNormalsToPlanes::showSurfaces(cv::Mat &kImage)
{
  // create color image
  kImage = cv::Mat_<cv::Vec3b>::zeros(height,width);
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    uchar r = std::rand()%255;
    uchar g = std::rand()%255;
    uchar b = std::rand()%255;
    
    if(!(surfaces.at(i)->selected))
      continue;
    
    for(unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++)
    {
      int row = surfaces.at(i)->indices.at(j) / cloud->width;
      int col = surfaces.at(i)->indices.at(j) % cloud->width;
      cv::Vec3b &cvp = kImage.at<cv::Vec3b> (row, col);
      cvp[0] = r;
      cvp[1] = g;
      cvp[2] = b;
    }
  }
}

} //-- THE END --

