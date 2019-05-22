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
 * @file BoundaryDetector.h
 * @author Andreas Richtsfeld, Ekaterina Potapova
 * @date November 2013
 * @version 0.1
 * @brief Estimate all boundary structures of surface models and views.
 */

#include "BoundaryDetector.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

BoundaryDetector::BoundaryDetector() : ContourDetector()
{
  ClassName = "BoundaryDetector";
}

BoundaryDetector::~BoundaryDetector()
{
}

// ================================= Private functions ================================= //



// ================================= Public functions ================================= //

void BoundaryDetector::recursiveClustering(int x, int y, int id0, int id1, bool isHorizontal, aEdge &_ec)
{

  int dx[6] = {0, 0, 0,0, 1,1};
  int dy[6] = {0,-1,-1,1,-1,0};
  bool horizontal[6] = {true,false,true,false,true,true};
  int newId0[6] = {id0,id0,id1,id0,id0,id1};
  int newId1[6] = {id1,id1,id0,id1,id1,id0};
  
  int cornerX[6] = {-1, 0,-1,0, 1,1};
  int cornerY[6] = { 0,-2,-1,1,-1,0};
  
  if(isHorizontal)
  {
    for(int j = 0; j < 6; ++j)
    {
      horizontal[j] = !(horizontal[j]);
      int temp = dx[j];
      dx[j] = dy[j];
      dy[j] = temp;
      
      int tempCorner = cornerX[j];
      cornerX[j] = cornerY[j];
      cornerY[j] = tempCorner;
    }
  }


  for(int j = 0; j < 6; ++j)
  {
    int nx = x + dx[j];
    int ny = y + dy[j];

    if( (nx < 0) || (nx >= width) || (ny < 0) || (ny >= height) )
      continue;
    
    int idx = getIdx(nx,ny);

    int nCornerX = x + cornerX[j];
    int nCornerY = y + cornerY[j];
    
    int cornerIdx = -1;
    //corner is out of image range
    if( (nCornerX >= 0) && (nCornerX < width) && (nCornerX >= 0) && (nCornerY < height) )
    {
      cornerIdx = getIdx(nCornerX,nCornerY);
    }
    

    
    if( horizontal[j] )
    { 
      if( (pre_edgels.at(idx).h_valid) && (pre_edgels.at(idx).h_ids[0] == newId0[j]) && (pre_edgels.at(idx).h_ids[1] == newId1[j]) )
      {
	
        surface::aEdgel egl;
	egl.index = pre_edgels.at(idx).index;
	egl.horizontal = true;
	egl.h_valid = true;
	egl.vertical = false;
	egl.v_valid = false;
	egl.h_ids[0] = pre_edgels.at(idx).h_ids[0];
	egl.h_ids[1] = pre_edgels.at(idx).h_ids[1];
	
        bd_edgels.push_back(egl);
	pre_edgels.at(idx).h_valid = false;
	
        _ec.edgels.push_back(bd_edgels.size()-1);
	
	if(cornerIdx < 0)
	{
	  _ec.corners[1] = -2;
	  return;
	}
	
        if( (pre_corners.at(cornerIdx).index != -1) && (_ec.corners[0] != pre_corners.at(cornerIdx).index) )
        {
          _ec.corners[1] = pre_corners.at(cornerIdx).index;
          return;
        }
        else
        {
          recursiveClustering(nx,ny,newId0[j],newId1[j],true,_ec);
          return;
        }
      }
    }
    else
    {
      
      if( (pre_edgels.at(idx).v_valid) && (pre_edgels.at(idx).v_ids[0] == newId0[j]) && (pre_edgels.at(idx).v_ids[1] == newId1[j]) )
      {
	
	surface::aEdgel egl;
	egl.index = pre_edgels.at(idx).index;
	egl.horizontal = false;
	egl.h_valid = false;
	egl.vertical = true;
	egl.v_valid = true;
	egl.v_ids[0] = pre_edgels.at(idx).v_ids[0];
	egl.v_ids[1] = pre_edgels.at(idx).v_ids[1];
	
        bd_edgels.push_back(egl);
	pre_edgels.at(idx).v_valid = false;
	
        _ec.edgels.push_back(bd_edgels.size()-1);
	
	if(cornerIdx < 0)
	{
	  _ec.corners[1] = -2;
	  return;
	}
	
        if( (pre_corners.at(cornerIdx).index != -1) && (_ec.corners[0] != pre_corners.at(cornerIdx).index) )
        {
          _ec.corners[1] = pre_corners.at(cornerIdx).index;
          return;
        }
        else
        {
	  recursiveClustering(nx,ny,newId0[j],newId1[j],false,_ec);
          return;
        }
      }
    }
  }
}


/**
 * @brief 
 */
void BoundaryDetector::computeEdges()
{
  for(unsigned int i = 0; i < pre_corners.size(); i++)
  {
    // if there is a corner at this point
    if(pre_corners.at(i).index != -1)
    {
      
      int tx = X(pre_corners.at(i).index);
      int ty = Y(pre_corners.at(i).index);
      
      int dcol[4] = {0,1,1,0};
      int drow[4] = {0,0,1,1};
      bool horizontal[4] = {false,true,false,true};
      bool valid[4] = {false,false,false,false};

      //verify, that we do not overight some contours
      for(int j1 = 0; j1 < 4; j1++)
      {
        
        int j2 = (j1+1)%4;
        
        if(pre_corners.at(i).ids[j1] != pre_corners.at(i).ids[j2])
        {
          int x1 = tx + dcol[j1];
          int y1 = ty + drow[j1];
          int idx1 = getIdx(x1,y1);
          
          int x2 = tx + dcol[j2];
          int y2 = ty + drow[j2];
          int idx2 = getIdx(x2,y2);

          int idx = (idx1 < idx2 ? idx1 : idx2);

          if(horizontal[j1])
          {
            if(pre_edgels.at(idx).h_valid)
            {
              pre_edgels.at(idx).h_valid = false;
              valid[j1] = true;
            }
          }
          else
          {
            if(pre_edgels.at(idx).v_valid)
            {
              pre_edgels.at(idx).v_valid = false;
              valid[j1] = true;
            }
          }
        }
      }
      
      for(int j1 = 0; j1 < 4; j1++)
      {
         
        int j2 = (j1+1)%4;
      
        if(pre_corners.at(i).ids[j1] != pre_corners.at(i).ids[j2])
        {
          int x1 = tx + dcol[j1];                                                        
          int y1 = ty + drow[j1];
          int idx1 = getIdx(x1,y1);
	  
	  int x2 = tx + dcol[j2];                                                        
          int y2 = ty + drow[j2];
          int idx2 = getIdx(x2,y2);
	  
	  int idx = (idx1 < idx2 ? idx1 : idx2);
	  int id0 = (idx1 < idx2 ? pre_corners.at(i).ids[j1] : pre_corners.at(i).ids[j2]);
	  int id1 = (idx1 < idx2 ? pre_corners.at(i).ids[j2] : pre_corners.at(i).ids[j1]);
	  
          if(horizontal[j1])
          {
            if(valid[j1])
            {
              surface::aEdge ec;
              ec.surfaces[0] = id0;
              ec.surfaces[1] = id1;
              ec.corners[0] = pre_corners.at(i).index;
              ec.corners[1] = -1;

	      surface::aEdgel egl;
	      egl.index = pre_edgels.at(idx).index;
	      egl.horizontal = true;
	      egl.h_valid = true;
	      egl.vertical = false;
	      egl.v_valid = false;
	      egl.h_ids[0] = pre_edgels.at(idx).h_ids[0];
	      egl.h_ids[1] = pre_edgels.at(idx).h_ids[1];
	      
              bd_edgels.push_back(pre_edgels.at(idx));
              pre_edgels.at(idx).v_valid = false;

              ec.edgels.push_back(bd_edgels.size()-1);              
              recursiveClustering(X(idx),Y(idx),id0,id1,true,ec);
          
              if(ec.corners[0] == -1 || ec.corners[1] == -1)
              {
                printf(" 1 => edge %lu => corners undefined: %i-%i\n", bd_edges.size(), ec.corners[0], ec.corners[1]);
              }
              else {
                bd_edges.push_back(ec);
              }
            }
          }
          else
          {
            if(valid[j1])
            {
              
              surface::aEdge ec;
              ec.surfaces[0] = id0;
              ec.surfaces[1] = id1;
              ec.corners[0] = pre_corners.at(idx).index;
              ec.corners[1] = -1;
              
	      surface::aEdgel egl;
	      egl.index = pre_edgels.at(idx).index;
	      egl.horizontal = false;
	      egl.h_valid = false;
	      egl.vertical = true;
	      egl.v_valid = true;
	      egl.v_ids[0] = pre_edgels.at(idx).v_ids[0];
	      egl.v_ids[1] = pre_edgels.at(idx).v_ids[1];
	      
              bd_edgels.push_back(egl);
              pre_edgels.at(idx).v_valid = false;
              
              ec.edgels.push_back(bd_edgels.size()-1);
              
              recursiveClustering(X(idx),Y(idx),id0,id1,false,ec);
              
              if(ec.corners[0] == -1 || ec.corners[1] == -1)
              {
                printf(" 1 => edge %lu => corners undefined: %i-%i\n", bd_edges.size(), ec.corners[0], ec.corners[1]);
              }
              else {
                bd_edges.push_back(ec);
              }
            }
          }
        }
      }
    }
  }
  
  /// TODO Are there still unused edges => Edges without corner point.
  /// While edgels are still there:
  ///   create new edge with same corner point.
  ///
}


void BoundaryDetector::copyEdges()
{
  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    surfaces.at(i)->edges.clear();
  }
  
  corners.clear();
  for(unsigned int i = 0; i < pre_corners.size(); i++)
  {
    //there is a corner
    if(pre_corners.at(i).index != -1)
    {
      surface::Corner corner;
      corner.x = ((float)X(pre_corners.at(i).index)) + 0.5;
      corner.y = ((float)Y(pre_corners.at(i).index)) + 0.5;
      //why are we doing it???
      pre_corners.at(i).index = corners.size();
      corners.push_back(corner);
    }
  }
  
  // copy edges and edgels
  edges.clear();
  edgels.clear();
  for(unsigned int i=0; i < bd_edges.size(); i++)
  {
    surface::Edge edge;
    int cornerIdx1 = bd_edges.at(i).corners[0];
    int cornerIdx2 = bd_edges.at(i).corners[1];

    if(cornerIdx1 < 0)
    {
      edge.corner[0] = -2;
    }
    else
    {
      edge.corner[0] = pre_corners.at(cornerIdx1).index;
    }

    if(cornerIdx2 < 0)
    {
      edge.corner[1] = -2;
    }
    else
    {
      edge.corner[1] = pre_corners.at(cornerIdx2).index;
    }
    edge.surface[0] = bd_edges.at(i).surfaces[0];
    edge.surface[1] = bd_edges.at(i).surfaces[1];

    surface::aEdge currentEdge = bd_edges.at(i);
    
    // copy the edgels
    for(unsigned int j=0; j < currentEdge.edgels.size(); j++)
    {

      int edgelIdx = currentEdge.edgels.at(j);
      surface::aEdgel currentEdgel = bd_edgels.at(edgelIdx);

      // horizontal edge
      if(currentEdgel.horizontal)
      { 
        surface::Edgel e;
        e.x = ((float) X(currentEdgel.index));
        e.y = ((float) Y(currentEdgel.index)) + 0.5;
        edgels.push_back(e);
        edge.edgels.push_back(edgels.size()-1);
      }
      // vertical edge
      if(currentEdgel.vertical)
      {
        surface::Edgel e;
        e.x = ((float) X(currentEdgel.index)) + 0.5;
        e.y = ((float) Y(currentEdgel.index));
        edgels.push_back(e);
        edge.edgels.push_back(edgels.size()-1);
      }
    }

    edges.push_back(edge);
    if(edge.surface[0] != -1)
    {
      surfaces[edge.surface[0]]->edges.push_back(edges[edges.size()-1]);
    }
    if(edge.surface[1] != -1)
    {
      surfaces[edge.surface[1]]->edges.push_back(edges[edges.size()-1]);
    }
  }
}


void BoundaryDetector::compute()
{     
  if((!have_cloud) || (!have_surfaces))
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud and surfaces.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  if(!have_contours)
  {
    ContourDetector::compute();
  }

  corners.clear();
  edges.clear();
  edgels.clear();
  
  bd_edgels.clear();
  bd_edges.clear();

  printf("[BoundaryDetector::computeBoundaryNetwork] computeEdges start\n");
  computeEdges();               //< Compute edges
  printf("[BoundaryDetector::computeBoundaryNetwork] computeEdges done\n");

  copyEdges();                  //< Copy edges with corners and edgels to view

}

} // end surface












