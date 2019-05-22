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
 * @file ContourDetector.cpp
 * @author Richtsfeld
 * @date January 2013
 * @version 0.1
 * @brief Base class for contour detection.
 */

#include "ContourDetector.h"

namespace surface
{


/************************************************************************************
 * Constructor/Destructor
 */

ContourDetector::ContourDetector()
: EPBase()
{
  initialized = false;
  have_contours = false;
  ClassName = "ContourDetector";
}

ContourDetector::~ContourDetector()
{
}

// ================================= Public functions ================================= //

//
void ContourDetector::setSurfaces(const std::vector<SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  
  pre_corners.resize(width * height);
  pre_edgels.resize(width * height);

  for(unsigned int i = 0; i < surfaces.size(); i++)
  {
    surfaces.at(i)->contours.clear();
    surfaces.at(i)->edges.clear();
  }
  
  have_surfaces = true;
}

void ContourDetector::createPatchImage() 
{
  patchImage = cv::Mat_<int>(height, width);
  patchImage.setTo(-1);
  
  if(width > 0)
  {
    for(unsigned int i = 0; i < surfaces.size(); i++)
    {
      if(!(surfaces.at(i)->selected))
	continue;
      
      for(unsigned int j = 0; j < surfaces.at(i)->indices.size(); j++)
      {
        int x = X(surfaces.at(i)->indices.at(j));
	int y = Y(surfaces.at(i)->indices.at(j));
	patchImage.at<int>(y,x) = i;
      }
    }
  }
}

//@ep: this should be rewritten using depth-first search, and no recursion!!!
void ContourDetector::recursiveContourClustering(int id, int start_x, int start_y, int x, int y,
                                                 int dir, std::vector<int> &contour, bool &end)
{
  if(end) {
    printf("[ContourDetector::recursiveContourClustering] Warning: Unexpected end found.\n");
    return;
  }

  // 7 0 1
  // 6 x 2
  // 5 4 3
  
  //bool found = false;
  // how we look at the current situation
  // where we came from - b
  // where we are - n
  //
  // * b 1
  // * n *
  // we want to start from 1 => _5
  // change direction
  dir += 5;
  
  int dx[8] = { 0, 1, 1, 1, 0,-1,-1,-1};
  int dy[8] = {-1,-1, 0, 1, 1, 1, 0,-1};
  
  for(int i = 0; i < 8; i++)
  {
    int di = i + dir;
    di = di % 8;

    int nx = x + dx[di];
    int ny = y + dy[di];

    // if pixel is valid and belongs to the same contour
    if( (nx >= 0) && (nx < width) && (ny >= 0) && (ny < height) && (contours(ny,nx) == id) )
    {
      //this shouldn't be like this, not doubled boundaries
      if( (nx == start_x) && (ny == start_y) && (contour.size() > 3) )
      {
        end = true;
        return;
      }
      contour.push_back(getIdx(nx,ny));
      recursiveContourClustering(id,start_x,start_y,nx,ny,di,contour,end);
      break;
    }
  }
}


/**
 * @brief Construct patch and contour image. Initialize pre_edgels and pre_corners
 */
void ContourDetector::initialize()
{
  if((!have_cloud) || (!have_surfaces)) 
  {
    char* error_message = new char[200];
    sprintf(error_message,"[%s::compute()]: I suggest you first set the point cloud and surfaces.",ClassName.c_str());
    throw std::runtime_error(error_message);
  }

  createPatchImage();
  
//   printPatches("patches.txt");
//   exit(0);
  
  contours = cv::Mat_<int>(height,width);
  contours.setTo(-1);
  
  #pragma omp parallel for      // => 4ms
  for(int row = 0; row < height; row++)
  {
    for(int col = 0; col < width; col++)
    {
      int idx = getIdx(col,row);

      // initialize corner
      pre_corners.at(idx).index = -1;
      
      // initialize edge
      pre_edgels.at(idx).index = -1;
      pre_edgels.at(idx).horizontal = false;
      pre_edgels.at(idx).h_valid = false;
      pre_edgels.at(idx).vertical = false;
      pre_edgels.at(idx).v_valid = false;

      // contour at image border
      // if it is image border than its definetely patch border
      if( ( (row == 0) || (row == (height-1)) || (col == 0) || (row == (width-1)) ) &&  (patchImage.at<int>(row, col) != -1) )
      {
        contours.at<int>(row,col) = patchImage.at<int>(row,col);
      }

      if( col < (width-1) )
      {
        // horizontal edge
        if(patchImage.at<int>(row,col) != patchImage.at<int>(row,col+1))
        {    
          contours.at<int>(row,col) = patchImage.at<int>(row,col);
          contours.at<int>(row,col+1) = patchImage.at<int>(row,col+1);
        }
      }
        
      if( row < (height-1) )
      {
        // vertical edge
        if(patchImage.at<int>(row, col) != patchImage.at<int>(row+1, col))
        {    
          contours.at<int>(row,col) = patchImage.at<int>(row,col);
          contours.at<int>(row+1,col) = patchImage.at<int>(row+1,col);
        }
      }
    }
  }
  
  #pragma omp parallel for     // => 7ms
  for(int row = 0; row < height; row++)
  {
    for(int col = 0; col < width; col++)
    {
      
      //structure
      // * | *
      // --c--
      // * | *

      int dcol[4] = {0,1,1,0};
      int drow[4] = {0,0,1,1};
      bool horizontal[4] = {false,true,false,true};

      int patchNumbers[4] = {-2,-2,-2,-2};

      //@ep: This is redundant
      
      for(int i1 = 0; i1 < 4; ++i1)
      {
        int col1 = col + dcol[i1];
        int row1 = row + drow[i1];

        if( (col1 >= width) || (row1 >= height))
          continue;
        
        int idx1 = getIdx(col1,row1);
        int patchNum1 = patchImage.at<int>(row1,col1);

        for(int j = 0; j < 4; j++)
        {
          
          if(patchNumbers[j] == -2)
          {
            patchNumbers[j] = patchNum1;
            break;
          }
          
          if(patchNumbers[j] == patchNum1)
          {
            break;
          }
        }

        int i2 = (i1+1) % 4;
        int col2 = col + dcol[i2];
        int row2 = row + drow[i2];

        if( (col2 >= width) || (row2 >= height))
          continue;

        int idx2 = getIdx(col2,row2);
        int patchNum2 = patchImage.at<int>(row2,col2);

        int idx = (idx1 < idx2 ? idx1 : idx2);
        
        if(patchNum1 != patchNum2)
        {
          pre_edgels.at(idx).index = idx;
          if(horizontal[i1])
          {
            pre_edgels.at(idx).horizontal = true;
            pre_edgels.at(idx).h_valid = true;
            pre_edgels.at(idx).h_ids[0] = (idx1 < idx2 ? patchNum1 : patchNum2);
            pre_edgels.at(idx).h_ids[1] = (idx1 < idx2 ? patchNum2 : patchNum1);
          }
          else
          {
            pre_edgels.at(idx).vertical = true;
            pre_edgels.at(idx).v_valid = true;
            pre_edgels.at(idx).v_ids[0] = (idx1 < idx2 ? patchNum1 : patchNum2);
            pre_edgels.at(idx).v_ids[1] = (idx1 < idx2 ? patchNum2 : patchNum1);
          }
        }
       
      }

      // number of borders found aroung the pixel
      int found = 0;
      for(int j = 0; j < 4; ++j)
      {
        if(patchNumbers[j] != -2)
          found++;
      }
      
      // it can be 3 only when (col < (widht-1)) && (row < (height-1))
      if(found >= 3)
      {        
        int idx = getIdx(col,row);
        
        // add corners
        pre_corners.at(idx).index = idx;
        pre_corners.at(idx).ids[0] = patchImage.at<int>(row,col);
        pre_corners.at(idx).ids[1] = patchImage.at<int>(row,col+1);
        pre_corners.at(idx).ids[2] = patchImage.at<int>(row+1,col+1);
        pre_corners.at(idx).ids[3] = patchImage.at<int>(row+1,col);
      }

    }
  }
  
  initialized = true;
}


void ContourDetector::compute()
{
  initialize();
  
//   printContourMap("contour_map.txt");
//   exit(0);
  
  size_t min_contour_size = 5;

  // start at top left and go through contours
  for(int row = 0; row < height; row++)
  {
    for(int col = 0; col < width; col++)
    {
      // if there is contour at this point
      if(contours(row,col) != -1)
      {
        int id = contours(row,col);
        std::vector<int> new_contour;
        
        new_contour.push_back(getIdx(col,row));
        bool end = false;
        recursiveContourClustering(id,col,row,col,row,4,new_contour,end);
        
        if(!end && new_contour.size() > 6)
        {
          printf("[ContourDetector::computeContours] This has found NO end: %u => size: %lu\n", id, new_contour.size());
        }
        
        // save contour
        if(new_contour.size() > min_contour_size)
        {
          surfaces.at(id)->contours.push_back(new_contour);
        }
        else
        {
          printf("[ContourDetector::computeContours] Warning: Contour %u with too few points: %lu\n", id, new_contour.size());
        }
        
        // delete contour points from contours map!
        for(unsigned int i = 0; i < new_contour.size(); i++)
        {
          contours(Y(new_contour.at(i)), X(new_contour.at(i))) = -1;
        }
      }
    }
  }
  
//   printContours("contours.txt");
//   exit(0);
  
  have_contours = true;
}

void ContourDetector::printContours(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(unsigned int i = 0; i < surfaces.size(); ++i)
  {
    for(unsigned int j = 0; j < surfaces.at(i)->contours.size(); ++j)
    {
      for(unsigned int k = 0; k < surfaces.at(i)->contours.at(j).size(); ++k)
      {
        fprintf(f,"%d ",surfaces.at(i)->contours.at(j).at(k));
      }
      fprintf(f,"\n");
    }
    fprintf(f,"\n");
  }
  
  fclose(f);
}

void ContourDetector::printContourMap(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(int i = 0; i < contours.rows; ++i)
  {
    for(int j = 0; j < contours.cols; ++j)
    {
      fprintf(f,"%d ",contours.at<int>(i,j));
    }
    fprintf(f,"\n");
  }
  
  fclose(f);
}

void ContourDetector::printPatches(std::string file_name)
{
  FILE *f = fopen(file_name.c_str(), "w");
  
  for(int i = 0; i < patchImage.rows; ++i)
  {
    for(int j = 0; j < patchImage.cols; ++j)
    {
      fprintf(f,"%d ",patchImage.at<int>(i,j));
    }
    fprintf(f,"\n");
  }
  
  fclose(f);
}

} // end surface












