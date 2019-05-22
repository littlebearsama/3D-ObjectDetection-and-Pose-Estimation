/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
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


#include "TJ.hpp"

namespace AttentionModule
{

int dy8[8] = {-1,-1,-1,0,1,1,1,0};
int dx8[8] = {-1,0,1,1,1,0,-1,-1};

int dx4[4] = {-1,1,0,0};
int dy4[4] = {0,0,-1,1};

SaliencyLine::SaliencyLine()
{
  points.clear();
  saliency = 0;
  points_num = 0;
}

void SaliencyLine::clear()
{
  points.clear();
  saliency = 0;
  points_num = 0;
}

float SaliencyLine::getSaliency()
{
  return(saliency);
}

std::vector<JunctionNode> SaliencyLine::getPoints()
{
  return(points);
}

int SaliencyLine::getPointsNumber()
{
  return(points_num);
}

void SaliencyLine::addPoint(JunctionNode node)
{
  points.push_back(node);
  saliency = ((saliency * points_num) + node.saliency)/(points_num + 1);
  points_num += 1;
}

bool calculateSaliencyLine(cv::Mat mask, const cv::Mat symmetry, SaliencyLine &saliencyLine, unsigned int th)
{
  assert(mask.size() == symmetry.size());
  
  int width = mask.cols;
  int height = mask.rows;
  
  saliencyLine.clear();
  
  //we assume that each mask contains only one connected component
  int count = 0;
  for(int i = 0; i < height; ++i)
  {
    for(int j = 0; j < width; ++j)
    {
      if(mask.at<uchar>(i,j))
      {
	count++;
	mask.at<uchar>(i,j) = count;
      }
    }
  }
  
  //create saliency line
  for(int i = 0; i < height; ++i)
  {
    for(int j = 0; j < width; ++j)
    {
      if(mask.at<uchar>(i,j))
      {
	JunctionNode currentNode;
	
	currentNode.x = j;
	currentNode.y = i;
	currentNode.num = mask.at<uchar>(i,j);
	currentNode.type = UNKNOWN;
	currentNode.saliency = symmetry.at<float>(i,j);
	
	for(int k = 0; k < 8; ++k)
        {
          int nx = j + dx8[k];
          int ny = i + dy8[k];
      
          if((ny < 0) || (nx < 0) || (ny >= height) || (nx >= width))
           continue;
        
          if(mask.at<uchar>(ny,nx) > 0)
          {
	    currentNode.edges_num += 1;
	    currentNode.edges.push_back(mask.at<uchar>(ny,nx));
	    currentNode.type += 1;
          }
        }
	
	saliencyLine.addPoint(currentNode);
	
	//if(currentNode.type >= T_JUNCTION)
	//  std::cerr << "smth is definitely wrong here" << std::endl;
      }
    }
  }
  
  if(saliencyLine.getPoints().size() < th)
  {
    return(false);
  }
  
  return(true);
}

bool findTJunctions(SaliencyLine saliencyLine, std::vector<int> &tjunctionPointsIdx)
{
  tjunctionPointsIdx.clear();
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  for(unsigned int i = 0; i < nodes.size(); ++i)
  {
    if(nodes.at(i).type >= T_JUNCTION)
    {
      //std::cerr << "T_Junctions" << std::endl;
      tjunctionPointsIdx.push_back(i);
    }
  }
  
  if(tjunctionPointsIdx.size())
    return(true);
  
  return(false);
}

std::vector<int> findEndPoints(SaliencyLine saliencyLine, std::vector<int> segment)
{
  std::vector<JunctionNode> points = saliencyLine.getPoints();
  
  std::vector<int> endPoints;
  for(unsigned int i = 0; i < segment.size(); ++i)
  {
    if(points.at(segment.at(i)).type == END_POINT)
    {
      endPoints.push_back(segment.at(i));
    }
  }
  return(endPoints);
}

std::vector<int> findEndPoints(SaliencyLine saliencyLine)
{
  std::vector<JunctionNode> points = saliencyLine.getPoints();
  
  std::vector<int> endPoints;
  for(unsigned int i = 0; i < points.size(); ++i)
  {
    if(points.at(i).type == END_POINT)
    {
      endPoints.push_back(i);
    }
  }
  
  if(!(endPoints.size()))
    endPoints.push_back(0);
  
  return(endPoints);
}

std::vector<int> getEdges(std::vector<JunctionNode> nodes,int nodeIdx)
{
  std::vector<int> edgesIdx;
  for(unsigned int i = 0; i < nodes.at(nodeIdx).edges.size(); ++i)
  {
    int edgeIdxCurrent = nodes.at(nodeIdx).edges.at(i);
    
    for(unsigned int j = 0; j < nodes.size(); ++j)
    {
      if(nodes.at(j).num == edgeIdxCurrent)
      {
	edgesIdx.push_back(j);
	break;
      }
    }
  }
  
  return(edgesIdx);
}

void breakIntoSegments(SaliencyLine saliencyLine, std::vector<std::vector<int> > &segments)
{
  int pointsNumber = saliencyLine.getPointsNumber();
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  
  std::vector<int> tjunctionPointsIdx;
  if(!findTJunctions(saliencyLine,tjunctionPointsIdx))
  {
    std::vector<int> segmentCurrent;
    segmentCurrent.resize(pointsNumber);
    for(int i = 0; i < pointsNumber; ++i)
    {
      segmentCurrent.at(i) = i;
    }
    segments.push_back(segmentCurrent);
    return;
  }
  
  std::vector<bool> usedPoints(pointsNumber,false);
  
  // go over all t-junctions
  for(unsigned int i = 0; i < tjunctionPointsIdx.size(); ++i)
  {
    int tjunctionPointIdxCurrent = tjunctionPointsIdx.at(i);
    usedPoints.at(tjunctionPointIdxCurrent) = true;
    
    //find endges
    std::vector<int> currentEdges = getEdges(nodes,tjunctionPointIdxCurrent);
    
    // go over all segments
    for(unsigned int j = 0; j < currentEdges.size(); ++j)
    {
      std::vector<int> segmentCurrent;
      int currentNodeIdx = currentEdges.at(j);
      while(!usedPoints.at(currentNodeIdx))
      {
	usedPoints.at(currentNodeIdx) = true;
	segmentCurrent.push_back(currentNodeIdx);
	std::vector<int> currentEdgesTemp = getEdges(nodes,currentNodeIdx);
	for(unsigned int k = 0; k < currentEdgesTemp.size(); ++k)
	{
	  if((!usedPoints.at(currentEdgesTemp.at(k))) && (nodes.at(currentEdgesTemp.at(k)).type < T_JUNCTION))
	  {
	    currentNodeIdx = currentEdgesTemp.at(k);
	    break;
	  }
	}
      }
      segments.push_back(segmentCurrent);
    }
  }
}

void modifySymmetryLine(SaliencyLine saliencyLine, std::vector<bool> &usedPoints, float th)
{
  std::vector<std::vector<int> > segments;
  breakIntoSegments(saliencyLine,segments);
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  
  usedPoints.resize(saliencyLine.getPointsNumber(),true);
  
  for(unsigned int i = 0; i < segments.size(); ++i)
  {
    std::vector<int> segmentCurrent = segments.at(i);
    std::vector<int> endPoints = findEndPoints(saliencyLine,segmentCurrent);
    
    if(endPoints.size() <= 0)
      continue;
    
    unsigned int nodesToDeleteNum = (unsigned int)(segmentCurrent.size() - (int)(th*(segmentCurrent.size())));
    unsigned int j  = 0;
    while(j < nodesToDeleteNum)
    {
      for(unsigned int k = 0; k < endPoints.size(); ++k)
      {
	int endPointIdx = endPoints.at(k);
	usedPoints.at(endPointIdx) = false;
        std::vector<int> currentEdges = getEdges(nodes,endPointIdx);
        for(unsigned int l = 0; l < currentEdges.size(); ++l)
        {
	  if(usedPoints.at(currentEdges.at(l)))
	  {
	    usedPoints.at(currentEdges.at(l)) = false;
	    endPoints.at(k) = currentEdges.at(l);
	    j++;
	    break;
	  }
	  else if( (l+1) == currentEdges.size() )
	  {
// 	    std::cerr << "!" << std::endl;
	    j++;
	  }
        }
      }
    }
  }
}

void selectSaliencyCenterPoint(SaliencyLine saliencyLine, PointSaliency &center)
{
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  
  std::vector<int> tjunctionPointsIdx;
  bool isTJunction = findTJunctions(saliencyLine,tjunctionPointsIdx);
  int centerIdx = 0;
  
  if(isTJunction)
  {
//     std::cerr << "1!"<< std::endl;
    centerIdx = tjunctionPointsIdx.at(0);
    for(unsigned int i = 1; i < tjunctionPointsIdx.size(); ++i)
    {
//       std::cerr << i << " " << tjunctionPointsIdx.size() << std::endl;
      if(nodes.at(centerIdx).type < nodes.at(tjunctionPointsIdx.at(i)).type)
      {
// 	std::cerr << "2!"<< std::endl;
	centerIdx = tjunctionPointsIdx.at(i);
      }
//       std::cerr << "3!"<< std::endl;
    }
  }
  else
  {
//     std::cerr << "4!"<< std::endl;
    std::vector<int> endPoints = findEndPoints(saliencyLine);
//     std::cerr << "5!"<< std::endl;
    centerIdx = endPoints.at(0);
//     std::cerr << "7!"<< std::endl;
    unsigned int nodesToDeleteNum = nodes.size()/2;
//     std::cerr << "8!"<< std::endl;
    std::vector<bool> usedPoints(nodes.size(),true);
//     std::cerr << "9!"<< std::endl;
    
    unsigned int j  = 0;
    while(j < nodesToDeleteNum)
    {
//       std::cerr << j << "(" << nodesToDeleteNum << ")" << std::endl;
      usedPoints.at(centerIdx) = false;
      std::vector<int> currentEdges = getEdges(nodes,centerIdx);
      for(unsigned int k = 0; k < currentEdges.size(); ++k)
      {
// 	std::cerr << k << ":" << currentEdges.size() << std::endl;
	if(usedPoints.at(currentEdges.at(k)))
	{
	  centerIdx = currentEdges.at(k);
	  usedPoints.at(centerIdx) = false;
	  j++;
	  break;
	}
	else if( (k+1) == currentEdges.size() )
	{
// 	  std::cerr << "!" << std::endl;
	  j++;
	}
// 	std::cerr << "6!"<< std::endl;
      }
    }
  }
  
  center.point = cv::Point(nodes.at(centerIdx).x,nodes.at(centerIdx).y);
  center.saliency = saliencyLine.getSaliency();
}

void createSimpleLine(SaliencyLine saliencyLine, std::vector<cv::Point> &points)
{
  std::vector<JunctionNode> nodes = saliencyLine.getPoints();
  points.resize(nodes.size());
  for(unsigned int i = 0; i < nodes.size(); ++i)
  {
    int x = nodes.at(i).x;
    int y = nodes.at(i).y;
    
    points.at(i) = cv::Point(x,y);
  }
}

bool extractSaliencyLine(cv::Mat mask, cv::Mat map, SaliencyLine &saliencyLine, unsigned int th)
{
  cv::Mat skeleton;
  EPUtils::Skeleton(mask,skeleton);
  
  //cv::imshow("skeleton",255*skeleton);
  //cv::waitKey();
  
  if(!calculateSaliencyLine(skeleton,map,saliencyLine, th))
  {
    return false;
  }
  
  return(true);
  
}

void createAttentionPoints(std::vector<PointSaliency> saliencyPoints, std::vector<cv::Point> &attentionPoints)
{
  attentionPoints.clear();
  for(unsigned int i = 0; i < saliencyPoints.size(); ++i)
  {
    attentionPoints.push_back(saliencyPoints.at(i).point);
  }
}

/*struct DensePoints {
  std::vector<cv::Point> points;
  float saliency;
};

void DrawDensePoints(DensePoints points, cv::Mat &image, cv::Scalar color)
{
  for(int i = 0; i < points.points.size(); ++i)
  {
    cv::circle(image,points.points.at(i),5,color,-1);
  }
}

void DrawDensePoints(std::vector<DensePoints> points, cv::Mat &image)
{
  cv::Scalar color(0);
  for(int j = 0; j < points.size(); ++j)
  {
    color(0) = j+1;
    
    for(int i = 0; i < points.at(j).points.size(); ++i)
    {
      cv::circle(image,points.at(j).points.at(i),5,color,-1);
    }
  }
}

void SelectSymmetryDensePoints(SymmetryLine symmetry_line, DensePoints &dense_points)
{
  dense_points.points.clear();
  
  // find all junction points
  bool isTJunction = false;
  
  for(int i = 0; i < symmetry_line.points.size(); ++i)
  {
    if(symmetry_line.points.at(i).type >= T_JUNCTION)
    {
      isTJunction = true;
      Node cur_point = symmetry_line.points.at(i);
      dense_points.points.push_back(cv::Point(cur_point.x,cur_point.y));
    }
  }
  
  if(!isTJunction)
  {
    Node cur_point = symmetry_line.points.at(symmetry_line.points.size()/2);
    dense_points.points.push_back(cv::Point(cur_point.x,cur_point.y));
  }
  
  std::vector<int> end_points;
  if(!findEndPoints(symmetry_line,end_points))
    return;
  
  std::vector<int> segment_ends;
  std::vector<int> segment_sizes;
  breakIntoSegments(symmetry_line,end_points,segment_ends,segment_sizes);
  
  for(int i = 0; i < end_points.size(); ++i)
  {
    int parent_index = end_points.at(i);
    int segment_size = segment_sizes.at(i);
    int current_index = symmetry_line.points.at(parent_index).edges_with.at(0);
    
    //std::cerr << "current_index_to_remove = " << current_index_to_remove << std::endl;
    //std::cerr << i << " : segment_size = " << segment_size << std::endl;
    
    for(int j = 0; j < ((int)(0.5*segment_size)); ++j)
    {
      if(symmetry_line.points.at(current_index).type != REGULAR_POINT)
      {
	break;
      }
      
      int index1 = symmetry_line.points.at(current_index).edges_with.at(0);
      int index2 = symmetry_line.points.at(current_index).edges_with.at(1);
      
      if(index1 == parent_index)
      {
	parent_index = current_index;
	current_index = index2;
      }
      else
      {
	parent_index = current_index;
	current_index = index1;
      }
      
    }
    
    Node cur_point = symmetry_line.points.at(parent_index);
    dense_points.points.push_back(cv::Point(cur_point.x,cur_point.y));
    
  }
  
  dense_points.saliency = symmetry_line.saliency;
  
}*/

/*

void DrawLines(std::vector<ConnectedComponent> components, cv::Mat &image)
{
  for(int i = 0; i < components.size(); ++i)
  {
    for(unsigned j = 0; j < components.at(i).points.size(); ++j)
    {
    
      int x = components.at(i).points.at(j).x;
      int y = components.at(i).points.at(j).y;
    
      image.at<uchar>(y,x) = i+1;
    }
  }
}

void DrawSymmetryLine(SymmetryLine symmetry_line, cv::Mat &mask)
{
  for(unsigned j = 0; j < symmetry_line.points.size(); ++j)
  {
    int x = symmetry_line.points.at(j).x;
    int y = symmetry_line.points.at(j).y;
    
    mask.at<uchar>(y,x) = 1;
  }
}

void SelectSymmetryLine(SymmetryLine symmetry_line, ConnectedComponent &cur_connected_component, int width, int height)
{
  float saliency_value = symmetry_line.saliency;
  //std::cerr << symmetry_line.points.size() << std::endl;
  ModifySymmetryLine(symmetry_line,0.5);
  //std::cerr << symmetry_line.points.size() << std::endl;
  
  cv::Mat mask = cv::Mat_<uchar>::zeros(height,width);
  DrawSymmetryLine(symmetry_line,mask);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
  cv::dilate(mask, mask, element, cv::Point(-1,-1), 2);
  mask.convertTo(mask,CV_32F,1.0f);
  
  std::vector<ConnectedComponent> connected_components;  
  float th = 0.1;
  extractConnectedComponents(mask,connected_components,th);
  cur_connected_component = connected_components.at(0);
  cur_connected_component.average_saliency = saliency_value;
}

void DrawSkeleton(cv::Mat &image, cv::Mat result_skeleton)
{
  for(int i = 0; i < image.rows; ++i)
  {
    for(int j = 0; j < image.cols; ++j)
    {
      if(result_skeleton.at<uchar>(i,j) > 0)
      {
	image.at<uchar>(i,3*j+0) = 0;
        image.at<uchar>(i,3*j+1) = 255;
        image.at<uchar>(i,3*j+2) = 0;
      }
    }
  }
}*/

//bool lines_all_sort_function(ConnectedComponent i, ConnectedComponent j)  { return (i.average_saliency>j.average_saliency); }
//bool dense_points_all_sort_function(DensePoints i, DensePoints j)  { return (i.saliency>j.saliency); }
  
} //AttentionModule
