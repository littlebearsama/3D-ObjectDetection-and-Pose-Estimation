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


#include <opencv2/opencv.hpp>

#include "v4r/AttentionModule/AttentionModule.hpp"
#include "v4r/EPUtils/EPUtils.hpp"

// This program create strokes from 3D Symmetry Map

void printUsage(const char *argv0)
{
  printf(
    "Creates strokes from 3D Symmetry Map\n"
    "usage: %s saliency.png output.txt result.png\n"
    "  saliency.png          ... saliency image\n"
    "  points.txt            ... output text file with points\n"
    "  result_base.png            ... output file name\n", argv0);
  printf(" Example: %s saliency.png points.txt result.png\n",argv0);
}

int main(int argc, char** argv)
{
  if(argc != 4)
  {
    printUsage(argv[0]);
    return(0);
  }
  
  srand ( time(NULL) );
  
  std::string saliency_map_name(argv[1]);
  std::string attention_points_filename(argv[2]);
  std::string result_base(argv[3]);
    
  cv::Mat saliencyMap = cv::imread(saliency_map_name,-1);
  saliencyMap.convertTo(saliencyMap,CV_32F,1.0f/255);
  
  std::vector<cv::Point> attentionPoints;
  EPUtils::readAttentionPoints(attentionPoints,attention_points_filename);
    
  for(size_t at_p = 0; at_p < attentionPoints.size(); ++at_p)
  {
  
    float th = 0.1;
    std::vector<EPUtils::ConnectedComponent> connectedComponents;
    EPUtils::extractConnectedComponents(saliencyMap,connectedComponents,attentionPoints.at(at_p),th);
  
    cv::Mat final_mask = cv::Mat_<uchar>::zeros(saliencyMap.rows,saliencyMap.cols);
    
    assert( connectedComponents.size() == 1 );    
    for(unsigned int i = 0; i < connectedComponents.size(); ++ i)
    {  
      cv::Mat mask_temp = cv::Mat_<uchar>::zeros(saliencyMap.rows,saliencyMap.cols);
      EPUtils::drawConnectedComponent(connectedComponents.at(i),mask_temp,cv::Scalar(1));
   
      AttentionModule::SaliencyLine saliencyLineCurent;
      if(AttentionModule::extractSaliencyLine(mask_temp,saliencyMap,saliencyLineCurent,0))
      {
        std::vector<bool> usedPoints;
        AttentionModule::modifySymmetryLine(saliencyLineCurent,usedPoints,0.9);
      
        std::vector<AttentionModule::JunctionNode> points_from_symmetry_line = saliencyLineCurent.getPoints();
 
        cv::Mat temp_mask_for_symmetry_line = cv::Mat_<uchar>::zeros(saliencyMap.rows,saliencyMap.cols);
        for(size_t sym_pi = 0; sym_pi < points_from_symmetry_line.size(); ++sym_pi)
        {
	  if(usedPoints.at(sym_pi))
	  {
	    temp_mask_for_symmetry_line.at<uchar>(points_from_symmetry_line.at(sym_pi).y,points_from_symmetry_line.at(sym_pi).x) = 255;
	  }
        }     
        
        cv::Mat temp_mask_for_symmetry_line2;
        cv::dilate(temp_mask_for_symmetry_line,temp_mask_for_symmetry_line2,cv::Mat());
	
	int min_r = temp_mask_for_symmetry_line2.rows;
	int max_r = 0;
	
	int min_c = temp_mask_for_symmetry_line2.cols;
	int max_c = 0;
	
        for(int r_idx = 0; r_idx < temp_mask_for_symmetry_line2.rows; ++r_idx)
        {
	  for(int c_idx = 0; c_idx < temp_mask_for_symmetry_line2.cols; ++c_idx)
	  {
	    
	    if(temp_mask_for_symmetry_line2.at<uchar>(r_idx,c_idx))
	    {
	      final_mask.at<uchar>(r_idx,c_idx) = 1;
// 	      final_mask.at<uchar>(r_idx,3*c_idx+1) = 255;
// 	      final_mask.at<uchar>(r_idx,3*c_idx+2) = 255;
	      
	      if(r_idx > max_r)
		max_r = r_idx;
	      
	      if(r_idx < min_r)
		min_r = r_idx;
	      
	      if(c_idx > max_c)
		max_c = c_idx;
	      
	      if(c_idx < min_c)
		min_c = c_idx;
	    }
	  }
        }
        
        // detect the bounding box
        //std::cerr << min_r << std::endl;
        min_r = min_r - 0.5*(max_r-min_r);
	min_r = (min_r < 0 ? 0 : min_r);
	//std::cerr << min_r << std::endl;
	
	max_r = max_r + 0.5*(max_r-min_r);
	max_r = (max_r >= temp_mask_for_symmetry_line2.rows ? temp_mask_for_symmetry_line2.rows : max_r);
	
	min_c = min_c - 0.5*(max_c-min_c);
	min_c = (min_c < 0 ? 0 : min_c);
	
	max_c = max_c + 0.5*(max_c-min_c);
	max_c = (max_c >= temp_mask_for_symmetry_line2.cols ? temp_mask_for_symmetry_line2.cols : max_c);
	
	cv::Mat temp_final = cv::Mat_<uchar>::zeros(saliencyMap.rows,saliencyMap.cols);
	for(int r_idx = min_r; r_idx < max_r; ++r_idx)
        {
	  for(int c_idx = min_c; c_idx < max_c; ++c_idx)
	  {
	    temp_final.at<uchar>(r_idx,c_idx) = 255;
	  }
	}
	
// 	cv::imshow("temp_final",temp_final);
 
        bool haveBG = false;
        
        for(int r_idx = 0.1*(temp_mask_for_symmetry_line2.rows)-1; r_idx <= 0.1*(temp_mask_for_symmetry_line2.rows)+1; ++r_idx)
	{
	  for(int c_idx = 0.1*(temp_mask_for_symmetry_line2.cols); c_idx <= 0.9*(temp_mask_for_symmetry_line2.cols); ++c_idx)
	  {
	    if(temp_final.at<uchar>(r_idx,c_idx) > 0)
	      continue;
	    final_mask.at<uchar>(r_idx,c_idx) = 2;
	    haveBG = true;
	  }
	}
	
	for(int r_idx = 0.9*(temp_mask_for_symmetry_line2.rows)-1; r_idx <= 0.9*(temp_mask_for_symmetry_line2.rows)+1; ++r_idx)
	{
	  for(int c_idx = 0.1*(temp_mask_for_symmetry_line2.cols); c_idx <= 0.9*(temp_mask_for_symmetry_line2.cols); ++c_idx)
	  {
	    if(temp_final.at<uchar>(r_idx,c_idx) > 0)
	      continue;
	    final_mask.at<uchar>(r_idx,c_idx) = 2;
	    haveBG = true;
	  }
	}
	
	for(int r_idx = 0.1*(temp_mask_for_symmetry_line2.rows); r_idx <= 0.9*(temp_mask_for_symmetry_line2.rows); ++r_idx)
	{
	  for(int c_idx = 0.1*(temp_mask_for_symmetry_line2.cols)-1; c_idx <= 0.1*(temp_mask_for_symmetry_line2.cols)+1; ++c_idx)
	  {
	    if(temp_final.at<uchar>(r_idx,c_idx) == 0)
	      final_mask.at<uchar>(r_idx,c_idx) = 2;
	    haveBG = true;
	  }
	}
	
	for(int r_idx = 0.1*(temp_mask_for_symmetry_line2.rows); r_idx <= 0.9*(temp_mask_for_symmetry_line2.rows); ++r_idx)
	{
	  for(int c_idx = 0.9*(temp_mask_for_symmetry_line2.cols)-1; c_idx <= 0.9*(temp_mask_for_symmetry_line2.cols)+1; ++c_idx)
	  {
	    if(temp_final.at<uchar>(r_idx,c_idx) == 0)
	      final_mask.at<uchar>(r_idx,c_idx) = 2;
	      haveBG = true;
	  }
	}
	
      if(!haveBG)
      {
	for(int r_idx = 0.05*(temp_mask_for_symmetry_line2.rows)-1; r_idx <= 0.05*(temp_mask_for_symmetry_line2.rows)+1; ++r_idx)
	{
	  for(int c_idx = 0.05*(temp_mask_for_symmetry_line2.cols); c_idx <= 0.95*(temp_mask_for_symmetry_line2.cols); ++c_idx)
	  {
// 	    if(temp_final.at<uchar>(r_idx,c_idx) > 0)
// 	      continue;
	    final_mask.at<uchar>(r_idx,c_idx) = 2;
	    haveBG = true;
	  }
	}
	
	for(int r_idx = 0.95*(temp_mask_for_symmetry_line2.rows)-1; r_idx <= 0.95*(temp_mask_for_symmetry_line2.rows)+1; ++r_idx)
	{
	  for(int c_idx = 0.05*(temp_mask_for_symmetry_line2.cols); c_idx <= 0.95*(temp_mask_for_symmetry_line2.cols); ++c_idx)
	  {
// 	    if(temp_final.at<uchar>(r_idx,c_idx) > 0)
// 	      continue;
	    final_mask.at<uchar>(r_idx,c_idx) = 2;
	    haveBG = true;
	  }
	}
	
	for(int r_idx = 0.05*(temp_mask_for_symmetry_line2.rows); r_idx <= 0.95*(temp_mask_for_symmetry_line2.rows); ++r_idx)
	{
	  for(int c_idx = 0.05*(temp_mask_for_symmetry_line2.cols)-1; c_idx <= 0.05*(temp_mask_for_symmetry_line2.cols)+1; ++c_idx)
	  {
// 	    if(temp_final.at<uchar>(r_idx,c_idx) == 0)
	      final_mask.at<uchar>(r_idx,c_idx) = 2;
	    haveBG = true;
	  }
	}
	
	for(int r_idx = 0.05*(temp_mask_for_symmetry_line2.rows); r_idx <= 0.95*(temp_mask_for_symmetry_line2.rows); ++r_idx)
	{
	  for(int c_idx = 0.95*(temp_mask_for_symmetry_line2.cols)-1; c_idx <= 0.95*(temp_mask_for_symmetry_line2.cols)+1; ++c_idx)
	  {
// 	    if(temp_final.at<uchar>(r_idx,c_idx) == 0)
	      final_mask.at<uchar>(r_idx,c_idx) = 2;
	      haveBG = true;
	  }
	}
      }
      }
      
      char temp[50];
      sprintf(temp,"%d",at_p);
    
      std::string current_mask_name = result_base + ".png_object_" + temp + ".png";
      cv::imwrite(current_mask_name,final_mask);
        
        //cv::imshow("final_mask",final_mask);
        //cv::waitKey(-1);
      
    }
  }
}
