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


#ifndef EPDRAWUTILS_H
#define EPDRAWUTILS_H

#include "headers.hpp"
#include "connectedComponents.hpp"

namespace EPUtils
{

/**
 * draw one segmentation masks
 * */
void drawSegmentationMask(cv::Mat &image, cv::Mat mask, cv::Scalar color, int line_width = 2);
  
/**
 * draw a banch of segmentation masks
 * */
void drawSegmentationMasks(cv::Mat &image, std::vector<cv::Mat> &masks, int line_width = 2);

/**
 * draw a segmentation masks and attetnion points
 * */
void drawSegmentationResults(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                             std::vector<std::vector<cv::Point> > &contours, bool drawAttentionPoints = true,
                             bool drawSegmentationResults = true, bool drawLines = false, unsigned int num = -1);

/**
 * draws segmentation and attention results
 * */
void drawSegmentationResults(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                             std::vector<cv::Mat> &binMasks, std::vector<std::vector<cv::Point> > &contours,
                             bool drawAttentionPoints, bool drawSegmentationResults);
void drawSegmentationResults(cv::Mat &image, cv::Point p1, cv::Mat &masks, bool drawAttentionPoints, bool drawSegmentationResults, int num);

//revision
/**
 * draws attention points
 * */
void drawAttentionPoints(cv::Mat &image, std::vector<cv::Point> &attentionPoints,
                         unsigned int maxNumber = 0, bool connect_points = false);
//end revision

/**
 * draws path through the map
 */
void drawPath(cv::Mat &image, std::vector<cv::Point> &path, cv::Mat &mapx, cv::Mat &mapy);

/**
 * draws line
 */
void drawLine(cv::Mat &image, std::vector<cv::Point> points, cv::Scalar color = cv::Scalar(0));

} //namespace EPUtils

#endif //EPDRAWUTILS_H