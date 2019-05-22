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


#ifndef EPCONNECTEDCOMPONENTS_H
#define EPCONNECTEDCOMPONENTS_H

#include "headers.hpp"

namespace EPUtils
{
  
struct ConnectedComponent {
  std::vector<cv::Point> points;
  std::vector<float> saliency_values;
  float average_saliency;
  ConnectedComponent();
};
/**
 * extracts connected components from the map using given threshold
 * map is considered to be in the range 0..1 with type CV_32F
 * */
void extractConnectedComponents(cv::Mat map, std::vector<ConnectedComponent> &connected_components, float th = 0.1);
void extractConnectedComponents(cv::Mat map, std::vector<ConnectedComponent> &connected_components, cv::Point attention_point, float th = 0.1);

/**
 * extracts connected components from the map using given threshold
 * map is considered to be in the range 0..1 with type CV_32F
 * */
//void extractConnectedComponents2(cv::Mat map, std::vector<ConnectedComponent> &connected_components, float th = 0.1);

/**
 * draws single connected component over the image
 */
void drawConnectedComponent(ConnectedComponent component, cv::Mat &image, cv::Scalar color);

/**
 * draws connected components over the image
 */
void drawConnectedComponents(std::vector<ConnectedComponent> components, cv::Mat &image, cv::Scalar color);

} //namespace EPUtils

#endif // EPCONNECTEDCOMPONENTS_H