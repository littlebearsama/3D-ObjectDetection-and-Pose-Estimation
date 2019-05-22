/**
 *  Copyright (C) 2012  
 *    Andreas Richtsfeld, Johann Prankl, Thomas Mörwald
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1170 Vienn, Austria
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
 * @file Vs3ArcRelations.cpp
 * @author Richtsfeld
 * @date March 2013
 * @version 0.1
 * @brief Relations based on arc groupings from canny edges.
 */

#include "Vs3ArcRelations.h"

namespace surface
{
  
/************************************************************************************
 * Constructor/Destructor
 */

Vs3ArcRelations::Vs3ArcRelations()
{
  have_input_image = false;
  have_view = false;
  preprocessed = false;
}

Vs3ArcRelations::~Vs3ArcRelations()
{
}

// ================================= Private functions ================================= //



// ================================= Public functions ================================= //

void Vs3ArcRelations::setInputImage(cv::Mat &_matImage)
{
  matImage = _matImage;
  cv::Mat gray_image;
  double lowThreshold = 5.;
  double highThreshold = 140.;
  int kernel_size = 3;
  
  cv::cvtColor(matImage, gray_image, CV_BGR2GRAY );                             /// TODO Berechnung wird auch in texture gemacht!!!
  cv::blur(gray_image, edges_image, cv::Size(3,3));
  cv::Canny(edges_image, edges_image, lowThreshold, highThreshold, kernel_size);
  
  have_input_image = true;
}


void Vs3ArcRelations::setView(surface::View *_view)
{
  view = _view;
  have_view = true;
}

void Vs3ArcRelations::preprocess()
{
  if(!have_input_image || !have_view) {
    printf("[Vs3ArcRelations::preprocess] Error: Missing input image or view. Exit\n");
    exit(-1);
  }
  
  // welche Informationen brauche ich um die Segmente in vs3 reinzustopfen
  // - Die beiden Nachbarn: Edge->surface[0/1]
  // - Die Position der canny edges
  
  // Ich gehe alle depth-edges aus dem surface model durch und kopiere die
  
//   cv::Mat contours;
//   unsigned radius = 3;
//   for(unsigned i=0; i<view->surfaces.size(); i++) {
//     for(unsigned j=0; j<view->surfaces[i].contours.size(); j++) {
//       for(unsigned k=0; k< view->surfaces[i].contours[j].size(); k++) {
//         // copy all edges which are on contours.
//         unsigned x = X(view->surfaces[i].contours[j][k]);
//         unsigned y = Y(view->surfaces[i].contours[j][k]);
//       }
//     }
//   }
  
  preprocessed = true;
}

// void Vs3ArcRelations::(int _id_0, int _id_1)
// {
//   // get relation between surface with id_0 and id_1
// }



} // end surface models





