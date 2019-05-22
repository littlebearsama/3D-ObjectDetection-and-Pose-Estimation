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
 * @file SVMScale.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Scales features.
 */


#ifndef SVM_SCALE_H
#define SVM_SCALE_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>

namespace svm
{
  
  
/**
 * @brief Class SVMScale: 
 */
class SVMScale
{
 
public:
  
private:

  double lower, upper;
  std::string save_filename;
  std::string features_filename;
  std::string features_scaled_filename;
  bool have_save_filename;
  bool have_features_filename;
  bool have_features_scaled_filename;
  
  std::vector<double> feature_max;
  std::vector<double> feature_min;
  
  void scale(int index, double &value);
  
public:
  SVMScale(); 
  ~SVMScale();
  
  void setLower(double _lower) {lower = _lower;};
  void setUpper(double _upper) {upper = _upper;};
  void setSaveFileName(std::string _save_filename) {save_filename = _save_filename; have_save_filename = true;};
  void setFeaturesFileName(std::string _features_filename) {features_filename = _features_filename; have_features_filename = true;};
  void setFeaturesScaledFileName(std::string _features_scaled_filename) {features_scaled_filename = _features_scaled_filename; have_features_scaled_filename = true;};
  
  void compute();
};

}

#endif //SVM_SCALE_H