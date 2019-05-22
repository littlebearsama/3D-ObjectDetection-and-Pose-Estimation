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
 * @file SVMPredictorSingle.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */

#ifndef SVM_PREDICTOR_SINGLE_H
#define SVM_PREDICTOR_SINGLE_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <ostream>

#include "svm.h"
#include "v4r/SurfaceUtils/SurfaceModel.hpp"

namespace svm
{
  
  
/**
 * @brief Class SVMPredictorSingle: 
 */
class SVMPredictorSingle
{
 
public:
  
private:
  
  std::string model_filename;
  
  bool have_model_filename;
  bool have_model_node;
  bool have_relations;
  bool have_type;
  
  struct svm_model *model;                      ///< SVM model
  
  bool predict_probability;                     ///< Predict with probability values
  std::vector<surface::Relation> relations;
  int type;
  int max_nr_attr;                              ///< Maximum attributes = maximum size of feature vector
  struct svm_node *node;                        ///< node of svm
  bool scale;                                   ///< set scaling on/off
  double lower, upper;                          ///< lower/upper limits
  std::vector<double> feature_max;              ///< maximum feature value for scaling
  std::vector<double> feature_min;              ///< minimum feature value for scaling
  std::vector<bool> feature_scaled;
  
  void checkSmallPatches(unsigned int max_size);
  void scaleValues(std::vector<double> &val);
  
  bool have_surfaces;
  std::vector<surface::SurfaceModel::Ptr> surfaces;              ///< Surfaces
  
public:
  SVMPredictorSingle(); 
  ~SVMPredictorSingle();
  
  void setPredictProbability(bool _predict_probability) { predict_probability = _predict_probability; };
  void setModelFilename(std::string _model_filename);
  void setRelations(std::vector<surface::Relation> _relations);
  /** Set surfaces **/
  void setSurfaces(const std::vector<surface::SurfaceModel::Ptr> _surfaces);
  void setType(int _type);
  /** Classification of all feature vectors of a view of a specific type (1=neighboring / 2=non-neighboring **/
  void compute();
  /** Set scaling of result vector **/
  void setScaling(bool _scale, std::string filename);
  
  /** Get modified relations **/
  inline std::vector<surface::Relation> getRelations();
  
  double predict(std::vector<double> &val, std::vector<double> &prob);
  
};

inline std::vector<surface::Relation> SVMPredictorSingle::getRelations()
{
  return relations;
}

}

#endif //SVM_PREDICTOR_SINGLE_H

