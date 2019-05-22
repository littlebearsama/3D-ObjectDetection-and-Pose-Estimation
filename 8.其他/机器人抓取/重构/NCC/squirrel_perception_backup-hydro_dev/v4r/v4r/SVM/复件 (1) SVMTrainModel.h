#ifndef SVM_TRAIN_MODEL_H
#define SVM_TRAIN_MODEL_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <string>
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
 * @file SVMTrainModel.h
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Trains svm.
 */

#include <vector>
#include <cstring>
#include "svm.h"

namespace svm
{
  
  
/**
 * @brief Class SVMTrainModel: 
 */
class SVMTrainModel
{
 
public:
  
private:

  struct svm_parameter param;  // set by parse_command_line
  struct svm_problem prob;     // set by read_problem
  struct svm_model *model;
  struct svm_node *x_space;
  int cross_validation;
  int nr_fold;

  char input_file_name[1024];
  char model_file_name[1024];

  bool have_input_file_name;
  bool have_model_file_name;
  
public:
  SVMTrainModel(); 
  ~SVMTrainModel() {};
  
  void setSVMType(int _svm_type);
  void setKernelType(int _kernel_type);
  void setDegree(int _degree);
  void setGamma(double _gamma);
  void setCoef0(double _coef0);
  void setNu(double _nu);
  void setCacheSize(double _cache_size);
  void setC(double _C);
  void setEps(double _eps);
  void setP(double _p);
  void setShrinking(int _shrinking);
  void setProbability(int _probability);
  void setCrossValidation(int _nr_fold);
  void setWeight(int _i, float _weight);
  void setInputFileName(std::string _input_file_name);
  void setModelFileName(std::string _input_file_name);
  void setNoPrint(bool _no_print);
  int train(double &RecRate, std::vector<int> &ConfusionTable);

private:
  void readProblem(const char *filename);
  void do_cross_validation(double &RecRate, std::vector<int> &ConfusionTable);
  
};

}

#endif //SVM_TRAIN_MODEL_H