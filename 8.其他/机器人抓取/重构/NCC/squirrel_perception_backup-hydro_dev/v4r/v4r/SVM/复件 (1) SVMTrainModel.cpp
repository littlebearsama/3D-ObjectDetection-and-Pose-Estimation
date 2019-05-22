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
 * @file SVMTrainModel.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Trains svm.
 */


#include "SVMTrainModel.h"
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

namespace svm {

static char *line = NULL;
static int max_line_len;

void print_null(const char *s) {}

void exit_input_error(int line_num)
{
  fprintf(stderr,"Wrong input format at line %d\n", line_num);
  exit(1);
}

static char* readline(FILE *input)
{
  int len;
  
  if(fgets(line,max_line_len,input) == NULL)
    return NULL;
  
  while(strrchr(line,'\n') == NULL)
  {
    max_line_len *= 2;
    line = (char *) realloc(line,max_line_len);
    len = (int) strlen(line);
    if(fgets(line+len,max_line_len-len,input) == NULL)
      break;
  }
  return line;
}

SVMTrainModel::SVMTrainModel()
{
  // default values
  param.svm_type = C_SVC;
  param.kernel_type = RBF;
  param.degree = 3;
  param.gamma = 0;        // 1/num_features
  param.coef0 = 0;
  param.nu = 0.5;
  param.cache_size = 100;
  param.C = 1;
  param.eps = 1e-3;
  param.p = 0.1;
  param.shrinking = 1;
  param.probability = 0;
  param.nr_weight = 0;
  param.weight_label = NULL;
  param.weight = NULL;
  cross_validation = 0;

  void (*print_func)(const char*) = NULL;       // default printing to stdout
  svm_set_print_string_function(print_func);

  have_input_file_name = false;
  have_model_file_name = false;
}

void SVMTrainModel::setSVMType(int _svm_type)
{
  param.svm_type = _svm_type;
}

void SVMTrainModel::setKernelType(int _kernel_type)
{
  param.kernel_type = _kernel_type;
}

void SVMTrainModel::setDegree(int _degree)
{
  param.degree = _degree;
}

void SVMTrainModel::setGamma(double _gamma)
{
  param.gamma = _gamma;
}

void SVMTrainModel::setCoef0(double _coef0)
{
  param.coef0 = _coef0;
}

void SVMTrainModel::setNu(double _nu)
{
  param.nu = _nu;
}

void SVMTrainModel::setCacheSize(double _cache_size)
{
  param.cache_size = _cache_size;
}

void SVMTrainModel::setC(double _C)
{
  param.C = _C;
}

void SVMTrainModel::setEps(double _eps)
{
  param.eps = _eps;
}

void SVMTrainModel::setP(double _p)
{
  param.p = _p;
}

void SVMTrainModel::setShrinking(int _shrinking)
{
  param.shrinking = _shrinking;
}

void SVMTrainModel::setProbability(int _probability)
{
  param.probability = _probability;
}

void SVMTrainModel::setCrossValidation(int _nr_fold)
{
  cross_validation = 0;
  if(_nr_fold > 0)
  {
    cross_validation = 1;
    nr_fold = _nr_fold;
    
  }
}

//for class i weight _weight
void SVMTrainModel::setWeight(int _i, float _weight)
{
  ++param.nr_weight;
  param.weight_label = (int *)realloc(param.weight_label,sizeof(int)*param.nr_weight);
  param.weight = (double *)realloc(param.weight,sizeof(double)*param.nr_weight);
  param.weight_label[param.nr_weight-1] = _i;
  param.weight[param.nr_weight-1] = _weight;
}
  
void SVMTrainModel::setInputFileName(std::string _input_file_name)
{
  have_input_file_name = true;
  std::strcpy (input_file_name, _input_file_name.c_str());
  //input_file_name = _input_file_name.c_str();
}

void SVMTrainModel::setModelFileName(std::string _model_file_name)
{
  have_model_file_name = true;
  std::strcpy (model_file_name, _model_file_name.c_str());
  //model_file_name = _model_file_name.c_str();
}

void SVMTrainModel::setNoPrint(bool _no_print)
{
  if(_no_print)
  {
    void (*print_func)(const char*) = &print_null;
    svm_set_print_string_function(print_func);
  }
  else
  {
    void (*print_func)(const char*) = NULL;       // default printing to stdout
    svm_set_print_string_function(print_func);
  }
}

int SVMTrainModel::train(double &RecRate, std::vector<int> &ConfusionTable)
{ 

  if((!have_input_file_name) || (!have_model_file_name))
  {
    fprintf(stderr,"ERROR: Set Input and Model files first!\n");
    exit(1);
  }

  const char *error_msg;
  
  readProblem(input_file_name);
  error_msg = svm_check_parameter(&prob,&param);
  
  if(error_msg)
  {
    fprintf(stderr,"ERROR: %s\n",error_msg);
    exit(1);
  }
  
  if(cross_validation)
  {
    do_cross_validation(RecRate,ConfusionTable);
  }
  else
  {
    model = svm_train(&prob,&param);
    if(svm_save_model(model_file_name,model))
    {
      fprintf(stderr, "can't save model to file %s\n", model_file_name);
      exit(1);
    }
    svm_free_and_destroy_model(&model);
  }
  svm_destroy_param(&param);
  free(prob.y);
  free(prob.x);
  free(x_space);
  free(line);
  
  return 0;
}

void SVMTrainModel::do_cross_validation(double &RecRate, std::vector<int> &ConfusionTable)
{
  int i;
  int total_correct = 0;
  double total_error = 0;
  double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
  double *target = Malloc(double,prob.l);
  
  svm_cross_validation(&prob,&param,nr_fold,target);
  if(param.svm_type == EPSILON_SVR ||
    param.svm_type == NU_SVR)
  {
    for(i=0;i<prob.l;i++)
    {
      double y = prob.y[i];
      double v = target[i];
      total_error += (v-y)*(v-y);
      sumv += v;
      sumy += y;
      sumvv += v*v;
      sumyy += y*y;
      sumvy += v*y;
    }
    printf("Cross Validation Mean squared error = %g\n",total_error/prob.l);
    printf("Cross Validation Squared correlation coefficient = %g\n",
           ((prob.l*sumvy-sumv*sumy)*(prob.l*sumvy-sumv*sumy))/
           ((prob.l*sumvv-sumv*sumv)*(prob.l*sumyy-sumy*sumy))
    );
  }
  else
  {
    ConfusionTable.at(0) = 0; ConfusionTable.at(1) = 0; ConfusionTable.at(2) = 0; ConfusionTable.at(3) = 0;
    for(i=0;i<prob.l;i++)
    {
      if(target[i] == prob.y[i])
      {
        ++total_correct;
        if(target[i] == 0)
        {
          ConfusionTable.at(0) += 1;
        }
        else
        {
          ConfusionTable.at(3) += 1;
        }
      }
      else
      {
        if(target[i] == 0)
        {
          ConfusionTable.at(2) += 1;
        }
        else
        {
          ConfusionTable.at(1) += 1;
        }
      }
    }
      //printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/prob.l);
    RecRate = 100.0*total_correct/prob.l;
  }
  free(target);
}

// read in a problem (in svmlight format)
void SVMTrainModel::readProblem(const char *filename)
{
  int elements, max_index, inst_max_index, i, j;
  FILE *fp = fopen(filename,"r");
  char *endptr;
  char *idx, *val, *label;
  
  if(fp == NULL)
  {
    fprintf(stderr,"can't open input file %s\n",filename);
    exit(1);
  }
  
  prob.l = 0;
  elements = 0;
  
  max_line_len = 1024;
  line = Malloc(char,max_line_len);
  while(readline(fp)!=NULL)
  {
    char *p = strtok(line," \t"); // label
    
    // features
    while(1)
    {
      p = strtok(NULL," \t");
      if(p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
                                break;
      ++elements;
    }
    ++elements;
    ++prob.l;
  }
  rewind(fp);
  
  prob.y = Malloc(double,prob.l);
  prob.x = Malloc(struct svm_node *,prob.l);
  x_space = Malloc(struct svm_node,elements);
  
  max_index = 0;
  j=0;
  for(i=0;i<prob.l;i++)
  {
    inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
    readline(fp);
    prob.x[i] = &x_space[j];
    label = strtok(line," \t\n");
    if(label == NULL) // empty line
                        exit_input_error(i+1);
    
    prob.y[i] = strtod(label,&endptr);
    if(endptr == label || *endptr != '\0')
      exit_input_error(i+1);
    
    while(1)
    {
      idx = strtok(NULL,":");
      val = strtok(NULL," \t");
      
      if(val == NULL)
        break;
      
      errno = 0;
      x_space[j].index = (int) strtol(idx,&endptr,10);
      if(endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index)
        exit_input_error(i+1);
      else
        inst_max_index = x_space[j].index;
      
      errno = 0;
      x_space[j].value = strtod(val,&endptr);
      if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr)))
        exit_input_error(i+1);
      
      ++j;
    }
    
    if(inst_max_index > max_index)
      max_index = inst_max_index;
    x_space[j++].index = -1;
  }
  
  if(param.gamma == 0 && max_index > 0)
    param.gamma = 1.0/max_index;
  
  if(param.kernel_type == PRECOMPUTED)
    for(i=0;i<prob.l;i++)
    {
      if (prob.x[i][0].index != 0)
      {
        fprintf(stderr,"Wrong input format: first column must be 0:sample_serial_number\n");
        exit(1);
      }
      if ((int)prob.x[i][0].value <= 0 || (int)prob.x[i][0].value > max_index)
      {
        fprintf(stderr,"Wrong input format: sample_serial_number out of range\n");
        exit(1);
      }
    }
    
    fclose(fp);
}

} //namespace svm
