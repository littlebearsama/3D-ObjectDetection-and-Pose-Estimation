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
 * @file SVMPredictorSingle.cpp
 * @author Andreas Richtsfeld
 * @date August 2011
 * @version 0.1
 * @brief Predict results from a trained SVM.
 */

#include "SVMPredictorSingle.h"

namespace svm
{

#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))

  
/**
 * @brief Constructor of SVMPredictorSingle
 */
SVMPredictorSingle::SVMPredictorSingle()
{
  have_model_filename = false;
  have_model_node = false;
  have_relations = false;
  predict_probability = true;
  have_type = false;
  max_nr_attr = 64;
  scale = false;
  lower = -1.;
  upper = 1.;
}

/**
 * @brief Destructor of SVMPredictorSingle
 */
SVMPredictorSingle::~SVMPredictorSingle()
{
  if(have_model_filename)
    svm_free_and_destroy_model(&model);
  if(have_model_node)
    free(node);
}

void SVMPredictorSingle::setSurfaces(const std::vector<surface::SurfaceModel::Ptr> _surfaces)
{
  surfaces = _surfaces;
  have_surfaces = true;
}

void SVMPredictorSingle::setModelFilename(std::string _model_filename) 
{ 
  model_filename = _model_filename; 
  
  // allocate memory for model (and nodes)
  if((model=svm_load_model(model_filename.c_str()))==0)
  {
    fprintf(stderr,"can't open model file %s\n",model_filename.c_str());
    exit(1);
  }
  
  have_model_filename = true; 
  
  node = (struct svm_node *) malloc(max_nr_attr*sizeof(struct svm_node));
  if(predict_probability) {
    if(svm_check_probability_model(model) == 0) {
      printf("[SVMPredictorSingle::SVMPredictorSingle] Error: Model does not support probability estimates.\n");
      return;
    }
  }
  else {
    if(svm_check_probability_model(model) != 0)
      printf("[SVMPredictorSingle::SVMPredictorSingle] Warning: Model supports probability estimates, but disabled in prediction.");
  }
  
  have_model_node = true;
}

void SVMPredictorSingle::setRelations(std::vector<surface::Relation> _relations)
{
//   std::cerr << "relations 3 " << _relations.size() << std::endl;
  relations = _relations;
//   std::cerr << "relations 4 " << relations.size() << std::endl;
  have_relations = true;
}

void SVMPredictorSingle::setType(int _type)
{
  type = _type;
  have_type = true;
}

/** 
 * @brief Classify all feature vectors of a view
 * @param view View with surface models 
 * @param type Type of relations to process (1=structural / 2= assembly level)
 */
void SVMPredictorSingle::compute()
{
  if(!have_model_filename)
  {
    fprintf(stderr,"do not have model file\n");
    exit(1);
  }
  
  if(!have_model_node)
  {
    fprintf(stderr,"do not have model node\n");
    exit(1);
  }
  
  if(!have_surfaces)
  {
    fprintf(stderr,"do not have surfaces\n");
    exit(1);
  }
  
  if(!have_relations)
  {
    fprintf(stderr,"do not have relations\n");
    exit(1);
  }
  
  if(!have_type)
  {
    fprintf(stderr,"do not know what type (structural, assembly) of relationships I need to classify\n");
    exit(1);
  }
  
  //@ep: reallocate node structure according to the size of the feature vector
  
  for(unsigned int i = 0; i < relations.size(); i++) 
  {
//     if(!(relations.at(i).valid))
//       continue;
   
    if(relations.at(i).type == type)
    {
//       std::cerr << "relations.rel_value " << relations.at(i).rel_value.size() << std::endl;
      relations.at(i).prediction = predict(relations.at(i).rel_value,relations.at(i).rel_probability);
    }
  } 
  
//   std::cerr << "relations " << relations.size() << std::endl;
  
  //@ep: this function seems to be wrong to me
  //checkSmallPatches(30);
	
}

/**
 * @brief Process the relation extraction algorithm
 * @param type Type of SVM relation
 * @param val Feature vector of relation
 * @param prob Probability for correct predicton
 */
double SVMPredictorSingle::predict(std::vector<double> &val, std::vector<double> &prob)
{
  if(scale)
    scaleValues(val);
  
  int svm_type = svm_get_svm_type(model);
  int nr_class = svm_get_nr_class(model);
  double *prob_estimates=NULL;
  prob.clear();

  if(predict_probability) 
  {
    if (svm_type == NU_SVR || svm_type == EPSILON_SVR)
      printf("Prob. model for test data: target value = predicted value + z,\nz: Laplace distribution e^(-|z|/sigma)/(2sigma),sigma=%g\n", svm_get_svr_probability(model));
    else
      prob_estimates = (double *) malloc(nr_class*sizeof(double));
  }
  
  // we copy now the feature vector
  //@ep: BUG in case the number of attributes is bigger than 63 we will run into a bug
  double predict_label;
  for(unsigned idx = 0; idx < val.size(); idx++) 
  {
    node[idx].index = idx+1;
    node[idx].value = val.at(idx);
    node[idx+1].index = -1;
  }

  if (predict_probability && (svm_type==C_SVC || svm_type==NU_SVC)) 
  {
    predict_label = svm_predict_probability(model, node, prob_estimates);
    prob.resize(nr_class);
    for(int j=0; j <nr_class; j++)
      prob.at(j) = prob_estimates[j];
  }
  else
  {
    predict_label = svm_predict(model, node);
  }
  
  return predict_label;
}


/**
 * @brief Set scaling for structural level with supplied model.
 * @param _scale on/off
 */
void SVMPredictorSingle::setScaling(bool _scale, std::string filename)
{
  scale = _scale;
  if(!scale)
    return;
  
  printf("Scaling values...\n");
  
  /* pass 1: find out max index of attributes */
  int max_index = 0;
  //int min_index = 1;

  std::ifstream fp1(filename.c_str());
  if( fp1.is_open() )
  {
    std::string line;
    getline(fp1,line);
    getline(fp1,line);
    std::stringstream ss;
    ss << line;
    ss >> lower >> upper;
    
    while( fp1.good() )
    {
      std::string line2;
      getline(fp1,line2);
      
      if(line2.empty())
	break;
      
      std::stringstream ss2;
      ss2 << line2;
      
      int index;
      double min_value, max_value;
      ss2 >> index >> min_value >> max_value;
      
      if(index > max_index)
	max_index = index;
    }
    fp1.close();
  }
  else
  {
    scale = false;
    return;
  }
  
  feature_max.resize(max_index+1);
  feature_min.resize(max_index+1);
  feature_scaled.resize(max_index+1);
  
  for(int i = 0; i <= max_index; ++i)
  {
    feature_min.at(i) = 0;
    feature_max.at(i) = 0;
    feature_scaled.at(i) = false;
  }
  
  std::ifstream fp2(filename.c_str());
  if( fp2.is_open() )
  {
    std::string line;
    getline(fp2,line);
    getline(fp2,line);
    
    while( fp2.good() )
    {
      std::string line2;
      getline(fp2,line2);
      
      if(line2.empty())
	break;
      
      std::stringstream ss;
      ss << line2;
      
      int index;
      double min_value, max_value;
      ss >> index >> min_value >> max_value;
      
      feature_min.at(index) = min_value;
      feature_max.at(index) = max_value;
      feature_scaled.at(index) = true;
    }
    fp2.close();
  }
  else
  {
    scale = false;
    return;
  }
  
}

void SVMPredictorSingle::scaleValues(std::vector<double> &val)
{
  if((val.size()+1) != feature_min.size())
  {
    printf("SVMPredictorSingle::scaleValues val.size %d != feature_min.size %d\n", (int)val.size(), (int)feature_min.size());
    return;
  }
  
  for(unsigned index=0; index<val.size(); index++)
  {
    if(!feature_scaled.at(index+1))
    {
      continue;
    }
    
    double value = val.at(index);
    if(fabs(feature_max.at(index+1)-feature_min.at(index+1)) < 0.000001) 
    {
      printf("[SVMPredictorSingle::scaleValues] Warning: feature_maxat(index+1) ~= feature_minat(index+1): %4.3f\n", feature_max.at(index+1));
      return;
    }

    if(fabs(value-feature_min.at(index+1)) < 0.000001)
      value = lower;
    else if(fabs(value-feature_max.at(index+1)) < 0.000001)
      value = upper;
    else
      value = lower + (upper-lower) * (value-feature_min.at(index+1))/(feature_max.at(index+1)-feature_min.at(index+1));

    val.at(index) = value;
  }
}

/** HACK: We do not allow small patches to be connected to two big patches. **/
void SVMPredictorSingle::checkSmallPatches(unsigned int max_size)
{
  for(unsigned int i = 0; i < surfaces.size(); i++) 
  {
    if(surfaces.at(i)->indices.size() < max_size) 
    {
      int biggest = -1;
      double biggest_value = 0.0;
      for(unsigned int j = 0; j < relations.size(); j++) 
      {
        if(relations.at(j).id_0 == (int)i || relations.at(j).id_1 == (int)i) 
	{
          //@ep: this works only because we assume that there are always two classes and that we are using model with probabilitites
	  //the probability that those patches are connected
	  //@ep: this is wrong, we basically always disconnect the first connection, whenever it is big or small
	  //@ep: and then we keep doing it, because probability can't be smaller than 0
	  //@ep: am i wrong here???
	  if(relations.at(j).rel_probability[1] < biggest_value) 
	  {
            relations.at(j).rel_probability[0] = 0.999;
            relations.at(j).rel_probability[1] = 0.001;
          }
          else 
	  {
            biggest_value = relations.at(j).rel_probability[1];
            if(biggest != -1) 
	    {
              relations[biggest].rel_probability[0] = 0.999;
              relations[biggest].rel_probability[1] = 0.001;
            }
            biggest = j;
          }
        }
      }
    }
  }      
}

} 











