/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_HH
#define KP_FEATURE_HH


#include <iostream>
#include <fstream>
#include <float.h>
#include <vector>
#include <Eigen/Dense>
#include "FeatureDetector.hh"
#include "v4r/KeypointTools/SmartPtr.hpp"

namespace kp 
{

class Feature
{
public:
  FeatureDetector::Type type;
  int view;
  int num;
  int octave;
  
  unsigned nb;
  static unsigned nbcnt;

  EIGEN_ALIGN16 Eigen::Vector3f pt3;
  EIGEN_ALIGN16 Eigen::Vector3f normal;
  EIGEN_ALIGN16 Eigen::Vector3f vr;

  std::vector<float> desc;

  Feature(FeatureDetector::Type _type=FeatureDetector::UNDEF, int _view=0, int _num=0, int _octave=0);
  ~Feature();

  static void write(std::ofstream &os, const Feature &feature);
  static void read(std::ifstream &os, Feature &feature);

  typedef SmartPtr< ::kp::Feature> Ptr;
  typedef SmartPtr< ::kp::Feature const> ConstPtr;
};

typedef SmartPtr< std::vector<Feature> > FeaturesPtr;


/*************************** INLINE METHODES **************************/



} //--END--

#endif

