/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */


#include "Feature.hh"




namespace kp 
{


using namespace std;

unsigned Feature::nbcnt=0;

/************************************************************************************
 * Constructor/Destructor
 */
Feature::Feature(FeatureDetector::Type _type, int _view, int _num, int _octave)
 : type(_type), view(_view), num(_num), octave(_octave), nb(0)
{
}

Feature::~Feature()
{
}

/***************************************************************************************/

/**
 * write
 */
void Feature::write(std::ofstream &os, const Feature &feature)
{
  int tmp;

  os.write((char*)&feature.type, sizeof(FeatureDetector::Type) );
  os.write((char*)&feature.view, sizeof(int));
  os.write((char*)&feature.num, sizeof(int));
  os.write((char*)&feature.octave, sizeof(int));
  os.write((char*)&feature.pt3[0], sizeof(Eigen::Vector3f));
  os.write((char*)&feature.normal[0], sizeof(Eigen::Vector3f));
  os.write((char*)&feature.vr[0], sizeof(Eigen::Vector3f));

  tmp = feature.desc.size();
  os.write((char*)&tmp, sizeof(int));
  os.write((char*)&feature.desc[0], sizeof(float)*tmp);
}

/**
 * read
 */
void Feature::read(std::ifstream &is, Feature &feature)
{
  int tmp;
  
  is.read((char*)&feature.type, sizeof( FeatureDetector::Type ) );
  is.read((char*)&feature.view, sizeof(int));
  is.read((char*)&feature.num, sizeof(int));
  is.read((char*)&feature.octave, sizeof(int));
  is.read((char*)&feature.pt3[0], sizeof(Eigen::Vector3f));
  is.read((char*)&feature.normal[0], sizeof(Eigen::Vector3f));
  is.read((char*)&feature.vr[0], sizeof(Eigen::Vector3f));

  is.read((char*)&tmp, sizeof(int));
  feature.desc.resize(tmp);
  is.read((char*)&feature.desc[0], sizeof(float)*tmp);
}




}












