/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#include "LinkedKeypoint.hh"




namespace kp 
{


using namespace std;

const float LinkedKeypoint::NaN = std::numeric_limits<float>::quiet_NaN();
int LinkedKeypoint::nb_cnt = 0;

/************************************************************************************
 * Constructor/Destructor
 */
LinkedKeypoint::LinkedKeypoint()
  : tile_label(0), dist_tmp(FLT_MAX), nb(-1), pt3(Eigen::Vector3f(NaN,NaN,NaN)), normal(Eigen::Vector3f(NaN,NaN,NaN)), pt3_glob(-1)
{ 
  pt_pred = pt;
}

LinkedKeypoint::LinkedKeypoint(const cv::KeyPoint &_key, int _tile_label)
  : cv::KeyPoint(_key), tile_label(_tile_label), dist_tmp(FLT_MAX), nb(-1), pt3(Eigen::Vector3f(NaN,NaN,NaN)), pt3_glob(-1)
{
  pt_pred = pt;
}


LinkedKeypoint::~LinkedKeypoint()
{
}

/***************************************************************************************/





}












