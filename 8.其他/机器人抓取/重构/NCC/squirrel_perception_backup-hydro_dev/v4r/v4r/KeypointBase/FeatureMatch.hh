/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_FEATURE_MATCH_HH
#define KP_FEATURE_MATCH_HH

#include <opencv2/core/core.hpp>

namespace kp 
{

class FeatureMatch
{
public:
  int partIdx;
  int featureIdx;
  cv::Point2f query_pt;

  union
  {
    float distance;
    float probability;
  };
  FeatureMatch() {}
  FeatureMatch(int _partIdx, int _featureIdx, const cv::Point2f &_query_pt, float _distance) 
   : partIdx(_partIdx), featureIdx(_featureIdx), query_pt(_query_pt), distance(_distance) {}
  virtual ~FeatureMatch(){}
};

}

#endif

