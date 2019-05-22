/**
 *  Copyright (C) 2012  
 *    Ekaterina Potapova
 *    Automation and Control Institute
 *    Vienna University of Technology
 *    Gusshausstraße 25-29
 *    1040 Vienna, Austria
 *    potapova(at)acin.tuwien.ac.at
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


#ifndef HITRATIO_HPP
#define HITRATIO_HPP

#include "headers.hpp"

namespace AttentionModule {

struct ObjectDistance{
  float distance;
  int attentionPointIdx;
};

struct PointEvaluation{
  float distance;
  int objectIdx;
  float hitRatio;
  float firstDistance;
  float bestDistance;
};

class AttentionPointsEvaluation
{
public:
  AttentionPointsEvaluation();
  ~AttentionPointsEvaluation();
  
  void setAttentionPoints(std::vector<cv::Point> attention_points_);
  std::vector<cv::Point> getAttentionPoints();
  
  void setMask(cv::Mat &mask_);
  bool getMask(cv::Mat &mask_);
  
  bool calculate();
  
  bool writeToFile(std::string file_name);
  
  std::vector<PointEvaluation> getEvaluatedPoints();
  
private:
  std::vector<cv::Point> attention_points;
  
  bool haveMask;
  cv::Mat mask;
  
  std::vector<PointEvaluation> evaluated_points;
  
  void calculateCenters();
  std::vector<cv::Point> centers;
  std::vector<float> maxDist2Center;
  void labeling2Mask(cv::Mat &mask_i, int maskNum);
  void hitRatio();
  
  void calculateDistance2objects();
};
  
} //namespace EPEvaluation

#endif //HITRATIO_HPP