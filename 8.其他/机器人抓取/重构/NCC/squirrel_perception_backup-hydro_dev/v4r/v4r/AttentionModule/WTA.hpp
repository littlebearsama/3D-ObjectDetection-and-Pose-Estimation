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


/**
 * @brief this implementation was inspired by http://www.saliencytoolbox.net
 */

#ifndef WTA_HPP
#define WTA_HPP

#include "headers.hpp"

namespace AttentionModule
{

struct LIF {
  float timeStep;  // time step for integration (in sec).
  float Eleak;     // leak potential (in V).
  float Eexc;      // potential for excitatory channels (positive, in V).
  float Einh;      // potential for inhibitory channels (negative, in V).
  float Gleak;     // leak conductivity (in S).
  float Gexc;      // conductivity of excitatory channels (in S).
  cv::Mat Ginh;    // conductivity of inhibitory channels (in S).
  float GinhDecay; // time constant for decay of inhibitory conductivity (in S).
  float Ginput;    // input conductivity (in S).
  float Vthresh;   // threshold potential for firing (in V).
  float C;         // capacity (in F).
  float time;      // current time (in sec).
  cv::Mat V;       // current membrane potential (in V) - can be an array for several neurons.
  cv::Mat I;       // current input current (in A) - can be an array for several neurons.
  float DoesFire;  // neuron can (1) or cannot (0) fire.
};

struct WTA {
  LIF sm;    // LIF neuron field for input from the saliency map.
  LIF exc;   // excitatory LIF neurons field.
  LIF inhib; // inhibitory inter-neuron.
};

struct Params
{
  float IORdecay;
  float smOutputRange;
  float noiseAmpl;
  float noiseConst;
  bool  useRandom;
  float foaSize;
  int   mapLevel;
  bool  useCentering;
  bool  useMorphologyOpenning;
};

void defaultLeakyIntFire(LIF& lif);
void defaultParams(Params& params);
void initializeWTA(WTA& wta, const cv::Mat& salmap, Params& salParams);
void evolveLeakyIntFire(LIF& lif, float t, cv::Mat& spikes);
void evolveWTA(WTA& wta, cv::Point& winner);
bool fastSegmentMap(cv::Mat& resultMap, cv::Mat& map, cv::Point& seedPoint, int& Number);
bool estimateShape(cv::Mat& binMap, cv::Mat& segmentedMap, cv::Mat& shapeMap, cv::Mat& salmap, cv::Point& winner, Params& params, cv::Mat &image);
void applyIOR(WTA& wta, cv::Point& winner, Params& params);
void diskIOR(WTA& wta, cv::Point& winner, Params& params);
//void shapeIOR(WTA& wta, cv::Point& winner, Params& params, cv::Mat& binaryMap, cv::Mat& binMap);
void winnerToImgCoords(cv::Point& win2, cv::Point& winner, Params& params, cv::Mat& img, const cv::Mat& _salmap);
void plotSalientLocation(cv::Point& win2, cv::Point& lastWinner, cv::Mat& img, Params& params, int pointNumber);
int  CalculateWTA(cv::Mat& img, cv::Mat& _salmap, std::vector<cv::Point>& attented_points, int AttentionPointsNumber, Params &params);
void PrintImage(const cv::Mat &img);
void UpdateWinner(cv::Mat &salmap, cv::Point &winner,Params& params);
void UpdateWinner2(cv::Mat &salmap, cv::Point &winner);

} //namespace AttentionModule

#endif //WTA_HPP