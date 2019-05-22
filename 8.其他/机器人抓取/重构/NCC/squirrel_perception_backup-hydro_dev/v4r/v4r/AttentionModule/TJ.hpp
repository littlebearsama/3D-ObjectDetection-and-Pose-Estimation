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

#ifndef TJ_HPP
#define TJ_HPP

#include "headers.hpp"

namespace AttentionModule
{

extern int dy8[8];
extern int dx8[8];

extern int dx4[4];
extern int dy4[4];
  
enum PointJunctionType {
  UNKNOWN       = 0,
  T_JUNCTION    = 3,
  END_POINT     = 1,
  REGULAR_POINT = 2,
};

struct JunctionNode {
  int num;
  int x,y;
  std::vector<int> edges;
  int edges_num;
  int type;
  float saliency;
  JunctionNode()
  {
    edges_num = 0;
    type = UNKNOWN;
  }
};

class SaliencyLine
{
private:
  std::vector<JunctionNode> points;
  int points_num;
  float saliency;
public:
  SaliencyLine();
  void clear();
  float getSaliency();
  std::vector<JunctionNode> getPoints();
  void addPoint(JunctionNode node);
  int getPointsNumber();
};

struct PointSaliency {
  cv::Point point;
  float saliency;
};

bool calculateSaliencyLine(cv::Mat mask, const cv::Mat symmetry, SaliencyLine &saliencyLine, unsigned int th = 10);
bool findTJunctions(SaliencyLine saliencyLine, std::vector<int> &tjunctionPoints);
std::vector<int> findEndPoints(SaliencyLine saliencyLine, std::vector<int> segment);
std::vector<int> findEndPoints(SaliencyLine saliencyLine);
std::vector<int> getEdges(std::vector<JunctionNode> nodes,int nodeIdx);
void breakIntoSegments(SaliencyLine saliencyLine, std::vector<std::vector<int> > &segments);
void modifySymmetryLine(SaliencyLine saliencyLine, std::vector<bool> &usedPoints, float th = 0.5);
void selectSaliencyCenterPoint(SaliencyLine saliencyLine, PointSaliency &center);
void createSimpleLine(SaliencyLine saliencyLine, std::vector<cv::Point> &points);
bool extractSaliencyLine(cv::Mat mask, cv::Mat map, SaliencyLine &saliencyLine, unsigned int th = 10);
void createAttentionPoints(std::vector<PointSaliency> saliencyPoints, std::vector<cv::Point> &attentionPoints);

inline bool saliencyPointsSort (PointSaliency p1, PointSaliency p2) { return (p1.saliency>p2.saliency); }

} //namespace AttentionModule

#endif //TJ_HPP
