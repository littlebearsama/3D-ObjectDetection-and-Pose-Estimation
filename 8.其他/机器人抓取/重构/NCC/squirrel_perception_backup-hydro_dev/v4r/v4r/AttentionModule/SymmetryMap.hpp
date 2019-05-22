 // modified by Ekaterina Potapovas
 /* * * * * * * * * * * * * * * * * * * * * * * *
 *              Symmetry_map               	*
 *						*
 * Author:   Dominik Kohl			*
 * Email:    e0726126@sudent.tuwien.ac.at	*
 * Date:     15 June 2011 			*
 * Supervisor: Ekaterina Potapova		*
 *						*
 * Based on the Code from Gert Kootstra		*
 * and Niklas Bergstrom [1] and [2]		*
 * (http://www.csc.kth.se/~kootstra/) 		*
 *						*
 *						*
 * * * * * * * * * * * * * * * * * * * * * * * */


 /*
 * DESCRIPTION
 *
 * This is software for the bottom-up detection of unknown objects.
 * It is based on a symmetry-saliency map, as described in [1] and [2]
 *
 * Symmetry is an important Gestalt principle and can be used for
 * figure-ground segregation or to find the centerpoint of an object.
 *
 * The code works with ROS and OpenCV 2.1 or higher and IPP 7.0
 * To get the symmetry-map all usable OpenCV instructions are used
 * and with IPP for speed-optimisation.
 *
 *
 * REFERENCES
 *
 * [1] Kootstra, G., Bergstr&ouml;m, N., & Kragic, D. (2010). Using Symmetry to
 * Select Fixation Points for Segmentation. To be presented at the
 * International Conference on Pattern Recognition (ICPR), August 23-26,
 * 2010, Istanbul, Turkey.
 *
 * [2] Kootstra, G. & Schomaker, L.R.B. (2009) Using Symmetrical
 * Regions-of-Interest to Improve Visual SLAM. In: Proceedings of the
 * International Conference on Intelligent RObots and Systems (IROS),
 * pp. 930-935, Oct 11-15, 2009, St. Louis, USA. doi:
 * 10.1109/IROS.2009.5354402.
*/
 
#ifndef SYMMETRY_MAP_HPP
#define SYMMETRY_MAP_HPP

#include "BaseMap.hpp"

namespace AttentionModule
{

/**
* calculates symmetry saliency map
**/
  
class SymmetryMap: public BaseMap
{
  
public:
  SymmetryMap();//
  ~SymmetryMap();//
  
  void setR1(int R1_);//
  int getR1();//
  
  void setR2(int R2_);//
  int getR2();//
  
  void setS(int S_);//
  int getS();//
  
  virtual int calculate();
  virtual void reset();//
  virtual void print();//
  
private:
  int     R1;               //Set the smaler Box that woun't be calculated
  int     R2;               //Set the bigger Box
  int     S;                //Set the standard deviaion for the distance between the pixel
  
  cv::Mat pixelAngles;
  std::vector<float> distanceWeight;
  cv::Mat cosAr;
  int  cosArSize;
  
  void symmetryMap(cv::Mat &image_cur, int image_width, int image_height, int R1, int R2, int S, cv::Mat &map_cur);
  void initialize();
  
protected:  
  virtual int checkParameters();//
  virtual int calculatePyramidSimple();
  
  virtual int combinePyramid(BasePyramid::Ptr pyramid);
};

} //namespace AttentionModule

#endif //SYMMETRY_MAP_HPP