/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_POSE_IO_HPP
#define KP_POSE_IO_HPP

#ifdef Success
#undef Success
#endif

#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>


namespace kp
{

/**
 * writePose
 */
void writePose(const std::string &filename, const std::string &label, const Eigen::Matrix4f &pose)
{
  std::ofstream out(filename.c_str(), std::ios::out); //ios::app
  if (label.size()!=0) out<<label<<' ';
  for (unsigned v=0; v<4; v++)
    for (unsigned u=0; u<4; u++)
      out<<pose(v,u)<<' ';
  out.close();
}

/**
 * readPose
 */
bool readPose(const std::string &filename, std::string &label, Eigen::Matrix4f &pose)
{
  std::ifstream in(filename.c_str(), std::ios::in);
  if (in.is_open())
  {
    in>>label;
    for (unsigned v=0; v<4; v++)
      for (unsigned u=0; u<4; u++)
        in>>pose(v,u);
    in.close();
    return true;
  }
  return false;
}

}

#endif
