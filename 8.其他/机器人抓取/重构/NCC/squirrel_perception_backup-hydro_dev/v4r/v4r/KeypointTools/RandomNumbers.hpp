/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_RANDOM_NUMBERS_HPP
#define KP_RANDOM_NUMBERS_HPP


namespace kp
{

inline bool contains(const std::vector<int> &idx, int num)
{
  for (unsigned i=0; i<idx.size(); i++)
    if (idx[i]==num)
      return true;
  return false;
}

/**
 * @brief Returns a pseudo random number in [0.0, 1.0]
 */
inline float frand()
{
  return rand()/((float)RAND_MAX + 1.);
}

inline float expPdf(float lambda)
{
  float dum;
  do
    dum = frand();
  while (dum == 0.);
  return -log(dum)/lambda;
}


/**
 * expSelect
 */
inline int expSelect(int max)
{
  int i;
  /* we want 99% probability of getting with expdev() a number smaller max
   * this requires a lambda of the exponential distribution:
   * lambda = -log(0.01)/max;    (-log(0.01) = 4.6) */
  float lambda = 4.6/(float)max;
  do
    i = (int)(expPdf(lambda));
  while(i > max);
  return i;
}

/**
 * getRandIdx
 */
void getRandIdx(int size, int num, std::vector<int> &idx)
{
  int temp;
  idx.clear();
  for (int i=0; i<num; i++)
  {
    do{
      temp = rand()%size;
    }while(contains(idx,temp));
    idx.push_back(temp);
  }
}

/**
 * getExpRandIdx
 */
void getExpRandIdx(int size, int num, std::vector<int> &idx)
{
  int temp;
  idx.clear();
  for (int i=0; i<num; i++)
  {
    do{
      temp = expSelect(size);
    }while(contains(idx,temp));
    idx.push_back(temp);
  }
}



} //--END--

#endif




