/**
 * $Id$
 *
 * Copyright (c) 2014, Johann Prankl
 * @author Johann Prankl (prankl@acin.tuwien.ac.at)
 */

#ifndef KP_TRIPLE_HPP
#define KP_TRIPLE_HPP


namespace kp 
{

/**
 * triple
 */
template <class T1, class T2, class T3>
class triple
{
public:
  T1 first;
  T2 second;
  T3 third;

  triple(){}
  triple(const T1 &_f, const T2 &_s, const T3 &_t) : first(_f), second(_s), third(_t) {}
};

/**
 * tripleIIF
 */
class tripleIIF : public triple<int, int, float>
{
public:
  tripleIIF() {}
  tripleIIF(const int &_f, const int &_s, const float &_t) : triple(_f,_s,_t) {}
};

inline bool CmpIncIIF(const tripleIIF &i, const tripleIIF &j)
{
  return (i.third<j.third);
}

inline bool CmpDecIIF(const tripleIIF &i, const tripleIIF &j)
{
  return (i.third>j.third);
}



} //--END--

#endif

