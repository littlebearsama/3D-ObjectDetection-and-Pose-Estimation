/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

// disjoint-set forests using union-by-rank and path compression (sort of).

#ifndef GC_DISJOINT_SET
#define GC_DISJOINT_SET

#include <cstdio>

namespace gc
{
  
typedef struct {
  int rank;         // rank of element (starts with 0)
  int p;            // Parent element
  int size;         // How many elements are in a set
} uni_elt;


class universe {
public:
  universe(int elements);
  ~universe();
  int find(int x);  
  void join(int x, int y);
  int size(int x) const { return elts[x].size; }
  int num_sets() const { return num; }
  void printAll();

private:
  uni_elt *elts;
  int num;
};

}

#endif
