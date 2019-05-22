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

#include "disjoint-set.h"

namespace gc
{
  
universe::universe(int elements) 
{
  elts = new uni_elt[elements];
  num = elements;
  for (int i = 0; i < elements; i++) {
    elts[i].rank = 0;
    elts[i].size = 1;
    elts[i].p = i;
  }
}
  
universe::~universe() {
  delete [] elts;
}

int universe::find(int x) {
  int y = x;
  while (y != elts[y].p) {
    y = elts[y].p;
  }
  elts[x].p = y;
  return y;
}

void universe::join(int x, int y) 
{
  if (elts[x].rank > elts[y].rank) {
    elts[y].p = x;
    elts[x].size += elts[y].size;
  } else {
    elts[x].p = y;
    elts[y].size += elts[x].size;
    if (elts[x].rank == elts[y].rank)
      elts[y].rank++;
  }
  num--;
}

void universe::printAll() 
{
  for(int i=0; i<num; i++)
    printf("[universe::printAll] %u: elts: rank: %u - p: %u - size: %u\n", i, elts[i].rank, elts[i].p, elts[i].size);
}

}

