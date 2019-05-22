/*
 * hv_go_bin_opt.hpp
 *
 *  Created on: Feb 27, 2013
 *      Author: aitor
 */

#include "ghv_opt.h"
#include <numeric>

template<typename ModelT, typename SceneT>
size_t
faat_pcl::GHVreplace_hyp_move<ModelT, SceneT>::hash () const
{
  return static_cast<size_t> (sol_size_ + sol_size_ * i_ + j_);
}

template<typename ModelT, typename SceneT>
bool
faat_pcl::GHVreplace_hyp_move<ModelT, SceneT>::operator== (const mets::mana_move& m) const
{
    try
    {
        //std::cout << "Going to cast replace_hyp_move" << std::endl;
        const GHVreplace_hyp_move& mm = dynamic_cast<const GHVreplace_hyp_move&> (m);
        //std::cout << "Finished casting replace_hyp_move" << std::endl;
        return (mm.i_ == i_) && (mm.j_ == j_);

    }
    catch(std::bad_cast & bc)
    {
        std::cout << "bad cast:" << bc.what() << "\n";
        return false;
    }

}

template<typename ModelT, typename SceneT>
size_t
faat_pcl::GHVmove<ModelT, SceneT>::hash () const
{
  return static_cast<size_t> (index_);
}

template<typename ModelT, typename SceneT>
bool
faat_pcl::GHVmove<ModelT, SceneT>::operator== (const mets::mana_move& m) const
{
  std::cout << "Going to cast move, should not happen" << std::endl;
  const GHVmove& mm = dynamic_cast<const GHVmove&> (m);
  return mm.index_ == index_;
}

template<typename ModelT, typename SceneT>
size_t
faat_pcl::GHVmove_activate<ModelT, SceneT>::hash () const
{
  return static_cast<size_t> (index_);
}

template<typename ModelT, typename SceneT>
bool
faat_pcl::GHVmove_activate<ModelT, SceneT>::operator== (const mets::mana_move& m) const
{
    try
    {
        //std::cout << "Going to cast move activate" << std::endl;
        const GHVmove_activate& mm = dynamic_cast<const GHVmove_activate&> (m);
        //std::cout << "Finished cast move activate" << std::endl;
        return mm.index_ == index_;
    }
    catch(std::bad_cast & bc)
    {
        std::cout << "bad cast:" << bc.what() << "\n";
        return false;
    }
}

template<typename ModelT, typename SceneT>
size_t
faat_pcl::GHVmove_deactivate<ModelT, SceneT>::hash () const
{
  return static_cast<size_t> (index_ + problem_size_);
}

template<typename ModelT, typename SceneT>
bool
faat_pcl::GHVmove_deactivate<ModelT, SceneT>::operator== (const mets::mana_move& m) const
{
    try
    {
        //std::cout << "Going to cast move deactivate" << std::endl;
        const GHVmove_deactivate& mm = dynamic_cast<const GHVmove_deactivate&> (m);
        //std::cout << "Finished cast move deactivate" << std::endl;
        return mm.index_ == index_;
    }
    catch(std::bad_cast & bc)
    {
        std::cout << "bad cast:" << bc.what() << "\n";
        return false;
    }

}

///////////////////////////////////////////////////////////////
///////////// move manager ////////////////////////////////////
///////////////////////////////////////////////////////////////

template<typename ModelT, typename SceneT>
void
faat_pcl::GHVmove_manager<ModelT, SceneT>::refresh(mets::feasible_solution& s)
{
  for (iterator ii = begin (); ii != end (); ++ii)
    delete (*ii);

  GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
  moves_m.clear();
  moves_m.resize(model.solution_.size() + model.solution_.size()*model.solution_.size());
  for (int ii = 0; ii != model.solution_.size(); ++ii)
  {
      if(!model.solution_[ii])
      {
        moves_m[ii]  = new GHVmove_activate<ModelT, SceneT> (ii);
      }
      else
      {
        moves_m[ii]  = new GHVmove_deactivate<ModelT, SceneT> (ii, problem_size_);
      }
  }

  if(use_replace_moves_) {
    //based on s and the explained point intersection, create some replace_hyp_move
    //go through s and select active hypotheses and non-active hypotheses
    //check for each pair if the intersection is big enough
    //if positive, create a replace_hyp_move that will deactivate the act. hyp and activate the other one
    //MAYBE it would be interesting to allow this changes when the temperature is low or
    //there has been some iterations without an improvement
    std::vector<int> active, inactive;
    active.resize(model.solution_.size());
    inactive.resize(model.solution_.size());
    int nact, ninact;
    nact = ninact = 0;
    for(int i=0; i <static_cast<int>(model.solution_.size()); i++) {
      if(model.solution_[i]) {
        active[nact] = i;
        nact++;
      } else {
        inactive[ninact] = i;
        ninact++;
      }
    }

    active.resize(nact);
    inactive.resize(ninact);

    int nm=0;
    for(size_t i=0; i < active.size(); ++i) {
      for(size_t j=(i+1); j < inactive.size(); ++j) {
        std::map< std::pair<int, int>, bool>::iterator it;
        it = intersections_->find(std::make_pair<int, int>(std::min(active[i], inactive[j]),std::max(active[i], inactive[j])));
        assert(it != intersections_->end());
        if((*it).second) {
          moves_m[model.solution_.size() + nm] = new GHVreplace_hyp_move<ModelT, SceneT> (active[i], inactive[j], model.solution_.size());
          nm++;
        }
      }
    }

    moves_m.resize(model.solution_.size() + nm);
  } else {
    moves_m.resize(model.solution_.size());
  }
  std::random_shuffle (moves_m.begin (), moves_m.end ());
}
