/*
 * hv_go_opt.h
 *
 *  Created on: Feb 27, 2013
 *      Author: aitor
 */

#ifndef FAAT_PCL_GHV_OPT_H_
#define FAAT_PCL_GHV_OPT_H_

#include <pcl/pcl_macros.h>
#include <pcl/common/common.h>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>
//#include "pcl/recognition/3rdparty/metslib/mets.hh"
#include "v4rexternal/metslib/mets.hh"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <map>
#include <iostream>
#include <fstream>
//#include <faat_pcl/utils/voxel_dist_transform.h>

#ifdef _MSC_VER
#ifdef FAAT_REC_EXPORTS
#define FAAT_REC_API __declspec(dllexport)
#else
#define FAAT_REC_API __declspec(dllimport)
#endif
#else
#define FAAT_REC_API
#endif

namespace faat_pcl
{

  //Helper classes
  template<typename ModelT>
  struct GHVRecognitionModel
  {
    public:
      std::vector<int> explained_; //indices vector referencing explained_by_RM_
      std::vector<float> explained_distances_; //closest distances to the scene for point i
      std::vector<int> unexplained_in_neighborhood; //indices vector referencing unexplained_by_RM_neighboorhods
      std::vector<float> unexplained_in_neighborhood_weights; //weights for the points not being explained in the neighborhood of a hypothesis
      std::vector<int> outlier_indices_; //outlier indices of this model
      std::vector<int> color_outliers_indices_;
      std::vector<int> outliers_3d_indices_;
      std::vector<int> complete_cloud_occupancy_indices_;
      std::vector<bool> scene_point_explained_by_hypothesis_; //boolean vector indicating if a scene point is explained by this model or not
      typename pcl::PointCloud<ModelT>::Ptr cloud_;
      typename pcl::PointCloud<ModelT>::Ptr complete_cloud_;
      typename pcl::PointCloud<pcl::Normal>::Ptr complete_cloud_normals_;
      float bad_information_;
      float outliers_weight_;
      pcl::PointCloud<pcl::Normal>::Ptr normals_;
      pcl::PointCloud<pcl::Normal>::Ptr normals_from_visible_;
      int id_;
      float extra_weight_; //descriptor distance weight for instance
      float color_similarity_;
      float median_;
      float mean_;
      Eigen::MatrixXf color_mapping_;
      float hyp_penalty_;
      std::string id_s_;
      std::vector<Eigen::Vector3f> cloud_LAB_;
      std::vector<Eigen::Vector3f> cloud_LAB_original_;
      std::vector<Eigen::Vector3f> cloud_RGB_;
      std::vector<float> cloud_GS_;
      float min_contribution_; //based on the amount of explained points and the amount of information in the hypotheses
      std::vector<float> normal_angle_histogram_;
      std::vector<float> color_diff_histogram_;
      float normal_entropy_;
      float color_entropy_;
      std::vector<int> cloud_indices_specified_;
      float color_diff_trhough_specification_;
      pcl::PointCloud<pcl::PointXYZL>::Ptr visible_labels_;

      //inlier indices and distances for cloud_ (this avoids recomputing radius searches twice (one for specification and one for inlier/outlier detection)
      std::vector<std::vector<int> > inlier_indices_;
      std::vector<std::vector<float> > inlier_distances_;
  };

  template<typename ModelT, typename SceneT> class GHV;

  template<typename ModelT, typename SceneT>
  class GHVSAModel : public mets::evaluable_solution
  {

    typedef GHV<ModelT, SceneT> SAOptimizerT;

  public:
    std::vector<bool> solution_;
    SAOptimizerT * opt_;
    mets::gol_type cost_;

    //Evaluates the current solution
    mets::gol_type
    cost_function () const
    {
      return cost_;
    }

    void
    copy_from (const mets::copyable& o)
    {
      const GHVSAModel& s = dynamic_cast<const GHVSAModel&> (o);
      solution_ = s.solution_;
      opt_ = s.opt_;
      cost_ = s.cost_;
    }

    void
    copy_from (const mets::feasible_solution& o)
    {
      const GHVSAModel& s = dynamic_cast<const GHVSAModel&> (o);
      solution_ = s.solution_;
      opt_ = s.opt_;
      cost_ = s.cost_;
    }

    mets::gol_type
    what_if (int /*index*/, bool /*val*/) const
    {
      return static_cast<mets::gol_type> (0);
    }

    mets::gol_type
    apply_and_evaluate (int index, bool val)
    {
      solution_[index] = val;
      mets::gol_type sol = opt_->evaluateSolution (solution_, index); //this will update the state of the solution
      cost_ = sol;
      return sol;
    }

    void
    apply (int /*index*/, bool /*val*/)
    {

    }

    void
    unapply (int index, bool val)
    {
      solution_[index] = val;
      //update optimizer solution
      cost_ = opt_->evaluateSolution (solution_, index); //this will udpate the cost function in opt_
    }
    void
    setSolution (std::vector<bool> & sol)
    {
      solution_ = sol;
    }

    void
    setOptimizer (SAOptimizerT * opt)
    {
      opt_ = opt;
    }
  };

  /*
   * Represents a generic move from which all move types should inherit
   */

  class GHVgeneric_move : public mets::mana_move
  {
  public:
    virtual mets::gol_type
    evaluate (const mets::feasible_solution& cs) const = 0;
    virtual mets::gol_type
    apply_and_evaluate (mets::feasible_solution& cs) = 0;
    virtual void
    apply (mets::feasible_solution& s) const = 0;
    virtual void
    unapply (mets::feasible_solution& s) const = 0;
    virtual mets::clonable*
    clone () const = 0;
    virtual size_t
    hash () const = 0;
    virtual bool
    operator== (const mets::mana_move&) const = 0;
  };

  /*
   * Represents a move that deactivates an active hypothesis replacing it by an inactive one
   * Such moves should be done when the temperature is low
   * It is based on the intersection of explained points between hypothesis
   */

  template<typename ModelT, typename SceneT>
  class GHVreplace_hyp_move : public GHVgeneric_move
  {
    int i_, j_; //i_ is an active hypothesis, j_ is an inactive hypothesis
    int sol_size_;

  public:
    GHVreplace_hyp_move (int i, int j, int sol_size) :
      i_ (i), j_ (j), sol_size_ (sol_size)
    {
    }

    mets::gol_type
    evaluate (const mets::feasible_solution& cs) const
    {
      GHVSAModel<ModelT, SceneT> model;
      model.copy_from (cs);
      model.apply_and_evaluate (i_, !model.solution_[i_]);
      mets::gol_type cost = model.apply_and_evaluate (j_, !model.solution_[j_]);
      //unapply moves now
      model.unapply (j_, !model.solution_[j_]);
      model.unapply (i_, !model.solution_[i_]);
      return cost;
    }

    mets::gol_type
    apply_and_evaluate (mets::feasible_solution& cs)
    {
      GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (cs);
      assert (model.solution_[i_]);
      model.apply_and_evaluate (i_, !model.solution_[i_]);
      assert (!model.solution_[j_]);
      return model.apply_and_evaluate (j_, !model.solution_[j_]);
    }

    void
    apply (mets::feasible_solution& s) const
    {
      GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
      model.apply_and_evaluate (i_, !model.solution_[i_]);
      model.apply_and_evaluate (j_, !model.solution_[j_]);
    }

    void
    unapply (mets::feasible_solution& s) const
    {
      //go back
      GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
      model.unapply (j_, !model.solution_[j_]);
      model.unapply (i_, !model.solution_[i_]);
    }

    mets::clonable*
    clone () const
    {
      GHVreplace_hyp_move * m = new GHVreplace_hyp_move (i_, j_, sol_size_);
      return static_cast<mets::clonable*> (m);
    }

    size_t
    hash () const;
    /*{
      return static_cast<size_t> (sol_size_ + sol_size_ * i_ + j_);
    }*/

    bool
    operator== (const mets::mana_move& m) const;
    /*{
      const replace_hyp_move& mm = dynamic_cast<const replace_hyp_move&> (m);
      return (mm.i_ == i_) && (mm.j_ == j_);
    }*/
  };

  /*
   * Represents a move, activate/deactivate an hypothesis
   */

  template<typename ModelT, typename SceneT>
  class GHVmove : public GHVgeneric_move
  {
    int index_;
  public:
    GHVmove (int i) :
      index_ (i)
    {
    }

    int
    getIndex ()
    {
      return index_;
    }

    mets::gol_type
    evaluate (const mets::feasible_solution& cs) const
    {
      //mets::copyable copyable = dynamic_cast<mets::copyable> (&cs);
      GHVSAModel<ModelT, SceneT> model;
      model.copy_from (cs);
      mets::gol_type cost = model.apply_and_evaluate (index_, !model.solution_[index_]);
      model.apply_and_evaluate (index_, !model.solution_[index_]);
      return cost;
    }

    mets::gol_type
    apply_and_evaluate (mets::feasible_solution& cs)
    {
      GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (cs);
      return model.apply_and_evaluate (index_, !model.solution_[index_]);
    }

    void
    apply (mets::feasible_solution& s) const
    {
      GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
      model.apply_and_evaluate (index_, !model.solution_[index_]);
    }

    void
    unapply (mets::feasible_solution& s) const
    {
      GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
      model.unapply (index_, !model.solution_[index_]);
    }

    mets::clonable*
    clone () const
    {
      GHVmove * m = new GHVmove (index_);
      return static_cast<mets::clonable*> (m);
    }

    size_t
    hash () const;

    bool
    operator== (const mets::mana_move& m) const;
  };

  template<typename ModelT, typename SceneT>
    class GHVmove_activate : public GHVgeneric_move
    {
      int index_;
    public:
      GHVmove_activate (int i) :
        index_ (i)
      {
      }

      int
      getIndex ()
      {
        return index_;
      }

      mets::gol_type
      evaluate (const mets::feasible_solution& cs) const
      {
        //mets::copyable copyable = dynamic_cast<mets::copyable> (&cs);
        GHVSAModel<ModelT, SceneT> model;
        model.copy_from (cs);
        mets::gol_type cost = model.apply_and_evaluate (index_, true);
        model.apply_and_evaluate (index_, false);
        return cost;
      }

      mets::gol_type
      apply_and_evaluate (mets::feasible_solution& cs)
      {
        GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (cs);
        return model.apply_and_evaluate (index_, true);
      }

      void
      apply (mets::feasible_solution& s) const
      {
        GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
        model.apply_and_evaluate (index_, true);
      }

      void
      unapply (mets::feasible_solution& s) const
      {
        GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
        model.unapply (index_, false);
      }

      mets::clonable*
      clone () const
      {
        GHVmove_activate * m = new GHVmove_activate (index_);
        return static_cast<mets::clonable*> (m);
      }

      size_t
      hash () const;

      bool
      operator== (const mets::mana_move& m) const;
    };

    template<typename ModelT, typename SceneT>
      class GHVmove_deactivate : public GHVgeneric_move
      {
        int index_;
        int problem_size_;
      public:
        GHVmove_deactivate (int i, int problem_size) :
            index_ (i), problem_size_(problem_size)
        {
        }

        int
        getIndex ()
        {
          return index_;
        }

        mets::gol_type
        evaluate (const mets::feasible_solution& cs) const
        {
          //mets::copyable copyable = dynamic_cast<mets::copyable> (&cs);
          GHVSAModel<ModelT, SceneT> model;
          model.copy_from (cs);
          mets::gol_type cost = model.apply_and_evaluate (index_, false);
          model.apply_and_evaluate (index_, true);
          return cost;
        }

        mets::gol_type
        apply_and_evaluate (mets::feasible_solution& cs)
        {
          GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (cs);
          return model.apply_and_evaluate (index_, false);
        }

        void
        apply (mets::feasible_solution& s) const
        {
          GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
          model.apply_and_evaluate (index_, false);
        }

        void
        unapply (mets::feasible_solution& s) const
        {
          GHVSAModel<ModelT, SceneT>& model = dynamic_cast<GHVSAModel<ModelT, SceneT>&> (s);
          model.unapply (index_, true);
        }

        mets::clonable*
        clone () const
        {
          GHVmove_deactivate * m = new GHVmove_deactivate (index_, problem_size_);
          return static_cast<mets::clonable*> (m);
        }

        size_t
        hash () const;

        bool
        operator== (const mets::mana_move& m) const;
      };

    template<typename ModelT, typename SceneT>
      class GHVmove_manager
      {
        bool use_replace_moves_;
      public:
        std::vector<GHVgeneric_move*> moves_m;
        boost::shared_ptr<std::map<std::pair<int, int>, bool> > intersections_;
        typedef typename std::vector<GHVgeneric_move*>::iterator iterator;
        int problem_size_;
        iterator
        begin ()
        {
          return moves_m.begin ();
        }
        iterator
        end ()
        {
          return moves_m.end ();
        }

        GHVmove_manager (int problem_size, bool rp_moves = true)
        {
          use_replace_moves_ = rp_moves;
          problem_size_ = problem_size;

          /*for (int ii = 0; ii != problem_size; ++ii)
            moves_m.push_back (new move<ModelT, SceneT> (ii));*/
        }

        ~GHVmove_manager ()
        {
          // delete all moves
          for (iterator ii = begin (); ii != end (); ++ii)
            delete (*ii);
        }

        void
        setExplainedPointIntersections (boost::shared_ptr<std::map<std::pair<int, int>, bool> > & intersections)
        {
          intersections_ = intersections;
        }

        void
        refresh (mets::feasible_solution& s);
      };

  template<typename ModelT, typename SceneT>
  class GHVCostFunctionLogger : public mets::solution_recorder
  {
    std::vector<float> costs_;
    std::vector<float> costs_each_time_evaluated_;
    int times_evaluated_;
    boost::function<void (const std::vector<bool> &, float, int)> visualize_function_;

  public:
    GHVCostFunctionLogger ();

    GHVCostFunctionLogger (mets::evaluable_solution& best) :
      mets::solution_recorder (), best_ever_m (best)
    {
      times_evaluated_ = 0;
      costs_.resize (0);
    }

    void setVisualizeFunction(boost::function<void (const std::vector<bool> &, float, int)> & f)
    {
        visualize_function_ = f;
    }

    void
    writeToLog (std::ofstream & of)
    {
      of << times_evaluated_ << "\t\t";
      of << costs_.size () << "\t\t";
      of << costs_[costs_.size () - 1] << std::endl;
    }

    void
    writeEachCostToLog (std::ofstream & of)
    {
      for (size_t i = 0; i < costs_each_time_evaluated_.size (); i++)
      {
        of << costs_each_time_evaluated_[i] << "\t";
      }
      of << std::endl;
    }

    void
    addCost (float c)
    {
      costs_.push_back (c);
    }

    void
    addCostEachTimeEvaluated (float c)
    {
      costs_each_time_evaluated_.push_back (c);
    }

    void
    increaseEvaluated ()
    {
      times_evaluated_++;
    }

    int
    getTimesEvaluated ()
    {
      return times_evaluated_;
    }

    size_t
    getAcceptedMovesSize ()
    {
      return costs_.size ();
    }

    bool
    accept (const mets::feasible_solution& sol)
    {
      const mets::evaluable_solution& s = dynamic_cast<const mets::evaluable_solution&> (sol);
      if (s.cost_function () < best_ever_m.cost_function ())
      {
        best_ever_m.copy_from (s);
        const GHVSAModel<ModelT, SceneT>& ss = static_cast<const GHVSAModel<ModelT, SceneT>&> (sol);
        costs_.push_back (ss.cost_);
        std::cout << "Move accepted:" << ss.cost_ << std::endl;

        if(visualize_function_)
        {
            visualize_function_(ss.solution_, ss.cost_, times_evaluated_);
        }
        return true;
      }
      return false;
    }

    /// @brief Returns the best solution found since the beginning.
    const mets::evaluable_solution&
    best_seen () const
    {
      return best_ever_m;
    }

    mets::gol_type
    best_cost () const
    {
      return best_ever_m.cost_function ();
    }

    void
    setBestSolution (std::vector<bool> & sol)
    {
      GHVSAModel<ModelT, SceneT>& ss = static_cast<GHVSAModel<ModelT, SceneT>&> (best_ever_m);
      for (size_t i = 0; i < sol.size (); i++)
      {
        ss.solution_[i] = sol[i];
        //std::cout << "setBestSolution" << ss.solution_[i] << " " << sol[i] << std::endl;
      }
    }

  protected:
    /// @brief Records the best solution
    mets::evaluable_solution& best_ever_m;
  };
}
#endif /* FAAT_PCL_HV_GO_OPT_H_ */
