//-------------------------------------------------------------------------
//                  Single Finger Pushing Tool (SPinTa)
//-------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Single Finger Pushing Tool (SPinTa), 
//       University of Birmingham, nor the names of its contributors may be 
//       used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     20/07/2012
//-------------------------------------------------------------------------
#pragma once
#ifndef _SPINTA_PLANNER_RRT_H_
#define _SPINTA_PLANNER_RRT_H_

//-------------------------------------------------------------------------

#include <Spam/MCPlan/Planner.h>
#include <Spam/MCPlan/GaussianProcess.h>

//-------------------------------------------------------------------------

namespace spam {

//-------------------------------------------------------------------------

class RRTPlayer;

//-------------------------------------------------------------------------

/**  The base class for the planners based on Rapidly-exploring 
Random Trees.  In the base class, a single tree is generated without
any regard to the GoalState.  The best planners to try are 
RRTGoalBias and RRTGoalZoom for single trees, and RRTConCon and 
RRTExtExt for dual trees.  Dual tree approaches are much more efficient
than single tree approaches, assuming dual trees can be applied.  */

//! The base class, which generates a single Rapidly-exploring Random Tree.
class RRT : public IncrementalPlanner {
public:
	friend class RRTPlayer;

	/** Planner descriptor */
	class Desc : public IncrementalPlanner::Desc {
	public:
		Desc() {
			setToDefault();
		}
		void setToDefault() {
			IncrementalPlanner::Desc::setToDefault();
		}
		bool isValid() const {
			if (!IncrementalPlanner::Desc::isValid())
				return false;
			return true;
		}

		/** Creates the object from the description. */
		Planner::Ptr create(const golem::Context& context) const {
			RRT* pObject = new RRT(context);
			Planner::Ptr pointer(pObject);
			pObject->create(*this);
			return pointer;
		}
	};

	/** Reset the planner */
	virtual void reset();
	/** Reset the planner with a new initial pose */
	virtual void reset(golem::Vec3& newInitPose);
	/** Attempt to solve an Initial-Goal query by growing an RRT */
	virtual bool plan();
	/** Essentially do nothing (no precomputation for incremental planners) */
	virtual void construct() {};

	/** Virtual desctructor */
	virtual ~RRT();
	
protected:
	/** The maximum amount of time to move in a Connect step (default = INFINITY) */
	double ConnectTimeLimit;
	/** The distance of the closest RRT Node to the goal */
	golem::Real goalDist; 
	/** The closest state to the goal so far (not used in dual-tree planners) */
	golem::Vec3 bestState;

	/** Return the nearest neighbour in the graph */
	virtual Node3d* selectNode(const golem::Vec3& x, Tree3d* t, bool forward = true) const;
	/** Select the input that gets closest to x2 from x1 */
	virtual golem::Vec3 selectInput(const golem::Vec3& xNear, const golem::Vec3& xRand) const;
	/** Incrementally extend the RRT */
	virtual Node3d* extend(const golem::Vec3& x, Tree3d* t, bool forward);
	/** Iterated Extend */
	virtual bool connect(const golem::Vec3& x, Tree3d* t, Node3d*& nn, bool forward);

	/** Create RRT planner from description */
	bool create(const Desc& desc);
	/** Constructor */
	RRT(const golem::Context& context);
};

//-------------------------------------------------------------------------

//void XMLData(RRT::Desc &val, golem::Context *context, golem::XMLContext* xmlcontext, bool create = false);

//-------------------------------------------------------------------------

class AtlasRRT : public RRT {
public:
	class Desc : virtual public RRT::Desc {
	public:
		Desc() {
		}
		void setToDefault() {
			RRT::Desc::setToDefault();
		}
		bool isValid() const {
			if (!RRT::Desc::isValid())
				return false;
			return true;
		}

		/** Creates the object from the description. */
		Planner::Ptr create(const golem::Context& context) const {
			AtlasRRT* pObject = new AtlasRRT(context);
			Planner::Ptr pointer(pObject);
			pObject->create(*this);
			return pointer;
		}
	};

	/** Container of the GP pointer */
	LaplaceRegressor::Ptr gpPtr;

	virtual ~AtlasRRT();

protected:
	bool create(const Desc &desc);
	AtlasRRT(const golem::Context& context);
};

//-------------------------------------------------------------------------

/*! Instead of choosing a state at random, this planner chooses with
    probability GoalProb the GoalState.  It can be considered as a 
    biased coin toss in which heads yields the goal state, and tails
    yields a random sample.
*/
/** With some probability, choose the goal instead of a random sample */
//class RRTGoalBias : public RRT {
//public:
//	class Desc : virtual public RRT::Desc {
//	public:
//		Desc() {
//		}
//		void setToDefault() {
//			RRT::Desc::setToDefault();
//		}
//		bool isValid() const {
//			if (!RRT::Desc::isValid())
//				return false;
//			return true;
//		}
//
//		golem::Real goalProb;
//
//		/** Creates the object from the description. */
//		Planner::Ptr create(const golem::Context& context) const {
//			RRTGoalBias* pObject = new RRTGoalBias(context);
//			Planner::Ptr pointer(pObject);
//			pObject->create(*this);
//			return pointer;
//		}
//	};
//	
//	virtual ~RRTGoalBias();
//
//protected:
//	/** Probability of selecting the goal */
//	golem::Real goalProb;
//
//	/** Select randomly state according to the goal probability */
//	virtual golem::Vec3 chooseState();
//
//	bool create(const Desc &desc);
//	RRTGoalBias(const golem::Context& context);
//};

//-------------------------------------------------------------------------

}; /* namespace */

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_RRT_H_ */