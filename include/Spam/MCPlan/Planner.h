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
// @Author:   Claudio Zito
// @Date:     19/07/2012
//-------------------------------------------------------------------------
#ifndef _SPINTA_PLANNER_PLANNER_H_
#define _SPINTA_PLANNER_PLANNER_H_

//-------------------------------------------------------------------------

#include <Spam/MCPlan/Model.h>

//-------------------------------------------------------------------------

namespace spam {

//-------------------------------------------------------------------------

/** Exceptions */
//MESSAGE_DEF(MsgPlanner, golem::Message)

class RRTPlayer;
//-------------------------------------------------------------------------

class Planner {
public:
	/** Pointer */
	typedef boost::shared_ptr<Planner> Ptr;
	friend class Desc;
	friend class RRTPlayer;

	/** Description file */
	class Desc {
	public:	
		/** Pointer */
		typedef boost::shared_ptr<Desc> Ptr;

		Model::Desc::Ptr pModelDesc;

		/** Max error */ 
		double error;
		/** Number of nodes to generate in a single execution of Plan or Construct */
		int numNodes;
		/** Number of tollerated failures */
		int nMaxFails; 
		/** Bidirectional tree */
		bool bidirectional;
		/** Time step to use for incremental planners */
		double plannerDeltaT;

		/** Initial state */
		golem::Vec3 initialState;
		/** Goal State */
		golem::Vec3 goalState;

		/** Constructor */
		Desc() {
			setToDefault();
		}
		/** Set parameter to default */
		void setToDefault() {
			pModelDesc.reset(new Model::Desc);

			error = 0.01;
			numNodes = 100;
			nMaxFails = 2;
			bidirectional = false;

			plannerDeltaT = 0.1;

			initialState.setToDefault();
			goalState.setToDefault();

//			pPushPlannerDesc.reset(new PushPlanner::Desc);
		}
		/** Check if valid */
		bool isValid() const {
			if (numNodes < 1)
				return false;
			if (!pModelDesc->isValid())
				return false;

			return true;
		}

		/** Creates the object from the description. */
		virtual Planner::Ptr create(const golem::Context& context) const = 0;
	};

	/** This will make "regular" path planning much faster. */
	bool holonomic;

	/** Reset the planner */
	void reset();

	/** Generate a planning graph */
	virtual void construct() = 0;
	/** Attempt to solve an Initial-Goal query */
	virtual bool plan() = 0;

	/** Determine if the gap error is satisfied */
	bool gapSatisfied(const golem::Vec3& x1, const golem::Vec3& x2);

	//! Write trees to a file
	virtual void writeGraphs(std::ofstream &fout) = 0;
	//! Read trees from a file
	virtual void readGraphs(std::ifstream &fin) = 0;

	/** Virtual desctuctor */
	virtual ~Planner();

protected:
	/** Context */
	golem::Context context;

	/** Pointer to the model */
	Model::Ptr modelPtr;

	/** Linear max distance error in x,y,z */ 
	double error;
	/** Number of nodes to generate in a single execution of Plan or Construct */
	int numNodes;
	/** Number of tollerated failures */
	int nMaxFails; 
	/** Number of times the collision checker has been called */
	int satisfiedCount;
	/** Bidirectional tree */
	bool bidirectional;
	/** Time step to use for incremental planners */
	double plannerDeltaT;

	/** Generator of pseudo random numbers */
	golem::Rand rand;

	/** RRTree from root to goal */
	Tree3d *pRRTree;
	/** RRTree form goal to root */
//	Tree3d *pRRTree2;

	/** Initial state */
	golem::Vec3 initialState;
	/** Goal State */
	golem::Vec3 goalState;

	/** Select a random state as a 3d vector */
	virtual golem::Vec3 getNextState();
	/** Select a random state as a 3d vector */
	virtual golem::Vec3 getNextState(const golem::Vec3& x);

	/** Create planner from description */
	bool create(const Desc &desc);
	/** Default constructor takes a controller */
	Planner(const golem::Context& context);
};

//-------------------------------------------------------------------------

class IncrementalPlanner: public Planner {
 public:
	 friend class RRTPlayer;
	 class Desc : public Planner::Desc {
	 public:
		 Desc() {
			 setToDefault();
		 }
		 void setToDefault() {
			 Planner::Desc::setToDefault();
		 }
		 bool isValid() const {
			 if (!Planner::Desc::isValid())
				 return false;
			 return true;
		 }

		 virtual Planner::Ptr create(const golem::Context& context) const = 0;
	 };

	//! Essentially do nothing (no precomputation for incremental planners)
	virtual void construct() {};

	////! Convert a path in the graph to Path and Policy
	void recordSolution(const Node3d::SeqPtr& glist, const Node3d::SeqPtr& g2list);

	void recordSolution(const Node3d::SeqPtr& glist);

	//! Write trees to a file
	virtual void writeGraphs(std::ofstream& fout);

	//! Read trees from a file
	virtual void readGraphs(std::ifstream& fin);

	virtual ~IncrementalPlanner();

protected:
	/** Create planner from description */
	bool create(const Desc &desc);
	/** Constructor */
	IncrementalPlanner(const golem::Context& context);
};

//-------------------------------------------------------------------------

}; /* namespace */

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_PLANNER_H_ */