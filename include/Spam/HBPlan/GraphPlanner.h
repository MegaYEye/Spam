//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     02/04/2013
//------------------------------------------------------------------------------
#pragma once
#ifndef _SPAM_GRAPHPLANNER_H_
#define _SPAM_GRAPHPLANNER_H_

//------------------------------------------------------------------------------

#include <Spam/HBPlan/Heuristic.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Performance monitor */
#define _GRAPHPLANNER_PERFMON

//------------------------------------------------------------------------------

/** Abstract class for Arm movement planinng using Probabilistic Road Map approach. */
//class RagPathFinder : public golem::PathFinder {
//public:
//	typedef shared_ptr<RagPathFinder> Ptr;
//	friend class Desc;
//	friend class RagGraphPlanner;
//
//	/** Path finder description */
//	class Desc : public golem::PathFinder::Desc {
//	public:
//		typedef shared_ptr<Desc> Ptr;
//
//		/** Constructs the description object. */
//		Desc() {
//			Desc::setToDefault();
//		}
//
//		/** virtual destructor is required */
//		virtual ~Desc() {}
//
//		/** Creates the object from the description. */
//		CREATE_FROM_OBJECT_DESC1(RagPathFinder, golem::PathFinder::Ptr, DEKinematics&)
//
//		/** Sets the parameters to the default values */
//		virtual void setToDefault() {
//			golem::PathFinder::Desc::setToDefault();
//		}
//
//		/** Checks if the description is valid. */
//		virtual bool isValid() const {
//			return golem::PathFinder::Desc::isValid();
//		}
//	};
//
//protected:
//	/** Generates random waypoint */
//	virtual bool generateWaypoint(Waypoint& w, const WaypointGenerator &generator, U32 trials) const;
//
//	/** Creates PathFinder from the description.
//	* @param desc		PathFinder description
//	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise
//	*/
//	bool create(const Desc& desc);
//
//	/** PathFinder constructor */
//	RagPathFinder(golem::DEKinematics& kinematics);
//
//};

//------------------------------------------------------------------------------

/** General planner debug info */
std::string plannerDebug(golem::Planner& planner);
/** Configspace coordinate planner debug info */
std::string plannerConfigspaceDebug(golem::Planner& planner, const golem::ConfigspaceCoord* c = nullptr);
/** Workspace coordinate planner debug info */
std::string plannerWorkspaceDebug(golem::Planner& planner, const golem::WorkspaceChainCoord* w = nullptr);

//------------------------------------------------------------------------------

/** Abstract class for Arm movement planinng using Probabilistic Road Map approach. */
class RagGraphPlanner : public golem::GraphPlanner {
public:
	typedef golem::shared_ptr<RagGraphPlanner> Ptr;
	friend class Desc;

	///** Path finder description */
	//class RagPathFinderDesc : public golem::GraphPlanner::PathFinderDesc {
	//public:
	//	/** Global path finder description */
	//	RagPathFinder::Desc::Ptr pGlobalPathFinderDesc;
	//	/** Local path finder description */
	//	RagPathFinder::Desc::Ptr pLocalPathFinderDesc;
	//	/** Constructs the description object. */
	//	RagPathFinderDesc() {
	//		setToDefault();
	//	}
	//	/** Sets the parameters to the default values */
	//	void setToDefault() {
	//		golem::GraphPlanner::PathFinderDesc::setToDefault();
	//		pGlobalPathFinderDesc.reset(new RagPathFinder::Desc);
	//		pGlobalPathFinderDesc->graphSize = 2000;
	//		pLocalPathFinderDesc.reset(new RagPathFinder::Desc);
	//		pLocalPathFinderDesc->graphSize = 1000;
	//	}
	//	/** Checks if the description is valid. */
	//	bool isValid() const {
	//		if (!golem::GraphPlanner::PathFinderDesc::isValid())
	//			return false;
	//		if (pGlobalPathFinderDesc == NULL || !pGlobalPathFinderDesc->isValid())
	//			return false;
	//		if (pLocalPathFinderDesc != NULL && !pLocalPathFinderDesc->isValid())
	//			return false;
	//		return true;
	//	}
	//};
	/////** Path optimiser description */
	////class PathOptimisationDesc {
	////public:
	////	/** number of iterations per path waypoint: (0, inf) */
	////	U32 numOfIterations;
	////	/** temperature settings: inf > Tinit > Tfinal > 0 */
	////	Real Tinit, Tfinal;
	////	/** energy normalization */
	////	Real Enorm;
	////	/** crossover probability: [0, 1] */
	////	Real crossProb;
	////	/** path (minimum) distance threshold */
	////	Real distPathThr;
	////	
	////	/** Constructs the description object. */
	////	PathOptimisationDesc() {
	////		setToDefault();
	////	}
	////	/** Sets the parameters to the default values */
	////	void setToDefault() {
	////		numOfIterations = 500;
	////		Tinit = Real(0.1);
	////		Tfinal = Real(0.01);
	////		Enorm = Real(3.0e4);
	////		crossProb = Real(0.3);
	////		distPathThr = Real(0.25); // (0, 1)
	////	}
	////	/** Checks if the description is valid. */
	////	bool isValid() const {
	////		if (numOfIterations <= 0)
	////			return false;
	////		if (Tinit <= REAL_ZERO || Tfinal <= REAL_ZERO)
	////			return false;
	////		if (Enorm <= REAL_ZERO)
	////			return false;
	////		if (crossProb < REAL_ZERO || crossProb > REAL_ONE)
	////			return false;
	////		if (distPathThr <= REAL_ZERO || distPathThr >= REAL_ONE)
	////			return false;
	////		return true;
	////	}
	////};
	/////** Local finder description */
	////class LocalFinderDesc {
	////public:
	////	/** Search range */
	////	std::vector<Real> range;
	////	/** Constructs the description object. */
	////	LocalFinderDesc() {
	////		setToDefault();
	////	}
	////	
	////	/** Sets the parameters to the default values */
	////	void setToDefault() {
	////		range.assign(Configspace::DIM, Real(0.1)*REAL_PI);
	////	}
	////	/** Checks if the description is valid. */
	////	bool isValid() const {
	////		for (std::vector<Real>::const_iterator i = range.begin(); i != range.end(); ++i)
	////			if (*i <= REAL_ZERO)
	////				return false;
	////		return true;
	////	}
	////};

	///** Planner description */
	class Desc : public golem::GraphPlanner::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		
		/** Constructs the description object. */
		Desc() {
			Desc::setToDefault();
		}

		/** Creates the object from the description. */
		GRASP_CREATE_FROM_OBJECT_DESC1(RagGraphPlanner, golem::Planner::Ptr, golem::Controller&)
		
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			golem::GraphPlanner::Desc::setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!golem::GraphPlanner::Desc::isValid())
				return false;
			return true;
		}
		/** Loads planner description from dynamic library.
		* @param context		program context
		* @param libraryPath		library path
		* @param configPath		xml configuration path
		* @return				pointer to the Planner description if no errors have occured, throws otherwise
		*/
		static Planner::Desc::Ptr load(golem::Context* context, const std::string& libraryPath, const std::string& configPath);


		/** Loads planner description from dynamic library.
		* @param context		program context
		* @param xmlcontext		xmlcontext which contains information about planner
		* @return				pointer to the Planner description if no errors have occured, throws otherwise
		*/
		static Planner::Desc::Ptr load(golem::Context* context, golem::XMLContext* xmlcontext);
	};

protected:
	/** Enable/disable uncertainty */
	bool enableUnc;
	///** Overwrites the path finder desc */
	//RagPathFinderDesc ragPathFinderDesc;
	//RagPathFinder::Ptr pGlobalPathFinder, pLocalPathFinder;

	/** Controller state info */
	golem::Controller::State::Info armInfo;
	/** Controller state info */
	golem::Controller::State::Info handInfo;
	std::vector<bool> jointDescSeq;

	void disableHandPlanning();
	void enableHandPlanning();

	/** Generates random waypoint */
//	virtual bool generateWaypoints(const golem::Controller::Trajectory &trajectory, golem::WaypointGenerator::Seq &generators, golem::U32 steps = 4) const;

	/** Performs local search on waypoint path */
	virtual bool localFind(const golem::ConfigspaceCoord &begin, const golem::ConfigspaceCoord &end, golem::Waypoint::Seq &localPath);

	/** Creates Planner from the description. 
	* @param desc		Planner description
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	*/
	bool create(const Desc& desc);

	/** Planner constructor */
	RagGraphPlanner(golem::Controller& controller);

public:
	inline golem::Waypoint::Seq getLocalPath() { return localPath; };
	/** FT Driven Heuristic */
	FTDrivenHeuristic* getFTDrivenHeuristic() const;
//	inline golem::WaypointGenerator::Seq getGlobalPathGenerators() { return pGlobalPathFinder->getGenerators(); };

	/** Finds (optimal) trajectory target in the obstacle-free configuration space.
	 */
	virtual bool findTarget(const golem::GenConfigspaceState &begin, const golem::GenWorkspaceChainState& wend, golem::GenConfigspaceState &cend);

	/** Finds obstacle-free (global search) trajectory in the configuration space from begin to end.
	 * @param begin		trajectory begin in the configuration space
	 * @param end		trajectory end in the configuration space
	 * @param path		trajectory
	 * @param iter		instertion point
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
	virtual bool findGlobalTrajectory(const golem::Controller::State &begin, const golem::Controller::State &end, golem::Controller::Trajectory &trajectory, golem::Controller::Trajectory::iterator iter, const golem::GenWorkspaceChainState* wend = NULL);

	/** Finds obstacle-free (local search) trajectory in the workspace form trajectory workspace increments.
	* @param cbegin	trajectory begin in the configuration space
	* @param wbegin	trajectory begin in the workspace
	* @param end		trajectory end in the workspace
	* @param path		generated trajectory in the configuration space
	* @param iter		trajectory instertion point
	* @param timeOut	maximum work time
	* @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise
	*/
	virtual bool findLocalTrajectory(const golem::Controller::State &cbegin, golem::GenWorkspaceChainState::Seq::const_iterator wbegin, golem::GenWorkspaceChainState::Seq::const_iterator end, golem::Controller::Trajectory &trajectory, golem::Controller::Trajectory::iterator iter, golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);

	/** Prints state for debugging */

	/** Finds obstacle-free (local search) trajectory in the workspace form trajectory workspace increments.
	 * @param cbegin	trajectory begin in the configuration space
	 * @param wbegin	trajectory begin in the workspace
	 * @param end		trajectory end in the workspace
	 * @param path		generated trajectory in the configuration space
	 * @param iter		trajectory instertion point
	 * @param timeOut	maximum work time
	 * @return			<code>TRUE</code> no errors; <code>FALSE</code> otherwise 
	 */
//	virtual bool findLocalTrajectory(const golem::Controller::State &cbegin, golem::GenWorkspaceChainState::Seq::const_iterator wbegin, golem::GenWorkspaceChainState::Seq::const_iterator end, golem::Controller::Trajectory &trajectory, golem::Controller::Trajectory::iterator iter, golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
};

//------------------------------------------------------------------------------

void XMLData(RagGraphPlanner::Desc &val, golem::XMLContext* context, bool create = false);
//void XMLData(RagPathFinder::Desc &val, golem::XMLContext* context, bool create = false);
//void XMLData(RagGraphPlanner::RagPathFinderDesc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPAM_GRAPHPLANNER_H_*/
