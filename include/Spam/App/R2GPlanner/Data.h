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
//! @Date:     27/10/2012
//------------------------------------------------------------------------------
#pragma once
#ifndef _SPAM_RAGPLANNER_DATA_H_
#define _SPAM_RAGPLANNER_DATA_H_

//------------------------------------------------------------------------------

#include <Grasp/Core/Ctrl.h>
#include <Grasp/Core/Data.h>
#include <Grasp/Core/RB.h>
#include <Golem/Planner/Data.h>
#include <Golem/Planner/GraphPlanner/Data.h>

//------------------------------------------------------------------------------

namespace spam {
namespace data {

//------------------------------------------------------------------------------

/** Initialises handler.
*	(Optionally) Implemented by Handler.
*/
class HandlerR2GPlan {
public:
	/** Planner index. */
	virtual golem::U32 getPlannerIndex() const = 0;
	/** Sets planner and controllers. */
	virtual void set(golem::Planner& planner, const grasp::ControllerId::Seq& controllerIDSeq) = 0;
};

/** Trajectory collection and tools.
*	(Optionally) Implemented by Item.
*/
class R2GTrajectory {
public:
	enum Type {
		NONE = 0,
		APPROACH,
		ACTION
	};

	/** Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const grasp::Waypoint::Seq& waypoints) = 0;
	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const grasp::Waypoint::Seq& waypoints, const Type type) = 0;
	/** Returns waypoints without velocity profile. */
	virtual const grasp::Waypoint::Seq& getWaypoints() const = 0;
	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const grasp::Waypoint::Seq& getWaypoints(const Type type) const = 0;

	/** Returns command trajectory with velocity profile. */
	virtual void createTrajectory(golem::Controller::State::Seq& trajectory) = 0;
	/** (Mycroft) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const golem::Controller::State& begin, golem::Controller::State::Seq& trajectory) = 0;
	/** (IR3ne) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createIGTrajectory(const golem::Controller::State& begin, golem::Controller::State::Seq& trajectory) = 0;


	/** Last waypoint of the reach-to-grasp trajectory */
	size_t pregraspIdx;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_SPAM_RAGPLANNER_DATA_H_*/
