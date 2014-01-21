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
#ifndef _SPAM_SPAM_ROBOT_H_
#define _SPAM_SPAM_ROBOT_H_

//------------------------------------------------------------------------------

#include <Grasp/Grasp/Robot.h>
#include <Spam/Spam/Heuristic.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Robot is the interafce to a robot (right arm + hand), sensing devices and objects. */
class Robot : public grasp::Robot {
public:
	typedef golem::shared_ptr<Desc> Ptr;
	
	/** Robot factory */
	class Desc : public grasp::Robot::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(Robot, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Enables/disables object to be static */
		bool staticObject;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Robot::Desc::setToDefault();
			staticObject = false;
//			physPlannerDesc->pPlannerDesc->pHeuristicDesc.reset(new FTDrivenHeuristic::Desc);
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!grasp::Robot::Desc::isValid())
				return false;
			return true;
		}
		/** virtual destructor is required */
		virtual ~Desc() {
		}
	};
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
//	virtual void createTrajectory(const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);

	/** Checks and returns triggered guards for the hand */
	int getTriggeredGuards(std::vector<golem::Configspace::Index> &triggeredJoints, golem::Controller::State &state);
	/** Checks if triggered and return in case a vector of indeces */
	virtual int getTriggeredGuards(std::vector<grasp::FTGuard> &triggeredGuards, golem::Controller::State &state);

	/** Reads torque/force values from the state */
	void readFT(const golem::Controller::State &state, grasp::RealSeq &force) const;

	/** Activates collision detection with group of bounds */
	inline void setCollisionBoundsGroup(golem::U32 collisionGroup) {
		physPlanner->setCollisionBoundsGroup(collisionGroup);
	}
	/** Actives/deactivates collision detection during planning */
	inline void setCollisionDetection(bool collisionDetection) {
		pFTDrivenHeuristic->setCollisionDetection(collisionDetection);
	}

	/** Checks if the object is in the hand */
	size_t isGrasping(std::vector<golem::Configspace::Index> &triggeredJoints, golem::Controller::State &state);
	/** Checks if the object is in the hand */
	size_t isGrasping(std::vector<grasp::FTGuard> &triggeredJoints, golem::Controller::State &state);

	/** Finds a target in configuration space in a new reference frame */
	void findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend);

	/** Initialises the cycle */
	inline void initControlCycle() { 
		arm->initControlCycle();
		hand->initControlCycle();
	}

protected:
	/** Force/torque driven heuristic for robot controller */
	FTDrivenHeuristic* pFTDrivenHeuristic;
	/** Enables/disables object to be static */
	bool staticObject;

	// golem::Object interface
	virtual void render();

	// construction
	Robot(golem::Scene &scene);
	bool create(const Desc& desc);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPAM_SPAM_ROBOT_H_*/
