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

#include <Grasp/Core/Robot.h>
#include <Spam/Spam/Heuristic.h>
#include <Grasp/Core/Cloud.h>
#include <Spam/Spam/GraphPlanner.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** General active ctroller debug info */
std::string controllerDebug(grasp::ActiveCtrl &ctrl);

//------------------------------------------------------------------------------

/** Robot is the interafce to a robot (right arm + hand), sensing devices and objects. */
class Robot : public grasp::Robot {
public:
	typedef golem::shared_ptr<Desc> Ptr;
	
	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
	typedef std::function<void(const golem::Controller::State&, grasp::RealSeq&, std::vector<golem::Configspace::Index>&)> GuardsReader;

	/** Robot factory */
	class Desc : public grasp::Robot::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(Robot, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Robot::Desc::setToDefault();
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

	/** (Global search) trajectory of the entire robot from the configuration space and/or workspace target */
	virtual void createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	grasp::RBDist trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	grasp::RBDist findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);

	/** Returns a seq of triggered guards if any */
//	FTGuard::Seq getTriggeredGuard() { return triggeredGuards; };

	///** Checks if triggered and return in case a vector of indeces */
	//virtual int getTriggeredGuards(FTGuard::Seq &triggeredGuards, golem::Controller::State &state);

	///** Checks guards on justin and bham robot */
	//int checkGuards(std::vector<int> &triggeredGuards, golem::Controller::State &state);

	///** Reads torque/force values from the state */
	void readFT(const golem::Controller::State &state, grasp::RealSeq &force) const;

	/** Activates collision detection with group of bounds */
	inline void setCollisionBoundsGroup(golem::U32 collisionGroup) {
		physPlanner->setCollisionBoundsGroup(collisionGroup);
	}
	/** Actives/deactivates collision detection during planning */
	inline void setCollisionDetection(bool collisionDetection) {
		pFTDrivenHeuristic->setCollisionDetection(collisionDetection);
	}

	///** Checks if the object is in the hand */
	//size_t isGrasping(FTGuard::Seq &triggeredJoints, golem::Controller::State &state);

	/** Finds a target in configuration space in a new reference frame */
	void findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting = false);

	/** Initialises the cycle */
	inline void initControlCycle() { 
		arm->initControlCycle();
		hand->initControlCycle();
	}

	/** Stop active controller */
	void stopActiveController();
	/** Activate active controller */
	void startActiveController(const golem::I32 steps = grasp::ActiveCtrl::STEP_DEFAULT);
	/** Emergency mode for the controller */
	void emergencyActiveController();
	/** Checks for reions with high likelihood of contacts */
	inline bool expectedCollisions(const golem::Controller::State& state) const {
		return pFTDrivenHeuristic->expectedCollisions(state);
	}

	inline golem::Real getFilterForce(const golem::I32 i) {
		return handFilteredForce[i];
	}
	inline grasp::RealSeq getFilterForce() {
		return handFilteredForce;
	}

	/** Force reader */
	GuardsReader guardsReader;
	/** Collect history of FT readings for statistics */
	grasp::ActiveCtrl::ForceReader collectFTInp;
	/** FT 2-oreder high-pass filter */
	grasp::ActiveCtrl::ForceReader ftFilter;

	//golem::shared_ptr<grasp::Cloud::PointSeq> objectPointCloudPtr;
	//Collision::Ptr collision;
	//Collision::Waypoint w;
	///** Manipulator pointer */
	//grasp::Manipulator::Ptr manipulator;
	///** Acquires manipulator */
	//inline void setManipulator(grasp::Manipulator *ptr) { manipulator.reset(ptr); collision.reset(new Collision(context, *manipulator)); };
	//golem::Rand rand;
protected:
	/** Force/torque driven heuristic for robot controller */
	FTDrivenHeuristic* pFTDrivenHeuristic;

	/** Number of averaging steps before stopping collecting readings */
	golem::U32 windowSize;
	golem::I32 steps;
	// gaussian filter mask
	grasp::RealSeq mask;
	// computes gaussian
	template <typename _Real> inline _Real N(const _Real mean, const _Real stdev) {
		const _Real norm = golem::numeric_const<_Real>::ONE / (stdev*Math::sqrt(2 * golem::numeric_const<_Real>::PI));
		return norm*golem::Math::exp(-.5*Math::sqr(_Real(mean) / _Real(stdev))); // gaussian
	}
	// computes guassian on a vector
	template <typename _Ptr, typename _Real> inline std::vector<_Real> N(_Ptr begin, _Ptr end, const size_t dim, const _Real stdev) {
		std::vector<_Real> output;
		output.assign(dim, golem::numeric_const<_Real>::ZERO);
		size_t idx = 0;
		for (Ptr i = begin; i != end; ++i) {
			output[idx++] = N(*i, stdev);
		}
		return output;
	}


	/** Input force at sensor, sequence */
	std::vector<grasp::RealSeq> forceInpSensorSeq;
	/** Filtered forces for the hand */
	grasp::RealSeq handFilteredForce;
	/** force reader access cs */
	golem::CriticalSection csHandForce;


	inline golem::I32 dimensions() { return (golem::I32)handInfo.getJoints().size(); }

	/** Trigguered F/T guards */
	FTGuard::Seq triggeredGuards;

	/** Robot descriptor */
	Desc desc;
	// golem::Object interface
	virtual void render();

	// construction
	Robot(golem::Scene &scene);
	bool create(const Desc& desc);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(spam::Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPAM_SPAM_ROBOT_H_*/
