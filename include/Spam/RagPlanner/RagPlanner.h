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
#ifndef _SPAM_RAGPLANNER_RAGPLANNER_H_
#define _SPAM_RAGPLANNER_RAGPLANNER_H_

//------------------------------------------------------------------------------

#include <Spam/ShapePlanner/ShapePlanner.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** 
	RAGPlanner. 
	Basic class for planning Reach-and-grasp trajectory.
	This class provides (1) a sampling method to generate a low dimension
	representation of uncertainty over object poses, and (2) a planner which
	builds trajectories that are more likely to distinguish between the 
	miximum likelihood pose of the object and any sampled poses.
	Based on Platt R. et al. "A hypothesis-based algorithm for planning and
	control in non-Gaussian belief spaces", 2011.
*/
class RagPlanner : public ShapePlanner {
public:
	/** Data */
	class Data : public ShapePlanner::Data {
	public:
		friend class RagPlanner;

		/** Cache (local): OpenGL settings */
		golem::Scene::OpenGL openGL;
		
		/** Specifies if guard have been triggered during the perform of the action */
		int triggered;

		/** Specifies if the replanning should be triggered */
		bool replanning;
		/** Enable the release of the object before withdrawing */
		bool release;

		/** Reset data during construction */
		Data() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ShapePlanner::Data::setToDefault();
			triggered = 0;
			replanning = false;
			release = false;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			ShapePlanner::Data::assertValid(ac);
		}

		/** Reads/writes object from/to a given XML context */
		virtual void xmlData(golem::XMLContext* context, bool create = false) const;
	};

	class Desc : public ShapePlanner::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(RagPlanner, golem::Object::Ptr, golem::Scene&)

	public:
		/** Smart pointer */
		typedef golem::shared_ptr<Desc> Ptr;

		/** 3D surface samples' point feature appearance */
		grasp::Director::Data::Appearance sampleAppearance;

		/** Enables/disables explicit use of uncertainty in planning */
		bool uncEnable;
		/** Enables/disables replanning */
		bool singleGrasp;
		/** Enables/disables withdrawing to the home pose */
		bool withdrawToHomePose;

		/** Downsampling parameter */
		size_t maxModelPoints;

		/** Collision description file. Used for collision with the ground truth */
		Collision::Desc::Ptr objCollisionDescPtr;

		///** Query transformation */
		//grasp::RBCoord queryPointsTrn;
		
		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Desc::setToDefault();

			data.reset(new Data);

			robotDesc.reset(new Robot::Desc);
			pRBPoseDesc.reset(new Belief::Desc);

			data->appearance.setToDefault();

			uncEnable = true;
			singleGrasp = false;
			withdrawToHomePose = false;

			maxModelPoints = 5000;

			objCollisionDescPtr.reset(new Collision::Desc());

//			queryPointsTrn.fromMat34(golem::Mat34::identity());
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PosePlanner::Desc::isValid())
				return false;
			return true;
		}
	};

	/** Profile state sequence */
	virtual void profile(golem::SecTmReal duration, const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& out, const bool silent = false) const;

	virtual ~RagPlanner();

protected:
	/** RAG robot */
	Robot *robot;
	/** Particles renderer */
	golem::DebugRenderer sampleRenderer;
	/** ground truth renderer */
	golem::DebugRenderer gtRenderer;
	/** Debug renderer */
	golem::DebugRenderer debugRenderer;

	/** Smart pointer to the belief state */
//	Belief* pBelief;
	/** Smart pointer to the ft driven heuristic */
	FTDrivenHeuristic* pHeuristic;

	/** Object pose data */
	Data::Map::iterator poseDataPtr;
	
	/** Description file */
	Desc ragDesc;

	/** 3d surface samples' point feature appearance */
	grasp::Director::Data::Appearance sampleAppearance;
	/** Show original colour of the point cloud */
	bool showSampleColour;
	/** Show Sample frame */
	bool showSampleFrame;
	/** Show high dimension distribution */
	bool showDistribution;
	/** Show maximum likelihood estimation distribution */
	bool showMLEFrame;
	/** Show MLE points */
	bool showMLEPoints;
	/** Enables/disables esplicit use of uncertainty in planning */
	bool uncEnable;
	/** Enables/disables replanning */
	bool singleGrasp;
	/** Enables/disables withdrawing to the home pose */
	bool withdrawToHomePose;
	/** Shows the posterior distribution */
	bool posterior;

	bool printing;
	golem::Bounds::Seq handBounds, waypointBounds;

	/** Pointer to collision detection with the ground truth */
	Collision::Ptr collisionPtr;

	/** Resets the controllers */
	bool enableControllers;
	/** Checks collision with the query point cloud */
	bool enableSimContact;
	/** Enables/disables force reading */
	bool enableForceReading;
	bool forcereadersilent;
	bool contactOccured;

	/** Query transformation */
	grasp::RBCoord queryPointsTrn;

	/** Safety configurations of the robot */
	grasp::RobotState::Seq homeStates;
	/** Combined action waypoints */
	golem::Controller::State::Seq executedTrajectory;
	/** Contains the index of the triggered guards */
	FTGuard::Seq triggeredGuards;
	golem::Controller::State::Seq grasps;

	/** Iterations counter */
	size_t iterations;

	/** Checks the validity of a sample */
	template <typename _PTR> bool isValidSample(const grasp::RBPose::Sample &sample, _PTR begin, _PTR end, golem::Real eps = golem::REAL_EPS) const {
//		std::printf("particle frame<%.4f,%.4f,%.4f> w=%.5f\n", sample.p.x, sample.p.y, sample.p.z, sample.weight);
		//if (sample.weight < theta)
		//	return false;
		for (_PTR i = begin; i != end; ++i) {
			if (pRBPose->distance(sample, *i) < eps)
				return false;
		}
		return true;
	}

	/** Get current points transformed */
	grasp::Cloud::PointSeqMap::iterator getTrnPoints(Data::Map::iterator dataPtr, const golem::Mat34 &trn);
	/** Get current points transformed */
	grasp::Cloud::RawPointSeqMultiMap::iterator getTrnRawPoints(Data::Map::iterator dataPtr, const golem::Mat34 &trn);

	/** Simulates contacts between the robot's hand and object's partial point cloud */
	golem::Real simContacts(const golem::Bounds::Seq::const_iterator &begin, const golem::Bounds::Seq::const_iterator &end, const golem::Mat34 pose);
	/** Object real point cloud (testing purposes) */
	golem::shared_ptr<grasp::Cloud::PointSeq> objectPointCloudPtr;

	/** Updates belief state */
	void updateAndResample(Data::Map::iterator dataPtr);

	/** Builds and performs reach and grasp actions */
	virtual void performApproach(Data::Map::iterator dataPtr);
	/** Builds and performs a moving back trajectory */
	virtual void performWithdraw(Data::Map::iterator dataPtr);
	/** Builds and performs a moving back trajectory */
	virtual void performManip(Data::Map::iterator dataPtr);
	/** Builds and performs single attempt to grasp */
	virtual void performSingleGrasp(Data::Map::iterator dataPtr);
	/** Performs trial action (trajectory) */
	virtual void perform(const std::string& name, const golem::Controller::State::Seq& trajectory, bool silent = false);
	bool execute(Data::Map::iterator dataPtr, golem::Controller::State::Seq& trajectory);
	/** Creates new data */
	virtual Data::Ptr createData() const;

	/** Overwrite pose planner render trial data */
	virtual void renderData(Data::Map::const_iterator dataPtr);
	void renderContacts();
	void renderPose(const golem::Mat34 &pose);
	golem::DebugRenderer testPose;
	void renderUpdate(const golem::Mat34 &pose, const grasp::RBPose::Sample::Seq &samples);
	golem::DebugRenderer testUpdate;
	void renderHand(const golem::Controller::State &state, const golem::Bounds::Seq &bounds, bool clear = false);
	golem::BoundsRenderer handRenderer;
	void renderWaypoints(const golem::Bounds::Seq &bounds, bool clear = false);
	golem::BoundsRenderer waypointRenderer;

	// clear states
	grasp::Robot::State::Seq robotStates;

	bool unlockContact();
	bool showIndices;
	/** Prints out a trajectory */
	void printTrajectory(const golem::Controller::State::Seq &trajectory, const golem::Configspace::Index &begin, const golem::Configspace::Index &end) const;
	/** Prints out a state of the robot */
	void printState(const golem::Controller::State &state, const golem::Configspace::Index &begin, const golem::Configspace::Index &end, const std::string &label = "", const bool readForce = false) const;

	///** Shape planner demo */
	//void run(const Demo::Map::value_type& demo);

	/** User interface: menu function */
	virtual void function(Data::Map::iterator& dataPtr, int key);

	virtual void render();

	RagPlanner(golem::Scene &scene);
	bool create(const Desc& desc);
};

/** Reads/writes object from/to a given XML context */
void XMLData(RagPlanner::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** ShapePlanner application */
class RagPlannerApp : public golem::Application {
protected:
	/** Runs Application */
	virtual void run(int argc, char *argv[]);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPAM_RAGPLANNER_RAGPLANNER_H_*/
