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
#ifndef _SPAM_RAGPLANNER_SHAPEPLANNER_H_
#define _SPAM_RAGPLANNER_SHAPEPLANNER_H_

//------------------------------------------------------------------------------

#include <Grasp/PosePlanner/PosePlanner.h>
#include <Grasp/Grasp/Grasp.h>
#include <Spam/Spam/Belief.h>
#include <Spam/Spam/Data.h>

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
class RagPlanner : public grasp::PosePlanner {
public:
	class Desc : public grasp::PosePlanner::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(RagPlanner, golem::Object::Ptr, golem::Scene&)

	public:
		/** Smart pointer */
		typedef golem::shared_ptr<Desc> Ptr;

		/** Default number of sampled object poses */
		size_t K;
		/** Minimum threshold required for the sampled importance weights */
		golem::Real theta;
		/** Maximum number of sampling iterations */
		size_t maxSamplingIter;

		/** Backward moving factor */
		golem::Real timestampFac;

		/** 3D surface samples' point feature appearance */
		grasp::Director::Data::Appearance sampleAppearance;

		/** Enables/disables explicit use of uncertainty in planning */
		bool uncEnable;
		/** Enables/disables replanning */
		bool singleGrasp;
		/** Enables/disables withdrawing to the home pose */
		bool withdrawToHomePose;

		/** Bounding box for the graspable object */
		golem::Bounds::Desc::Seq objectBounds;
		/** Real pose of the object. Ground truth for testing */
		golem::Mat34 objectPose;
		
		/** Enables/disables adding noise to the estimated (query) point cloud */
		bool gtNoiseEnable;
		/** Noise lin/ang components */
		grasp::RBDist gtPoseStddev;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::PosePlanner::Desc::setToDefault();
			robotDesc.reset(new Robot::Desc);
			pRBPoseDesc.reset(new Belief::Desc);
			K = 5;
			theta = golem::Real(0.20);
			maxSamplingIter = 100;
			timestampFac = 0.25;
			//pointAppearance.setToDefault();
			data->appearance.setToDefault();
			uncEnable = true;
			singleGrasp = false;
			withdrawToHomePose = false;
			objectBounds.clear();
			objectPose.setId();
			gtNoiseEnable = false;
			gtPoseStddev.set();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!grasp::PosePlanner::Desc::isValid())
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = objectBounds.begin(); i != objectBounds.end(); ++i)
				if (!(*i)->isValid())
					return false;
			return true;
		}
	};

	/** Returns the boundig box for the object to be grasped */
	inline golem::Bounds::SeqPtr getObjectBounds() const { 
		golem::Bounds::SeqPtr pBounds(new golem::Bounds::Seq());
		for (golem::Bounds::Desc::Seq::iterator i = objectBounds->begin(); i != objectBounds->end(); ++i)
			pBounds->push_back((*i)->create());
		return pBounds; 
	}

	virtual ~RagPlanner();

protected:
	/** RAG robot */
	Robot *robot;
	/** Particles renderer */
	golem::DebugRenderer sampleRenderer;
	/** ground truth renderer */
	golem::DebugRenderer gtRenderer;

	/** Smart pointer to the belief state */
	Belief::Ptr pBelief;
	/** Smart pointer to the ft driven heuristic */
	FTDrivenHeuristic::Ptr pHeuristic;

	/** Data copllector */
	TrialData::Map dataMap;
	/** Object pose data */
	TrialData::Map::iterator poseDataPtr;
	
	/** Description file */
	Desc ragDesc;

	/** Maximum likelihood pose */
	grasp::RBPose::Sample mlFrame;
	/** Initial ML estimation */
	golem::Mat34 initActionFrame;
	/** Default number of sampled object poses */
	size_t K;
	/** Minimum threshold required for the sampled importance weights */
	golem::Real theta;
	/** Maximum number of sampling iterations */
	size_t maxSamplingIter;
	/** Backward moving factor */
	golem::Real timestampFac;
	/** 3d surface samples' point feature appearance */
	grasp::Director::Data::Appearance sampleAppearance;
	/** Show original colour of the point cloud */
	bool showSampleColour;
	/** Show Sample frame */
	bool showSampleFrame;
	/** Show sample points */
	bool showSamplePoints;
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
	/** Ground truth points */
	grasp::Cloud::PointSeq groundTruthPoints;

	/** Enables/disables the transformation in the action frame */
	bool trnEnable;

	grasp::Manipulator::Ptr manipulator;
	golem::Bounds::Seq handBounds;

	/** Sent trajectory */
	golem::Controller::State::Seq currentTraj;

	/** Bounding box for the graspable object */
	golem::Bounds::Desc::SeqPtr objectBounds;

	/** Object real pose on the scene (ground truth) */
	grasp::RBCoord objectRealPose;
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

	/** Generates high dimension pdf */
	void generate(grasp::RBPose::Sample::Seq::const_iterator begin, grasp::RBPose::Sample::Seq::const_iterator end, TrialData::Map::iterator dataPtr, grasp::RBPose::Sample::Seq &candidates) const;
	/** Updates belief state */
	void updateAndResample(TrialData::Map::iterator dataPtr);
//	void updateAndResample(const std::vector<golem::Configspace::Index> &triggeredGuards, const golem::Controller::State &pose, TrialData::Map::iterator dataPtr, grasp::RBPose::Sample::Seq &candidates) const;
	/** Evaluate the likelihood of reading a contact between robot's pose and the sample */
	golem::Real evaluate(const grasp::RBCoord &Pose, const grasp::RBPose::Sample &sample) const;

	/** Builds and performs reach and grasp actions */
	virtual void performApproach(TrialData::Map::iterator dataPtr);
	/** Builds and performs a moving back trajectory */
	virtual void performWithdraw(TrialData::Map::iterator dataPtr);
	/** Builds and performs a moving back trajectory */
	virtual void performManip(TrialData::Map::iterator dataPtr);
	/** Builds and performs single attempt to grasp */
	virtual void performSingleGrasp(TrialData::Map::iterator dataPtr);
	/** Performs trial action (trajectory) */
	virtual void perform(TrialData::Map::iterator dataPtr);
	/** Profiles state sequence */
	virtual void profile(TrialData::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const;

	/** Render trial data */
	virtual void renderTrialData(TrialData::Map::const_iterator dataPtr);
	/** Overwrite pose planner render trial data */
	virtual void renderData(Data::Map::const_iterator dataPtr);
	void renderContacts();
	void renderPose(const golem::Mat34 &pose);
	golem::DebugRenderer testPose;
	void renderUpdate(const golem::Mat34 &pose, const grasp::RBPose::Sample::Seq &samples);
	golem::DebugRenderer testUpdate;
	void renderHand(const golem::Controller::State &state);
	golem::BoundsRenderer handRenderer;

	/** Prints out a trajectory */
	void printTrajectory(const golem::Controller::State::Seq &trajectory, const golem::Configspace::Index &begin, const golem::Configspace::Index &end) const;
	/** Prints out a state of the robot */
	void printState(const golem::Controller::State &state, const golem::Configspace::Index &begin, const golem::Configspace::Index &end, const std::string &label = "") const;

	/** User interface: menu function */
	virtual void function(Data::Map::iterator& dataPtr, int key);
	/** User interface: help message */
//	virtual std::string help() const;

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

#endif /*_SPAM_RAGPLANNER_SHAPEPLANNER_H_*/
