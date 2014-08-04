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
//! @Date:     25/03/2014
//------------------------------------------------------------------------------
#pragma once
#ifndef _SPAM_POSEPLANNER_POSEPLANNER_H_
#define _SPAM_POSEPLANNER_POSEPLANNER_H_

//------------------------------------------------------------------------------

#include <Grasp/Player/Player.h>
#include <Spam/Spam/Belief.h>
#include <Golem/Phys/Application.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** SPAM Pose planner. */
class PosePlanner : public grasp::Player {
public:
	/** Data */
	class Data : public grasp::Player::Data {
	public:
		friend class PosePlanner;

		/** Action frame */
//		golem::Mat34 actionFrame;
		/** Query frame */
		golem::Mat34 modelFrame;
		/** Query transformation */
		golem::Mat34 queryTransform;
		/** Query frame */
		golem::Mat34 queryFrame;
		/** Model points */
		grasp::Cloud::PointSeq queryPoints;

		/** High dim rep pose distribution **/
		grasp::RBPose::Sample::Seq poses;
		/** Low dim rep pose distribution **/
		grasp::RBPose::Sample::Seq hypotheses;

		/** Belief file name extension */
		std::string extSamples;
		
		/** Reset data during construction */
		Data() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Data::setToDefault();
			
//			actionFrame.setId();
			queryTransform.setId();
			queryFrame.setId();
			queryPoints.clear();

			poses.clear();
			hypotheses.clear();

			extSamples = ".rbs";
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			Player::Data::assertValid(ac);
		}

		/** Reads/writes object from/to a given XML context */
		virtual void xmlData(golem::XMLContext* context, bool create = false) const;

		/** Creates new data */
		virtual Ptr clone() const;
	};

	/** Pose planner description */
	class Desc : public grasp::Player::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(PosePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Pose distribution */
		grasp::RBPose::Desc::Ptr pRBPoseDesc;
		/** Model feature appearance */
		grasp::RBPose::Feature::Appearance modelAppearance;
		/** Query feature appearance */
		grasp::RBPose::Feature::Appearance queryAppearance;
		/** Feature frame size */
		golem::Vec3 featureFrameSize;
		/** Distribution frame size */
		golem::Vec3 distribFrameSize;
		/** Distribution num of samples */
		size_t distribSamples;

		///** Distribution num of poses **/
		//size_t numPoses;
		///** Distibution num of hypotheses **/
		//size_t numHypotheses;

		/** Default manipulation action */
		grasp::RobotPose actionManip;

		/** Model point transformation **/
		golem::Mat34 modelTrn;

		/** Enables/disable screen capture from simulation */
//		bool screenCapture;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Player::Desc::setToDefault();

			data.reset(new Data);

			pRBPoseDesc.reset(new grasp::RBPose::Desc);
			modelAppearance.setToDefault();
			queryAppearance.setToDefault();
			featureFrameSize.set(golem::Real(0.1));
			distribFrameSize.set(golem::Real(0.01));
			distribSamples = 100;

			actionManip.configspace = false;
			actionManip.workspace = true;
			actionManip.setId();
			actionManip.p.z += -golem::Real(0.15);

			//numPoses = 100;
			//numHypotheses = 5;

			modelTrn.setToDefault();

//			screenCapture = false;
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Player::Desc::isValid())
				return false;
			
			if (pRBPoseDesc == NULL || !pRBPoseDesc->isValid())
				return false;
			if (!modelAppearance.isValid() || !queryAppearance.isValid() || !featureFrameSize.isPositive() || !distribFrameSize.isPositive())
				return false;
			actionManip.assertValid(grasp::Assert::Context("grasp::PosePlanner::Desc::isValid(): actionManip."));

			return true;
		}
	};

protected:
	/** Generator of pseudo random numbers */
	golem::Rand rand;

	/** Pose distribution */
	grasp::RBPose::Ptr pRBPose;
	Belief* pBelief; //grasp::RBPose::Ptr pRBPose;

	/** Model feature appearance */
	grasp::RBPose::Feature::Appearance modelAppearance;
	/** Query feature appearance */
	grasp::RBPose::Feature::Appearance queryAppearance;
	/** Feature frame size */
	golem::Vec3 featureFrameSize;
	/** Distribution frame size */
	golem::Vec3 distribFrameSize;
	/** Distribution num of samples */
	size_t distribSamples;

	/** Default manipulation action */
	grasp::RobotPose actionManip;

	/** Feature renderer */
	golem::DebugRenderer pointFeatureRenderer;
	/** Particles renderer */
	golem::DebugRenderer sampleRenderer;
	golem::DebugRenderer testRenderer;


	/** Model data */
	Data::Map::iterator modelDataPtr;
	/** Model frame */
	golem::Mat34 modelFrame;
	/** Model points */
	grasp::Cloud::PointSeq modelPoints;
	/** Model transformation frame **/
	golem::Mat34 modelTrn;

	/** Query data */
	Data::Map::iterator queryDataPtr;

		/** 3d surface samples' point feature appearance */
	grasp::Director::Data::Appearance sampleAppearance;
	/** Show model points */
	bool showModelPoints;
	/** Show model features */
	bool showModelFeatures;
	/** Show query distribution */
	bool showQueryDistrib;
	/** Show pose distribution **/
	bool showPoseDistrib;
	/** Show hypothesis distribution **/
	bool showSamplePoints;
	/** Displayed feature index */
	golem::U32 featureIndex;

	///** Distribution num of poses **/
	//size_t numPoses;
	///** Distibution num of hypotheses **/
	//size_t numHypotheses;

	/** Enables/disable screen capture from simulation */
	bool screenCapture;

	///** Profile state sequence */
	//virtual void profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const;
	///** Profile state sequence */
	//virtual void profileManip(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const;

	/** Creates new data */
	virtual Data::Ptr createData() const;

	/** Create trajectory */
	virtual void createWithManipulation(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent = false) const;
	/** Create trajectory */
	virtual void create(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent = false) const;

	/** Render data */
	virtual void renderData(Data::Map::const_iterator dataPtr);
	/** Render belief state */
	void renderUncertainty(const grasp::RBPose::Sample::Seq &samples);
	golem::DebugRenderer uncRenderer;
	/** Reset data pointers */
	void resetDataPointers();

	/** Point selection */
	grasp::Cloud::PointSeqMap::iterator getPointsTrn(Data::Map::iterator dataPtr, golem::Mat34 &trn);

	/** User interface: menu function */
	virtual void function(Data::Map::iterator& dataPtr, int key);

	virtual void render();

	PosePlanner(golem::Scene &scene);
	bool create(const Desc& desc);
};

/** Reads/writes object from/to a given XML context */
void XMLData(PosePlanner::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** PosePlanner application */
class PosePlannerApp : public golem::Application {
protected:
	/** Runs Application */
	virtual void run(int argc, char *argv[]);
};

//------------------------------------------------------------------------------

};	// namespace

namespace golem {

template <> void Stream::read(spam::PosePlanner::Data &data) const;
template <> void Stream::write(const spam::PosePlanner::Data &data);

};	// namespace


#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
