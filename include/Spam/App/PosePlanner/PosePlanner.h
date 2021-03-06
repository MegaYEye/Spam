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

#include <Grasp/App/Player/Player.h>
#include <Grasp/Core/Ctrl.h>
#include <Grasp/Contact/Model.h>
#include <Grasp/Contact/Query.h>
#include <Spam/App/PosePlanner/Data.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** SPAM Pose planner. */
class PosePlanner : public grasp::Player {
public:
	/** Callback to handle a ground truth pose of the object */
	typedef std::function<void(const grasp::Cloud::PointSeq&)> SimulateHandler;

	/** Action types */
	enum  action {
		NONE_ACTION = 0,
		GRASP,
		IG_PLAN_ON_QUERY,
		IG_PLAN_M2Q,
		IG_PLAN_LIFT,
		IG_TRAJ_OPT
	};
	/** Prints actions */
	static std::string actionToString(action type) {
		std::string str;
		switch (type) {
		case action::NONE_ACTION:
			str.assign("NONE");
			break;
		case action::GRASP:
			str.assign("GRASP");
			break;
		case action::IG_PLAN_M2Q:
			str.assign("IG_PLAN_M2Q");
			break;
		case action::IG_PLAN_ON_QUERY:
			str.assign("IG_PLAN_ON_QUERY");
			break;
		case action::IG_TRAJ_OPT:
			str.assign("IG_TRAJ_OPT");
			break;
		case action::IG_PLAN_LIFT:
			str.assign("IG_PLAN_LIFT");
			break;
		}

		return str;
	}

	/** Type of implemented algorithms */
	enum Strategy {
		NONE_STRATEGY = 0,
		ELEMENTARY,
		MYCROFT,
		IR3NE,
	};

	/** Data */
	class Data : public grasp::Player::Data {
	public:
		friend class PosePlanner;

		/** Mode */
		enum Mode {
			/** DEFAULT */
			MODE_DEFAULT,
			/** Model data */
			MODE_MODEL,
			/** Query density */
			MODE_QUERY,
		};

		/** Mode name */
		static const std::string ModeName[MODE_QUERY + 1];

		/** Data bundle description */
		class Desc : public grasp::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Data bundle default name */
		std::string dataName;

		/** Current Mode */
		Mode mode;

		/** Query frame */
		golem::Mat34 modelFrame;
		/** Model points */
		grasp::Cloud::PointSeq modelPoints;

		/** Query transformation */
		golem::Mat34 queryTransform;
		/** Query frame: queryTransform * modelFrame */
		golem::Mat34 queryFrame;
		/** Query points */
		grasp::Cloud::PointSeq queryPoints;
		/** Simulated location of the object */
		grasp::Cloud::PointSeq simulateObjectPose;

		/** Belief file name extension */
		std::string extSamples;

		/** type of action to execute */
		action actionType;

		/** Strategy to execute */
		Strategy stratType;

		/** Manager */
		virtual void setOwner(grasp::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Demo */
		PosePlanner* owner;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	/** Pose planner description */
	class Desc : public grasp::Player::Desc {
	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(PosePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Data bundle default name */
		std::string dataName;

		/** Pose distribution */
		Belief::Desc::Ptr pBeliefDesc;


		/** Model pose estimation camera */
		std::string modelCamera;
		/** Model data handler (scan) */
		std::string modelHandlerScan;
		/** Model data handler */
		std::string modelHandler;
		/** Model data item */
		std::string modelItem;
		/** Model data item object */
		std::string modelItemObj;
		/** Model data handler */
		std::string modelGraspHandler;
		/** Model data item */
		std::string modelGraspItem;

		/** Model scan pose */
		grasp::ConfigMat34::Seq modelScanPoseSeq;

		/** Model trajectory handler */
		std::string modelHandlerTrj;
		/** Model trajectory item */
		std::string modelItemTrj;

		/** Query pose estimation camera */
		std::string queryCamera;
		/** Model data handler (scan) */
		std::string queryHandlerScan;
		/** Query data handler */
		std::string queryHandler;
		/** Query data item */
		std::string queryItem;
		/** Query data item object */
		std::string queryItemObj;
		/** Query data handler */
		std::string queryGraspHandler;
		/** Query data item */
		std::string queryGraspItem;

		/** Object scan pose */
		grasp::ConfigMat34::Seq queryScanPoseSeq;

		/** Query trajectory handler */
		std::string queryHandlerTrj;
		/** Query trajectory item */
		std::string queryItemTrj;

		/** Pose estimation for simulated object */
		grasp::RBPose::Desc::Ptr simRBPoseDesc;
		/** Ground truth data item */
		std::string simulateItem;
		/** Ground truth data handler */
		std::string simulateHandler;

		/** Belief data handler */
		std::string beliefHandler;
		/** Belief data item */
		std::string beliefItem;

		/** Model descriptions */
		grasp::Model::Desc::Map modelDescMap;
		/** Contact appearance */
		grasp::Contact3D::Appearance contactAppearance;

		/** Query descriptions */
		grasp::Query::Desc::Map queryDescMap;

		/** Grasp force sensor */
		std::string graspSensorForce;
		/** Grasp force threshold */
		golem::Twist graspThresholdForce;
		/** Grasp force event - hand close time wait */
		golem::SecTmReal graspEventTimeWait;
		/** Grasp hand close duration */
		golem::SecTmReal graspCloseDuration;
		/** Grasp pose open (pre-grasp) */
		grasp::ConfigMat34 graspPoseOpen;
		/** Grasp pose closed (grasp) */
		grasp::ConfigMat34 graspPoseClosed;

		/** Manipulator description */
		grasp::Manipulator::Desc::Ptr manipulatorDesc;
		/** Manipulator Appearance */
		grasp::Manipulator::Appearance manipulatorAppearance;

		/** Appereance for point clouds: hypothesis point clouds */
		grasp::Cloud::Appearance hypothesisAppearance;
		/** Appereance for point clouds: debug point clouds */
		grasp::Cloud::Appearance meanposeAppearance;
		/** Appereance for point clouds: ground truth point clouds */
		grasp::Cloud::Appearance groundTruthAppearance;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Player::Desc::setToDefault();

			dataDesc.reset(new Data::Desc);
			dataName.clear();

			pBeliefDesc.reset(new Belief::Desc);

			modelCamera.clear();
			modelHandlerScan.clear();
			modelHandler.clear();
			modelItem.clear();
			modelItemObj.clear();
			modelGraspHandler.clear();
			modelGraspItem.clear();

			modelScanPoseSeq.clear();

			modelHandlerTrj.clear();
			modelItemTrj.clear();

			queryCamera.clear();
			queryHandlerScan.clear();
			queryHandler.clear();
			queryItem.clear();
			queryItemObj.clear();
			queryGraspHandler.clear();
			queryGraspItem.clear();

			queryScanPoseSeq.clear();

			queryHandlerTrj.clear();
			queryItemTrj.clear();

			simRBPoseDesc.reset(new grasp::RBPose::Desc);
			simulateItem.clear();
			simulateHandler.clear();

			beliefHandler.clear();
			beliefItem.clear();

			modelDescMap.clear();
			contactAppearance.setToDefault();

			queryDescMap.clear();

			graspSensorForce.clear();
			graspThresholdForce.setZero();
			graspEventTimeWait = golem::SecTmReal(2.0);
			graspCloseDuration = golem::SecTmReal(2.0);
			graspPoseOpen.setToDefault();
			graspPoseClosed.setToDefault();

			manipulatorDesc.reset(new grasp::Manipulator::Desc);
			manipulatorAppearance.setToDefault();

			hypothesisAppearance.setToDefault();
			meanposeAppearance.setToDefault();
			groundTruthAppearance.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			Player::Desc::assertValid(ac);
			
			grasp::Assert::valid(dataDesc != nullptr && grasp::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			grasp::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");
			pBeliefDesc->assertValid(grasp::Assert::Context(ac, "Belief desc: invalid"));

			grasp::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(grasp::Assert::Context(ac, "manipulatorDesc->"));
			manipulatorAppearance.assertValid(grasp::Assert::Context(ac, "manipulatorAppearance."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Reset belief state */
	inline void createBeliefState() {
		pBelief = myDesc.pBeliefDesc->create(context);  // throws
	}

protected:
	/** Generator of pseudo random numbers */
	golem::Rand rand;

	/** Descriptor file */
	Desc myDesc;

	/** Appereance for point clouds: debug point clouds */
	grasp::Cloud::Appearance hypothesisAppearance, meanposeAppeareance, groundtruthAppearance;
	bool showMeanPoseOnly, showSimulate;
	/** Callback to handle simulate ground truth */
	SimulateHandler simulateHandlerCallback;

	/** Force to draw the belief state */
	bool drawBeliefState;
	/** Pointer to the current belief state */
	grasp::data::Item::Map::iterator currentBeliefPtr;
	/** Belief data handler */
	grasp::data::Handler* beliefHandler;
	/** Belief data item */
	std::string beliefItem;
	/** Current belief data item */
	std::string currentBeliefItem;
	/** Pose distribution */
	Belief::Ptr pBelief; //grasp::RBPose::Ptr pRBPose;

	/** Model pose estimation camera */
	grasp::Camera* modelCamera;
	/** Model data handler (scan) */
	grasp::data::Handler* modelHandlerScan;
	/** Model data handler */
	grasp::data::Handler* modelHandler;
	/** Model data item */
	std::string modelItem;
	/** Model data item object */
	std::string modelItemObj;
	/** Model data handler */
	grasp::data::Handler* modelGraspHandler;
	/** Model data item */
	std::string modelGraspItem;
	/** Model scan pose */
	grasp::ConfigMat34::Seq modelScanPoseSeq;

	/** Model trajectory handler */
	grasp::data::Handler* modelHandlerTrj;
	/** Model trajectory item */
	std::string modelItemTrj;

	/** Query pose estimation camera */
	grasp::Camera* queryCamera;
	/** Query data handler (scan) */
	grasp::data::Handler* queryHandlerScan;
	/** Query data handler */
	grasp::data::Handler* queryHandler;
	/** Query data item */
	std::string queryItem;
	/** Query data item object */
	std::string queryItemObj;
	/** Query data handler */
	grasp::data::Handler* queryGraspHandler;
	/** Query data item */
	std::string queryGraspItem;
	/** Query scan pose */
	grasp::ConfigMat34::Seq queryScanPoseSeq;

	/** Query trajectory handler */
	grasp::data::Handler* queryHandlerTrj;
	/** Query trajectory item */
	std::string queryItemTrj;

	/** Grasp force sensor */
	grasp::FT* graspSensorForce;
	/** Grasp force threshold */
	golem::Twist graspThresholdForce;
	/** Grasp force event - hand close time wait */
	golem::SecTmReal graspEventTimeWait;
	/** Grasp hand close duration */
	golem::SecTmReal graspCloseDuration;
	/** Grasp pose open (pre-grasp) */
	grasp::ConfigMat34 graspPoseOpen;
	/** Grasp pose closed (grasp) */
	grasp::ConfigMat34 graspPoseClosed;

	/** Accurate pose estimation for the simulated object */
	grasp::RBPose::Ptr simRBPose;
	/** Reference frames */
	golem::Mat34 simModelFrame, simQueryFrame;
	/** Query data item */
	std::string simulateItem;
	/** Ground truth data handler */
	grasp::data::Handler* simulateHandler;

	/** Models */
	grasp::Model::Map modelMap;
	/** Contact appearance */
	grasp::Contact3D::Appearance contactAppearance;

	/** Query densities */
	grasp::Query::Map queryMap;

	/** Manipulator */
	grasp::Manipulator::Ptr manipulator;
	/** Manipulator Appearance */
	grasp::Manipulator::Appearance manipulatorAppearance;

	/** Distribution num of samples */
	size_t distribSamples;

	/** Query views (random selected) */
	size_t queryViews;

	/** Model data */
	Data::Map::iterator modelDataPtr;
	/** Model frame */
	golem::Mat34 modelFrame;
	/** Model points */
	grasp::Cloud::PointSeq modelPoints;

	golem::Bounds::Seq handBounds;
	golem::DebugRenderer debugRenderer;

	/** Smart pointer to the ft driven heuristic */
	FTDrivenHeuristic* pHeuristic;

	/** Pose estimation */
	grasp::data::Item::Map::iterator estimatePose(const Data::Mode mode, std::string &itemName);
	/** Grasp and capture object */
	grasp::data::Item::Map::iterator objectCapture(const Data::Mode mode, std::string &itemName);
	/** Process object image and add to data bundle */
	grasp::data::Item::Map::iterator objectProcess(const Data::Mode mode, grasp::data::Item::Map::iterator ptr);
	/** Retrieve a point cloud from the data budle */
	grasp::Cloud::PointSeq getPoints(Data::Map::iterator dataPtr, const std::string &itemName) const;

	/** Reset data pointers */
	void resetDataPointers();

	inline golem::Controller::State lookupState(golem::SecTmReal time = golem::SEC_TM_REAL_MAX) const {
		golem::Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		controller->lookupState(time, dflt);
		dflt.cvel.setToDefault(info.getJoints());
		dflt.cacc.setToDefault(info.getJoints());
		return dflt;
	}

	inline golem::Controller::State lookupCommand(golem::SecTmReal time = golem::SEC_TM_REAL_MAX) const {
		golem::Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		controller->lookupCommand(time, dflt);
		dflt.cvel.setToDefault(info.getJoints());
		dflt.cacc.setToDefault(info.getJoints());
		return dflt;
	}

	virtual void render() const;

	bool create(const Desc& desc);
	PosePlanner(golem::Scene &scene);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
