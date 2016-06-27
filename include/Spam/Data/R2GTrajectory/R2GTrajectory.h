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
#ifndef _SPAM_DATA_R2GPLANNER_H_
#define _SPAM_DATA_R2GPLANNER_H_

//------------------------------------------------------------------------------

#include <Spam/App/R2GPlanner/Data.h>
#include <Spam/HBPlan/Heuristic.h>
#include <Grasp/App/Manager/Data.h>
#include <Grasp/App/Player/Data.h>
#include <Golem/UI/Renderer.h>
#include <Grasp/Core/UI.h>
#include <Golem/UICtrl/Data.h>


//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* graspDescLoader(void);
};

//------------------------------------------------------------------------------

namespace spam {
namespace data {

//------------------------------------------------------------------------------

class ItemR2GTrajectory;
class HandlerR2GTrajectory;

/** Data item representing trajectory.
*/
class GOLEM_LIBRARY_DECLDIR ItemR2GTrajectory : public grasp::data::Item, public R2GTrajectory {
public:
	friend class HandlerR2GTrajectory;

	typedef std::map<R2GTrajectory::Type, grasp::Waypoint::Seq> WaypointMap;

	/** Waypoints collection */
	grasp::Waypoint::Seq waypoints;

	/** Waypoints collection map */
	WaypointMap waypointMap;

	/** Waypoints file */
	mutable grasp::data::File waypointFile;

	/** Path position */
	golem::Real pathPosition;

	/** Path waypoint */
	size_t pathWaypoint;
	/** Path interpolation */
	golem::Real pathInterpol;

	/** Path waypoint at contact */
	size_t contactPathWaypoint;
	/** Path interpolation at contact */
	golem::Real contactPathInterpol;

	/** Clones item. */
	virtual grasp::data::Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const grasp::Waypoint::Seq& waypoints);
	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const grasp::Waypoint::Seq& waypoints, const R2GTrajectory::Type type);

	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const grasp::Waypoint::Seq& getWaypoints() const;
	/** Trajectory: Returns waypoints without velocity profile. */
	virtual const grasp::Waypoint::Seq& getWaypoints(const R2GTrajectory::Type type) const;

	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(golem::Controller::State::Seq& trajectory);
	/** (Mycroft) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const golem::Controller::State& begin, golem::Controller::State::Seq& trajectory);
	/** (IR3ne) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createIGTrajectory(const golem::Controller::State& begin, golem::Controller::State::Seq& trajectory);

protected:
	/** Data handler */
	HandlerR2GTrajectory& handler;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemR2GTrajectory(HandlerR2GTrajectory& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerR2GTrajectory : public grasp::data::Handler, public grasp::UI, public HandlerR2GPlan, public grasp::data::Import, public grasp::data::Transform, public golem::Profile::CallbackDist {
public:
	friend class ItemR2GTrajectory;

	/** Bounds of kinematic chains */
	typedef golem::Chainspace::Coord<golem::Bounds::Seq> ChainBoundsSeq;
	/** Bounds poses of kinematic chains */
	typedef golem::Chainspace::Coord<grasp::Mat34Seq> ChainMat34Seq;
	/** Bounds of kinematic joints */
	typedef golem::Configspace::Coord<golem::Bounds::Seq> JointBoundsSeq;
	/** Bounds poses of kinematic joints */
	typedef golem::Configspace::Coord<grasp::Mat34Seq> JointMat34Seq;

	/** Import state */
	class GOLEM_LIBRARY_DECLDIR ImportState {
	public:
		/** Map */
		typedef std::vector<ImportState> Map;

		/** State variable pointer type */
		enum Type {
			/** Time stamp (Real) */
			TYPE_TIME = 0,
			/** Position (Configspace, Real) */
			TYPE_POSITION,
			/** Velocity (Configspace, Real) */
			TYPE_VELOCITY,
			/** Acceleration (Configspace, Real) */
			TYPE_ACCELERATION,
			/** Reserved area (Real) */
			TYPE_RESERVED,
		};

		/** State variable pointer type */
		Type type;
		/** Input pointer */
		golem::U32 inp;
		/** Output pointer */
		golem::U32 out;
		/** Offset */
		golem::Real offset;
		/** Scale */
		golem::Real scale;

		/** Input pointer */
		std::string inpStr;
		/** Output pointer */
		std::string outStr;

		/** Set to default */
		ImportState() {
			setToDefault();
		}
		/** Custom create */
		ImportState(Type type, golem::U32 inp, golem::U32 out, golem::Real offset, golem::Real scale) : type(type), inp(inp), out(out), offset(offset), scale(scale) {
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			type = TYPE_POSITION;
			inp = 0;
			out = 0;
			offset = golem::REAL_ZERO;
			scale = golem::REAL_ONE;
			inpStr.clear();
			outStr.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(type == TYPE_RESERVED ? out < (golem::U32)golem::Reservedspace::DIM : out < (golem::U32)golem::Configspace::DIM, ac, "out: not in range");
			grasp::Assert::valid(golem::Math::isFinite(offset), ac, "offset: invalid");
			grasp::Assert::valid(golem::Math::isFinite(scale), ac, "scale: invalid");
		}
		/** Assert that the description is valid. */
		static void assertValid(const grasp::Assert::Context& ac, const Map& map) {
			grasp::Assert::valid(!map.empty(), ac, ": empty");
			for (ImportState::Map::const_iterator i = map.begin(); i != map.end(); ++i)
				i->assertValid(grasp::Assert::Context(ac, "[i]."));
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Update controller state */
		void update(const grasp::RealSeq& data, golem::Controller::State& state) const;
		/** Update controller state */
		static void update(const Map& map, const grasp::RealSeq& data, golem::Controller::State& state);

		/** Extract rule */
		static void extract(const std::string& str, grasp::U32Seq& seq);
		/** Extract inp and out rules */
		void extract(Map& map) const;
		/** Extract inp and out rules */
		static void extract(const Map& inp, Map& out);
	};

	/** Import rigid body pose/velocity/acceleration */
	class GOLEM_LIBRARY_DECLDIR ImportFrame {
	public:
		/** Variable type */
		enum Type {
			/** Quaternion */
			TYPE_QUAT = 0,
			/** Euler angles */
			TYPE_EULER,
			/** Axis */
			TYPE_AXIS,
		};

		/** Variable type */
		Type type;
		/** Linear variable pointer */
		golem::U32 lin;
		/** Angular variable pointer */
		golem::U32 ang;

		/** Set to default */
		ImportFrame() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			type = TYPE_QUAT;
			lin = 0;
			ang = 3;
		}
		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);

		/** Update frame */
		void update(const grasp::RealSeq& data, golem::Mat34& trn) const;
	};

	/** Arm and hand setup */
	class GOLEM_LIBRARY_DECLDIR FactorDesc {
	public:
		/** Arm */
		golem::Real arm;
		/** Hand */
		golem::Real hand;
		/** Other controllers */
		golem::Real other;

		/** Set to default */
		FactorDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			arm = golem::REAL_ONE;
			hand = golem::REAL_ONE;
			other = golem::REAL_ONE;
		}
		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(golem::Math::isFinite(arm), ac, "arm: invalid");
			grasp::Assert::valid(golem::Math::isFinite(hand), ac, "hand: invalid");
			grasp::Assert::valid(golem::Math::isFinite(other), ac, "other: invalid");
		}
		/** Load descritpion from xml context. */
		void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Trajectory import */
	class GOLEM_LIBRARY_DECLDIR ImportRobotTrjDesc {
	public:
		/** Time interval */
		golem::Real interval;
		/** Begin waypoint */
		golem::U32 begin;
		/** End waypoint */
		golem::U32 end;
		/** Subsampling interval */
		golem::U32 subsampling;

		/** State map */
		ImportState::Map stateMap;
		/** Command map */
		ImportState::Map commandMap;

		/** Set to default */
		ImportRobotTrjDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			interval = golem::REAL_ONE;
			subsampling = 1;
			begin = 0;
			end = golem::U32(-1);
			stateMap.clear();
			commandMap.clear();
		}
		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(interval > golem::REAL_EPS, ac, "interval: < eps");
			grasp::Assert::valid(begin < end, ac, ": begin >= end");
			grasp::Assert::valid(subsampling > 0, ac, "subsampling: = 0");
			//ImportState::assertValid(grasp::Assert::Context(ac, "stateMap"), stateMap);
			//ImportState::assertValid(grasp::Assert::Context(ac, "commandMap"), commandMap);
		}
		/** Load descritpion from xml context. */
		void load(const golem::XMLContext* xmlcontext);
	};

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public grasp::data::Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Waypoint suffix */
		std::string waypointSuffix;

		/** Planner index */
		golem::U32 plannerIndex;

		/** Trajectory profile description */
		golem::Profile::Desc::Ptr profileDesc;
		/** Trajectory profile configspace distance multiplier */
		grasp::RealSeq distance;
		/** Trajectory extrapolation */
		grasp::RealSeq extrapolation;
		/** Trajectory motor command */
		grasp::RealSeq command;

		/** Velocity multiplication factor */
		FactorDesc velFac;
		/** Acceleration multiplication factor */
		FactorDesc accFac;
		/** Distance multiplication factor */
		FactorDesc disFac;
		/** Extrapolation multiplication factor */
		FactorDesc extFac;
		/** Command multiplication factor */
		FactorDesc cmdFac;

		/** Trajectory extrapolation */
		golem::Real trjExtrapolation;
		/** Trajectory duration */
		golem::Real trjDuration;
		/** IG Trajectory duration */
		golem::Real trjR2GDuration;
		/** Trajectory idle */
		golem::Real trjIdle;

		/** Action */
		grasp::Mat34Seq action;

		/** Bounds solid colour */
		golem::RGBA boundsSolidColour;
		/** Path renderer */
		golem::GraphRenderer pathRenderer;
		/** Path increment */
		golem::Real pathIncLarge;
		/** Path increment */
		golem::Real pathIncSmall;

		/** Show commands/states */
		bool showCommands;

		/** Robot trajectory import */
		ImportRobotTrjDesc importRobotTrj;
		/** Import from HDF5 dump file: file extension */
		std::string importHDF5FileExt;
		/** Import from HDF5 dump file: robot trajectory dataset */
		std::string importHDF5RobotTrj;

		/** Pregrasp hand pose */
		grasp::RealSeq handPregraspPose;
		/** Grasp hand pose */
		grasp::RealSeq handGraspPose;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			grasp::data::Handler::Desc::setToDefault();

			waypointSuffix = getFileExtWaypoint();

			plannerIndex = 0;

			profileDesc.reset(new golem::Profile::Desc);
			distance.assign(golem::Configspace::DIM, golem::REAL_ONE);
			extrapolation.assign(golem::Configspace::DIM, golem::REAL_ONE);
			command.assign(golem::Configspace::DIM, golem::REAL_ONE);

			velFac.setToDefault();
			accFac.setToDefault();
			disFac.setToDefault();
			extFac.setToDefault();
			cmdFac.setToDefault();

			trjExtrapolation = golem::Real(0.0);
			trjDuration = golem::Real(5.0);
			trjR2GDuration = golem::Real(20.0);
			trjIdle = golem::Real(1.0);

			action.clear();

			boundsSolidColour = golem::RGBA(192, 192, 0, 100);
			pathRenderer.setToDefault();
			pathRenderer.edgeShow = true;
			pathRenderer.show = true;
			pathIncLarge = golem::Real(0.05);
			pathIncSmall = golem::Real(0.005);

			showCommands = false;

			importRobotTrj.setToDefault();
			importHDF5FileExt = ".hdf5dump";
			importHDF5RobotTrj = "RobotTrajectory";

			handPregraspPose.assign(20, golem::REAL_ZERO);
			handGraspPose.assign(20, golem::REAL_ONE);
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::data::Handler::Desc::assertValid(ac);

			grasp::Assert::valid(waypointSuffix.length() > 0, ac, "waypointSuffix: empty");

			grasp::Assert::valid(profileDesc != nullptr && profileDesc->isValid(), ac, "profileDesc: invalid");
			grasp::Assert::valid(!distance.empty(), ac, "distance: invalid");
			for (grasp::RealSeq::const_iterator i = distance.begin(); i != distance.end(); ++i)
				grasp::Assert::valid(*i >= golem::REAL_ZERO, ac, "distance[i] < 0");
			grasp::Assert::valid(!extrapolation.empty(), ac, "extrapolation: invalid");
			for (grasp::RealSeq::const_iterator i = extrapolation.begin(); i != extrapolation.end(); ++i)
				grasp::Assert::valid(*i >= golem::REAL_ZERO, ac, "extrapolation[i] < 0");
			grasp::Assert::valid(!command.empty(), ac, "command: invalid");
			for (grasp::RealSeq::const_iterator i = command.begin(); i != command.end(); ++i)
				grasp::Assert::valid(*i >= golem::REAL_ZERO, ac, "command[i] < 0");

			velFac.assertValid(grasp::Assert::Context(ac, "velFac."));
			accFac.assertValid(grasp::Assert::Context(ac, "accFac."));
			disFac.assertValid(grasp::Assert::Context(ac, "disFac."));
			extFac.assertValid(grasp::Assert::Context(ac, "extFac."));
			cmdFac.assertValid(grasp::Assert::Context(ac, "cmdFac."));

			grasp::Assert::valid(trjExtrapolation >= golem::REAL_ZERO, ac, "trjExtrapolation < 0");
			grasp::Assert::valid(trjDuration > golem::REAL_EPS, ac, "trjDuration < eps");
			grasp::Assert::valid(trjR2GDuration > golem::REAL_EPS, ac, "trjDuration < eps");
			grasp::Assert::valid(trjIdle >= golem::REAL_ZERO, ac, "trjIdle < 0");

			for (grasp::Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i)
				grasp::Assert::valid(i->isValid(), ac, "action[i]: invalid");

			grasp::Assert::valid(pathRenderer.isValid(), ac, "pathRenderer: invalid");
			grasp::Assert::valid(pathIncLarge > golem::REAL_EPS && pathIncLarge <= golem::REAL_ONE, ac, "pathIncLarge < eps or pathIncLarge > 1");
			grasp::Assert::valid(pathIncSmall > golem::REAL_EPS && pathIncSmall <= golem::REAL_ONE, ac, "pathIncSmall < eps or pathIncSmall > 1");

			importRobotTrj.assertValid(grasp::Assert::Context(ac, "importRobotTrj."));
			grasp::Assert::valid(!importHDF5FileExt.empty(), ac, "importHDF5FileExt: invalid");
			grasp::Assert::valid(!importHDF5RobotTrj.empty(), ac, "importHDF5RobotTrj: invalid");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual grasp::data::Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: waypoint */
	static std::string getFileExtWaypoint();

protected:
	/** distRemoved callback */
	typedef std::function<void(size_t)> DistRemovedCallback;

	/** Rendering */
	mutable golem::BoundsRenderer boundsRenderer;
	/** Bounds solid colour */
	golem::RGBA boundsSolidColour;
	/** Path renderer */
	golem::GraphRenderer pathRenderer;
	/** Path increment */
	golem::Real pathIncLarge;
	/** Path increment */
	golem::Real pathIncSmall;

	/** Bounds of kinematic chains */
	ChainBoundsSeq chainBoundsSeq;
	/** Bounds poses of kinematic chains */
	ChainMat34Seq chainMat34Seq;
	/** Bounds of kinematic joints */
	JointBoundsSeq jointBoundsSeq;
	/** Bounds poses of kinematic joints */
	JointMat34Seq jointMat34Seq;
	/** All bounds */
	golem::Bounds::ConstSeq boundsSeq;
	/** Show bounds */
	bool boundsShow;
	/** Path position increment */
	golem::Real pathPositionInc;
	/** Path request at contact */
	bool contactPathRequest;

	/** waypoint suffix */
	std::string waypointSuffix;

	/** Planner index */
	golem::U32 plannerIndex;

	/** Planner */
	golem::Planner* planner;
	/** Pointer to IG heuristic */
	FTDrivenHeuristic* pHeuristic;
	/** Controller */
	const golem::Controller* controller;
	/** Arm controller */
	const golem::Controller* arm;
	/** Hand controller */
	const golem::Controller* hand;
	/** Controller state info */
	golem::Controller::State::Info info;
	/** Arm controller state info */
	golem::Controller::State::Info armInfo;
	/** Hand controller state info */
	golem::Controller::State::Info handInfo;


	/** Trajectory profile description */
	golem::Profile::Desc::Ptr profileDesc;
	/** Trajectory profile */
	golem::Profile::Ptr pProfile;

	/** Trajectory profile configspace distance multiplier */
	golem::ConfigspaceCoord distance;
	/** Trajectory extrapolation */
	golem::ConfigspaceCoord extrapolation;
	/** Trajectory motor command */
	golem::ConfigspaceCoord command;

	/** Velocity multiplication factor */
	FactorDesc velFac;
	/** Acceleration multiplication factor */
	FactorDesc accFac;
	/** Distance multiplication factor */
	FactorDesc disFac;
	/** Extrapolation multiplication factor */
	FactorDesc extFac;
	/** Command multiplication factor */
	FactorDesc cmdFac;

	/** Trajectory extrapolation */
	golem::Real trjExtrapolation;
	/** Trajectory duration */
	golem::Real trjDuration;
	/** IG Trajectory duration */
	golem::Real trjR2GDuration;

	/** Trajectory idle */
	golem::Real trjIdle;

	/** Trajectory velocity */
	golem::ConfigspaceCoord velocityFac;
	/** Trajectory acceleration */
	golem::ConfigspaceCoord accelerationFac;
	/** Trajectory distance */
	golem::ConfigspaceCoord distanceFac;
	/** Trajectory extrapolation */
	golem::ConfigspaceCoord extrapolationFac;
	/** Trajectory command */
	golem::ConfigspaceCoord commandFac;

	/** Action */
	grasp::Mat34Seq action;

	/** distRemoved callback */
	DistRemovedCallback distRemovedCallback;

	/** Import types */
	grasp::StringSeq importTypes;

	/** Show commands/states */
	bool showCommands;

	/** Robot trajectory import */
	ImportRobotTrjDesc importRobotTrj;
	/** Import from HDF5 dump file: file extension */
	std::string importHDF5FileExt;
	/** Import from HDF5 dump file: robot trajectory dataset */
	std::string importHDF5RobotTrj;

	/** Creates render buffer */
	void createRender(const ItemR2GTrajectory& item);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual grasp::data::Item::Ptr create() const;

	/** golem::Profile::CallbackDist: Configuration space coordinate distance metric */
	virtual golem::Real distConfigspaceCoord(const golem::ConfigspaceCoord& prev, const golem::ConfigspaceCoord& next) const;
	/** golem::Profile::CallbackDist: Coordinate distance metric */
	virtual golem::Real distCoord(golem::Real prev, golem::Real next) const;
	/** golem::Profile::CallbackDist: Coordinate enabled state */
	virtual bool distCoordEnabled(const golem::Configspace::Index& index) const;
	/** golem::Profile::CallbackDist: Coordinate interpolate state */
	virtual bool distCoordInterpolate(const golem::Configspace::Index& index) const;
	/** golem::Profile::CallbackDist: Prune state sequence state */
	virtual void distRemoved(size_t index) const;

	/** Creates trajectory from state sequence */
	virtual void create(const golem::ConfigspaceCoord& delta, golem::Controller::State::Seq& trajectory) const;

	/** Profile state sequence */
	virtual void profile(golem::SecTmReal duration, golem::Controller::State::Seq& trajectory) const;

	/** HandlerPlanner: Planner index. */
	virtual golem::U32 getPlannerIndex() const;
	/** HandlerPlan: Sets planner and controllers. */
	virtual void set(golem::Planner& planner, const grasp::ControllerId::Seq& controllerIDSeq);

	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const ItemR2GTrajectory& item, golem::Controller::State::Seq& trajectory);
	/** (Mycroft) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const ItemR2GTrajectory& item, const golem::Controller::State& begin, golem::Controller::State::Seq& trajectory);
	/** (IR3ne) Compute approaching trajectory: Returns waypoints with velocity profile. */
	virtual void createIGTrajectory(const ItemR2GTrajectory& item, const golem::Controller::State& begin, golem::Controller::State::Seq& trajectory);

	/** Pregrasp hand pose */
	grasp::RealSeq handPregraspPose;
	/** Grasp hand pose */
	grasp::RealSeq handGraspPose;

	/** Import: Import from file */
	virtual grasp::data::Item::Ptr import(const std::string& path);
	/** Import: Available file types */
	virtual const grasp::StringSeq& getImportFileTypes() const;

	/** Transform interfaces */
	grasp::StringSeq transformInterfaces;

	/** Transform: Transform input items */
	virtual grasp::data::Item::Ptr transform(const grasp::data::Item::List& input);
	/** Transform: return available interfaces */
	virtual const grasp::StringSeq& getTransformInterfaces() const;
	/** Transform: is supported by the interface */
	virtual bool isTransformSupported(const grasp::data::Item& item) const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerR2GTrajectory(golem::Context &context);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(spam::data::HandlerR2GTrajectory::ImportState::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};  // namespace data

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(const std::string &attr, data::HandlerR2GTrajectory::ImportState::Type& val, golem::XMLContext* xmlcontext, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(const std::string &attr, data::HandlerR2GTrajectory::ImportFrame::Type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; // namespace spam

#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
