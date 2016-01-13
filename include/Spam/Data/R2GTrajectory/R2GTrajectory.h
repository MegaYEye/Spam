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
#include <Grasp/Contact/Manipulator.h>
#include <Spam/HBPlan/Heuristic.h>

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

/** Map of pose sequences */
typedef std::map<std::string, grasp::Mat34Seq> Mat34MapSeq;

/** Data item representing trajectory.
*/
class GOLEM_LIBRARY_DECLDIR ItemR2GTrajectory : public grasp::data::ItemTrajectory, public grasp::data::Convert, public grasp::data::Export {
public:
	friend class HandlerR2GTrajectory;

	/** Robot hand trajectory */
	grasp::Mat34Seq robotHandPoses;
	/** Joint poses */
	Mat34MapSeq jointsPoses;

	/** Robot hand trajectory */
	mutable grasp::data::File robotHandPoseFile;
	/** Object trajectory file */
	mutable grasp::data::File objectPoseFile;
	/** Object trajectory file */
	mutable grasp::data::File objectPointFile;
	/** Joint poses file */
	mutable grasp::data::File jointsPoseFile;

	/** Clones item. */
	virtual grasp::data::Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const grasp::Waypoint::Seq& waypoints);

	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(golem::Controller::State::Seq& trajectory);

protected:
	/** Data handler */
	HandlerR2GTrajectory& handler;

	/** Convert: Convert current item */
	virtual grasp::data::Item::Ptr convert(const grasp::data::Handler& handler);
	/** Convert: return available interfaces */
	virtual const grasp::StringSeq& getConvertInterfaces() const;
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const grasp::data::Handler& handler) const;

	/** Export: Export to file */
	virtual void exportt(const std::string& path) const;
	/** Export: Available file types */
	virtual const grasp::StringSeq& getExportFileTypes() const;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemR2GTrajectory(HandlerR2GTrajectory& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerR2GTrajectory : public grasp::data::HandlerTrajectory {
public:
	friend class ItemR2GTrajectory;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public grasp::data::HandlerTrajectory::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Manipulator description */
		grasp::Manipulator::Desc::Ptr manipulatorDesc;

		/** Pose suffix */
		std::string poseSuffix;

		/** Robot hand pose */
		grasp::data::HandlerTrajectory::ImportFrame importRobotHandPose;
		/** Object pose */
		grasp::data::HandlerTrajectory::ImportFrame importObjectPose;
		/** Joints pose */
		grasp::data::HandlerTrajectory::ImportFrame importJointsPose;
		/** Transform */
		golem::Mat34 importRobotTrjTransform;
		/** Import from HDF5 dump file: hand pose dataset */
		std::string importHDF5RobotHandPose;
		/** Import from HDF5 dump file: object pose dataset */
		std::string importHDF5ObjectPose;
		/** Import from HDF5 dump file: joints pose dataset */
		grasp::StringSeq importHDF5JointsPose;

		/** Appearance frame size */
		golem::Vec3 appearanceHandFrameSize;
		/** Appearance frame size */
		golem::Vec3 appearanceJointsFrameSize;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			grasp::data::HandlerTrajectory::Desc::setToDefault();

			manipulatorDesc.reset(new grasp::Manipulator::Desc);

			poseSuffix = getFileExtPose();

			importRobotHandPose.setToDefault();
			importObjectPose.setToDefault();
			importJointsPose.setToDefault();
			importRobotTrjTransform.setId();
			importHDF5RobotHandPose = "RobotHandPose";
			importHDF5ObjectPose = "ObjectPose";
			importHDF5JointsPose.clear();

			appearanceHandFrameSize.set(golem::Real(0.1));
			appearanceJointsFrameSize.set(golem::Real(0.02));
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::data::HandlerTrajectory::Desc::assertValid(ac);

			grasp::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(grasp::Assert::Context(ac, "manipulatorDesc->"));

			grasp::Assert::valid(poseSuffix.length() > 0, ac, "poseSuffix: empty");

			importRobotHandPose.assertValid(grasp::Assert::Context(ac, "importRobotHandPose."));
			importObjectPose.assertValid(grasp::Assert::Context(ac, "importObjectPose."));
			importJointsPose.assertValid(grasp::Assert::Context(ac, "importJointsPose."));
			grasp::Assert::valid(importRobotTrjTransform.isValid(), ac, "importRobotTrjTransform: invalid");
			grasp::Assert::valid(!importHDF5RobotHandPose.empty(), ac, "importHDF5RobotHandPose: invalid");
			grasp::Assert::valid(!importHDF5ObjectPose.empty(), ac, "importHDF5ObjectPose: invalid");
			for (grasp::StringSeq::const_iterator i = importHDF5JointsPose.begin(); i != importHDF5JointsPose.end(); ++i)
				grasp::Assert::valid(!i->empty(), ac, "importHDF5JointsPose[i]: invalid");

			grasp::Assert::valid(appearanceHandFrameSize.isPositive(), ac, "appearanceHandFrameSize: invalid");
			grasp::Assert::valid(appearanceJointsFrameSize.isPositive(), ac, "appearanceJointsFrameSize: invalid");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual grasp::data::Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: pose sequence */
	static std::string getFileExtPose();

protected:
	/** Manipulator description */
	grasp::Manipulator::Desc::Ptr manipulatorDesc;
	/** Manipulator */
	grasp::Manipulator::Ptr manipulator;

	/** Smart pointer to the ft driven heuristic */
	FTDrivenHeuristic* pHeuristic;

	/** Pose suffix */
	std::string poseSuffix;

	/** Robot hand pose */
	grasp::data::HandlerTrajectory::ImportFrame importRobotHandPose;
	/** Object pose */
	grasp::data::HandlerTrajectory::ImportFrame importObjectPose;
	/** Joints pose */
	grasp::data::HandlerTrajectory::ImportFrame importJointsPose;
	/** Transform */
	golem::Mat34 importRobotTrjTransform;
	/** Import from HDF5 dump file: hand pose dataset */
	std::string importHDF5RobotHandPose;
	/** Import from HDF5 dump file: object pose dataset */
	std::string importHDF5ObjectPose;
	/** Import from HDF5 dump file: joints pose dataset */
	grasp::StringSeq importHDF5JointsPose;

	/** Appearance frame size */
	golem::Vec3 appearanceHandFrameSize;
	/** Appearance frame size */
	golem::Vec3 appearanceJointsFrameSize;

	/** Convert interfaces */
	grasp::StringSeq convertInterfaces;

	/** Export types */
	grasp::StringSeq exportTypes;

	/** Rendering */
	golem::DebugRenderer renderer;

	/** HandlerPlan: Sets planner and controllers. */
	virtual void set(golem::Planner& planner, const grasp::StringSeq& controllerIDSeq);

	/** Creates trajectory from state sequence */
	virtual void create(const golem::ConfigspaceCoord& delta, golem::Controller::State::Seq& trajectory) const;
	/** Profile state sequence */
	virtual void profile(golem::SecTmReal duration, golem::Controller::State::Seq& trajectory) const;
	/** Trajectory: Returns waypoints with velocity profile. */
	virtual void createTrajectory(const ItemR2GTrajectory& item, golem::Controller::State::Seq& trajectory);

	/** Convert: Convert current item */
	virtual grasp::data::Item::Ptr convert(ItemR2GTrajectory& item, const grasp::data::Handler& handler);
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const grasp::data::Handler& handler) const;

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

	/** Import: Import from file */
	virtual grasp::data::Item::Ptr import(const std::string& path);

	/** Export: Export to file */
	virtual void exportt(const ItemR2GTrajectory& item, const std::string& path) const;
	/** Export: Available file types */
	virtual const grasp::StringSeq& getExportFileTypes() const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerR2GTrajectory(golem::Context &context);
};

//------------------------------------------------------------------------------

};  // namespace
};	// namespace

#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
