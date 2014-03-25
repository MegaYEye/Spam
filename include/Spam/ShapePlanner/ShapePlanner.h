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
#ifndef _SPAM__SHAPEPLANNER_SHAPEPLANNER_H_
#define _SPAM_SHAPEPLANNER_SHAPEPLANNER_H_

//------------------------------------------------------------------------------

#include <Spam/PosePlanner/PosePlanner.h>
#include <Grasp/Grasp/Grasp.h>
#include <Golem/Phys/Application.h>
#include <Golem/Tools/XMLParser.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** ShapePlanner. */
class ShapePlanner : public PosePlanner {
public:
	/** Data */
	class Data : public PosePlanner::Data {
	public:
		/** Hand grasp poses */
		grasp::Grasp::Pose::Seq graspPoses;
		/** Hand grasp pose clusters */
		grasp::Grasp::Cluster::Seq graspClusters;
		/** Hand grasp pose cluster pointer */
		size_t graspClusterPtr, graspClusterSolutionPtr;
		
		/** Grasp poses file name extension */
		std::string extGraspPose;
		/** Grasp pose clusters file name extension */
		std::string extGraspCluster;

		/** Reset data during construction */
		Data() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Data::setToDefault();
			
			graspPoses.clear();
			graspClusters.clear();

			resetDataPointers();

			extGraspPose = ".grasppose";
			extGraspCluster = ".graspcluster";
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			Player::Data::assertValid(ac);

			grasp::Assert::valid(extGraspPose.length() > 0, ac, "extGraspPose: length 0");
			grasp::Assert::valid(extGraspCluster.length() > 0, ac, "extGraspPose: length 0");
		}

		/** Reads/writes object from/to a given XML context */
		virtual void xmlData(golem::XMLContext* context, bool create = false) const;

		/** Creates new data */
		virtual Ptr clone() const;

		/** Reset data pointers */
		void resetDataPointers() {
			graspClusterPtr = graspClusterSolutionPtr = 0;
		}
		/** Grasp poses */
		inline bool hasGraspPoses() const {
			return !graspPoses.empty() && graspClusters.size() > graspClusterPtr && graspPoses.size() > graspClusters[graspClusterPtr].begin + graspClusterSolutionPtr;
		}
		/** Grasp pose begin */
		inline grasp::Grasp::Pose::Seq::const_iterator getGraspPoseBegin() const {
			return graspPoses.begin() + graspClusters[graspClusterPtr].begin;
		}
		/** Grasp pose end */
		inline grasp::Grasp::Pose::Seq::const_iterator getGraspPoseEnd() const {
			return graspPoses.begin() + graspClusters[graspClusterPtr].end;
		}
		/** Current grasp pose */
		inline grasp::Grasp::Pose::Seq::const_iterator getGraspPose() const {
			return hasGraspPoses() ? graspPoses.begin() + graspClusters[graspClusterPtr].begin + graspClusterSolutionPtr : graspPoses.end();
		}
	};

	class Appearance {
	public:
		/** Grasp learning data appearance */
		grasp::Grasp::Data::Appearance graspData;

		/** Distribution num of samples */
		size_t distribSamples;
		/** Distribution num of bounds */
		size_t distribBounds;
		/** Samples colour */
		golem::RGBA samplesColour;
		/** Samples point size */
		golem::Real samplesPointSize;

		/** Appearance, path segments */
		golem::U32 pathSegments;
		/** Qppearance, path colour */
		golem::RGBA pathColour;

		/** Show frames, not points */
		bool showFrames;
		/** Grip appearance, GRASP_MODE_GRIP solid/wireframe */
		bool gripSolid;
		/** Config appearance, GRASP_MODE_CONFIG solid/wireframe */
		bool configDistribSolid;
		/** Contact appearance, GRASP_MODE_CONTACT solid/wireframe */
		bool contactDistribSolid;
		/** Appearance, solid colour */
		golem::RGBA solidColour, solidColourSelect;
		/** Appearance, wire colour */
		golem::RGBA wireColour, wireColourSelect;
		/** Appearance, wireframe thickness */
		golem::Real wireWidth;

		/** Constructs from description object */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			graspData.setToDefault();
			distribSamples = 100;
			distribBounds = 1;
			samplesColour = golem::RGBA(golem::RGBA::RED._rgba.r, golem::RGBA::RED._rgba.g, golem::RGBA::RED._rgba.b, 50);
			samplesPointSize = golem::Real(3.0);
			pathSegments = 20;
			pathColour = golem::RGBA::WHITE;
			showFrames = false;
			gripSolid = true;
			configDistribSolid = false;
			contactDistribSolid = false;
			solidColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			solidColourSelect = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(255));
			wireColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireColourSelect = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(255));
			wireWidth = golem::Real(2.0);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!graspData.isValid())
				return false;
			if (wireWidth <= golem::REAL_ZERO)
				return false;
			return true;
		}
	};

	typedef std::map<std::string, grasp::Grasp::Desc::Ptr> GraspDescMap;
	typedef std::map<std::string, grasp::Grasp::Ptr> GraspMap;
	static const std::string GRASP_NAME_DFLT;

	typedef std::map<std::string, grasp::Classifier::Desc::Ptr> ClassifierDescMap;
	typedef std::map<std::string, grasp::Classifier::Ptr> ClassifierMap;
	static const std::string CLASSIFIER_NAME_DFLT;

	class DemoDesc {
	public:
		typedef std::map<std::string, DemoDesc> Map;

		/** Model data name */
		std::string dataModelName;
		/** Query data name */
		std::string dataQueryName;
		
		/** Grasp trajectory extrapolation factor */
		golem::Real trjApproachExtrapolFac;
		/** Global frame */
		grasp::Mat34Seq frames;
		/** Trajectory search maximum number of tries per cluster */
		golem::U32 trjClusterSize;
		/** Estimate object pose */
		bool estimatePose;

		/** Constructs from description object */
		DemoDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			dataModelName = "model";
			dataQueryName = "query";
			trjApproachExtrapolFac = golem::Real(1.0);
			frames.clear();
			trjClusterSize = 20;
			estimatePose = false;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			//if (dataModelName.empty() || dataQueryName.empty())
			//	return false;
			if (trjApproachExtrapolFac < golem::REAL_ZERO || trjClusterSize < 1)
				return false;
			return true;
		}
	};

	class Desc : public PosePlanner::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(ShapePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Grasp estimators */
		GraspDescMap graspDescMap;
		/** Grasp classifiers */
		ClassifierDescMap classifierDescMap;
		
		/** Demo descriptions */
		DemoDesc::Map demoDescMap;

		/** Appearance */
		Appearance appearance;

		/** Grasp classifier file name extension */
		std::string extGraspClass;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Desc::setToDefault();

			data.reset(new Data);
			
			//graspDescMap.insert(GraspDescMap::value_type(GRASP_NAME_DFLT, GraspDesc()));
			//classifierDescMap.insert(ClassifierDescMap::value_type(CLASSIFIER_NAME_DFLT, ClassifierDesc()));
			
			demoDescMap.clear();
			appearance.setToDefault();
			extGraspClass = ".graspclass";
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PosePlanner::Desc::isValid())
				return false;
			
			for (ShapePlanner::GraspDescMap::const_iterator i = graspDescMap.begin(); i != graspDescMap.end(); ++i)
				if (i->first.empty() || !i->second->isValid())
					return false;
			for (ShapePlanner::ClassifierDescMap::const_iterator i = classifierDescMap.begin(); i != classifierDescMap.end(); ++i)
				if (i->first.empty() || !i->second->isValid())
					return false;

			for (ShapePlanner::DemoDesc::Map::const_iterator i = demoDescMap.begin(); i != demoDescMap.end(); ++i)
				if (!i->second.isValid())
					return false;
			if (!appearance.isValid())
				return false;
			if (extGraspClass.length() <= 0)
				return false;

			return true;
		}
	};

protected:
	/** grasp mode */
	enum GraspMode {
		GRASP_MODE_DISABLED,
		GRASP_MODE_GRIP,
		GRASP_MODE_CONFIG,
		GRASP_MODE_CONTACT,
	};
	/** Contact default name */
	static const char* GraspModeName[GRASP_MODE_CONTACT + 1];
	/** grasp contact */
	enum GraspContact {
		GRASP_CONTACT_DISABLED,
		GRASP_CONTACT_ALL,
		GRASP_CONTACT_SELECT,
	};
	/** Contact default name */
	static const char* GraspContactName[GRASP_CONTACT_SELECT + 1];

	/** Manipulator proxy */
	grasp::Manipulator::Ptr manipulator;

	/** Grasp estimator descriptions */
	GraspDescMap graspDescMap;
	/** Current grasp estimator */
	std::pair<std::string, grasp::Grasp::Ptr> grasp;

	/** Grasp classifier descriptions */
	ClassifierDescMap classifierDescMap;
	/** Current grasp classifier */
	std::pair<std::string, grasp::Classifier::Ptr> classifier;

	/** Demo descriptions */
	DemoDesc::Map demoDescMap;

	/** Show contact relations */
	grasp::Feature::Relation showFeatureRelation;
	/** Grasp model renderers */
	golem::DebugRenderer graspModelContactRenderer, graspModelConfigRenderer;
	/** Grasp query renderer */
	golem::DebugRenderer graspQueryRenderer;
	/** Bounds renderer */
	golem::BoundsRenderer boundsRenderer;
	/** Hand bounds */
	golem::Bounds::Seq handBounds, handBoundsSelect;

	/** Grasp mode */
	golem::U32 graspMode;
	/** Grasp contact */
	golem::U32 graspContact;
	/** Config index */
	golem::U32 configIndex;
	/** Contact index */
	golem::U32 contactIndex;
	/** Pose subspace distance */
	golem::I32 poseSubspaceDist;

	/** Appearance */
	Appearance appearance;

	/** Grasp classifier file name extension */
	std::string extGraspClass;

	/** Source and target data */
	Data::Map::iterator targetDataPtr;
	bool shapeDataRender;

	/** Grip info */
	void printGripInfo();

	void addBounds(const grasp::Manipulator::Pose& pose);

	/** Profile state sequence */
	virtual void profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const;

	/** Render data */
	virtual void renderData(Data::Map::const_iterator dataPtr);
	/** Reset grasp data pointers */
	void resetDataPointers();

	/** User interface: menu function */
	virtual void function(Data::Map::iterator& dataPtr, int key);

	virtual void render();
	virtual void mouseHandler(int button, int state, int x, int y);

	ShapePlanner(golem::Scene &scene);
	bool create(const Desc& desc);
};

/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::Appearance &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::GraspDescMap::value_type &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::ClassifierDescMap::value_type &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::DemoDesc &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::DemoDesc::Map::value_type &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** ShapePlanner application */
class ShapePlannerApp : public golem::Application {
protected:
	/** Runs Application */
	virtual void run(int argc, char *argv[]);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GRASP_SHAPEPLANNER_SHAPEPLANNER_H_*/
