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
//! @Date:     08/05/2014
//------------------------------------------------------------------------------
#pragma once
#ifndef _SPAM_SHAPEPLANNER_SHAPEPLANNER_H_
#define _SPAM_SHAPEPLANNER_SHAPEPLANNER_H_

//------------------------------------------------------------------------------

#include <Spam/PosePlanner/PosePlanner.h>
#include <Spam/Spam/Data.h>
#include <Grasp/Grasp/Grasp.h>
#include <Grasp/Grasp/Classifier.h>
#include <Grasp/Grasp/Cluster.h>
#include <Golem/Phys/Application.h>
#include <Golem/Tools/XMLParser.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** ShapePlanner. */
class ShapePlanner : public PosePlanner {
public:
	/** Grasp mode */
	enum Mode {
		MODE_DISABLED,
		MODE_DATA,
		MODE_CLUSTER,
		MODE_CONFIG,
		MODE_CONFIG_MODEL,
		MODE_CONTACT_MODEL,
		MODE_SIZE = MODE_CONTACT_MODEL,
	};
	/** Mode default name */
	static const char* ModeName[MODE_SIZE + 1];

	/** Data */
	class Data : public PosePlanner::Data {
	public:
		/** Grasp configs */
		grasp::Grasp::Config::Seq graspConfigs;
		/** Grasp config clusters */
		grasp::Cluster::Seq graspClusters;

		/** Grasp configs file name extension */
		std::string extGraspConfig;
		/** Grasp config clusters file name extension */
		std::string extGraspCluster;

		/** Grasp mode */
		golem::U32 graspMode;
		/** Grasp data pointer */
		mutable golem::U32 graspDataPtr;
		/** Grasp config cluster pointer */
		mutable golem::U32 graspClusterPtr, graspConfigPtr;

		/** Data appearance */
		mutable grasp::Grasp::Data::Appearance appearanceData;
		/** Config appearance */
		mutable grasp::Grasp::Config::Appearance appearanceConfig;

		/** Reset data during construction */
		Data() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Data::setToDefault();

			graspConfigs.clear();
			graspClusters.clear();

			extGraspConfig = ".graspconfig";
			extGraspCluster = ".graspcluster";

			resetDataPointers();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			Player::Data::assertValid(ac);

			grasp::Assert::valid(extGraspConfig.length() > 0, ac, "extGraspConfig: length 0");
			grasp::Assert::valid(extGraspCluster.length() > 0, ac, "extGraspCluster: length 0");
		}

		/** Reads/writes object from/to a given XML context */
		virtual void xmlData(golem::XMLContext* context, bool create = false) const;

		/** Reset data pointers */
		void resetDataPointers() {
			graspMode = MODE_DISABLED;
			graspDataPtr = 0;
			graspClusterPtr = graspConfigPtr = 0;
		}

		/** Retrieve grasp data from pointer */
		template <typename _GraspMap, typename _GraspMapIter, typename _GraspDataMapIter> bool getGraspData(_GraspMap& map, _GraspMapIter& grasp, _GraspDataMapIter& data) const {
			golem::U32 n = -1;

			// search for index
			for (_GraspMapIter i = map.begin(); i != map.end(); ++i)
				for (_GraspDataMapIter j = i->second->getDataMap().begin(); j != i->second->getDataMap().end(); ++j) {
				grasp = i;
				data = j;
				if (++n == graspDataPtr)
					return true;
				}

			// there is some data
			if (n != -1) {
				graspDataPtr = n;
				return true;
			}

			return false;
		}
		/** Set grasp data pointer */
		void setGraspData(const grasp::Grasp::Map& map, grasp::Grasp::Data::Map::const_iterator data);

		/** Grasp configs */
		inline void assertGraspConfigs() const {
			graspClusterPtr = graspClusters.empty() ? 0 : graspClusterPtr >= (golem::U32)graspClusters.size() ? (golem::U32)graspClusters.size() - 1 : graspClusterPtr;
			graspConfigPtr = graspClusters.empty() || graspConfigs.empty() || graspClusters[graspClusterPtr].end <= 0 ? 0 : graspConfigPtr >= graspClusters[graspClusterPtr].end ? graspClusters[graspClusterPtr].end - 1 : graspConfigPtr;
		}
		/** Grasp configs */
		inline bool hasGraspConfigs() const {
			assertGraspConfigs();
			return !graspConfigs.empty() && (golem::U32)graspClusters.size() > graspClusterPtr && (golem::U32)graspConfigs.size() > graspClusters[graspClusterPtr].begin + graspConfigPtr;
		}
		/** Grasp config begin */
		inline grasp::Grasp::Config::Seq::const_iterator getGraspConfigBegin() const {
			return hasGraspConfigs() ? graspConfigs.begin() + graspClusters[graspClusterPtr].begin : graspConfigs.end();
		}
		/** Grasp config end */
		inline grasp::Grasp::Config::Seq::const_iterator getGraspConfigEnd() const {
			return hasGraspConfigs() ? graspConfigs.begin() + graspClusters[graspClusterPtr].end : graspConfigs.end();
		}
		/** Grasp config size */
		inline golem::U32 getGraspConfigSize() const {
			return hasGraspConfigs() ? graspClusters[graspClusterPtr].size() : 0;
		}
		/** Current grasp pose */
		inline grasp::Grasp::Config::Seq::const_iterator getGraspConfig() const {
			return hasGraspConfigs() ? graspConfigs.begin() + graspClusters[graspClusterPtr].begin + graspConfigPtr : graspConfigs.end();
		}

		/** Grasp info */
		void printGraspInfo(const grasp::Manipulator& manipulator) const;
	};

	/** Grasp demo */
	class Demo {
	public:
		typedef std::map<std::string, Demo> Map;

		/** Object detection */
		class ObjectDetection {
		public:
			typedef std::vector<ObjectDetection> Seq;

			/** Scan pose */
			Robot::Pose scanPose;
			/** Camera name */
			std::string cameraName;
			/** Minimum size */
			golem::U32 minSize;
			/** Object detection delta change */
			golem::U32 deltaSize;
			/** Object detection delta depth */
			golem::Real deltaDepth;

			/** Constructs from description */
			ObjectDetection() {
				setToDefault();
			}
			/** Sets the parameters to the default values */
			void setToDefault() {
				scanPose.setToDefault();
				cameraName.clear();
				minSize = 10000;
				deltaSize = 100;
				deltaDepth = golem::Real(0.001);
			}
			/** Checks if the description is valid. */
			bool isValid() const {
				scanPose.assertValid(grasp::Assert::Context("grasp::ShapePlanner::Desc::isValid(): scanPose."));
				if (cameraName.length() <= 0 || minSize < 1 || deltaDepth < golem::REAL_EPS)
					return false;
				return true;
			}
		};

		/** Pose estimation */
		bool poseEstimation;

		/** Object detection */
		ObjectDetection::Seq objectDetectionSeq;

		/** Classifier name */
		std::string classifierName;

		/** Object model path */
		std::string modelObject;
		/** Trajectory model path */
		grasp::StringSeq modelTrajectories;

		/** Poses */
		Robot::Pose::Seq poses;

		/** Constructs from description */
		Demo() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			poseEstimation = true;

			objectDetectionSeq.clear();

			classifierName.clear();
			modelObject.clear();
			modelTrajectories.clear();

			poses.clear();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (objectDetectionSeq.empty())
				return false;
			for (ObjectDetection::Seq::const_iterator i = objectDetectionSeq.begin(); i != objectDetectionSeq.end(); ++i)
				if (!i->isValid())
					return false;

			if (poseEstimation ? modelObject.length() <= 0 || modelTrajectories.empty() : classifierName.length() <= 0)
				return false;
			for (grasp::StringSeq::const_iterator i = modelTrajectories.begin(); i != modelTrajectories.end(); ++i)
				if (i->length() <= 0)
					return false;

			for (grasp::RobotPose::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) i->assertValid(grasp::Assert::Context("grasp::ShapePlanner::Demo::isValid(): poses[]."));

			return true;
		}
	};

	class Desc : public PosePlanner::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(ShapePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Grasp classifiers */
		grasp::Manipulator::Desc::Ptr manipulatorDesc;

		/** Grasp classifiers */
		grasp::Classifier::Desc::Map classifierDescMap;

		/** Clustering algorithm */
		grasp::Cluster::Desc clusterDesc;

		/** Demo descriptions */
		Demo::Map demoMap;

		/** Grasp classifier file name extension */
		std::string extGraspClass;

		/** Data appearance */
		grasp::Grasp::Data::Appearance appearanceData;
		/** Config appearance */
		grasp::Grasp::Config::Appearance appearanceConfig;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Desc::setToDefault();

			data.reset(new Data);

			manipulatorDesc.reset(new grasp::Manipulator::Desc);
			classifierDescMap.clear();
			clusterDesc.setToDefault();
			demoMap.clear();
			extGraspClass = ".graspclass";

			appearanceData.setToDefault();
			appearanceConfig.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!PosePlanner::Desc::isValid())
				return false;

			if (manipulatorDesc == nullptr || !manipulatorDesc->isValid())
				return false;

			if (classifierDescMap.empty())
				return false;
			for (grasp::Classifier::Desc::Map::const_iterator i = classifierDescMap.begin(); i != classifierDescMap.end(); ++i)
				if (i->first.length() <= 0 || !i->second->isValid())
					return false;

			if (!clusterDesc.isValid())
				return false;

			for (ShapePlanner::Demo::Map::const_iterator i = demoMap.begin(); i != demoMap.end(); ++i)
				if (!i->second.isValid())
					return false;

			if (extGraspClass.length() <= 0)
				return false;

			if (!appearanceData.isValid() || !appearanceConfig.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Manipulator proxy */
	grasp::Manipulator::Ptr manipulator;

	/** Grasp classifier descriptions */
	grasp::Classifier::Desc::Map classifierDescMap;
	/** Current grasp classifier */
	grasp::Classifier::Ptr classifier;
	/** Clustering algorithm */
	grasp::Cluster::Desc clusterDesc;

	/** Grasp model renderers */
	golem::DebugRenderer graspRenderer;

	/** Demo descriptions */
	Demo::Map demoMap;

	/** Grasp classifier file name extension */
	std::string extGraspClass;

	/** Creates new data */
	virtual Data::Ptr createData() const;

	/** Load classifier */
	grasp::Classifier::Ptr load(const grasp::Classifier::Desc& desc);

	/** Render data */
	virtual void renderData(Data::Map::const_iterator dataPtr);
	/** Reset grasp data pointers */
	void resetDataPointers();

	/** Shape planner demo */
	void run(const Demo::Map::value_type& demo);

	/** User interface: menu function */
	virtual void function(Data::Map::iterator& dataPtr, int key);

	virtual void render();
	virtual void mouseHandler(int button, int state, int x, int y);

	ShapePlanner(golem::Scene &scene);
	bool create(const Desc& desc);
};

/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::Demo::ObjectDetection &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::Demo &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ShapePlanner::Demo::Map::value_type &val, golem::XMLContext* xmlcontext, bool create = false);
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
