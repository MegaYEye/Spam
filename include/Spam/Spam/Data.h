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
#ifndef _SPAM_DATA_H_
#define _SPAM_DATA_H_

//------------------------------------------------------------------------------

//#include <Golem/Tools/XMLParser.h>
//#include <Golem/Tools/XMLData.h>
//#include <Golem/Tools/Data.h>
#include <Spam/Spam/Robot.h>
#include <Spam/Spam/GraphPlanner.h>
#include <Grasp/Grasp/Grasp.h>
#include <Spam/Spam/Spam.h>

//------------------------------------------------------------------------------

namespace spam {

////------------------------------------------------------------------------------
//
//void XMLData(FTDrivenHeuristic::FTModelDesc& val, golem::XMLContext* context, bool create = false);
//
//void XMLData(FTDrivenHeuristic::Desc& val, golem::XMLContext* context, bool create = false);

///** Reads/writes object from/to a given XML context */
//void XMLData(spam::Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//void XMLData(RagGraphPlanner::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Trial data for a single demonstration and/or test trial in a binary format (stored on disk) */
class TrialData {
public:
	typedef std::map<std::string, TrialData> Map;
	friend class golem::Stream;

	/** Header name */
	static const char headerName [];
	/** Header version */
	static const golem::U32 headerVersion;

	/** Cache (local): OpenGL settings */
	golem::Scene::OpenGL openGL;
	/** Cache (local): action */
	golem::Controller::State::Seq action;

	/** Ieration index */
	std::size_t iteration;

	/** Object label */
	std::string objectLabel;
	/** Grasps */
	golem::Controller::State::Seq grasps;

	///** Approach action waypoints */
	//grasp::RobotState::List approachAction;
	///** Manipulation action waypoints */
	//grasp::RobotState::List manipAction;
	///** Withdraw action waypoints */
	//grasp::RobotState::List approachWithdraw;
	///** Combined action waypoints */
	golem::Controller::State::Seq executedTrajectory;

	/** High dim representation of pdf */
	grasp::RBPose::Sample::Seq density;
	/** High dim rep covariance */
	grasp::RBCoord densityCov;
	/** Low dim representation of pdf */
	grasp::RBPose::Sample::Seq hypotheses;
	/** High dim rep covariance */
	grasp::RBCoord hypothesesCov;
	/** Normalise factor */
	golem::Real normFac;

	/** Specified if the trial has noise */
	bool gtNoiseEnable;
	/** Trial noise: linear component */
	golem::Vec3 noiseLin;
	/** Trial noise: angular component */
	golem::Quat noiseAng;

	/** Specifies if guard have been triggered during the perform of the action */
	int triggered;
	/** Contains the index of the triggered guards */
//	std::vector<golem::Configspace::Index> triggeredGuards;
	std::vector<FTGuard> triggeredGuards;
	/** State of the robot at the time a contact occurred */
	golem::Controller::State::Seq triggeredStates;

	/** Safety configurations of the robot */
	grasp::RobotState::Seq homeStates;

	/** Specifies if the replanning should be triggered */
	bool replanning;

	/** Constructor */
	TrialData(const golem::Controller& controller) : controller(controller) {
	}

protected:
	const golem::Controller& controller;
};

//------------------------------------------------------------------------------

//class Collision {
//public:
//	typedef std::shared_ptr<Collision> Ptr;
//	/** Bounds */
//	class Bounds {
//	public:
//		typedef std::vector<Bounds> Seq;
//
//		/** Surface */
//		struct Surface {
//			typedef std::vector<Surface> Seq;
//			typedef std::vector<Seq> SeqSeq;
//			golem::Vec3 point;
//			golem::Vec3 normal;
//		};
//		/** Triangle */
//		struct Triangle : public Surface {
//			typedef std::vector<Triangle> Seq;
//			typedef std::vector<Seq> SeqSeq;
//			golem::Real distance;
//		};
//
//		/** Create bounds from convex meshes */
//		void create(const golem::Bounds::Seq& bounds);
//
//		/** Pose */
//		static inline void setPose(const golem::Mat34& pose, const Surface& surface, Triangle& triangle) {
//			pose.multiply(triangle.point, surface.point);
//			pose.R.multiply(triangle.normal, surface.normal);
//			triangle.distance = triangle.normal.dot(triangle.point);
//		}
//		/** Pose */
//		static inline void setPose(const golem::Mat34& pose, const Surface::Seq& surfaces, Triangle::Seq& triangles) {
//			triangles.resize(surfaces.size());
//			for (size_t i = 0; i < triangles.size(); ++i)
//				setPose(pose, surfaces[i], triangles[i]);
//		}
//		/** Pose */
//		inline void setPose(const golem::Mat34& pose) {
//			for (size_t i = 0; i < triangles.size(); ++i)
//				setPose(pose, surfaces[i], triangles[i]);
//		}
//
//		/** Penetration depth of a given point */
//		static inline golem::Real getDepth(const Triangle::Seq& triangles, const golem::Vec3& point) {
//			golem::Real depth = golem::REAL_MAX;
//			for (Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
//				const golem::Real d = i->distance - i->normal.dot(point);
//				if (d < golem::REAL_ZERO) // no collision
//					return d;
//				if (depth > d) // search for minimum
//					depth = d;
//			}
//			return depth;
//		}
//		/** Penetration depth of a given point, zero if none */
//		inline golem::Real getDepth(const golem::Vec3& point) const {
//			golem::Real depth = golem::REAL_ZERO; // if no bounds or collisions, no effect
//			for (Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
//				const golem::Real d = getDepth(*i, point);
//				if (depth < d) // search for maximum depth (can collide only with one mesh within bounds)
//					depth = d;
//			}
//			return depth;
//		}
//
//		/** Empty */
//		inline bool empty() const {
//			return surfaces.empty();
//		}
//
//		/** Triangles */
//		const Triangle::SeqSeq& getTriangles() const {
//			return triangles;
//		}
//		/** Surfaces */
//		const Surface::SeqSeq& getSurfaces() const {
//			return surfaces;
//		}
//
//	private:
//		/** Triangles */
//		Triangle::SeqSeq triangles;
//		/** Surfaces */
//		Surface::SeqSeq surfaces;
//	};
//
//	/** Collision waypoint */
//	class Waypoint {
//	public:
//		typedef std::vector<Waypoint> Seq;
//
//		/** Path distance */
//		golem::Real pathDist;
//		/** Number of points */
//		golem::U32 points;
//		/** Distance standard deviation */
//		golem::Real depthStdDev;
//		/** Likelihood multiplier */
//		golem::Real likelihood;
//
//		/** Constructs description object */
//		Waypoint() {
//			Waypoint::setToDefault();
//		}
//		/** Sets the parameters to the default values */
//		void setToDefault() {
//			pathDist = golem::Real(0.0);
//			points = 1000;
//			depthStdDev = golem::Real(1000.0);
//			likelihood = golem::Real(1000.0);
//		}
//		/** Checks if the parameters are valid. */
//		bool isValid() const {
//			if (!golem::Math::isFinite(pathDist) || depthStdDev < golem::REAL_EPS || likelihood < golem::REAL_ZERO)
//				return false;
//			return true;
//		}
//	};
//
//	/** Collision description */
//	class Desc {
//	public:
//		/** Enable collisions during last optimisation step */
//		bool enabledLast;
//		/** Enable collisions during all optimisation steps */
//		bool enabledAll;
//		/** Collision waypoints */
//		Waypoint::Seq waypoints;
//
//		/** Constructs description object */
//		Desc() {
//			Desc::setToDefault();
//		}
//		/** Sets the parameters to the default values */
//		void setToDefault() {
//			enabledLast = true;
//			enabledAll = false;
//			waypoints.clear();
//			waypoints.push_back(Waypoint());
//		}
//		/** Checks if the description is valid. */
//		bool isValid() const {
//			if (waypoints.empty())
//				return false;
//			for (Waypoint::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i)
//				if (!i->isValid())
//					return false;
//			return true;
//		}
//	};
//
//	/** Create */
//	Collision(const grasp::Manipulator& manipulator);
//	/** Collision likelihood estimation at a given waypoint */
//	golem::Real evaluate(const Robot *robot, const Waypoint& waypoint, const grasp::Cloud::PointSeq& points, golem::Rand& rand, const grasp::Manipulator::Pose& pose, std::vector<golem::Configspace::Index> &joints, bool debug = false);
//	/** Collision likelihood estimation at a given waypoint */
//	golem::Real evaluate(const Waypoint& waypoint, const grasp::Cloud::PointSeq& points, golem::Rand& rand, const grasp::Manipulator::Pose& pose, bool debug = false);
//	/** Bounds */
//	const Bounds& getBounds(size_t index) const {
//		return bounds[index];
//	}
//
//protected:
//	/** Manipulator */
//	const grasp::Manipulator& manipulator;
//	/** Joints + base */
//	Bounds bounds[grasp::Manipulator::JOINTS + 1];
//};

//------------------------------------------------------------------------------

}; /** namespace */

//------------------------------------------------------------------------------

//namespace golem {
//
//template <> void Stream::read(spam::TrialData& trialData) const;
//template <> void Stream::write(const spam::TrialData& trialData);
//
//};	// namespace

//------------------------------------------------------------------------------

#endif /** _SPAM_DATA_H_ */