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
#ifndef _SPAM_SPAM_RBPOSE_H_
#define _SPAM_SPAM_RBPOSE_H_

//------------------------------------------------------------------------------

//#include <Grasp/Grasp/Cloud.h>
//#include <Grasp/Grasp/RBPose.h>
//#include <Grasp/Core/Robot.h>
#include <Spam/Spam/Spam.h>

////------------------------------------------------------------------------------
//
//namespace flann {
//	template <typename T> struct L2_Simple;
//};
//
//namespace pcl {
//	struct PointXYZ;
//	template <typename T, typename Dist> class KdTreeFLANN;
//	struct PolygonMesh;
//};
//
//namespace grasp {
//	class Manipulator;
//};
//
//namespace spam {
//	class FTDrivenHeuristic;
//};

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Pose distribution and estimation with tactile and visual feedback */
class Belief : public grasp::RBPose {
public:
	friend class FTDrivenHeuristic;
	typedef golem::shared_ptr<Belief> Ptr;

	/** Description file of tactile observational model */
	class SensoryDesc {
	public:
		/** Max distance rancge for the sensory model **/
		golem::Real sensoryRange;
		/** Importance weight for contacts **/
		golem::Real contactFac;
		/** Importance weight for non contacts **/
		golem::Real noContactFac;

		/** Constructs description. */
		SensoryDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			sensoryRange = golem::Real(.75);
			contactFac = golem::Real(.75);
			noContactFac = golem::Real(.25);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			//for (size_t i = 0; i < grasp::RBCoord::N; ++i)
			//	if (!golem::Math::isPositive(covariance[i]))
			//		return false;
			return true;
		}
	};

	/** Description file */
	class Desc : public grasp::RBPose::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Sensory model description file */
		SensoryDesc sensory;

		/** Hypothesis description file */
		Hypothesis::Desc::Ptr hypothesisDescPtr;

		/** Number of hypothesis per model **/
		size_t numPoses;
		/** Number of hypothesis per model **/
		size_t numHypotheses;
		/** Max number of surface points in the kd-trees **/
		size_t maxSurfacePoints;
		/** Sample covariance metaparameters **/
		grasp::RBCoord covariance;

		/** Metaparameter for density function, e.g. e^(lambda*x) **/
		golem::Real lambda;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Creates object from the description. */
		GRASP_CREATE_FROM_OBJECT_DESC1(Belief, grasp::RBPose::Ptr, golem::Context&)
		/** Sets the parameters to the default values. */
		void setToDefault() {
			grasp::RBPose::Desc::setToDefault();
//			tactile.setToDefault();
			hypothesisDescPtr.reset(new Hypothesis::Desc());
			numHypotheses = 5;
			maxSurfacePoints = 10000;
			std::fill(&covariance[0], &covariance[3], golem::Real(0.02)); // Vec3
			std::fill(&covariance[3], &covariance[7], golem::Real(0.005)); // Quat
			lambda = 1;
		}
		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::RBPose::Desc::assertValid(grasp::Assert::Context(ac, "RBPose::Desc: invalid."));
			grasp::Assert::valid(hypothesisDescPtr != nullptr, ac, "Hypothesis: null pointer.");
			grasp::Assert::valid(hypothesisDescPtr->isValid(), ac, "HypothesisDesc: invalid.");
		}
	};

	/** Probability density value=p(s) for c */
	golem::Real density(const grasp::RBCoord &c) const;

	/** Transformation samples */
	inline const Hypothesis::Seq& getHypotheses() const {
		return hypotheses;
	}
	/** Transformation samples */
	inline Hypothesis::Seq& getHypotheses() {
		return hypotheses;
	}
	/** Returns samples properties */
	inline golem::SampleProperty<golem::Real, grasp::RBCoord> getSampleProperties() { return sampleProperties; };

	/** Sets the hypothesis for planning. NOTE: returns the action frame **/
	grasp::RBPose::Sample createHypotheses(const grasp::Cloud::PointSeq& model, const golem::Mat34 &transform/*, const bool init = true*/);
	/** Returns the low-dimensional representation of the density **/
	grasp::RBPose::Sample::Seq getHypothesesToSample() const;

	/** Creates query object pose distribution */
	void createQuery(const grasp::Cloud::PointSeq& points);
	/** Creates a new set of poses (resampling wheel algorithm) */
	virtual void createResample();
	/** Creates belief update (on importance weights) given the robot's pose and the current belief state. NOTE: weights are normalised. */
	void createUpdate(const grasp::Manipulator *manipulator, const golem::Controller::State::Info handInfo, const golem::Waypoint &w, const FTGuard::Seq &triggeredGuards, const grasp::RealSeq &force);
	/** Creates belief update (on importance weights) given the robot's pose and the current belief state. NOTE: weights are normalised. */
	void createUpdate(const Collision::Ptr collision, const golem::Waypoint &w, const FTGuard::Seq &triggeredGuards, const grasp::RBCoord &rbPose);
	/** Evaluates the likelihood of reading a contact between robot's pose and the sample */
	golem::Real evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const grasp::RealSeq &forces, std::vector<bool> &triggered, bool intersect = true);
	//golem::Real evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const golem::Real &force, bool jointZero, bool intersect = true);

	/** Checks collision with the max likelihood fitting estimation */
//	bool intersect(const golem::Bounds::Seq &bounds, const golem::Mat34 &pose) const;

	/** Probability density value=p(d) for a given distance between finger and hypothesis */
	golem::Real density(const golem::Real dist) const;

	/** Normalised weight for the input pose */
	inline golem::Real normalise(const grasp::RBPose::Sample &pose) const {
		return normaliseFac > golem::REAL_ZERO ? pose.weight / normaliseFac : pose.weight;
	}

	/** Returns the max weight associated to the corrent samples */
	golem::Real maxWeight(const bool normalised = false) const;

	/** Sets belief */
	void set(const grasp::RBPose::Sample::Seq &poseSeq, const grasp::RBPose::Sample::Seq &hypothesisSeq, const golem::Mat34 &trn, const grasp::Cloud::PointSeq &points);

	/** Sets poses */
	void setPoses(const grasp::RBPose::Sample::Seq &poseSeq); // throws exception

	/** Sets hypotheses */
	void setHypotheses(const grasp::RBPose::Sample::Seq &hypothesisSeq); // throws exception

	/** Resets the high representation distribution */
	inline void reset() {
		sampleProperties = initProperties;
		poses.clear();
		for (grasp::RBPose::Sample::Seq::const_iterator i = initPoses.begin(); i != initPoses.end(); ++i)
			poses.push_back(*i);
		context.write("spam::Belief::createQuery(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
	}

	/** Gets the covariance det associated with the hypotheses **/
	inline golem::Real getCovarianceDet() { return covarianceDet; };

	/** Draw samples */
	void drawSamples(const size_t numSamples, golem::DebugRenderer& renderer) const;

	/** Draw hypotheses */
	void drawHypotheses(golem:: DebugRenderer &renderer, const bool showOnlyMeanPose = false) const;

	/** Draw volumetric region for uncertainty */
	golem::Bounds::Seq uncertaintyRegionBounds();

	/** Acquires manipulator */
	inline void setManipulator(grasp::Manipulator *ptr) { manipulator.reset(ptr); /*collision.reset(new Collision(context, *manipulator));*/ };

	/** Pose description */
	Desc myDesc;

	/** Real pose of the object */
	grasp::RBCoord realPose;

protected:
	/** Pointer to the RBPose object */
	grasp::RBPose::Ptr pRBPose;

	/** Appearance */
	Hypothesis::Appearance appearance;


	/** Model point cloud **/
	grasp::Cloud::PointSeq modelPoints;
	/** Model frme reference **/
	golem::Mat34 modelFrame;

	/** Hypothesis container **/
	Hypothesis::Seq hypotheses;
	/** Covariance det associated with the samples **/
	golem::Real covarianceDet;

	/** Kernel function */
	golem::Real kernel(golem::Real x, golem::Real lambda = golem::REAL_ONE) const;

	///** Returns the max weight associated to the corrent samples */
	//golem::Real maxWeight(const bool normalised = false) const;

	/** Initial belief distribution. NOTE: Used for the reset method. */
	grasp::RBPose::Sample::Seq initPoses;
	/** Transformation samples properties */
	golem::SampleProperty<golem::Real, grasp::RBCoord> sampleProperties, initProperties;
	/** Normalise factor */
	golem::Real normaliseFac;

	/** Uncertainty region */
	golem::BoundingBox::Desc uncertaintyDesc;

	/** Manipulator pointer */
	grasp::Manipulator::Ptr manipulator;

	/** Creates/initialises the object */
	bool create(const Desc& desc);
	/** Creates the object */
	Belief(golem::Context& context);
};

//------------------------------------------------------------------------------

void XMLData(Belief::Desc& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

}; // namespace spam

//------------------------------------------------------------------------------

namespace golem {
	template <> void Stream::read(spam::Belief& belief) const;
	template <> void Stream::write(const spam::Belief& belief);
};	// namespace

//------------------------------------------------------------------------------

#endif /** _SPAM_SPAM_RBPOSE_H_ */