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
class Belief/* : public grasp::RBPose*/ {
public:
	friend class FTDrivenHeuristic;
	typedef golem::shared_ptr<Belief> Ptr;
	typedef grasp::RBHeuristic<grasp::RBCoord, 1> Heuristic;
	typedef golem::DEOptimisation<Heuristic> Optimisation;

	/** Description file */
	class Desc/* : public grasp::RBPose::Desc*/ {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Pointer to RBPose desc file */
		grasp::RBPose::Desc::Ptr rbPoseDescPtr;

		/** Number of distribution kernels */
		size_t kernels;
		/** number of neighbours */
		size_t neighbours;
		/** Maximum distance between vector and kernel */
		golem::Real distanceRange;

		/** Kernel diagonal covariance scale metaparameter */
		grasp::RBCoord covariance;
		/** Standard deviation */
		grasp::RBDist poseStdDev;
		/** Feature normal epsilon */
		golem::Real featNormEps;
		/** Feature product */
		bool distProd;
		/** Rigid body distance */
		grasp::RBDist dist;
		/** Feature distance */
		golem::Real distFeature;

		/** local alignment enabled */
		bool localEnabled;
		/** population size */
		size_t populationSize;
		/** number of generations min */
		size_t generationsMin;
		/** number of generations max */
		size_t generationsMax;
		/** distance difference threshold */
		golem::Real distanceDiff;

		/** Optimisation */
		Optimisation::Desc optimisationDesc;
		/** Max point model distance */
		golem::Real distanceMax;

		/** Hypothesis description file */
		Hypothesis::Desc::Ptr hypothesisDescPtr;

		/** Number of hypothesis per model **/
		size_t numHypotheses;
		/** Max number of surface points in the kd-trees **/
		size_t maxSurfacePoints;

		/** Metaparameter for density function, e.g. e^(lambda*x) **/
		golem::Real lambda;

		/** Enable/disable hierarchical clustering */
		bool cluster;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			rbPoseDescPtr.reset(new grasp::RBPose::Desc());

			kernels = 250;
			neighbours = 100;
			distanceRange = golem::Real(10.0);

			poseStdDev.set(golem::Real(0.01), golem::Real(100.0));
			std::fill(&covariance[0], &covariance[3], golem::Real(0.01)); // Vec3
			std::fill(&covariance[3], &covariance[7], golem::Real(0.01)); // Quat
			featNormEps = golem::Real(1e-7);
			distProd = true;
			dist.set(golem::Real(1.0), golem::Real(1.0));
			distFeature = golem::Real(0.0);

			localEnabled = true;
			populationSize = 100;
			generationsMin = 10;
			generationsMax = 100;
			distanceDiff = golem::Real(1e-5);

			optimisationDesc.setToDefault();
			distanceMax = golem::Real(0.01);
			hypothesisDescPtr.reset(new Hypothesis::Desc());
			numHypotheses = 5;
			maxSurfacePoints = 10000;
			lambda = 1;

			cluster = true;
		}
		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(rbPoseDescPtr != nullptr, ac, "RBPose desc: null pointer.");

			grasp::Assert::valid(kernels > 0, ac, "kernels: < 1");
			grasp::Assert::valid(neighbours > 0, ac, "neighbours: < 1");
			grasp::Assert::valid(neighbours <= kernels, ac, "neighbours: > kernels");
			grasp::Assert::valid(distanceRange > golem::REAL_ZERO, ac, "distanceRange: <= 0");

			for (size_t i = 0; i < grasp::RBCoord::N; ++i)
				grasp::Assert::valid(covariance[i] > golem::REAL_ZERO, ac, "covariance[]: <= 0");
			grasp::Assert::valid(poseStdDev.isValid(), ac, "poseStdDev: invalid");
			grasp::Assert::valid(dist.isValid(), ac, "dist: invalid");
			grasp::Assert::valid(distFeature >= golem::REAL_ZERO, ac, "distFeature: < 0");
			grasp::Assert::valid(!golem::Math::equals(featNormEps, golem::REAL_ZERO, golem::REAL_EPS), ac, "featNormEps: ~ 0");

			grasp::Assert::valid(populationSize > 0, ac, "populationSize: < 1");
			grasp::Assert::valid(generationsMin > 0, ac, "generationsMin: < 1");
			grasp::Assert::valid(generationsMin <= generationsMax, ac, "generationsMin: > generationsMax");
			grasp::Assert::valid(distanceDiff >= golem::REAL_ZERO, ac, "distanceDiff: < 0");

			grasp::Assert::valid(optimisationDesc.isValid(), ac, "optimisationDesc: invalid");
			grasp::Assert::valid(!golem::Math::equals(distanceMax, golem::REAL_ZERO, golem::REAL_EPS), ac, "distanceMax: ~ 0");

			grasp::Assert::valid(hypothesisDescPtr != nullptr, ac, "Hypothesis: null pointer.");
			grasp::Assert::valid(hypothesisDescPtr->isValid(), ac, "HypothesisDesc: invalid.");
		}
		/** Creates the object from the description. */
		virtual Belief::Ptr create(golem::Context &context) const;
	};

	/** Transformation samples */
	const grasp::RBPose::Sample::Seq& getSamples() const {
		return poses;
	}
	/** Transformation samples */
	grasp::RBPose::Sample::Seq& getSamples() {
		return poses;
	}
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

	/** Creates object frame */
	inline golem::Mat34 createFrame(const grasp::Vec3Seq& points) {
		return rbPosePtr->createFrame(points);
	};

	/** Creates query object pose distribution */
	void createQuery(const grasp::Cloud::PointSeq& points);
	/** Creates a new set of poses (resampling wheel algorithm) */
	virtual void createResample(const grasp::Manipulator::Pose& robotPose);
	/** Creates belief update (on importance weights) given the robot's pose and the current belief state. NOTE: weights are normalised. */
//	void createUpdate(const grasp::Manipulator *manipulator, const golem::Controller::State::Info handInfo, const golem::Waypoint &w, const FTGuard::Seq &triggeredGuards, const grasp::RealSeq &force);
	/** Creates belief update (on importance weights) given the robot's pose and the current belief state. NOTE: weights are normalised. */
	void createUpdate(const Collision::Ptr collision, const golem::Waypoint &w, FTGuard::Seq &triggeredGuards, const grasp::RBCoord &rbPose);
	/** Evaluates the likelihood of reading a contact between robot's pose and the sample */
//	golem::Real evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const grasp::RealSeq &forces, std::vector<bool> &triggered, bool intersect = true);

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
	Desc desc;

	/** Real pose of the object */
	grasp::RBCoord realPose;

	/** Creates model features and search index fro point cloud */
	template <typename _Seq> void createModel(const _Seq& points) {
		rbPosePtr->createModel(points);
	}

	/** Maximum likelihood estimation */
	grasp::RBPose::Sample maximum();

	/** Sample pose from distribution */
	grasp::RBCoord sample() const;

	/** Prints out debug info for cluster analysis */
	std::string clusterToStr() { return clusterStr; }

	Belief::~Belief();

protected:
	/** Context object */
	golem::Context &context;
	/** Generator of pseudo random numbers */
	golem::Rand rand;
	/** Pointer to the RBPose object */
	grasp::RBPose::Ptr rbPosePtr;

	/** Transformation samples */
	grasp::RBPose::Sample::Seq poses;
	/** Transformation samples properties */
	golem::SampleProperty<golem::Real, grasp::RBCoord> pose;
	/** Pose distribution covariance */
	grasp::RBDist poseCov, poseCovInv;

	/** Distance between a and b */
	golem::Real distance(const grasp::RBCoord& a, const grasp::RBCoord& b) const;

	/** Kernel function */
	golem::Real kernel(golem::Real distance) const;

	/** Probability density value=p(s) for c */
	golem::Real density(const grasp::RBCoord &c) const;

	/** Global alignment */
	void alignGlobal(grasp::RBPose::Sample& solution, golem::Real& solutionEval, golem::U32& votes);
	/** Mean-shift clustering */
	void meanShiftClustering(grasp::RBPose::Sample& solution, grasp::RBPose::Sample::Seq& clusters);

	/** Stream string to print out the debug info for cluster analysis */
	std::string clusterStr;

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