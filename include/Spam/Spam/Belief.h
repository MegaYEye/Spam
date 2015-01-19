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
#include <Grasp/Core/Robot.h>
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

	///** Appearance */
	//class Appearance {
	//public:
	//	/** Show frame */
	//	bool showFrames;
	//	/** Show point cloud */
	//	bool showPoints;
	//	/** Frame size of the sample */
	//	golem::Vec3 frameSize;
	//	/** clolour of the point cloud */
	//	golem::RGBA colour;

	//	/** Constructs from description object */
	//	Appearance() {
	//		setToDefault();
	//	}
	//	/** Sets the parameters to the default values */
	//	void setToDefault() {
	//		showFrames = true;
	//		showPoints = true;
	//		frameSize.set(golem::Real(0.02));
	//		colour = golem::RGBA::MAGENTA;
	//	}
	//	/** Checks if the description is valid. */
	//	bool isValid() const {
	//		if (!frameSize.isPositive())
	//			return false;
	//		return true;
	//	}
	//};

	/** Forward model to describe hand-object interactions */
	class RigidBodyTransformation {
	public:
		/** Smart pointer */
		typedef golem::shared_ptr<RigidBodyTransformation> Ptr;

		/** Contructor */
		RigidBodyTransformation() {};
		
		/** Sets the initial extimated pose */
		void set(golem::Mat34 &p) { pose = p; };
		/** Returns the trasnformation from the a priori pose to the a posteriori */
		golem::Mat34 transform(golem::Mat34 &p);

	protected:
		/** A priori pose of the object */
		golem::Mat34 pose;
	};

	//	/** Hypothesis over object poses */
	//class Hypothesis {
	//public:
	//	friend class FTDrivenHeuristic;
	//	friend class Belief;
	//	typedef golem::shared_ptr<Hypothesis> Ptr;
	//	typedef std::map<golem::U32, Ptr> Map;
	//	typedef std::vector<Ptr> Seq;

	//	/** Default construtor */
	//	Hypothesis() {
	//		appearance.setToDefault();
	//	}
	//	/** Complete constructor */
	//	Hypothesis(const golem::U32 idx, const golem::Mat34 &trn, const grasp::RBPose::Sample &s, grasp::Cloud::PointSeq &p) {
	//		index = idx;
	//		modelFrame = trn;
	//		sample = s;
	//		for (grasp::Cloud::PointSeq::const_iterator i = p.begin(); i != p.end(); ++i)
	//			points.push_back(*i);
	//		build();
	//		//buildMesh();

	//		appearance.setToDefault();
	//	}
	//	/** Destrutor */
	//	~Hypothesis() {
	//		pTree.release();
	//		pTriangles.release();
	//	}

	//	/** Distance to nearest k points on the object's surface */
	//	golem::Real dist2NearestKPoints(const grasp::RBCoord &pose, const golem::Real &maxDist = golem::Real(1.0), const size_t clusters = 10, const size_t &nIndeces = 100, const bool normal = true) const;

	//	/** Nearest K points */
	//	size_t nearestKPoints(const grasp::RBCoord &pose, grasp::Cloud::PointSeq &points, std::vector<float> &distances, const size_t clusters = 50) const;

	//	/** Returns this sample in model frame **/
	//	inline grasp::RBPose::Sample toRBPoseSample() const { return sample; };
	//	/** Returns this sample in global frame (default: robot frame) **/
	//	inline grasp::RBPose::Sample toRBPoseSampleGF() const { return grasp::RBPose::Sample(sample.toMat34() * modelFrame, sample.weight, sample.cdf); };
	//	/** Returns the point cloud in global frame */
	//	inline grasp::Cloud::PointSeq getCloud() const { return points; };

	//	/** Draw hypothesis */
	//	void draw(golem::DebugRenderer& renderer) const;

	//	Appearance appearance;

	//protected:
	//	/** Builds a pcl::PointCloud and its kd tree */
	//	bool build();
	//	/** Builds a pcl::PointCloud and its mesh */
	//	bool buildMesh();

	//	/** Identifier */
	//	golem::U32 index;
	//	/** Model frame **/
	//	golem::Mat34 modelFrame;
	//	/** Hypothesis. NOTE: contains the query (or sample) frame w.r.t model frame **/
	//	grasp::RBPose::Sample sample;
	//	/** Point cloud */
	//	grasp::Cloud::PointSeq points;
	//	/** Kd tree */
	//	golem::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>> pTree;
	//	/** Polygon mesh */
	//	golem::shared_ptr<pcl::PolygonMesh> pTriangles;
	//};

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
		CREATE_FROM_OBJECT_DESC1(Belief, grasp::RBPose::Ptr, golem::Context&)
		/** Sets the parameters to the default values. */
		void setToDefault() {
			grasp::RBPose::Desc::setToDefault();
//			tactile.setToDefault();
			numHypotheses = 5;
			maxSurfacePoints = 10000;
			std::fill(&covariance[0], &covariance[3], golem::Real(0.02)); // Vec3
			std::fill(&covariance[3], &covariance[7], golem::Real(0.005)); // Quat
			lambda = 1;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!grasp::RBPose::Desc::isValid())
				return false;
			//if (!tactile.isValid())
			//	return false;
			return true;
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
	inline golem::SampleProperty<golem::Real, grasp::RBCoord, grasp::RBCoord::N> getSampleProperties() { return sampleProperties; };

	/** Sets the hypothesis for planning. NOTE: returns the action frame **/
	grasp::RBPose::Sample createHypotheses(const grasp::Cloud::PointSeq& model, const golem::Mat34 &transform/*, const bool init = true*/);
	/** Returns the low-dimensional representation of the density **/
	grasp::RBPose::Sample::Seq getHypothesesToSample() const;

	/** Creates query object pose distribution */
	void createQuery(const grasp::Cloud::PointSeq& points);
	/** Creates a new set of poses (resampling wheel algorithm) */
	virtual void createResample();
	/** Creates belief update (on importance weights) given the robot's pose and the current belief state. NOTE: weights are normalised. */
	void createUpdate(const grasp::Manipulator *manipulator, const grasp::Robot *robot, const golem::Waypoint &w, const FTGuard::Seq &triggeredGuards, const grasp::RealSeq &force);
	/** Evaluates the likelihood of reading a contact between robot's pose and the sample */
	golem::Real evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const grasp::RealSeq &forces, std::vector<bool> &triggered, bool intersect = true);
	//golem::Real evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const golem::Real &force, bool jointZero, bool intersect = true);

	/** Checks collision with the max likelihood fitting estimation */
//	bool intersect(const golem::Bounds::Seq &bounds, const golem::Mat34 &pose) const;

	/** Probability density value=p(d) for a given distance between finger and hypothesis */
	golem::Real density(const golem::Real dist) const;

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

	/** Sets initial pose to the forward model */
	inline void setInitObjPose(golem::Mat34 &p) {
		trn.set(p);
	};

	/** Computes the transformation */
	inline golem::Mat34 transform(golem::Mat34 &p) {
		return trn.transform(p);
	};

	/** Gets the covariance det associated with the hypotheses **/
	inline golem::Real getCovarianceDet() { return covarianceDet; };

	/** Draw samples */
	void drawSamples(const size_t numSamples, golem::DebugRenderer& renderer) const;

	/** Draw hypotheses */
	void drawHypotheses(golem:: DebugRenderer &renderer, const bool showOnlyMeanPose = false) const;

	/** Acquires manipulator */
	inline void setManipulator(grasp::Manipulator *ptr) { manipulator.reset(ptr); /*collision.reset(new Collision(context, *manipulator));*/ };

	/** Pose description */
	Desc myDesc;

	/** Real pose of the object */
	grasp::RBCoord realPose;

protected:
	/** Pointer to the RBPose object */
	grasp::RBPose::Ptr pRBPose;
	/** Forward model of hand-object interaction */
	RigidBodyTransformation trn;

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

	/** Returns the max weight associated to the corrent samples */
	golem::Real maxWeight() const;

	/** Initial belief distribution. NOTE: Used for the reset method. */
	grasp::RBPose::Sample::Seq initPoses;
	/** Transformation samples properties */
	golem::SampleProperty<golem::Real, grasp::RBCoord, grasp::RBCoord::N> sampleProperties, initProperties;

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