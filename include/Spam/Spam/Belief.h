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

#include <Grasp/Grasp/Data.h>
#include <Grasp/Grasp/RBPose.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Pose distribution and estimation with tactile and visual feedback */
class Belief : public grasp::RBPose {
public:
	typedef golem::shared_ptr<Belief> Ptr;

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

	/** Description file of tactile observational model */
	class TactileDesc {
	public:
		typedef golem::shared_ptr<TactileDesc> Ptr;

		/** Number of distribution kernels */
		size_t kernels;
		/** Posteriori (after contact) kernel diagonal covariance scale metaparameter */
		grasp::RBCoord covariance;
		
		/** Enables/disables building a fake distribution for testing porpuses */
		bool test;
		/** Noise lin/ang components */
		grasp::RBDist stddev;

		/** Constructs description. */
		TactileDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			kernels = 10000;
			std::fill(&covariance[0], &covariance[3], golem::Real(0.0001)); // Vec3
			std::fill(&covariance[3], &covariance[7], golem::Real(0.0001)); // Quat
			test = false;
			stddev.set();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (kernels < 1)
				return false;
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

		/** Tactile description file */
		TactileDesc tactile;

		/** Max number of clusters for the sampling algorithm */
		size_t kmeans;
		/** Rejection threashold for clusters */
		golem::Real rejection;
		/** Max number of iteration */
		size_t iterMax;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Creates object from the description. */
		CREATE_FROM_OBJECT_DESC1(Belief, grasp::RBPose::Ptr, golem::Context&)
		/** Sets the parameters to the default values. */
		void setToDefault() {
			grasp::RBPose::Desc::setToDefault();
			tactile.setToDefault();
			kmeans = 0;
			rejection = golem::Real(0.01);
			iterMax = 10;
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!grasp::RBPose::Desc::isValid())
				return false;
			if (!tactile.isValid())
				return false;
			return true;
		}
	};

	/** Sample pose from distribution */
	grasp::RBCoord sample() const;
	/** Sample pose from distribution */
	grasp::RBCoord sample(const grasp::RBCoord &kernel) const;

	/** Maximum likelihood estimation */
	Sample maximum();

	/** Probability density value=p(s) for c */
	golem::Real density(const grasp::RBCoord &c) const;

	/** Transformation samples */
	const Sample::Seq& getHypotheses() const {
		return mfsePoses;
	}
	/** Transformation samples */
	Sample::Seq& getHypotheses() {
		return mfsePoses;
	}
	/** Samples hypothesis as ML pose plus noise */
	Sample sampleHypothesis();
	/** Returns samples properties */
	inline golem::SampleProperty<golem::Real, grasp::RBCoord, grasp::RBCoord::N> getSampleProperties() { return mfseProperties; };

	/** Creates query object pose distribution */
	void createQuery(const grasp::Point::Seq& points, golem::U32 label = grasp::Point::LABEL_OBJECT);
	/** Creates a new set of weights for the current poses */
	virtual void createUpdate(const golem::Mat34 &trn);
	/** Creates a new set of poses (resampling wheel algorithm) */
	virtual void createResample();

	/** Resets the high representation distribution */
	inline void reset() {
		mfseProperties = initProperties;
		mfsePoses.clear();
		for (grasp::RBPose::Sample::Seq::const_iterator i = initPoses.begin(); i != initPoses.end(); ++i)
			mfsePoses.push_back(*i);
		context.write("spam::Belief::createQuery(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", mfseProperties.covariance[0], mfseProperties.covariance[1], mfseProperties.covariance[2], mfseProperties.covariance[3], mfseProperties.covariance[4], mfseProperties.covariance[5], mfseProperties.covariance[6]);
	}

	/** Sets initial pose to the forward model */
	inline void setInitObjPose(golem::Mat34 &p) {
		trn.set(p);
	};

	/** Computes the transformation */
	inline golem::Mat34 transform(golem::Mat34 &p) {
		return trn.transform(p);
	};

	/** Pose description */
	Desc myDesc;

	/** Real pose of the object */
	grasp::RBCoord realPose;

protected:
	/** Pointer to the RBPose object */
	grasp::RBPose::Ptr pRBPose;
	/** Forward model of hand-object interaction */
	RigidBodyTransformation trn;

	/** Returns the max weight associated to the corrent samples */
	golem::Real maxWeight() const;

	/** Transformation samples for the maximum likelihood estimation */
	grasp::RBPose::Sample::Seq mfsePoses, initPoses;
	/** Transformation samples properties */
	golem::SampleProperty<golem::Real, grasp::RBCoord, grasp::RBCoord::N> mfseProperties, initProperties;
	/** Transformation samples for the low dimentional representation */
	grasp::RBPose::Sample::Seq ldPoses;
	/** Transformation samples properties */
	golem::SampleProperty<golem::Real, grasp::RBCoord, grasp::RBCoord::N> ldpose;

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
#endif /** _SPAM_SPAM_RBPOSE_H_ */