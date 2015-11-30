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
#ifndef _SPAM_HYPOTHESIS_H_
#define _SPAM_HYPOTHESIS_H_

//------------------------------------------------------------------------------

#include <Grasp/Core/RBPose.h>
#include <Spam/Core/Collision.h>

//------------------------------------------------------------------------------

namespace flann {
	template <typename T> struct L2_Simple;
};

namespace pcl {
	struct PointXYZ;
	template <typename T, typename Dist> class KdTreeFLANN;
	struct PolygonMesh;
};

namespace grasp {
	class Manipulator;
};

namespace spam {
	class FTDrivenHeuristic;
	class Belief;
};

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Hypothesis over object poses */
class Hypothesis {
public:
	friend class FTDrivenHeuristic;
	friend class Belief;
	typedef golem::shared_ptr<Hypothesis> Ptr;
	typedef std::map<golem::U32, Ptr> Map;
	typedef std::vector<Ptr> Seq;

	/** Bounds Appearance */
	class BoundsAppearance {
	public:
		/** Show bounds solid */
		bool showSolid;
		/** Show bounds wire frames */
		bool showWire;
		/** Bounds solid colour */
		golem::RGBA solidColour;
		/** Bounds wire colour */
		golem::RGBA wireColour;
		/** Bounds wireframe thickness */
		golem::Real wireWidth;

		/** Constructs from description object */
		BoundsAppearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showSolid = false;
			showWire = true;
			solidColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireWidth = golem::Real(1.0);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (wireWidth <= golem::REAL_ZERO)
				return false;
			return true;
		}

		/** Draw bounds */
		void draw(const golem::Bounds::Seq& bounds, golem::DebugRenderer& renderer) const;
	};

	/** Appearance */
	class Appearance {
	public:
		/** Show frame */
		bool showFrames;
		/** Show point cloud */
		bool showPoints;
		/** Frame size of the sample */
		golem::Vec3 frameSize;
		/** clolour of the point cloud */
		golem::RGBA colour;

		/** Bounds colour */
		BoundsAppearance bounds;

		/** Constructs from description object */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showFrames = true;
			showPoints = true;
			frameSize.set(golem::Real(0.02));
			colour = golem::RGBA::MAGENTA;
			bounds.setToDefault();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!bounds.isValid())
				return false;
			if (!frameSize.isPositive())
				return false;
			return true;
		}
	};

	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		Appearance appearance;
		Collision::Desc::Ptr collisionDescPtr;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Hypothesis::Ptr create(const grasp::Manipulator& manipulator) const {
			return Hypothesis::Ptr(new Hypothesis(manipulator, *this));
		}
		/** Sets description to default values */
		void setToDefault() {
			appearance.setToDefault();
			collisionDescPtr.reset(new Collision::Desc());
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!appearance.isValid())
				return false;
			if (collisionDescPtr != nullptr && !collisionDescPtr->isValid())
				return false;
			return true;
		}

	};

	/** Create */
	virtual void create(const golem::U32 idx, const golem::Mat34 &trn, const grasp::RBPose::Sample &s, golem::Rand& rand, const grasp::Cloud::PointSeq& points);

	/** Returns this sample in model frame **/
	inline grasp::RBPose::Sample toRBPoseSample() const { return sample; };
	/** Returns this sample in global frame (default: robot frame) **/
	inline grasp::RBPose::Sample toRBPoseSampleGF() const { return grasp::RBPose::Sample(sample.toMat34() * modelFrame, sample.weight, sample.cdf); };
	/** Returns the point cloud in global frame */
	inline grasp::Cloud::PointSeq getCloud() const { return points; };

	/** Collision detection at a given waypoint */
	inline bool check(const Collision::Waypoint& waypoint, const grasp::Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->check(waypoint, config, debug);
	};
	/** Collision detection at a given waypoint */
	inline bool check(const Collision::FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->check(desc, rand, config, debug);
	}

	/** Collision detection at a given waypoint */
	inline virtual golem::Real estimate(const Collision::FlannDesc& desc, const grasp::Manipulator::Config& config, golem::Real maxDist = golem::REAL_MAX, bool debug = false) const {
		return collisionPtr->estimate(desc, config, maxDist, debug);
	}

	/** Collision likelihood estimation at a given waypoint */
	inline golem::Real evaluate(const Collision::Waypoint& waypoint, const grasp::Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->evaluate(waypoint, config, debug);
	}
	/** Collision likelihood estimation at a given waypoint */
	inline golem::Real evaluate(const Collision::FlannDesc& desc, const grasp::Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->evaluate(desc, config, debug);
	}

	/** Return seq of bounds */
	golem::Bounds::Seq bounds();

	/** Prints global pose of the hypothesis */
	std::string str() const;

	/** Draw hypotheses */
	void draw(golem::DebugRenderer &renderer) const;

	/** Draw collisions */
	void draw(const Collision::Waypoint &waypoint, const grasp::Manipulator::Config& config, golem::DebugRenderer& renderer) const;
	/** Draw collision using kdtree */
	void draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Config& config) const;
	/** Draw estimate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Config& config, const Collision::FlannDesc& desc) const {
		collisionPtr->draw(renderer, config, desc);
	}
	/** Draw simulate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Config& config, std::vector<golem::Configspace::Index> &joints, grasp::RealSeq &forces, const Collision::FlannDesc& desc) const {
		collisionPtr->draw(renderer, config, joints, forces, desc);
	}
	Appearance appearance;

protected:
	/** Identifier */
	golem::U32 index;
	/** Model frame **/
	golem::Mat34 modelFrame;
	/** Hypothesis. NOTE: contains the query (or sample) frame w.r.t model frame **/
	grasp::RBPose::Sample sample;
	/** Point cloud */
	grasp::Cloud::PointSeq points;

	/** Bounding box desc for the object */
	golem::BoundingBox::Desc boundsDesc;


	/** Manipulator */
	const grasp::Manipulator& manipulator;
	/** Description */
	const Desc desc;

	/** Collision detection pointer */
	spam::Collision::Ptr collisionPtr;

	/** Create */
	Hypothesis(const grasp::Manipulator& manipulator, const Desc& desc);
};

//------------------------------------------------------------------------------

void XMLData(Hypothesis::Desc& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _SPAM_HYPOTHESIS_H_ */