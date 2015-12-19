//-------------------------------------------------------------------------
//                  Single Finger Pushing Tool (SPinTa)
//-------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Single Finger Pushing Tool (SPinTa), 
//       University of Birmingham, nor the names of its contributors may be 
//       used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------
// @Author:   Claudio Zito
// @Date:     24/07/2012
//-------------------------------------------------------------------------
#ifndef _SPINTA_PLANNER_PUSHPLANNER_H_
#define _SPINTA_PLANNER_PUSHPLANNER_H_

//-------------------------------------------------------------------------

#include <Spinta/Planner/Controller.h>
#include <Spinta/Planner/Model.h>
#include <Golem/Math/Rand.h>

//-------------------------------------------------------------------------

namespace spinta {

//-------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgPushPlanner, golem::Message)
MESSAGE_DEF(MsgPushPlannerInvalidDesc, MsgPushPlanner)
MESSAGE_DEF(MsgPushlannerXMLParserNameNotFound, golem::MsgXMLParserNameNotFound)

//-------------------------------------------------------------------------

class PushPlanner {
public:
	/** Pointer */
	typedef golem::shared_ptr<PushPlanner> Ptr;
	friend class Planner;
	friend class RRT;
	friend class TwoLevelRRT;

	/** Description file */
	class Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC2(PushPlanner, PushPlanner::Ptr, Model::Ptr, Robot *)

	public:	
		/** Pointer */
		typedef golem::shared_ptr<Desc> Ptr;

		/** Number of exploring directions */
		golem::U8 numDirections;
		/** Number of pushing */
		golem::U8 numPushes;
		/** Spatial step size of integration in 3D */
		golem::Vec3 deltaS;
		/** non Gaussian perturbation on the action */
		bool nonGaussianPerturbation;
		
		/** Max number of fails */
		golem::U8 numMaxFails;
		/** Max number of iterations */
		golem::U8 numMaxIters;

		/** Accuracy threshold to target pose */
		golem::Real accToTarget;

		/** Trajectory offset */
		golem::Vec3 offset;

		/** Constructor */
		Desc() {
			setToDefault();
		}
		/** Set parameter to default */
		void setToDefault() {
			numDirections = 4;
			numPushes = 1;
			deltaS.setZero();
			nonGaussianPerturbation = true;
			numMaxFails = 3;
			numMaxIters = 3;
			offset = golem::Vec3(0.08, 0.15, 0.08);
			accToTarget = 0.009;
		}
		/** Check if valid */
		bool isValid() const {
			return true;
		}

		friend class Planner;
		friend class RRT;
		friend class TwoLevelRRT;
	};

	/** Reset the planner */
	virtual void reset();
	/** Determine if the gap error is satisfied */
	virtual bool satisfied(const golem::Mat34 &x);
	/** Determine the input to extend the RRT planner */
	virtual PushAction::Seq getInput(const golem::Mat34 &xNear, const golem::Mat34 &xRand, golem::Mat34 *objPose, bool &success);

	/** Virtual desctuctor */
	virtual ~PushPlanner();

protected:
	/** Model */
	Model::Ptr pModel;
	/** Pointer to robot */
	Robot *pRobot;
	/** Generator of pseudo random numbers */
	golem::Rand rand;
	/** Context */
	golem::Context *context;

	/** Number of exploring directions */
	golem::U8 numDirections;
	/** Number of pushes */
	golem::U8 numPushes;
	/** Spatial step size of integration in 3D */
	golem::Vec3 deltaS;
	/** non Gaussian perturbation on the action */
	bool nonGaussianPerturbation;
	/** Max number of fails */
	golem::U8 numMaxFails;
	/** Max number of iterations */
	golem::U8 numMaxIters;
	/** Trajectory offset */
	golem::Vec3 offset;

	/** Accuracy threshold to target pose */
	golem::Real accToTarget;

	/** Select a random state on 2.5 dimensions */
	virtual golem::Mat34 randomState();
	
	/** Select random push actions */
	PushAction::Seq selectPushActions();
	/** Check for accuracy */
	inline bool satisfied(const golem::Mat34 &current, const golem::Mat34 &target) {
		return pModel->cost(current, target) < accToTarget;
	}
	/** Checks for positive feedback */
	inline bool satisfied(const golem::Mat34 &x1, const golem::Mat34 &x2, const golem::Mat34 &target) const {
		return (pModel->cost(x2, target) - pModel->cost(x1, target) <= accToTarget);
	}
	/** Transition dynamics */
	golem::Mat34 integrate(const golem::Mat34 &current, const golem::Mat34 &target, PushAction *action);
	/** Move the robot from current pose to the target one */
	void generateTrajectorySim(const golem::Mat34 &target); 

	/** Create planner from description */
	bool create(const Desc &desc);
	/** Default constructor takes a controller */
	PushPlanner(Model::Ptr model, Robot *robot);
};

void XMLData(PushPlanner::Desc &val, golem::Context *context, golem::XMLContext *xmlcontext, bool create = false);

//-------------------------------------------------------------------------

//class ICubPushPlanner {
// public:
//	 typedef golem::shared_ptr<ICubPushPlanner> Ptr;
//	 friend class TwoLevelRRT;
//
//	 class Desc {
//	 protected:
//		/** Creates the object from the description. */
//		CREATE_FROM_OBJECT_DESC1(ICubPushPlanner, ICubPushPlanner::Ptr, ICubModel::Ptr)
//
//	 public:
//		 typedef golem::shared_ptr<Desc> Ptr;
//		 friend class TwoLevelRRT;
//
//		 /** Pointer */
//		 ICubController::Desc::Ptr pControllerDesc;
//
//		/** Number of pushing */
//		golem::U8 numPushes;
//		/** Max number of fails */
//		golem::U8 numMaxFails;
//		/** Max number of iterations */
//		golem::U8 numMaxIters;
//
//		/** Constructor */
//		Desc() {
//			setToDefault();
//		}
//		/** Set parameter to default */
//		void setToDefault() {
//			numPushes = 4;
//			numMaxFails = 3;
//			numMaxIters = 3;
//			pControllerDesc.reset();
//			 pControllerDesc.reset(new ICubController::Desc);
//		 }
//		 bool isValid() const {
//			 return true;
//		 }
//	 };
//
//	/** Determine the input to extend the RRT planner */
//	PushAction getInput(const golem::Mat34 &xNear, const golem::Mat34 &xRand, golem::Mat34 *objPos);
//
//	virtual ~ICubPushPlanner();
//
//protected:
//	/** Robot controller */
//	ICubController::Ptr pController;
//	/** Model */
//	ICubModel::Ptr pModel;
////	ICubModel::Ptr pICubModel;
//
//	golem::U8 numPushes;
//	/** Max number of fails */
//	golem::U8 numMaxFails;
//	/** Max number of iterations */
//	golem::U8 numMaxIters;
//
//	/** Create planner from description */
//	bool create(const Desc &desc);
//	/** Constructor */
//	ICubPushPlanner(ICubModel::Ptr model);
//};

//-------------------------------------------------------------------------

//void XMLData(ICubPushPlanner::Desc &val, golem::Context *context, golem::XMLContext *xmlcontext, bool create = false);

//-------------------------------------------------------------------------

}; /* namespace */

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_PUSHPLANNER_H_ */