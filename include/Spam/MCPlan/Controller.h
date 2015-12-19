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
//! @Author:   Claudio Zito
//! @Date:     21/07/2012
//-------------------------------------------------------------------------
#pragma once
#ifndef _SPINTA_PLANNER_CONTROLLER_H_
#define _SPINTA_PLANNER_CONTROLLER_H_

//-------------------------------------------------------------------------

#include <Spinta/Planner/Data.h>
#include <Grasp/Grasp/Robot.h>

//-------------------------------------------------------------------------

MESSAGE_DEF(MsgCtrl, golem::Message)
MESSAGE_DEF(MsgCtrlInvalidDesc, MsgCtrl)
MESSAGE_DEF(MsgCtrlXMLParserNameNotFound, golem::MsgXMLParserNameNotFound)

//-------------------------------------------------------------------------

namespace spinta {

//-------------------------------------------------------------------------

class PushController {
public:
	typedef golem::shared_ptr<PushController> Ptr;
	friend class PushPlanner;
	friend class Desc;

	class Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC1(PushController, PushController::Ptr, Robot*)

	public:
		/** Smart pointer */
		typedef golem::shared_ptr<Desc> Ptr;
		friend class PushPlanner;

		/** Default constructor */
		Desc() {
			setToDefault();
		}
		/** Set to default values all the parameters */
		void setToDefault() {
		}
		/** Check for validity */
		bool isValid() const {
			return true;
		}
	};

	/** Compute the inverse model */
//	virtual PushAction getInput(const golem::Mat34 &xNear, const golem::Mat34 &xRand, golem::Mat34 *objPos) = 0;
	/** Return if the model is satisfied */
//	inline bool satisfied(const golem::Mat34 &x) { return model->satisfied(x); }

	/** Configspace action */
//	inline void configspaceAction(const golem::Controller::State& target) { pRobot->configspaceAction(target); };
	/** Workspace action (arm end-effector only) */
//	inline void workspaceAction(const golem::Mat34& target) { pRobot->workspaceAction(target); };

	/** Virtual destructor */
	virtual ~PushController();

protected:
	/** Pointer to the robot */
	Robot *pRobot;

	/** Create a heuristic from description */
	bool create(const Desc &desc);
	/** Default constructor */
	PushController(Robot *robot);
};

//-------------------------------------------------------------------------

void XMLData(PushController::Desc &val, golem::XMLContext* context, bool create = false);

//-------------------------------------------------------------------------

}; /* namespace */

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_CONTROLLER_H_ */