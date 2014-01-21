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

#include <Golem/Tools/XMLParser.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Spam/Spam/Robot.h>
#include <Spam/Spam/GraphPlanner.h>
#include <Grasp/Grasp/Grasp.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

void XMLData(FTDrivenHeuristic::FTModelDesc& val, golem::XMLContext* context, bool create = false);

void XMLData(FTDrivenHeuristic::Desc& val, golem::XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(spam::Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

void XMLData(RagGraphPlanner::Desc &val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Trial data for a single demonstration and/or test trial in a binary format (stored on disk) */
class TrialData : public grasp::TrialData {
public:
	typedef std::map<std::string, TrialData> Map;
	friend class golem::Stream;

	/** Header name */
	static const char headerName [];
	/** Header version */
	static const golem::U32 headerVersion;

	/** High dim representation of pdf */
	grasp::RBPose::Sample::Seq pdf;
	/** Inverted covariance */
	grasp::RBCoord covarianceInv;
	/** Squared covariance */
	grasp::RBCoord covarianceSqrt;
	/** Low dim representation of pdf */
	grasp::RBPose::Sample::Seq samples;
	/** Low dim representation of pdf */
	grasp::RBPose::Sample::Seq init;
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
	std::vector<grasp::FTGuard> triggeredGuards;
	/** State of the robot at the time a contact occurred */
	golem::Controller::State::Seq triggeredStates;

	/** Safety configurations of the robot */
	grasp::State::Seq homeStates;

	/** Specifies if the replanning should be triggered */
	bool replanning;

	/** Combined action waypoints */
	golem::Controller::State::Seq executedTrajectory;
	/** Withdraw action waypoints */
	grasp::State::List approachWithdraw;

	/** Constructor */
	TrialData(const golem::Controller& controller) : grasp::TrialData(controller) {
	}
};

}; /** namespace */

//------------------------------------------------------------------------------

namespace golem {

template <> void Stream::read(spam::TrialData& trialData) const;
template <> void Stream::write(const spam::TrialData& trialData);

};	// namespace

//------------------------------------------------------------------------------

#endif /** _SPAM_DATA_H_ */