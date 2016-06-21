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
//! @Date:     25/03/2014
//------------------------------------------------------------------------------
#pragma once
#ifndef _SPAM_POSEPLANNER_DATA_H_
#define _SPAM_POSEPLANNER_DATA_H_

//------------------------------------------------------------------------------

#include <Spam/HBPlan/Belief.h>
#include <Spam/HBPlan/Heuristic.h>
#include <Spam/HBPlan/GraphPlanner.h>

//------------------------------------------------------------------------------

namespace spam {
namespace data {

//------------------------------------------------------------------------------

class BeliefState {
public:
	virtual const golem::Mat34& getModelFrame() const = 0;
	virtual const golem::Mat34& getQueryTransform() const = 0;
	virtual Belief::Desc::Ptr getBeliefDesc() const = 0;
	virtual void set(const golem::Mat34 modelFrame, const golem::Mat34 queryTransform, const grasp::RBPose::Sample::Seq& poses, const grasp::RBPose::Sample::Seq& hypotheses) = 0;
};

//------------------------------------------------------------------------------

};	// namespace
}; // namespace

//namespace golem {
//
//template <> void Stream::read(spam::Data &data) const;
//template <> void Stream::write(const spam::Data &data);
//
//};	// namespace


#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
