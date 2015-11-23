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
#ifndef _SPAM_JCONTACT_H_
#define _SPAM_JCONTACT_H_

//------------------------------------------------------------------------------

#include <Grasp/Contact/Manipulator.h>
#include <Golem/Ctrl/Data.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Type of guards for Justin */
enum FTGuardTypes {
	/** Absolute values */
	FTGUARD_ABS = 0,
	/** Less than */
	FTGUARD_LESSTHAN,
	/**Greater than */
	FTGUARD_GREATERTHAN,
};
/** HandChains */
enum HandChains {
	UNKNOWN = 0,
	THUMB,
	INDEX,
	MIDDLE,
	RING,
	PINKY,
};

/** Force/torque guards for Justin and BHAM robots */
class FTGuard {
public:
	typedef std::vector<FTGuard> Seq;
	 
	/** Name of the guard {right,left}_{tcp,thumb,tip,middle,ring} */
	std::string name;
	/** Type of guard {|>,<,>} */
	FTGuardTypes type;
	/** Force/torque threshold for the guard */
	golem::Real threshold;
	/** Force/torque measured */
	golem::Real force;

	/** Last arm's joint index */
	golem::Configspace::Index armIdx;
	/** Joint index */
	golem::Configspace::Index jointIdx;
	/** Number of chains in the hand */
	golem::U32 handChains;
	/** Number of joints per fingers */
	golem::U32 fingerJoints;

	/** C'ctor */
	FTGuard(const grasp::Manipulator &manipulator);

	/** D'ctor */
	virtual ~FTGuard() {};

	/** Prints the guard in the format for Justin.
	Example: {right,left}_{tcp,thumb,tip,middle,ring} {0,1,2,[3,4]} {|>,<,>} value **/
	std::string str() const;

	/** Returns the chain index to the finger [1, 5] */
	inline golem::U32 getHandChain() {
		return ((golem::U32)(jointIdx - armIdx) / fingerJoints) + 1;
	}
	/** Returns the chain index to the finger [1, 5] */
	inline const golem::U32 getHandChain() const {
		return ((golem::U32)(jointIdx - armIdx) / fingerJoints) + 1;
	}
	/** Returns the index of the joint per finger [0,3] */
	inline golem::U32 getHandJoint() {
		return (golem::U32)(jointIdx - armIdx) % fingerJoints;
	}
	/** Returns the index of the joint per finger [0,3] */
	inline const golem::U32 getHandJoint() const {
		return (golem::U32)(jointIdx - armIdx) % fingerJoints;
	}

	/** Sets the chain and joint iterators */
	void create(const golem::Configspace::Index& joint);
};
/** Reads/writes guards from/to a given XML context */
void XMLData(FTGuard &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _SPAM_JCONTACT_H_ */
