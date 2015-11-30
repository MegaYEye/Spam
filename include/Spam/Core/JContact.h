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
enum HandChain {
	UNKNOWN = 0,
	THUMB,
	INDEX,
	MIDDLE,
	RING,
	PINKY,
};
/** Mode */
enum Mode {
	DISABLE = 0,
	ENABLE,
	INCONTACT,
};

//------------------------------------------------------------------------------
// base interface for guards
class Guard {
public:
	typedef std::vector<Guard> Seq;

	/** Force/torque threshold for the guard */
	golem::Real threshold;
	/** Force/torque measured */
	golem::Real force;

	class Desc {
	public:
		std::string name;
		
	};
};

/** Force/torque guards for Justin and BHAM robots */
class FTGuard {
public:
	typedef std::vector<FTGuard> Seq;

	/** FT sensors in 6 dimensions */
	static const size_t DIM = 6;
	/** Arm joints */
	static const size_t ARM = 7;
	/** Joint per finger */
	static const size_t JOINTSPERFINGER = 4;

	class Desc {
	public:
		/** Type of guard {|>,<,>} */
		FTGuardTypes type;
		/** Force/torque threshold for the guard */
		grasp::RealSeq limits;
		/** Hand chain */
		HandChain chain;
		/** Initial mode */
		Mode mode;

		/** Joint index at which the sensor is attached */
		golem::Configspace::Index jointIdx;

		Desc() {
			setToDefault();
		}

		void setToDefault() {
			type = FTGuardTypes::FTGUARD_ABS;
			limits.assign(DIM, golem::Real(0.1));
			chain = HandChain::UNKNOWN;
			jointIdx = golem::Configspace::Index(0);
		}

		/** Assert that the description is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(limits.size() != DIM, ac, "limits dimention does not agree");
			for (auto i = limits.begin(); i != limits.end(); ++i)
				grasp::Assert::valid(std::isfinite(*i), ac, "elements in limit is not finite");
		}
		/** Creates the object from the description. */
		virtual FTGuard* create() const;
	};

	/** Mode of the ft guard */
	Mode mode;
	/** Wrench treshold */
	grasp::RealSeq limits;
	/** Current FT sensor wrench */
	golem::Twist wrench;
	/** Joint index at which the sensor is attached */
	golem::Configspace::Index jointIdx;

	/** Check for disable mode */
	inline bool isDisable() const { return this->mode == Mode::DISABLE; }
	/** Check for enable mode */
	inline bool isEnable() const { return this->mode == Mode::ENABLE; }
	/** Check for contact mode */
	inline bool isInContact() const { return this->mode == Mode::INCONTACT; }

	/** Chain name */
	static const std::string ChainName[];
	/** Mode name */
	static const char* ModeName[];

	/** Set mode */
	inline void setMode(const Mode& mode) {
		this->mode = mode;
	}

	/** Guard in string */
	std::string str() const;

	std::string strForces() const;


	/** Check a list of sensors for contacts */
	static inline golem::U32 trigguered(const FTGuard::Seq& seq) {
		golem::U32 contacts = golem::U32(0);
		for (auto i = seq.begin(); i != seq.end(); ++i)
			if (i->isInContact())
				++contacts;
		return contacts;
	}

protected:
	/** Type of guard {|>,<,>} */
	FTGuardTypes type;

	/** Finger on which the FT sensor is attached */
	HandChain chain;

	/** Creates from a description file */
	void create(const Desc& desc);

	/** C'ctor */
	FTGuard();
};
/** Reads/writes guards from/to a given XML context */
//void XMLData(FTGuard::Desc& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _SPAM_JCONTACT_H_ */
