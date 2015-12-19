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
//! @Date:     20/07/2012
//-------------------------------------------------------------------------
#pragma once
#ifndef _SPINTA_PLANNER_HEURISTIC_H_
#define _SPINTA_PLANNER_HEURISTIC_H_

//-------------------------------------------------------------------------

#include <Spam/MCPlan/RRTree.h>

//-------------------------------------------------------------------------

namespace spam {

//-------------------------------------------------------------------------

class Model;

//-------------------------------------------------------------------------


class Heuristic {
public:
	typedef boost::shared_ptr<Heuristic> Ptr;

	class Desc {
	public:
		typedef boost::shared_ptr<Desc> Ptr;
		friend class Model;

		/** Linear distance factor */
		golem::Real linDistFac;
		/** Angular distance factor */
		golem::Real angDistFac;

		/** Default constructor */
		Desc() {
			setToDefault();
		}
		/** Set to default values all the parameters */
		void setToDefault() {
			linDistFac = 0.5;
			angDistFac = 0.5;
		}
		/** Check for validity */
		bool isValid() const {
			return true;
		}
	protected:
		/** Creates the object from the description. */
		Heuristic::Ptr create() const {
			Heuristic* pObject = new Heuristic();
			Heuristic::Ptr pointer(pObject);
			pObject->create(*this);
			return pointer;
		}
	};

	/** Computes cost function between two points */
	virtual golem::Real cost(const golem::Vec3& x1, const golem::Vec3& x2) const;
	/** Computes cost function of a state */
	virtual golem::Real cost(const golem::Vec3& x) const;

protected:
	/** Linear distance factor */
	golem::Real linDistFac;
	/** Angular distance factor */
	golem::Real angDistFac;

	/** Create a heuristic from description */
	bool create(const Desc &desc);
	/** Default constructor */
	Heuristic();
};

//-------------------------------------------------------------------------

//void XMLData(spinta::Heuristic::Desc &val, golem::Context *context, golem::XMLContext *xmlcontext, bool create = false);

//-------------------------------------------------------------------------

}; /* namespace */

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_HEURISTIC_H_ */