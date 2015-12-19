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
#ifndef _SPAM_DEMO_R2G_H_
#define _SPAM_DEMO_R2G_H_

//------------------------------------------------------------------------------

#include <Spam/App/R2GPlanner/R2GPlanner.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Reach-2-grasp demo. */
class R2GDemo : public R2GPlanner {
public:
	/** R2G demo description */
	class Desc : public R2GPlanner::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(R2GDemo, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	R2GDemo(golem::Scene& scene);
	~R2GDemo();
};

//------------------------------------------------------------------------------

};

//------------------------------------------------------------------------------

#endif /*_SPAM_DEMO_R2G_H_*/
