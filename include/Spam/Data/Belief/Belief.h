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
#ifndef _SPAM_DATA_BELIEF_H_
#define _SPAM_DATA_BELIEF_H_

//------------------------------------------------------------------------------

#include <Spam/HBPlan/Belief.h>
#include <Golem/Tools/Library.h>
#include <Golem/UI/Renderer.h>
#include <Grasp/Core/Data.h>
#include <Grasp/Core/UI.h>
#include <Spam/App/PosePlanner/Data.h>
#include <Spam/App/R2GPlanner/Data.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* graspDescLoader(void);
};

//------------------------------------------------------------------------------

namespace spam {
namespace data {

//------------------------------------------------------------------------------

class ItemBelief;
class HandlerBelief;

/** Data item representing belief state data.
*/
class GOLEM_LIBRARY_DECLDIR ItemBelief : public grasp::data::Item, public spam::data::BeliefState {
public:
	friend class HandlerBelief;

	/** Query file */
	mutable grasp::data::File dataFile;

	virtual const golem::Mat34& getModelFrame() const {
		return modelFrame;
	}
	virtual const golem::Mat34& getQueryTransform() const {
		return queryTransform;
	}

	Belief::Desc::Ptr getBeliefDesc() const;
	void set(const golem::Mat34 modelFrame, const golem::Mat34 queryTransform, const grasp::RBPose::Sample::Seq& poses, const grasp::RBPose::Sample::Seq& hypotheses);

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

protected:
	/** Data handler */
	HandlerBelief& handler;

	/** Query frame */
	golem::Mat34 modelFrame;
	/** Query transformation */
	golem::Mat34 queryTransform;

	/** Transformation samples */
	grasp::RBPose::Sample::Seq poses;
	/** Transformation sub-samples */
	grasp::RBPose::Sample::Seq hypotheses;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemBelief(HandlerBelief& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerBelief : public grasp::data::Handler {
public:
	friend class ItemBelief;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public grasp::data::Handler::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Pointer to belief desc file */
		Belief::Desc::Ptr pBeliefDescPtr;

		/** Belief suffix */
		std::string beliefSuffix;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			pBeliefDescPtr.reset(new Belief::Desc());
			beliefSuffix = getFileExtBelief();
		}
		/** Assert that the object is valid. */
		void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(pBeliefDescPtr != nullptr, ac, "Belief desc: null pointer.");	
			grasp::Assert::valid(beliefSuffix.length() > 0, ac, "beliefSuffix: empty");
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual grasp::data::Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: hypothesis-based state (.hbs) */
	static std::string getFileExtBelief();

protected:
	/** Belief suffix */
	std::string beliefSuffix;
	/** Planner index */
	golem::U32 plannerIndex;

	/** Pointer to the descriptor file */
	HandlerBelief::Desc desc;

	/** Creates render buffer */
	void createRender(const ItemBelief& item);

	/** Construct empty item, accessible only by Data. */
	virtual grasp::data::Item::Ptr create() const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerBelief(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_SPAM_DATA_BELIEF_H_*/
