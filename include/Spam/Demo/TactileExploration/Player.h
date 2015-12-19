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
// @Date:     19/07/2012
//-------------------------------------------------------------------------
#ifndef _SPINTA_PLAYER_PLAYER_H_
#define _SPINTA_PLAYER_PLAYER_H_

//------------------------------------------------------------------------------

#include <Spam/MCPlan/RRT.h>
#include <Spam/MCPlan/GaussianProcess.h>
#include <Spam/App/R2GPlanner/R2GPlanner.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** RRTPlayer. */
class RRTPlayer : public R2GPlanner {
public:
	class Data : public R2GPlanner::Data {
	public:
		friend class RRTPlayer;

		/** Data bundle description */
		class Desc : public R2GPlanner::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Manager */
		virtual void setOwner(grasp::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Demo */
		RRTPlayer* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	/** RRTPlayer demo description */
	class Desc : public R2GPlanner::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Planner */
		RRT::Desc::Ptr pRRTDesc;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			R2GPlanner::Desc::setToDefault();
			dataDesc.reset(new Data::Desc);
			pRRTDesc.reset(new RRT::Desc);
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(pRRTDesc != nullptr, ac, "RRT description: invalid");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(RRTPlayer, golem::Object::Ptr, golem::Scene&)
	};

protected:
	/** Pointer to the planner */
	RRT::Ptr pPlanner;
	
	grasp::Cloud::PointSeq cloudPoints;
	/** Debug renderer */
	golem::DebugRenderer debugRenderer;
	/** Appereance for point clouds: model point cloud */
	grasp::Cloud::Appearance pointAppearance;

	/** golem::UIRenderer interface */
	virtual void render() const;

	void create(const Desc& desc);
	RRTPlayer(golem::Scene& scene);
	~RRTPlayer();
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPINTA_PLAYER_PLAYER_H_*/
