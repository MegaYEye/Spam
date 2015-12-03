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
#ifndef _SPAM_RAGPLANNER_RAGPLANNER_H_
#define _SPAM_RAGPLANNER_RAGPLANNER_H_

//------------------------------------------------------------------------------

#include <Spam/App/PosePlanner/PosePlanner.h>
#include <Spam/App/R2GPlanner/Data.h>
#include <Grasp/ActiveCtrl/ArmHandForce/ArmHandForce.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** 
	RAGPlanner. 
	Basic class for planning Reach-and-grasp trajectory.
	This class provides (1) a sampling method to generate a low dimension
	representation of uncertainty over object poses, and (2) a planner which
	builds trajectories that are more likely to distinguish between the 
	miximum likelihood pose of the object and any sampled poses.
	Based on Platt R. et al. "A hypothesis-based algorithm for planning and
	control in non-Gaussian belief spaces", 2011.
*/
class R2GPlanner : public PosePlanner, protected golem::Profile::CallbackDist {
public:
	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
	typedef std::function<void(const golem::Controller::State&, grasp::RealSeq&, std::vector<golem::Configspace::Index>&)> GuardsReader;
	/** Force reader. Overwrite the ActiveCtrl reader. this retrieves the triggered joints */
	typedef std::function<std::vector<size_t>(const golem::Controller::State&, grasp::RealSeq&)> GuardFTReader;
	/** FT Sequence */
	typedef std::vector<grasp::FT*> FTSensorSeq;

	/** Data */
	class Data : public PosePlanner::Data {
	public:
		friend class R2GPlanner;
		
		/** Data bundle description */
		class Desc : public PosePlanner::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Specifies if guard have been triggered during the perform of the action */
		int triggered;

		/** Specifies if the replanning should be triggered */
		bool replanning;
		/** Enable the release of the object before withdrawing */
		bool release;

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Demo */
		R2GPlanner* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	class Desc : public PosePlanner::Desc {
	protected:
		GRASP_CREATE_FROM_OBJECT_DESC1(R2GPlanner, golem::Object::Ptr, golem::Scene&)

	public:
		/** Smart pointer */
		typedef golem::shared_ptr<Desc> Ptr;

		/** Enables/disables explicit use of uncertainty in planning */
		bool uncEnable;
		/** Enables/disables replanning */
		bool singleGrasp;
		/** Enables/disables withdrawing to the home pose */
		bool withdrawToHomePose;

		/** Downsampling parameter */
		size_t maxModelPoints;

		/** Collision description file. Used for collision with the ground truth */
		Collision::Desc::Ptr objCollisionDescPtr;

		/** Guards to retrieve a contact */
		grasp::RealSeq fLimit;

		/** Trajectory duration */
		golem::SecTmReal trjDuration;
		/** Trajectory idle time */
		golem::SecTmReal trjIdle;
		/** Trajectory extrapolation */
		grasp::RealSeq trjExtrapol;
		/** Trajectory extrapolation factor */
		golem::Real trjExtrapolFac;
		/** Performance duration offset */
		golem::SecTmReal trjPerfOff;
		/** Trajectory profile description */
		golem::Profile::Desc::Ptr pProfileDesc;
		/** Trajectory profile configspace distance multiplier */
		grasp::RealSeq distance;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			PosePlanner::Desc::setToDefault();

			trjDuration = golem::SecTmReal(2.0);
			trjIdle = golem::SecTmReal(1.0);
			trjExtrapolFac = golem::Real(1.0);
			trjExtrapol.assign(golem::Configspace::DIM, golem::REAL_ONE);
			trjPerfOff = golem::SecTmReal(0.0);
			pProfileDesc.reset(new golem::Profile::Desc);
			distance.assign(golem::Configspace::DIM, golem::REAL_ONE);

			uncEnable = true;
			singleGrasp = false;
			withdrawToHomePose = false;

			fLimit.assign(20, golem::REAL_ZERO);

			maxModelPoints = 5000;

			objCollisionDescPtr.reset(new Collision::Desc());
		}

		/** Checks if the description is valid. */
		virtual void assertValid(const grasp::Assert::Context &ac) const {
			PosePlanner::Desc::assertValid(ac);

			grasp::Assert::valid(maxModelPoints > 0, ac, "Max model point is not positive");
			grasp::Assert::valid(objCollisionDescPtr != nullptr, ac, "Collision description: invalid");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	inline golem::Real getFilterForce(const golem::I32 i) {
		return handFilteredForce[i];
	}
	inline grasp::RealSeq getFilterForce() {
		return handFilteredForce;
	}

	/** Force reader */
	GuardsReader guardsReader;
	GuardFTReader guardsFTReader;
	/** Collect history of FT readings for statistics */
	grasp::ActiveCtrlForce::ForceReader collectFTInp;
	/** FT 2-oreder high-pass filter */
	grasp::ActiveCtrlForce::ForceReader ftFilter;

protected:
	/** Mode of Active Ctrl */
	grasp::ActiveCtrlForce::Mode armMode, handMode;
	/** Sequence of FT sensors */
	FTSensorSeq ftSensorSeq;
	/** Pointer to the hand active controller */
	grasp::ArmHandForce *armHandForce;
	/** Guards to retrieve a contact */
	grasp::RealSeq fLimit;
	/** Number of averaging steps before stopping collecting readings */
	golem::U32 windowSize;
	golem::I32 steps;
	// gaussian filter mask
	grasp::RealSeq mask;
	// computes gaussian
	template <typename _Real> inline _Real N(const _Real x, const _Real stdev) {
		const _Real norm = golem::numeric_const<_Real>::ONE / (stdev*golem::Math::sqrt(2 * golem::numeric_const<_Real>::PI));
		return norm*golem::Math::exp(-.5*golem::Math::sqr(_Real(x) / _Real(stdev))); // gaussian
    };
	// computes guassian on a vector
	template <typename _Ptr, typename _Real> inline std::vector<_Real> N(_Ptr begin, _Ptr end, const size_t dim, const _Real stdev) {
		std::vector<_Real> output;
		output.assign(dim, golem::numeric_const<_Real>::ZERO);
		size_t idx = 0;
		for (_Ptr i = begin; i != end; ++i) {
			output[idx++] = N(*i, stdev);
		}
		return output;
    };

	/** Input force at sensor, sequence */
	std::vector<grasp::RealSeq> forceInpSensorSeq;
	/** Filtered forces for the hand */
	grasp::RealSeq handFilteredForce;
	/** force reader access cs */
	golem::CriticalSection csHandForce;
	// Return hand DOFs */
	inline size_t dimensions() const { return 18; }// (size_t)handInfo.getJoints().size();

	/** Checks for reions with high likelihood of contacts */
	inline bool expectedCollisions(const golem::Controller::State& state) const {
		return pHeuristic->expectedCollisions(state);
	}


	/** Particles renderer */
	golem::DebugRenderer sampleRenderer;
	/** ground truth renderer */
	golem::DebugRenderer gtRenderer;
	/** Debug renderer */
//	golem::DebugRenderer debugRenderer;

	/** Object pose data */
	Data::Map::iterator poseDataPtr;
	
	/** Description file */
	Desc ragDesc;

	/** Show original colour of the point cloud */
	bool showSampleColour;
	/** Show Sample frame */
	bool showSampleFrame;
	/** Show high dimension distribution */
	bool showDistribution;
	/** Show maximum likelihood estimation distribution */
	bool showMLEFrame;
	/** Show MLE points */
	bool showMLEPoints;
	/** Enables/disables esplicit use of uncertainty in planning */
	bool uncEnable;
	/** Enables/disables replanning */
	bool singleGrasp;
	/** Enables/disables withdrawing to the home pose */
	bool withdrawToHomePose;
	/** Shows the posterior distribution */
	bool posterior;
	/** Avoid to activate the force reader if moving from the pre-grasp to the grasp */
	bool isGrasping;

	/** Select index */
	template <typename _Seq, typename _Index> void selectIndex(const _Seq& seq, _Index& index, const std::string& name) {
		if (seq.empty())
            throw grasp::Cancel("Empty!");
		// select index within the range
		std::stringstream str;
		golem::Math::clamp(index, (_Index)1, (_Index)seq.size());
		str << "Enter " << name << " index <1.." << seq.size() << ">: ";
		readNumber(str.str().c_str(), index);
		if (size_t(index) < 1 || size_t(index) > seq.size())
			throw grasp::Cancel("Invalid index");
    };


	bool printing;
//	golem::Bounds::Seq handBounds, waypointBounds;

	/** Item to remove from data after a trial is saved */
	std::vector<std::string> itemPerformedTrj;

	/** Pointer to collision detection with the ground truth */
	Collision::Ptr collisionPtr;

	/** Resets the controllers */
	bool enableControllers;
	/** Checks collision with the query point cloud */
	bool enableSimContact;
	/** Enables/disables force reading */
	bool enableForceReading;
	bool forcereadersilent;
	bool contactOccured;
	
	/** File to collect data from the ft sensor of the hand */
	std::ofstream dataFTRaw, dataFTFiltered, dataSimContact;
	bool record, contact, brecord;
	/** Query transformation */
	grasp::RBCoord queryPointsTrn;

	/** Combined action waypoints */
	golem::Controller::State::Seq robotStates;
	/** Contains the index of the triggered guards */
//	FTGuard::Seq triggeredGuards;
	/** Structure for FT sensors */
	FTGuard::SeqPtr ftGuards;

	/** Iterations counter */
	size_t iterations;

	/** Checks the validity of a sample */
//	template <typename _PTR> bool isValidSample(const grasp::RBPose::Sample &sample, _PTR begin, _PTR end, golem::Real eps = golem::REAL_EPS) const {
//		std::printf("particle frame<%.4f,%.4f,%.4f> w=%.5f\n", sample.p.x, sample.p.y, sample.p.z, sample.weight);
		//if (sample.weight < theta)
		//	return false;
//		for (_PTR i = begin; i != end; ++i) {
			//if (pRBPose->distance(sample, *i) < eps)
//            if (grasp::RBPose::Sample::distance(sample, *i) < eps)
//				return false;
//		}
//		return true;
//	};

	/** Get current points transformed */
//	grasp::Cloud::PointSeqMap::iterator getTrnPoints(Data::Map::iterator dataPtr, const golem::Mat34 &trn);
	/** Get current points transformed */
//	grasp::Cloud::RawPointSeqMultiMap::iterator getTrnRawPoints(Data::Map::iterator dataPtr, const golem::Mat34 &trn);

	/** Simulates contacts between the robot's hand and object's partial point cloud */
	golem::Real simContacts(const golem::Bounds::Seq::const_iterator &begin, const golem::Bounds::Seq::const_iterator &end, const golem::Mat34 pose);
	/** Object real point cloud (testing purposes) */
	golem::shared_ptr<grasp::Cloud::PointSeq> objectPointCloudPtr;

	/** Updates belief state */
	void updateAndResample(Data::Map::iterator dataPtr);

	/** Finds a target in configuration space in a new reference frame */
	void findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting = false);
	// Finds a grasp frame w.r.t. the world's reference frame
	void findTarget(const golem::Mat34& queryTrn, const golem::Mat34& modelFrame, const golem::Controller::State& target, golem::Controller::State& cend, const bool lifting = false);

	/** (Global search) trajectory of the entire robot from the configuration space and/or workspace target */
	virtual void createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	grasp::RBDist trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);
	/** (Local search) trajectory of the arm only from a sequence of configuration space targets in a new reference frame */
	grasp::RBDist findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory);

	/** Trajectory profile configspace distance multiplier */
	golem::ConfigspaceCoord distance;
	/** Trajectory duration */
	golem::SecTmReal trjDuration;
	/** Trajectory idle time */
	golem::SecTmReal trjIdle;
	/** Trajectory extrapolation */
	golem::ConfigspaceCoord trjExtrapol;
	/** Trajectory extrapolation factor */
	golem::Real trjExtrapolFac;
	/** Performance duration offset */
	golem::SecTmReal trjPerfOff;
	/** Trajectory profile */
	golem::Profile::Ptr pProfile;

	std::string sepField;
	/** golem::Profile::CallbackDist: Configuration space coordinate distance metric */
	virtual golem::Real distConfigspaceCoord(const golem::ConfigspaceCoord& prev, const golem::ConfigspaceCoord& next) const;
	/** golem::Profile::CallbackDist: Coordinate distance metric */
	virtual golem::Real distCoord(golem::Real prev, golem::Real next) const;
	/** golem::Profile::CallbackDist: Coordinate enabled state */
	virtual bool distCoordEnabled(const golem::Configspace::Index& index) const;

	/** Trajectory profile */
	void profile(golem::SecTmReal duration, const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& out, const bool silent = false) const;
	/** Perform trajectory */
	virtual void perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory = true);

	/** Plan and execute r2g, grasp, lift operations */
	bool execute(grasp::data::Data::Map::iterator dataPtr, grasp::Waypoint::Seq& trajectory);

	virtual void render() const;
	void renderHand(const golem::Controller::State &state, const golem::Bounds::Seq &bounds, bool clear = true);

	R2GPlanner(golem::Scene &scene);
	//~R2GPlanner();
	bool create(const Desc& desc);
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPAM_RAGPLANNER_RAGPLANNER_H_*/
