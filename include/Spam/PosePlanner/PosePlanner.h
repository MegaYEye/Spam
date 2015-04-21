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
#ifndef _SPAM_POSEPLANNER_POSEPLANNER_H_
#define _SPAM_POSEPLANNER_POSEPLANNER_H_

//------------------------------------------------------------------------------

#include <Grasp/Player/Player.h>
#include <Spam/Spam/Belief.h>
#include <Golem/Phys/Application.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** SPAM Pose planner. */
class PosePlanner : public grasp::Player {
public:
	/** Action types */
	enum  action {
		NONE_ACTION = 0,
		GRASP,
		IG_PLAN_ON_QUERY,
		IG_PLAN_M2Q,
		IG_TRAJ_OPT
	};
	/** Prints actions */
	static std::string actionToString(action type) {
		std::string str;
		switch (type) {
		case action::NONE_ACTION:
			str.assign("NONE");
			break;
		case action::GRASP:
			str.assign("GRASP");
			break;
		case action::IG_PLAN_M2Q:
			str.assign("IG_PLAN_M2Q");
			break;
		case action::IG_PLAN_ON_QUERY:
			str.assign("IG_PLAN_ON_QUERY");
			break;
		case action::IG_TRAJ_OPT:
			str.assign("IG_TRAJ_OPT");
			break;
		}
		return str;
	}

	/** Type of implemented algorithms */
	enum Strategy {
		NONE_STRATEGY = 0,
		ELEMENTARY,
		MYCROFT,
		IR3NE,
	};

	/** Data */
	class Data : public grasp::Player::Data {
	public:
		friend class PosePlanner;

		/** Action frame */
//		golem::Mat34 actionFrame;
		/** Query frame */
		golem::Mat34 modelFrame;
		/** Model points */
		grasp::Cloud::PointSeq modelPoints;
		/** Query transformation */
		golem::Mat34 queryTransform;
		/** Query frame: queryTransform * modelFrame */
		golem::Mat34 queryFrame;
		/** Query points */
		grasp::Cloud::PointSeq queryPoints;
		/** Simulated location of the object */
		grasp::Cloud::PointSeq simulateObjectPose;

		/** High dim rep pose distribution **/
		grasp::RBPose::Sample::Seq poses;
		/** Low dim rep pose distribution **/
		grasp::RBPose::Sample::Seq hypotheses;

		/** Belief file name extension */
		std::string extSamples;

		/** type of action to execute */
		action actionType;

		/** Strategy to execute */
		Strategy stratType;
		
		/** Reset data during construction */
		Data() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			grasp::Player::Data::setToDefault();
			
//			actionFrame.setId();
			queryTransform.setId();
			queryFrame.setId();
			queryPoints.clear();

			simulateObjectPose.clear();

			modelFrame.setId();
			modelPoints.clear();

			poses.clear();
			hypotheses.clear();

			extSamples = ".rbs";

			actionType = action::NONE_ACTION;
			stratType = Strategy::NONE_STRATEGY;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			Player::Data::assertValid(ac);
		}

		/** Reads/writes object from/to a given XML context */
		virtual void xmlData(golem::XMLContext* context, bool create = false) const;
	};

	class Trajectory {
	public:
		class Waypoint {
		public:
			typedef golem::shared_ptr<Waypoint> Ptr;
			typedef std::vector<Ptr> SeqPtr;

			/** Controller position in configuration space coordinates. */
			golem::ConfigspaceCoord cpos;

			/** Tool positions and orientations. */
			golem::WorkspaceChainCoord wpos;
			/** All joints positions and orientations. */
			golem::WorkspaceJointCoord wposex;
			/** Positions and orientations of chain origins. */
			golem::WorkspaceChainCoord wposchainex;
			/** Tool orientations as quaternions. */
			golem::Chainspace::Coord<golem::Quat> qrot;

			/** Default constructor initialises only GraphSearch::Node */
			Waypoint(){}

			/** Constructs Waypoint from Config */
			Waypoint(const golem::Controller &controller, const golem::ConfigspaceCoord &cpos, bool tr = true, bool ex = true) {
				setup(controller, cpos, tr, ex);
			}

			void setup(const golem::Controller &controller, const golem::ConfigspaceCoord &cpos, bool tr, bool ex) {
				this->cpos = cpos;
				setup(controller, tr, ex);
			}
			void setup(const golem::Controller &controller, bool tr, bool ex) {
				const golem::Controller::State::Info& stateInfo = controller.getStateInfo();

				if (ex) {
					controller.jointForwardTransform(cpos, wposex);
					for (golem::Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
						const golem::Chain* chain = controller.getChains()[i];
						const golem::Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();

						wpos[i] = wposex[stateInfo.getJoints(i).end() - 1]; // the last joint in the chain i
						wposchainex[i].multiply(linkedChainIndex < i ? wpos[linkedChainIndex] : controller.getGlobalPose(), chain->getLocalPose());
					}
				}
				else {
					controller.chainForwardTransform(cpos, wpos);
				}

				if (tr) {
					for (golem::Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
						wpos[i].multiply(wpos[i], controller.getChains()[i]->getReferencePose()); // reference pose
						qrot[i].fromMat33(wpos[i].R);
					}
				}
			}

			/** To string */
			std::string toString(const golem::Controller &controller) {
				const golem::Controller::State::Info& stateInfo = controller.getStateInfo();
				std::stringstream ss;
				//for (auto i = stateInfo.getChains().begin(); i != stateInfo.getChains().end(); ++i) {
				//	const golem::U32 chainId = i - stateInfo.getChains().begin();
				//	ss << "chain =" << chainId << " <";
				//	for (auto j = stateInfo.getJoints(i).begin(); j != stateInfo.getJoints(i).end(); ++j)
				//		ss << cpos[j] << (j < stateInfo.getJoints(i).end() - 1 ? " " : "> ");
				//}
				ss << "waypoints...\n";
				return ss.str();
			}
		};

		typedef golem::shared_ptr<Trajectory> Ptr;
		typedef std::map<std::string, Ptr> Map;

		Trajectory(const action &type, const golem::Controller &controller, const golem::Controller::State::Seq trajectory) {
			this->type = type;
			waypoints.clear();
			for (auto i = trajectory.begin(); i != trajectory.end(); ++i)
				waypoints.push_back(Waypoint::Ptr(new Waypoint(controller, i->cpos)));
		}

		/** To string */
		std::string toString(const golem::Controller &controller) {
			std::stringstream ss;
			ss << "Trajectory type=" << actionToString(type).c_str() << " size=" << waypoints.size() << "\n";
			for (auto i = waypoints.begin(); i != waypoints.end(); ++i)
				ss << (*i)->toString(controller);
			return ss.str();
		}

		action type;
		Waypoint::SeqPtr waypoints;
	};
	class State {
	public:
		typedef golem::shared_ptr<State> Ptr;
		typedef std::vector<Ptr> SeqPtr;

		grasp::RBPose::Sample::Seq poses;
		grasp::RBPose::Sample::Seq hypotheses;

		State(const Belief &belief) {
			poses.clear();
			hypotheses.clear();
			poses = belief.getSamples();
			hypotheses = belief.getHypothesesToSample();
		}

		std::string toString() {
			std::stringstream ss;
			grasp::RBPose::Sample c = *hypotheses.begin();
			ss << "Estimated pose <" << c.p.x << ", " << c.p.y << ", " << c.p.z << "> [" << c.q.w << ", " << c.q.x << ", " << c.q.y << ", " << c.q.z << "]";
			return ss.str();
		}
	};
	class TrialData {
	public:
		friend class PosePlanner;
		/** Trial pointer */
		typedef golem::shared_ptr<TrialData> Ptr;
		/** Trial container: name -> trial */
		typedef std::map<std::string, Ptr> Map;
		/** RobotState collection */
		//typedef std::map<golem::U32, grasp::RobotState::Seq> RobotStateMap;
		
		class Iteration {
		public:
			typedef golem::shared_ptr<Iteration> Ptr;
			typedef std::map<golem::U32, Ptr> Map;

			/** Trial path */
			std::string path;
			/** Name separator */
			std::string sepName;
			/** Data file name extension */
			std::string extIter;

			/** Belief state */
			State::Ptr state;
			/** Executed trajectory */
			//Trajectory::Ptr trajectory;
			golem::Controller::State::Seq trajectory;
			/** Action type to execute */
			action actionType;
			/** Robot pose at the moment of the contact */
			grasp::RobotPose pose;
			/** chain and joint in contact. If no contact -1 */
			golem::U32 chainIdx, jointIdx;

			Iteration(const std::string &path) {
				this->path = path;
				setToDefault();
			}
			Iteration(const std::string &path, const Belief *belief) {
				this->path = path;
				setToDefault();
				state.reset(new State(*belief));
			}

			void setToDefault() {
				this->state.reset();
				this->trajectory.clear();//.reset();
				actionType = action::NONE_ACTION;
				sepName = "-";
				extIter = ".rbs";
			}

			//void setTrajectory(const action &type, const golem::Controller &controller, const golem::Controller::State::Seq &trajectory) {
			//	this->trajectory.reset(new Trajectory(type, controller, trajectory));
			//}
			void setTrajectory(const golem::Controller::State::Seq &trajectory) {
				this->trajectory = trajectory;
			}

			/** Name */
			std::string getName() const {
				return path;
			}

			std::string toString(const golem::Controller &controller) {
				std::stringstream ss;
				ss << state->toString() << "\n";
				//ss << trajectory->toString(controller) << "\n";
				return ss.str();
			}

			/** Reads/writes object from/to a given XML context */
			virtual void xmlData(golem::XMLContext* context, bool create = false) const;
		};

		/** Data ptr */
		std::string dataPath;

		/** Trial name */
		std::string name;
		/** Trial path */
		std::string path;
		/** Object name */
		std::string object;
		/** Trial extension */
		std::string extTrial;
		/** Name separator */
		std::string sepName;

		/** Meta parameter to generate a pseudo-random rigid body trasformation on partial point cloud (query transformation) */
		grasp::RBDist queryStdDev;
		/** Meta parameter to generate a pseudo-random rigid body trasformation on estimated pose (ground truth in simulation) */
		grasp::RBDist objPoseStdDev;
		/** Object transformation (for test porpuses) */
		grasp::RBCoord objectPoseTrn;
		/** Query transformation */
		grasp::RBCoord queryPointsTrn;

		/** Enables/disables writing on file */
		bool silent;
		/** Enables/disables pseudo-random transformations */
		bool enable;

		/** Iteration counter */
//		golem::U32 itIndex;
		/** Collection of iterations: num of iteration -> iteration */
//		Iteration::Map iterations;

		/** Trajectory file name extension */
		std::string extTrajectory;
		/** Collection of executed trajectories */
		grasp::RobotState::Map trajectories;

		/** Specifies if the trial converges to a grasp */
		bool grasped;

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			dataPath = "./data/boris/jug/spam.xml";

			name = "Default";
			path = "./data/rag_analysis/jug.xml";
			extTrial = ".xml";
			sepName = "-";

			object = "";
			silent = false;
			enable = false;

			//itIndex = 0;
			//iterations.clear();
			extTrajectory = ".trj";
			trajectories.clear();
			grasped = false;

			queryStdDev.set(golem::Real(0.02), golem::Real(1000.0));
			objPoseStdDev.set(golem::Real(0.002), golem::Real(1000.0));

			objectPoseTrn.fromMat34(golem::Mat34::identity());
			queryPointsTrn.fromMat34(golem::Mat34::identity());
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			grasp::Assert::valid(name.length() > 0, ac, "name: length 0");
			grasp::Assert::valid(path.length() > 0, ac, "path: length 0");
			grasp::Assert::valid(extTrial.length() > 0, ac, "extTrial: length 0");
			grasp::Assert::valid(queryStdDev.isValid(), ac, "queryStdDev: not valid");
			grasp::Assert::valid(objPoseStdDev.isValid(), ac, "objPoseStdDev: not valid");
			grasp::Assert::valid(objectPoseTrn.toMat34().isFinite(), ac, "objectPoseTrn: not valid");
			grasp::Assert::valid(queryPointsTrn.toMat34().isFinite(), ac, "queryPointsTrn: not valid");
		}

		/** Constructor */
		TrialData() {
			TrialData::setToDefault();
		}

		/** Destructor */
		virtual ~TrialData() {}

		/** set up the trial */
		virtual void setup(const golem::Context &context,  golem::Rand &rand) {
			using namespace golem;
			Mat33 rot = Mat33::identity();
			rot.rotZ(rand.nextUniform<Real>()*queryStdDev.ang);
			Vec3 v(REAL_ONE, REAL_ONE, REAL_ONE);
			Vec3 vquery = v*-queryStdDev.lin;
			queryPointsTrn.set(vquery + Vec3(rand.nextUniform<Real>(), rand.nextUniform<Real>(), rand.nextUniform<Real>())*queryStdDev.lin * 2, rot);
			queryPointsTrn.p.z *= 0.01;
			if (!silent) context.write("trialData->queryPointsTrn <%f %f %f> [%f %f %f %f] (lin=%f, ang=%f)\n", queryPointsTrn.p.x, queryPointsTrn.p.y, queryPointsTrn.p.z,
				queryPointsTrn.q.w, queryPointsTrn.q.x, queryPointsTrn.q.y, queryPointsTrn.q.z, queryStdDev.lin, queryStdDev.ang);
			rot = Mat33::identity();
			rot.rotZ(rand.nextUniform<Real>()*objPoseStdDev.ang);
			Vec3 vobj = v*-objPoseStdDev.lin;
			objectPoseTrn.set(vobj + Vec3(rand.nextUniform<Real>(), rand.nextUniform<Real>(), rand.nextUniform<Real>())*objPoseStdDev.lin * 2, rot);
			if (!silent) context.write("trialData->objectPoseTrn <%f %f %f> [%f %f %f %f] (lin=%f, ang=%f)\n", objectPoseTrn.p.x, objectPoseTrn.p.y, objectPoseTrn.p.z,
				objectPoseTrn.q.w, objectPoseTrn.q.x, objectPoseTrn.q.y, objectPoseTrn.q.z, objPoseStdDev.lin, objPoseStdDev.ang);
		}

		/** Name */
		std::string getName() const {
			std::string name = path;
			const size_t pos = name.rfind(extTrial);
			if (pos != std::string::npos) name.erase(pos);
			return name;
		}

		/** Load from disk */
		virtual void load();
		/** Save to disk */
		virtual void save() const;

		/** Adds new entry */
		//void add(const Iteration::Ptr &iteration) {
		//	(void)iterations.insert(iterations.end(), Iteration::Map::value_type(++itIndex, iteration));
		//}

		/** Insert new executed trajectory */
		inline size_t insert(const grasp::RobotState::Seq &trj) {
			trajectories.insert(trajectories.end(), grasp::RobotState::Map::value_type(std::to_string(trajectories.size() + 1), trj));
			// return number of the iterations
			return trajectories.size();
		}

		std::string successed() {
			return grasped ? "YES" : "NO";
		}

		/** To string */
		std::string toString(const golem::Controller &controller) {
			std::stringstream ss;
			ss << "TrialData\nname=" << name << " path=" << path << " success=" << successed() << "\niterations=" << trajectories.size() << "\n";
			//golem::U32 countIt = 1;
			//for (auto i = iterations.begin(); i != iterations.end(); ++i) 
			//	ss << "It=" << countIt++ << " " << (*i).second->toString(controller).c_str() << "\n";
			return ss.str();
		}

		/** Reads/writes object from/to a given XML context */
		virtual void xmlData(golem::XMLContext* context, bool create = false) const; // throws

		protected:
			golem::Controller *controller;
	};

	/** Pose planner description */
	class Desc : public grasp::Player::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(PosePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Trial Data */
		TrialData::Ptr trialData;

		/** Pose distribution */
		grasp::RBPose::Desc::Ptr pRBPoseDesc;
		/** Model feature appearance */
		grasp::RBPose::Feature::Appearance modelAppearance;
		/** Query feature appearance */
		grasp::RBPose::Feature::Appearance queryAppearance;
		/** Feature frame size */
		golem::Vec3 featureFrameSize;
		/** Distribution frame size */
		golem::Vec3 distribFrameSize;
		/** Distribution num of samples */
		size_t distribSamples;

		///** Distribution num of poses **/
		//size_t numPoses;
		///** Distibution num of hypotheses **/
		//size_t numHypotheses;

		/** Default manipulation action */
		grasp::RobotPose actionManip;

		/** Model point transformation **/
		golem::Mat34 modelTrn;

		/** Object transformation (for test porpuses) */
//		golem::Mat34 objectTrn;

		/** Enables/disable screen capture from simulation */
		bool screenCapture;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Player::Desc::setToDefault();

			data.reset(new Data);
			trialData.reset(new TrialData);

			pRBPoseDesc.reset(new grasp::RBPose::Desc);
			modelAppearance.setToDefault();
			queryAppearance.setToDefault();
			featureFrameSize.set(golem::Real(0.1));
			distribFrameSize.set(golem::Real(0.01));
			distribSamples = 100;

			actionManip.configspace = false;
			actionManip.position = actionManip.orientation = true;
			actionManip.w.setId();
			actionManip.w.p.z += -golem::Real(0.15);

			//numPoses = 100;
			//numHypotheses = 5;

			modelTrn.setToDefault();

			screenCapture = false;
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Player::Desc::isValid())
				return false;
			
			if (pRBPoseDesc == NULL || !pRBPoseDesc->isValid())
				return false;
			if (!modelAppearance.isValid() || !queryAppearance.isValid() || !featureFrameSize.isPositive() || !distribFrameSize.isPositive())
				return false;
			actionManip.assertValid(grasp::Assert::Context("grasp::PosePlanner::Desc::isValid(): actionManip."));

			return true;
		}
	};

	/** Process algortihm */
	virtual void processPoints(Data::Map::const_iterator dataPtr, const Selection& selection, const bool silent = false);

	/** data collection */
	TrialData::Map& getTrialData() {
		return trialDataMap;
	}
	/** data collection */
	const TrialData::Map& getTrialData() const {
		return trialDataMap;
	}

	/** Reset belief state */
	inline void createBeliefState() {
		pRBPose = myDesc.pRBPoseDesc->create(context); // throws
		pBelief = static_cast<Belief*>(pRBPose.get());
	}

protected:
	/** Generator of pseudo random numbers */
	golem::Rand rand;

	/** Descriptor file */
	Desc myDesc;

	/** Collection of trials */
	TrialData::Map trialDataMap;
	/** Trail data pointer */
	TrialData::Ptr trial;
	/** Trial data map iterator */
	TrialData::Map::iterator trialPtr;
	/** Creates a new trial data */
	TrialData::Ptr createTrialData();

	/** Pose distribution */
	grasp::RBPose::Ptr pRBPose;
	Belief* pBelief; //grasp::RBPose::Ptr pRBPose;

	/** Model feature appearance */
	grasp::RBPose::Feature::Appearance modelAppearance;
	/** Query feature appearance */
	grasp::RBPose::Feature::Appearance queryAppearance;
	/** Feature frame size */
	golem::Vec3 featureFrameSize;
	/** Distribution frame size */
	golem::Vec3 distribFrameSize;
	/** Distribution num of samples */
	size_t distribSamples;

	/** Default manipulation action */
	grasp::RobotPose actionManip;

	/** Feature renderer */
	golem::DebugRenderer pointFeatureRenderer;
	/** Particles renderer */
	golem::DebugRenderer sampleRenderer;
	golem::DebugRenderer testRenderer;


	/** Model data */
	Data::Map::iterator modelDataPtr;
	/** Model frame */
	golem::Mat34 modelFrame;
	/** Model points */
	grasp::Cloud::PointSeq modelPoints;
	/** Model transformation frame **/
	golem::Mat34 modelTrn;
	/** Object transformation (for test porpuses) */
	golem::Mat34 objectTrn;

	/** Query data */
	Data::Map::iterator queryDataPtr;

	/** 3d surface samples' point feature appearance */
	grasp::Director::Data::Appearance sampleAppearance;
	/** Show model points */
	bool showModelPoints;
	/** Show query points */
	bool showQueryPoints;
	/** Show model features */
	bool showModelFeatures;
	/** Show query distribution */
	bool showQueryDistrib;
	/** Show pose distribution **/
	bool showPoseDistrib;
	/** Show mean hypothesis */
	bool showMeanHypothesis;
	/** Show hypothesis distribution **/
	bool showSamplePoints;
	/** Show query distribution */
	bool showDistrPoints;
	/** Show simulated pose of the object */
	bool showObject;
	/** Displayed feature index */
	golem::U32 featureIndex;
	/** Show processed points */
	bool showPoints;

	///** Distribution num of poses **/
	//size_t numPoses;
	///** Distibution num of hypotheses **/
	//size_t numHypotheses;

	/** Enables/disable screen capture from simulation */
	bool screenCapture;

	///** Profile state sequence */
	//virtual void profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const;
	///** Profile state sequence */
	//virtual void profileManip(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const;

	/** Creates new data */
	virtual Data::Ptr createData() const;

	/** Create trajectory */
	virtual void createWithManipulation(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent = false) const;
	/** Create trajectory */
	virtual void create(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent = false) const;

	/** Render data */
	virtual void renderData(Data::Map::const_iterator dataPtr);
	/** Render belief state */
//	void renderUncertainty(const grasp::RBPose::Sample::Seq &samples);
//	golem::DebugRenderer uncRenderer;
	/** Reset data pointers */
	void resetDataPointers();

	/** Point selection */
	grasp::Cloud::PointSeqMap::iterator getPointsTrn(Data::Map::iterator dataPtr, golem::Mat34 &trn);

	/** User interface: menu function */
	virtual void function(Data::Map::iterator& dataPtr, int key);

	virtual void render();

	PosePlanner(golem::Scene &scene);
	bool create(const Desc& desc);
};

/** Reads/writes object from/to a given XML context */
void XMLData(PosePlanner::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** PosePlanner application */
class PosePlannerApp : public golem::Application {
protected:
	/** Runs Application */
	virtual void run(int argc, char *argv[]);
};

//------------------------------------------------------------------------------

};	// namespace

namespace golem {

template <> void Stream::read(spam::PosePlanner::Data &data) const;
template <> void Stream::write(const spam::PosePlanner::Data &data);

};	// namespace


#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
