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

#include <Grasp/App/Player/Player.h>
#include <Grasp/Core/Ctrl.h>
#include <Grasp/Contact/Model.h>
#include <Grasp/Contact/Query.h>
#include <Spam/App/PosePlanner/Data.h>

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
		IG_PLAN_LIFT,
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
		case action::IG_PLAN_LIFT:
			str.assign("IG_PLAN_LIFT");
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

		/** Mode */
		enum Mode {
			/** DEFAULT */
			MODE_DEFAULT,
			/** Model data */
			MODE_MODEL,
			/** Query density */
			MODE_QUERY,
		};

		/** Mode name */
		static const std::string ModeName[MODE_QUERY + 1];

		/** Data bundle description */
		class Desc : public grasp::Player::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual grasp::data::Data::Ptr create(golem::Context &context) const;
		};

		/** Data bundle default name */
		std::string dataName;

		/** Current Mode */
		Mode mode;

		/** Model triangles */
		grasp::Vec3Seq modelVertices;
		/** Model triangles */
		grasp::TriangleSeq modelTriangles;

		/** Query frame */
		golem::Mat34 modelFrame;
		/** Model points */
		grasp::Cloud::PointSeq modelPoints;

		/** Query triangles */
		grasp::Vec3Seq queryVertices;
		/** Query triangles */
		grasp::TriangleSeq queryTriangles;

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

		/** Manager */
		virtual void setOwner(grasp::Manager* owner);

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

	protected:
		/** Demo */
		PosePlanner* owner;

		/** Load from xml context */
		virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap);
		/** Save to xml context */
		virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
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
			grasp::ConfigMat34 pose;
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
		/** Trial path only to the last directory */
		std::string dirPath;
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
		grasp::ConfigMat34::Map trajectories;

		/** Specifies if the trial converges to a grasp */
		bool grasped;

		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			dataPath = "./data/boris/jug/spam.xml";

			name = "Default";
			path = "./data/rag_analysis/jug.xml";
			dirPath = "./data/rag_analysis/";
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
		inline size_t insert(const grasp::ConfigMat34::Seq &trj) {
//			trajectories.insert(trajectories.end(), grasp::ConfigMat34::Map::value_type(std::to_string(trajectories.size() + 1), trj));
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
		GRASP_CREATE_FROM_OBJECT_DESC1(PosePlanner, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Data bundle default name */
		std::string dataName;

		/** Trial Data */
		TrialData::Ptr trialData;

		/** Pose distribution */
		Belief::Desc::Ptr pBeliefDesc;


		/** Model pose estimation camera */
		std::string modelCamera;
		/** Model data handler (scan) */
		std::string modelHandlerScan;
		/** Model data handler */
		std::string modelHandler;
		/** Model data item */
		std::string modelItem;
		/** Model data item object */
		std::string modelItemObj;
		/** Model data handler */
		std::string modelGraspHandler;
		/** Model data item */
		std::string modelGraspItem;

		/** Model scan pose */
		grasp::ConfigMat34::Seq modelScanPoseSeq;
		/** Model colour solid */
		golem::RGBA modelColourSolid;
		/** Model colour wireframe */
		golem::RGBA modelColourWire;

		/** Model trajectory handler */
		std::string modelHandlerTrj;
		/** Model trajectory item */
		std::string modelItemTrj;

		/** Query pose estimation camera */
		std::string queryCamera;
		/** Model data handler (scan) */
		std::string queryHandlerScan;
		/** Query data handler */
		std::string queryHandler;
		/** Query data item */
		std::string queryItem;
		/** Query data item object */
		std::string queryItemObj;
		/** Query data handler */
		std::string queryGraspHandler;
		/** Query data item */
		std::string queryGraspItem;

		/** Object scan pose */
		grasp::ConfigMat34::Seq queryScanPoseSeq;
		/** Query colour solid */
		golem::RGBA queryColourSolid;
		/** Query colour wireframe */
		golem::RGBA queryColourWire;

		/** Query trajectory handler */
		std::string queryHandlerTrj;
		/** Query trajectory item */
		std::string queryItemTrj;

		/** Object capture camera */
		std::string objectCamera;
		/** Object data handler (scan) */
		std::string objectHandlerScan;
		/** Object data handler (processed) */
		std::string objectHandler;
		/** Object data item (scan) */
		std::string objectItemScan;
		/** Object data item (processed) */
		std::string objectItem;
		/** Object scan pose */
		grasp::ConfigMat34::Seq objectScanPoseSeq;
		/** Object manual frame adjustment */
		grasp::RBAdjust objectFrameAdjustment;
		/** Pose estimation for simulated object */
		grasp::RBPose::Desc::Ptr simRBPoseDesc;

		/** Belief data handler */
		std::string beliefHandler;
		/** Belief data item */
		std::string beliefItem;

		/** Model descriptions */
		grasp::Model::Desc::Map modelDescMap;
		/** Contact appearance */
		grasp::Contact3D::Appearance contactAppearance;

		/** Query descriptions */
		grasp::Query::Desc::Map queryDescMap;

		/** Grasp force sensor */
		std::string graspSensorForce;
		/** Grasp force threshold */
		golem::Twist graspThresholdForce;
		/** Grasp force event - hand close time wait */
		golem::SecTmReal graspEventTimeWait;
		/** Grasp hand close duration */
		golem::SecTmReal graspCloseDuration;
		/** Grasp pose open (pre-grasp) */
		grasp::ConfigMat34 graspPoseOpen;
		/** Grasp pose closed (grasp) */
		grasp::ConfigMat34 graspPoseClosed;

		/** Model feature appearance */
		grasp::Cloud::Appearance modelAppearance;
		/** Query feature appearance */
		grasp::Cloud::Appearance queryAppearance;
		/** Appereance for point clouds: hypothesis point clouds */
		grasp::Cloud::Appearance hypothesisAppearance;
		/** Appereance for point clouds: debug point clouds */
		grasp::Cloud::Appearance debugAppearance;
		/** Appereance for point clouds: ground truth point clouds */
		grasp::Cloud::Appearance groundTruthAppearance;
		///** Feature frame size */
		//golem::Vec3 featureFrameSize;
		///** Distribution frame size */
		//golem::Vec3 distribFrameSize;

		/** Distribution num of samples */
		size_t distribSamples;

		/** Model point transformation **/
		golem::Mat34 modelTrn;

		/** Enables/disable screen capture from simulation */
		bool screenCapture;

		/** Manipulator description */
		grasp::Manipulator::Desc::Ptr manipulatorDesc;
		/** Manipulator Appearance */
		grasp::Manipulator::Appearance manipulatorAppearance;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Player::Desc::setToDefault();

			dataDesc.reset(new Data::Desc);
			dataName.clear();
			trialData.reset(new TrialData);

			pBeliefDesc.reset(new Belief::Desc);

			modelCamera.clear();
			modelHandlerScan.clear();
			modelHandler.clear();
			modelItem.clear();
			modelItemObj.clear();
			modelGraspHandler.clear();
			modelGraspItem.clear();

			modelScanPoseSeq.clear();
			modelColourSolid.set(golem::RGBA::RED._U8[0], golem::RGBA::RED._U8[1], golem::RGBA::RED._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
			modelColourWire.set(golem::RGBA::RED);

			modelHandlerTrj.clear();
			modelItemTrj.clear();

			queryCamera.clear();
			queryHandlerScan.clear();
			queryHandler.clear();
			queryItem.clear();
			queryItemObj.clear();
			queryGraspHandler.clear();
			queryGraspItem.clear();

			queryScanPoseSeq.clear();
			queryColourSolid.set(golem::RGBA::GREEN._U8[0], golem::RGBA::GREEN._U8[1], golem::RGBA::GREEN._U8[2], golem::numeric_const<golem::U8>::MAX / 8);
			queryColourWire.set(golem::RGBA::GREEN);

			queryHandlerTrj.clear();
			queryItemTrj.clear();

			objectCamera.clear();
			objectHandlerScan.clear();
			objectHandler.clear();
			objectItemScan.clear();
			objectItem.clear();
			objectScanPoseSeq.clear();
			objectFrameAdjustment.setToDefault();
			simRBPoseDesc.reset(new grasp::RBPose::Desc);

			beliefHandler.clear();
			beliefItem.clear();

			modelDescMap.clear();
			contactAppearance.setToDefault();

			queryDescMap.clear();

			graspSensorForce.clear();
			graspThresholdForce.setZero();
			graspEventTimeWait = golem::SecTmReal(2.0);
			graspCloseDuration = golem::SecTmReal(2.0);
			graspPoseOpen.setToDefault();
			graspPoseClosed.setToDefault();

			modelAppearance.setToDefault();
			queryAppearance.setToDefault();
			hypothesisAppearance.setToDefault();
			debugAppearance.setToDefault();
			groundTruthAppearance.setToDefault();
			//featureFrameSize.set(golem::Real(0.1));
			//distribFrameSize.set(golem::Real(0.01));
			distribSamples = 100;

			//actionManip.configspace = false;
			//actionManip.position = actionManip.orientation = true;
			//actionManip.w.setId();
			//actionManip.w.p.z += -golem::Real(0.15);

			//numPoses = 100;
			//numHypotheses = 5;

			modelTrn.setToDefault();

			screenCapture = false;

			manipulatorDesc.reset(new grasp::Manipulator::Desc);
			manipulatorAppearance.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual void assertValid(const grasp::Assert::Context& ac) const {
			Player::Desc::assertValid(ac);
			
			grasp::Assert::valid(dataDesc != nullptr && grasp::is<Data::Desc>(dataDesc.get()), ac, "dataDesc: unknown type");

			grasp::Assert::valid(dataName.length() > 0, ac, "dataName: invalid");
			pBeliefDesc->assertValid(grasp::Assert::Context(ac, "Belief desc: invalid"));

			grasp::Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(grasp::Assert::Context(ac, "manipulatorDesc->"));
			manipulatorAppearance.assertValid(grasp::Assert::Context(ac, "manipulatorAppearance."));

			//grasp::Assert::valid(graspSensorForce.length() > 0, ac, "graspSensorForce: invalid");
			//grasp::Assert::valid(graspThresholdForce.isPositive(), ac, "graspThresholdForce: negative");
			//grasp::Assert::valid(graspEventTimeWait > golem::SEC_TM_REAL_ZERO, ac, "graspEventTimeWait: <= 0");
			//grasp::Assert::valid(graspCloseDuration > golem::SEC_TM_REAL_ZERO, ac, "graspCloseDuration: <= 0");
			//graspPoseOpen.assertValid(grasp::Assert::Context(ac, "graspPoseOpen."));
			//graspPoseClosed.assertValid(grasp::Assert::Context(ac, "graspPoseClosed."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
	};

	/** Process algortihm */
//	virtual void processPoints(Data::Map::const_iterator dataPtr, const Selection& selection, const bool silent = false);

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
//		pRBPose = myDesc.pRBPoseDesc->create(context); // throws
		pBelief = myDesc.pBeliefDesc->create(context);  //static_cast<Belief*>(pRBPose.get());
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
//	grasp::RBPose::Ptr pRBPose;
	Belief::Ptr pBelief; //grasp::RBPose::Ptr pRBPose;

	/** Model pose estimation camera */
	grasp::Camera* modelCamera;
	/** Model data handler (scan) */
	grasp::data::Handler* modelHandlerScan;
	/** Model data handler */
	grasp::data::Handler* modelHandler;
	/** Model data item */
	std::string modelItem;
	/** Model data item object */
	std::string modelItemObj;
	/** Model data handler */
	grasp::data::Handler* modelGraspHandler;
	/** Model data item */
	std::string modelGraspItem;

	/** Model scan pose */
	grasp::ConfigMat34::Seq modelScanPoseSeq;
	/** Model colour solid */
	golem::RGBA modelColourSolid;
	/** Model colour wireframe */
	golem::RGBA modelColourWire;

	/** Model trajectory handler */
	grasp::data::Handler* modelHandlerTrj;
	/** Model trajectory item */
	std::string modelItemTrj;

	/** Query pose estimation camera */
	grasp::Camera* queryCamera;
	/** Query data handler (scan) */
	grasp::data::Handler* queryHandlerScan;
	/** Query data handler */
	grasp::data::Handler* queryHandler;
	/** Query data item */
	std::string queryItem;
	/** Query data item object */
	std::string queryItemObj;
	/** Query data handler */
	grasp::data::Handler* queryGraspHandler;
	/** Query data item */
	std::string queryGraspItem;

	/** Query scan pose */
	grasp::ConfigMat34::Seq queryScanPoseSeq;
	/** Query colour solid */
	golem::RGBA queryColourSolid;
	/** Query colour wireframe */
	golem::RGBA queryColourWire;

	/** Query trajectory handler */
	grasp::data::Handler* queryHandlerTrj;
	/** Query trajectory item */
	std::string queryItemTrj;

	/** Grasp force sensor */
	grasp::FT* graspSensorForce;
	/** Grasp force threshold */
	golem::Twist graspThresholdForce;
	/** Grasp force event - hand close time wait */
	golem::SecTmReal graspEventTimeWait;
	/** Grasp hand close duration */
	golem::SecTmReal graspCloseDuration;
	/** Grasp pose open (pre-grasp) */
	grasp::ConfigMat34 graspPoseOpen;
	/** Grasp pose closed (grasp) */
	grasp::ConfigMat34 graspPoseClosed;

	/** Object capture camera */
	grasp::Camera* objectCamera;
	/** Object data handler (scan) */
	grasp::data::Handler* objectHandlerScan;
	/** Object data handler (processed) */
	grasp::data::Handler* objectHandler;
	/** Object data item (scan) */
	std::string objectItemScan;
	/** Object data item (processed) */
	std::string objectItem;
	/** Object scan pose */
	grasp::ConfigMat34::Seq objectScanPoseSeq;
	/** Object manual frame adjustment */
	grasp::RBAdjust objectFrameAdjustment;
	/** Accurate pose estimation for the simulated object */
	grasp::RBPose::Ptr simRBPose;
	/** Reference frames */
	golem::Mat34 simModelFrame, simQueryFrame;

	/** Belief data handler */
	grasp::data::Handler* beliefHandler;
	/** Belief data item */
	std::string beliefItem;

	/** Models */
	grasp::Model::Map modelMap;
	/** Contact appearance */
	grasp::Contact3D::Appearance contactAppearance;

	/** Query densities */
	grasp::Query::Map queryMap;

	/** Manipulator */
	grasp::Manipulator::Ptr manipulator;
	/** Manipulator Appearance */
	grasp::Manipulator::Appearance manipulatorAppearance;

	/** Distribution num of samples */
	size_t distribSamples;

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

	/** Appereance for point clouds: model point cloud */
	grasp::Cloud::Appearance modelAppearance;
	/** Show model point cloud */
	bool showModelPointCloud;
	/** Show model features */
	bool showModelFeatures;
	/** Displayed feature index */
	golem::U32 featureIndex;
	/** Model renderer */
	golem::DebugRenderer modelRenderer;

	/** Appereance for point clouds: query point cloud */
	grasp::Cloud::Appearance queryAppearance;
	/** Show the query point cloud */
	bool showQueryPointCloud;
	/** Query renderer */
	golem::DebugRenderer queryRenderer;

	/** Appereance for point clouds: hypothesis point clouds */
	grasp::Cloud::Appearance hypothesisAppearance;
	/** Show the query point cloud */
	bool showHypothesesPointClouds;
	/** Show only the mean hypothesis */
	bool showMeanHypothesisPointClouds;
	/** Show only the mean hypothesis */
	bool showHypothesisBounds;
	/** Hypothesis renderer */
	golem::DebugRenderer hypothesisRenderer;

	/** Appereance for point clouds: debug point clouds */
	grasp::Cloud::Appearance debugAppearance;
	/** Show query distribution (as frames) */
	bool showQueryDistribFrames;
	/** Show pose distribution **/
	bool showQueryDistribPointClouds;
	/** Query renderer */
	golem::DebugRenderer beliefRenderer;

	golem::Bounds::Seq handBounds;
	golem::DebugRenderer debugRenderer;


	/** Appereance for point clouds: ground truth point clouds */
	grasp::Cloud::Appearance groundTruthAppearance;
	/** Show simulated pose of the object */
	bool showGroundTruth;
	/** Query renderer */
	golem::DebugRenderer groundTruthRenderer;

	/** Rendering model frame size */
	golem::Vec3 modelFrameSize;
	/** Rendering query frame size */
	golem::Vec3 queryFrameSize;
	/** Rendering distribution frame size */
	golem::Vec3 hypothesisFrameSize, distrFrameSize;
	/** Appereance for point clouds */
	grasp::Cloud::Appearance sampleAppearance;

	/** Enables/disable screen capture from simulation */
	bool screenCapture;

	/** Smart pointer to the ft driven heuristic */
	FTDrivenHeuristic* pHeuristic;

	/** Pose estimation */
	grasp::data::Item::Map::iterator estimatePose(const Data::Mode mode, std::string &itemName);
	/** Grasp and capture object */
	grasp::data::Item::Map::iterator objectCapture(const Data::Mode mode, std::string &itemName);
	/** Process object image and add to data bundle */
	grasp::data::Item::Map::iterator objectProcess(const Data::Mode mode, grasp::data::Item::Map::iterator ptr);

	/** Create trajectory */
	virtual void createWithManipulation(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent = false) const;
	/** Create trajectory */
	virtual void create(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent = false) const;

	/** Reset data pointers */
	void resetDataPointers();

	/** Point selection */
//	grasp::Cloud::PointSeqMap::iterator getPointsTrn(Data::Map::iterator dataPtr, golem::Mat34 &trn);

	inline golem::Controller::State lookupState(golem::SecTmReal time = golem::SEC_TM_REAL_MAX) const {
		golem::Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		controller->lookupState(time, dflt);
		dflt.cvel.setToDefault(info.getJoints());
		dflt.cacc.setToDefault(info.getJoints());
		return dflt;
	}

	inline golem::Controller::State lookupCommand(golem::SecTmReal time = golem::SEC_TM_REAL_MAX) const {
		golem::Controller::State dflt = controller->createState();
		controller->setToDefault(dflt);

		controller->lookupCommand(time, dflt);
		dflt.cvel.setToDefault(info.getJoints());
		dflt.cacc.setToDefault(info.getJoints());
		return dflt;
	}

	virtual void render() const;

	bool create(const Desc& desc);
	PosePlanner(golem::Scene &scene);
	//~PosePlanner();
};

/** Reads/writes object from/to a given XML context */
//void XMLData(PosePlanner::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

///** PosePlanner application */
//class PosePlannerApp : public golem::Application {
//protected:
//	/** Runs Application */
//	virtual void run(int argc, char *argv[]);
//};

//------------------------------------------------------------------------------

};	// namespace

namespace golem {

template <> void Stream::read(spam::PosePlanner::Data &data) const;
template <> void Stream::write(const spam::PosePlanner::Data &data);

};	// namespace


#endif /*_GRASP_POSEPLANNER_POSEPLANNER_H_*/
