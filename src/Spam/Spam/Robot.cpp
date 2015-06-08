/** @file Robot.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Spam/Spam/Robot.h>
#include <Golem/Device/MultiCtrl/MultiCtrl.h>
//#include <Golem/Tools/XMLData.h>
//#include <Golem/Plan/Data.h>
//#include <Golem/Tools/XMLData.h>
//#include <Golem/Tools/Data.h>
#include <Golem/PhysCtrl/Data.h>
#include <Golem/Device/RobotJustin/RobotJustin.h>
#include <numeric>

#ifdef WIN32
	#pragma warning (push)
	#pragma warning (disable : 4244)
	#pragma warning (disable : 4996)
#endif
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#ifdef WIN32
	#pragma warning (pop)
#endif

#ifdef _GRASP_HITHANDII_DEBUG
#include <Golem/Device/DLR/DLRHitHandII.h>
#endif // _GRASP_HITHANDII_DEBUG

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

std::string spam::controllerDebug(grasp::ActiveCtrl &ctrl) {
	std::stringstream str;

	//const Heuristic& heuristic = planner.getHeuristic();
	//FTDrivenHeuristic *pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner.getHeuristic());
	//const Controller& controller = planner.getController();
	//const Heuristic::JointDesc::JointSeq& jointDesc = heuristic.getJointDesc();
	//const Chainspace::Range chains = controller.getStateInfo().getChains();
	//golem::U32 enabled = 0;
	//for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
	//	const Configspace::Range joints = controller.getStateInfo().getJoints(i);
	//	for (Configspace::Index j = joints.begin(); j < joints.end(); ++j)
	//		if (jointDesc[j]->enabled) ++enabled;
	//}

	//str << controller.getName() << ": chains=" << controller.getStateInfo().getChains().size() << ", joints=" << controller.getStateInfo().getJoints().size() << "(enabled=" << enabled << "), collisions=" << (heuristic.getDesc().collisionDesc.enabled ? "ENABLED" : "DISABLED") << ", non-Euclidian metrics=" << (pHeuristic && pHeuristic->enableUnc ? "ENABLE" : "DISABLE") << ", point cloud collisions=" << (pHeuristic && pHeuristic->getPointCloudCollision() ? "ENABLE" : "DISABLE");

	return str.str();
}

//------------------------------------------------------------------------------

Robot::Robot(Scene &scene) : grasp::Robot(scene) {
}
	
bool Robot::create(const Desc& desc) {
	grasp::Robot::create(desc); // throws

	this->desc = desc;
	// overwrite the heuristic
	pFTDrivenHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner->getHeuristic());

	steps = 0;
	windowSize = 40;
	// compute guassian pdf over x=[-2:.1:2]
	 // vector index
	grasp::RealSeq v;
	v.assign(windowSize + 1, REAL_ZERO);
	const Real delta(.1);
	Real i = -2;
	for (I32 j = 0; j < windowSize + 1; j++) {
		v[j] = N(i, REAL_ONE);
		i += delta;
	}

	// compute the derivative mask=diff(v)
	mask.assign(windowSize, REAL_ZERO);
	for (I32 i = 0; i < windowSize; ++i)
		mask[i] = v[i + 1] - v[i];

	forceInpSensorSeq.resize(dimensions());
	for (std::vector<grasp::RealSeq>::iterator i = forceInpSensorSeq.begin(); i != forceInpSensorSeq.end(); ++i)
		i->assign(windowSize, REAL_ZERO);
	handFilteredForce.assign(dimensions(), REAL_ZERO);

	triggeredGuards.clear();
	triggeredGuards.reserve(fLimit.size());

	//objectPointCloudPtr.reset(new grasp::Cloud::PointSeq());
	//manipulator.reset();
	//w.setToDefault();
	//w.points = 5000;// desc.maxModelPoints;
	//collision.reset();

	//desc.handCtrlDesc->forceReader = [=](const golem::Controller::State& state, grasp::RealSeq& force) {
	//	// no force by default
	//	force.assign(state.getInfo().getJoints().size(), REAL_ZERO);
	//	// use simulated force
	//	if (simHandForceMode != FORCE_MODE_DISABLED) {
	//		const Vec3 simForce(simModeVec.z*simForceGain.v.z, simModeVec.x*simForceGain.v.x, simModeVec.y*simForceGain.v.y);

	//		Chainspace::Index i = state.getInfo().getChains().begin() + simHandForceMode - 1; // simHandForceMode corresponds to chain
	//		for (Configspace::Index j = state.getInfo().getJoints(i).begin(); j < state.getInfo().getJoints(i).end(); ++j) {
	//			const size_t k = j - state.getInfo().getJoints().begin(); // from the first joint
	//			const size_t l = j - state.getInfo().getJoints(i).begin(); // from the first joint in the chain
	//			//if (!objectPointCloudPtr->empty())
	//			//	context.write("Chainspace::Index i=%d, j=%d, from first joint k=%d, from first joint in chain l=%d, triggerred guard=%d (%d)\n", i, j, k, l, joints.front(), joints.size());
	//			force[k] = simForce[std::min(size_t(2), l)];
	//		}
	//	}
	//	// read from the state variable (if supported)
	//	else {
	//		try {
	//			const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	//			std::vector<Configspace::Index> joints;
	//			Real value = !objectPointCloudPtr->empty() ? collision->evaluate(this, w, *objectPointCloudPtr.get(), rand, manipulator->getPose(recvState().command), joints)*5. : REAL_ZERO;
	//			if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
	//				for (Configspace::Index j = state.getInfo().getJoints().begin(); j < state.getInfo().getJoints().end(); ++j) {
	//					const size_t k = j - state.getInfo().getJoints().begin();
	//					force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
	//				}
	//				for (auto j = joints.begin(); j != joints.end(); ++j) {
	//					const size_t k = (*j) - state.getInfo().getJoints().begin();
	//					if (k > force.size()) {
	//						context.write("k=%d > force.size()=%d\n", k, force.size());
	//						continue;
	//					}
	//					force[k] = value;
	//				}
	//				if (!joints.empty()) {
	//					const size_t k = joints.front() - state.getInfo().getJoints().begin();
	//					//context.write("-----------COLLISSION JOINT=%d k=%d force=%3.3f limit=%3.3f (size=%d)--------------\n", joints.front(), k,
	//					//	force[k], fLimit[k], joints.size());
	//				}
	//			}
	//		}
	//		catch (const Message &msg) { context.write("%s\n", msg.str()); }
	//	}

	//	bool emergency = false;
	//	try {
	//		golem::CriticalSectionWrapper csw(csData);
	//		// use customised force reader if available, otherwise the default one
	//		handForceReader ? handForceReader(state, force) : handForceReaderDflt(state, force);
	//	}
	//	catch (const std::exception& ex) {
	//		// if force threshold limit is reached
	//		if (handCtrl->getMode() != grasp::ActiveCtrl::MODE_EMERGENCY) {
	//			emergency = true;
	//			// print exception
	//			context.write("%s\n", ex.what());
	//			// set emergency mode
	//			armCtrl->setMode(grasp::ActiveCtrl::MODE_EMERGENCY);
	//			handCtrl->setMode(grasp::ActiveCtrl::MODE_EMERGENCY);
	//			// stop controller and cleanup the command queue
	//			controller->stop();
	//		}
	//	}

	//	{
	//		golem::CriticalSectionWrapper csw(csData);
	//		handForce = force;
	//		// emergency mode handler
	//		if (emergency && emergencyModeHandler) emergencyModeHandlerThread.start(emergencyModeHandler);
	//	}
	//};
	//handCtrl = desc.handCtrlDesc->create(*hand); // create and install callback (throws)

	collectFTInp = [=](const Controller::State&, grasp::RealSeq& force) {
		golem::CriticalSectionWrapper csw(csHandForce);
		for (I32 i = 0; i < dimensions(); ++i)
			forceInpSensorSeq[i][steps] = force[i];
		steps = (I32)(++steps % windowSize);
	};

	ftFilter = [=](const Controller::State&, grasp::RealSeq& force) {
		// find index as for circular buffer
		auto findIndex = [&](const I32 idx) -> I32 {
			return (I32)((steps + idx) % windowSize);
		};
		// high pass filter implementation
		auto hpFilter = [&]() {
			Real b = 1 / windowSize;

		};
		// gaussian filter
		auto conv = [&]() -> grasp::RealSeq {
			grasp::RealSeq output;
			output.assign(dimensions(), REAL_ZERO);
			{
				golem::CriticalSectionWrapper csw(csHandForce);
				for (I32 i = 0; i < dimensions(); ++i) {
					Real tmp = REAL_ZERO;
					grasp::RealSeq& seq = forceInpSensorSeq[i];
					// conv y[k] = x[k]h[0] + x[k-1]h[1] + ... + x[0]h[k]
					for (I32 j = 0; j < windowSize; ++j) {
						tmp += seq[findIndex(windowSize - j - 1)] * mask[j];
					}
					output[i] = tmp;
				}
			}
			return output;
		};

		force = conv();
	};

	handContactDetector = nullptr;

	guardsReader = [=](const Controller::State &state, grasp::RealSeq& force, std::vector<golem::Configspace::Index> &joints) {
		joints.clear();
		joints.reserve(getStateHandInfo().getJoints().size());
		ftFilter(state, handFilteredForce);
		// the loop skips the last joint because it's coupled with the 3rd joint.
		for (Configspace::Index i = getStateHandInfo().getJoints().begin(); i != getStateHandInfo().getJoints().end(); ++i) {
			const size_t k = i - getStateHandInfo().getJoints().begin();
			if (Math::abs(handFilteredForce[k]) > fLimit[k])
				joints.push_back(i);
		}
		//for (size_t i = 0; i < size; ++i)
		//	if (Math::abs(force[i]) > fLimit[i])
		//		throw Message(Message::LEVEL_NOTICE, "Robot::handForceReaderDflt(): F[%u/%u] = (%3.3lf)", i, size, force[i]);
	};

	return true;
}

//------------------------------------------------------------------------------

void Robot::render() {
	grasp::Robot::render();
}

//------------------------------------------------------------------------------

void Robot::findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(target.cpos, gwcs.wpos);
	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
	gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
	gwcs.t = target.t;
//	gwcs.wpos[armChain].p.x += 0.045;
	if (lifting) gwcs.wpos[armChain].p.z += 0.07;

	cend = target;
	{
		// lock controller
		golem::CriticalSectionWrapper csw(csController);
		// Find initial target position
		if (!planner->findTarget(target, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "spam::Robot::findTarget(): Unable to find initial target configuration");
	}
//	context.write(">\n"); //std::cout << ">\n"; //context.write(">\n");
	// set fingers pose
	for (auto i = handInfo.getJoints().begin(); i != handInfo.getJoints().end(); ++i)
		cend.cpos[i] = target.cpos[i];

	// update arm configurations and compute average error
	grasp::RBDist err;
	WorkspaceChainCoord wcc;
	controller->chainForwardTransform(cend.cpos, wcc);
	wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
	err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(gwcs.wpos[armChain])));
	context.write("Robot::findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
}

//------------------------------------------------------------------------------

//int Robot::checkGuards(std::vector<int> &triggeredGuards, golem::Controller::State &state) {
//	//golem::RobotJustin *justin = getRobotJustin();
//	//if (justin) 
//	//	return justin->getTriggeredGuards(triggeredGuards, state);
//	//else {
//		//const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
//	
//		//context.write("spam::Robot::getTriggeredGuards(): retrieving forces from state\n");
//		//grasp::RealSeq force;
//		//force.assign(handInfo.getJoints().size(), REAL_ZERO);
//		//if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
//		//	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
//		//		const size_t k = j - handInfo.getJoints().begin();
//		//		const size_t z = j - armInfo.getJoints().begin();
//		//		force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
//		//		if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
//		//			context.write("hand joint %d force=%f guard=%f\n", k, force[k], ftHandGuards[k]);
//		//			triggeredGuards.push_back(z);
//		//		}
//		//	}
//		//}
//		//return triggeredGuards.size();
//	//}
//	return triggeredGuards.empty() ? -1 : triggeredGuards.size();
//
//}
//
//int Robot::getTriggeredGuards(FTGuard::Seq &triggeredJoints, golem::Controller::State &state) {	
//	triggeredJoints.clear();
//	triggeredJoints.reserve(ftGuards.size());
//
//	const int NUM_JOINTS_CTRL = handInfo.getJoints(handInfo.getChains().begin()).size() - 1;
//	int ret = 0;
//	// in case justin moves with enable guards
//	golem::RobotJustin *justin = getRobotJustin();
//	if (justin) {
//		std::vector<int> triggeredIndeces;
//		triggeredIndeces.reserve(ftGuards.size());
//		if ((ret = justin->getTriggeredGuards(triggeredIndeces, state)) > 0 /*|| !staticObject*/) {
//			context.write("spam::Robot::getTriggeredGuards(): num=%d triggered joint(s):", triggeredIndeces.size());
//			for (std::vector<int>::const_iterator i = triggeredIndeces.begin(); i != triggeredIndeces.end(); ++i) {
//				triggeredJoints.push_back(ftGuards[*i]);
//				context.write(" <chain=%d joint=%d>", ftGuards[*i].chainIdx, ftGuards[*i].jointIdx);
//			}
//			context.write("\n");
////			return triggeredJoints.size(); // return ret;
//		}
//	}
//	else {
//		// in case robot bham
//		for (FTGuard::Seq::const_iterator i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i)
//			triggeredJoints.push_back(*i);
//		
//		//const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
//	
//		//context.write("spam::Robot::getTriggeredGuards(): retrieving forces from state. triggered joint(s): joints=%d guards=%d\n", handInfo.getJoints().size(), ftGuards.size());
//		//grasp::RealSeq force;
//		//force.assign(handInfo.getJoints().size(), REAL_ZERO);
//		//if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
//		//	for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
//		//		const size_t k = j - handInfo.getJoints().begin();
//		//		force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
//		//		if (state.get<ConfigspaceCoord>(forceOffset)[j] < ftGuards[2*k].value) {
//		//			context.write("hand chain %d joint %d (k=%d) force=%f guard=%f\n", ftGuards[2*k].chainIdx, ftGuards[2*k].joint, k, force[k], ftGuards[2*k].value);
//		//			triggeredJoints.push_back(ftGuards[2*k]);
//		//		}
//		//		else if (state.get<ConfigspaceCoord>(forceOffset)[j] > ftGuards[2*k+1].value) {
//		//			context.write("hand chain %d joint %d (k=%d) force=%f guard=%f\n", ftGuards[2*k+1].chainIdx, ftGuards[2*k+1].joint, k, force[k], ftGuards[2*k+1].value);
//		//			triggeredJoints.push_back(ftGuards[2*k+1]);
//		//		}
//		//	}
//		//}
//		ret = triggeredJoints.size();
//	}
//	//	if (!triggeredJoints.empty())
//	//		return true;
//	//}
//	context.write("spam::Robot::getTriggeredGuards(): %d triggered guard(s).\n", ret);
//	return ret;
//}
//
void Robot::readFT(const Controller::State &state, grasp::RealSeq &force) const {
	// use simulated force
	if (simHandForceMode != FORCE_MODE_DISABLED) {
		const Vec3 simForce(simModeVec.z*simForceGain.v.z, simModeVec.x*simForceGain.v.x, simModeVec.y*simForceGain.v.y);

		Chainspace::Index i = handInfo.getChains().begin(); // simHandForceMode corresponds to chain
		for (Configspace::Index j = handInfo.getJoints(i).begin(); j < handInfo.getJoints(i).end(); ++j) {
			const size_t k = j - handInfo.getJoints().begin(); // from the first joint
			const size_t l = j - handInfo.getJoints(i).begin(); // from the first joint in the chain
			force[k] = simForce[std::min(size_t(2), l)];
		}
	}
	// read from the state variable (if supported)
	else {
		const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
		if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
			Real max = REAL_ZERO;
			Real min = REAL_ZERO;
			static int jj = 0;
			for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
				const size_t k = j - handInfo.getJoints().begin();
				force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
				if (force[k] > max)
					max = force[k];
				if (force[k] < min)
					min = force[k];
			}
			//if (++jj % 100 == 0)
			//	printf("[%f/%f]\n", max, min);
		}
	}
}
//
//}
//
////size_t Robot::isGrasping(std::vector<golem::Configspace::Index> &triggeredJoints, golem::Controller::State &state) {
////	// in case justin tried to grasp with no guards enabled
////	triggeredJoints.clear();
////	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
////	state = recvState().config;
////	
////	context.write("spam::Robot::isGrasping(): retrieving forces from state (hand joints size %d, guards size %d)\n", handInfo.getJoints().size(), ftHandGuards.size());
////	grasp::RealSeq force;
////	readFT(state, force);
////	force.assign(handInfo.getJoints().size(), REAL_ZERO);
////	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
////		for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
////			const size_t k = j - handInfo.getJoints().begin();
////			//force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
////			//if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
////			if (Math::abs(force[k]) > ftHandGuards[k]) {
////				context.write("hand joint %d force=%f guard=%f\n", k, force[k], ftHandGuards[k]);
////				triggeredJoints.push_back(j);
////			}
////		}
////	}
////	context.write("robot:isGrasping: done./n");
////	return triggeredJoints.size();
////
////	// Read forces at the hand
////	//const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
////	//const Controller::State s = recvState().config;
////	//
////	//grasp::RealSeq force;
////	//force.assign(s.getInfo().getJoints().size(), REAL_ZERO);
////	//if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
////	//	for (Configspace::Index j = s.getInfo().getJoints().begin(); j < s.getInfo().getJoints().end(); ++j) {
////	//		const size_t k = j - s.getInfo().getJoints().begin();
////	//		force[k] = s.get<ConfigspaceCoord>(forceOffset)[j];
////	//	}
////	//}
////	//for (grasp::RealSeq::const_iterator i = force.begin(), j = ftHandGuards.begin(); i != force.end() && j != ftHandGuards.end(); ++i, ++j)
////	//	if (Math::abs(*i) >= *j ) 
////	//		return true;
////	//
////	//return false;
////}
//
//size_t Robot::isGrasping(FTGuard::Seq &triggeredJoints, golem::Controller::State &state) {
//	// in case justin tried to grasp with no guards enabled
//	triggeredJoints.clear();
//	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
//	state = recvState().config;
//	
//	context.write("spam::Robot::isGrasping(): retrieving forces from state (hand joints size %d, guards size %d)\n", handInfo.getJoints().size(), ftHandGuards.size());
//	grasp::RealSeq force;
//	force.assign(handInfo.getJoints().size(), REAL_ZERO);
//	size_t res = 0; // counts the triggered guards. but leaves triggeredjoint empty to avoid the belief update.
//	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
//		for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
//			const size_t k = j - handInfo.getJoints().begin();
//			force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
//			if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
//				context.write("hand joint %d force=%f guard=%f\n", k, force[k], ftHandGuards[k]);
//				res++;
////				triggeredJoints.push_back(j);
//				// TODO generate ftguards for each contact
//			}
//		}
//	}
//
//	return res; //triggeredJoints.size();
//}

void Robot::stopActiveController() {
	context.write("Disabled active controller\n");
	if (armCtrl->getMode() != grasp::ActiveCtrl::MODE_DISABLED) armCtrl.get()->setMode(grasp::ActiveCtrl::MODE_DISABLED);
	if (handCtrl->getMode() != grasp::ActiveCtrl::MODE_DISABLED) handCtrl.get()->setMode(grasp::ActiveCtrl::MODE_DISABLED);
	//simHandForceMode = FORCE_MODE_DISABLED;
	//simArmForceMode = FORCE_MODE_DISABLED;
	//workspaceMode = WORKSPACE_MODE_DISABLED;
}

void Robot::startActiveController(const golem::I32 steps) {
	context.write("Enabled active controller\n");
	if (armCtrl->getMode() != grasp::ActiveCtrl::MODE_ENABLED) armCtrl.get()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
	if (handCtrl->getMode() != grasp::ActiveCtrl::MODE_ENABLED) handCtrl.get()->setMode(grasp::ActiveCtrl::MODE_ENABLED, steps == grasp::ActiveCtrl::STEP_DEFAULT ? grasp::ActiveCtrl::STEP_DEFAULT : steps);
	//simHandForceMode = FORCE_MODE_DISABLED;
	//simArmForceMode = FORCE_MODE_DISABLED;
	//workspaceMode = WORKSPACE_MODE_DISABLED;
}

void Robot::emergencyActiveController() {
	if (handCtrl->getMode() != grasp::ActiveCtrl::MODE_EMERGENCY) {
		// set emergency mode
		armCtrl->setMode(grasp::ActiveCtrl::MODE_EMERGENCY);
		handCtrl->setMode(grasp::ActiveCtrl::MODE_EMERGENCY);
		// stop controller and cleanup the command queue
		controller->stop();
	}
}


//------------------------------------------------------------------------------

grasp::RBDist Robot::trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState::Seq seq;
	//for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i) {
	//	GenWorkspaceChainState gwcs;
	//	controller->chainForwardTransform(i->cpos, gwcs.wpos);
	//	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
	//	gwcs.t = i->t;
	//	seq.push_back(gwcs);
	//}
	//trajectory.clear();

	for (Controller::State::Seq::const_iterator i = begin; i != end; ++i) {
		GenWorkspaceChainState gwcs;
		controller->chainForwardTransform(i->cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
		// define the grasp frame
		Mat34 poseFrameInv, graspFrame, graspFrameInv;
		poseFrameInv.setInverse(gwcs.wpos[armChain]);
		graspFrame.multiply(poseFrameInv, actionFrame * modelFrame);
		graspFrameInv.setInverse(graspFrame);
		gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
//		context.write("trnTrajectory(): grasp frame at model <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
		gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
//		context.write("trnTrajectory(): grasp frame at new query <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);	
		gwcs.t = i->t; 
		seq.push_back(gwcs);
	}
	// planner debug
	//context.verbose("%s\n", plannerDebug(*planner).c_str());
	Controller::State::Seq ctrajectory;
	{
		// lock controller
		golem::CriticalSectionWrapper csw(csController);
		// Find initial target position
		Controller::State cend = *begin;
		// planner debug
		//context.verbose("%s\n", plannerWorkspaceDebug(*planner, &seq[0].wpos).c_str());
		if (!planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find initial target configuration");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	grasp::RBDist err;
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(armInfo.getJoints().begin(), armInfo.getJoints().end(), ctrajectory[i]);
		// error
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(trajectory.back().cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(seq[i].wpos[armChain])));
	}
	context.debug("Robot::createTrajectory(2): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);

	return err;
}

grasp::RBDist Robot::findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState::Seq seq;
	//// the starting pose of the robot does not need a tranformation
	//GenWorkspaceChainState gStart;
	//controller->chainForwardTransform(startPose.cpos, gStart.wpos);
	//gStart.wpos[armChain].multiply(gStart.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
	//gStart.t = startPose.t;
	//seq.push_back(gStart);
	for (Controller::State::Seq::const_iterator i = begin; i != end; ++i) {
		GenWorkspaceChainState gwcs;
		controller->chainForwardTransform(i->cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
		gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
		gwcs.t = i->t;
		seq.push_back(gwcs);
	}
	// planner debug
	//context.verbose("%s\n", plannerDebug(*planner).c_str());
	Controller::State::Seq initTrajectory, ctrajectory;
	{
		// lock controller
		golem::CriticalSectionWrapper csw(csController);
		// Find initial target position
		Controller::State cend = *begin;
		// planner debug
		//context.debug("Seq[0]: %s\n", grasp::plannerWorkspaceDebug(*planner, &seq[0].wpos).c_str());
		//context.debug("Seq[2]: %s\n", grasp::plannerWorkspaceDebug(*planner, &seq[2].wpos).c_str());
		if (!planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find initial target configuration");
		// compute the initial trajectory to move thee robot from current pose to the beginning of the approach trajectory
		if (!planner->findGlobalTrajectory(startPose, cend, initTrajectory, initTrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find initial trajectory");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	trajectory.insert(trajectory.end(), initTrajectory.begin(), initTrajectory.end());	
	grasp::RBDist err;
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(armInfo.getJoints().begin(), armInfo.getJoints().end(), ctrajectory[i]);
		// error
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(trajectory.back().cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(seq[i].wpos[armChain])));
	}
	context.debug("Robot::transformTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
	return err;
}


void Robot::createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory) {
	if (!pcend && !pwend)
		throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): no target specified");

	// Trajectory from initial position to end position
	for (golem::U32 i = 0; i < trajectoryTrials; ++i) {
		if (universe.interrupted())
			throw grasp::Interrupted();
//		context.debug("Robot::findTrajectory(): Planning movement...\n");
		// lock controller
		golem::CriticalSectionWrapper csw(csController);
		// All bounds are treated as obstacles
		physPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
		// planner debug
		//context.verbose("%s\n", plannerDebug(*planner).c_str());

		// Setup configspace target
		Controller::State cend = pcend ? *pcend : begin;
		// Workspace target
		if (pwend) {
			// Setup workspace target
			GenWorkspaceChainState wend;
			wend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
			wend.wpos[armInfo.getChains().begin()] = *pwend;
			// planner debug
			//context.verbose("%s\n", plannerWorkspaceDebug(*planner, &wend.wpos).c_str());
			if (!planner->findTarget(begin, wend, cend))
				continue;
			// update configspace coords of the hand
			if (pcend) cend.cpos.set(handInfo.getJoints(), pcend->cpos);
			// error
			WorkspaceChainCoord wcc;
			controller->chainForwardTransform(cend.cpos, wcc);
			wcc[armInfo.getChains().begin()].multiply(wcc[armInfo.getChains().begin()], controller->getChains()[armInfo.getChains().begin()]->getReferencePose());
			grasp::RBDist err;
			err.set(grasp::RBCoord(*pwend), grasp::RBCoord(wcc[armInfo.getChains().begin()]));
			context.debug("Robot::findTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
		}

		// planner debug
		//context.verbose("%s\n", plannerConfigspaceDebug(*planner, &cend.cpos).c_str());
		// Find collision-free trajectory and wait until the device is ready for new commands
		cend.t = begin.t + (t > SEC_TM_REAL_ZERO ? t : trajectoryDuration);
		if (planner->findGlobalTrajectory(begin, cend, trajectory, trajectory.begin()))
			return;// success
	}

	throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): unable to find trajectory");
}


//------------------------------------------------------------------------------

void spam::XMLData(Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create) {
	//	grasp::XMLData((grasp::Robot::Desc&)val, context, xmlcontext, create);
	RagGraphPlanner::Desc* pRagGraphPlanner(new RagGraphPlanner::Desc);
	(golem::Planner::Desc&)*pRagGraphPlanner = *val.physPlannerDesc->pPlannerDesc;
	val.physPlannerDesc->pPlannerDesc.reset(pRagGraphPlanner);
	val.physPlannerDesc->pPlannerDesc = RagGraphPlanner::Desc::load(context, xmlcontext->getContextFirst("planner"));

	FTDrivenHeuristic::Desc* pFTDrivenHeuristic(new FTDrivenHeuristic::Desc);
	(golem::Heuristic::Desc&)*pFTDrivenHeuristic = *val.physPlannerDesc->pPlannerDesc->pHeuristicDesc;
	val.physPlannerDesc->pPlannerDesc->pHeuristicDesc.reset(pFTDrivenHeuristic);
	spam::XMLData((FTDrivenHeuristic::Desc&)*val.physPlannerDesc->pPlannerDesc->pHeuristicDesc, xmlcontext->getContextFirst("rag_planner heuristic"), create);
}



//grasp::RBDist Robot::trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
//	if (begin == end)
//		throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Empty input trajectory");
//	// arm chain and joints pointers
//	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
//	const golem::Configspace::Range armJoints = armInfo.getJoints();
//	// Compute a sequence of targets corresponding to the transformed arm end-effector
//	GenWorkspaceChainState::Seq seq;
//	for (Controller::State::Seq::iterator i = trajectory.begin(); i != trajectory.end(); ++i) {
//		GenWorkspaceChainState gwcs;
//		controller->chainForwardTransform(i->cpos, gwcs.wpos);
//		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
//		gwcs.t = i->t;
//		seq.push_back(gwcs);
//	}
//	trajectory.clear();
//
//	for (Controller::State::Seq::const_iterator i = begin; i != end; ++i) {
//		GenWorkspaceChainState gwcs;
//		controller->chainForwardTransform(i->cpos, gwcs.wpos);
//		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
//		// define the grasp frame
//		Mat34 poseFrameInv, graspFrame, graspFrameInv;
//		poseFrameInv.setInverse(gwcs.wpos[armChain]);
//		graspFrame.multiply(poseFrameInv, actionFrame * modelFrame);
//		graspFrameInv.setInverse(graspFrame);
//		gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
//		//		context.write("trnTrajectory(): grasp frame at model <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
//		gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
//		//		context.write("trnTrajectory(): grasp frame at new query <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);	
//	}
//	// planner debug
//	//context.verbose("%s\n", plannerDebug(*planner).c_str());
//	Controller::State::Seq ctrajectory;
//	{
//		// lock controller
//		golem::CriticalSectionWrapper csw(csController);
//		// Find initial target position
//		Controller::State cend = *begin;
//		// planner debug
//		//context.verbose("%s\n", plannerWorkspaceDebug(*planner, &seq[0].wpos).c_str());
//		if (!planner->findTarget(*begin, seq[0], cend))
//			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find initial target configuration");
//		// Find remaining position sequence
//		if (seq.size() == 1)
//			ctrajectory.push_back(cend);
//		else if (seq.size() > 1 && !planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
//			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find trajectory");
//	}
//	// update arm configurations and compute average error
//	grasp::RBDist err;
//	Controller::State::Seq::const_iterator j = begin;
//	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
//		// copy config
//		trajectory.push_back(*j);
//		trajectory.back().set(armInfo.getJoints().begin(), armInfo.getJoints().end(), ctrajectory[i]);
//		// error
//		WorkspaceChainCoord wcc;
//		controller->chainForwardTransform(trajectory.back().cpos, wcc);
//		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
//		err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(seq[i].wpos[armChain])));
//	}
//	context.write("Robot::createTrajectory(2): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
//	return err;
//}
