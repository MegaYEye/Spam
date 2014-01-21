/** @file Robot.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Spam/Spam/Robot.h>
#include <Golem/Device/MultiCtrl/MultiCtrl.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Plan/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/PhysCtrl/Data.h>

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

Robot::Robot(Scene &scene) : grasp::Robot(scene) {
}
	
bool Robot::create(const Desc& desc) {
	grasp::Robot::create(desc); // throws

	pFTDrivenHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner->getHeuristic());

	staticObject = desc.staticObject;

	return true;
}

//------------------------------------------------------------------------------

void Robot::render() {
	grasp::Robot::render();
}

//------------------------------------------------------------------------------

void Robot::findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(target.cpos, gwcs.wpos);
	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1
	gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame
	gwcs.t = target.t;

	cend = target;
	{
		// lock controller
		golem::CriticalSectionWrapper csw(csController);
		// Find initial target position
		if (!planner->findTarget(target, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "spam::Robot::findTarget(): Unable to find initial target configuration");
	}
	// update arm configurations and compute average error
	grasp::RBDist err;
	WorkspaceChainCoord wcc;
	controller->chainForwardTransform(cend.cpos, wcc);
	wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
	err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(gwcs.wpos[armChain])));
	context.debug("spam::Robot::findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);

}

//------------------------------------------------------------------------------

int Robot::getTriggeredGuards(std::vector<golem::Configspace::Index> &triggeredJoints, golem::Controller::State &state) {
	triggeredJoints.clear();
	std::vector<int> triggeredIndeces;
	const int NUM_JOINTS_CTRL = handInfo.getJoints(handInfo.getChains().begin()).size() - 1;
	int ret = 0;
	// in case justin moves with enable guards
	if ((ret = grasp::Robot::getTriggeredGuards(triggeredIndeces, state)) > 0 || !staticObject) {
		context.write("spam::Robot::getTriggeredGuards(): num=%d triggered joint(s):", triggeredIndeces.size());
		for (std::vector<int>::const_iterator i = triggeredIndeces.begin(); i != triggeredIndeces.end(); ++i) {
			const int index = *i;//*i - 6;
			const int chain = index/NUM_JOINTS_CTRL;
			triggeredJoints.push_back(handInfo.getJoints(handInfo.getChains().begin() + chain).begin() + (index - chain*NUM_JOINTS_CTRL));
			if (index - chain*NUM_JOINTS_CTRL == 2)
				triggeredJoints.push_back(handInfo.getJoints(handInfo.getChains().begin() + chain).begin() + (index - chain*NUM_JOINTS_CTRL) + 1);
			context.write(" <chain=%d joint=%d>", chain, index - chain*NUM_JOINTS_CTRL);
		}
		context.write("\n");
		return !staticObject && ret < 0 ? triggeredIndeces.size() : ret; // return ret;
	}
	//else {
	//	// in case justin tried to grasp with no guards enabled
	//	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	//	state = recvState().config;
	//
	//	context.write("spam::Robot::getTriggeredGuards(): retrieving forces from state\n");
	//	grasp::RealSeq force;
	//	force.assign(handInfo.getJoints().size(), REAL_ZERO);
	//	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
	//		for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
	//			const size_t k = j - handInfo.getJoints().begin();
	//			force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
	//			if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
	//				context.write("hand joint %d force=%f\n", k, force[k]);
	//				triggeredJoints.push_back(j);
	//			}
	//		}
	//	}
	//	if (!triggeredJoints.empty())
	//		return true;
	//}
	context.write("spam::Robot::getTriggeredGuards(): no triggered guards\n");
	return ret;
}

int Robot::getTriggeredGuards(std::vector<grasp::FTGuard> &triggeredJoints, golem::Controller::State &state) {
	triggeredJoints.clear();
	std::vector<int> triggeredIndeces;
	const int NUM_JOINTS_CTRL = handInfo.getJoints(handInfo.getChains().begin()).size() - 1;
	int ret = 0;
	// in case justin moves with enable guards
	if ((ret = grasp::Robot::getTriggeredGuards(triggeredIndeces, state)) > 0 || !staticObject) {
		context.write("spam::Robot::getTriggeredGuards(): num=%d triggered joint(s):", triggeredIndeces.size());
		for (std::vector<int>::const_iterator i = triggeredIndeces.begin(); i != triggeredIndeces.end(); ++i) {
			triggeredJoints.push_back(ftGuards[*i]);
			context.write(" <chain=%d joint=%d>", ftGuards[*i].chainIdx, ftGuards[*i].jointIdx);
		}
		context.write("\n");
		return !staticObject && ret < 0 ? triggeredIndeces.size() : ret; // return ret;
	}
	//else {
	//	// in case justin tried to grasp with no guards enabled
	//	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	//	state = recvState().config;
	//
	//	context.write("spam::Robot::getTriggeredGuards(): retrieving forces from state\n");
	//	grasp::RealSeq force;
	//	force.assign(handInfo.getJoints().size(), REAL_ZERO);
	//	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
	//		for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
	//			const size_t k = j - handInfo.getJoints().begin();
	//			force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
	//			if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
	//				context.write("hand joint %d force=%f\n", k, force[k]);
	//				triggeredJoints.push_back(j);
	//			}
	//		}
	//	}
	//	if (!triggeredJoints.empty())
	//		return true;
	//}
	context.write("spam::Robot::getTriggeredGuards(): no triggered guards\n");
	return ret;
}

void Robot::readFT(const Controller::State &state, grasp::RealSeq &force) const {
	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	const Chainspace::Index handChain = state.getInfo().getChains().begin()+1;

	force.assign(state.getInfo().getJoints().size(), REAL_ZERO);
	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
		for (Configspace::Index j = state.getInfo().getJoints(handChain).begin(); j < state.getInfo().getJoints().end(); ++j) {
			const size_t k = j - state.getInfo().getJoints(handChain).begin();
			force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
		}
	}

}

size_t Robot::isGrasping(std::vector<golem::Configspace::Index> &triggeredJoints, golem::Controller::State &state) {
	// in case justin tried to grasp with no guards enabled
	triggeredJoints.clear();
	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	state = recvState().config;
	
	context.write("spam::Robot::isGrasping(): retrieving forces from state\n");
	grasp::RealSeq force;
	force.assign(handInfo.getJoints().size(), REAL_ZERO);
	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
		for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
			const size_t k = j - handInfo.getJoints().begin();
			force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
			if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
				context.write("hand joint %d force=%f guard=%f\n", k, force[k], ftHandGuards[k]);
				triggeredJoints.push_back(j);
			}
		}
	}

	return triggeredJoints.size();

	// Read forces at the hand
	//const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	//const Controller::State s = recvState().config;
	//
	//grasp::RealSeq force;
	//force.assign(s.getInfo().getJoints().size(), REAL_ZERO);
	//if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
	//	for (Configspace::Index j = s.getInfo().getJoints().begin(); j < s.getInfo().getJoints().end(); ++j) {
	//		const size_t k = j - s.getInfo().getJoints().begin();
	//		force[k] = s.get<ConfigspaceCoord>(forceOffset)[j];
	//	}
	//}
	//for (grasp::RealSeq::const_iterator i = force.begin(), j = ftHandGuards.begin(); i != force.end() && j != ftHandGuards.end(); ++i, ++j)
	//	if (Math::abs(*i) >= *j ) 
	//		return true;
	//
	//return false;
}

size_t Robot::isGrasping(std::vector<grasp::FTGuard> &triggeredJoints, golem::Controller::State &state) {
	// in case justin tried to grasp with no guards enabled
	triggeredJoints.clear();
	const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
	state = recvState().config;
	
	context.write("spam::Robot::isGrasping(): retrieving forces from state\n");
	grasp::RealSeq force;
	force.assign(handInfo.getJoints().size(), REAL_ZERO);
	if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
		for (Configspace::Index j = handInfo.getJoints().begin(); j < handInfo.getJoints().end(); ++j) {
			const size_t k = j - handInfo.getJoints().begin();
			force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
			if (Math::abs(state.get<ConfigspaceCoord>(forceOffset)[j]) > ftHandGuards[k]) {
				context.write("hand joint %d force=%f guard=%f\n", k, force[k], ftHandGuards[k]);
//				triggeredJoints.push_back(j);
				// TODO generate ftguards for each contact
			}
		}
	}

	return triggeredJoints.size();
}

//------------------------------------------------------------------------------
