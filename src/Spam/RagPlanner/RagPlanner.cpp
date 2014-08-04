/** @file RAGPlanner.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Plan/Data.h>
#include <Golem/PhysCtrl/Data.h>
#include <Spam/RagPlanner/RagPlanner.h>
#include <algorithm>

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

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::RagPlanner::Data::xmlData(golem::XMLContext* context, bool create) const {
	spam::PosePlanner::Data::xmlData(context, create);

	//try {
	//	if (!create || !queryPoints.empty()) {
	//		golem::XMLData(const_cast<golem::Mat34&>(actionFrame), context->getContextFirst("action_frame", create), create);
	//		golem::XMLData(const_cast<golem::Mat34&>(queryFrame), context->getContextFirst("query_frame", create), create);
	//		xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(queryPoints), std::string("query_points"), context, create);
	//	}
	//}
	//catch (const golem::MsgXMLParser& msg) {
	//	if (create)
	//		throw msg;
	//}
}

spam::RagPlanner::Data::Ptr spam::RagPlanner::Data::clone() const {
	return Ptr(new Data(*this));
}

//------------------------------------------------------------------------------

RagPlanner::RagPlanner(Scene &scene) : PosePlanner(scene), pBelief(nullptr), pHeuristic(nullptr) {
}

RagPlanner::~RagPlanner() {
}

bool RagPlanner::create(const Desc& desc) {
	PosePlanner::create(desc); // throws
	
	robot = dynamic_cast<Robot*>(PosePlanner::robot);
	if (robot == NULL)
		throw Message(Message::LEVEL_CRIT, "Player::create(): unable to cast to Robot");

	poseDataPtr = getData().end();

	pBelief = static_cast<Belief*>(pRBPose.get());
	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&robot->getPlanner()->getHeuristic());
	pHeuristic->setBelief(pBelief);

	objectBounds.reset(new Bounds::Desc::Seq());
	objectBounds->insert(objectBounds->begin(), desc.objectBounds.begin(), desc.objectBounds.end());

	//obstacleBounds.reset(new Bounds::Desc::Seq());
	//obstacleBounds->insert(obstacleBounds->begin(), desc.obstacleBounds.begin(), desc.obstacleBounds.end());
	//Bounds::SeqPtr obstacles(new Bounds::Seq());
	//for (Bounds::Desc::Seq::iterator i = obstacleBounds->begin(); i != obstacleBounds->end(); ++i)
	//	obstacles->push_back((*i)->create());
	//pHeuristic->setCollisionBounds(obstacles);

	uncEnable = desc.uncEnable;
	singleGrasp = desc.singleGrasp;
	withdrawToHomePose = desc.withdrawToHomePose;
	posterior = true;

	K = desc.K;
	theta = desc.theta;
	sampleAppearance = desc.sampleAppearance;
	timestampFac = desc.timestampFac;
	maxSamplingIter = desc.maxSamplingIter;

	trnModelPointCloud = desc.trnModelPointCloud;

	homeStates.clear();
	executedTrajectory.clear();
	triggeredGuards.clear();

	// actionFrame should be the identity if no query have been done
	//actionFrame.setId();

	pregraspFrameM.setId();
	graspFrameM.setId();
	graspFrameQ.setId();
	pregraspFrameQ.setId();

	actionFrameT.setId();

	manipulator.reset(new grasp::Manipulator(robot->getController()));
	handBounds.clear();

	trnEnable = false;
	ragDesc = desc;

	return true;
}

void RagPlanner::render() {
	PosePlanner::render();
	handRenderer.reset();
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		handRenderer.setWireColour(RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(255)));
		handRenderer.setLineWidth(Real(2.0));
		handRenderer.renderWire(handBounds.begin(), handBounds.end());
		testPose.render();
		testUpdate.render();
		debugRenderer.render();
	}
}

//------------------------------------------------------------------------------

void RagPlanner::renderTrialData(Data::Map::const_iterator dataPtr) {
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);


	}
}

void RagPlanner::renderData(Data::Map::const_iterator dataPtr) {
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		debugRenderer.reset();
	}
	PosePlanner::renderData(dataPtr);
}

void RagPlanner::renderContacts() {
	context.write("render contacts...\n");
	GenWorkspaceChainState gwcs;
	robot->getController()->chainForwardTransform(robot->recvState().config.cpos, gwcs.wpos);
	const size_t size = 10000;
	for (grasp::Cloud::PointSeq::iterator i = grasp::to<Data>(currentDataPtr)->queryPoints.begin(); i != grasp::to<Data>(currentDataPtr)->queryPoints.end(); ++i) {
		Real maxDist = REAL_MAX;
		for (Chainspace::Index j = robot->getStateHandInfo().getChains().begin(); j != robot->getStateHandInfo().getChains().end(); ++j) {
			const Real d = grasp::Cloud::getPoint(*i).distance(gwcs.wpos[j].p);
			if (d < maxDist)
				maxDist = d;
		}
		const Real lhd = pBelief->density(maxDist);
		//context.write("lhd %f\n", lhd);
		grasp::Cloud::setColour((lhd < 0.15) ? RGBA::BLUE : (lhd < 0.20) ? RGBA::YELLOW : (lhd < 0.25) ? RGBA::MAGENTA : RGBA::RED, *i);//RGBA(lhd*255, 0, 0, 0);
	}
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		pointRenderer.reset();
		data->appearance.draw(grasp::to<Data>(currentDataPtr)->queryPoints, pointRenderer);
	}
	context.write("Done.\n");
}

void RagPlanner::renderPose(const Mat34 &pose) {
//	testPose.reset();
	testPose.addAxes(pose, featureFrameSize*10);
}

void RagPlanner::renderUpdate(const golem::Mat34 &pose, const grasp::RBPose::Sample::Seq &samples) {
	testUpdate.reset();
	renderPose(pose);
	for (grasp::RBPose::Sample::Seq::const_iterator i = samples.begin(); i != samples.end(); ++i)
		testUpdate.addAxes(Mat34(i->q, i->p), featureFrameSize*i->weight*10);
}


void RagPlanner::renderHand(const golem::Controller::State &state, bool clear) {
	if (clear)
		handBounds.clear();
	{		
		golem::CriticalSectionWrapper csw(csDataRenderer);
		Bounds::Seq bounds = manipulator->getBounds(manipulator->getPose(state));
		handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
//		handRenderer.renderWire(handBounds.begin(), handBounds.end());
		//context.write("Hand bounds (size=%d)\n", handBounds.size());
		//for (Bounds::Seq::const_iterator i = handBounds.begin(); i != handBounds.end(); ++i)
		//	context.write("%s pose <%f %f %f>\n", (*i)->getName(), (*i)->getPose().p.x, (*i)->getPose().p.y, (*i)->getPose().p.z);
	}

//	WorkspaceJointCoord jointPoses;
//	robot->getController()->jointForwardTransform(state.cpos, jointPoses);
//	{
//		golem::CriticalSectionWrapper csw(csDataRenderer);
//		handRenderer.reset();
//		Bounds::Desc::SeqPtr jointBounds;
//		Bounds::SeqPtr boundsSeq;
//		for (Configspace::Index i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i) {
//			Bounds::Desc::SeqPtr boundDescSeq = robot->getController()->getJoints()[i]->getBoundsDescSeq();
//			for (Bounds::Desc::Seq::iterator boundDesc = boundDescSeq->begin(); boundDesc != boundDescSeq->end(); ++boundDesc)
//				if ((*boundDesc) != NULL) { 
//					(*boundDesc)->pose = jointPoses[i];
//					jointBounds->push_back(*boundDesc);
//				}
////			handRenderer.renderWire(boundDescSeq->begin(), boundDescSeq->end());
//		}
//		for (Bounds::Desc::Seq::iterator boundDesc = jointBounds->begin(); boundDesc != jointBounds->end(); ++boundDesc)
//			boundsSeq->push_back(boundDesc->get()->create());
//		handRenderer.renderWire(boundsSeq->begin(), boundsSeq->end());
//	}
}

//------------------------------------------------------------------------------

void RagPlanner::profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) {
	if (inp.size() < 2)
	throw Message(Message::LEVEL_ERROR, "RagPlanner::profile(): At least two waypoints required");

	grasp::Player::profile(dataPtr, inp, dur, idle);

	//if (!grasp::to<Data>(dataPtr)->queryPoints.empty() && trnEnable) {
	//	context.write("RagPlanner::profile(): Computing trajectory in a new frame...\n");
	//	golem::Controller::State::Seq seq;
	//	grasp::RBPose::Sample s = *pBelief->getSamples().begin();
	//	//context.write("action frame teach <%f %f %f> model frame <%f %f %f> action frame <%f %f %f> hypothesis <%f %f %f>\n", 
	//	//	actionFrameT.p.x, actionFrameT.p.y, actionFrameT.p.z,
	//	//	modelFrame.p.x, modelFrame.p.y, modelFrame.p.z,
	//	//	grasp::to<Data>(dataPtr)->actionFrame.p.x, grasp::to<Data>(dataPtr)->actionFrame.p.y, grasp::to<Data>(dataPtr)->actionFrame.p.z,
	//	//	s.p.x, s.p.y, s.p.z);
	//	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, inp.begin(), inp.end(), seq);
	//	//pregraspFrameQ.multiply(grasp::to<Data>(dataPtr)->actionFrame, pregraspFrameM);
	//	//graspFrameQ.multiply(grasp::to<Data>(dataPtr)->actionFrame, graspFrameM);
	//	//renderData(dataPtr);
	//	//context.write("approaching trajectory after (dur+%f)\n", dur);
	//	printTrajectory(seq, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
	//	//robot->createTrajectory(grasp::to<Data>(dataPtr)->actionFrame, inp.begin(), inp.end(), seq);
	//	grasp::Player::profile(dataPtr, seq, dur, idle);
	//}
	//else
	//	grasp::Player::profile(dataPtr, inp, dur, idle);
	//golem::Controller::State::Seq& out = grasp::to<Data>(dataPtr)->action;
	//golem::Controller::State::Seq::iterator ptr = out.end(), tmpend;
	//const golem::SecTmReal t = out.empty() ? context.getTimer().elapsed() : out.back().t;
	//const Controller::State begin(inp.front(), t), end(inp.back(), begin.t + dur);

	//// create trajectory
	//tmpend = out.end();
	//pProfile->create(inp.begin(), inp.end(), begin, end, out, ptr, tmpend);
	//// profile trajectory
	//tmpend = out.end();
	//pProfile->profile(out, ptr, tmpend);
	//// stationary waypoint in the end
	//out.push_back(Controller::State(end, end.t + idle));
}

//void RagPlanner::perform(Data::Map::iterator dataPtr) {
//	if (grasp::to<Data>(dataPtr)->action.size() < 2)
//		throw Message(Message::LEVEL_ERROR, "RagPlanner::perform(): At least two waypoints required");
//
//	context.debug("RagPlanner::perform():\n");
//
//	// compute the link between the current pose of the robot to the beginning of the action
//	golem::Controller::State::Seq initTrajectory;
//	SecTmReal init = context.getTimer().elapsed();
//	robot->createTrajectory(robot->recvState().config, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, initTrajectory);
//	executedTrajectory = initTrajectory;
//
//	golem::Controller::State::Seq completeTrajectory = initTrajectory;
//	completeTrajectory.insert(completeTrajectory.end(), grasp::to<Data>(dataPtr)->action.begin(), grasp::to<Data>(dataPtr)->action.end());
//	//context.write("trajectory to execute\ninit\n");
//	//printTrajectory(initTrajectory, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
//	//context.write("action\n");
//	//printTrajectory(grasp::to<Data>(dataPtr)->action, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
////	std::cout << "spame:perform 7\n";
//	if (!testTrajectory(completeTrajectory.begin(), completeTrajectory.end()))
//		return;
//
//	std::stringstream prefix;
////	std::cout << "spame:perform 9\n";
//	prefix << std::fixed << std::setprecision(3) << dataPtr->first << "-" << context.getTimer().elapsed();
//
//	// start recording the approach trajectory if isRecordingApp() is set
//	for (auto i: cameraSeq)
//		if (i->isRecordingApp()) i->start(prefix.str().c_str());
//	// block untilthe  first image arrives
//	for (auto i: cameraSeq)
//		if (i->isRecordingApp()) i->wait();
//
////	robot->getController()->initControlCycle();
//	// go to initial state
//	universe.postScreenCaptureFrames(-1);
//	std::cout << "spam:perform 1\n";
//	robot->sendTrajectory(initTrajectory, true);
//	std::cout << "spam:perform 2\n";
//	robot->waitForEnd();
//	// set the current state of the robot after the trajectory is over
//	Controller::State state = robot->recvState().config; //getController()->createState();
////	robot->getController()->lookupState(context.getTimer().elapsed(), state);
//	//renderContacts();
//	std::cout << "spam:perform 3\n";
//	::Sleep(2000);
//	std::cout << "spam:perform 4\n";
//	universe.postScreenCaptureFrames(0);	
//	
//	handBounds.clear();
//	renderHand(state);
//	//context.debug("init trajectory timesteps: %f - %f (state %f)\n", initTrajectory.front().t, initTrajectory.back().t, state.t);
//
//	// reset guards allowing the bham to move again
//	golem::RobotJustin *justin = robot->getRobotJustin();
//	if (!justin) {
//		Controller::State::Seq seq, out;
//		robot->initControlCycle();
//		grasp::RobotState::List tmp;
//		grasp::RobotState s = robot->recvState(context.getTimer().elapsed());
//		s.config = state;
//		s.command = state;
//		tmp.push_back(s);
//		tmp.push_back(s);
//
//		extrapolate(makeCommand(tmp), trjApproachExtrapolFac, false, seq);
//		// compute profile
//		golem::Controller::State::Seq::iterator ptr = out.end(), tmpend;
//		const golem::SecTmReal t = out.empty() ? context.getTimer().elapsed() : out.back().t;
//		const Controller::State begin(seq.front(), t), end(seq.back(), begin.t + trjApproachDuration);
//
//		// create trajectory
//		tmpend = out.end();
//		pProfile->create(seq.begin(), seq.end(), begin, end, out, ptr, tmpend);
//		// profile trajectory
//		tmpend = out.end();
//		pProfile->profile(out, ptr, tmpend);
//
//		robot->resumeGuards(out);
//	}
//
//	//std::cout << "spam:perform 6\n";
//	//Controller::State state = robot->getController()->createState();
//	std::cout << "spam:perform 7\n";
//	grasp::to<Data>(dataPtr)->triggered = robot->getTriggeredGuards(/*grasp::to<Data>(dataPtr)->*/triggeredGuards, state); 
//
//	// -1 means no contact between the object and the hand (most likely a contact with the table).
//	if (grasp::to<Data>(dataPtr)->triggered == -1 && grasp::to<Data>(dataPtr)->replanning)
//		return;
//	
//	//Controller::State command = robot->getController()->createState(), config = robot->getController()->createState();
//	//robot->getController()->lookupState(state.t, config);
//	//printState(config, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end(), "config", true);
//	//context.write("command state with triggered guards\n");
//	//robot->getController()->lookupCommand(state.t, command);
//	//printState(command, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end(), "command", true);
//
//	// triggered == 0 means no unexpected contact (we can grasp)
//	if (grasp::to<Data>(dataPtr)->triggered == 0) {
//		// clear states
////		dataPtr->second.robotStates.clear();
//
//		universe.postScreenCaptureFrames(-1);
//		// send trajectory
//		robot->enableGuards(false);
//		robot->sendTrajectory(grasp::to<Data>(dataPtr)->action, true);
//
//		// repeat every send waypoint until trajectory end
//		for (U32 i = 0; robot->waitForBegin(); ++i) {
//			if (universe.interrupted())
//				throw grasp::Interrupted();
//			if (robot->waitForEnd(0))
//				break;
//
//			// print every 10th robot state
//			if (i%10 == 0)
//				context.write("State #%d\r", i);
//
////			dataPtr->second.robotStates.push_back(robot->recvState());
//		}
//		::Sleep(100);
//		universe.postScreenCaptureFrames(0);	
//		const int key = waitKey("YN", "Do you want to accept this grasp? (Y/N)...");
//		if (key == 'Y')
//			grasp::to<Data>(dataPtr)->triggered = robot->isGrasping(/*grasp::to<Data>(dataPtr)->*/triggeredGuards, state);
//
//		if (grasp::to<Data>(dataPtr)->replanning = grasp::to<Data>(dataPtr)->triggered == 0)
//			grasp::to<Data>(dataPtr)->release = true;
//		context.write("Perform(1): return (triggered=%d, replanning=%s)\n",grasp::to<Data>(dataPtr)->triggered, grasp::to<Data>(dataPtr)->replanning ? "true" : "false"); 
//	}
//	// if a contact occured but we do not want to trigger replanning
//	else if (!grasp::to<Data>(dataPtr)->replanning) {
//		// ensure that config and command are the same
//		::Sleep(2000);
//		robot->initControlCycle();
//		robot->enableGuards(false);
//		// clear states
////		dataPtr->second.robotStates.clear();
//		Controller::State::Seq seq;
//		Controller::State graspPose = robot->recvState().config;
//		// setting only the fingers to grasp pose
//		for (Configspace::Index j = robot->getStateHandInfo().getJoints().begin(); j != robot->getStateHandInfo().getJoints().end(); ++j)
//			graspPose.cpos[j] = grasp::to<Data>(dataPtr)->actionApproach.begin()->command.cpos[j];
//		seq.push_back(robot->recvState().config);
//		seq.push_back(graspPose);
//		// define the profile
//		grasp::to<Data>(dataPtr)->action.clear();
//		Player::profile(dataPtr, seq, trjApproachDuration, trjApproachIdle);
//		// copy the planned trajectory in my trial data
//		//poseDataPtr->second.action.clear();
//		//for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
//		//	poseDataPtr->second.action.push_back(*i);
//		// move robot
//		universe.postScreenCaptureFrames(-1);
//		robot->sendTrajectory(grasp::to<Data>(dataPtr)->action, true);
//		// repeat every send waypoint until trajectory end
//		for (U32 i = 0; robot->waitForBegin(); ++i) {
//			if (universe.interrupted())
//				throw grasp::Interrupted();
//			if (robot->waitForEnd(0))
//				break;
//
//			// print every 10th robot state
//			if (i%10 == 0)
//				context.write("State #%d\r", i);
//
////			dataPtr->second.robotStates.push_back(robot->recvState());
//		}
//		::Sleep(100);
//		universe.postScreenCaptureFrames(0);	
//		context.write("(Single grasp attempt) Performance finished!\n");
//		return;
//	}
//
//	// stop recording at the last frame
//	// HACK: comment out for Justin! (stop manually due to faulty sync)
//	for (auto i: cameraSeq)
//		i->stop(context.getTimer().elapsed() + trjManipPerfOff);//robotStates.back().config.t;
//
//	//std::cout << "spame:perform 37\n";
//	//grasp::to<Data>(dataPtr)->triggeredStates.clear();
//	//std::cout << "spame:perform 38\n";
//	//grasp::to<Data>(dataPtr)->triggeredStates.push_back(robot->recvState().config/*state*/);
//	//std::cout << "spame:perform 39\n";
//	//grasp::to<Data>(dataPtr)->triggeredStates.push_back(Controller::State(state));
//	//std::cout << "spam:perform(): print config state with triggered guards (n. of states: " << grasp::to<Data>(dataPtr)->triggeredStates.size() << ")\n";
//	//std::cout << "spam:perform(): print first element config state with triggered guards\n";
//	//std::cout << "cpos[0]=" << /*grasp::to<Data>(dataPtr)->triggeredStates.begin()->*/robot->recvState().config.cpos[robot->getStateInfo().getJoints().begin()];
//	//printState(*grasp::to<Data>(dataPtr)->triggeredStates.begin(), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
//
//	universe.postScreenCaptureFrames(-1);
//	std::cout << "spame:perform 40\n";
//	updateAndResample(dataPtr);
////	std::cout << "spame:perform 41\n";
//	::Sleep(100);
//	universe.postScreenCaptureFrames(0);	
//	//dataPtr->second.replanning = false;
//	//for (size_t i = 0; i < grasp::RBCoord::N; ++i) {
//	//	if (pBelief->getSampleProperties().covarianceSqrt[i] > 0.0000001) {
//	//		dataPtr->second.replanning = true;
//	//		break;
//	//	}
//	//}
//
////	std::cout << "spame:perform 42\n";
//	if (grasp::to<Data>(dataPtr)->triggered > 0/* && dataPtr->second.replanning*/) {
//		context.write("Perform(2): return (triggered=%s, replanning=%s)\n", grasp::to<Data>(dataPtr)->triggered > 0 ? "true" : "false", grasp::to<Data>(dataPtr)->replanning ? "true" : "false"); 
////	std::cout << "spame:perform 43\n";
//		robot->initControlCycle();
////		return;
////	std::cout << "spame:perform 44\n";
//	}	
//
////	std::cout << "spame:perform 45\n";
//	
//	context.write("Performance finished!\n");
//}

void RagPlanner::perform(Data::Map::iterator dataPtr) {
	if (grasp::to<Data>(dataPtr)->action.size() < 2)
		throw Message(Message::LEVEL_ERROR, "RagPlanner::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	robot->createTrajectory(robot->recvState().command, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, initTrajectory);
	
	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), grasp::to<Data>(dataPtr)->action.begin(), grasp::to<Data>(dataPtr)->action.end());
	if (!testTrajectory(completeTrajectory.begin(), completeTrajectory.end()))
		return;

	std::stringstream prefix;
	prefix << std::fixed << std::setprecision(3) << dataPtr->first << "-" << context.getTimer().elapsed();

	// start recording the approach trajectory if isRecordingApp() is set
	//for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
	//	if ((*i)->isRecordingApp()) (*i)->start(prefix.str().c_str());
	// block untilthe  first image arrives
	//for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
	//	if ((*i)->isRecordingApp()) (*i)->wait();

	// go to initial state
	robot->sendTrajectory(initTrajectory, true);
	// wait until the last trajectory segment is sent
	robot->waitForEnd();
	
	// start recording the remaining trajectory part
	for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i) 
		if (!(*i)->isRecordingApp()) (*i)->start(prefix.str().c_str());
	// block until the first image arrives
	for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i) 
		if (!(*i)->isRecordingApp()) (*i)->wait();
	
	// clear states
	Robot::State::Seq robotStates;

	// send trajectory
	if (screenCapture) universe.postScreenCaptureFrames(-1);
	robot->resumeGuards();
	robot->sendTrajectory(grasp::to<Data>(dataPtr)->action);

	// repeat every send waypoint until trajectory end
	for (U32 i = 0; robot->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw grasp::Interrupted();
		if (robot->waitForEnd(0))
			break;

		// print every 10th robot state
		if (i%10 == 0) {
			context.write("State #%d\r", i);
			//if (i>100) {
			//	context.write("fakes contacts\n");
			//	robot->stop = true;
			//}
			if (!robot->objectPointCloudPtr->empty()) {
				robot->triggeredGuards.clear();
				Controller::State state = robot->recvState().command;
				grasp::RealSeq force;
				force.assign(state.getInfo().getJoints().size(), REAL_ZERO);
				for (Configspace::Index j = state.getInfo().getJoints().begin(); j < state.getInfo().getJoints().end(); ++j) {
					const size_t k = j - state.getInfo().getJoints().begin();
					golem::Bounds::Seq bounds;// = manipulator->getBounds(manipulator->getPose(state));
					golem::WorkspaceJointCoord wjc;
					golem::Mat34 poses[grasp::Manipulator::JOINTS];
					//manipulator->getPoses(manipulator->getPose(state.cpos), poses);
					robot->getController()->jointForwardTransform(state.cpos, wjc);
					const golem::U32 joint = manipulator->getArmJoints() + k;
					manipulator->getJointBounds(golem::U32(j - manipulator->getController()->getStateInfo().getJoints().begin()), /*poses[joint]*/wjc[j], bounds);
					bool intersect = false;
					for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b) {
						for (size_t i = 0; i < 10000; ++i) {
							const Vec3 point = grasp::Cloud::getPoint(robot->objectPointCloudPtr->at(size_t(rand.next())%robot->objectPointCloudPtr->size()));
							//context.write("force reader\n");
							//context.write("bound pose <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
							if ((*b)->intersect(point)) {
								context.write("Force reader:\n");
								context.write("intersection bound<%f %f %f> point<%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z, point.x, point.y, point.z);
								//Vec3 v;
								//Mat34 jointFrameInv;
								//jointFrameInv.setInverse(wjc[j]);
								//jointFrameInv.multiply(v, point);
								//v.normalise();
								//force[k] = v.z > REAL_ZERO ? -REAL_ONE : REAL_ONE;
								//robot->triggeredGuards.push_back(robot->getFTGuard(v.z < REAL_ZERO ? 2*k : 2*k+1));
								//intersect = true;
								//context.write("intersection=true joint=%d force=%d\n", golem::U32(j-state.getInfo().getJoints().begin()), force[k]);
								context.write("---------------------------------------------------\n");
							}
						}
					}
				}
			}
			//		if (intersect) {
			//			golem::Controller::State::Seq tmp;
			//			robot->createTrajectory(robot->recvState().command, &state, NULL, 0, tmp);
			//			robot->sendTrajectory(tmp, true);
			//			return;
			//		}
			//	}
			//}
		}
		robotStates.push_back(robot->recvState());
	}
	::Sleep(100);
	if (screenCapture) universe.postScreenCaptureFrames(0);	

	// stop recording at the last frame
	// HACK: comment out for Justin! (stop manually due to faulty sync)
	for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
		(*i)->stop(context.getTimer().elapsed() + trjManipPerfOff);//robotStates.back().config.t;
	
	context.write("Performance finished!\n");

	// writing data into a text file, data conversion
	auto strMat34 = [=] (std::ostream& ostr, const Mat34& mat) {
		Quat quat(mat.R);
		ostr << mat.p.x << grasp::to<Data>(data)->sepField << mat.p.y << grasp::to<Data>(data)->sepField << mat.p.z << grasp::to<Data>(data)->sepField << quat.x << grasp::to<Data>(data)->sepField << quat.y << grasp::to<Data>(data)->sepField << quat.z << grasp::to<Data>(data)->sepField << quat.w << grasp::to<Data>(data)->sepField;
	};
	auto strMat34Desc = [=] (std::ostream& ostr, const std::string& prefix, U32 index) {
		ostr << prefix << "x" << index << grasp::to<Data>(data)->sepField << prefix << "y" << index << grasp::to<Data>(data)->sepField << prefix << "z" << index << grasp::to<Data>(data)->sepField << prefix << "qx" << index << grasp::to<Data>(data)->sepField << prefix << "qy" << index << grasp::to<Data>(data)->sepField << prefix << "qz" << index << grasp::to<Data>(data)->sepField << prefix << "qw" << index << grasp::to<Data>(data)->sepField;
	};
	auto strState = [=] (std::ostream& ostr, const Controller::State& state, const Controller::State::Info& info) {
		ostr << state.t << grasp::to<Data>(data)->sepField;
		WorkspaceJointCoord wjc;
		robot->getController()->jointForwardTransform(state.cpos, wjc);
		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
			ostr << state.cpos[i] << grasp::to<Data>(data)->sepField;
			strMat34(ostr, wjc[i]);
		}
	};
	auto strStateDesc = [=] (std::ostream& ostr, const Controller::State::Info& info, const std::string& prefix) {
		ostr << prefix << "t" << grasp::to<Data>(data)->sepField;
		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
			ostr << prefix << "j" << *i << grasp::to<Data>(data)->sepField;
			strMat34Desc(ostr, prefix, *i);
		}
	};
	auto strFT = [=] (std::ostream& ostr, const Twist& twist) {
		ostr << twist.v.x << grasp::to<Data>(data)->sepField << twist.v.y << grasp::to<Data>(data)->sepField << twist.v.z << grasp::to<Data>(data)->sepField << twist.w.x << grasp::to<Data>(data)->sepField << twist.w.y << grasp::to<Data>(data)->sepField << twist.w.z << grasp::to<Data>(data)->sepField;
	};
	auto strFTDesc = [=] (std::ostream& ostr, const std::string& prefix) {
		ostr << prefix << "vx" << grasp::to<Data>(data)->sepField << prefix << "vy" << grasp::to<Data>(data)->sepField << prefix << "vz" << grasp::to<Data>(data)->sepField << prefix << "wx" << grasp::to<Data>(data)->sepField << prefix << "wy" << grasp::to<Data>(data)->sepField << prefix << "wz" << grasp::to<Data>(data)->sepField;
	};

	// writing data into a text file, open file
	const std::string dataIndexPath = grasp::makeString("%s%s.txt", data->dir.c_str(), prefix.str().c_str());
	std::ofstream dataIndex(dataIndexPath);

	// writing data into a text file, prepare headers
	dataIndex << "#index" << grasp::to<Data>(data)->sepField;
	strStateDesc(dataIndex, robot->getController()->getStateInfo(), std::string("cmd_"));
	strStateDesc(dataIndex, robot->getController()->getStateInfo(), std::string("cfg_"));
	strFTDesc(dataIndex, std::string("ft_"));
	dataIndex << std::endl;

	// writing data into a text file
	U32 j = 0;
	for (grasp::RobotState::Seq::const_iterator i = robotStates.begin(); i != robotStates.end(); ++i) {
		dataIndex << ++j << grasp::to<Data>(data)->sepField;
		strState(dataIndex, i->command, robot->getController()->getStateInfo());
		strState(dataIndex, i->config, robot->getController()->getStateInfo());
		strFT(dataIndex, i->ftSensor);
		dataIndex << std::endl;
	}
}

void RagPlanner::performApproach(Data::Map::iterator dataPtr) {
	if (grasp::to<Data>(queryDataPtr)->queryPoints.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::performApproach(): no query is present.");

	context.debug("RagPlanner::performApproach(): build and perform approach trajectory\n");
	// fixed properties of the planner
	pHeuristic->enableUnc = this->uncEnable;
	pHeuristic->setCollisionDetection(Bounds::GROUP_ALL);
	pHeuristic->setPointCloudCollision(true);

	context.debug("RagPlanner::performApproach(): enable guards\n");
	robot->enableGuards();
	grasp::to<Data>(dataPtr)->replanning = true;
	robot->setCollisionDetection(true);		

	// handle the approaching action to generate a sequence of states 
	//context.write("approaching trajetory (list)\n");
	//printTrajectory(makeCommand(dataPtr->second.approachAction), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
//	createGrasp(dataPtr);
	//context.write("approaching trajetory (list)\n");
	//printTrajectory(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
	Controller::State::Seq seq;
	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);

	// compute the approach traectory in a new frame and its profile
	context.debug("RagPlanner::performApproach(): Computing trajectory in a new frame and generating profile...\n");
	Controller::State::Seq seqTrn, trjApproach, trjGrasp;
	// compute trajectory to the pre-grasp pose with point cloud collision detection ON
	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.begin(), seq.begin() + 1, seqTrn);

	// debug: shows pregrasp and grasp poses of the hand
	context.debug("RagPlanner::performApproach(): render pre-grasp pose.\n");
	Controller::State pregrasp = seqTrn.front();
	renderHand(pregrasp, true);
	//Controller::State grasp = seqTrn.back();
	//renderHand(grasp);
	grasp::Manipulator::Pose pose1 = manipulator->getPose(pregrasp);
	context.debug("RagPlanner::performApproach(): manipulator pose <%f %f %f>\n", pose1.p.x, pose1.p.y, pose1.p.z);

	robot->createTrajectory(robot->recvState().config, &pregrasp, NULL, 0, trjApproach);
	grasp::to<Data>(dataPtr)->action.clear();
//	trjApproach.insert(trjApproach.end(), seqTrn.begin(), seqTrn.end()-1);
	context.debug("RagPlanner::performApproach(): compute profile.\n");
	profile(dataPtr, trjApproach, trjApproachDuration, trjApproachIdle);

	// perform approaching action
	context.debug("RagPlanner::performApproach(): perform action.\n");
	perform(dataPtr);

	// set the current state of the robot after the trajectory is over
	context.debug("RagPlanner::performApproach(): retrieve current state of the robot and check guards.\n");
	Controller::State state = robot->recvState().config;
	//renderHand(state, true);
	grasp::Manipulator::Pose pose = manipulator->getPose(state);
	context.debug("RagPlanner::performApproach(): manipulator pose <%f %f %f>\n", pose.p.x, pose.p.y, pose.p.z);
	grasp::to<Data>(dataPtr)->triggered = robot->getTriggeredGuards(/*grasp::to<Data>(dataPtr)->*/triggeredGuards, state); 

	// -1 means no contact between the object and the hand (most likely a contact with the table).
	if (grasp::to<Data>(dataPtr)->triggered == -1)
		return;

	// triggered == 0 means no unexpected contact (we can grasp)
	if (grasp::to<Data>(dataPtr)->triggered == 0) {
		context.debug("RagPlanner::performApproach(): No unexpected contact:. we grasp! (disable guards)\n");
		pHeuristic->setPointCloudCollision(false);
//		pHeuristic->setCollisionDetection(false);
		// compute trajectory to the grasp pose with point cloud collision detection OFF
		seqTrn.clear();
		robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.end() - 1, seq.end(), seqTrn);
		robot->createTrajectory(robot->recvState().command, &seqTrn.front(), NULL, 0, trjGrasp);
		grasp::to<Data>(dataPtr)->action.clear();
		profile(dataPtr, trjGrasp, SecTmReal(5.0), trjApproachIdle);
		robot->enableGuards(false);
		// perform grasping action
		perform(dataPtr);
		printTrajectory(grasp::to<Data>(dataPtr)->action, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
		if (grasp::to<Data>(dataPtr)->triggered = robot->isGrasping(/*grasp::to<Data>(dataPtr)->*/triggeredGuards, robot->recvState().config) > 0) {
			const int key = waitKey("YN", "Do you accept this grasp? (Y/N)...");
			grasp::to<Data>(dataPtr)->replanning = key == 'Y'? false : true;
		}
	}
	//else {
	//	context.debug("RagPlanner::performApproach(): Contacts occurred: restore EMERGENCY MODE for BHAM robot\n");
	//	// reset guards allowing the bham to move again
	//	golem::RobotJustin *justin = robot->getRobotJustin();
	//	if (!justin) {
	//		Controller::State::Seq seq, out;
	//		robot->initControlCycle();
	//		grasp::RobotState::List tmp;
	//		grasp::RobotState s = robot->recvState(context.getTimer().elapsed());
	//		s.config = state;
	//		s.command = state;
	//		tmp.push_back(s);
	//		tmp.push_back(s);

	//		extrapolate(makeCommand(tmp), trjApproachExtrapolFac, false, seq);
	//		// compute profile
	//		golem::Controller::State::Seq::iterator ptr = out.end(), tmpend;
	//		const golem::SecTmReal t = out.empty() ? context.getTimer().elapsed() : out.back().t;
	//		const Controller::State begin(seq.front(), t), end(seq.back(), begin.t + SecTmReal(1.0));

	//		// create trajectory
	//		tmpend = out.end();
	//		pProfile->create(seq.begin(), seq.end(), begin, end, out, ptr, tmpend);
	//		// profile trajectory
	//		tmpend = out.end();
	//		pProfile->profile(out, ptr, tmpend);

	//		robot->resumeGuards(out);
	//	}
	//}

	// save executed trajectory
	context.debug("RagPlanner::performApproach(): saving executed trajectory (length=%d)\n", grasp::to<Data>(dataPtr)->action.size());
	executedTrajectory.clear();
	executedTrajectory.reserve(grasp::to<Data>(dataPtr)->action.size());
	for (Controller::State::Seq::const_iterator i = grasp::to<Data>(dataPtr)->action.begin(); i != grasp::to<Data>(dataPtr)->action.end(); ++i)
		executedTrajectory.push_back(*i);

	context.debug("RagPlanner::performApproach(): update and resample belief\n");
	updateAndResample(dataPtr);

	context.write("RagPlanner::performApproach(): return (triggered=%s, replanning=%s)\n", grasp::to<Data>(dataPtr)->triggered > 0 ? "true" : "false", grasp::to<Data>(dataPtr)->replanning ? "true" : "false"); 
//	robot->initControlCycle();
}

void RagPlanner::performWithdraw(Data::Map::iterator dataPtr) {
	context.debug("RagPlanner::performWithdraw(): build and perform withdraw trajectory (vanilla version withdraw to initial state).\n");
	// handle the approaching action to generate a sequence of states 
	Controller::State::Seq seq, tmp, trjWithdraw;
//	robot->initControlCycle();

	// fixed properties of the planner
	robot->enableGuards(/*false*/);
	pHeuristic->enableUnc = false;
	pHeuristic->setPointCloudCollision(false);
	robot->setCollisionDetection(true);
//	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionWithdraw), trjApproachExtrapolFac, false, seq);

	//robot->resumeGuards();
	//if (screenCapture) universe.postScreenCaptureFrames(-1);
	//robot->set(grasp::to<Data>(dataPtr)->actionWithdraw.front());
	//::Sleep(100);
	//if (screenCapture) universe.postScreenCaptureFrames(0);	

	// compute the approach traectory in a new frame and its profile
	robot->createTrajectory(robot->recvState().config, &grasp::to<Data>(dataPtr)->actionWithdraw.front().command, NULL, 0, trjWithdraw);
	
	grasp::to<Data>(dataPtr)->action.clear();
	profile(dataPtr, trjWithdraw, trjApproachDuration, trjApproachIdle);

	// perform manip action
	perform(dataPtr);
//
//	//grasp::to<Data>(dataPtr)->action.clear();
//	//golem::Controller::State::Seq trjWithdraw;
//	//tmp.push_back(robot->recvState().config);
//	//tmp.insert(tmp.end(), seq.begin(), seq.end());
//	//profile(dataPtr, trjWithdraw, trjApproachDuration, trjApproachIdle);
//	//robot->createTrajectory(robot->recvState().config, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, trjWithdraw);
////	dataPtr->second.executedTrajectory = initTrajectory;
//	
//
//	if (!testTrajectory(trjWithdraw.begin(), trjWithdraw.end()))
//		return;
//
//	robot->resumeGuards();
//	if (screenCapture) universe.postScreenCaptureFrames(-1);
//	robot->sendTrajectory(trjWithdraw, true);
//	::Sleep(100);
//	if (screenCapture) universe.postScreenCaptureFrames(0);	
//	robot->waitForEnd();
}

void RagPlanner::performManip(Data::Map::iterator dataPtr) {
	context.write("RagPlanner::performManip(): build and perform manipulative trajectory (vanilla version just lift the object up).\n");
	// handle the approaching action to generate a sequence of states 
//	robot->initControlCycle();
	// fixed properties of the planner
	robot->enableGuards(false);
	pHeuristic->enableUnc = false;
	pHeuristic->setPointCloudCollision(false);
//	robot->setCollisionDetection(false);

	Controller::State::Seq seq;
	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionManip), trjApproachExtrapolFac, false, seq);

		// compute the approach traectory in a new frame and its profile
	context.debug("RagPlanner::performApproach(): Computing trajectory in a new frame and generating profile...\n");
	Controller::State::Seq seqTrn, trjManip;
	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.begin(), seq.end(), seqTrn);

	robot->createTrajectory(robot->recvState().config, &seqTrn.front(), NULL, 0, trjManip);

	// compute the approach traectory in a new frame and its profile
	grasp::to<Data>(dataPtr)->action.clear();
	profile(dataPtr, trjManip, trjManipDuration, trjManipIdle);

	perform(dataPtr);
	//dataPtr->second.action.clear();
	//queryDataPtr->second.action.clear();
}


void RagPlanner::performSingleGrasp(Data::Map::iterator dataPtr) {
	if (grasp::to<Data>(queryDataPtr)->queryPoints.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::performSingleGrasp(): no query is present.");

	context.write("RagPlanner::performSingleGrasp(): build and perform approach trajectory\n");
	// fixed properties of the planner
	pHeuristic->enableUnc = false;
	pHeuristic->setCollisionDetection(Bounds::GROUP_ALL);
	pHeuristic->setPointCloudCollision(true);

	robot->enableGuards(false);
	grasp::to<Data>(dataPtr)->replanning = false;
	robot->setCollisionDetection(true);		

	// handle the approaching action to generate a sequence of states 
	//context.write("approaching trajetory (list)\n");
	//printTrajectory(makeCommand(dataPtr->second.approachAction), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());		
	Controller::State::Seq seq;
	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
	Controller::State::Seq seqTrn, trjApproach, trjGrasp;
	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.begin(), seq.end(), seqTrn);

	// debug: shows pregrasp and grasp poses of the hand
	context.debug("RagPlanner::performApproach(): render pre-grasp pose.\n");
	Controller::State pregrasp = seqTrn.front();
	renderHand(pregrasp, true);
	//Controller::State grasp = seqTrn.back();
	//renderHand(grasp);

	robot->createTrajectory(robot->recvState().config, &pregrasp, NULL, 0, trjApproach);
	// compute the approach traectory in a new frame and its profile
	grasp::to<Data>(dataPtr)->action.clear();
	//grasp::to<Data>(poseDataPtr)->action.clear();
	profile(dataPtr, trjApproach, trjApproachDuration, trjApproachIdle);
	//// copy the planned trajectory in my trial data
	//grasp::to<Data>(poseDataPtr)->action.clear();
	//for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
	//	poseDataPtr->second.action.push_back(*i);

	// perform action
	perform(dataPtr);

	::Sleep(2000);
	robot->initControlCycle();
	robot->enableGuards(false);
	// clear states
//		dataPtr->second.robotStates.clear();
	seq.clear();
	Controller::State graspPose = robot->recvState().config;
	// setting only the fingers to grasp pose
	for (Configspace::Index j = robot->getStateHandInfo().getJoints().begin(); j != robot->getStateHandInfo().getJoints().end(); ++j)
		graspPose.cpos[j] = grasp::to<Data>(dataPtr)->actionApproach.begin()->command.cpos[j];
	seq.push_back(robot->recvState().config);
	seq.push_back(graspPose);
	// define the profile
	grasp::to<Data>(dataPtr)->action.clear();
	profile(dataPtr, seq, trjApproachDuration, trjApproachIdle);

	// move robot
	robot->sendTrajectory(grasp::to<Data>(dataPtr)->action, true);
	// repeat every send waypoint until trajectory end
	for (U32 i = 0; robot->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw grasp::Interrupted();
		if (robot->waitForEnd(0))
			break;

		// print every 10th robot state
		if (i%10 == 0)
			context.write("State #%d\r", i);
	}
	::Sleep(100);
	context.debug("(Single grasp attempt) Performance finished!\n");
}


//------------------------------------------------------------------------------

//void RagPlanner::generate(grasp::RBPose::Sample::Seq::const_iterator begin, grasp::RBPose::Sample::Seq::const_iterator end, TrialData::Map::iterator dataPtr, grasp::RBPose::Sample::Seq &candidates) const {
//	// copies the full set of samples as a high dimension approximation of the pdf
//	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
//	for (grasp::RBPose::Sample::Seq::const_iterator sample = begin; sample != end; ++sample) {
//		grasp::RBPose::Sample s = *sample;
//		s.weight = pRBPose->density(s);
//		dataPtr->second.pdf.push_back(s);
//		golem::kahanSum(norm, c, s.weight);
////			std::printf("sample frame [%.4f,%.4f,%.4f,%.4f]<%.4f,%.4f,%.4f>  w=%.5f  norm=%.5f\n", 
////				s.q.w, s.q.x, s.q.y, s.q.z, s.p.x, s.p.y, s.p.z, s.weight, norm);
//	}
//
//	// computes the normalised importance weight associated to each sample
//	dataPtr->second.normFac = norm = golem::REAL_ONE/norm;
//	candidates.clear();
//	candidates.reserve(dataPtr->second.pdf.size());
//		
//	for (grasp::RBPose::Sample::Seq::iterator i = dataPtr->second.pdf.begin(); i != dataPtr->second.pdf.end(); ++i)
//		if ((i->weight *= norm) > theta)
//			candidates.push_back(*i);
//}
//
//golem::Real RagPlanner::evaluate(const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample) const {
//	// compute the relative transformation between the pose of the joint and the sample
//	//context.write("RagPlanner::evaluate(): pose <%5.7f %5.7f %5.7f>, sample <%5.7f %5.7f %5.7f>, dist=%5.7f\n",
//	//	pose.p.x, pose.p.y, pose.p.z, sample.p.x, sample.p.y, sample.p.z, sample.p.distance(pose.p));
//	Mat34 queryFrameInv, sampleFrame(sample.q, sample.p), relativeJointPose;
//	queryFrameInv.setInverse(sampleFrame/* * modelFrame*/);
//	relativeJointPose.multiply(queryFrameInv, Mat34(pose.q, pose.p));
//	//context.write("        query-to-joint frame <%5.7f %5.7f %5.7f>\n",
//	//	relativeJointPose.p.x, relativeJointPose.p.y, relativeJointPose.p.z, modelFrame.p.x, modelFrame.p.y, modelFrame.p.z, modelFrame.p.distance(relativeJointPose.p));
//
//	relativeJointPose.multiply(modelFrame, relativeJointPose);
//	//context.write("                    new pose <%5.7f %5.7f %5.7f>,  model <%5.7f %5.7f %5.7f>, dist=%5.7f\n",
//	//	relativejointpose.p.x, relativejointpose.p.y, relativejointpose.p.z, modelframe.p.x, modelframe.p.y, modelframe.p.z, modelframe.p.distance(relativejointpose.p));
//	Real distMin = golem::REAL_ZERO;
//	for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
//		const Real dist = relativeJointPose.p.distance(grasp::Cloud::getPoint(*point));
//		if (dist > distMin)
//			distMin = dist;
//	}
////	context.write(" -> dist(min)=%5.7f, prob of touching=%5.7f\n", distMin, Math::exp<golem::Real>(-distMin));
//	return Math::exp<golem::Real>(-distMin);
//}

void RagPlanner::updateAndResample(Data::Map::iterator dataPtr) {
	context.debug("RagPlanner::updateAndResample(): %d triggered guards:", /*grasp::to<Data>(dataPtr)->*/triggeredGuards.size());
	//for (std::vector<Configspace::Index>::const_iterator i = dataPtr->second.triggeredGuards.begin(); i != dataPtr->second.triggeredGuards.end(); ++i)
	//	context.write(" %d ", *i);
	//for (grasp::FTGuard::Seq::const_iterator i = /*grasp::to<Data>(dataPtr)->*/triggeredGuards.begin(); i != /*grasp::to<Data>(dataPtr)->*/triggeredGuards.end(); ++i)
	//	context.write(" %s ", i->str().c_str());
	//context.write("\n");

	//golem::Waypoint w(*robot->getController(), grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos);
	grasp::RealSeq force;
	force.assign(robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
//	printState(*grasp::to<Data>(dataPtr)->triggeredStates.begin(), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
	robot->readFT(robot->recvState().config, force);
	//std::cout << "spam:update&resample force <"; //context.write("force <");
	//for (grasp::RealSeq::const_iterator i = force.begin(); i != force.end(); ++i)
	//	std::cout << *i << " "; //context.write("%f ", *i);
	//std::cout << ">\n"; //context.write(">\n");

//	context.write("update hypothesis poses/n");
	// retrieve current object pose
	//Mat34 objectPose, modelFrameInv;
	//modelFrameInv.setInverse(modelFrame);
	//::Sleep(2000);
	//robot->getFrameObject(objectPose);
	//objectPose.multiply(objectPose, modelFrameInv);
	// render a priori
	//bool tmp = showMLEFrame;
	//showMLEFrame = true;
	//renderTrialData(dataPtr);
	//// move hypotheses
	//pBelief->createUpdate(pBelief->transform(objectPose));
	//// render a posteriori
	//::Sleep(2000);
	//renderTrialData(dataPtr);
	//::Sleep(2000);
	//showMLEFrame = tmp;
	
	// update samples' weights
	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
	golem::Waypoint w(*robot->getController(), robot->recvState().config.cpos/*grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos*/);
	pBelief->createUpdate(manipulator.get(), robot, w, /*grasp::to<Data>(dataPtr)->*/triggeredGuards, force);
	//for (grasp::RBPose::Sample::Seq::iterator sample = pBelief->getHypotheses().begin(); sample !=  pBelief->getHypotheses().end(); ++sample) {
	//	sample->weight = pHeuristic->evaluate(manipulator.get(), w, *sample, dataPtr->second.triggeredGuards, force, modelFrame);
	//	golem::kahanSum(norm, c, sample->weight);
	//}
//	context.write("normalising weights norm=%f (%.30f)\n", golem::REAL_ONE/norm, norm);
	// computes the normalised importance weight associated to each sample
	//dataPtr->second.normFac = norm = golem::REAL_ONE/norm;
	//for (grasp::RBPose::Sample::Seq::iterator sample = pBelief->getHypotheses().begin(); sample !=  pBelief->getHypotheses().end(); ++sample) 
	//	sample->weight *= norm;

	context.write("resampling (wheel algorithm)\n");
	// resampling (wheel algorithm)
	pBelief->createResample();
	
	//dataPtr->second.samples.clear();
	//dataPtr->second.samples.reserve(K);

	// update the query frame
	mlFrame = pBelief->createHypotheses(modelPoints, modelFrame);;
	grasp::to<Data>(dataPtr)->actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame = Mat34(mlFrame.q, mlFrame.p);
	//context.write("action frame poseData <%f %f %f> queryData <%f %f %f>\n", 
	//	grasp::to<Data>(dataPtr)->actionFrame.p.x, grasp::to<Data>(dataPtr)->actionFrame.p.y, grasp::to<Data>(dataPtr)->actionFrame.p.z,
	//	grasp::to<Data>(queryDataPtr)->actionFrame.p.x, grasp::to<Data>(queryDataPtr)->actionFrame.p.y, grasp::to<Data>(queryDataPtr)->actionFrame.p.z);

	// update query settings
	grasp::to<Data>(queryDataPtr)->queryFrame.multiply(grasp::to<Data>(queryDataPtr)->actionFrame, modelFrame);
	grasp::Cloud::transform(grasp::to<Data>(queryDataPtr)->actionFrame, modelPoints, grasp::to<Data>(queryDataPtr)->queryPoints);
	
	//dataPtr->second.samples.push_back(mlFrame);
	//for (size_t i = 1; i < K; ++i)
	//	dataPtr->second.samples.push_back(pBelief->sampleHypothesis());
	//showQueryDistrib = true;
	//renderData(dataPtr);
	//showQueryDistrib = false;
	showSamplePoints = true;
	if (screenCapture) universe.postScreenCaptureFrames(-1);
	::Sleep(50);
	renderUncertainty(pBelief->getSamples());
	renderData(dataPtr);
	::Sleep(100);
	if (screenCapture) universe.postScreenCaptureFrames(0);	
	size_t p = 1;
	std::cout << "density:\n";
	for (grasp::RBPose::Sample::Seq::const_iterator i = pBelief->getSamples().begin(); i != pBelief->getSamples().end(); ++i, ++p)
		std::cout << "pose " << p << " <" << i->p.x << ", " << i->p.y << ", " << i->p.z << "> w:" << i->weight << std::endl; 
	std::cout << "spam:update&resample 15\n";
	
	context.write("end\n");
}

//------------------------------------------------------------------------------

grasp::RobotState::List RagPlanner::make(const grasp::RobotState::List &action, const Mat34 &trn) const {
	// update approach and manip actions
	context.write("Update approach actions (size %d)...\n", action.size());
	grasp::RobotState::List stateList;
	if (!action.empty()) {
		for(grasp::RobotState::List::const_iterator i = action.begin(); i != action.end(); ++i) {
			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
				
			GenWorkspaceChainState gwcs;
			robot->getController()->chainForwardTransform(i->config.cpos, gwcs.wpos);
			gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
			gwcs.wpos[armChain].multiply(trn, gwcs.wpos[armChain]); // new waypoint frame

			GenWorkspaceChainState wend;
			wend.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
			wend.wpos[armChain] = gwcs.wpos[armChain];
			context.write("wend ,%f %f %f>\n", wend.wpos[armChain].p.x, wend.wpos[armChain].p.y, wend.wpos[armChain].p.z);
			// lock controller
			golem::CriticalSectionWrapper csw(robot->getControllerCS());
			// Find end position
			grasp::RobotState state = robot->recvState();
			if (!robot->getPlanner()->findTarget(state.config, wend, state.config))
				throw Message(Message::LEVEL_CRIT, "RagPlanner:Creating Model: unable to find grasp configuration");
			state.command = state.config;
			stateList.push_back(state);
			// error
			WorkspaceChainCoord wcc;
			robot->getController()->chainForwardTransform(state.config.cpos, wcc);
			wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
			context.write("wcc ,%f %f %f>\n", wcc[armChain].p.x, wcc[armChain].p.y, wcc[armChain].p.z);
			const grasp::RBDist error(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(wend.wpos[armChain]));
			context.write("RagPlanner:Creating Model: Pose error: lin=%.9f, ang=%.9f\n", error.lin, error.ang);
		}
	}
	return stateList;
}

//------------------------------------------------------------------------------

Mat34 RagPlanner::getGraspFrame(Data::Map::iterator dataPtr) {
	if (modelDataPtr._Ptr == NULL && queryDataPtr._Ptr == NULL)
		throw Message(Message::LEVEL_ERROR, "RagPlanner::getGraspFrame(): create model and query for the grasp.");

	if (grasp::to<Data>(modelDataPtr)->actionApproach.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::getGraspFrame(): At least a waypont in action approach.");

	const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();

	// grasp pose for an object sittin in the query pose. NOTE: the query pose must be the pose in which the grasp has been tought
	grasp::RobotState pose = *grasp::to<Data>(dataPtr)->actionApproach.begin();
//	handBounds.clear();
//	renderHand(pose.config);

	// grasp frame (tcp) in global coordinates
	GenWorkspaceChainState gwcs;
	gwcs.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
	robot->getController()->chainForwardTransform(pose.config.cpos, gwcs.wpos);
	//				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
	context.write("pose frame to teach pose (global) [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [0.0, 0.0, 0.0, 1.0]]\n", 
		gwcs.wpos[armChain].R.m11, gwcs.wpos[armChain].R.m12, gwcs.wpos[armChain].R.m13, gwcs.wpos[armChain].p.x, 
		gwcs.wpos[armChain].R.m21, gwcs.wpos[armChain].R.m22, gwcs.wpos[armChain].R.m23, gwcs.wpos[armChain].p.y, 
		gwcs.wpos[armChain].R.m31, gwcs.wpos[armChain].R.m32, gwcs.wpos[armChain].R.m33, gwcs.wpos[armChain].p.z);
//	r = gwcs.wpos[armChain];
	Mat34 poseFrameInv, graspFrame, graspFrameInv;
	poseFrameInv.setInverse(gwcs.wpos[armChain]);
	graspFrame.multiply(poseFrameInv, actionFrameT * modelFrame);
	graspFrameInv.setInverse(graspFrame);
	//				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
	gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
	r2 = gwcs.wpos[armChain];
	showTest = true;
	renderData(dataPtr);
	return gwcs.wpos[armChain];
}

void RagPlanner::createGrasp(Data::Map::iterator dataPtr) {
	if (queryDataPtr._Ptr == NULL)
		throw Message(Message::LEVEL_ERROR, "RagPlanner::createGrasp(): create a new query for the grasp.");
	
	if (grasp::to<Data>(dataPtr)->actionApproach.size() < 2)
		throw Message(Message::LEVEL_ERROR, "RagPlanner::createGrasp(): At least a waypont in action approach.");
				
	const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
	grasp::RobotState grasp = *grasp::to<Data>(dataPtr)->actionApproach.begin();
	printState(grasp.config, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end(), "createGrasp(): grasp");
	grasp::RobotState pregrasp = *(--grasp::to<Data>(dataPtr)->actionApproach.end());
	printState(pregrasp.config, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end(), "createGrasp(): pregrasp");
	renderHand(grasp.config);
	renderHand(pregrasp.config);

	Mat34 end/*(graspFrame)*/;
	end.multiply(grasp::to<Data>(dataPtr)->actionFrame, end);
	r = end;
	renderData(dataPtr);
	Robot::State state = robot->recvState();
	{
		// Setup target position
		const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
		GenWorkspaceChainState wend;
		wend.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
		wend.wpos[armChain] = end;
		// lock controller
		golem::CriticalSectionWrapper csw(robot->getControllerCS());
		// Find end position
		if (!robot->getPlanner()->findTarget(state.config, wend, state.config))
			throw Message(Message::LEVEL_CRIT, "Import::actionJustinPreGrasp(): unable to find grasp configuration");
		// error
		WorkspaceChainCoord wcc;
		robot->getController()->chainForwardTransform(state.config.cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
		const grasp::RBDist error(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(wend.wpos[armChain]));
		context.debug("Import::actionJustinPreGrasp(): Pose error: lin=%.9f, ang=%.9f\n", error.lin, error.ang);
	}

	//Mat34 frame;
	//frame.multiply(grasp::to<Data>(dataPtr)->actionFrame, graspFrame);
	//r = frame;
	//renderData(dataPtr);
	//context.write("RagPlanner::createGrasp(): action frame <%f %f %f>\n", grasp::to<Data>(dataPtr)->actionFrame.p.x, grasp::to<Data>(dataPtr)->actionFrame.p.y, grasp::to<Data>(dataPtr)->actionFrame.p.z);
	//grasp::RobotState pose = pregrasp;
	//{
	//	GenWorkspaceChainState wend;
	//	wend.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
	//	wend.wpos[armChain] = frame;
	//	context.write("grasp frame to model (global) [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [0.0, 0.0, 0.0, 1.0]]\n", 
	//		wend.wpos[armChain].R.m11, wend.wpos[armChain].R.m12, wend.wpos[armChain].R.m13, wend.wpos[armChain].p.x, 
	//		wend.wpos[armChain].R.m21, wend.wpos[armChain].R.m22, wend.wpos[armChain].R.m23, wend.wpos[armChain].p.y, 
	//		wend.wpos[armChain].R.m31, wend.wpos[armChain].R.m32, wend.wpos[armChain].R.m33, wend.wpos[armChain].p.z);
	//	// lock controller
	//	golem::CriticalSectionWrapper csw(robot->getControllerCS());
	//	if (!robot->getPlanner()->findTarget(pose.config, wend, pose.config))
	//		throw Message(Message::LEVEL_CRIT, "RagPlanner:Creating Model: unable to find grasp configuration");
	//	pose.command = pose.config;
	//	r2 = wend.wpos[armChain];
	//	renderData(dataPtr);
	//	//handBounds.clear();
	//	//renderHand(pose.config);
	//	// error
	//	WorkspaceChainCoord wcc;
	//	robot->getController()->chainForwardTransform(pose.config.cpos, wcc);
	////				wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
	//	context.write("New grasp frame to model (global) [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [0.0, 0.0, 0.0, 1.0]]\n", 
	//		wcc[armChain].R.m11, wcc[armChain].R.m12, wcc[armChain].R.m13, wcc[armChain].p.x, 
	//		wcc[armChain].R.m21, wcc[armChain].R.m22, wcc[armChain].R.m23, wcc[armChain].p.y, 
	//		wcc[armChain].R.m31, wcc[armChain].R.m32, wcc[armChain].R.m33, wcc[armChain].p.z);
	//	const grasp::RBDist error(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(wend.wpos[armChain]));
	//	context.write("RagPlanner:Creating Model: Pose error: lin=%.9f, ang=%.9f\n", error.lin, error.ang);
	//}
	//for (grasp::RobotState::List::iterator s = grasp::to<Data>(dataPtr)->actionApproach.begin(); s != grasp::to<Data>(dataPtr)->actionApproach.end(); ++s) {
	//	for (Configspace::Index i = robot->getStateArmInfo().getJoints().begin(); i != robot->getStateArmInfo().getJoints().begin(); ++i)
	//		s->config.cpos[i] = pose.config.cpos[i];
	//	s->command = s->config;
	//}

	grasp::to<Data>(dataPtr)->actionApproach.clear();
	for(size_t k = 0; k < 2 ; ++k) {
		for (Configspace::Index i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().begin(); ++i)
			state.config.cpos[i] = k == 0 ? grasp.config.cpos[i] : pregrasp.config.cpos[i];
		state.command = state.config;
		grasp::to<Data>(dataPtr)->actionApproach.push_back(state);
		renderHand(state.config, k == 0);
	}
}

//------------------------------------------------------------------------------

void RagPlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'S':
	{
		if (poseDataPtr._Ptr == NULL)
			PosePlanner::function(dataPtr, key);
		else {
			std::string index = poseDataPtr->first;
			readString("Enter trial data name: ", index);

//			TrialData trialData = poseDataPtr->second;
			const std::string path = grasp::makeString("%s%s.dat", data->dir.c_str(), index.c_str());
			// save to file
			context.write("Saving trial data to: %s\n", path.c_str());
			FileWriteStream fws(path.c_str());
//			fws << trialData;
			context.write("Done!\n");
		}
		return;
	}
	case 'L':
	{
		if (poseDataPtr._Ptr == NULL)
			PosePlanner::function(dataPtr, key);
		else {
			
//			std::string index = (poseDataPtr._Ptr != NULL) ? poseDataPtr->first : dataPtr->first;
//			readString("Enter trial data name: ", index);
//			const std::string path = grasp::makeString("%s%s.dat", data->dir.c_str(), index.c_str());
//			context.write("Loading trial data from: %s\n", path.c_str());
//			// load from file
//			TrialData trialData(*robot->getController());
//			FileReadStream frs(path.c_str());
//			frs >> trialData;
//			getData().erase(index); // delete if exists
//			scene.getOpenGL((poseDataPtr._Ptr != NULL) ? poseDataPtr->second.openGL : grasp::to<Data>(dataPtr)->openGL);
//			poseDataPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(index, trialData)); // insert new
//			scene.getOpenGL(poseDataPtr->second.openGL);
//			showSampleColour = false;
//			showSampleFrame = false;
//			showSamplePoints = true;
//			showMLEFrame = true;
//			showMLEPoints = false;
////			pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
////			pHeuristic->setBelief(pBelief);
//			pHeuristic->enableUnc = uncEnable;			
//			renderTrialData(poseDataPtr);
//			context.write("Done!\n");
		}
		return;
	}
	case 'G':
	{
		context.write("Load object ground truth pose\n");
		grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
		robot->objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(points->second)); 
		return;
	}
	case 'R':
	{
		if (poseDataPtr._Ptr == NULL) {
			context.write("Unable to reset.\n");
			return;
		}
		pBelief->reset();
		//poseDataPtr->second.samples.clear();
		//for (grasp::RBPose::Sample::Seq::const_iterator i = poseDataPtr->second.init.begin(); i != poseDataPtr->second.init.end(); ++i)
		//	poseDataPtr->second.samples.push_back(*i);
		showSampleColour = false;
		showSampleFrame = false;
		showSamplePoints = true;
		showMLEFrame = true;
		showMLEPoints = false;
//		pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
//		pHeuristic->setBelief(pBelief);
		pHeuristic->enableUnc = uncEnable;	
		grasp::to<Data>(dataPtr)->actionFrame = initActionFrame;
		mlFrame = grasp::RBPose::Sample(grasp::to<Data>(dataPtr)->actionFrame);
		grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->actionFrame, modelFrame);	
		grasp::Cloud::transform(grasp::to<Data>(dataPtr)->actionFrame, grasp::to<Data>(dataPtr)->queryPoints, grasp::to<Data>(dataPtr)->queryPoints);
		showModelPoints = false;
		showModelFeatures = false;
		showQueryDistrib = true;
		showDistribution = false;
		renderData(dataPtr);
		renderTrialData(poseDataPtr);
		return;
	}
	case 'B':
	{
		context.debug("testing active controller.\n");
		robot->enableGuards();
		Controller::State::Seq trj;
		robot->createTrajectory(robot->recvState().config, NULL, &poseSeq[2], 0, trj);
		
		grasp::to<Data>(dataPtr)->action.clear();
		//grasp::to<Data>(poseDataPtr)->action.clear();
		profile(dataPtr, trj, trjApproachDuration, trjApproachIdle);
		perform(dataPtr);
		// retrieve robot pose from real justin
		// print current robot joint position
		//Controller::State state = robot->recvState().config;
		//std::stringstream str;
		//for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
		//	str << " c" << (*i - *state.getInfo().getJoints().begin() + 1) << "=\"" << state.cpos[i] << "\"";
		//context.write("<pose dim=\"%d\"%s/>\n", state.getInfo().getJoints().size(), str.str().c_str());
		//golem::WorkspaceJointCoord wc;
		//robot->getController()->jointForwardTransform(state.cpos, wc);
		//for (golem::Chainspace::Index i = robot->getStateInfo().getChains().begin(); i < robot->getStateInfo().getChains().end(); ++i) {
		//	context.write("Name: %s\n", robot->getController()->getChains()[i]->getName().c_str());
		//	for (golem::Configspace::Index j = robot->getStateInfo().getJoints(i).begin(); j < robot->getStateInfo().getJoints(i).end(); ++j) {
		//		const U32 k = U32(j - robot->getStateInfo().getJoints(i).begin());
		//		const Mat34& m = wc[j];
		//		context.write("Joint %d: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", k, m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
		//	}
		//}
		break;
		//if (poseDataPtr._Ptr == NULL) {
		//	context.write("Unable to find the withdraw trajectory.\n");
		//	return;
		//}
		//performWithdraw(poseDataPtr);
		//return;
		//if (modelPoints.empty() || queryDataPtr == getData().end() || grasp::to<Data>(queryDataPtr)->queryPoints.empty()) {
		//	context.write("Unable to find a model or query. Please make sure you have generated a model and query.\n");
		//	return;
		//}
		//context.write("Learning observational model\n");
		//Controller::State::Seq seq;
		//extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
		//// compute the approach traectory in a new frame and its profile
		//grasp::to<Data>(dataPtr)->action.clear();
		//grasp::to<Data>(queryDataPtr)->action.clear();
		//PosePlanner::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
		//// copy the planned trajectory in my trial data
		//grasp::to<Data>(poseDataPtr)->action.clear();
		//const Controller::State pregrasp = *grasp::to<Data>(dataPtr)->action.begin();
		//for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
		//	grasp::to<Data>(poseDataPtr)->action.push_back(*i);
		//
		//golem::Controller::State::Seq initTrajectory;
		//robot->createTrajectory(robot->recvState().command, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, initTrajectory);
		//robot->enableGuards(false);
		//robot->sendTrajectory(initTrajectory, true);
		//robot->waitForEnd();

		//Controller::State state = pregrasp;
		//const Chainspace::Index i = robot->getStateHandInfo().getChains().begin() + 1;
		//return;
	}
	case 'X':
	{
		if (modelPoints.empty() || pBelief->getSamples().empty()) {
			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
			return;
		}
		// update pose settings
		poseDataPtr = dataPtr;
		grasp::to<Data>(dataPtr)->actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame;
		grasp::to<Data>(dataPtr)->queryFrame = grasp::to<Data>(queryDataPtr)->queryFrame;
		grasp::to<Data>(dataPtr)->actionApproach = grasp::to<Data>(queryDataPtr)->actionApproach;
		grasp::to<Data>(dataPtr)->actionManip = grasp::to<Data>(queryDataPtr)->actionManip;
		grasp::to<Data>(dataPtr)->action = grasp::to<Data>(queryDataPtr)->action;
		grasp::to<Data>(dataPtr)->queryPoints = grasp::to<Data>(queryDataPtr)->queryPoints;

		grasp::to<Data>(dataPtr)->actionWithdraw.clear();
		const grasp::RobotState homeState = robot->recvState();
//		grasp::to<Data>(dataPtr)->actionWithdraw.push_back(robot->recvState());
//		grasp::to<Data>(dataPtr)->actionWithdraw.push_back(robot->recvState());

		grasp::to<Data>(dataPtr)->replanning = false;

		if (grasp::to<Data>(dataPtr)->actionApproach.empty())
			context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");

		robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
		robot->setCollisionDetection(true);

AGAIN:
		context.debug("RagPlanner:function(X): performing approach trajectory\n");
		try {
			if (!singleGrasp) {
				performApproach(dataPtr);
			}
			else {
				context.write("Single grasp performs\n");
				performSingleGrasp(dataPtr);
//				goto GRASP_QUALITY;
			}
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
		}
		context.write("perform approach is over.\n");
		
		if (grasp::to<Data>(dataPtr)->replanning) {
			//if (!executedTrajectory.empty()) {
			//	context.write("Computing withdrawing pose.\n");
			//	Real distMin(REAL_MAX);
			//	Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
			//	// skip the last element of the executed trajectory which is supposed to be the final state
			//	for (Controller::State::Seq::const_iterator a = executedTrajectory.begin(), b = executedTrajectory.begin() + 1; b != executedTrajectory.end() - 1; ++a, ++b) {
			//		Real dist(REAL_ZERO);
			//		for (Configspace::Index i = robot->getStateInfo().getJoints().begin(); i != robot->getStateInfo().getJoints().end(); ++i)				
			//			dist += Math::sqrt(Math::sqr(robotConfig.cpos[i] - (a->cpos[i] + (a->cpos[i] - b->cpos[i])/2))); //Math::sqrt(Math::sqr(robotConfig.cpos[i] - a->cpos[i]));
			//		if (dist/* = Math::sqrt(dist)*/ < distMin) {
			//			distMin = dist;
			//			waypoint = *a;
			//		}
			//	}
			//	grasp::RobotState w(*robot->getController());
			//	w.command = w.config = waypoint;
			//	while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2)
			//		grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(w);
			//}
			//else {
			//	context.write("Vanilla withdraw (home pose).\n");
			//	grasp::to<Data>(dataPtr)->actionWithdraw.push_back(homeState);
			//	grasp::to<Data>(dataPtr)->actionWithdraw.push_back(homeState);
			//}
			//performWithdraw(dataPtr);
			const int key = waitKey("YN", "Do you want to repeat? (Y/N)...");
			if (key == 'Y') {
				goto AGAIN;
			}
			else if (key == 'N')
				return;
		}
		else
			performManip(dataPtr);
		return;
		


		//grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
		//// query create
		//context.write("Creating %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
		//pBelief->createQuery(points->second);
		//// create hypotheses and return the action frame for this query
		//actionFrameT = pBelief->maximum().toMat34();
		//context.write("action frame TEACH <%f %f %f>\n", actionFrameT.p.x, actionFrameT.p.y, actionFrameT.p.z);

		//try {
		//	graspFrame = getGraspFrame(dataPtr);
		//}
		//catch (const Message& msg) {
		//	context.write("%s\n", msg.str().c_str());
		//}		
		return;
	}
	case 'U':
	{
		if (modelPoints.empty() || pBelief->getSamples().empty()) {
			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
			return;
		}
		// update pose settings
		poseDataPtr = dataPtr;
		grasp::to<Data>(poseDataPtr)->actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame;
		grasp::to<Data>(poseDataPtr)->queryFrame = grasp::to<Data>(queryDataPtr)->queryFrame;
		grasp::to<Data>(poseDataPtr)->actionApproach = grasp::to<Data>(queryDataPtr)->actionApproach;
		grasp::to<Data>(poseDataPtr)->actionManip = grasp::to<Data>(queryDataPtr)->actionManip;
		grasp::to<Data>(poseDataPtr)->action = grasp::to<Data>(queryDataPtr)->action;
		grasp::to<Data>(poseDataPtr)->queryPoints = grasp::to<Data>(queryDataPtr)->queryPoints;
		//grasp::to<Data>(dataPtr)->queryPoints.clear();
		//grasp::to<Data>(dataPtr)->queryPoints.reserve(grasp::to<Data>(queryDataPtr)->queryPoints.size());
		//std::for_each(grasp::to<Data>(queryDataPtr)->queryPoints.begin(), grasp::to<Data>(queryDataPtr)->queryPoints.end(), [=] (grasp::Cloud::PointSeq::const_iterator p) {
		//	grasp::to<Data>(dataPtr)->queryPoints.push_back(*p);
		//});

		// set home pose
		//grasp::to<Data>(poseDataPtr)->homeStates.clear();
		//std::cout << "spam:function: 7 bis\n";
		//grasp::to<Data>(poseDataPtr)->homeStates.reserve(1);
		//std::cout << "spam:function: 8 riceiving state\n";
		//grasp::RobotState homeState = robot->recvState();
		//std::cout << "spam:function: 8 bis\n";
		//grasp::to<Data>(poseDataPtr)->homeStates.push_back(homeState);
		homeStates.clear();
		homeStates.push_back(robot->recvState());

		bool conclude = false;
		grasp::to<Data>(poseDataPtr)->replanning = false;
		grasp::to<Data>(poseDataPtr)->release = false;
		switch (waitKey("PE", "Press a key to (P)lan a reach-and-grasp trajectory, (E)nable/Disable planning with uncertainty...")) {
		case 'P':
		{			
			if (grasp::to<Data>(poseDataPtr)->actionApproach.empty())
				context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");

			// approach action is the entire trajectory taught to the robot
			// NOTE: states are memorised from the last (pre grasp = begin()) to the first (initial pose = end())
			// Generate a new approach action formed by [pregrasp, corrent conf of the robot]
			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
			GenWorkspaceChainState gwcs;
			gwcs.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
			grasp::RobotState grasp = *grasp::to<Data>(poseDataPtr)->actionApproach.begin();
			robot->getController()->chainForwardTransform(grasp.config.cpos, gwcs.wpos);
			Mat34 poseFrameInv, graspFrame, graspFrameInv;
			poseFrameInv.setInverse(gwcs.wpos[armChain]);
			graspFrame.multiply(poseFrameInv, modelFrame/*grasp::to<Data>(poseDataPtr)->queryFrame*/);
			graspFrameInv.setInverse(graspFrame);
//				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
			graspFrameM.multiply(modelFrame, graspFrameInv);
			renderHand(grasp.config, true);

			grasp::RobotState pregrasp = *(--grasp::to<Data>(poseDataPtr)->actionApproach.end());			
			robot->getController()->chainForwardTransform(pregrasp.config.cpos, gwcs.wpos);
			Mat34 preposeFrameInv, pregraspFrame, pregraspFrameInv;
			preposeFrameInv.setInverse(gwcs.wpos[armChain]);
			pregraspFrame.multiply(preposeFrameInv, modelFrame/*grasp::to<Data>(poseDataPtr)->queryFrame*/);
			pregraspFrameInv.setInverse(pregraspFrame);
//				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
			pregraspFrameM.multiply(modelFrame, pregraspFrameInv);
			renderHand(pregrasp.config);
			showTest = false;
			renderData(dataPtr);

			trjApproachExtrapolFac = REAL_ZERO;
			robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
			robot->setCollisionDetection(true);		
//			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
			iterations = 1;

REPLAN_TEST:
			try {
				if (!singleGrasp) {
					testTransformation(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, grasp::to<Data>(dataPtr)->actionApproach);
					performApproach(poseDataPtr);
					context.write("perform approach is over.\n");
				}
				else {
					context.write("Single grasp performs\n");
					performSingleGrasp(poseDataPtr);
					goto GRASP_QUALITY;
				}
			}
			catch (const Message& msg) {
				context.write("%s\n", msg.str().c_str());
			}
			
			std::cout << "check for computing the grasp quality.\n";
			if (grasp::to<Data>(poseDataPtr)->triggered > 0 && !grasp::to<Data>(poseDataPtr)->replanning) {
				// todo: compute if grasped
GRASP_QUALITY:
				std::cout << "grasp quality: step 1\n";
				context.write("Check the grasp quality (triggered=%s, replanning=%s)\n", grasp::to<Data>(poseDataPtr)->triggered > 0 ? "true" : "false", grasp::to<Data>(poseDataPtr)->replanning ? "true" : "false"); 

				std::cout << "grasp quality: step 2\n";
				grasp::RealSeq force;
				//Real graspQuality = REAL_ZERO;
				//try {
				//	graspQuality = robot->analyseGrasp(force);
				//}
				//catch (const golem::Message &msg) {
				//	context.write("%s\n", msg.str().c_str());
				//}
				//std::cout << "grasp quality: step 3\n";
				//replanning = !(graspQuality > 0);
				//context.write("Grasp %s. quality = %f\n", !replanning ? "succeess (no replanning)" : "failure (replanning)", graspQuality);
				//if (!replanning) {
					const int key = waitKey("YN", "Do you want to exit? (Y/N)...");
					if (key == 'Y') {
						performManip(poseDataPtr);
						return;
						conclude = true;
					} 
					else {
						const int key = waitKey("YN", "Do you want to replan a trajectory? (Y/N)...");
						if (key == 'Y')
							grasp::to<Data>(poseDataPtr)->replanning = true;
						else return;
					}
				goto MOVING_BACK;
			}
			else {
MOVING_BACK:
				trnEnable = false;
				pHeuristic->enableUnc = false;
				grasp::to<Data>(poseDataPtr)->actionWithdraw.clear();
				// if replanning is not required the robot is moved back to the home pose
				if (!grasp::to<Data>(poseDataPtr)->replanning || withdrawToHomePose) {
//					if (!/*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.empty()) {
						// set as first waypont the pregrasp pose or the home pose
						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(grasp::to<Data>(poseDataPtr)->release && !grasp::to<Data>(poseDataPtr)->actionApproach.empty() ? *(--grasp::to<Data>(poseDataPtr)->actionApproach.end()) : homeStates.front());
						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(homeStates.front());
//					}
					grasp::to<Data>(poseDataPtr)->release = false;
					//while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2 && !grasp::to<Data>(poseDataPtr)->executedTrajectory.empty())
					//	grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(grasp::to<Data>(poseDataPtr)->homeStates.front());
				// otherwise the previous waypoint in the trajectory is withdrawn
				} else {
					Real distMin(REAL_MAX);
					Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
					for (Controller::State::Seq::const_iterator a = /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.begin(), b = /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.begin() + 1; b != /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.end(); ++a, ++b) {
						Real dist(REAL_ZERO);
						for (Configspace::Index i = robot->getStateInfo().getJoints().begin(); i != robot->getStateInfo().getJoints().end(); ++i)				
							dist += Math::sqrt(Math::sqr(robotConfig.cpos[i] - (a->cpos[i] + (a->cpos[i] - b->cpos[i])/2))); //Math::sqrt(Math::sqr(robotConfig.cpos[i] - a->cpos[i]));
						if (dist/* = Math::sqrt(dist)*/ < distMin) {
							distMin = dist;
							waypoint = *a;
						}
					}
					grasp::RobotState w(*robot->getController());
					w.command = waypoint; w.config = waypoint;
					while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2)
						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(w);
				}
//				renderTrialData(poseDataPtr);
				// move back to home pose
				performWithdraw(poseDataPtr);
				goto REPLAN_TEST;
			}
			return;
		}
		case 'G':
		{
			golem::Waypoint w(*robot->getController(), robot->recvState().command.cpos);
			Mat34 tcpFrameInv, trnFromTcpToObj, tcpFrame = w.wpos[robot->getStateInfo().getChains().begin()];
			tcpFrameInv.setInverse(tcpFrame);
			trnFromTcpToObj.multiply(tcpFrameInv, grasp::to<Data>(dataPtr)->queryFrame);
			context.write("tcpFrame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n", 
				tcpFrame.R.m11, tcpFrame.R.m12, tcpFrame.R.m13, tcpFrame.p.x, 
				tcpFrame.R.m21, tcpFrame.R.m22, tcpFrame.R.m23, tcpFrame.p.y, 
				tcpFrame.R.m31, tcpFrame.R.m32, tcpFrame.R.m33, tcpFrame.p.z);
			context.write("obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n", 
				grasp::to<Data>(dataPtr)->queryFrame.R.m11, grasp::to<Data>(dataPtr)->queryFrame.R.m12, grasp::to<Data>(dataPtr)->queryFrame.R.m13, grasp::to<Data>(dataPtr)->queryFrame.p.x, 
				grasp::to<Data>(dataPtr)->queryFrame.R.m21, grasp::to<Data>(dataPtr)->queryFrame.R.m22, grasp::to<Data>(dataPtr)->queryFrame.R.m23, grasp::to<Data>(dataPtr)->queryFrame.p.y, 
				grasp::to<Data>(dataPtr)->queryFrame.R.m31, grasp::to<Data>(dataPtr)->queryFrame.R.m32, grasp::to<Data>(dataPtr)->queryFrame.R.m33, grasp::to<Data>(dataPtr)->queryFrame.p.z);
			context.write("Trn from tcp frame to obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
				trnFromTcpToObj.R.m11, trnFromTcpToObj.R.m12, trnFromTcpToObj.R.m13, trnFromTcpToObj.p.x,
				trnFromTcpToObj.R.m21, trnFromTcpToObj.R.m22, trnFromTcpToObj.R.m23, trnFromTcpToObj.p.y,
				trnFromTcpToObj.R.m31, trnFromTcpToObj.R.m32, trnFromTcpToObj.R.m33, trnFromTcpToObj.p.z);
			pBelief->setInitObjPose(tcpFrame);
			Mat34 trn = pBelief->transform(grasp::to<Data>(dataPtr)->queryFrame);
			context.write("BELIEF Trn from tcp frame to obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
				trn.R.m11, trn.R.m12, trn.R.m13, trn.p.x,
				trn.R.m21, trn.R.m22, trn.R.m23, trn.p.y,
				trn.R.m31, trn.R.m32, trn.R.m33, trn.p.z);
			trn.multiply(tcpFrame, trnFromTcpToObj);
			context.write("BELIEF obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
				trn.R.m11, trn.R.m12, trn.R.m13, trn.p.x,
				trn.R.m21, trn.R.m22, trn.R.m23, trn.p.y,
				trn.R.m31, trn.R.m32, trn.R.m33, trn.p.z);

			Mat34 m = robot->getController()->getGlobalPose();
			context.write("Robot global pose\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n det:%5.7f\n",
				m.R.m11, m.R.m12, m.R.m13, m.p.x,
				m.R.m21, m.R.m22, m.R.m23, m.p.y,
				m.R.m31, m.R.m32, m.R.m33, m.p.z,
				m.R.determinant());

			Real distMax = golem::REAL_ZERO;
			for (grasp::Cloud::PointSeq::const_iterator p = modelPoints.begin(); p != modelPoints.end(); ++p) {
				// CHECK max magnitude of absolute coordinates of points is related to the object coords but NOT its size!
				//const Real d = p->frame.p.magnitude();
				const Real d = grasp::Cloud::getPoint(*p).magnitude();
				if (d > distMax)
					distMax = d;
			}
			context.write("Size of the object %5.7f\n", distMax);
			// go to to predefined pose
			//context.write("Goto Pose test\n");
			//(void)gotoPose();
			//std::vector<Configspace::Index> triggeredGuards;
			//if (robot->getTriggeredGuards(triggeredGuards)) {
			//	context.write("RagPlanner::Test triggered guards on Justin");
			//	for (std::vector<Configspace::Index>::const_iterator i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i)
			//		context.write(" %d", *i);
			//	context.write("\n");
			//}
			return;
		}
		case 'T':
		{
			switch (waitKey("OU", "Press a key to test (O)bservations or (U)pdate..")) {
			case 'O':
			{			
				context.write("test observation\n");
				Mat34 pose;
				pose.R.setId();
				pose.R.rotX(-REAL_HALF*REAL_PI);
				pose.p = Vec3(-0.1, 0.40, 0.1);
				for (Real i = 0; i < 60; ++i) {
					pose.p.x += 0.001*i;
					pHeuristic->testObservations(pose, true);
					renderPose(pose);
					::Sleep(1000);
				}
				return;
			}
			case 'U':
			{
				context.write("test update\n");
				grasp::RBPose::Sample::Seq samples;
				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.80, -0.36, 0.01), Quat())));
				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.65, -0.36, 0.01), Quat())));
				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.50, -0.36, 0.01), Quat())));
				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.30, -0.36, 0.01), Quat())));
				Mat34 pose;
				pose.R.setId();
				pose.R.rotX(REAL_HALF*REAL_PI);
				pose.p = Vec3(0.4, -0.45, 0.1);
				for (size_t i = 0; i < 20; ++i) {
					pose.p.x += 0.01*i;
					context.write("Iteration n.%d pose <%f %f %f>\n", i + 1, pose.p.x, pose.p.y, pose.p.z);
					//for (grasp::RBPose::Sample::Seq::iterator j = samples.begin(); j != samples.end(); ++j)
					//	j->weight = pHeuristic->evaluate(grasp::RBCoord(pose), *j, -0.5, modelFrame);
					context.write("\n");
					renderUpdate(pose, samples);
					::Sleep(1000);
				}
				return;
			}
			}
		}
		case 'E':
			uncEnable = !uncEnable;
			context.write("Planning with uncertainty %s\n", uncEnable ? "ON" : "OFF");
			return;
		}
		break;
	}
	//case 'T':
	//{
	//	switch (waitKey("OU", "Press a key to test (O)bservations or (U)pdate..")) {
	//	case 'O':
	//	{			
	//		context.write("test observation\n");
	//		Mat34 pose;
	//		pose.R.setId();
	//		pose.R.rotX(-REAL_HALF*REAL_PI);
	//		pose.p = Vec3(-0.4, 0.70, 0.15);
	//		for (Real i = 0; i < 60; ++i) {
	//			pose.p.x += 0.001*i;
	//			pHeuristic->testObservations(pose, true);
	//			renderPose(pose);
	//			::Sleep(1000);
	//		}
	//		return;
	//	}
	//	case 'U':
	//	{
	//		context.write("test update\n");
	//		grasp::RBPose::Sample::Seq samples;
	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.80, -0.36, 0.01), Quat())));
	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.65, -0.36, 0.01), Quat())));
	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.50, -0.36, 0.01), Quat())));
	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.30, -0.36, 0.01), Quat())));
	//		Mat34 pose;
	//		pose.R.setId();
	//		pose.R.rotX(REAL_HALF*REAL_PI);
	//		pose.p = Vec3(0.4, -0.45, 0.1);
	//		for (size_t i = 0; i < 20; ++i) {
	//			pose.p.x += 0.01*i;
	//			context.write("Iteration n.%d pose <%f %f %f>\n", i + 1, pose.p.x, pose.p.y, pose.p.z);
	//			//for (grasp::RBPose::Sample::Seq::iterator j = samples.begin(); j != samples.end(); ++j)
	//			//	j->weight = pHeuristic->evaluate(grasp::RBCoord(pose), *j, -0.5, modelFrame);
	//			context.write("\n");
	//			renderUpdate(pose, samples);
	//			::Sleep(1000);
	//		}
	//		return;
	//	}
	//	}
	//}
	case ',':
	{
		handBounds.clear();
		return;
	}
	case '.':
	{
		renderHand(grasp::to<Data>(dataPtr)->actionApproach[grasp::to<Data>(dataPtr)->actionApproach.size()-1].config);
		return;
//			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
//			GenWorkspaceChainState gwcs;
//			robot->getController()->chainForwardTransform(robot->recvState().config.cpos, gwcs.wpos);
////			gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
//			renderPose(gwcs.wpos[armChain]);
//			grasp::RBCoord b(gwcs.wpos[armChain]);
//			context.write("render pose triggered state <%f %f %f> <%f %f %f %f>\n", b.p.x, b.p.y, b.p.z, b.q.w, b.q.x, b.q.y, b.q.z);
//			{		
//				golem::CriticalSectionWrapper csw(csDataRenderer);
//				const Bounds::Seq bounds = manipulator->getBounds(manipulator->getPose(robot->recvState().config));
//				handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
//			}	
//			return;
	}
	case '0':
	{
		Controller::State state = robot->getController()->createState();
		robot->getController()->lookupState(SEC_TM_REAL_MAX, state);
		printState(state, robot->getController()->getStateInfo().getJoints().begin(), robot->getController()->getStateInfo().getJoints().end(), "Current robot pose", true);
		return;
	}
	case '_':
		scene.getOpenGL(grasp::to<Data>(poseDataPtr)->openGL);
		if (poseDataPtr == dataMap.begin()) poseDataPtr = --dataMap.end(); else --poseDataPtr;
		scene.setOpenGL(grasp::to<Data>(poseDataPtr)->openGL);
		context.write("Trial data name: %s\n", poseDataPtr->first.c_str());
		renderTrialData(poseDataPtr);
		return;
	case '=':
		scene.getOpenGL(grasp::to<Data>(poseDataPtr)->openGL);
		if (++poseDataPtr == dataMap.end()) poseDataPtr = dataMap.begin();
		scene.setOpenGL(grasp::to<Data>(poseDataPtr)->openGL);
		context.write("Trial data name: %s\n", poseDataPtr->first.c_str());
		renderTrialData(poseDataPtr);
		return;
	case '!':
		singleGrasp = !singleGrasp;
		context.write("Single grasp attempt %s\n", singleGrasp ? "ON" : "OFF");
		return;
	} // switch
	PosePlanner::function(dataPtr, key);
}

//------------------------------------------------------------------------------

void RagPlanner::printTrajectory(const golem::Controller::State::Seq &trajectory, const golem::Configspace::Index &begin, const golem::Configspace::Index &end) const {
	context.write("RagPlanner::printTrajectory():\n");
	for (Controller::State::Seq::const_iterator j = trajectory.begin(); j != trajectory.end(); ++j)
		printState(*j, begin, end);
}

void RagPlanner::printState(const golem::Controller::State &state, const golem::Configspace::Index &begin, const golem::Configspace::Index &end, const std::string &label, const bool readForce) const {
	context.write("%s cpos <", label.c_str()); //std::cout << label.c_str() << " cpos <"; //context.write("%s cpos <", label.c_str());
	for (Configspace::Index i = begin; i != end; ++i) {
		context.write("%f ", state.cpos[i]); //std::cout << state.cpos[i] << " "; //context.write("%f ", state.cpos[i]);
	}
	context.write(">\n"); //std::cout << ">\n"; //context.write(">\n");
	
	if (!readForce) 
		return;

	grasp::RealSeq forces;
	forces.assign(robot->getStateInfo().getJoints().size(), REAL_ZERO);
	robot->readFT(state, forces);
	const Configspace::Index k = robot->getStateInfo().getJoints().begin();
	std::cout << label.c_str() << " torques <"; //context.write("%s torques <", label.c_str());
	for (Configspace::Index i = begin; i != end; ++i) {
		const size_t idx = k - begin;
		std::cout << forces[idx] << " "; //context.write("%f ", forces[idx]);
	}
	std::cout << ">\n"; //context.write(">\n");

}

void RagPlanner::testTransformation(const golem::Mat34 &teachFrame, const golem::Mat34 &modelFrame, const golem::Mat34 &queryFrame, const grasp::RobotState::List &action) {
	const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
	context.write("RagPlanner::testTransformation(): approaching trajetory (list)\n");
	printTrajectory(makeCommand(action), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
	for (grasp::RobotState::List::const_iterator i = action.begin(); i != action.end(); ++i) {
		GenWorkspaceChainState gwcs;
		robot->getController()->chainForwardTransform(i->config.cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
		// define the grasp frame
		Mat34 poseFrameInv, graspFrame, graspFrameInv;
		poseFrameInv.setInverse(gwcs.wpos[armChain]);
		graspFrame.multiply(poseFrameInv, teachFrame * modelFrame);
		graspFrameInv.setInverse(graspFrame);
		gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
		context.write("trnTrajectory(): grasp frame at model <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
		gwcs.wpos[armChain].multiply(queryFrame, gwcs.wpos[armChain]); // new waypoint frame
		context.write("trnTrajectory(): grasp frame at new query <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
		
		if (i == action.begin())
			graspFrameQ = gwcs.wpos[armChain];
		else
		pregraspFrameQ = gwcs.wpos[armChain];
	}
	handBounds.clear();
	renderData(currentDataPtr);
}


//------------------------------------------------------------------------------

void spam::XMLData(RagPlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	// load PosePlanner description
	spam::XMLData((PosePlanner::Desc&)val, context, xmlcontext, create);

	// Load spam::Robot
	spam::XMLData((Robot::Desc&)*val.robotDesc, context, xmlcontext, create);

	// Load RagPlanner
	xmlcontext = xmlcontext->getContextFirst("rag_planner");

	golem::XMLData("num_sampled_poses", val.K, xmlcontext);
	golem::XMLData("imp_weights_threshold", val.theta, xmlcontext);
	golem::XMLData("max_sampling_iterations", val.maxSamplingIter, xmlcontext);
	golem::XMLData("timestamp_factor", val.timestampFac, xmlcontext);
	golem::XMLData("planning_uncertainty", val.uncEnable, xmlcontext);
	golem::XMLData("single_grasp_attempt", val.singleGrasp, xmlcontext);
	golem::XMLData("withdraw_to_home_pose", val.withdrawToHomePose, xmlcontext);
	val.sampleAppearance.xmlData(xmlcontext->getContextFirst("point_appearance"), create);
	try {
		golem::XMLData(&val.trnModelPointCloud[0], &val.trnModelPointCloud[grasp::RBCoord::N], "c", xmlcontext->getContextFirst("model_points_trn"), create);
		XMLData(val.objectBounds, val.objectBounds.max_size(), xmlcontext->getContextFirst("object_bounding_box"), "bounds", create);
		XMLData(val.obstacleBounds, val.obstacleBounds.max_size(), xmlcontext->getContextFirst("obstacles"), "bounds", create);
	} catch (const golem::MsgXMLParser& msg) {}
	golem::XMLData("enable_noise", val.gtNoiseEnable, xmlcontext->getContextFirst("point_cloud_noise"), create);
	grasp::XMLData(val.gtPoseStddev, xmlcontext->getContextFirst("point_cloud_noise gt_pose_stddev"), create);

	Belief::Desc* pBeliefDesc(new Belief::Desc);
	(grasp::RBPose::Desc&)*pBeliefDesc = *((PosePlanner::Desc&)val).pRBPoseDesc;
	val.pRBPoseDesc.reset(pBeliefDesc);
	spam::XMLData((Belief::Desc&)*val.pRBPoseDesc, xmlcontext, create);
}

//------------------------------------------------------------------------------

void RagPlannerApp::run(int argc, char *argv[]) {
	// Setup PosePlanner
	RagPlanner::Desc ragHandDesc;
	XMLData(ragHandDesc, context(), xmlcontext());

	RagPlanner *pRagPlanner = dynamic_cast<RagPlanner*>(scene()->createObject(ragHandDesc)); // throws
	if (pRagPlanner == NULL)
		throw Message(Message::LEVEL_CRIT, "PosePlannerApp::run(): Unable to cast to PosePlanner");

	// Random number generator seed
	context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);
	try {
		pRagPlanner->main();
	}
	catch (const grasp::Interrupted&) {
	}
	
	context()->info("Good bye!\n");
	scene()->releaseObject(*pRagPlanner);
};

//------------------------------------------------------------------------------

#ifdef _SPAM_RAG_MAIN_
int main(int argc, char *argv[]) {
	return spam::RagPlannerApp().main(argc, argv);
}
#endif // _GRASP_PosePlanner_MAIN_