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

RagPlanner::RagPlanner(Scene &scene) : grasp::PosePlanner(scene), pBelief(nullptr), pHeuristic(nullptr) {
}

RagPlanner::~RagPlanner() {
}

bool RagPlanner::create(const Desc& desc) {
	grasp::PosePlanner::create(desc); // throws
	
	robot = dynamic_cast<Robot*>(grasp::PosePlanner::robot);
	if (robot == NULL)
		throw Message(Message::LEVEL_CRIT, "Player::create(): unable to cast to Robot");

	Mat34 objectPose;
	objectPose = desc.objectPose;
	robot->getFrameObject(objectPose);
	objectRealPose = grasp::RBCoord(objectPose);
	context.debug("Object real pose <%f %f %f> <%f %f %f %f>", 
		objectRealPose.p.x, objectRealPose.p.y, objectRealPose.p.z, objectRealPose.q.w, objectRealPose.q.x, objectRealPose.q.y, objectRealPose.q.z);
	iterations = 0;

	pBelief = dynamic_cast<Belief*>(pRBPose.get());
	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&robot->getPlanner()->getHeuristic());
	uncEnable = desc.uncEnable;
	singleGrasp = desc.singleGrasp;
	withdrawToHomePose = desc.withdrawToHomePose;
	posterior = true;

	K = desc.K;
	theta = desc.theta;
	sampleAppearance = desc.sampleAppearance;
	timestampFac = desc.timestampFac;
	maxSamplingIter = desc.maxSamplingIter;

	objectBounds.reset(new Bounds::Desc::Seq());
	objectBounds->insert(objectBounds->begin(), desc.objectBounds.begin(), desc.objectBounds.end());

	// actionFrame should be the identity if no query have been done
	//actionFrame.setId();

	manipulator.reset(new grasp::Manipulator(robot->getController()));
	handBounds.clear();

	trnEnable = true;
	ragDesc = desc;

	return true;
}

void RagPlanner::render() {
	grasp::PosePlanner::render();
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		sampleRenderer.render();
//		gtRenderer.render();
		testPose.render();
		testUpdate.render();
		handRenderer.setWireColour(RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(255)));
		handRenderer.setLineWidth(Real(2.0));
		handRenderer.renderWire(handBounds.begin(), handBounds.end());
		pointRenderer.render();
	}
}

//------------------------------------------------------------------------------

void RagPlanner::renderTrialData(TrialData::Map::const_iterator dataPtr) {
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);

		// remove points rendered at the initial stage
		pointRenderer.reset();
		sampleRenderer.reset();
		//grasp::RBPose::Sample::Seq::const_iterator begin = (showMLEPoses) ? pBelief->getHypotheses().begin() : dataPtr->second.samples.begin();
		//grasp::RBPose::Sample::Seq::const_iterator end = (showMLEPoses) ? pBelief->getHypotheses().end() : dataPtr->second.samples.end();
		// draw samples
		for (grasp::RBPose::Sample::Seq::const_iterator i = dataPtr->second.samples.begin(); i != dataPtr->second.samples.end(); ++i) {
			const Mat34 actionFrame(i->q, i->p);
			const Mat34 sampleFrame(actionFrame * modelFrame);
			//context.write("modelFrame <%.4f,%.4f,%.4f>\nqueryFrame <%.4f,%.4f,%.4f>\nactionFrame <%.4f,%.4f,%.4f>\nsampleFrame <%.4f,%.4f,%.4f>\n\n",
			//	modelFrame.p.x, modelFrame.p.y, modelFrame.p.z, queryFrame.p.x, queryFrame.p.y, queryFrame.p.z, actionFrame.p.x, actionFrame.p.y, actionFrame.p.z,
			//	sampleFrame.p.x, sampleFrame.p.y, sampleFrame.p.z);
			if (showSampleFrame) 
				sampleRenderer.addAxes(sampleFrame, distribFrameSize);
		
			if (showSamplePoints) {
				grasp::Cloud::PointSeq sample; // = modelPoints;
				sample.reserve(modelPoints.size());
				for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
					grasp::Cloud::Point p = *point;
					if (!showSampleColour) grasp::Cloud::setColour(i == dataPtr->second.samples.begin() ? golem::RGBA::WHITE : golem::RGBA::BLUE, p);
					sample.push_back(p);
				}
				grasp::Cloud::transform(actionFrame, sample, sample);
				sampleAppearance.draw(sample, sampleRenderer);
			}
		}
		// draw MLE
		size_t idx = 0;
		for (grasp::RBPose::Sample::Seq::const_iterator i = pBelief->getHypotheses().begin(); i != pBelief->getHypotheses().end(); ++i, ++idx) {
			const Mat34 actionFrame(i->q, i->p);
			const Mat34 sampleFrame(actionFrame * modelFrame);
			//context.write("modelFrame <%.4f,%.4f,%.4f>\nqueryFrame <%.4f,%.4f,%.4f>\nactionFrame <%.4f,%.4f,%.4f>\nsampleFrame <%.4f,%.4f,%.4f>\n\n",
			//	modelFrame.p.x, modelFrame.p.y, modelFrame.p.z, queryFrame.p.x, queryFrame.p.y, queryFrame.p.z, actionFrame.p.x, actionFrame.p.y, actionFrame.p.z,
			//	sampleFrame.p.x, sampleFrame.p.y, sampleFrame.p.z);
			if (showMLEFrame) 
				sampleRenderer.addAxes(sampleFrame, distribFrameSize);
		
			const bool show = pBelief->getHypotheses().size() < 100 || (pBelief->getHypotheses().size() >= 100 && idx%12 == 0);
			if (showMLEPoints && show) {
				grasp::Cloud::PointSeq sample; // = modelPoints;
				sample.reserve(modelPoints.size());
				const golem::RGBA colour(255*i->weight, 0, 0, 255);
				for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
					grasp::Cloud::Point p = *point;
					grasp::Cloud::setColour(golem::RGBA::MAGENTA, p);
					sample.push_back(p);
				}
				grasp::Cloud::transform(actionFrame, sample, sample);
				sampleAppearance.draw(sample, sampleRenderer);
			}
		}

	}
}

void RagPlanner::renderData(Data::Map::const_iterator dataPtr) {
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
	
		pointFeatureRenderer.reset();
		const bool showmodel = modelDataPtr == dataPtr;
		const bool showquery = queryDataPtr == dataPtr;

		if (showmodel || showquery) {
			if (showmodel) {
				pointFeatureRenderer.addAxes(modelFrame, featureFrameSize);
			}
		
			if (showquery) {
				if (featureIndex >= (golem::U32)pRBPose->getQueryFeatures().size()) {
					featureIndex = (golem::U32)rand.next()%pRBPose->getQueryFeatures().size();
				}
				//if (showQueryDistrib) {
				//	for (size_t i = 0; i < distribSamples; ++i) {
				//		pointFeatureRenderer.addAxes(pRBPose->sample().toMat34() * modelFrame, distribFrameSize);
				//	}
				//}
				if (!showModelPoints) {
					if (showModelFeatures) {
						const grasp::RBPose::Feature& queryFeature = pRBPose->getQueryFeatures()[featureIndex];
						queryFeature.draw(pointFeatureRenderer, queryAppearance);
					}
				}
				if (showModelPoints) {
					if (showModelFeatures) {
						const I32 modelFeatureIndex = pRBPose->getIndices()[featureIndex];
						grasp::RBPose::Feature modelFeature = pRBPose->getModelFeatures()[modelFeatureIndex];
						modelFeature.frame.multiply(grasp::to<Data>(dataPtr)->actionFrame, modelFeature.frame);
						modelFeature.draw(pointFeatureRenderer, modelAppearance);
					}
				}
			}
		}
		
		if (!grasp::to<Data>(dataPtr)->queryPoints.empty()) {
			pointFeatureRenderer.addAxes(grasp::to<Data>(dataPtr)->queryFrame, featureFrameSize);
			if (showModelPoints)
				grasp::to<Data>(dataPtr)->draw(grasp::to<Data>(dataPtr)->queryPoints, pointFeatureRenderer);
		}
	}

	Player::renderData(dataPtr);

	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		data->appearance.draw(groundTruthPoints, pointRenderer);
	}
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
		const Real lhd = this->pHeuristic->density(maxDist);
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


void RagPlanner::renderHand(const golem::Controller::State &state) {
	{		
		golem::CriticalSectionWrapper csw(csDataRenderer);
		Bounds::Seq bounds = manipulator->getBounds(manipulator->getPose(state));
		handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());

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

void RagPlanner::profile(TrialData::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
	if (trnEnable) {
		context.debug("PosePlanner::profile(): Computing trajectory in a new frame...\n");
		golem::Controller::State::Seq seq;
		context.write("approaching trajectory before\n");
		printTrajectory(inp, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
		robot->createTrajectory(grasp::to<Data>(currentDataPtr)->actionFrame, inp.begin(), inp.end(), seq);
		context.write("approaching trajectory after\n");
		printTrajectory(seq, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
	}
	if (inp.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::profile(): At least two waypoints required");

	golem::Controller::State::Seq& out = dataPtr->second.action;
	golem::Controller::State::Seq::iterator ptr = out.end(), tmpend;
	const golem::SecTmReal t = out.empty() ? context.getTimer().elapsed() : out.back().t;
	const Controller::State begin(inp.front(), t), end(inp.back(), begin.t + dur);

	// create trajectory
	tmpend = out.end();
	pProfile->create(inp.begin(), inp.end(), begin, end, out, ptr, tmpend);
	// profile trajectory
	tmpend = out.end();
	pProfile->profile(out, ptr, tmpend);
	// stationary waypoint in the end
	out.push_back(Controller::State(end, end.t + idle));
}

void RagPlanner::perform(TrialData::Map::iterator dataPtr) {
	context.debug("RagPlanner::perform(): Computing trajectory in a new frame...\n");

	if (dataPtr->second.action.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	SecTmReal init = context.getTimer().elapsed();
	robot->createTrajectory(robot->recvState().command, &dataPtr->second.action.front(), NULL, 0, initTrajectory);
	std::printf("RagPlanner::perform: computational time %.7f\n", context.getTimer().elapsed() - init);
	dataPtr->second.executedTrajectory = initTrajectory;

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), dataPtr->second.action.begin(), dataPtr->second.action.end());
	//context.write("trajectory to execute\ninit\n");
	//printTrajectory(initTrajectory, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
	//context.write("action\n");
	//printTrajectory(dataPtr->second.action, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
	if (!testTrajectory(completeTrajectory.begin(), completeTrajectory.end()))
		return;

	std::stringstream prefix;
	prefix << std::fixed << std::setprecision(3) << dataPtr->first << "-" << context.getTimer().elapsed();

	// start recording the approach trajectory if isRecordingApp() is set
	for (auto i: cameraSeq)
		if (i->isRecordingApp()) i->start(prefix.str().c_str());
	// block untilthe  first image arrives
	for (auto i: cameraSeq)
		if (i->isRecordingApp()) i->wait();

//	robot->getController()->initControlCycle();
	// go to initial state
	universe.postScreenCaptureFrames(-1);
	robot->sendTrajectory(initTrajectory, true);
	robot->waitForEnd();
	//renderContacts();
	::Sleep(2000);
	universe.postScreenCaptureFrames(0);	

	// start recording the remaining trajectory part
	for (auto i: cameraSeq)
		if (!i->isRecordingApp()) i->start(prefix.str().c_str());
	// block until the first image arrives
	for (auto i: cameraSeq)
		if (!i->isRecordingApp()) i->wait();

	dataPtr->second.triggeredGuards.clear();
	Controller::State state = robot->getController()->createState();
	dataPtr->second.triggered = robot->getTriggeredGuards(dataPtr->second.triggeredGuards, state);
	// -1 means no contact between the object and the hand (most likely a contact with the table).
	if (dataPtr->second.triggered == -1 && dataPtr->second.replanning)
		return;
	
	Controller::State command = robot->getController()->createState();
	robot->getController()->lookupCommand(state.t, command);
	//context.write("config state with triggered guards\n");
	//printState(state, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
	//context.write("command state with triggered guards\n");
	//printState(command, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());

	// triggered == 0 means no unexpected contact (we can grasp)
	if (dataPtr->second.triggered == 0) {
		// clear states
//		dataPtr->second.robotStates.clear();

		universe.postScreenCaptureFrames(-1);
		// send trajectory
		robot->enableGuards(false);
		robot->sendTrajectory(dataPtr->second.action, true);

		// repeat every send waypoint until trajectory end
		for (U32 i = 0; robot->waitForBegin(); ++i) {
			if (universe.interrupted())
				throw grasp::Interrupted();
			if (robot->waitForEnd(0))
				break;

			// print every 10th robot state
			if (i%10 == 0)
				context.write("State #%d\r", i);

//			dataPtr->second.robotStates.push_back(robot->recvState());
		}
		::Sleep(100);
		universe.postScreenCaptureFrames(0);	
		dataPtr->second.triggered = robot->isGrasping(dataPtr->second.triggeredGuards, state);
		dataPtr->second.replanning = dataPtr->second.triggered == 0;
		context.write("Perform(): return (triggered=%d, replanning=%s)\n", dataPtr->second.triggered, dataPtr->second.replanning ? "true" : "false"); 
	}
	// if a contact occured but we do not want to trigger replanning
	else if (!dataPtr->second.replanning) {
		// ensure that config and command are the same
		robot->initControlCycle();
		robot->enableGuards(false);
		// clear states
//		dataPtr->second.robotStates.clear();
		Controller::State::Seq seq;
		Controller::State graspPose = robot->recvState().config;
		// setting only the fingers to grasp pose
		for (Configspace::Index j = robot->getStateHandInfo().getJoints().begin(); j != robot->getStateHandInfo().getJoints().end(); ++j)
			graspPose.cpos[j] = dataPtr->second.approachAction.begin()->command.cpos[j];
		seq.push_back(robot->recvState().config);
		seq.push_back(graspPose);
		// define the profile
		dataPtr->second.action.clear();
		grasp::to<Data>(queryDataPtr)->action.clear();
		Player::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
		// copy the planned trajectory in my trial data
		poseDataPtr->second.action.clear();
		for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
			poseDataPtr->second.action.push_back(*i);
		// move robot
		universe.postScreenCaptureFrames(-1);
		robot->sendTrajectory(dataPtr->second.action, true);
		// repeat every send waypoint until trajectory end
		for (U32 i = 0; robot->waitForBegin(); ++i) {
			if (universe.interrupted())
				throw grasp::Interrupted();
			if (robot->waitForEnd(0))
				break;

			// print every 10th robot state
			if (i%10 == 0)
				context.write("State #%d\r", i);

//			dataPtr->second.robotStates.push_back(robot->recvState());
		}
		::Sleep(100);
		universe.postScreenCaptureFrames(0);	
		context.write("(Single grasp attempt) Performance finished!\n");
		return;
	}

	// stop recording at the last frame
	// HACK: comment out for Justin! (stop manually due to faulty sync)
	for (auto i: cameraSeq)
		i->stop(context.getTimer().elapsed() + trjManipPerfOff);//robotStates.back().config.t;

	dataPtr->second.triggeredStates.clear();
	dataPtr->second.triggeredStates.push_back(state);
	universe.postScreenCaptureFrames(-1);
	updateAndResample(dataPtr);
	::Sleep(100);
	universe.postScreenCaptureFrames(0);	
	//dataPtr->second.replanning = false;
	//for (size_t i = 0; i < grasp::RBCoord::N; ++i) {
	//	if (pBelief->getSampleProperties().covarianceSqrt[i] > 0.0000001) {
	//		dataPtr->second.replanning = true;
	//		break;
	//	}
	//}

	if (dataPtr->second.triggered > 0/* && dataPtr->second.replanning*/) {
		context.write("Perform(): return (triggered=%s, replanning=%s)\n", dataPtr->second.triggered > 0 ? "true" : "false", dataPtr->second.replanning ? "true" : "false"); 
		robot->initControlCycle();
//		return;
	}	

	
	context.write("Performance finished!\n");
}

void RagPlanner::performApproach(TrialData::Map::iterator dataPtr) {
	if (grasp::to<Data>(queryDataPtr)->queryPoints.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::performApproach(): no query is present.");

	context.write("RagPlanner::performApproach(): build and perform approach trajectory\n");
	// fixed properties of the planner
	pHeuristic->enableUnc = this->uncEnable;
	robot->enableGuards();
	dataPtr->second.replanning = true;
//	robot->setCollisionDetection(true);		

	// handle the approaching action to generate a sequence of states 
	//context.write("approaching trajetory (list)\n");
	//printTrajectory(makeCommand(dataPtr->second.approachAction), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());		
	Controller::State::Seq seq;
	extrapolate(makeCommand(dataPtr->second.approachAction), trjApproachExtrapolFac, false, seq);
	// compute the approach traectory in a new frame and its profile
	dataPtr->second.action.clear();
	grasp::to<Data>(queryDataPtr)->action.clear();
	PosePlanner::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
	// copy the planned trajectory in my trial data
	poseDataPtr->second.action.clear();
	for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
		poseDataPtr->second.action.push_back(*i);

	// perform action
	perform(poseDataPtr);
	//dataPtr->second.action.clear();
	//queryDataPtr->second.action.clear();
}

void RagPlanner::performWithdraw(TrialData::Map::iterator dataPtr) {
	context.write("RagPlanner::performWithdraw(): build and perform withdraw trajectory (vanilla version withdraw to initial state).\n");
	// handle the approaching action to generate a sequence of states 
	Controller::State::Seq seq;
	extrapolate(makeCommand(dataPtr->second.approachWithdraw), trjApproachExtrapolFac, false, seq);
	// compute the approach traectory in a new frame and its profile
	dataPtr->second.action.clear();
	grasp::to<Data>(queryDataPtr)->action.clear();
	Player::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
	// copy the planned trajectory in my trial data
	poseDataPtr->second.action.clear();
	for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
		poseDataPtr->second.action.push_back(*i);

	// fixed properties of the planner
	robot->enableGuards(false);
	pHeuristic->enableUnc = false;
//	robot->setCollisionDetection(false);

	golem::Controller::State::Seq initTrajectory;
	robot->createTrajectory(robot->recvState().command, &dataPtr->second.action.front(), NULL, 0, initTrajectory);
	dataPtr->second.executedTrajectory = initTrajectory;

	if (!testTrajectory(initTrajectory.begin(), initTrajectory.end()))
		return;

	universe.postScreenCaptureFrames(-1);
	robot->sendTrajectory(initTrajectory, true);
	robot->waitForEnd();
	::Sleep(100);
	universe.postScreenCaptureFrames(0);	
	// perform action
//	Player::perform(queryDataPtr);
	//dataPtr->second.action.clear();
	//queryDataPtr->second.action.clear();
}

void RagPlanner::performManip(TrialData::Map::iterator dataPtr) {
	context.write("RagPlanner::performManip(): build and perform manipulative trajectory (vanilla version just lift the object up).\n");
	// handle the approaching action to generate a sequence of states 
	Controller::State::Seq seq;
	extrapolate(makeCommand(dataPtr->second.manipAction), trjApproachExtrapolFac, false, seq);
	// compute the approach traectory in a new frame and its profile
	dataPtr->second.action.clear();
	grasp::to<Data>(queryDataPtr)->action.clear();
	Player::profile(queryDataPtr, seq, trjManipDuration, trjManipIdle);
	// copy the planned trajectory in my trial data
	poseDataPtr->second.action.clear();
	for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
		poseDataPtr->second.action.push_back(*i);

	// fixed properties of the planner
	robot->enableGuards(false);
	pHeuristic->enableUnc = false;
//	robot->setCollisionDetection(false);

	golem::Controller::State::Seq initTrajectory;
	robot->createTrajectory(robot->recvState().command, &dataPtr->second.action.front(), NULL, 0, initTrajectory);
	dataPtr->second.executedTrajectory = initTrajectory;

	if (!testTrajectory(initTrajectory.begin(), initTrajectory.end()))
		return;

	universe.postScreenCaptureFrames(-1);
	robot->sendTrajectory(initTrajectory, true);
	robot->waitForEnd();
	::Sleep(100);
	universe.postScreenCaptureFrames(0);	
	// perform action
//	Player::perform(queryDataPtr);
	//dataPtr->second.action.clear();
	//queryDataPtr->second.action.clear();
}


void RagPlanner::performSingleGrasp(TrialData::Map::iterator dataPtr) {
	if (grasp::to<Data>(queryDataPtr)->queryPoints.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::performSingleGrasp(): no query is present.");

	context.write("RagPlanner::performSingleGrasp(): build and perform approach trajectory\n");
	// fixed properties of the planner
	pHeuristic->enableUnc = this->uncEnable;
	robot->enableGuards();
	dataPtr->second.replanning = false;
//	robot->setCollisionDetection(true);		

	// handle the approaching action to generate a sequence of states 
	//context.write("approaching trajetory (list)\n");
	//printTrajectory(makeCommand(dataPtr->second.approachAction), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());		
	Controller::State::Seq seq;
	extrapolate(makeCommand(dataPtr->second.approachAction), trjApproachExtrapolFac, false, seq);
	// compute the approach traectory in a new frame and its profile
	dataPtr->second.action.clear();
	grasp::to<Data>(queryDataPtr)->action.clear();
	PosePlanner::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
	// copy the planned trajectory in my trial data
	poseDataPtr->second.action.clear();
	for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
		poseDataPtr->second.action.push_back(*i);

	// perform action
	perform(poseDataPtr);
	//dataPtr->second.action.clear();
	//queryDataPtr->second.action.clear();
}


//------------------------------------------------------------------------------

void RagPlanner::generate(grasp::RBPose::Sample::Seq::const_iterator begin, grasp::RBPose::Sample::Seq::const_iterator end, TrialData::Map::iterator dataPtr, grasp::RBPose::Sample::Seq &candidates) const {
	// copies the full set of samples as a high dimension approximation of the pdf
	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::const_iterator sample = begin; sample != end; ++sample) {
		grasp::RBPose::Sample s = *sample;
		s.weight = pRBPose->density(s);
		dataPtr->second.pdf.push_back(s);
		golem::kahanSum(norm, c, s.weight);
//			std::printf("sample frame [%.4f,%.4f,%.4f,%.4f]<%.4f,%.4f,%.4f>  w=%.5f  norm=%.5f\n", 
//				s.q.w, s.q.x, s.q.y, s.q.z, s.p.x, s.p.y, s.p.z, s.weight, norm);
	}

	// computes the normalised importance weight associated to each sample
	dataPtr->second.normFac = norm = golem::REAL_ONE/norm;
	candidates.clear();
	candidates.reserve(dataPtr->second.pdf.size());
		
	for (grasp::RBPose::Sample::Seq::iterator i = dataPtr->second.pdf.begin(); i != dataPtr->second.pdf.end(); ++i)
		if ((i->weight *= norm) > theta)
			candidates.push_back(*i);
}

golem::Real RagPlanner::evaluate(const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample) const {
	// compute the relative transformation between the pose of the joint and the sample
	//context.write("RagPlanner::evaluate(): pose <%5.7f %5.7f %5.7f>, sample <%5.7f %5.7f %5.7f>, dist=%5.7f\n",
	//	pose.p.x, pose.p.y, pose.p.z, sample.p.x, sample.p.y, sample.p.z, sample.p.distance(pose.p));
	Mat34 queryFrameInv, sampleFrame(sample.q, sample.p), relativeJointPose;
	queryFrameInv.setInverse(sampleFrame/* * modelFrame*/);
	relativeJointPose.multiply(queryFrameInv, Mat34(pose.q, pose.p));
	//context.write("        query-to-joint frame <%5.7f %5.7f %5.7f>\n",
	//	relativeJointPose.p.x, relativeJointPose.p.y, relativeJointPose.p.z, modelFrame.p.x, modelFrame.p.y, modelFrame.p.z, modelFrame.p.distance(relativeJointPose.p));

	relativeJointPose.multiply(modelFrame, relativeJointPose);
	//context.write("                    new pose <%5.7f %5.7f %5.7f>,  model <%5.7f %5.7f %5.7f>, dist=%5.7f\n",
	//	relativejointpose.p.x, relativejointpose.p.y, relativejointpose.p.z, modelframe.p.x, modelframe.p.y, modelframe.p.z, modelframe.p.distance(relativejointpose.p));
	Real distMin = golem::REAL_ZERO;
	for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
		const Real dist = relativeJointPose.p.distance(grasp::Cloud::getPoint(*point));
		if (dist > distMin)
			distMin = dist;
	}
//	context.write(" -> dist(min)=%5.7f, prob of touching=%5.7f\n", distMin, Math::exp<golem::Real>(-distMin));
	return Math::exp<golem::Real>(-distMin);
}

void RagPlanner::updateAndResample(TrialData::Map::iterator dataPtr) {
	context.debug("RagPlanner::updateAndResample(): %d triggered guards:", dataPtr->second.triggeredGuards.size());
	//for (std::vector<Configspace::Index>::const_iterator i = dataPtr->second.triggeredGuards.begin(); i != dataPtr->second.triggeredGuards.end(); ++i)
	//	context.write(" %d ", *i);
	for (std::vector<grasp::FTGuard>::const_iterator i = dataPtr->second.triggeredGuards.begin(); i != dataPtr->second.triggeredGuards.end(); ++i)
		context.write(" %s ", i->str());
	context.write("\n");

	golem::Waypoint w(*robot->getController(), dataPtr->second.triggeredStates.begin()->cpos);
	grasp::RealSeq force;
	robot->readFT(*dataPtr->second.triggeredStates.begin(), force);
	context.write("force <");
	for (grasp::RealSeq::const_iterator i = force.begin(); i != force.end(); ++i)
		context.write("%f ", *i);
	context.write(">\n");

	context.write("update hypothesis poses/n");
	// retrieve current object pose
	Mat34 objectPose, modelFrameInv;
	modelFrameInv.setInverse(modelFrame);
	::Sleep(2000);
	robot->getFrameObject(objectPose);
	objectPose.multiply(objectPose, modelFrameInv);
	// render a priori
	bool tmp = showMLEFrame;
	showMLEFrame = true;
	renderTrialData(dataPtr);
	// move hypotheses
	pBelief->createUpdate(pBelief->transform(objectPose));
	// render a posteriori
	::Sleep(2000);
	renderTrialData(dataPtr);
	::Sleep(2000);
	showMLEFrame = tmp;
	
	context.write("updating weigths\n");
	// update samples' weights
	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::iterator sample = pBelief->getHypotheses().begin(); sample !=  pBelief->getHypotheses().end(); ++sample) {
		sample->weight = pHeuristic->evaluate(manipulator.get(), w, *sample, dataPtr->second.triggeredGuards, force, modelFrame);
		golem::kahanSum(norm, c, sample->weight);
	}
	context.write("normalising weights norm=%f (%.30f)\n", golem::REAL_ONE/norm, norm);
	// computes the normalised importance weight associated to each sample
	dataPtr->second.normFac = norm = golem::REAL_ONE/norm;
	for (grasp::RBPose::Sample::Seq::iterator sample = pBelief->getHypotheses().begin(); sample !=  pBelief->getHypotheses().end(); ++sample) 
		sample->weight *= norm;

	context.write("resampling (wheel algorithm)\n");
	// resampling (wheel algorithm)
	pBelief->createResample();
	
	dataPtr->second.samples.clear();
	dataPtr->second.samples.reserve(K);

	// update the query frame
	mlFrame = pBelief->maximum();
	grasp::to<Data>(queryDataPtr)->actionFrame = Mat34(mlFrame.q, mlFrame.p);
	// update query settings
	grasp::to<Data>(queryDataPtr)->queryFrame.multiply(grasp::to<Data>(queryDataPtr)->actionFrame, modelFrame);
	grasp::Cloud::transform(grasp::to<Data>(queryDataPtr)->actionFrame, modelPoints, grasp::to<Data>(queryDataPtr)->queryPoints);
	
	dataPtr->second.samples.push_back(mlFrame);
	for (size_t i = 1; i < K; ++i)
		dataPtr->second.samples.push_back(pBelief->sampleHypothesis());
	renderTrialData(dataPtr);
	
	context.write("end\n");
}

//------------------------------------------------------------------------------

void RagPlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'S':
	{
		if (poseDataPtr._Ptr == NULL)
			grasp::PosePlanner::function(dataPtr, key);
		else {
			std::string index = poseDataPtr->first;
			readString("Enter trial data name: ", index);

			TrialData trialData = poseDataPtr->second;
			const std::string path = grasp::makeString("%s%s.dat", data->dir.c_str(), index.c_str());
			// save to file
			context.write("Saving trial data to: %s\n", path.c_str());
			FileWriteStream fws(path.c_str());
			fws << trialData;
			context.write("Done!\n");
		}
		return;
	}
	case 'L':
	{
		if (poseDataPtr._Ptr == NULL)
			grasp::PosePlanner::function(dataPtr, key);
		else {
			
			std::string index = (poseDataPtr._Ptr != NULL) ? poseDataPtr->first : dataPtr->first;
			readString("Enter trial data name: ", index);
			const std::string path = grasp::makeString("%s%s.dat", data->dir.c_str(), index.c_str());
			context.write("Loading trial data from: %s\n", path.c_str());
			// load from file
			TrialData trialData(*robot->getController());
			FileReadStream frs(path.c_str());
			frs >> trialData;
			getData().erase(index); // delete if exists
			scene.getOpenGL((poseDataPtr._Ptr != NULL) ? poseDataPtr->second.openGL : grasp::to<Data>(dataPtr)->openGL);
			poseDataPtr = dataMap.insert(dataMap.begin(), TrialData::Map::value_type(index, trialData)); // insert new
			scene.getOpenGL(poseDataPtr->second.openGL);
			showSampleColour = false;
			showSampleFrame = false;
			showSamplePoints = true;
			showMLEFrame = true;
			showMLEPoints = false;
			pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
			pHeuristic->enableUnc = uncEnable;			
			renderTrialData(poseDataPtr);
			context.write("Done!\n");
		}
		return;
	}
	case 'R':
	{
		if (poseDataPtr._Ptr == NULL) {
			context.write("Unable to reset.\n");
			return;
		}
		pBelief->reset();
		poseDataPtr->second.samples.clear();
		for (grasp::RBPose::Sample::Seq::const_iterator i = poseDataPtr->second.init.begin(); i != poseDataPtr->second.init.end(); ++i)
			poseDataPtr->second.samples.push_back(*i);
		showSampleColour = false;
		showSampleFrame = false;
		showSamplePoints = true;
		showMLEFrame = true;
		showMLEPoints = false;
		pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
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
	case 'M':
	{
		grasp::PosePlanner::function(dataPtr, key);
		const Mat34 transform(Mat33::identity(), Vec3(golem::REAL_ZERO, golem::REAL_ZERO, golem::REAL_ZERO));
		pHeuristic->setModel(modelPoints.begin(), modelPoints.end(), transform);
		return;
	}
	case 'Q':
	{
		grasp::Cloud::PointSeqMap::iterator points = getPoints(dataPtr);

		// move the point cloud for testing purposes
		groundTruthPoints.clear();
		Vec3 v;
		Quat q;
		if (ragDesc.gtNoiseEnable) {
			v.next(rand); // |v|==1
			v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, ragDesc.gtPoseStddev.lin)), v);
			q.next(rand, ragDesc.gtPoseStddev.ang);		
			for (grasp::Cloud::PointSeq::iterator p = points->second.begin(); p != points->second.end(); ++p) {
				//grasp::Point point(*p);
				//point.colour = golem::RGBA::YELLOW;
				//groundTruthPoints.push_back(point);
				//grasp::RBCoord kernel(p->frame);
				//kernel.p.add(kernel.p, v);
				//kernel.q.multiply(kernel.q, q);
				//p->frame = Mat34(kernel.q, kernel.p);
				grasp::Cloud::Point point(*p);
				grasp::Cloud::setColour(golem::RGBA::YELLOW, point);
				groundTruthPoints.push_back(point);
				p->x += (float)v.x;
				p->y += (float)v.y;
				p->z += (float)v.z;
			}
		}

		// query create
		context.write("Creating %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
		Mat34 objectPose(Mat33(objectRealPose.q), objectRealPose.p), modelFrameInv;
		modelFrameInv.setInverse(modelFrame);
		objectPose.multiply(objectPose, modelFrameInv);
		pBelief->realPose = grasp::RBCoord(objectPose);
		pBelief->setInitObjPose(objectPose);
		pBelief->createQuery(points->second); //((Belief&)*pRBPose.get()).createQuery(dataPtr->second.pointCloud, objectLabel);
		queryDataPtr = dataPtr;
		// compute frame
		mlFrame = pBelief->maximum();
		grasp::to<Data>(queryDataPtr)->actionFrame = Mat34(mlFrame.q, mlFrame.p);
		initActionFrame = Mat34(mlFrame.q, mlFrame.p);
		context.write("<ML Frame v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\"/>\n", mlFrame.p.x, mlFrame.p.y, mlFrame.p.z, mlFrame.q.q0, mlFrame.q.q1, mlFrame.q.q2, mlFrame.q.q3);

		// update query settings
		featureIndex = golem::numeric_const<golem::U32>::MAX;
		grasp::to<Data>(queryDataPtr)->queryFrame.multiply(grasp::to<Data>(queryDataPtr)->actionFrame, modelFrame);	
		grasp::RBCoord query(grasp::to<Data>(queryDataPtr)->queryFrame);
		context.write("<Query Frame v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\"/>\n", query.p.x, query.p.y, query.p.z, query.q.w, query.q.x, query.q.y, query.q.z);

		grasp::to<Data>(queryDataPtr)->actionApproach = grasp::to<Data>(modelDataPtr)->actionApproach;
		grasp::to<Data>(queryDataPtr)->actionManip = grasp::to<Data>(modelDataPtr)->actionManip;
		grasp::to<Data>(queryDataPtr)->action = grasp::to<Data>(modelDataPtr)->action;
		grasp::Cloud::transform(grasp::to<Data>(queryDataPtr)->actionFrame, modelPoints, grasp::to<Data>(dataPtr)->queryPoints);

		showModelPoints = true;
		showModelFeatures = false;
		showQueryDistrib = true;
		showDistribution = false;
		context.write("Done!\n");

		// copy to spam::TrialData
		TrialData trialData(*robot->getController());
		//(grasp::TrialData&)trialData = dataPtr->second;
		trialData.approachAction = grasp::to<Data>(queryDataPtr)->actionApproach;
		trialData.manipAction = grasp::to<Data>(queryDataPtr)->actionManip;
		trialData.action = grasp::to<Data>(queryDataPtr)->action;
		trialData.openGL = grasp::to<Data>(queryDataPtr)->openGL;
		poseDataPtr = dataMap.insert(dataMap.begin(), TrialData::Map::value_type("1", trialData));
			
		// save noise parameter of the trial
		poseDataPtr->second.gtNoiseEnable = ragDesc.gtNoiseEnable;
		poseDataPtr->second.noiseLin = v;
		poseDataPtr->second.noiseAng = q;

		// set the home pose of the robot
		poseDataPtr->second.homeStates.push_back(robot->recvState());
		// save the current configuration as the home pose
		//if (poseDataPtr->second.approachWithdraw.empty()) {
		//	poseDataPtr->second.approachWithdraw.push_back(robot->recvState());
		//	poseDataPtr->second.approachWithdraw.push_back(robot->recvState());
		//}
		renderData(dataPtr);
		return;
	}
	case 'B':
	{
		if (poseDataPtr._Ptr == NULL) {
			context.write("Unable to find the withdraw trajectory.\n");
			return;
		}
		performWithdraw(poseDataPtr);
		return;
		if (modelPoints.empty() || queryDataPtr == getData().end() || grasp::to<Data>(queryDataPtr)->queryPoints.empty()) {
			context.write("Unable to find a model or query. Please make sure you have generated a model and query.\n");
			return;
		}
		context.write("Learning observational model\n");
		Controller::State::Seq seq;
		extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
		// compute the approach traectory in a new frame and its profile
		grasp::to<Data>(dataPtr)->action.clear();
		grasp::to<Data>(queryDataPtr)->action.clear();
		PosePlanner::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
		// copy the planned trajectory in my trial data
		poseDataPtr->second.action.clear();
		const Controller::State pregrasp = *grasp::to<Data>(dataPtr)->action.begin();
		for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
			poseDataPtr->second.action.push_back(*i);
		
		golem::Controller::State::Seq initTrajectory;
		robot->createTrajectory(robot->recvState().command, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, initTrajectory);
		robot->enableGuards(false);
		robot->sendTrajectory(initTrajectory, true);
		robot->waitForEnd();

		Controller::State state = pregrasp;
		const Chainspace::Index i = robot->getStateHandInfo().getChains().begin() + 1;
		return;
	}
	case 'Z':
	{
		grasp::Cloud::PointSeqMap::iterator points = getPoints(dataPtr);

//		context.write("ground truth (size=%d)\n", dataPtr->second.pointCloud.size());
		groundTruthPoints.clear();
		// Linear component
		Vec3 v;
		v.next(rand); // |v|==1
		v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, ragDesc.gtPoseStddev.lin)), v);
		// Angular component
		Quat q;
		q.next(rand, ragDesc.gtPoseStddev.ang);		
		for (grasp::Cloud::PointSeq::iterator p = points->second.begin(); p != points->second.end(); ++p) {
			//grasp::Point point(*p);
			//point.colour = golem::RGBA::YELLOW;
			//groundTruthPoints.push_back(point);
			//grasp::RBCoord kernel(p->frame);
			//kernel.p.add(kernel.p, v);
			//kernel.q.multiply(kernel.q, q);
			//p->frame = Mat34(kernel.q, kernel.p);
			grasp::Cloud::Point point(*p);
			grasp::Cloud::setColour(golem::RGBA::YELLOW, point);
			groundTruthPoints.push_back(point);
			p->x += (float)v.x;
			p->y += (float)v.y;
			p->z += (float)v.z;
		}
		renderData(dataPtr);
		return;
	}
	case 'U':
	{
		if (modelPoints.empty() || pBelief->getSamples().empty()) {
			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
			return;
		}
		bool replanning = false, conclude = false;
		grasp::RBPose::Sample::Seq candidates;
		switch (waitKey("SGPET", "Press a key to (S)ample object poses, (G)enerate a pre-grasp pose, (P)lan a reach-and-grasp trajectory, (E)nable/Disable planning with uncertainty...")) {
		case 'P':
		{			
			if (grasp::to<Data>(dataPtr)->actionApproach.empty())
				context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");

			// approach action is the entire trajectory taught to the robot
			// NOTE: states are memorised from the last (pre grasp = begin()) to the first (initial pose = end())
			// Generate a new approach action formed by [pregrasp, corrent conf of the robot]
			grasp::RobotState grasp = *grasp::to<Data>(dataPtr)->actionApproach.begin();
			grasp::RobotState pregrasp = *(--grasp::to<Data>(dataPtr)->actionApproach.end());			

			trjApproachExtrapolFac = REAL_ZERO;
			robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
			robot->setCollisionDetection(true);		
			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
			iterations = 1;

REPLAN_TEST:
			Real distanceToGoal = pHeuristic->distance(objectRealPose, grasp::to<Data>(dataPtr)->queryFrame, true);
			context.write("Iteration n.%d, distance to goal %f (lin=%f ang=%f)\n", 
				iterations++, distanceToGoal, objectRealPose.p.distance(grasp::to<Data>(dataPtr)->queryFrame.p), distanceToGoal - objectRealPose.p.distance(grasp::to<Data>(dataPtr)->queryFrame.p));			

			try {
				if (!singleGrasp)
					performApproach(poseDataPtr);
				else {
					context.write("Single grasp performs\n");
					performSingleGrasp(poseDataPtr);
					goto GRASP_QUALITY;
				}
			}
			catch (const Message& msg) {
				context.write("%s\n", msg.str().c_str());
			}
			
			if (poseDataPtr->second.triggered > 0 && !poseDataPtr->second.replanning) {
				// todo: compute if grasped
GRASP_QUALITY:
				context.write("Check the grasp quality (triggered=%s, replanning=%s)\n", poseDataPtr->second.triggered > 0 ? "true" : "false", poseDataPtr->second.replanning ? "true" : "false"); 

				grasp::RealSeq force;
				Real graspQuality = REAL_ZERO;
				try {
					graspQuality = robot->analyseGrasp(force);
				}
				catch (const golem::Message &msg) {
					context.write("%s\n", msg.str().c_str());
				}
				replanning = !(graspQuality > 0);
				context.write("Grasp %s. quality = %f\n", !replanning ? "succeess (no replanning)" : "failure (replanning)", graspQuality);
				//if (!replanning) {
					const int key = waitKey("YN", "Do you want to exit? (Y/N)...");
					if (key == 'Y')
						conclude = true;
				//}
				goto MOVING_BACK;
			}
			else {
MOVING_BACK:
				// set belief state
				context.write("setting new belief state in heuristic...\n");
				pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
				pHeuristic->enableUnc = uncEnable;

				context.write("done!\n");

				poseDataPtr->second.approachWithdraw.clear();
				// if replanning is not required the robot is moved back to the home pose
				if (!poseDataPtr->second.replanning || withdrawToHomePose) 
					while (poseDataPtr->second.approachWithdraw.size() < 2 && !poseDataPtr->second.executedTrajectory.empty())
						poseDataPtr->second.approachWithdraw.push_back(poseDataPtr->second.homeStates.front());
				// otherwise the previous waypoint in the trajectory is withdrawn
				else {
					Real distMin(REAL_MAX);
					Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
					for (Controller::State::Seq::const_iterator a = poseDataPtr->second.executedTrajectory.begin(), b = poseDataPtr->second.executedTrajectory.begin() + 1; b != poseDataPtr->second.executedTrajectory.end(); ++a, ++b) {
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
					while (poseDataPtr->second.approachWithdraw.size() < 2)
						poseDataPtr->second.approachWithdraw.push_back(w);
				}
				renderTrialData(poseDataPtr);
				// move back to home pose
				performWithdraw(poseDataPtr);
				//return;
				if (conclude) {
					performManip(poseDataPtr);
					return;
				}
				goto REPLAN_TEST;
			}
			return;
		}
		case 'S':
		{
			showSampleColour = false;
			showSampleFrame = false;
			showSamplePoints = true;
			showMLEFrame = true;
			showMLEPoints = true;

			// select threshold for samples' weights
			grasp::RBPose::Sample poseMaxLhd = mlFrame;
			golem::Real wPoseMaxLhd = pBelief->density(poseMaxLhd);

			// generate the set of K samples
			context.write("Generating set of %d samples...\n", K);
//			GENERATE_SAMPLES:
			candidates.clear();
			candidates.reserve(K);
			// NOTE: first sample is the maximum likelihood pose
			candidates.push_back(poseMaxLhd);
			size_t iter = 1;
			while (candidates.size() < K && iter++ < maxSamplingIter)
				candidates.push_back(pBelief->sampleHypothesis());
			//candidates = pBelief->sampleHypotheses(poseMaxLhd);

			context.write("done!\n");

			// copy candidates in data trial
			poseDataPtr->second.samples.clear();
			poseDataPtr->second.samples.reserve(K);
			for (grasp::RBPose::Sample::Seq::iterator s = candidates.begin(); s != candidates.end(); ++s) {
				poseDataPtr->second.samples.push_back(*s);
				poseDataPtr->second.init.push_back(*s);
			}
			
			pHeuristic->setBeliefState(candidates, modelFrame);
			pHeuristic->enableUnc = uncEnable;

			context.write("done!\n");
			renderTrialData(poseDataPtr);
			if (replanning) {
				replanning = false;
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
				pose.p = Vec3(0.4, -0.60, 0.1);
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
			context.write("Planning with uncertainty %s\n", (uncEnable = !uncEnable) ? "ON" : "OFF");
			return;
		}
		break;
	}
	case '.':
	{
			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
			GenWorkspaceChainState gwcs;
			robot->getController()->chainForwardTransform(robot->recvState().config.cpos, gwcs.wpos);
//			gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
			renderPose(gwcs.wpos[armChain]);
			grasp::RBCoord b(gwcs.wpos[armChain]);
			context.write("render pose triggered state <%f %f %f> <%f %f %f %f>\n", b.p.x, b.p.y, b.p.z, b.q.w, b.q.x, b.q.y, b.q.z);
			{		
				golem::CriticalSectionWrapper csw(csDataRenderer);
				const Bounds::Seq bounds = manipulator->getBounds(manipulator->getPose(robot->recvState().config));
				handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
			}	
			return;
	}
	case '\\':
		if (poseDataPtr._Ptr == NULL)
			grasp::PosePlanner::function(dataPtr, key);
		else {
			showSampleColour = (showSamplePoints)?!showSampleColour:false;
			showSamplePoints = (!showSampleColour)?!showSamplePoints:true;
			showMLEPoints = (!showSampleColour && !showSamplePoints) ? !showMLEPoints : false;
			context.write("Sample points %s w/ original colour %s, MLE points %s\n", showSamplePoints ? "ON" : "OFF",
				showSampleColour ? "ON" : "OFF", showMLEPoints ? "ON" : "OFF");
			renderTrialData(poseDataPtr);
		}
		return;
	case '|':
		if (poseDataPtr._Ptr == NULL)
			grasp::PosePlanner::function(dataPtr, key);
		else {
			showSampleFrame = !showSampleFrame;
			context.write("Sample Frames %s\n", showSampleFrame ? "ON" : "OFF");
			renderTrialData(poseDataPtr);
		}
		return;
	case '8':
		showDistribution = !showDistribution;
		context.write("Distibution poses %s\n", showDistribution ? "ON" : "OFF");
		renderTrialData(poseDataPtr);
		return;
	case '9':
		showMLEFrame = !showMLEFrame;
		context.write("Distibution MLE frames %s\n", showMLEFrame ? "ON" : "OFF");
		renderTrialData(poseDataPtr);
		return;
	case '_':
		scene.getOpenGL(poseDataPtr->second.openGL);
		if (poseDataPtr == dataMap.begin()) poseDataPtr = --dataMap.end(); else --poseDataPtr;
		scene.setOpenGL(poseDataPtr->second.openGL);
		context.write("Trial data name: %s\n", poseDataPtr->first.c_str());
		renderTrialData(poseDataPtr);
		return;
	case '=':
		scene.getOpenGL(poseDataPtr->second.openGL);
		if (++poseDataPtr == dataMap.end()) poseDataPtr = dataMap.begin();
		scene.setOpenGL(poseDataPtr->second.openGL);
		context.write("Trial data name: %s\n", poseDataPtr->first.c_str());
		renderTrialData(poseDataPtr);
		return;
	case '!':
		singleGrasp = !singleGrasp;
		context.write("Single grasp attempt %s\n", singleGrasp ? "ON" : "OFF");
		return;
	} // switch
	grasp::PosePlanner::function(dataPtr, key);
}

//------------------------------------------------------------------------------

void RagPlanner::printTrajectory(const golem::Controller::State::Seq &trajectory, const golem::Configspace::Index &begin, const golem::Configspace::Index &end) const {
	context.write("RagPlanner::printTrajectory():\n");
	for (Controller::State::Seq::const_iterator j = trajectory.begin(); j != trajectory.end(); ++j)
		printState(*j, begin, end);
}

void RagPlanner::printState(const golem::Controller::State &state, const golem::Configspace::Index &begin, const golem::Configspace::Index &end, const std::string &label) const {
	context.write("%s <", label.c_str());
	for (Configspace::Index i = begin; i != end; ++i)
		context.write("%f ", state.cpos[i]);
	context.write(">\n");
}


//------------------------------------------------------------------------------

void spam::XMLData(RagPlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	// load PosePlanner description
	grasp::XMLData((grasp::PosePlanner::Desc&)val, context, xmlcontext, create);

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
	XMLData(val.objectBounds, val.objectBounds.max_size(), xmlcontext->getContextFirst("object_bounding_box"), "bounds", create);
	golem::XMLData("enable_noise", val.gtNoiseEnable, xmlcontext->getContextFirst("point_cloud_noise"), create);
	grasp::XMLData(val.gtPoseStddev, xmlcontext->getContextFirst("point_cloud_noise gt_pose_stddev"), create);

	Belief::Desc* pBeliefDesc(new Belief::Desc);
	(grasp::RBPose::Desc&)*pBeliefDesc = *((grasp::PosePlanner::Desc&)val).pRBPoseDesc;
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