/** @file PosePlanner.cpp
 * 
 * @author	Marek Kopicki
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Plan/Data.h>
#include <Spam/PosePlanner/PosePlanner.h>
#include <Golem/Device/RobotJustin/RobotJustin.h>
#include <Grasp/Grasp/Grasp.h>

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::PosePlanner::Data::xmlData(golem::XMLContext* context, bool create) const {
	grasp::Player::Data::xmlData(context, create);

	try {
		if (!create || !queryPoints.empty()) {
			golem::XMLData(const_cast<golem::Mat34&>(actionFrame), context->getContextFirst("action_frame", create), create);
			golem::XMLData(const_cast<golem::Mat34&>(queryFrame), context->getContextFirst("query_frame", create), create);
			xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(queryPoints), std::string("query_points"), context, create);
		}
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create)
			throw msg;
	}
}

spam::PosePlanner::Data::Ptr spam::PosePlanner::Data::clone() const {
	return Ptr(new Data(*this));
}

//------------------------------------------------------------------------------

spam::PosePlanner::PosePlanner(Scene &scene) : grasp::Player(scene), rand(context.getRandSeed()) {
}
	
bool spam::PosePlanner::create(const Desc& desc) {
	grasp::Player::create(desc); // throws
	
	pRBPose = desc.pRBPoseDesc->create(context); // throws
	pBelief = dynamic_cast<Belief*>(pRBPose.get());

	modelAppearance = desc.modelAppearance;
	queryAppearance = desc.queryAppearance;

	featureFrameSize = desc.featureFrameSize;
	distribFrameSize = desc.distribFrameSize;
	distribSamples = desc.distribSamples;

	actionManip = desc.actionManip;

	modelFrame.setId();
	modelDataPtr = getData().end();
	resetDataPointers();

	numPoses = desc.numPoses;
	numHypotheses = desc.numHypotheses;
	
	scene.getHelp().insert(Scene::StrMapVal("080", "  M                                       model create\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  4                                       model points\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  5                                       model features\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  Q                                       query create\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  6                                       query distribution\n"));

	return true;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::render() {
	grasp::Player::render();
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		pointFeatureRenderer.render();
		sampleRenderer.render();
	}
}

//------------------------------------------------------------------------------

void spam::PosePlanner::resetDataPointers() {
	//auto ptr = getPtr<Data>(queryDataPtr);
	//if (ptr != nullptr) ptr->queryPoints.clear();
	queryDataPtr = getData().end();
	showModelPoints = false;
	showModelFeatures = false;
	showQueryDistrib = false;
	showSamplePoints = false;
	featureIndex = -1;
}

void spam::PosePlanner::renderData(Data::Map::const_iterator dataPtr) {
	grasp::Player::renderData(dataPtr);

	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
	
		pointFeatureRenderer.reset();
		sampleRenderer.reset();
		const bool showmodel = modelDataPtr == dataPtr;
		const bool showquery = queryDataPtr == dataPtr;

		if (showmodel || showquery) {
			if (showmodel) {
				pointFeatureRenderer.addAxes(modelFrame, featureFrameSize);
			}
		
			if (showquery) {
				if (featureIndex >= (golem::U32)pBelief->getQueryFeatures().size()) {
					featureIndex = (golem::U32)rand.next()%pBelief->getQueryFeatures().size();
				}
				if (showSamplePoints) {
					grasp::RBPose::Sample::Seq samples = pBelief->getSamples();
					for (grasp::RBPose::Sample::Seq::iterator i = samples.begin(); i != samples.end(); ++i) {
						const Mat34 actionFrame(i->q, i->p);
						const Mat34 sampleFrame(actionFrame * modelFrame);
						grasp::Cloud::PointSeq sample; // = modelPoints;
						sample.reserve(modelPoints.size());
						for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
							grasp::Cloud::Point p = *point;
							grasp::Cloud::setColour(i == samples.begin() ? golem::RGBA::WHITE : golem::RGBA::BLUE, p);
							sample.push_back(p);
						}
						grasp::Cloud::transform(actionFrame, sample, sample);
						sampleRenderer.addAxes(sampleFrame, distribFrameSize);
						sampleAppearance.draw(sample, sampleRenderer);
					}
				}
				if (showQueryDistrib) {
					for (size_t i = 0; i < distribSamples; ++i) {
						pointFeatureRenderer.addAxes(pRBPose->sample().toMat34() * modelFrame, distribFrameSize);
					}
				}
				if (!showModelPoints) {
					if (showModelFeatures) {
						const grasp::RBPose::Feature& queryFeature = pBelief->getQueryFeatures()[featureIndex];
						queryFeature.draw(pointFeatureRenderer, queryAppearance);
					}
				}
				if (showModelPoints) {
					if (showModelFeatures) {
						const I32 modelFeatureIndex = pBelief->getIndices()[featureIndex];
						grasp::RBPose::Feature modelFeature = pBelief->getModelFeatures()[modelFeatureIndex];
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
}

//------------------------------------------------------------------------------

void spam::PosePlanner::profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
	if (!grasp::to<Data>(queryDataPtr)->queryPoints.empty()) {
		context.write("PosePlanner::profile(): Computing trajectory in a new frame...\n");
		golem::Controller::State::Seq seq;
		robot->createTrajectory(grasp::to<Data>(dataPtr)->actionFrame, inp.begin(), inp.end(), seq);
		grasp::Player::profile(dataPtr, seq, dur, idle);
	}
	else
		grasp::Player::profile(dataPtr, inp, dur, idle);
}

void spam::PosePlanner::profileManip(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
	if (!inp.empty()) {
		profile(dataPtr, inp, dur, idle);
		return;
	}
	
	golem::Controller::State::Seq& out = grasp::to<Data>(dataPtr)->action;
	if (out.empty())
		throw Message(Message::LEVEL_ERROR, "PosePlanner::profile(): At least one waypoints required");

	golem::Controller::State::Seq seq;
	
	if (actionManip.workspace) {
		WorkspaceChainCoord wcc;
		robot->getController()->chainForwardTransform(out.back().cpos, wcc);
		Mat34 trn;
		trn.multiply(wcc[robot->getStateInfo().getChains().begin()], robot->getController()->getChains()[robot->getStateInfo().getChains().begin()]->getReferencePose());
		trn.multiply(trn, actionManip);
		robot->createTrajectory(out.back(), nullptr, &trn, dur, seq);
	}
	
	if (seq.empty())
		seq.push_back(out.back());
	for (size_t i = 0; i < (size_t)robot->getStateInfo().getJoints().size() && i < actionManip.c.size(); ++i)
		seq.back().cpos[robot->getStateInfo().getJoints().begin() + i] += actionManip.c[i];

	out.insert(out.end(), ++seq.begin(), seq.end());
	out.push_back(Controller::State(out.back(), out.back().t + idle));
}

grasp::Cloud::PointSeqMap::iterator PosePlanner::getPointsTrn(Data::Map::iterator dataPtr, Mat34 &trn) {
	for (grasp::Cloud::PointSeqMap::iterator p = grasp::to<Data>(dataPtr)->points.begin(); p != grasp::to<Data>(dataPtr)->points.end(); ++p) 
		grasp::Cloud::transform(trn, p->second, p->second);
	return grasp::Director::getPoints(dataPtr);
}


void spam::PosePlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'M':
	{
		grasp::Cloud::PointSeqMap::iterator points = getPointsTrn(dataPtr, grasp::Director::Import::make(grasp::to<Import>(import)->frames));
		// model create
		context.write("Creating %s model...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
		pBelief->createModel(points->second);

		// update model settings
		modelDataPtr = dataPtr;
		modelFrame = Belief::createFrame(points->second);
		modelPoints = points->second;
		resetDataPointers();

		// done
		context.write("Done!\n");
		renderData(dataPtr);
		return;
	}
	case 'Q':
	{
		grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
		// query create
		context.write("Creating %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
		pBelief->createQuery(points->second);
		// create hypotheses and return the action frame for this query
		const grasp::RBPose::Sample frame = pBelief->createHypotheses(modelPoints, modelFrame);
		//// create pose distribution
		//grasp::to<Data>(dataPtr)->poses.clear();
		//grasp::to<Data>(dataPtr)->poses.resize(numPoses);
		//for (size_t i = 0; i < numPoses; ++i)
		//	grasp::to<Data>(dataPtr)->poses.push_back(grasp::RBPose::Sample(pRBPose->sample()));
		// create hypothesis distribution (first element is the MLE)
		//grasp::to<Data>(dataPtr)->hypotheses.clear();
		//grasp::to<Data>(dataPtr)->hypotheses.resize(numHypotheses);
		//grasp::to<Data>(dataPtr)->hypotheses.push_back(frame);
		//for (size_t i = 1; i < numHypotheses; ++i)
		//	grasp::to<Data>(dataPtr)->hypotheses.push_back(*grasp::RBPose::Sample::sample<golem::Ref1, grasp::RBPose::Sample::Seq::const_iterator>(grasp::to<Data>(dataPtr)->poses, rand));

		grasp::to<Data>(dataPtr)->actionFrame = frame.toMat34();
		context.write("action frame <%f %f %f>\n", grasp::to<Data>(dataPtr)->actionFrame.p.x, grasp::to<Data>(dataPtr)->actionFrame.p.y, grasp::to<Data>(dataPtr)->actionFrame.p.z);
		// update query settings
		resetDataPointers();
		queryDataPtr = dataPtr;
		grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->actionFrame, modelFrame);
		grasp::to<Data>(dataPtr)->actionApproach = grasp::to<Data>(modelDataPtr)->actionApproach;
		grasp::to<Data>(dataPtr)->actionManip = grasp::to<Data>(modelDataPtr)->actionManip;
		grasp::to<Data>(dataPtr)->action = grasp::to<Data>(modelDataPtr)->action;
		grasp::Cloud::transform(grasp::to<Data>(dataPtr)->actionFrame, modelPoints, grasp::to<Data>(dataPtr)->queryPoints);
		// done
		context.info("<pose v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\"/>\n", frame.p.x, frame.p.y, frame.p.z, frame.q.q0, frame.q.q1, frame.q.q2, frame.q.q3);
		context.write("Done!\n");
		showSamplePoints = true;
		renderData(dataPtr);
		return;
	}
	case 'T':
	{
		// update approach and manip actions
		context.write("Update approach actions (size %d)...\n", grasp::to<Data>(modelDataPtr)->actionApproach.size());
		if (!grasp::to<Data>(modelDataPtr)->actionApproach.empty()) {
			for(grasp::RobotState::List::iterator state = grasp::to<Data>(dataPtr)->actionApproach.begin(); state != grasp::to<Data>(dataPtr)->actionApproach.end(); ++state) {
				const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
				
				GenWorkspaceChainState gwcs;
				robot->getController()->chainForwardTransform(state->config.cpos, gwcs.wpos);
				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
				gwcs.wpos[armChain].multiply(modelFrame, gwcs.wpos[armChain]); // new waypoint frame

				GenWorkspaceChainState wend;
				wend.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
				wend.wpos[armChain] = gwcs.wpos[armChain];
				context.write("wend ,%f %f %f>\n", wend.wpos[armChain].p.x, wend.wpos[armChain].p.y, wend.wpos[armChain].p.z);
				// lock controller
				golem::CriticalSectionWrapper csw(robot->getControllerCS());
				// Find end position
				grasp::Robot::State pose = robot->recvState();
				if (!robot->getPlanner()->findTarget(pose.config, wend, state->config))
					throw Message(Message::LEVEL_CRIT, "RagPlanner:Creating Model: unable to find grasp configuration");
				state->command = state->config;
				// error
				WorkspaceChainCoord wcc;
				robot->getController()->chainForwardTransform(state->config.cpos, wcc);
				wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
				context.write("wcc ,%f %f %f>\n", wcc[armChain].p.x, wcc[armChain].p.y, wcc[armChain].p.z);
				const grasp::RBDist error(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(wend.wpos[armChain]));
				context.write("RagPlanner:Creating Model: Pose error: lin=%.9f, ang=%.9f\n", error.lin, error.ang);
			}
		}
	}
	case '4':
		showModelPoints = !showModelPoints;
		context.write("Model points %s\n", showModelPoints ? "ON" : "OFF");
		renderData(dataPtr);
		return;
	case '5':
		showModelFeatures = !showModelFeatures;
		featureIndex = -1;
		context.write("Model features %s\n", showModelFeatures ? "ON" : "OFF");
		renderData(dataPtr);
		return;
	case '6':
		showQueryDistrib = !showQueryDistrib;
		context.write("Query distribution %s\n", showQueryDistrib ? "ON" : "OFF");
		renderData(dataPtr);
		return;
	case '7':
		showSamplePoints = !showSamplePoints;
		context.write("Hypotheses distribution %s\n", showSamplePoints ? "ON" : "OFF");
		renderData(dataPtr);
		return;

	}

	Player::function(dataPtr, key);
}

//------------------------------------------------------------------------------

void spam::XMLData(PosePlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	grasp::XMLData((grasp::Player::Desc&)val, context, xmlcontext, create);

	xmlcontext = xmlcontext->getContextFirst("pose_planner");

	XMLData((grasp::RBPose::Desc&)*val.pRBPoseDesc, xmlcontext, create);
	Belief::Desc* pBeliefDesc(new Belief::Desc);
	(grasp::RBPose::Desc&)*pBeliefDesc = *val.pRBPoseDesc;
	val.pRBPoseDesc.reset(pBeliefDesc);
	spam::XMLData((Belief::Desc&)*val.pRBPoseDesc, xmlcontext, create);

	golem::XMLData("num_poses", val.numPoses, xmlcontext);
	golem::XMLData("num_hypotheses", val.numHypotheses, xmlcontext);
	golem::XMLData("distrib_samples", val.distribSamples, xmlcontext->getContextFirst("appearance"), create);
	XMLData(val.modelAppearance, xmlcontext->getContextFirst("appearance model"), create);
	XMLData(val.queryAppearance, xmlcontext->getContextFirst("appearance query"), create);

	XMLData(val.actionManip, xmlcontext->getContextFirst("action_manip"), create);
}

//------------------------------------------------------------------------------

void PosePlannerApp::run(int argc, char *argv[]) {
	// Setup PosePlanner
	PosePlanner::Desc posePlannerDesc;
	XMLData(posePlannerDesc, context(), xmlcontext());

	PosePlanner *pPosePlanner = dynamic_cast<PosePlanner*>(scene()->createObject(posePlannerDesc)); // throws
	if (pPosePlanner == NULL)
		throw Message(Message::LEVEL_CRIT, "PosePlannerApp::run(): Unable to cast to PosePlanner");

	// Random number generator seed
	context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);
	
	try {
		pPosePlanner->main();
	}
	catch (const grasp::Interrupted&) {
	}
	
	context()->info("Good bye!\n");
	scene()->releaseObject(*pPosePlanner);
};

//------------------------------------------------------------------------------

#ifdef _SPAM_POSEPLANNER_MAIN_
int main(int argc, char *argv[]) {
	return spam::PosePlannerApp().main(argc, argv);
}
#endif // _SPAM_POSEPLANNER_MAIN_