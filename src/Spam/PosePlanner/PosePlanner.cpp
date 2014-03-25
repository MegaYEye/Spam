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
	featureIndex = -1;
}

void spam::PosePlanner::renderData(Data::Map::const_iterator dataPtr) {
	grasp::Player::renderData(dataPtr);

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
				if (showQueryDistrib) {
					for (size_t i = 0; i < distribSamples; ++i) {
						pointFeatureRenderer.addAxes(pRBPose->sample().toMat34() * modelFrame, distribFrameSize);
					}
				}
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
}

//------------------------------------------------------------------------------

void spam::PosePlanner::profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
	if (!grasp::to<Data>(dataPtr)->queryPoints.empty()) {
		context.debug("PosePlanner::profile(): Computing trajectory in a new frame...\n");
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

void spam::PosePlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'M':
	{
		grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
		// model create
		context.write("Creating %s model...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
		pRBPose->createModel(points->second);
		// update model settings
		modelDataPtr = dataPtr;
		modelFrame = grasp::RBPose::createFrame(points->second);
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
		pRBPose->createQuery(points->second);
		// create pose distribution
		grasp::to<Data>(dataPtr)->poses.clear();
		grasp::to<Data>(dataPtr)->poses.resize(numPoses);
		for (size_t i = 0; i < numPoses; ++i)
			grasp::to<Data>(dataPtr)->poses.push_back(grasp::RBPose::Sample(pRBPose->sample()));
		// compute frame
		const grasp::RBPose::Sample frame = pRBPose->maximum();
		// create hypothesis distribution (first element is the MLE)
		grasp::to<Data>(dataPtr)->hypotheses.clear();
		grasp::to<Data>(dataPtr)->hypotheses.resize(numHypotheses);
		grasp::to<Data>(dataPtr)->hypotheses.push_back(frame);
		for (size_t i = 1; i < numHypotheses; ++i)
			grasp::to<Data>(dataPtr)->hypotheses.push_back(*grasp::RBPose::Sample::sample<golem::Ref1, grasp::RBPose::Sample::Seq::const_iterator>(grasp::to<Data>(dataPtr)->poses, rand));

		grasp::to<Data>(dataPtr)->actionFrame = frame.toMat34();
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
		renderData(dataPtr);
		return;
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
	}

	Player::function(dataPtr, key);
}

//------------------------------------------------------------------------------

void spam::XMLData(PosePlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	grasp::XMLData((grasp::Player::Desc&)val, context, xmlcontext, create);

	xmlcontext = xmlcontext->getContextFirst("pose_planner");
	XMLData((grasp::RBPose::Desc&)*val.pRBPoseDesc, xmlcontext, create);
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