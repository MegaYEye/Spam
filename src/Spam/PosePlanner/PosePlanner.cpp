/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
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
			golem::XMLData(const_cast<golem::Mat34&>(queryTransform), context->getContextFirst("query_transform", create), create);
			golem::XMLData(const_cast<golem::Mat34&>(queryFrame), context->getContextFirst("query_frame", create), create);
			xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(queryPoints), std::string("query_points"), context, create);
//			xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(simulateObjectPose), std::string("object_points"), context, create);
			golem::XMLData(const_cast<golem::Mat34&>(modelFrame), context->getContextFirst("model_frame", create), create);
			xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(modelPoints), std::string("model_points"), context, create);
		}
		if (!poses.empty() && create) {
			const std::string name = "belief_pdf";
			std::stringstream str;
			str << "Save poses\n";
			size_t id = 1;
			for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
				str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
			str << "---------------------------------------------------------\n";
			std::printf("%s", str.str().c_str());
			xmlDataSave(context->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "pdf", extSamples.c_str()), poses);
		}
		if (!hypotheses.empty() && create){
			const std::string name = "belief_hypotheses";
			std::stringstream str;
			str << "Save hypotheses\n";
			size_t id = 1;
			for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
				str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
			std::printf("%s", str.str().c_str());
			str << "---------------------------------------------------------\n";
			xmlDataSave(context->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "hypotheses", extSamples.c_str()), hypotheses);
		}
		if (!create) {
			const std::string name = "belief_pdf";
			xmlDataLoad(context->getContextFirst(name.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(poses), grasp::RBPose::Sample());
			std::stringstream str;
			str << "Load poses\n";
			size_t id = 1;
			for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
				str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
			str << "---------------------------------------------------------\n";
			const std::string name1 = "belief_hypotheses";
			xmlDataLoad(context->getContextFirst(name1.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(hypotheses), grasp::RBPose::Sample());
			id = 1;
			str << "Load hypotheses\n";
			for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
				str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
			str << "---------------------------------------------------------\n";
			std::printf("%s", str.str().c_str());
		}

	}
	catch (const golem::MsgXMLParser& msg) {
		std::printf("%s\n", msg.what());
		if (create)
			throw msg;
	}
}

grasp::Director::Data::Ptr spam::PosePlanner::createData() const {
	return Data::Ptr(new Data(*grasp::to<Data>(data))); // assuming data has been initialised properly
}

//------------------------------------------------------------------------------

spam::PosePlanner::PosePlanner(Scene &scene) : grasp::Player(scene), rand(context.getRandSeed()) {
}
	
bool spam::PosePlanner::create(const Desc& desc) {
	grasp::Player::create(desc); // throws
	
	pRBPose = desc.pRBPoseDesc->create(context); // throws
	pBelief = static_cast<Belief*>(pRBPose.get());

	modelAppearance = desc.modelAppearance;
	queryAppearance = desc.queryAppearance;
	sampleAppearance.setToDefault();

	featureFrameSize = desc.featureFrameSize;
	distribFrameSize = desc.distribFrameSize;
	distribSamples = desc.distribSamples;

	actionManip = desc.actionManip;

	modelFrame.setId();
	context.write("PosePlannenr::Create(): model trn <%f %f %f>\n", desc.modelTrn.p.x, desc.modelTrn.p.y, desc.modelTrn.p.z);
	modelTrn = desc.modelTrn;
	objectTrn = desc.objectTrn;
	modelDataPtr = getData().end();
	resetDataPointers();
//	screenCapture = desc.screenCapture;

	//numPoses = desc.numPoses;
	//numHypotheses = desc.numHypotheses;
	
	scene.getHelp().insert(Scene::StrMapVal("080", "  M                                       model create\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  4                                       model points\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  5                                       model features\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  Q                                       query create\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  6                                       query distribution\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  7                                       hypotheses distribution\n"));

	return true;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::render() {
	grasp::Player::render();
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		pointFeatureRenderer.render();
		sampleRenderer.render();
//		testRenderer.render();
//		uncRenderer.render();
	}
}

//------------------------------------------------------------------------------

//void spam::PosePlanner::renderUncertainty(const grasp::RBPose::Sample::Seq &samples) {
//	uncRenderer.reset();
//	for (grasp::RBPose::Sample::Seq::const_iterator i = samples.begin(); i != samples.end(); ++i)
//		uncRenderer.addAxes(Mat34(i->q, i->p) * modelFrame, featureFrameSize*i->weight);
//}

void spam::PosePlanner::resetDataPointers() {
	//auto ptr = getPtr<Data>(queryDataPtr);
	//if (ptr != nullptr) ptr->queryPoints.clear();
	queryDataPtr = getData().end();
	showModelPoints = false;
	showQueryPoints = false;
	showModelFeatures = false;
	showQueryDistrib = false;
	showSamplePoints = false;
	showMeanHypothesis = false;
	showObject = false;
//	showTest = false;
	featureIndex = -1;
}

void spam::PosePlanner::renderData(Data::Map::const_iterator dataPtr) {
	grasp::Player::renderData(dataPtr);

	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
	
		pointFeatureRenderer.reset();
		sampleRenderer.reset();
//		testRenderer.reset();
//		sampleRenderer.addAxes(Mat34::identity(), featureFrameSize*5);
		const bool showmodel = modelDataPtr == dataPtr;
		const bool showquery = queryDataPtr == dataPtr;

//		std::printf("showmodel %s showquery %s showSamplePoints %s showQueryDistrib %s\n", showmodel ? "ON" : "OFF", showquery ? "ON" : "OFF", showSamplePoints ? "ON" : "OFF", showQueryDistrib ? "ON" : "OFF");
		if (showmodel || showquery) {
			if (showmodel) {
				pointFeatureRenderer.addAxes(modelFrame, featureFrameSize);
			}
		
			if (showquery) {
				if (featureIndex >= (golem::U32)pBelief->getQueryFeatures().size()) {
					featureIndex = (golem::U32)rand.next()%pBelief->getQueryFeatures().size();
				}
				if (showSamplePoints) {
//					pBelief->drawHypotheses(sampleRenderer);
//					context.write("showSamplePoints hypotheses size %d\n", grasp::to<Data>(dataPtr)->hypotheses.size());
					grasp::RBPose::Sample::Seq samples = grasp::to<Data>(dataPtr)->hypotheses;//pBelief->getHypothesesToSample();
					for (grasp::RBPose::Sample::Seq::iterator i = samples.begin(); i != samples.end(); ++i) {
						const Mat34 actionFrame(i->q, i->p);
						const Mat34 sampleFrame(actionFrame * modelFrame);
						grasp::Cloud::PointSeq sample; // = modelPoints;
						sample.reserve(modelPoints.size());
						for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
							grasp::Cloud::Point p = *point;
							grasp::Cloud::setColour(i == samples.begin() ? golem::RGBA::YELLOW : golem::RGBA::BLUE, p);
							sample.push_back(p);
						}
						grasp::Cloud::transform(actionFrame, sample, sample);
						sampleRenderer.addAxes(sampleFrame, distribFrameSize);
						sampleAppearance.draw(sample, sampleRenderer);
						if (showMeanHypothesis)
							break;
					//context.debug("cloud size %d\n", (*pBelief->getHypotheses().begin())->getCloud().size());
					//sampleAppearance.colourOverride = true;
					//sampleAppearance.colour = RGBA::MAGENTA;
					//sampleAppearance.draw((*pBelief->getHypotheses().begin())->getCloud(), sampleRenderer);
////						break;
					}
				}
				if (showObject)
					sampleAppearance.draw(grasp::to<Data>(dataPtr)->simulateObjectPose, pointFeatureRenderer);
				if (showQueryDistrib) {
//					pBelief->drawSamples(distribSamples, sampleRenderer);
//					context.write("showQueryDistrib sample size %d\n", distribSamples);
					for (size_t i = 0; i < distribSamples; ++i) {
						sampleRenderer.addAxes(pBelief->sample().toMat34() * modelFrame, distribFrameSize);
					}
				}
				if (!showModelPoints) {
					if (showModelFeatures) {
						const grasp::RBPose::Feature& queryFeature = pBelief->getQueryFeatures()[featureIndex];
						queryFeature.draw(queryAppearance, pointFeatureRenderer);
					}
				}
				if (showModelPoints) {
					if (showModelFeatures) {
						const I32 modelFeatureIndex = pBelief->getIndices()[featureIndex];
						grasp::RBPose::Feature modelFeature = pBelief->getModelFeatures()[modelFeatureIndex];
						modelFeature.frame.multiply(grasp::to<Data>(dataPtr)->queryTransform, modelFeature.frame);
						modelFeature.draw(modelAppearance, pointFeatureRenderer);
					}
				}
			}
		}
		
		if (!grasp::to<Data>(dataPtr)->queryPoints.empty()) {
			pointFeatureRenderer.addAxes(grasp::to<Data>(dataPtr)->queryFrame, featureFrameSize);
			if (showQueryPoints)
				grasp::to<Data>(dataPtr)->draw(grasp::to<Data>(dataPtr)->queryPoints, pointFeatureRenderer);
		}
	}
}

//------------------------------------------------------------------------------

void spam::PosePlanner::createWithManipulation(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent) const {
	Player::create(inp, trajectory, silent);

	golem::Controller::State::Seq seq;

	// add manipulation trajectory
	if (actionManip.workspace) {
		WorkspaceChainCoord wcc;
		robot->getController()->chainForwardTransform(trajectory.back().cpos, wcc);
		Mat34 trn;
		trn.multiply(wcc[robot->getStateInfo().getChains().begin()], robot->getController()->getChains()[robot->getStateInfo().getChains().begin()]->getReferencePose());
		trn.multiply(actionManip, trn);
		robot->findTrajectory(trajectory.back(), nullptr, &trn, this->trjDuration, seq);
	}

	if (seq.empty())
		seq.push_back(trajectory.back());
	for (size_t i = 0; i < (size_t)robot->getStateInfo().getJoints().size() && i < actionManip.c.size(); ++i)
		seq.back().cpos[robot->getStateInfo().getJoints().begin() + i] += actionManip.c[i];

	trajectory.insert(trajectory.end(), ++seq.begin(), seq.end());
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
}

void spam::PosePlanner::create(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent) const {
	if (!grasp::to<Data>(currentDataPtr)->queryPoints.empty()) {
		context.write("Computing trajectory in a new frame...\n");
		golem::Controller::State::Seq seq;
		robot->transformTrajectory(grasp::to<Data>(currentDataPtr)->queryTransform, inp.begin(), inp.end(), seq);
		createWithManipulation(seq, trajectory, silent);
	}
	else
		createWithManipulation(inp, trajectory, silent);
}

//------------------------------------------------------------------------------

//void spam::PosePlanner::profile(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
//	if (!grasp::to<Data>(queryDataPtr)->queryPoints.empty()) {
//		context.write("PosePlanner::profile(): Computing trajectory in a new frame...\n");
//		golem::Controller::State::Seq seq;
//		//Mat34 trn;
//		//trn.setInverse(grasp::to<Data>(dataPtr)->actionFrame);
//		robot->transformTrajectory(grasp::to<Data>(dataPtr)->actionFrame, inp.begin(), inp.end(), seq);
//		grasp::Player::profile(seq, dur, grasp::to<Data>(dataPtr)->action);
//	}
//	else
//		grasp::Player::profile(dataPtr, inp, dur, idle);
//}
//
//void spam::PosePlanner::profileManip(Data::Map::iterator dataPtr, const golem::Controller::State::Seq& inp, golem::SecTmReal dur, golem::SecTmReal idle) const {
//	if (!inp.empty()) {
//		profile(dataPtr, inp, dur, idle);
//		return;
//	}
//	
//	golem::Controller::State::Seq& out = grasp::to<Data>(dataPtr)->action;
//	if (out.empty())
//		throw Message(Message::LEVEL_ERROR, "PosePlanner::profile(): At least one waypoints required");
//
//	golem::Controller::State::Seq seq;
//	
//	if (actionManip.workspace) {
//		WorkspaceChainCoord wcc;
//		robot->getController()->chainForwardTransform(out.back().cpos, wcc);
//		Mat34 trn;
//		trn.multiply(wcc[robot->getStateInfo().getChains().begin()], robot->getController()->getChains()[robot->getStateInfo().getChains().begin()]->getReferencePose());
//		trn.multiply(trn, actionManip);
//		robot->createTrajectory(out.back(), nullptr, &trn, dur, seq);
//	}
//	
//	if (seq.empty())
//		seq.push_back(out.back());
//	for (size_t i = 0; i < (size_t)robot->getStateInfo().getJoints().size() && i < actionManip.c.size(); ++i)
//		seq.back().cpos[robot->getStateInfo().getJoints().begin() + i] += actionManip.c[i];
//
//	out.insert(out.end(), ++seq.begin(), seq.end());
//	out.push_back(Controller::State(out.back(), out.back().t + idle));
//}

grasp::Cloud::PointSeqMap::iterator PosePlanner::getPointsTrn(Data::Map::iterator dataPtr, Mat34 &trn) {
	for (grasp::Cloud::PointSeqMap::iterator p = grasp::to<Data>(dataPtr)->points.begin(); p != grasp::to<Data>(dataPtr)->points.end(); ++p) 
		grasp::Cloud::transform(trn, p->second, p->second);
	return grasp::Director::getPoints(dataPtr);
}


void spam::PosePlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'P':
		switch (waitKey("MQLS", "Press a key to create (M)odel/(Q)uery/(L)oad/(S)et object...")) {
		case 'M':
		{
			// model create
			if (waitKey("YN", "Load model from file (Y/N)...") == 'Y') {
				createModelFromFile(*pBelief, modelPoints);
				// update model settings
				modelDataPtr = dataPtr;
				modelFrame.setId();
			}
			else {
				grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
				context.write("Creating %s model...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
				pBelief->createModel(points->second);
				// update model settings
				modelDataPtr = dataPtr;
				modelFrame = Belief::createFrame(points->second);
				modelPoints = points->second;
			}
			grasp::to<Data>(dataPtr)->modelFrame = modelFrame;
			grasp::to<Data>(dataPtr)->modelPoints = modelPoints;
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
			// compute transformation model -> query
			const grasp::RBPose::Sample trn = pBelief->createHypotheses(modelPoints, modelFrame);
			grasp::to<Data>(dataPtr)->queryTransform = trn.toMat34();
			// update query settings
			resetDataPointers();
			queryDataPtr = dataPtr;
			grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->queryTransform, modelFrame);
			grasp::Cloud::transform(grasp::to<Data>(dataPtr)->queryTransform, modelPoints, grasp::to<Data>(dataPtr)->queryPoints);
			// update trajectories
			auto ptr = getPtr<Data>(modelDataPtr);
			if (ptr == nullptr || ptr->trajectory.empty())
				context.warning("Model trajectories not available!\n");
			else if (grasp::to<Data>(dataPtr)->trajectory.empty() || waitKey("YN", "Overwrite existing query trajectories (Y/N)...") == 'Y')
				grasp::to<Data>(dataPtr)->trajectory = ptr->trajectory;
			// done
			context.debug("Copy belief in data and render density function\n");
			grasp::to<Data>(dataPtr)->poses.clear();
			grasp::to<Data>(dataPtr)->poses = pBelief->getSamples();
			grasp::to<Data>(dataPtr)->hypotheses.clear();
			grasp::to<Data>(dataPtr)->hypotheses = pBelief->getHypothesesToSample();
//			renderUncertainty(grasp::to<Data>(dataPtr)->poses);

			context.info("<trn v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\"/>\n", trn.p.x, trn.p.y, trn.p.z, trn.q.q0, trn.q.q1, trn.q.q2, trn.q.q3);
			context.write("Done!\n");
			showSamplePoints = true; // shows hypotheses and mean pose
			showQueryPoints = true;
			renderData(dataPtr);
			return;
		}
		case 'L':
		{
			// compute model and model frame
			grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
			context.write("Loading %s model...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
//			pBelief->createModel(points->second);
			// update model settings
//			modelDataPtr = dataPtr;
			modelFrame = grasp::to<Data>(dataPtr)->modelFrame;
			modelPoints = grasp::to<Data>(dataPtr)->modelPoints; // points->second;
			resetDataPointers();
			renderData(dataPtr);

			// query create
			context.write("Load %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
			// update query settings
			queryDataPtr = dataPtr;
			pBelief->set(grasp::to<Data>(dataPtr)->poses, grasp::to<Data>(dataPtr)->hypotheses, modelFrame, modelPoints);
			showSamplePoints = true; // shows hypotheses and mean pose
			//showQueryPoints = true;

			if (waitKey("YN", "Do you want to load the simulated object? (Y/N)") == 'Y') {
				grasp::to<Data>(dataPtr)->simulateObjectPose.clear();
				grasp::to<Data>(dataPtr)->simulateObjectPose.reserve(modelPoints.size());
				for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
					grasp::Cloud::Point p = *point;
					grasp::Cloud::setColour(golem::RGBA::MAGENTA, p);
					grasp::to<Data>(dataPtr)->simulateObjectPose.push_back(p);
				}
				grasp::Cloud::transform(grasp::to<Data>(dataPtr)->queryTransform, grasp::to<Data>(dataPtr)->simulateObjectPose, grasp::to<Data>(dataPtr)->simulateObjectPose);
				grasp::Cloud::transform(objectTrn, grasp::to<Data>(dataPtr)->simulateObjectPose, grasp::to<Data>(dataPtr)->simulateObjectPose);
				context.write("done.\n");
				//showObject = true;
			}

//			renderUncertainty(grasp::to<Data>(dataPtr)->poses);
			// done
			context.write("Done!\n");
			renderData(dataPtr);
			return;
		}
		case 'S':
			context.write("Storing simulated pose for the object to grasp...\n");
			if (modelPoints.empty()) {
				context.write("Error: No model loaded.\n");
				return;
			}
			grasp::to<Data>(dataPtr)->simulateObjectPose.clear();
			grasp::to<Data>(dataPtr)->simulateObjectPose.reserve(modelPoints.size());
			for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
				grasp::Cloud::Point p = *point;
				grasp::Cloud::setColour(golem::RGBA::MAGENTA, p);
				grasp::to<Data>(dataPtr)->simulateObjectPose.push_back(p);
			}
			grasp::Cloud::transform(grasp::to<Data>(dataPtr)->queryTransform, grasp::to<Data>(dataPtr)->simulateObjectPose, grasp::to<Data>(dataPtr)->simulateObjectPose);
			grasp::Cloud::transform(objectTrn, grasp::to<Data>(dataPtr)->simulateObjectPose, grasp::to<Data>(dataPtr)->simulateObjectPose);
			context.write("done.\n");
			//showObject = true;
			renderData(dataPtr);
			return;
		}
		return;
		// hand collision debug
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
	case '=':
		showObject = showSamplePoints && showMeanHypothesis ? !showObject : showObject;
		showMeanHypothesis = showSamplePoints && !showObject ? !showMeanHypothesis : showObject ? !showMeanHypothesis : showMeanHypothesis;
		showSamplePoints = !showMeanHypothesis && !showObject ? !showSamplePoints : showObject ? !showSamplePoints : showSamplePoints;
		context.write("Hypotheses distribution %s\nShow Mean Pose %s\nObject Points %s\n-----------\n", showSamplePoints ? "ON" : "OFF", showMeanHypothesis ? "ON" : "OFF", showObject ? "ON" : "OFF");
		renderData(dataPtr);
		return;
	case '+':
		showQueryPoints = !showQueryPoints;
		context.write("Query points %s\n", showQueryPoints ? "ON" : "OFF");
		renderData(dataPtr);
		return;
	}
//	case 'M':
//	{
//		context.write("model trn <%f %f %f>\n", modelTrn.p.x, modelTrn.p.y, modelTrn.p.z);
//		grasp::Cloud::PointSeqMap::iterator points = getPointsTrn(dataPtr, modelTrn);
//		// model create
//		context.write("Creating %s model...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
//		pBelief->createModel(points->second);
//
//		// update model settings
//		modelDataPtr = dataPtr;
//		modelFrame = Belief::createFrame(points->second);
//		context.write("model frame <%f %f %f>\n", modelFrame.p.x, modelFrame.p.y, modelFrame.p.z);
//		modelPoints = points->second;
//		resetDataPointers();
//
//		// done
//		context.write("Done!\n");
//		if (screenCapture) universe.postScreenCaptureFrames(-1);
//		renderData(dataPtr);
//		::Sleep(100);
//		if (screenCapture) universe.postScreenCaptureFrames(0);	
//		return;
//	}
//	case 'Q':
//	{
//		grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
//		// query create
//		context.write("Creating %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
//		pBelief->createQuery(points->second);
//		// create hypotheses and return the action frame for this query
//		const grasp::RBPose::Sample frame = pBelief->createHypotheses(modelPoints, modelFrame);
//		//// create pose distribution
//		//grasp::to<Data>(dataPtr)->poses.clear();
//		//grasp::to<Data>(dataPtr)->poses.resize(numPoses);
//		//for (size_t i = 0; i < numPoses; ++i)
//		//	grasp::to<Data>(dataPtr)->poses.push_back(grasp::RBPose::Sample(pRBPose->sample()));
//		// create hypothesis distribution (first element is the MLE)
//		//grasp::to<Data>(dataPtr)->hypotheses.clear();
//		//grasp::to<Data>(dataPtr)->hypotheses.resize(numHypotheses);
//		//grasp::to<Data>(dataPtr)->hypotheses.push_back(frame);
//		//for (size_t i = 1; i < numHypotheses; ++i)
//		//	grasp::to<Data>(dataPtr)->hypotheses.push_back(*grasp::RBPose::Sample::sample<golem::Ref1, grasp::RBPose::Sample::Seq::const_iterator>(grasp::to<Data>(dataPtr)->poses, rand));
//
//		grasp::to<Data>(dataPtr)->actionFrame = frame.toMat34();
//		context.write("action frame <%f %f %f>\n", grasp::to<Data>(dataPtr)->actionFrame.p.x, grasp::to<Data>(dataPtr)->actionFrame.p.y, grasp::to<Data>(dataPtr)->actionFrame.p.z);
//		// update query settings
//		resetDataPointers();
//		queryDataPtr = dataPtr;
//		grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->actionFrame, modelFrame);
//		grasp::to<Data>(dataPtr)->actionApproach = grasp::to<Data>(modelDataPtr)->actionApproach;
//		grasp::to<Data>(dataPtr)->actionManip = grasp::to<Data>(modelDataPtr)->actionManip;
//		grasp::to<Data>(dataPtr)->action = grasp::to<Data>(modelDataPtr)->action;
//		grasp::Cloud::transform(grasp::to<Data>(dataPtr)->actionFrame, modelPoints, grasp::to<Data>(dataPtr)->queryPoints);
//		// done
//		context.info("<pose v1=\"%.7f\" v2=\"%.7f\" v3=\"%.7f\" q0=\"%.7f\" q1=\"%.7f\" q2=\"%.7f\" q3=\"%.7f\"/>\n", frame.p.x, frame.p.y, frame.p.z, frame.q.q0, frame.q.q1, frame.q.q2, frame.q.q3);
//		
//		context.debug("Copy belief in data\n");
//		grasp::to<Data>(dataPtr)->poses.clear();
//		grasp::to<Data>(dataPtr)->poses = pBelief->getSamples();
//		grasp::to<Data>(dataPtr)->hypotheses.clear();
//		grasp::to<Data>(dataPtr)->hypotheses = pBelief->getHypothesesToSample();
//		renderUncertainty(grasp::to<Data>(dataPtr)->poses);
//
//		context.write("Done!\n");
//		showSamplePoints = true;
//		if (screenCapture) universe.postScreenCaptureFrames(-1);
//		renderData(dataPtr);
//		::Sleep(100);
//		if (screenCapture) universe.postScreenCaptureFrames(0);	
//		return;
//	}
//	case 'T':
//	{
//		// update approach and manip actions
//		context.write("Update approach actions (size %d)...\n", grasp::to<Data>(modelDataPtr)->actionApproach.size());
//		if (!grasp::to<Data>(modelDataPtr)->actionApproach.empty() && modelDataPtr._Ptr != NULL && queryDataPtr._Ptr != NULL) {
//			for(grasp::RobotState::List::iterator state = grasp::to<Data>(dataPtr)->actionApproach.begin(); state != grasp::to<Data>(dataPtr)->actionApproach.end(); ++state) {
//				const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//				
//				GenWorkspaceChainState gwcs;
//				gwcs.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
//				//grasp::Robot::State pose = robot->recvState();
//				//for (Configspace::Index i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().begin(); ++i)
//				//	pose.config.cpos[i] = state->config.cpos[i];
//
//				robot->getController()->chainForwardTransform(state->config.cpos, gwcs.wpos);
////				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
//				context.write("pose frame to teach pose (global) [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [0.0, 0.0, 0.0, 1.0]]\n", 
//					gwcs.wpos[armChain].R.m11, gwcs.wpos[armChain].R.m12, gwcs.wpos[armChain].R.m13, gwcs.wpos[armChain].p.x, 
//					gwcs.wpos[armChain].R.m21, gwcs.wpos[armChain].R.m22, gwcs.wpos[armChain].R.m23, gwcs.wpos[armChain].p.y, 
//					gwcs.wpos[armChain].R.m31, gwcs.wpos[armChain].R.m32, gwcs.wpos[armChain].R.m33, gwcs.wpos[armChain].p.z);
//				r = gwcs.wpos[armChain];
//				Mat34 poseFrameInv, graspFrame, graspFrameInv;
//				poseFrameInv.setInverse(gwcs.wpos[armChain]);
//				graspFrame.multiply(poseFrameInv, grasp::to<Data>(dataPtr)->queryFrame);
//				graspFrameInv.setInverse(graspFrame);
////				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
//				gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
//				mygraspFrame = graspFrame;
//				r2 = gwcs.wpos[armChain];
//				showTest = true;
//				renderData(dataPtr);
//
//				GenWorkspaceChainState wend;
//				wend.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
//				wend.wpos[armChain] = gwcs.wpos[armChain];
//				context.write("grasp frame to model (global) [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [0.0, 0.0, 0.0, 1.0]]\n", 
//					wend.wpos[armChain].R.m11, wend.wpos[armChain].R.m12, wend.wpos[armChain].R.m13, wend.wpos[armChain].p.x, 
//					wend.wpos[armChain].R.m21, wend.wpos[armChain].R.m22, wend.wpos[armChain].R.m23, wend.wpos[armChain].p.y, 
//					wend.wpos[armChain].R.m31, wend.wpos[armChain].R.m32, wend.wpos[armChain].R.m33, wend.wpos[armChain].p.z);
//				// lock controller
//				golem::CriticalSectionWrapper csw(robot->getControllerCS());
//				// Find end position
//				//grasp::Robot::State pose = robot->recvState();
//				//pose.config = state->config;
//				//pose.command = state->command;
//				if (!robot->getPlanner()->findTarget(state->config, wend, state->config))
//					throw Message(Message::LEVEL_CRIT, "RagPlanner:Creating Model: unable to find grasp configuration");
//				state->command = state->config;
//				// error
//				WorkspaceChainCoord wcc;
//				robot->getController()->chainForwardTransform(state->config.cpos, wcc);
////				wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
//				context.write("New grasp frame to model (global) [[%f, %f, %f, %f], [%f, %f, %f, %f], [%f, %f, %f, %f], [0.0, 0.0, 0.0, 1.0]]\n", 
//					wcc[armChain].R.m11, wcc[armChain].R.m12, wcc[armChain].R.m13, wcc[armChain].p.x, 
//					wcc[armChain].R.m21, wcc[armChain].R.m22, wcc[armChain].R.m23, wcc[armChain].p.y, 
//					wcc[armChain].R.m31, wcc[armChain].R.m32, wcc[armChain].R.m33, wcc[armChain].p.z);
//				const grasp::RBDist error(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(wend.wpos[armChain]));
//				context.write("RagPlanner:Creating Model: Pose error: lin=%.9f, ang=%.9f\n", error.lin, error.ang);
//			}
//		}
//		return;
//	}
//	case 'X':
//	{
//		r.setId();
//		mygraspFrame.setId();
//		r.multiply(grasp::to<Data>(dataPtr)->actionFrame, r2);
//		r2.setId();
//		showTest = true;
//		renderData(dataPtr);
//		return;
//	}
//	case '4':
//		showModelPoints = !showModelPoints;
//		context.write("Model points %s\n", showModelPoints ? "ON" : "OFF");
//		renderData(dataPtr);
//		return;
//	case '5':
//		showModelFeatures = !showModelFeatures;
//		featureIndex = -1;
//		context.write("Model features %s\n", showModelFeatures ? "ON" : "OFF");
//		renderData(dataPtr);
//		return;
//	case '6':
//		showQueryDistrib = !showQueryDistrib;
//		context.write("Query distribution %s\n", showQueryDistrib ? "ON" : "OFF");
//		renderData(dataPtr);
//		return;
//	case '7':
//		showSamplePoints = !showSamplePoints;
//		context.write("Hypotheses distribution %s\n", showSamplePoints ? "ON" : "OFF");
//		renderData(dataPtr);
//		return;
//
//	}

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

	try {
		XMLData(val.modelTrn, xmlcontext->getContextFirst("model_trn_frame"), create);
		XMLData(val.objectTrn, xmlcontext->getContextFirst("object_points_trn"), create);
	}
	catch (const golem::MsgXMLParser& msg) { context->write("%s\n", msg.str().c_str());  }

	//golem::XMLData("num_poses", val.numPoses, xmlcontext);
	//golem::XMLData("num_hypotheses", val.numHypotheses, xmlcontext);
	golem::XMLData("distrib_samples", val.distribSamples, xmlcontext->getContextFirst("appearance"), create);
	XMLData(val.modelAppearance, xmlcontext->getContextFirst("appearance model"), create);
	XMLData(val.queryAppearance, xmlcontext->getContextFirst("appearance query"), create);

//	XMLData("screen_capture", val.screenCapture, xmlcontext);

	XMLData(val.actionManip, xmlcontext->getContextFirst("action_manip"), create);
}

//------------------------------------------------------------------------------

//const char spam::PosePlanner::Data::headerName [] = "spam::PosePlanner::Data";
//const golem::U32 spam::PosePlanner::Data::headerVersion = 1;

template <> void golem::Stream::read(spam::PosePlanner::Data &trialData) const {
	//char name[sizeof(trialData.headerName)];
	//read(name, sizeof(trialData.headerName));
	//name[sizeof(trialData.headerName) - 1] = '\0';
	//if (strncmp(trialData.headerName, name, sizeof(trialData.headerName)) != 0)
	//	throw Message(Message::LEVEL_CRIT, "Stream::read(spam::TrialData&): Unknown file name: %s", name);

	//golem::U32 version;
	//*this >> version;
	//if (version != trialData.headerVersion)
	//	throw Message(Message::LEVEL_CRIT, "Stream::read(spam::TrialData&): Unknown file version: %d", version);

	//trialData.approachAction.clear();
	//read(trialData.approachAction, trialData.approachAction.begin(), grasp::RobotState(trialData.controller));
	//trialData.manipAction.clear();
	//read(trialData.manipAction, trialData.manipAction.begin(), grasp::RobotState(trialData.controller));

	//trialData.approachWithdraw.clear();
	//read(trialData.approachWithdraw, trialData.approachWithdraw.begin(), grasp::RobotState(trialData.controller));
	////trialData.action.clear();
	////read(trialData.action, trialData.action.begin(), trialData.controller.createState());

	trialData.poses.clear();
	read(trialData.poses, trialData.poses.begin());

	trialData.hypotheses.clear();
	read(trialData.hypotheses, trialData.hypotheses.begin());
}

template <> void golem::Stream::write(const spam::PosePlanner::Data &trialData) {
	//*this << trialData.headerName << trialData.headerVersion;

	//write(trialData.approachAction.begin(), trialData.approachAction.end());
	//write(trialData.manipAction.begin(), trialData.manipAction.end());

	//write(trialData.approachWithdraw.begin(), trialData.approachWithdraw.end());
	////write(trialData.action.begin(), trialData.action.end());
	write(trialData.poses.begin(), trialData.poses.end());
	write(trialData.hypotheses.begin(), trialData.hypotheses.end());
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

#ifdef _SPAM_POSE_MAIN_
int main(int argc, char *argv[]) {
	return spam::PosePlannerApp().main(argc, argv);
}
#endif // _SPAM_POSEPLANNER_MAIN_