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

#include <Golem/Math/Rand.h>
#include <Grasp/Grasp/Model.h>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>
#include <Grasp/Core/Import.h>
#include <Grasp/App/Player/Data.h>
#include <Golem/UI/Data.h>
#include <Grasp/Core/RBPose.h>

using namespace golem;
using namespace grasp;
using namespace spam;

//------------------------------------------------------------------------------

//void spam::PosePlanner::Data::xmlData(golem::XMLContext* context, bool create) const {
//	try {
//		if (!create || !queryPoints.empty()) {
//			golem::XMLData(const_cast<golem::Mat34&>(queryTransform), context->getContextFirst("query_transform", create), create);
//			golem::XMLData(const_cast<golem::Mat34&>(queryFrame), context->getContextFirst("query_frame", create), create);
////			xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(simulateObjectPose), std::string("object_points"), context, create);
//			golem::XMLData(const_cast<golem::Mat34&>(modelFrame), context->getContextFirst("model_frame", create), create);
//		}
//		if (!poses.empty() && create) {
//			const std::string name = "belief_pdf";
//			std::stringstream str;
//			str << "Save poses\n";
//			size_t id = 1;
//			for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
//				str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
//			str << "---------------------------------------------------------\n";
//			std::printf("%s", str.str().c_str());
//			xmlDataSave(context->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "pdf", extSamples.c_str()), poses);
//		}
//		if (!hypotheses.empty() && create){
//			const std::string name = "belief_hypotheses";
//			std::stringstream str;
//			str << "Save hypotheses\n";
//			size_t id = 1;
//			for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
//				str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
//			std::printf("%s", str.str().c_str());
//			str << "---------------------------------------------------------\n";
//			xmlDataSave(context->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "hypotheses", extSamples.c_str()), hypotheses);
//		}
//		if (!create) {
//			const std::string name = "belief_pdf";
//			xmlDataLoad(context->getContextFirst(name.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(poses), grasp::RBPose::Sample());
//			std::stringstream str;
//			str << "Load poses\n";
//			size_t id = 1;
//			for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
//				str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
//			str << "---------------------------------------------------------\n";
//			const std::string name1 = "belief_hypotheses";
//			xmlDataLoad(context->getContextFirst(name1.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(hypotheses), grasp::RBPose::Sample());
//			id = 1;
//			str << "Load hypotheses\n";
//			for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
//				str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
//			str << "---------------------------------------------------------\n";
//			std::printf("%s", str.str().c_str());
//		}
//
//	}
//	catch (const golem::MsgXMLParser& msg) {
//		std::printf("%s\n", msg.what());
//		if (create)
//			printf("%s\n", msg.str().c_str());
//	}
//	try {
//		xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(modelPoints), std::string("model_points"), context, create);
//	}
//	catch (const golem::MsgXMLParser& msg) {
//		std::printf("%s\n", msg.what());
//		if (create)
//			printf("%s\n", msg.str().c_str());
//	}
//	try {
//		xmlDataCloud(const_cast<grasp::Cloud::PointSeq&>(queryPoints), std::string("query_points"), context, create);
//	}
//	catch (const golem::MsgXMLParser& msg) {
//		std::printf("%s\n", msg.what());
//		if (create)
//			printf("%s\n", msg.str().c_str());
//	}
//	grasp::Player::Data::xmlData(context, create);
//}
//
//grasp::Director::Data::Ptr spam::PosePlanner::createData() const {
//	return Data::Ptr(new Data(*grasp::to<Data>(data))); // assuming data has been initialised properly
//}

grasp::data::Data::Ptr spam::PosePlanner::Data::Desc::create(golem::Context &context) const {
	grasp::data::Data::Ptr data(new PosePlanner::Data(context));
	static_cast<PosePlanner::Data*>(data.get())->create(*this);
	return data;
}

spam::PosePlanner::Data::Data(golem::Context &context) : grasp::Player::Data(context), owner(nullptr) {
}

void spam::PosePlanner::Data::create(const Desc& desc) {
	Player::Data::create(desc);

	queryTransform.setId();
	queryFrame.setId();
	queryPoints.clear();

	simulateObjectPose.clear();

	modelFrame.setId();
	modelPoints.clear();

	poses.clear();
	hypotheses.clear();

	extSamples = ".rbs";

	mode = MODE_DEFAULT;

	actionType = action::NONE_ACTION;
	stratType = Strategy::NONE_STRATEGY;
}

void spam::PosePlanner::Data::setOwner(grasp::Manager* owner) {
	grasp::Player::Data::setOwner(owner);
	this->owner = grasp::is<PosePlanner>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::Data::setOwner(): unknown data owner");
}

void spam::PosePlanner::Data::createRender() {
	Player::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->getCS());
		owner->modelRenderer.reset();
		owner->objectRenderer.reset();
		owner->beliefRenderer.reset();

		// model/query
		const grasp::Vec3Seq& vertices = mode == MODE_MODEL ? modelVertices : queryVertices;
		const grasp::TriangleSeq& triangles = mode == MODE_MODEL ? modelTriangles : queryTriangles;
		const golem::Mat34& frame = mode == MODE_MODEL ? modelFrame : queryFrame;
		owner->modelRenderer.setColour(owner->modelColourSolid);
		owner->modelRenderer.addSolid(vertices.data(), (U32)vertices.size(), triangles.data(), (U32)triangles.size());
		owner->modelRenderer.setColour(owner->modelColourWire);
		owner->modelRenderer.addWire(vertices.data(), (U32)vertices.size(), triangles.data(), (U32)triangles.size());
		if (!vertices.empty() && !triangles.empty())
			owner->modelRenderer.addAxes3D(frame, Vec3(0.2));	

		const bool showmodel = !modelPoints.empty() && queryPoints.empty();
		const bool showquery = !queryPoints.empty();
		printf("renderData showmodel %s showquery %s showSamplePoints %s\n", showmodel ? "ON" : "OFF", showquery ? "ON" : "OFF", owner->showSamplePoints ? "ON" : "OFF");

		if (showmodel || showquery) {
			if (showmodel && !showquery) {
				owner->objectRenderer.addAxes(modelFrame, owner->modelFrameSize);
			}

			if (showquery) {
				if (owner->showDistrPoints) {
					U8 id = 0;
					const Real max = owner->pBelief->maxWeight(true);
					//1 / (2 * (sigma*sqrt(2 * pi)))*exp(-.5*((x - mu) / sigma). ^ 2);
					const Real sigma = 0.2, norm = 1 / (2 * (sigma*Math::sqrt(2 * REAL_PI)));
					for (auto i = 0; i < owner->pBelief->getSamples().size(); ++i) {
						if (owner->pBelief->getSamples().size() > 100 && i % 10 != 0) continue;
						grasp::RBPose::Sample &j = owner->pBelief->getSamples()[i];
						owner->beliefRenderer.addAxes(j.toMat34() * modelFrame, owner->distrFrameSize*owner->pBelief->normalise(j));
						owner->sampleAppearance.colourOverride = true;
						const Real weight = owner->pBelief->normalise(j) / max;
						const Real red = norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)) * 255;
						//context.write("red = %f, U8(red)=%f\n", norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)), U8(norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma))));
						const Real green = norm*Math::exp(-.5*Math::sqr((weight - .5) / sigma)) * 255;
						const Real blue = norm*Math::exp(-.5*Math::sqr((weight - .0) / sigma)) * 255;
						///*j.weight*/Math::log2(1+pBelief->normalise(j)/max)*255;
						//context.debug("weight=%f, normalised=%f, red=%u, green=%u, blue=%u\n", pBelief->normalise(j), pBelief->normalise(j) / max, U8(red), U8(green), U8(blue));
						owner->sampleAppearance.colour = RGBA(U8(red), U8(green), U8(blue), 200);//RGBA(red > 255 ? 255 : U8(red), 255 - red < 0 ? 0 : U8(1-red), 0, 200);
						grasp::Cloud::PointSeq m;
						grasp::Cloud::transform(j.toMat34(), modelPoints, m);
						owner->sampleAppearance.draw(m, owner->beliefRenderer);
					}
				}
				if (owner->showSamplePoints) {
					owner->sampleAppearance.colourOverride = true;
					//context.write("showSamplePoints hypotheses size %d\n", pBelief->getHypotheses().size());
					for (Hypothesis::Seq::const_iterator i = owner->pBelief->getHypotheses().begin(); i != owner->pBelief->getHypotheses().end(); ++i) {
						owner->objectRenderer.addAxes((*i)->toRBPoseSampleGF().toMat34(), owner->hypothesisFrameSize);
						owner->sampleAppearance.colour = RGBA::BLACK;
						owner->sampleAppearance.draw((*i)->getCloud(), owner->objectRenderer);
						if (owner->showMeanHypothesis)
							break;
					}
				}
				if (owner->showQueryDistrib) {
					for (size_t i = 0; i < owner->distribSamples; ++i) {
						owner->beliefRenderer.addAxes(owner->pBelief->sample().toMat34() * modelFrame, owner->distrFrameSize);
					}
				}
			}
		}

		if (!queryPoints.empty()) {
			if (owner->showQueryPoints) {
				owner->beliefRenderer.addAxes(queryFrame, owner->queryFrameSize);
				owner->sampleAppearance.draw(queryPoints, owner->beliefRenderer);
			}
		}
	}
}

void spam::PosePlanner::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap) {
	grasp::data::Data::load(prefix, xmlcontext, handlerMap);

	try {
		dataName.clear();
		golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext), false);

		if (dataName.length() > 0) {
			FileReadStream frs((prefix + sepName + dataName).c_str());

			modelVertices.clear();
			frs.read(modelVertices, modelVertices.end());
			modelTriangles.clear();
			frs.read(modelTriangles, modelTriangles.end());
			frs.read(modelFrame);

			if (!queryPoints.empty()) {
				frs.read(queryTransform);
				frs.read(queryFrame);
				queryVertices.clear();
				frs.read(queryVertices, queryVertices.end());
				queryTriangles.clear();
				frs.read(queryTriangles, queryTriangles.end());
			}
			if (!poses.empty()) {
				const std::string name = "belief_pdf";
				std::stringstream str;
				str << "Save poses\n";
				size_t id = 1;
				for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
					str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
				str << "---------------------------------------------------------\n";
				std::printf("%s", str.str().c_str());
				frs.read(poses, poses.end());
				//				xmlDataSave(xmlcontext->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "pdf", extSamples.c_str()), poses);
			}
			if (!hypotheses.empty()){
				const std::string name = "belief_hypotheses";
				std::stringstream str;
				str << "Save hypotheses\n";
				size_t id = 1;
				for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
					str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
				std::printf("%s", str.str().c_str());
				str << "---------------------------------------------------------\n";
				frs.read(hypotheses, hypotheses.end());
				//				xmlDataSave(xmlcontext->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "hypotheses", extSamples.c_str()), hypotheses);
			}
//			if (!create) {
//				const std::string name = "belief_pdf";
//				//				xmlDataLoad(xmlcontext->getContextFirst(name.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(poses), grasp::RBPose::Sample());
//				std::stringstream str;
//				str << "Load poses\n";
//				size_t id = 1;
//				for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
//					str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
//				str << "---------------------------------------------------------\n";
//				const std::string name1 = "belief_hypotheses";
////				xmlDataLoad(xmlcontext->getContextFirst(name1.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(hypotheses), grasp::RBPose::Sample());
//				id = 1;
//				str << "Load hypotheses\n";
//				for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
//					str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
//				str << "---------------------------------------------------------\n";
//				std::printf("%s", str.str().c_str());
//			}
		}
	}
	catch (const golem::MsgXMLParser& msg) {
		std::printf("%s\n", msg.what());
	}
}

void spam::PosePlanner::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	grasp::data::Data::save(prefix, xmlcontext);

	if (dataName.length() > 0) {
		golem::XMLData("data_name", const_cast<std::string&>(dataName), xmlcontext, true);
		FileWriteStream fws((prefix + sepName + dataName).c_str());

		fws.write(modelVertices.begin(), modelVertices.end());
		fws.write(modelTriangles.begin(), modelTriangles.end());
		// rbpose from origin to model
		fws.write(modelFrame);

		fws.write(queryVertices.begin(), queryVertices.end());
		fws.write(queryTriangles.begin(), queryTriangles.end());
		// rbpose from modelFrame to queryFrame (for the mean pose)
		fws.write(queryTransform);
		// rbpose from origin to query (for the mean pose)
		fws.write(queryFrame);

		// high-dim pdf. set of poses
		if (!poses.empty())
			fws.write(poses.begin(), poses.end());
		// low-dim pdf. subset of poses
		if (!hypotheses.empty())
			fws.write(hypotheses.begin(), hypotheses.end());
	}
}

//------------------------------------------------------------------------------

void PosePlanner::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	RagGraphPlanner::Desc* pRagGraphPlanner(new RagGraphPlanner::Desc);
	(golem::Planner::Desc&)*pRagGraphPlanner = *uiPlannerDesc->pPlannerDesc;
	uiPlannerDesc->pPlannerDesc.reset(pRagGraphPlanner);
	uiPlannerDesc->pPlannerDesc = RagGraphPlanner::Desc::load(&context, xmlcontext->getContextFirst("player planner"));

	xmlcontext = xmlcontext->getContextFirst("pose_planner");

	FTDrivenHeuristic::Desc* pFTDrivenHeuristic(new FTDrivenHeuristic::Desc);
	(golem::Heuristic::Desc&)*pFTDrivenHeuristic = *uiPlannerDesc->pPlannerDesc->pHeuristicDesc;
	uiPlannerDesc->pPlannerDesc->pHeuristicDesc.reset(pFTDrivenHeuristic);
	spam::XMLData((FTDrivenHeuristic::Desc&)*uiPlannerDesc->pPlannerDesc->pHeuristicDesc, xmlcontext->getContextFirst("heuristic"));

	try {
		XMLData((grasp::RBPose::Desc&)*pRBPoseDesc, const_cast<golem::XMLContext*>(xmlcontext));
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
	
	Belief::Desc* pBeliefDesc(new Belief::Desc);
	(grasp::RBPose::Desc&)*pBeliefDesc = *pRBPoseDesc;
	pRBPoseDesc.reset(pBeliefDesc);
	spam::XMLData((Belief::Desc&)*pRBPoseDesc, const_cast<golem::XMLContext*>(xmlcontext));

	try {
		XMLData(modelTrn, xmlcontext->getContextFirst("model_trn_frame"));
//		XMLData(val.objectTrn, xmlcontext->getContextFirst("object_points_trn"), create);
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str());  }
	
	try {
		trialData->xmlData(xmlcontext->getContextFirst("trial_data"));
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
	
	//golem::XMLData("num_poses", val.numPoses, xmlcontext);
	//golem::XMLData("num_hypotheses", val.numHypotheses, xmlcontext);
	golem::XMLData("distrib_samples", distribSamples, xmlcontext->getContextFirst("appearance"));
//	XMLData(modelAppearance, xmlcontext->getContextFirst("appearance model"));
//	XMLData(queryAppearance, xmlcontext->getContextFirst("appearance query"));
	
	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("screen_capture", screenCapture, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", modelCamera, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_obj", modelItemObj, xmlcontext->getContextFirst("model"));

	modelScanPose.xmlData(xmlcontext->getContextFirst("model scan_pose"));
	golem::XMLData(modelColourSolid, xmlcontext->getContextFirst("model colour solid"));
	golem::XMLData(modelColourWire, xmlcontext->getContextFirst("model colour wire"));

	golem::XMLData("handler_trj", modelHandlerTrj, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_trj", modelItemTrj, xmlcontext->getContextFirst("model"));

	golem::XMLData("camera", queryCamera, xmlcontext->getContextFirst("query"));
	golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_obj", queryItemObj, xmlcontext->getContextFirst("query"));

	golem::XMLData("handler_trj", queryHandlerTrj, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_trj", queryItemTrj, xmlcontext->getContextFirst("query"));

	golem::XMLData("camera", objectCamera, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler_scan", objectHandlerScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("handler", objectHandler, xmlcontext->getContextFirst("object"));
	golem::XMLData("item_scan", objectItemScan, xmlcontext->getContextFirst("object"));
	golem::XMLData("item", objectItem, xmlcontext->getContextFirst("object"));
	objectScanPoseSeq.clear();
	XMLData(objectScanPoseSeq, objectScanPoseSeq.max_size(), xmlcontext->getContextFirst("object"), "scan_pose");
	objectFrameAdjustment.load(xmlcontext->getContextFirst("object frame_adjustment"));

	modelDescMap.clear();
	golem::XMLData(modelDescMap, modelDescMap.max_size(), xmlcontext->getContextFirst("model"), "model", false);
	contactAppearance.load(xmlcontext->getContextFirst("model appearance"));

	queryDescMap.clear();
	golem::XMLData(queryDescMap, queryDescMap.max_size(), xmlcontext->getContextFirst("query"), "query", false);

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
	manipulatorAppearance.load(xmlcontext->getContextFirst("manipulator appearance"));
}

//------------------------------------------------------------------------------

const std::string PosePlanner::Data::ModeName[MODE_QUERY + 1] = {
	"Model",
	"Query",
};

//------------------------------------------------------------------------------

spam::PosePlanner::PosePlanner(Scene &scene) : grasp::Player(scene), rand(context.getRandSeed()), pHeuristic(nullptr) {
}

spam::PosePlanner::~PosePlanner() {}
	
bool spam::PosePlanner::create(const Desc& desc) {
	grasp::Player::create(desc); // throws

	trial = desc.trialData;
	trial->controller = this->controller;
	trial->setup(context, rand);
	golem::mkdir(trial->path.c_str()); // make sure that the directory exists
	trialPtr = trialDataMap.end();

	pRBPose = desc.pRBPoseDesc->create(context); // throws
	pBelief = static_cast<Belief*>(pRBPose.get());

	grasp::Sensor::Map::const_iterator modelCameraPtr = sensorMap.find(desc.modelCamera);
	modelCamera = modelCameraPtr != sensorMap.end() ? is<Camera>(modelCameraPtr->second.get()) : nullptr;
	if (!modelCamera)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model pose estimation camera: %s", desc.modelCamera.c_str());
	grasp::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	if (!modelHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;
	modelItemObj = desc.modelItemObj;

	modelScanPose = desc.modelScanPose;
	modelColourSolid = desc.modelColourSolid;
	modelColourWire = desc.modelColourWire;

	grasp::data::Handler::Map::const_iterator modelHandlerTrjPtr = handlerMap.find(desc.modelHandlerTrj);
	modelHandlerTrj = modelHandlerTrjPtr != handlerMap.end() ? modelHandlerTrjPtr->second.get() : nullptr;
	if (!modelHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model trajectory handler: %s", desc.modelHandlerTrj.c_str());
	modelItemTrj = desc.modelItemTrj;

	grasp::Sensor::Map::const_iterator queryCameraPtr = sensorMap.find(desc.queryCamera);
	queryCamera = queryCameraPtr != sensorMap.end() ? is<Camera>(queryCameraPtr->second.get()) : nullptr;
	if (!queryCamera)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query pose estimation camera: %s", desc.queryCamera.c_str());
	grasp::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	if (!queryHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query data handler: %s", desc.queryHandler.c_str());
	queryItem = desc.queryItem;
	queryItemObj = desc.queryItemObj;

	queryColourSolid = desc.queryColourSolid;
	queryColourWire = desc.queryColourWire;

	grasp::data::Handler::Map::const_iterator queryHandlerTrjPtr = handlerMap.find(desc.queryHandlerTrj);
	queryHandlerTrj = queryHandlerTrjPtr != handlerMap.end() ? queryHandlerTrjPtr->second.get() : nullptr;
	if (!queryHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query trajectory handler: %s", desc.queryHandlerTrj.c_str());
	queryItemTrj = desc.queryItemTrj;

	grasp::Sensor::Map::const_iterator objectCameraPtr = sensorMap.find(desc.objectCamera);
	objectCamera = objectCameraPtr != sensorMap.end() ? is<Camera>(objectCameraPtr->second.get()) : nullptr;
	if (!objectCamera)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown object capture camera: %s", desc.objectCamera.c_str());
	grasp::data::Handler::Map::const_iterator objectHandlerScanPtr = handlerMap.find(desc.objectHandlerScan);
	objectHandlerScan = objectHandlerScanPtr != handlerMap.end() ? objectHandlerScanPtr->second.get() : nullptr;
	if (!objectHandlerScan)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown object (scan) data handler: %s", desc.objectHandlerScan.c_str());
	grasp::data::Handler::Map::const_iterator objectHandlerPtr = handlerMap.find(desc.objectHandler);
	objectHandler = objectHandlerPtr != handlerMap.end() ? objectHandlerPtr->second.get() : nullptr;
	if (!objectHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown object (process) data handler: %s", desc.objectHandler.c_str());
	objectItemScan = desc.objectItemScan;
	objectItem = desc.objectItem;
	objectScanPoseSeq = desc.objectScanPoseSeq;
	objectFrameAdjustment = desc.objectFrameAdjustment;

	// models
	modelMap.clear();
	for (Model::Desc::Map::const_iterator i = desc.modelDescMap.begin(); i != desc.modelDescMap.end(); ++i)
		modelMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));
	contactAppearance = desc.contactAppearance;

	// query densities
	queryMap.clear();
	for (Query::Desc::Map::const_iterator i = desc.queryDescMap.begin(); i != desc.queryDescMap.end(); ++i)
		queryMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));

		//	modelAppearance = desc.modelAppearance;
//	queryAppearance = desc.queryAppearance;
////	sampleAppearance.setToDefault();
//
//	featureFrameSize = desc.featureFrameSize;
//	distribFrameSize = desc.distribFrameSize;
	distribSamples = desc.distribSamples;

	modelFrameSize = Vec3(.1, .1, .1);
	queryFrameSize = Vec3(.1, .1, .1);
	hypothesisFrameSize = Vec3(.05, .05, .05);
	distrFrameSize = Vec3(.02, .02, .02);

	modelFrame.setId();
//	context.write("PosePlannenr::Create(): model trn <%f %f %f>\n", desc.modelTrn.p.x, desc.modelTrn.p.y, desc.modelTrn.p.z);
	modelTrn = desc.modelTrn;
//	objectTrn = desc.objectTrn;
//	modelDataPtr = getData().end();
	resetDataPointers();
	screenCapture = desc.screenCapture;

	// manipulator
	manipulator = desc.manipulatorDesc->create(*planner, desc.controllerIDSeq);
	manipulatorAppearance = desc.manipulatorAppearance;

	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner->getHeuristic());
	pHeuristic->setBelief(pBelief);
	pHeuristic->setManipulator(manipulator.get());

	myDesc = desc;

	//numPoses = desc.numPoses;
	//numHypotheses = desc.numHypotheses;
	
	scene.getHelp().insert(Scene::StrMapVal("080", "  M                                       model create\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  4                                       model points\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  5                                       model features\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  Q                                       query create\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  6                                       query distribution\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  7                                       hypotheses distribution\n"));

	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (M)odel/(Q)ery estimation...";
	}));
	menuCmdMap.insert(std::make_pair("PM", [=]() {
		// estimate
		try {
			(void)estimatePose(Data::MODE_MODEL);
		}
			catch (const Message &msg) {
			context.write("%s\n", msg.what());
		}
	// finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("PQ", [=]() {
		// estimate
		try {
			(void)estimatePose(Data::MODE_QUERY);
		}
		catch (const Message &msg) {
			context.write("%s\n", msg.what());
		}
		// finish
		context.write("Done!\n");
	}));
	menuCtrlMap.insert(std::make_pair("Z", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Belief state rendering options...";
	}));
	menuCmdMap.insert(std::make_pair("Z-", [=]() {
		// render hypotheses
		showQueryDistrib = !showQueryDistrib;
		context.write("Query (low-dim) distribution %s\n", showQueryDistrib ? "ON" : "OFF");
		createRender();
	}));
	menuCmdMap.insert(std::make_pair("Z_", [=]() {
		// render query distribution
		showDistrPoints = !showDistrPoints;
		context.write("Query (high-dim) distribution %s\n", showDistrPoints ? "ON" : "OFF");
		createRender();
	}));
	menuCmdMap.insert(std::make_pair("Z=", [=]() {
		showObject = showSamplePoints && showMeanHypothesis ? !showObject : showObject;
		showMeanHypothesis = showSamplePoints && !showObject ? !showMeanHypothesis : showObject ? !showMeanHypothesis : showMeanHypothesis;
		showSamplePoints = !showMeanHypothesis && !showObject ? !showSamplePoints : showObject ? !showSamplePoints : showSamplePoints;
		context.write("Hypotheses distribution %s\nShow Mean Pose %s\nObject Points %s\n-----------\n", showSamplePoints ? "ON" : "OFF", showMeanHypothesis ? "ON" : "OFF", showObject ? "ON" : "OFF");		
		createRender();
	}));
	menuCmdMap.insert(std::make_pair("Z+", [=]() {
		showQueryPoints = !showQueryPoints;
		context.write("Query points %s\n", showQueryPoints ? "ON" : "OFF");
		createRender();
	}));

	return true;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::render() const {
	Player::render();

	golem::CriticalSectionWrapper cswRenderer(getCS());
	beliefRenderer.render();
	modelRenderer.render();
	objectRenderer.render();
}

//------------------------------------------------------------------------------

void spam::PosePlanner::resetDataPointers() {
	//auto ptr = getPtr<Data>(queryDataPtr);
	//if (ptr != nullptr) ptr->queryPoints.clear();
//	queryDataPtr = getData().end();
	showModelPoints = false;
	showQueryPoints = false;
	showModelFeatures = false;
	showQueryDistrib = false;
	showSamplePoints = false;
	showDistrPoints = false;
	showMeanHypothesis = false;
	showObject = false;
	showPoints = true;
//	showTest = false;
	featureIndex = -1;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::createWithManipulation(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent) const {
	//Player::create(inp, trajectory, silent);

	//golem::Controller::State::Seq seq;

	//// add manipulation trajectory
	//if (actionManip.position || actionManip.orientation) {
	//	WorkspaceChainCoord wcc;
	//	robot->getController()->chainForwardTransform(trajectory.back().cpos, wcc);
	//	Mat34 trn;
	//	trn.multiply(wcc[robot->getStateInfo().getChains().begin()], robot->getController()->getChains()[robot->getStateInfo().getChains().begin()]->getReferencePose());
	//	trn.multiply(actionManip.w, trn);
	//	robot->findTrajectory(trajectory.back(), nullptr, &trn, this->trjDuration, seq);
	//}

	//if (seq.empty())
	//	seq.push_back(trajectory.back());
	//for (size_t i = 0; i < (size_t)robot->getStateInfo().getJoints().size() && i < actionManip.c.size(); ++i)
	//	seq.back().cpos[robot->getStateInfo().getJoints().begin() + i] += actionManip.c[i];

	//trajectory.insert(trajectory.end(), ++seq.begin(), seq.end());
	//trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
}

void spam::PosePlanner::create(const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& trajectory, bool silent) const {
	//if (!grasp::to<Data>(currentDataPtr)->queryPoints.empty()) {
	//	context.write("Computing trajectory in a new frame...\n");
	//	golem::Controller::State::Seq seq;
	//	robot->transformTrajectory(grasp::to<Data>(currentDataPtr)->queryTransform, inp.begin(), inp.end(), seq);
	//	createWithManipulation(seq, trajectory, silent);
	//}
	//else
	//	createWithManipulation(inp, trajectory, silent);
}

//------------------------------------------------------------------------------

grasp::data::Item::Map::iterator spam::PosePlanner::estimatePose(Data::Mode mode) {
	const std::string itemName = mode != Data::MODE_MODEL ? queryItem : modelItem;
	grasp::data::Handler* handler = mode != Data::MODE_MODEL ? queryHandler : modelHandler;

	grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(itemName);
	if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
		throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", itemName.c_str());

	// retrive point cloud with curvature
	data::ItemPointsCurv *pointsCurv = is<data::ItemPointsCurv>(ptr->second.get());
	if (!pointsCurv)
		throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not support PointdCurv interface.");
	grasp::Cloud::PointCurvSeq curvPoints = *pointsCurv->cloud;

	// copy as a vector of points in 3D
	Vec3Seq mPoints;
	mPoints.resize(curvPoints.size());
	I32 idx = 0;
	std::for_each(curvPoints.begin(), curvPoints.end(), [&](const Cloud::PointCurv& i){
		mPoints[idx++] = Cloud::getPoint<Real>(i);
	});
	
	// copy as a generic point+normal point cloud (Cloud::pointSeq)
	Cloud::PointSeq points;
	points.resize(curvPoints.size());
	Cloud::copy(curvPoints, points);

	if (mode == Data::MODE_MODEL) {
		pBelief->createModel(curvPoints);
		modelFrame = Belief::createFrame(mPoints);
		grasp::to<Data>(dataCurrentPtr)->modelFrame = modelFrame;
		grasp::to<Data>(dataCurrentPtr)->modelPoints = modelPoints = points;
		resetDataPointers();
		showModelPoints = true;
		to<Data>(dataCurrentPtr)->createRender();
	}
	else {
		if (modelPoints.empty())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): no model has been created.");
		pBelief->createQuery(points);
		// compute transformation model -> query
		const grasp::RBPose::Sample trn = pBelief->createHypotheses(modelPoints, modelFrame);
		grasp::to<Data>(dataCurrentPtr)->queryTransform = trn.toMat34();
		grasp::to<Data>(dataCurrentPtr)->queryFrame.multiply(grasp::to<Data>(dataCurrentPtr)->queryTransform, modelFrame);
		grasp::Cloud::transform(grasp::to<Data>(dataCurrentPtr)->queryTransform, modelPoints, grasp::to<Data>(dataCurrentPtr)->queryPoints);
		grasp::to<Data>(dataCurrentPtr)->poses.clear();
		grasp::to<Data>(dataCurrentPtr)->poses = pBelief->getSamples();
		grasp::to<Data>(dataCurrentPtr)->hypotheses.clear();
		grasp::to<Data>(dataCurrentPtr)->hypotheses = pBelief->getHypothesesToSample();
		
		if (option("YN", "Do you want to load the simulated object? (Y/N)") == 'Y') {
			grasp::to<Data>(dataCurrentPtr)->simulateObjectPose.clear();
			grasp::to<Data>(dataCurrentPtr)->simulateObjectPose.reserve(modelPoints.size());
			for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
				grasp::Cloud::Point p = *point;
				grasp::Cloud::setColour(golem::RGBA::MAGENTA, p);
				grasp::to<Data>(dataCurrentPtr)->simulateObjectPose.push_back(p);
			}
			grasp::Cloud::transform(grasp::to<Data>(dataCurrentPtr)->queryTransform, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
		}
		resetDataPointers();
		showSamplePoints = true; // shows hypotheses and mean pose
		showQueryPoints = true;	
		to<Data>(dataCurrentPtr)->createRender();
	}

	return ptr;
}

grasp::data::Item::Map::iterator spam::PosePlanner::objectCapture(const Data::Mode mode) {
	const std::string itemName = mode == Data::MODE_DEFAULT ? objectItem : mode != Data::MODE_MODEL ? queryItem : modelItem;

	data::Capture* capture = is<data::Capture>(objectHandlerScan);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", objectHandlerScan->getID().c_str());

	RenderBlock renderBlock(*this);
	data::Item::Map::iterator ptr;
	{
		golem::CriticalSectionWrapper cswData(getCS());
		data::Item::Ptr item = capture->capture(*objectCamera, [&](const grasp::TimeStamp*) -> bool { return true; });

		// Finally: insert object scan, remove old one
		data::Item::Map& itemMap = to<Data>(dataCurrentPtr)->itemMap;
		itemMap.erase(objectItemScan);
		ptr = itemMap.insert(itemMap.end(), data::Item::Map::value_type(itemName, item));
		Data::View::setItem(itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}

	return ptr;
}

// Process object image and add to data bundle
grasp::data::Item::Map::iterator spam::PosePlanner::objectProcess(const Data::Mode mode, grasp::data::Item::Map::iterator ptr) {
	const std::string itemName = mode == Data::MODE_DEFAULT ? objectItem : mode != Data::MODE_MODEL ? queryItem : modelItem;
	grasp::data::Handler* handler = mode == Data::MODE_DEFAULT ? objectHandler : mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	// generate features
	data::Transform* transform = is<data::Transform>(handler);
	if (!transform)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", handler->getID().c_str());

	data::Item::List list;
	list.insert(list.end(), ptr);
	data::Item::Ptr item = transform->transform(list);

	// insert processed object, remove old one
	RenderBlock renderBlock(*this);
	golem::CriticalSectionWrapper cswData(getCS());
	to<Data>(dataCurrentPtr)->itemMap.erase(itemName);
	ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemName, item));
	Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	return ptr;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::TrialData::Iteration::xmlData(golem::XMLContext* context, bool create) const {
	//if (!state->poses.empty() && create) {
	//	const std::string name = "belief_pdf";
	//	std::stringstream str;
	//	str << "Save poses\n";
	//	size_t id = 1;
	//	for (grasp::RBPose::Sample::Seq::const_iterator i = state->poses.begin(); i != state->poses.end(); ++i)
	//		str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
	//	str << "---------------------------------------------------------\n";
	//	std::printf("%s", str.str().c_str());
	//	xmlDataSave(context->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "pdf", extIter.c_str()), state->poses);
	//}
	//if (!state->hypotheses.empty() && create){
	//	const std::string name = "belief_hypotheses";
	//	std::stringstream str;
	//	str << "Save hypotheses\n";
	//	size_t id = 1;
	//	for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
	//		str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
	//	std::printf("%s", str.str().c_str());
	//	str << "---------------------------------------------------------\n";
	//	xmlDataSave(context->createContext(name.c_str()), grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), "hypotheses", extIter.c_str()), hypotheses);
	//}
	//if (!create) {
	//	const std::string name = "belief_pdf";
	//	xmlDataLoad(context->getContextFirst(name.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(poses), grasp::RBPose::Sample());
	//	std::stringstream str;
	//	str << "Load poses\n";
	//	size_t id = 1;
	//	for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
	//		str << "pose " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
	//	str << "---------------------------------------------------------\n";
	//	const std::string name1 = "belief_hypotheses";
	//	xmlDataLoad(context->getContextFirst(name1.c_str()), "", const_cast<grasp::RBPose::Sample::Seq&>(hypotheses), grasp::RBPose::Sample());
	//	id = 1;
	//	str << "Load hypotheses\n";
	//	for (grasp::RBPose::Sample::Seq::const_iterator i = hypotheses.begin(); i != hypotheses.end(); ++i)
	//		str << "hypotheses " << id++ << " " << i->p.x << " " << i->p.y << " " << i->p.z << "\n";
	//	str << "---------------------------------------------------------\n";
	//	std::printf("%s", str.str().c_str());
	//}


}

void spam::PosePlanner::TrialData::xmlData(golem::XMLContext* context, bool create) const {
	golem::XMLData("name", const_cast<std::string&>(name), context, create);
	golem::XMLData("path", const_cast<std::string&>(path), context, create);
	golem::XMLData("ext_trial", const_cast<std::string&>(extTrial), context, create);
	try {
		golem::XMLData("data_path", const_cast<std::string&>(dataPath), context, create);
	}
	catch (const golem::MsgXMLParser &msg) { printf("%s: %s\n", msg.what(), msg.str().c_str()); }

	golem::XMLData("enable", const_cast<bool&>(enable), context, create);
	golem::XMLData("silent", const_cast<bool&>(silent), context, create);

	XMLData(const_cast<grasp::RBDist&>(queryStdDev), create ? context->createContext("meta_query_transformation") : context->getContextFirst("meta_query_transformation"), create);
	XMLData(const_cast<grasp::RBDist&>(objPoseStdDev), create ? context->createContext("meta_obj_pose_transformation") : context->getContextFirst("meta_obj_pose_transformation"), create);

	try {
		XMLData(const_cast<grasp::RBCoord&>(queryPointsTrn), create ? context->createContext("query_transformation") : context->getContextFirst("query_transformation"), create);
		XMLData(const_cast<grasp::RBCoord&>(objectPoseTrn), create ? context->createContext("obj_pose_transformation") : context->getContextFirst("obj_pose_transformation"), create);

	}
	catch (const golem::MsgXMLParser& msg) {
		std::printf("%s\n", msg.what());
		if (create)
			printf("%s\n", msg.str().c_str());
	}

	// TRAJECTORIES
	golem::XMLData("ext_trajectory", const_cast<std::string&>(extTrajectory), context, create);

	if (!controller) {
		printf("TrialData::xmlData: no controller.\n");
		//return;
	}

	//const std::string name = "trajectory";
	//if (create) {
	//	for (grasp::RobotState::Map::const_iterator i = trajectories.begin(); i != trajectories.end(); ++i) {
	//		if (!i->second.empty()) 
	//			grasp::Director::Data::xmlDataSave(context->createContext(name.c_str()), i->first, grasp::makeString("%s%s%s%s%s%s", getName().c_str(), sepName.c_str(), name.c_str(), sepName.c_str(), i->first.c_str(), extTrajectory.c_str()), i->second);
	//	}
	//}
	//else {
	//	const_cast<grasp::RobotState::Map&>(trajectories).clear();
	//	std::pair<XMLContext::XMLContextMap::const_iterator, XMLContext::XMLContextMap::const_iterator> range = context->getContextMap().equal_range(name.c_str());
	//	for (XMLContext::XMLContextMap::const_iterator i = range.first; i != range.second; ++i) {
	//		try {
	//			grasp::RobotState::Map::value_type val;
	//			grasp::Director::Data::xmlDataLoad(const_cast<XMLContext*>(&i->second), this->path, const_cast<std::string&>(val.first), val.second, grasp::RobotState(*controller));
	//			//grasp::RobotState::Map::value_type val_(val.first, val.second);
	//			const_cast<grasp::RobotState::Map&>(trajectories).insert(val);
	//		}
	//		catch (const golem::MsgXMLParser&) {}
	//	}
	//}

}

PosePlanner::TrialData::Ptr spam::PosePlanner::createTrialData() {
	return TrialData::Ptr(new TrialData(*trial));
}

void spam::PosePlanner::TrialData::load() {
	// Load xml context and data
	xmlData(XMLParser::load(path)->getContextRoot()->getContextFirst("grasp trial_data", false), false);
	// check validity
	assertValid(grasp::Assert::Context("spam::PosePlanner::TrialData::load(): this->"));
}

void spam::PosePlanner::TrialData::save() const {
	// check validity
	assertValid(grasp::Assert::Context("spam::PosePlanner::TrialData::save(): this->"));
	// Create XML parser
	XMLParser::Ptr pParser = XMLParser::Desc().create();
	// Create xml context and save data
	golem::mkdir(path.c_str()); // make sure that the directory exists
	xmlData(pParser->getContextRoot()->getContextFirst("grasp trial_data", true), true);
	FileWriteStream fws(path.c_str());
	pParser->store(fws);
}


//------------------------------------------------------------------------------

//grasp::Cloud::PointSeqMap::iterator PosePlanner::getPointsTrn(Data::Map::iterator dataPtr, Mat34 &trn) {
//	for (grasp::Cloud::PointSeqMap::iterator p = grasp::to<Data>(dataPtr)->points.begin(); p != grasp::to<Data>(dataPtr)->points.end(); ++p) 
//		grasp::Cloud::transform(trn, p->second, p->second);
//	return grasp::Director::getPoints(dataPtr);
//}

//void PosePlanner::processPoints(Data::Map::const_iterator dataPtr, const Selection& selection, const bool silent) {
//	{
//		golem::CriticalSectionWrapper csw(csDataRenderer);
//		renderBounds(cloudDesc.objectRegionDesc, cloudDesc.regionColourSolid, cloudDesc.regionColourWire, boundsRenderer);
//		boundsRenderer.addAxes3D(golem::Mat34::identity(), Vec3(0.2));
//	}
//	grasp::ScopeGuard guard([&] {
//		golem::CriticalSectionWrapper csw(csDataRenderer);
//		boundsRenderer.reset();
//	});
//
//	// camera
//	const bool hasCamera = !cameraSeq.empty();
//
//	// transforms
//	typedef std::vector< std::pair<std::string, golem::Mat34> > TrnSeq;
//	TrnSeq trnSeq;
//	trnSeq.push_back(std::make_pair(std::string("None"), golem::Mat34::identity()));
//	trnSeq.push_back(std::make_pair(std::string("Custom"), golem::Mat34::identity()));
//	if (hasCamera) trnSeq.push_back(std::make_pair(std::string("Deformation"), golem::Mat34::identity()));
//	trnSeq.insert(trnSeq.end(), transforms.begin(), transforms.end());
//	TrnSeq::const_iterator trnPtr = silent ? trnSeq.begin() : select(trnSeq.begin(), trnSeq.end(), "Transforms:\n", [](TrnSeq::const_iterator ptr) -> const std::string&{
//		return ptr->first;
//	});
//	const size_t trnIndex = trnPtr - trnSeq.begin();
//
//	// mode
//	const bool process = trnIndex == 0;
//	const bool custom = trnIndex == 1;
//	const bool deformation = hasCamera && trnIndex == 2;
//
//	// transform
//	Mat34 trn = trnPtr->second;
//
//	// custom transform
//	if (custom) {
//		readNumber("x [m] = ", trn.p.x);
//		readNumber("y [m] = ", trn.p.y);
//		readNumber("z [m] = ", trn.p.z);
//		Real roll, pitch, yaw;
//		trn.R.toEuler(roll, pitch, yaw);
//		roll = Math::radToDeg(roll);	readNumber("roll  [deg] = ", roll);
//		pitch = Math::radToDeg(pitch);	readNumber("pitch [deg] = ", pitch);
//		yaw = Math::radToDeg(yaw);		readNumber("yaw   [deg] = ", yaw);
//		trn.R.fromEuler(Math::degToRad(roll), Math::radToDeg(pitch), Math::radToDeg(yaw));
//	}
//
//	// deformation map
//	if (deformation) {
//		cameraIndex = (U32)(select(cameraSeq.begin(), cameraSeq.end(), "Available Cameras:\n", [](grasp::Recorder::Seq::const_iterator ptr) -> const std::string&{
//			return (*ptr)->getCamera()->getName();
//		}, cameraIndex) - cameraSeq.begin());
//	}
//
//	// all data?
//	const bool allData = silent ? false : getData().size() > 1 && waitKey("YN", "Process all data (Y/N)...") == 'Y';
//	Data::Map::const_iterator end = allData ? getData().end() : dataPtr, begin = allData ? getData().begin() : end++;
//
//	// iterate
//	for (Data::Map::const_iterator j = begin; j != end; ++j) {
//		if (allData)
//			context.write("Processing %s...\n", j->first.c_str());
//
//		grasp::Cloud::LabelMap labelMap;
//		grasp::Cloud::RawPointSeqPtrMultiMap inp;
//		std::string suffix;
//		if (silent) suffix += "-query";
//		for (Director::Selection::const_iterator i = selection.begin(); i != selection.end(); ++i) {
//			j->second->ptrPoints = false;
//			j->second->ptrIndex = i->first;
//			grasp::Cloud::RawPointSeqMultiMap::iterator pointsRawPtr = j->second->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(j->second->pointsRaw);
//			if (pointsRawPtr == j->second->pointsRaw.end())
//				continue;
//
//			inp.insert(grasp::Cloud::RawPointSeqPtrMultiMap::value_type(pointsRawPtr->first, pointsRawPtr));
//			labelMap[pointsRawPtr->first] = pointsRawPtr->first;
//			suffix += '-' + std::to_string(i->first + 1);
//		}
//		if (inp.empty())
//			continue;
//
//		try {
//			if (process) {
//				// label re-assignment
//				for (grasp::Cloud::LabelMap::iterator i = labelMap.begin(); i != labelMap.end(); ++i) {
//					std::string name = j->second->labelMap[i->first].name;
//					name += suffix;
//					if (!allData && !silent)
//						readString("Enter new label name: ", name);
//
//					if (name != j->second->labelMap[i->first].name) {
//						i->second = j->second->labelMap.rbegin()->first + 1; // increment label
//						j->second->labelMap[i->second].name = name; // add new label
//					}
//				}
//				// process
//				grasp::Cloud::process(context, cloudDesc, inp, j->second->points, [&](const grasp::Cloud::RawPointSeqVal* points) {
//					golem::CriticalSectionWrapper csw(csDataRenderer);
//					pointRenderer.reset();
//					if (points) j->second->draw(points->second, pointRenderer);
//				}, &labelMap);
//			}
//			else
//				// transform
//				for (grasp::Cloud::RawPointSeqPtrMultiMap::iterator i = inp.begin(); i != inp.end(); ++i)
//					grasp::Cloud::transform(deformation ? cameraSeq[cameraIndex]->getCamera()->getCalibration()->getDeformation(j->second->robotPoseMap[&i->second->second]) : trn, i->second->second, i->second->second);
//		}
//		catch (const golem::Message& msg) {
//			context.write("%s\n", msg.str().c_str());
//		}
//	}
//
//	//Cloud::LabelMap labelMap;
//	//Cloud::RawPointSeqPtrMultiMap inp;
//	//for (Director::Selection::const_iterator i = selection.begin(); i != selection.end(); ++i) {
//	//	inp.insert(Cloud::RawPointSeqPtrMultiMap::value_type(i->second->first, i->second));
//	//	labelMap[i->second->first] = i->second->first;
//	//}
//
//	//// label re-assignment
//	//for (Cloud::LabelMap::iterator i = labelMap.begin(); i != labelMap.end(); ++i) {
//	//	std::string name = to<Data>(dataPtr)->labelMap[i->first].name;
//	//	readString("Enter new label name: ", name);
//	//	if (name != to<Data>(dataPtr)->labelMap[i->first].name) {
//	//		i->second = to<Data>(dataPtr)->labelMap.rbegin()->first + 1; // increment label
//	//		to<Data>(dataPtr)->labelMap[i->second].name = name; // add new label
//	//	}
//	//}
//
//	//Cloud::process(context, cloudDesc, inp, to<Data>(dataPtr)->points, [&] (const Cloud::RawPointSeqVal* points) {
//	//	golem::CriticalSectionWrapper csw(csDataRenderer);
//	//	pointRenderer.reset();
//	//	if (points) to<Data>(dataPtr)->draw(points->second, pointRenderer);
//	//}, &labelMap);
//
//	grasp::to<Data>(dataPtr)->ptrPoints = process;
//	grasp::to<Data>(dataPtr)->ptrLabel = grasp::Cloud::LABEL_DEFAULT;
//	grasp::to<Data>(dataPtr)->ptrIndex = grasp::to<Data>(dataPtr)->points.size() - 1;
//}

//------------------------------------------------------------------------------

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

#ifdef _SPAM_POSE_MAIN_
int main(int argc, char *argv[]) {
	return spam::PosePlanner::Desc().main(argc, argv);
}
//int main(int argc, char *argv[]) {
//	return spam::PosePlannerApp().main(argc, argv);
//}
#endif // _SPAM_POSEPLANNER_MAIN_