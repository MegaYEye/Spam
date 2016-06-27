/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Planner/GraphPlanner/Data.h>
#include <Spam/App/PosePlanner/PosePlanner.h>
#include <Spam/Data/Belief/Belief.h>

#include <Golem/Math/Rand.h>
#include <Grasp/Contact/Model.h>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>
#include <Grasp/Core/Import.h>
#include <Golem/UI/Data.h>
#include <Grasp/Core/RBPose.h>

//-----------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;

//-----------------------------------------------------------------------------

namespace {
	std::string toXMLString(const golem::Mat34& m)
	{
		char buf[BUFSIZ], *begin = buf, *const end = buf + sizeof(buf) - 1;
		golem::snprintf(begin, end,
			"m11=\"%f\" m12=\"%f\" m13=\"%f\" m21=\"%f\" m22=\"%f\" m23=\"%f\" m31=\"%f\" m32=\"%f\" m33=\"%f\" v1=\"%f\" v2=\"%f\" v3=\"%f\"",
			m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33, m.p.x, m.p.y, m.p.z);
		return std::string(buf);
	}

	std::string toXMLString(const grasp::ConfigMat34& cfg, const bool shortFormat = false)
	{
		std::ostringstream os;
		os.precision(6);
		const size_t n = shortFormat ? 7 : cfg.c.size();
		for (size_t i = 0; i < n; ++i)
		{
			os << (i == 0 ? "c" : " c") << i + 1 << "=\"" << cfg.c[i] << "\"";
		}

		return os.str();
	}

	class ForceEvent
	{
	public:
		ForceEvent(grasp::FT* pFTSensor_, const golem::Twist& threshold_) :
			threshold(threshold_),
			bias(Vec3::zero(), Vec3::zero()),
			maxExcursion(Vec3::zero(), Vec3::zero()),
			//logFile("FT-debug.log", std::ios::out | std::ios::app),
			pFTSensor(pFTSensor_)
		{
			if (pFTSensor == nullptr)
				throw Message(Message::LEVEL_CRIT, "ForceEvent(): pFTSensor is nullptr");
		}

		void setBias()
		{
			grasp::FT::Data ftdata;
			pFTSensor->read(ftdata, true);
			bias = ftdata.wrench;
		}

		bool detected(golem::Context* pContext = nullptr)
		{
			bool tripped = false;
			grasp::FT::Data ftdata;
			pFTSensor->read(ftdata, true);
			golem::Twist current = ftdata.wrench;
			double currentFT[6], biasFT[6], thresholdFT[6], maxExcursionFT[6];
			current.get(currentFT);
			bias.get(biasFT);
			threshold.get(thresholdFT);
			maxExcursion.get(maxExcursionFT);
			for (size_t i = 0; i < 6; ++i)
			{
				const double excursion = abs(currentFT[i] - biasFT[i]);

				if (excursion > maxExcursionFT[i])
					maxExcursionFT[i] = excursion;

				if (excursion > thresholdFT[i])
				{
					tripped = true;
					if (pContext != nullptr)
						pContext->debug("Force event detected on axis %d: |%f - %f| > %f\n", i + 1, currentFT[i], biasFT[i], thresholdFT[i]);
					//break; // test on all axes
				}
			}
			maxExcursion.set(maxExcursionFT);
			return tripped;
		}

		void showMaxExcursion(golem::Context* pContext)
		{
			const golem::Twist& m = maxExcursion;
			pContext->debug("ForceEvent: max excursion: %g %g %g;  %g %g %g\n", m.v.x, m.v.y, m.v.z, m.w.x, m.w.y, m.w.z);
		}

		void showFT(golem::Context* pContext)
		{
			grasp::FT::Data ft1, ft2;
			pFTSensor->read(ft1, false);
			pFTSensor->read(ft2, true);

			SecTmReal t1 = ft1.timeStamp;
			golem::Twist raw = ft1.wrench;
			SecTmReal t2 = ft2.timeStamp;
			golem::Twist in = ft2.wrench;

			pContext->debug("FT raw[%8.3f]: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f; FT inertia[%8.3f]: %7.2f %7.2f %7.2f %7.2f %7.2f %7.2f\n",
				t1, raw.v.x, raw.v.y, raw.v.z, raw.w.x, raw.w.y, raw.w.z,
				t2, in.v.x, in.v.y, in.v.z, in.w.x, in.w.y, in.w.z);
		}

	public:
		golem::Twist threshold;
		golem::Twist bias;
		golem::Twist maxExcursion;

		//golem::FileStream logFile;

	private:
		grasp::FT* pFTSensor;

		ForceEvent();
	};

}

//------------------------------------------------------------------------------

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

		// draw ground truth
		if (!simulateObjectPose.empty() && owner->showSimulate) {
			owner->groundtruthAppearance.draw(simulateObjectPose, owner->debugRenderer);
		}

		// draw hypothesis
		if (owner->pBelief.get() && owner->drawBeliefState) {
			for (Hypothesis::Seq::const_iterator i = owner->pBelief->getHypotheses().begin(); i != owner->pBelief->getHypotheses().end(); ++i) {
				grasp::Cloud::Appearance& appearance = i == owner->pBelief->getHypotheses().begin() ? owner->meanposeAppeareance : owner->hypothesisAppearance;
				owner->debugRenderer.addAxes((*i)->toRBPoseSampleGF().toMat34(), appearance.frameSize);
				appearance.draw((*i)->getCloud(), owner->debugRenderer);
				if (owner->showMeanPoseOnly) // this parameter can be control by outside
					break;
			}
			owner->pHeuristic->renderHypothesisCollisionBounds(owner->debugRenderer);
		}

		// show constantly the belief state, if needed
		//const grasp::data::Item::Map::iterator ptr = itemMap.find(owner->currentBeliefItem);
		//if (owner->drawBeliefState && ptr != itemMap.end()) {
		//	//ptr->second->createRender();
		//	data::ItemBelief* pItem = grasp::to<data::ItemBelief>(ptr);
		//	if (pItem)
		//		owner->addRenderer(is<golem::UIRenderer>(&pItem->getHandler()));
		//		//pItem->customRender();
		//}
	}
}

//------------------------------------------------------------------------------

void PosePlanner::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	Player::Desc::load(context, xmlcontext);

	RagGraphPlanner::Desc* pRagGraphPlanner(new RagGraphPlanner::Desc);
	(golem::Planner::Desc&)*pRagGraphPlanner = *uiPlannerDesc->plannerDescSeq[0]; // Planner
	//(golem::UIPlanner::Desc&)*pRagGraphPlanner = *uiPlannerDesc; // Physic planner
	uiPlannerDesc->plannerDescSeq[0].reset(pRagGraphPlanner);
	uiPlannerDesc->plannerDescSeq[0] = RagGraphPlanner::Desc::load(&context, xmlcontext->getContextFirst("player planner"));

	xmlcontext = xmlcontext->getContextFirst("pose_planner");

	FTDrivenHeuristic::Desc* pFTDrivenHeuristic(new FTDrivenHeuristic::Desc);
	(golem::Heuristic::Desc&)*pFTDrivenHeuristic = *uiPlannerDesc->plannerDescSeq[0]->pHeuristicDesc;
	uiPlannerDesc->plannerDescSeq[0]->pHeuristicDesc.reset(pFTDrivenHeuristic);
	spam::XMLData((FTDrivenHeuristic::Desc&)*uiPlannerDesc->plannerDescSeq[0]->pHeuristicDesc, xmlcontext->getContextFirst("heuristic"));

	try {
		spam::XMLData((Belief::Desc&)*pBeliefDesc, xmlcontext->getContextFirst("belief")/*const_cast<golem::XMLContext*>(xmlcontext)*/);
	}
	catch (const golem::MsgXMLParser& msg) { 
		pBeliefDesc.reset();
		context.write("%s\n", msg.str().c_str()); 
	}

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", modelCamera, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler_scan", modelHandlerScan, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_obj", modelItemObj, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler_grasp", modelGraspHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_grasp", modelGraspItem, xmlcontext->getContextFirst("model"));

	modelScanPoseSeq.clear();
	XMLData(modelScanPoseSeq, modelScanPoseSeq.max_size(), xmlcontext->getContextFirst("model"), "scan_pose");

	golem::XMLData("handler_trj", modelHandlerTrj, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_trj", modelItemTrj, xmlcontext->getContextFirst("model"));

	golem::XMLData("camera", queryCamera, xmlcontext->getContextFirst("query"));
	golem::XMLData("handler_scan", queryHandlerScan, xmlcontext->getContextFirst("query"));
	golem::XMLData("handler", queryHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item", queryItem, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_obj", queryItemObj, xmlcontext->getContextFirst("query"));
	golem::XMLData("handler_grasp", queryGraspHandler, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_grasp", queryGraspItem, xmlcontext->getContextFirst("query"));

	queryScanPoseSeq.clear();
	XMLData(queryScanPoseSeq, queryScanPoseSeq.max_size(), xmlcontext->getContextFirst("query"), "scan_pose");

	golem::XMLData("handler_trj", queryHandlerTrj, xmlcontext->getContextFirst("query"));
	golem::XMLData("item_trj", queryItemTrj, xmlcontext->getContextFirst("query"));

	try {
		XMLData((grasp::RBPose::Desc&)*simRBPoseDesc, xmlcontext->getContextFirst("pose_estimation_sim"));
		golem::XMLData("item", simulateItem, xmlcontext->getContextFirst("pose_estimation_sim"));
		golem::XMLData("handler", simulateHandler, xmlcontext->getContextFirst("pose_estimation_sim"));
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }

	try{
		golem::XMLData("sensor", graspSensorForce, xmlcontext->getContextFirst("grasp"));
		golem::XMLData(graspThresholdForce, xmlcontext->getContextFirst("grasp threshold"));
		golem::XMLData("event_time_wait", graspEventTimeWait, xmlcontext->getContextFirst("grasp"));
		golem::XMLData("close_duration", graspCloseDuration, xmlcontext->getContextFirst("grasp"));
		graspPoseOpen.xmlData(xmlcontext->getContextFirst("grasp pose_open"));
		graspPoseClosed.xmlData(xmlcontext->getContextFirst("grasp pose_closed"));
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }

	golem::XMLData("handler", beliefHandler, xmlcontext->getContextFirst("belief_state"));
	golem::XMLData("item", beliefItem, xmlcontext->getContextFirst("belief_state"));

	modelDescMap.clear();
	golem::XMLData(modelDescMap, modelDescMap.max_size(), xmlcontext->getContextFirst("model"), "model", false);
	contactAppearance.load(xmlcontext->getContextFirst("model appearance"));

	queryDescMap.clear();
	golem::XMLData(queryDescMap, queryDescMap.max_size(), xmlcontext->getContextFirst("query"), "query", false);

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));
	manipulatorAppearance.load(xmlcontext->getContextFirst("manipulator appearance"));

	try {
		hypothesisAppearance.xmlData(xmlcontext->getContextFirst("hypothesis_appearance", false), false);
		meanposeAppearance.xmlData(xmlcontext->getContextFirst("meanpose_appearance", false), false);
		groundTruthAppearance.xmlData(xmlcontext->getContextFirst("groundtruth_appearance", false), false);
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
}

//------------------------------------------------------------------------------

const std::string PosePlanner::Data::ModeName[MODE_QUERY + 1] = {
	"Model",
	"Query",
};

//------------------------------------------------------------------------------

spam::PosePlanner::PosePlanner(Scene &scene) : grasp::Player(scene), rand(context.getRandSeed()), pHeuristic(nullptr), drawBeliefState(false), showMeanPoseOnly(false), showSimulate(true), currentBeliefItem() {
}
	
bool spam::PosePlanner::create(const Desc& desc) {
	grasp::Player::create(desc); // throws

	grasp::data::Handler::Map::const_iterator beliefHandlerPtr = handlerMap.find(desc.beliefHandler);
	beliefHandler = beliefHandlerPtr != handlerMap.end() ? beliefHandlerPtr->second.get() : nullptr;
	if (!beliefHandler) {
		context.write("spam::PosePlanner::create(): unknown belief data handler: %s\n", desc.beliefHandler.c_str());
		pBelief = desc.pBeliefDesc->create(context); //static_cast<Belief*>(pRBPose.get());
		if (!pBelief.get())
			throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown belief desc");
	}
	else {	
		beliefItem = desc.beliefItem;
		pBelief = dynamic_cast<spam::data::BeliefState*>(beliefHandler->create().get())->getBeliefDesc()->create(context);
	}

	// manipulator
	manipulator = desc.manipulatorDesc->create(*getPlanner().planner, getPlanner().controllerIDSeq);
	manipulatorAppearance = desc.manipulatorAppearance;

	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&getPlanner().planner->getHeuristic());
	pHeuristic->setBelief(&*pBelief);
	pHeuristic->setManipulator(manipulator.get());

	grasp::Sensor::Map::const_iterator modelCameraPtr = sensorMap.find(desc.modelCamera);
	modelCamera = modelCameraPtr != sensorMap.end() ? is<Camera>(modelCameraPtr->second.get()) : nullptr;
	if (!modelCamera)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model pose estimation camera: %s", desc.modelCamera.c_str());
	grasp::data::Handler::Map::const_iterator modelHandlerScanPtr = handlerMap.find(desc.modelHandlerScan);
	modelHandlerScan = modelHandlerScanPtr != handlerMap.end() ? modelHandlerScanPtr->second.get() : nullptr;
	if (!modelHandlerScan)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model (scan) data handler: %s", desc.modelHandlerScan.c_str());
	grasp::data::Handler::Map::const_iterator modelHandlerPtr = handlerMap.find(desc.modelHandler);
	modelHandler = modelHandlerPtr != handlerMap.end() ? modelHandlerPtr->second.get() : nullptr;
	if (!modelHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	modelItem = desc.modelItem;
	modelItemObj = desc.modelItemObj;
	grasp::data::Handler::Map::const_iterator modelGraspHandlerPtr = handlerMap.find(desc.modelGraspHandler);
	modelGraspHandler = modelGraspHandlerPtr != handlerMap.end() ? modelGraspHandlerPtr->second.get() : nullptr;
	if (!modelGraspHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model data handler: %s", desc.modelHandler.c_str());
	modelGraspItem = desc.modelGraspItem;

	modelScanPoseSeq = desc.modelScanPoseSeq;

	grasp::data::Handler::Map::const_iterator modelHandlerTrjPtr = handlerMap.find(desc.modelHandlerTrj);
	modelHandlerTrj = modelHandlerTrjPtr != handlerMap.end() ? modelHandlerTrjPtr->second.get() : nullptr;
	if (!modelHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model trajectory handler: %s", desc.modelHandlerTrj.c_str());
	modelItemTrj = desc.modelItemTrj;

	grasp::Sensor::Map::const_iterator queryCameraPtr = sensorMap.find(desc.queryCamera);
	queryCamera = queryCameraPtr != sensorMap.end() ? is<Camera>(queryCameraPtr->second.get()) : nullptr;
	if (!queryCamera)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query pose estimation camera: %s", desc.queryCamera.c_str());
	grasp::data::Handler::Map::const_iterator queryHandlerScanPtr = handlerMap.find(desc.queryHandlerScan);
	queryHandlerScan = queryHandlerScanPtr != handlerMap.end() ? queryHandlerScanPtr->second.get() : nullptr;
	if (!queryHandlerScan)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query (scan) data handler: %s", desc.queryHandlerScan.c_str());
	grasp::data::Handler::Map::const_iterator queryHandlerPtr = handlerMap.find(desc.queryHandler);
	queryHandler = queryHandlerPtr != handlerMap.end() ? queryHandlerPtr->second.get() : nullptr;
	if (!queryHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query data handler: %s", desc.queryHandler.c_str());
	grasp::data::Handler::Map::const_iterator queryGraspHandlerPtr = handlerMap.find(desc.queryGraspHandler);
	queryGraspHandler = queryGraspHandlerPtr != handlerMap.end() ? queryGraspHandlerPtr->second.get() : nullptr;
	if (!queryGraspHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query data handler: %s", desc.queryHandler.c_str());
	queryGraspItem = desc.queryGraspItem;
	queryItem = desc.queryItem;
	queryItemObj = desc.queryItemObj;

	queryScanPoseSeq = desc.queryScanPoseSeq;

	grasp::data::Handler::Map::const_iterator queryHandlerTrjPtr = handlerMap.find(desc.queryHandlerTrj);
	queryHandlerTrj = queryHandlerTrjPtr != handlerMap.end() ? queryHandlerTrjPtr->second.get() : nullptr;
	if (!queryHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown query trajectory handler: %s", desc.queryHandlerTrj.c_str());
	queryItemTrj = desc.queryItemTrj;

	// simulate a ground truth for debugging and testing
	simRBPose = desc.simRBPoseDesc->create(context); // throws
	grasp::data::Handler::Map::const_iterator simHandlerPtr = handlerMap.find(desc.simulateHandler);
	simulateHandler = simHandlerPtr != handlerMap.end() ? simHandlerPtr->second.get() : nullptr;
	if (!simulateHandler)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown simulate data handler: %s", desc.simulateHandler.c_str());
	simulateItem = desc.simulateItem;
	simulateHandlerCallback = nullptr;

	// models
	modelMap.clear();
	for (Model::Desc::Map::const_iterator i = desc.modelDescMap.begin(); i != desc.modelDescMap.end(); ++i)
		modelMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));
	contactAppearance = desc.contactAppearance;

	// query densities
	queryMap.clear();
	for (Query::Desc::Map::const_iterator i = desc.queryDescMap.begin(); i != desc.queryDescMap.end(); ++i)
		queryMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));

	modelFrame.setId();

	queryViews = 3;
	hypothesisAppearance = desc.hypothesisAppearance;
	meanposeAppeareance = desc.meanposeAppearance;
	groundtruthAppearance = desc.groundTruthAppearance;

	myDesc = desc;
	
	scene.getHelp().insert(Scene::StrMapVal("080", "  M                                       model create\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  4                                       model points\n"));
	scene.getHelp().insert(Scene::StrMapVal("080", "  5                                       model features\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  Q                                       query create\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  6                                       query distribution\n"));
	scene.getHelp().insert(Scene::StrMapVal("081", "  7                                       hypotheses distribution\n"));
	scene.getHelp().insert(Scene::StrMapVal("082", "  B                                       Belief state menu\n"));

	menuCtrlMap.insert(std::make_pair("B", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (L)oad belief state...";
	}));
	menuCmdMap.insert(std::make_pair("BL", [=]() {
		if (!pBelief.get()) 
			throw Cancel("No belief state supported\n");

		// find handlers supporting data::BeliefState
		typedef std::vector<std::pair<grasp::data::Item::Map::iterator, data::BeliefState*>> BeliefStateMap;
		BeliefStateMap beliefStateMap;
		for (grasp::data::Item::Map::iterator i = to<Data>(dataCurrentPtr)->itemMap.begin(); i != to<Data>(dataCurrentPtr)->itemMap.end(); ++i) {
			data::BeliefState* state = is<data::BeliefState>(i);
			if (state) beliefStateMap.push_back(std::make_pair(i, state));
		}
		if (beliefStateMap.empty())
			throw Cancel("No handlers support Belief state interface");
		// pick up handler
		BeliefStateMap::const_iterator beliefStatePtr = beliefStateMap.begin();
		select(beliefStatePtr, beliefStateMap.begin(), beliefStateMap.end(), "Belief state:\n", [](BeliefStateMap::const_iterator ptr) -> std::string {
			return std::string("Beliefs: ") + ptr->first->first.c_str();
		});

		currentBeliefItem = beliefStatePtr->first->first;
		currentBeliefPtr = beliefStatePtr->first;
		context.write("Loading belief state %s\n", currentBeliefItem.c_str());

		//retrieve model point cloud
		Cloud::PointSeq modelPoints;
		try {
			modelPoints = getPoints(dataCurrentPtr, beliefStatePtr->second->getModelItem());
		}
		catch (const Message& msg) { context.write("%s", msg.what()); }

		// set belief state
		pBelief->set(beliefStatePtr->second->getPoses(), beliefStatePtr->second->getHypotheses(), beliefStatePtr->second->getModelFrame(), modelPoints);
		to<Data>(dataCurrentPtr)->modelPoints = modelPoints;
		to<Data>(dataCurrentPtr)->modelFrame = beliefStatePtr->second->getModelFrame();
		to<Data>(dataCurrentPtr)->queryTransform = beliefStatePtr->second->getQueryTransform();
		// retrieve query point cloud
		Cloud::PointSeq queryPoints;
		try {
			queryPoints = getPoints(dataCurrentPtr, beliefStatePtr->second->getQueryItem());
		}
		catch (const Message& msg) { context.write("%s", msg.what()); }
		to<Data>(dataCurrentPtr)->queryPoints = queryPoints;

		// retrieve ground truth point cloud
		Cloud::PointSeq simulatedPoints;
		try {
			simulatedPoints = getPoints(dataCurrentPtr, beliefStatePtr->second->getQueryItemSim());
		}
		catch (const Message& msg) { context.write("%s", msg.what()); }
		to<Data>(dataCurrentPtr)->simulateObjectPose = simulatedPoints;
		if (simulateHandlerCallback)
			simulateHandlerCallback(simulatedPoints);

		beliefStatePtr->second->set(pBelief.get());
		// needed to determine when to expect collisions
		pHeuristic->setHypothesisBounds();
		RenderBlock renderBlock(*this);
		{
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, beliefStatePtr->first, to<Data>(dataCurrentPtr)->getView());
		}
		// render point cloud
		to<Data>(dataCurrentPtr)->createRender();

		// finish
		context.write("Done!\n");
	}));

	menuCtrlMap.insert(std::make_pair("P", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (M)odel/(Q)ery estimation...";
	}));
	menuCmdMap.insert(std::make_pair("PM", [=]() {
		// item name
		readString("Enter item name: ", modelItem);
		// scan, if needed
		if (option("YN", "Scan? (Y/N)") == 'Y') {
			try {
				(void)objectCapture(Data::MODE_MODEL, modelItem);
			}
			catch (const Message &msg) { context.write("%s\n", msg.what()); return;  }
		}
		// estimate
		try {
			(void)estimatePose(Data::MODE_MODEL, modelItem);
		}
			catch (const Message &msg) {
			context.write("%s\n", msg.what());
		}
	// finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("PQ", [=]() {
		// item name
		readString("Enter item name: ", queryItem);
		// scan, if needed
		if (option("YN", "Scan? (Y/N)") == 'Y') {
			try {
				(void)objectCapture(Data::MODE_QUERY, queryItem);
			}
			catch (const Message &msg) { context.write("%s\n", msg.what()); return; }
		}
		// estimate
		try {
			(void)estimatePose(Data::MODE_QUERY, queryItem);
		}
		catch (const Message &msg) {
			context.write("%s\n", msg.what());
		}
		// finish
		context.write("Done!\n");
	}));

	return true;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::render() const {
	Player::render();

	golem::CriticalSectionWrapper cswRenderer(getCS());
	debugRenderer.render();
}

//------------------------------------------------------------------------------

void spam::PosePlanner::resetDataPointers() {
	showMeanPoseOnly = false;
	showSimulate = true;

	//const grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(currentBeliefItem);
	//if (ptr != to<Data>(dataCurrentPtr)->itemMap.end()) {
	//	//ptr->second->createRender();
	//	data::ItemBelief* pItem = grasp::to<data::ItemBelief>(ptr);
	//	if (pItem) {
	//		pItem->showMeanPoseOnly(true);
	//		pItem->showQuery(false);
	//		pItem->showGroundTruth(true);
	//	}
	//}
}

//------------------------------------------------------------------------------

grasp::data::Item::Map::iterator spam::PosePlanner::estimatePose(Data::Mode mode, std::string &itemName) {
	grasp::data::Handler* handler = mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	
	grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(itemName);
	if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
		throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", itemName.c_str());

	// retrive point cloud with curvature
	grasp::data::ItemPointsCurv *pointsCurv = is<grasp::data::ItemPointsCurv>(ptr->second.get());
	if (!pointsCurv)
		throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not support PointsCurv interface.");
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
	// reset pointer for rendering
	resetDataPointers();
	if (mode == Data::MODE_MODEL) {
		context.write("Create model on %s handler:%s\n", ptr->first.c_str(), pointsCurv->getHandler().getID().c_str());
		pBelief->createModel(curvPoints);
		modelFrame = pBelief->createFrame(mPoints);
		grasp::to<Data>(dataCurrentPtr)->modelFrame = modelFrame;
		grasp::to<Data>(dataCurrentPtr)->modelPoints = modelPoints = points;
		simRBPose->createModel(curvPoints);
		simModelFrame = simRBPose->createFrame(mPoints);
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	}
	else {
		context.write("Create query on %s handler:%s\n", ptr->first.c_str(), pointsCurv->getHandler().getID().c_str());
		if (modelPoints.empty())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): no model has been created.");

		if (true/*option("YN", "Do you want to load the simulated object? (Y/N)") == 'Y'*/) {
			grasp::to<Data>(dataCurrentPtr)->simulateObjectPose.clear();
			grasp::to<Data>(dataCurrentPtr)->simulateObjectPose.reserve(modelPoints.size());
			for (grasp::Cloud::PointSeq::const_iterator point = modelPoints.begin(); point != modelPoints.end(); ++point) {
				grasp::Cloud::Point p = *point;
				grasp::Cloud::setColour(golem::RGBA::YELLOW, p);
				grasp::to<Data>(dataCurrentPtr)->simulateObjectPose.push_back(p);
			}
			if (true/*option("YN", "Do you want to tranform the simulated object? (Y/N)") == 'Y'*/) {
				simRBPose->createQuery(points);
				simQueryFrame = simRBPose->maximum().toMat34();
				grasp::Cloud::transform(simQueryFrame, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			}
			else
				grasp::Cloud::transform(grasp::to<Data>(dataCurrentPtr)->queryTransform, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
	
			// callback to handle groundtruth, if any
			if (simulateHandlerCallback)
				simulateHandlerCallback(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);

			grasp::data::Item::Ptr simPtr = ptr->second->clone();
			// save simulated point cloud
			RenderBlock renderBlock(*this);
			{
				golem::CriticalSectionWrapper cswData(scene.getCS());
				grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(simulateItem, simPtr));
				if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
					throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", simulateItem.c_str());
				// retrive point cloud with curvature
				//grasp::data::ItemPointsCurv *pCurv = is<grasp::data::ItemPointsCurv>(ptr->second.get());
				//if (!pCurv)
				//	throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not support PointsCurv interface.");			
				//*pCurv->cloud = *pointsCurv->cloud;
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
			}

			pBelief->realPose = simQueryFrame*modelFrame;

		}
		pBelief->createQuery(points);
		// compute transformation model -> query
		const grasp::RBPose::Sample trn = pBelief->createHypotheses(modelPoints, modelFrame);
		grasp::to<Data>(dataCurrentPtr)->queryTransform = trn.toMat34();
		grasp::to<Data>(dataCurrentPtr)->queryFrame.multiply(grasp::to<Data>(dataCurrentPtr)->queryTransform, modelFrame);
		grasp::Cloud::transform(grasp::to<Data>(dataCurrentPtr)->queryTransform, modelPoints, grasp::to<Data>(dataCurrentPtr)->queryPoints);
		grasp::to<Data>(dataCurrentPtr)->queryPoints = points;
		pHeuristic->setHypothesisBounds();
		
		currentBeliefItem = makeString("%s-%.3f", beliefItem.c_str(), context.getTimer().elapsed());
		// save belief
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(getCS());
			currentBeliefPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(currentBeliefItem, beliefHandler->create()));
			//to<Data>(dataCurrentPtr)->itemMap.find(beliefItem); 
			//to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, beliefHandler->create()));
			spam::data::BeliefState* beliefState = is<spam::data::BeliefState>(currentBeliefPtr->second.get());
			beliefState = currentBeliefPtr != to<Data>(dataCurrentPtr)->itemMap.end() ? beliefState : nullptr;
			if (!beliefState)
				throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): beliefState handler does not implement data::beliefState");
			// add current states
			beliefState->set(pBelief.get());
			beliefState->set(trn.toMat34(), pBelief->getSamples(), pBelief->getHypothesesToSample());
			beliefState->setModelPoints(modelItem, modelFrame, modelPoints);
			beliefState->setQueryPoints(itemName, points);
			beliefState->setSimObject(simulateItem, simQueryFrame, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			beliefState->showMeanPoseOnly(false);
			beliefState->showGroundTruth(true);
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, currentBeliefPtr, to<Data>(dataCurrentPtr)->getView());
			context.write("Save: handler %s, inputs %s, %s...\n", beliefHandler->getID().c_str(), currentBeliefPtr->first.c_str(), beliefItem.c_str());
		}
	}
	// render point cloud
	to<Data>(dataCurrentPtr)->createRender();
	return ptr;
}

grasp::data::Item::Map::iterator spam::PosePlanner::objectCapture(const Data::Mode mode, std::string &itemName) {
//	std::string& itemName = mode == Data::MODE_DEFAULT ? objectItem : mode != Data::MODE_MODEL ? queryItem : modelItem;
	const std::string itemNameRaw = itemName + "_raw";
	grasp::data::Handler* handler = mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	grasp::data::Handler* handlerScan = mode != Data::MODE_MODEL ? queryHandlerScan : modelHandlerScan;
	grasp::Camera* camera = mode != Data::MODE_MODEL ? queryCamera : modelCamera;
	grasp::ConfigMat34::Seq scanPoseSeq = mode != Data::MODE_MODEL ? queryScanPoseSeq : modelScanPoseSeq;
	
	grasp::data::Capture* capture = is<grasp::data::Capture>(handlerScan);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", handlerScan->getID().c_str());

	select(sensorCurrentPtr, sensorMap.begin(), sensorMap.end(), "Select Sensor:\n", [](Sensor::Map::const_iterator ptr) -> const std::string&{
		return ptr->second->getID();
	});

	if (is<FT>(sensorCurrentPtr))
		throw Message(Message::LEVEL_ERROR, "Current sensor %s does not support Scan interface", sensorCurrentPtr->second->getID().c_str());

	// Scan object
	const bool isEnabledDeformationMap = camera && camera->getCurrentCalibration()->isEnabledDeformationMap();
	ScopeGuard guard([&]() { if (camera) camera->getCurrentCalibration()->enableDeformationMap(isEnabledDeformationMap); });
	if (camera && camera->getCurrentCalibration()->hasDeformationMap())
		camera->getCurrentCalibration()->enableDeformationMap(true);

	if (mode != Data::MODE_MODEL) {
		scanPoseSeq.clear();
		queryViews = size_t(rand.next() % 3) + size_t(1);
		context.write("Query scan %d poses\n", queryViews);
		grasp::ConfigMat34::Seq seq = modelScanPoseSeq;
		std::random_shuffle(seq.begin(), seq.end(), rand);
		for (size_t i = 0; i < queryViews; ++i) {
			scanPoseSeq.push_back(seq[i]);
		}
	}
	ConfigMat34::Seq::const_iterator pose = scanPoseSeq.begin();
	size_t index = 0, size = scanPoseSeq.size();
	auto scanPoseCommand = [&]() -> bool {
		context.write("Going to scan pose #%d/%d\n", index + 1, size);
		this->gotoPose(*pose++);
		return ++index < size;
	};
	typedef std::vector<grasp::data::Item::Map::iterator> ItemPtrSeq;
	ItemPtrSeq itemPtrSeq; itemPtrSeq.resize(size);
	U32 ii = 0;
	for (bool stop = false; !stop;) {
		stop = !scanPoseCommand();
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			itemPtrSeq[ii] = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(itemNameRaw, capture->capture(*to<Camera>(sensorCurrentPtr), [&](const grasp::TimeStamp*) -> bool { return true; })));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, itemPtrSeq[ii++], to<Data>(dataCurrentPtr)->getView());
		}
	}

	// Transform to point curv
	// generate features
	grasp::data::Transform* transform = is<grasp::data::Transform>(handler);
	if (!transform)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", handler->getID().c_str());

	context.write("Transform items to %s\n", handler->getID().c_str());
	grasp::data::Item::List list;
	for (auto ptr = itemPtrSeq.begin(); ptr != itemPtrSeq.end(); ptr++)
		list.insert(list.end(), *ptr);
	grasp::data::Item::Ptr item = transform->transform(list);

	// item name
//	readString("Save point Curv as: ", itemName);

	// insert processed object, remove old one
	grasp::data::Item::Map::iterator pointCurvPtr;
	RenderBlock renderBlock(*this);
	{
		golem::CriticalSectionWrapper cswData(getCS());
		// remove the raw point clouds
		to<Data>(dataCurrentPtr)->itemMap.erase(itemNameRaw);
		to<Data>(dataCurrentPtr)->itemMap.erase(itemName);
		pointCurvPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(itemName, item));
		Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, pointCurvPtr, to<Data>(dataCurrentPtr)->getView());
	}

	return pointCurvPtr;
}

// Process object image and add to data bundle
grasp::data::Item::Map::iterator spam::PosePlanner::objectProcess(const Data::Mode mode, grasp::data::Item::Map::iterator ptr) {
	const std::string itemName = mode != Data::MODE_MODEL ? queryItem : modelItem;
	grasp::data::Handler* handler = mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	// generate features
	grasp::data::Transform* transform = is<grasp::data::Transform>(handler);
	if (!transform)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", handler->getID().c_str());

	grasp::data::Item::List list;
	list.insert(list.end(), ptr);
	grasp::data::Item::Ptr item = transform->transform(list);

	// insert processed object, remove old one
	RenderBlock renderBlock(*this);
	golem::CriticalSectionWrapper cswData(getCS());
	to<Data>(dataCurrentPtr)->itemMap.erase(itemName);
	ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(itemName, item));
	Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
	return ptr;
}

//------------------------------------------------------------------------------

grasp::Cloud::PointSeq spam::PosePlanner::getPoints(Data::Map::iterator dataPtr, const std::string &itemName) const {
	grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(itemName);
	if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
		throw Message(Message::LEVEL_ERROR, "PosePlanner: Does not find %s.", itemName.c_str());

	// retrive point cloud with curvature
	grasp::data::ItemPointsCurv *pointsCurv = is<grasp::data::ItemPointsCurv>(ptr->second.get());
	if (!pointsCurv)
		throw Cancel("PosePlanner: Does not support PointsCurv interface.");
	if (pointsCurv->cloud->empty())
		throw Cancel("PosePlanner: Model point cloud is empty.");
	grasp::Cloud::PointCurvSeq curvPoints = *pointsCurv->cloud;

	// copy as a generic point+normal point cloud (Cloud::pointSeq)
	Cloud::PointSeq points;
	points.resize(curvPoints.size());
	Cloud::copy(curvPoints, points);

	return points;
}
