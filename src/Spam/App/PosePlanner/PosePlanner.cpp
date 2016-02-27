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
#include <Spam/App/PosePlanner/PosePlanner.h>

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
		owner->beliefRenderer.reset();
		owner->hypothesisRenderer.reset();
		owner->groundTruthRenderer.reset();
		owner->queryRenderer.reset();
		//owner->debugRenderer.reset();
		owner->debugRenderer.setColour(RGBA::BLACK);
		owner->debugRenderer.setLineWidth(Real(2.0));
//		owner->debugRenderer.addWire(owner->handBounds.begin(), owner->handBounds.end());

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

		const bool showmodel = !modelPoints.empty() /*&& queryPoints.empty()*/;
		const bool showquery = !queryPoints.empty();
//		printf("renderData showmodel %s showquery %s showSamplePoints %s\n", showmodel ? "ON" : "OFF", showquery ? "ON" : "OFF", owner->showSamplePoints ? "ON" : "OFF");

		if (showmodel || showquery) {
			if (showmodel/* && !showquery*/) {
				owner->modelRenderer.addAxes(modelFrame, owner->modelFrameSize);
				if (owner->showModelPointCloud)
					owner->modelAppearance.drawPoints(modelPoints, owner->modelRenderer);
			}

			if (showquery) {
				if (owner->showQueryDistribPointClouds) {
					U8 id = 0;
					const Real max = owner->pBelief->maxWeight(true);
					//1 / (2 * (sigma*sqrt(2 * pi)))*exp(-.5*((x - mu) / sigma). ^ 2);
					const Real sigma = 0.2, norm = 1 / (2 * (sigma*Math::sqrt(2 * REAL_PI)));
					for (auto i = 0; i < owner->pBelief->getSamples().size(); ++i) {
						if (owner->pBelief->getSamples().size() > 10 && i % 10 != 0) continue;
						grasp::RBPose::Sample &j = owner->pBelief->getSamples()[i];
						owner->beliefRenderer.addAxes(j.toMat34() * modelFrame, owner->distrFrameSize*owner->pBelief->normalise(j));
						owner->debugAppearance.colourOverride = true;
						const Real weight = owner->pBelief->normalise(j) / max;
						const Real red = norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)) * 255;
						//context.write("red = %f, U8(red)=%f\n", norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)), U8(norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma))));
						const Real green = norm*Math::exp(-.5*Math::sqr((weight - .5) / sigma)) * 255;
						const Real blue = norm*Math::exp(-.5*Math::sqr((weight - .0) / sigma)) * 255;
						///*j.weight*/Math::log2(1+pBelief->normalise(j)/max)*255;
						//context.debug("weight=%f, normalised=%f, red=%u, green=%u, blue=%u\n", pBelief->normalise(j), pBelief->normalise(j) / max, U8(red), U8(green), U8(blue));
						owner->debugAppearance.colour = weight > REAL_ZERO ? RGBA(U8(red), U8(green), U8(blue), 200) : RGBA::BLACK;//RGBA(red > 255 ? 255 : U8(red), 255 - red < 0 ? 0 : U8(1-red), 0, 200);
						grasp::Cloud::PointSeq m;
						grasp::Cloud::transform(j.toMat34(), modelPoints, m);
//						if (weight > 0.7)
						owner->debugAppearance.draw(m, owner->beliefRenderer);
					}
				}
				if (owner->showHypothesesPointClouds) {
					owner->hypothesisAppearance.colourOverride = true;
					//context.write("showSamplePoints hypotheses size %d\n", pBelief->getHypotheses().size());
					for (Hypothesis::Seq::const_iterator i = owner->pBelief->getHypotheses().begin(); i != owner->pBelief->getHypotheses().end(); ++i) {
						owner->hypothesisRenderer.addAxes((*i)->toRBPoseSampleGF().toMat34(), owner->hypothesisFrameSize);
						owner->hypothesisAppearance.colour = i == owner->pBelief->getHypotheses().begin() ? RGBA::GREEN : RGBA::BLUE;
						owner->hypothesisAppearance.draw((*i)->getCloud(), owner->hypothesisRenderer);
						if (owner->showMeanHypothesisPointClouds)
							break;
					}
					if (owner->showHypothesisBounds) owner->pHeuristic->renderHypothesisCollisionBounds(owner->hypothesisRenderer);
				}
				if (owner->showGroundTruth) {
					owner->groundTruthAppearance.draw(simulateObjectPose, owner->groundTruthRenderer);
				}
				if (owner->showQueryDistribFrames) {
					for (size_t i = 0; i < owner->distribSamples; ++i) {
						owner->beliefRenderer.addAxes(owner->pBelief->sample().toMat34() * modelFrame, owner->distrFrameSize);
					}
				}
			}
		}

		if (!queryPoints.empty()) {
			if (owner->showQueryPointCloud) {
				owner->queryRenderer.addAxes(queryFrame, owner->queryFrameSize);
				owner->queryAppearance.draw(queryPoints, owner->queryRenderer);
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
	(golem::Planner::Desc&)*pRagGraphPlanner = *uiPlannerDesc->pPlannerDesc; // Planner
	//(golem::UIPlanner::Desc&)*pRagGraphPlanner = *uiPlannerDesc; // Physic planner
	uiPlannerDesc->pPlannerDesc.reset(pRagGraphPlanner);
	uiPlannerDesc->pPlannerDesc = RagGraphPlanner::Desc::load(&context, xmlcontext->getContextFirst("player planner"));

	xmlcontext = xmlcontext->getContextFirst("pose_planner");

	FTDrivenHeuristic::Desc* pFTDrivenHeuristic(new FTDrivenHeuristic::Desc);
	(golem::Heuristic::Desc&)*pFTDrivenHeuristic = *uiPlannerDesc->pPlannerDesc->pHeuristicDesc;
	uiPlannerDesc->pPlannerDesc->pHeuristicDesc.reset(pFTDrivenHeuristic);
	spam::XMLData((FTDrivenHeuristic::Desc&)*uiPlannerDesc->pPlannerDesc->pHeuristicDesc, xmlcontext->getContextFirst("heuristic"));

	//try {
	//	XMLData((grasp::RBPose::Desc&)*pRBPoseDesc, const_cast<golem::XMLContext*>(xmlcontext));
	//}
	//catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
	//
	//Belief::Desc* pBeliefDesc(new Belief::Desc);
	//(grasp::RBPose::Desc&)*pBeliefDesc = *pRBPoseDesc;
	//pRBPoseDesc.reset(pBeliefDesc);
	try {
		spam::XMLData((Belief::Desc&)*pBeliefDesc, xmlcontext->getContextFirst("belief")/*const_cast<golem::XMLContext*>(xmlcontext)*/);
	}
	catch (const golem::MsgXMLParser& msg) { 
		pBeliefDesc.reset();
		context.write("%s\n", msg.str().c_str()); 
	}

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

	modelAppearance.xmlData(xmlcontext->getContextFirst("model_appearance", false), false);
	queryAppearance.xmlData(xmlcontext->getContextFirst("query_appearance", false), false);
	hypothesisAppearance.xmlData(xmlcontext->getContextFirst("hypothesis_appearance", false), false);
	debugAppearance.xmlData(xmlcontext->getContextFirst("debug_appearance", false), false);
	groundTruthAppearance.xmlData(xmlcontext->getContextFirst("groundtruth_appearance", false), false);

	golem::XMLData("data_name", dataName, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("screen_capture", screenCapture, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData("camera", modelCamera, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler_scan", modelHandlerScan, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler", modelHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item", modelItem, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_obj", modelItemObj, xmlcontext->getContextFirst("model"));
	golem::XMLData("handler_grasp", modelGraspHandler, xmlcontext->getContextFirst("model"));
	golem::XMLData("item_grasp", modelGraspItem, xmlcontext->getContextFirst("model"));

	modelScanPoseSeq.clear();
	XMLData(modelScanPoseSeq, modelScanPoseSeq.max_size(), xmlcontext->getContextFirst("model"), "scan_pose");
	golem::XMLData(modelColourSolid, xmlcontext->getContextFirst("model colour solid"));
	golem::XMLData(modelColourWire, xmlcontext->getContextFirst("model colour wire"));

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
	golem::XMLData(queryColourSolid, xmlcontext->getContextFirst("query colour solid"));
	golem::XMLData(queryColourWire, xmlcontext->getContextFirst("query colour wire"));

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
	try {
		XMLData((grasp::RBPose::Desc&)*simRBPoseDesc, xmlcontext->getContextFirst("pose_estimation_sim"));
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
}

//------------------------------------------------------------------------------

const std::string PosePlanner::Data::ModeName[MODE_QUERY + 1] = {
	"Model",
	"Query",
};

//------------------------------------------------------------------------------

spam::PosePlanner::PosePlanner(Scene &scene) : grasp::Player(scene), rand(context.getRandSeed()), pHeuristic(nullptr) {
}

//spam::PosePlanner::~PosePlanner() {}
	
bool spam::PosePlanner::create(const Desc& desc) {
	grasp::Player::create(desc); // throws

	trial = desc.trialData;
	trial->controller = this->controller;
	trial->setup(context, rand);
	golem::mkdir(trial->path.c_str()); // make sure that the directory exists
	trialPtr = trialDataMap.end();

	//pRBPose = desc.pRBPoseDesc->create(context); // throws
	grasp::data::Handler::Map::const_iterator beliefHandlerPtr = handlerMap.find(desc.beliefHandler);
	beliefHandler = beliefHandlerPtr != handlerMap.end() ? beliefHandlerPtr->second.get() : nullptr;
	if (!beliefHandler) {
		context.write("spam::PosePlanner::create(): unknown belief data handler: %s\n", desc.beliefHandler.c_str());
		pBelief = desc.pBeliefDesc->create(context); //static_cast<Belief*>(pRBPose.get());
		if (!pBelief.get())
			throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown belief desc");
	}
	else {	
		//beliefItem = desc.beliefItem;
		//grasp::data::Item::Map::iterator beliefPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(beliefItem, beliefHandler->create()));
		//spam::data::BeliefState* beliefState = is<spam::data::BeliefState>(beliefPtr->second.get());
		//beliefState = beliefPtr != to<Data>(dataCurrentPtr)->itemMap.end() ? beliefState : nullptr;
		//if (!beliefState)
		//	throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): beliefState handler does not implement data::beliefState");
		//pBelief = beliefState->getBeliefDesc()->create(context);
		pBelief = dynamic_cast<spam::data::BeliefState*>(beliefHandler->create().get())->getBeliefDesc()->create(context);
	}

	// manipulator
	manipulator = desc.manipulatorDesc->create(*planner, desc.controllerIDSeq);
	manipulatorAppearance = desc.manipulatorAppearance;

	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner->getHeuristic());
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
	modelColourSolid = desc.modelColourSolid;
	modelColourWire = desc.modelColourWire;

	grasp::data::Handler::Map::const_iterator modelHandlerTrjPtr = handlerMap.find(desc.modelHandlerTrj);
	modelHandlerTrj = modelHandlerTrjPtr != handlerMap.end() ? modelHandlerTrjPtr->second.get() : nullptr;
	if (!modelHandlerTrj)
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model trajectory handler: %s", desc.modelHandlerTrj.c_str());
	modelItemTrj = desc.modelItemTrj;

	graspSensorForce = nullptr;
	//grasp::Sensor::Map::const_iterator graspSensorForcePtr = sensorMap.find(desc.graspSensorForce);
	//graspSensorForce = graspSensorForcePtr != sensorMap.end() ? is<FT>(graspSensorForcePtr->second.get()) : nullptr;
	//if (!graspSensorForce)
	//	throw Message(Message::LEVEL_CRIT, "pacman::Demo::create(): unknown grasp F/T sensor: %s", desc.graspSensorForce.c_str());
	//graspThresholdForce = desc.graspThresholdForce;
	//graspEventTimeWait = desc.graspEventTimeWait;
	//graspCloseDuration = desc.graspCloseDuration;
	//graspPoseOpen = desc.graspPoseOpen;
	//graspPoseClosed = desc.graspPoseClosed;

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
		throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model data handler: %s", desc.queryHandler.c_str());
	queryGraspItem = desc.queryGraspItem;
	queryItem = desc.queryItem;
	queryItemObj = desc.queryItemObj;

	queryScanPoseSeq = desc.queryScanPoseSeq;
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
	simRBPose = desc.simRBPoseDesc->create(context); // throws

	// models
	modelMap.clear();
	for (Model::Desc::Map::const_iterator i = desc.modelDescMap.begin(); i != desc.modelDescMap.end(); ++i)
		modelMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));
	contactAppearance = desc.contactAppearance;

	// query densities
	queryMap.clear();
	for (Query::Desc::Map::const_iterator i = desc.queryDescMap.begin(); i != desc.queryDescMap.end(); ++i)
		queryMap.insert(std::make_pair(i->first, i->second->create(context, i->first)));

	modelAppearance = desc.modelAppearance;
	queryAppearance = desc.queryAppearance;
	hypothesisAppearance = desc.hypothesisAppearance;
	debugAppearance = desc.debugAppearance;
	groundTruthAppearance = desc.groundTruthAppearance;
	resampleAppeareance = desc.queryAppearance;
	sampleAppearance.setToDefault();
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
		showModelPointCloud = false;
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
	//menuCtrlMap.insert(std::make_pair("Z", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
	//	desc = "Belief state rendering options...";
	//}));
	//menuCmdMap.insert(std::make_pair("Z-", [=]() {
	//	// render hypotheses
	//	showQueryDistrib = !showQueryDistrib;
	//	context.write("Query (low-dim) distribution %s\n", showQueryDistrib ? "ON" : "OFF");
	//	createRender();
	//}));
	//menuCmdMap.insert(std::make_pair("Z_", [=]() {
	//	// render query distribution
	//	showDistrPoints = !showDistrPoints;
	//	context.write("Query (high-dim) distribution %s\n", showDistrPoints ? "ON" : "OFF");
	//	createRender();
	//}));
	//menuCmdMap.insert(std::make_pair("Z=", [=]() {
	//	showObject = showSamplePoints && showMeanHypothesis ? !showObject : showObject;
	//	showMeanHypothesis = showSamplePoints && !showObject ? !showMeanHypothesis : showObject ? !showMeanHypothesis : showMeanHypothesis;
	//	showSamplePoints = !showMeanHypothesis && !showObject ? !showSamplePoints : showObject ? !showSamplePoints : showSamplePoints;
	//	context.write("Hypotheses distribution %s\nShow Mean Pose %s\nObject Points %s\n-----------\n", showSamplePoints ? "ON" : "OFF", showMeanHypothesis ? "ON" : "OFF", showObject ? "ON" : "OFF");		
	//	createRender();
	//}));
	//menuCmdMap.insert(std::make_pair("Z+", [=]() {
	//	showQueryPoints = !showQueryPoints;
	//	context.write("Query points %s\n", showQueryPoints ? "ON" : "OFF");
	//	createRender();
	//}));

	return true;
}

//------------------------------------------------------------------------------

void spam::PosePlanner::render() const {
	Player::render();

	golem::CriticalSectionWrapper cswRenderer(getCS());
	modelRenderer.render();
	beliefRenderer.render();
	hypothesisRenderer.render();
	groundTruthRenderer.render();
	queryRenderer.render();
	debugRenderer.render();
}

//------------------------------------------------------------------------------

void spam::PosePlanner::resetDataPointers() {
	showModelPointCloud = false;
	showModelFeatures = false;
	showQueryPointCloud = false;
	showHypothesesPointClouds = false;
	showMeanHypothesisPointClouds = false;
	showHypothesisBounds = false;
	showQueryDistribFrames = false;
	showQueryDistribPointClouds = false;
	showGroundTruth = false;
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

grasp::data::Item::Map::iterator spam::PosePlanner::estimatePose(Data::Mode mode, std::string &itemName) {
//	std::string &itemName = mode != Data::MODE_MODEL ? queryItem : modelItem;
	grasp::data::Handler* handler = mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	
	// item name
//	readString("Enter item name: ", itemName);
	grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(itemName);
	if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
		throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", itemName.c_str());

	// retrive point cloud with curvature
	grasp::data::ItemPointsCurv *pointsCurv = is<grasp::data::ItemPointsCurv>(ptr->second.get());
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
	// reset pointer for rendering
	resetDataPointers();
	if (mode == Data::MODE_MODEL) {
		context.write("Create model on %s handler:%s\n", ptr->first.c_str(), pointsCurv->getHandler().getID().c_str());
		pBelief->createModel(curvPoints);
		modelFrame = pBelief->createFrame(mPoints);
		grasp::to<Data>(dataCurrentPtr)->modelFrame = modelFrame;
		grasp::to<Data>(dataCurrentPtr)->modelPoints = modelPoints = points;
		showModelPointCloud = true;
		simRBPose->createModel(curvPoints);
		simModelFrame = simRBPose->createFrame(mPoints);
	}
	else {
		context.write("Create query on %s handler:%s\n", ptr->first.c_str(), pointsCurv->getHandler().getID().c_str());
		if (modelPoints.empty())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): no model has been created.");

		//Mat34 t;
		//t.setId();
		//t.p = Vec3(.1, .1, 0);
		//Cloud::transform(t, points, points);
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
				grasp::Cloud::transform(/*grasp::to<Data>(dataCurrentPtr)->queryTransform*/simQueryFrame, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
				showGroundTruth = true;
			}
			else
				grasp::Cloud::transform(grasp::to<Data>(dataCurrentPtr)->queryTransform, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);

			pBelief->realPose = simQueryFrame*modelFrame;

		}
		pBelief->createQuery(points);
		// compute transformation model -> query
		const grasp::RBPose::Sample trn = pBelief->createHypotheses(modelPoints, modelFrame);
		grasp::to<Data>(dataCurrentPtr)->queryTransform = trn.toMat34();
		grasp::to<Data>(dataCurrentPtr)->queryFrame.multiply(grasp::to<Data>(dataCurrentPtr)->queryTransform, modelFrame);
		grasp::Cloud::transform(grasp::to<Data>(dataCurrentPtr)->queryTransform, modelPoints, grasp::to<Data>(dataCurrentPtr)->queryPoints);
		grasp::to<Data>(dataCurrentPtr)->queryPoints = points;
		grasp::to<Data>(dataCurrentPtr)->poses.clear();
		grasp::to<Data>(dataCurrentPtr)->poses = pBelief->getSamples();
		grasp::to<Data>(dataCurrentPtr)->hypotheses.clear();
		grasp::to<Data>(dataCurrentPtr)->hypotheses = pBelief->getHypothesesToSample();
		pHeuristic->setHypothesisBounds();

		showModelPointCloud = false;
		showQueryPointCloud = false;
		showQueryDistribPointClouds = false;
		showQueryDistribFrames = true;
		showHypothesesPointClouds = true;
		showMeanHypothesisPointClouds = false;
		showHypothesisBounds = true;
		
		// save belief
		std::string trjItemName("belief");
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(getCS());
			grasp::data::Item::Map::iterator beliefPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(beliefItem, beliefHandler->create()));
			//to<Data>(dataCurrentPtr)->itemMap.find(beliefItem); 
			//to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, beliefHandler->create()));
			spam::data::BeliefState* beliefState = is<spam::data::BeliefState>(beliefPtr->second.get());
			beliefState = beliefPtr != to<Data>(dataCurrentPtr)->itemMap.end() ? beliefState : nullptr;
			if (!beliefState)
				throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): beliefState handler does not implement data::beliefState");
			// add current states
			beliefState->set(pBelief->getSamples(), pBelief->getHypothesesToSample());
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, beliefPtr, to<Data>(dataCurrentPtr)->getView());
			context.write("Transform: handler %s, inputs %s, %s...\n", beliefHandler->getID().c_str(), beliefPtr->first.c_str(), beliefItem.c_str());
		}


	}
	// render point cloud
	to<Data>(dataCurrentPtr)->createRender();
	return ptr;
}

grasp::data::Item::Map::iterator spam::PosePlanner::objectCapture(const Data::Mode mode, std::string &itemName) {
//	std::string& itemName = mode == Data::MODE_DEFAULT ? objectItem : mode != Data::MODE_MODEL ? queryItem : modelItem;
	const std::string itemNameRaw = itemName + "_raw";
	grasp::data::Handler* handler = mode == Data::MODE_DEFAULT ? objectHandler : mode != Data::MODE_MODEL ? queryHandler : modelHandler;
	grasp::data::Handler* handlerScan = mode != Data::MODE_MODEL ? queryHandlerScan : modelHandlerScan;
	grasp::Camera* camera = mode != Data::MODE_MODEL ? queryCamera : modelCamera;
	grasp::ConfigMat34::Seq scanPoseSeq = mode != Data::MODE_MODEL ? queryScanPoseSeq : modelScanPoseSeq;
	
	grasp::data::Capture* capture = is<grasp::data::Capture>(handlerScan);
	if (!capture)
		throw Message(Message::LEVEL_ERROR, "Handler %s does not support Capture interface", objectHandlerScan->getID().c_str());

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
	const std::string itemName = mode == Data::MODE_DEFAULT ? objectItem : mode != Data::MODE_MODEL ? queryItem : modelItem;
	grasp::data::Handler* handler = mode == Data::MODE_DEFAULT ? objectHandler : mode != Data::MODE_MODEL ? queryHandler : modelHandler;
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
	catch (const golem::MsgXMLParser &msg) { printf("%s\n", msg.str().c_str()); }

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
	golem::XMLData("ext_trj", const_cast<std::string&>(extTrajectory), context, create);

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
