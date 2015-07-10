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
//#define  GLUT_KEY_INSERT                    0x006C
//------------------------------------------------------------------------------

//-----------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;

//------------------------------------------------------------------------------

grasp::data::Data::Ptr spam::RagPlanner::Data::Desc::create(golem::Context &context) const {
	grasp::data::Data::Ptr data(new RagPlanner::Data(context));
	static_cast<RagPlanner::Data*>(data.get())->create(*this);
	return data;
}

spam::RagPlanner::Data::Data(golem::Context &context) : PosePlanner::Data(context), owner(nullptr) {
}

void spam::RagPlanner::Data::create(const Desc& desc) {
	PosePlanner::Data::create(desc);

	triggered = 0;
	replanning = false;
	release = false;
}

void spam::RagPlanner::Data::createRender() {
	PosePlanner::Data::createRender();
}

void spam::RagPlanner::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap) {
	PosePlanner::Data::load(prefix, xmlcontext, handlerMap);
}

void spam::RagPlanner::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	PosePlanner::Data::save(prefix, xmlcontext);
}

//------------------------------------------------------------------------------

void RagPlanner::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	PosePlanner::Desc::load(context, xmlcontext);
	
	xmlcontext = xmlcontext->getContextFirst("rag_planner");

	golem::XMLData("duration", trjDuration, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("idle", trjIdle, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData("extrapol_fac", trjExtrapolFac, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData(trjExtrapol, xmlcontext->getContextFirst("trajectory extrapol"));
	golem::XMLData("perf_off", trjPerfOff, xmlcontext->getContextFirst("trajectory"));

	golem::XMLData(*pProfileDesc, xmlcontext->getContextFirst("trajectory"));
	golem::XMLData(distance, xmlcontext->getContextFirst("trajectory distance"));

	golem::XMLData("planning_uncertainty", uncEnable, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("single_grasp_attempt", singleGrasp, const_cast<golem::XMLContext*>(xmlcontext));
	golem::XMLData("withdraw_to_home_pose", withdrawToHomePose, const_cast<golem::XMLContext*>(xmlcontext));

	golem::XMLData(fLimit, xmlcontext->getContextFirst("limit"), false);

	spam::XMLData(*objCollisionDescPtr, xmlcontext->getContextFirst("collision"));
}

//------------------------------------------------------------------------------

RagPlanner::RagPlanner(Scene &scene) : PosePlanner(scene) {
}

RagPlanner::~RagPlanner() {
	if (dataFTRaw.is_open()) dataFTRaw.close();
	if (dataFTRaw.is_open()) dataFTFiltered.close();
}

bool RagPlanner::create(const Desc& desc) {
	PosePlanner::create(desc); // throws

	if (desc.distance.size() < (size_t)info.getJoints().size())
		throw Message(Message::LEVEL_CRIT, "Player::create(): Invalid dimensionality of distance parameter");
	distance.set(desc.distance.data(), desc.distance.data() + info.getJoints().size(), *info.getJoints().begin());

	// profile
	desc.pProfileDesc->pCallbackDist = this;
	try {
		pProfile = desc.pProfileDesc->create(*controller); // throws
	}
	catch (const Message &msg) {
		throw Message(Message::LEVEL_ERROR, "Demo::performTrajectory(): unable to create profile");
	}

	trjDuration = desc.trjDuration;
	trjIdle = desc.trjIdle;

	if (desc.trjExtrapol.size() < (size_t)info.getJoints().size())
		throw Message(Message::LEVEL_CRIT, "Player::create(): Invalid dimensionality of trjExtrapol parameter");
	trjExtrapol.set(desc.trjExtrapol.data(), desc.trjExtrapol.data() + info.getJoints().size(), *info.getJoints().begin());
	trjExtrapolFac = desc.trjExtrapolFac;

	trjPerfOff = desc.trjPerfOff;

	//	robot->startActiveController();
	// point cloud associated with the true pose of the object (used only for sim tests)
	enableSimContact = true;
	enableForceReading = false;
	forcereadersilent = true;
	objectPointCloudPtr.reset(new grasp::Cloud::PointSeq());
	printing = false;
	//	w.setToDefault();
	//	w.points = 20000;// desc.maxModelPoints;
	//	collision.reset(new Collision(context, *manipulator));
	contactOccured = false;

	collisionPtr = desc.objCollisionDescPtr->create(*manipulator);

	std::stringstream prefix;
	prefix << std::fixed << std::setprecision(3) << "./data/boris/ftsensors/ftsensors" << "-" << context.getTimer().elapsed();
	contact = false;
	record = false;
	brecord = false;
	sepField = "\t";
	// writing data into a text file, open file
	auto strTorquesDesc = [=](std::ostream& ostr) {
		ostr << "idx" << "\t" <<  "timestamp" << "\t" << "contact" << "\t" << "thumb_0" << "\t" << "thumb_1" << "\t" << "thumb_2" << "\t" << "thumb_3" << "\t" <<
			"index_0" << "\t" << "index_1" << "\t" << "index_2" << "\t" << "index_3" << "\t" <<
			"middle_0" << "\t" << "middle_1" << "\t" << "middle_2" << "\t" << "middle_3" << "\t" <<
			"ring_0" << "\t" << "ring_1" << "\t" << "ring_2" << "\t" << "ring_3" << "\t" <<
			"pinky_0" << "\t" << "pinky_1" << "\t" << "pinky_2" << "\t" << "pinky_3" << "\t";/* <<
			"cthumb_0" << "\t" << "cthumb_1" << "\t" << "cthumb_2" << "\t" << "cthumb_3" << "\t" <<
			"cindex_0" << "\t" << "cindex_1" << "\t" << "cindex_2" << "\t" << "cindex_3" << "\t" <<
			"cmiddle_0" << "\t" << "cmiddle_1" << "\t" << "cmiddle_2" << "\t" << "cmiddle_3" << "\t" <<
			"cring_0" << "\t" << "cring_1" << "\t" << "cring_2" << "\t" << "cring_3" << "\t" <<
			"cpinky_0" << "\t" << "cpinky_1" << "\t" << "cpinky_2" << "\t" << "cpinky_3" << "\t";*/
	};
	//const std::string dataFTRawPath = makeString("%s_raw.txt", prefix.str().c_str());
	//dataFTRaw.open(dataFTRawPath);
	//strTorquesDesc(dataFTRaw);
	//dataFTRaw << std::endl;

	//const std::string dataFTFilteredPath = makeString("%s_filtered.txt", prefix.str().c_str());
	//dataFTFiltered.open(dataFTFilteredPath);
	//strTorquesDesc(dataFTFiltered);
	//dataFTFiltered << std::endl;

	//const std::string dataSimContactPath = makeString("%s_contact.txt", prefix.str().c_str());
	//dataSimContact.open(dataSimContactPath);
	//strTorquesDesc(dataSimContact);
	//dataSimContact << std::endl;

	steps = 0;
	windowSize = 40;

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
	handFilteredForce.assign(dimensions(), REAL_ZERO);
	// initialise table to memorise read forces size: dimensions() x windowSize
	forceInpSensorSeq.resize(dimensions());
	for (std::vector<RealSeq>::iterator i = forceInpSensorSeq.begin(); i != forceInpSensorSeq.end(); ++i)
		i->assign(windowSize, REAL_ZERO);
	// Forcereader utilities
	collectFTInp = [=](const Controller::State& state, grasp::RealSeq& force) {
//		golem::CriticalSectionWrapper csw(csHandForce);
		for (I32 i = 0; i < dimensions(); ++i)
			forceInpSensorSeq[i][steps] = force[i];
		steps = (I32)(++steps % windowSize);
		ftFilter(state, handFilteredForce);
	};

	ftFilter = [=](const Controller::State&, grasp::RealSeq& filteredforce) {
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

		filteredforce = conv();
	};

	/*ArmHandForce **/armHandForce = dynamic_cast<ArmHandForce*>(&*activectrlMap.find("ArmHandForce+ArmHandForce")->second);
	if (!armHandForce)
		throw Message(Message::LEVEL_ERROR, "RagPlanner::create(): armHandForce is invalid");
	armMode = armHandForce->getArmCtrl()->getMode();
	handMode = armHandForce->getHandCtrl()->getMode();
	context.write("Active control mode [arm/hand]: %s/%s\n", ActiveCtrlForce::ModeName[armMode], ActiveCtrlForce::ModeName[handMode]);
	// set guards for the hand
	fLimit = desc.fLimit;
	guardsReader = [=](const Controller::State &state, grasp::RealSeq& force, std::vector<golem::Configspace::Index> &joints) {
		joints.clear();
		joints.reserve(handInfo.getJoints().size());
		// the loop skips the last joint because it's coupled with the 3rd joint.
		for (Configspace::Index i = handInfo.getJoints().begin(); i != handInfo.getJoints().end(); ++i) {
			const size_t k = i - handInfo.getJoints().begin();
			if (Math::abs(handFilteredForce[k]) > fLimit[k])
				joints.push_back(i);
		}
		//for (size_t i = 0; i < size; ++i)
		//	if (Math::abs(force[i]) > fLimit[i])
		//		throw Message(Message::LEVEL_NOTICE, "Robot::handForceReaderDflt(): F[%u/%u] = (%3.3lf)", i, size, force[i]);
	};

	armHandForce->setHandForceReader([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws		
		// associate random noise [-0.1, 0.1]
		static int indexjj = 0;
		if (enableSimContact)
			for (auto i = 0; i < force.size(); ++i)
				force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);
		
		RealSeq links; links.assign(dimensions(), REAL_ZERO);
		size_t jointInCollision = enableSimContact && !objectPointCloudPtr->empty() ? collisionPtr->simulate(desc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(lookupState()), /*links*/force, true) : 0;

		auto strTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const RealSeq& guardSeq/*const bool contact*/) {
			ostr << state.t << "\t";
			std::string c = jointInCollision > 0 ? "y" : "n"; //contact ? "y" : "n";
			ostr << c.c_str() << "\t";
			//grasp::RealSeq torques;
			//torques.assign((size_t)robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
			//robot->readFT(state, torques);
			for (auto i = 0; i < forces.size(); ++i)
				ostr << forces[i] << "\t";
			//for (auto i = 0; i < guardSeq.size(); ++i)
			//	ostr << guardSeq[i] << "\t";
		};
		if (++indexjj % 10 == 0) 	{
			collectFTInp(state, force);
			if (record) {
				//breakPoint();
				dataFTRaw << indexjj << "\t";
				strTorques(dataFTRaw, state, force, links/*contact*/);
				dataFTRaw << std::endl;
				grasp::RealSeq f = getFilterForce();
				dataFTFiltered << indexjj << "\t";
				strTorques(dataFTFiltered, state, f, links/*contact*/);
				dataFTFiltered << std::endl;
			}
		}

		if (!enableForceReading)
			return;
		
		triggeredGuards.clear();
		std::vector<Configspace::Index> joints;
		guardsReader(state, force, joints);
		for (U32 i = 0; i != joints.size(); ++i) {
			FTGuard guard(*manipulator);
			guard.create(joints[i]);
			const size_t k = joints[i] - handInfo.getJoints().begin();
			guard.force = force[k];
			guard.threshold = fLimit[k];
			triggeredGuards.push_back(guard);
		}
		if (!triggeredGuards.empty()) {
//			std::stringstream str;
			for (FTGuard::Seq::const_iterator g = triggeredGuards.begin(); g != triggeredGuards.end(); ++g)
				g->str();
			if (force.size() >= 20) {
				context.write("Forces: Thumb: [%3.3lf %3.3lf %3.3lf %3.3lf] Index [%3.3lf %3.3lf %3.3lf %3.3lf] Middle [%3.3lf %3.3lf %3.3lf %3.3lf] Ring [%3.3lf %3.3lf %3.3lf %3.3lf] Pinky [%3.3lf %3.3lf %3.3lf %3.3lf]\n",
					force[0], force[1], force[2], force[3],
					force[4], force[5], force[6], force[7], 
					force[8], force[9], force[10], force[11], 
					force[12], force[13], force[14], force[15],
					force[16], force[17], force[18], force[19]);
				grasp::RealSeq f = getFilterForce();
				context.write("filter: Thumb: [%3.3lf %3.3lf %3.3lf %3.3lf] Index [%3.3lf %3.3lf %3.3lf %3.3lf] Middle [%3.3lf %3.3lf %3.3lf %3.3lf] Ring [%3.3lf %3.3lf %3.3lf %3.3lf] Pinky [%3.3lf %3.3lf %3.3lf %3.3lf]\n",
					f[0], f[1], f[2], f[3],
					f[4], f[5], f[6], f[7],
					f[8], f[9], f[10], f[11],
					f[12], f[13], f[14], f[15],
					f[16], f[17], f[18], f[19]);
			}
		
			throw Message(Message::LEVEL_NOTICE, "spam::Robot::handForceReader(): Triggered %d guard(s).\n", triggeredGuards.size());
		}
	}); // end robot->setHandForceReader

	armHandForce->setEmergencyModeHandler([=]() {
		enableForceReading = false;
		contactOccured = true;
		armHandForce->getArmCtrl()->setMode(armMode);
		armHandForce->getHandCtrl()->setMode(handMode);
	}); // end robot->setEmergencyModeHandler

	//	auto strTorquesDesc = [=](std::ostream& ostr) {
//		ostr << "timestamp" << grasp::to<Data>(data)->sepField << "contact" << grasp::to<Data>(data)->sepField << "thumb_0" << grasp::to<Data>(data)->sepField << "thumb_1" << grasp::to<Data>(data)->sepField << "thumb_2" << grasp::to<Data>(data)->sepField << "thumb_3" << grasp::to<Data>(data)->sepField <<
//			"index_0" << grasp::to<Data>(data)->sepField << "index_1" << grasp::to<Data>(data)->sepField << "index_2" << grasp::to<Data>(data)->sepField << "index_3" << grasp::to<Data>(data)->sepField <<
//			"middle_0" << grasp::to<Data>(data)->sepField << "middle_1" << grasp::to<Data>(data)->sepField << "middle_2" << grasp::to<Data>(data)->sepField << "middle_3" << grasp::to<Data>(data)->sepField <<
//			"ring_0" << grasp::to<Data>(data)->sepField << "ring_1" << grasp::to<Data>(data)->sepField << "ring_2" << grasp::to<Data>(data)->sepField << "ring_3" << grasp::to<Data>(data)->sepField <<
//			"pinky_0" << grasp::to<Data>(data)->sepField << "pinky_1" << grasp::to<Data>(data)->sepField << "pinky_2" << grasp::to<Data>(data)->sepField << "pinky_3" << grasp::to<Data>(data)->sepField;
//	};
//
//	// writing data into a text file, open file
//	const std::string dataIndexPathRaw = grasp::makeString("%s_raw.txt", prefix.str().c_str());
//	const std::string dataIndexPathFiltered = grasp::makeString("%s_filtered.txt", prefix.str().c_str());
//
////	if (enableSimContact) {
//	// raw data from drl hand FT
//		dataFTRaw.open(dataIndexPathRaw);
//		// writing data into a text file, prepare headers
//		dataFTRaw << "#index" << grasp::to<Data>(data)->sepField;
//		strTorquesDesc(dataFTRaw);
//		dataFTRaw << std::endl;
//		// filtered data from DLR hand FT
//		dataFTFiltered.open(dataIndexPathFiltered);
//		// writing data into a text file, prepare headers
//		dataFTFiltered << "#index" << grasp::to<Data>(data)->sepField;
//		strTorquesDesc(dataFTFiltered);
//		dataFTFiltered << std::endl;
////	}
//
//	// simulates contacts between robot's hand and the object's point cloud
//	robot->setHandForceReader([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws		
//		// associate random noise [-0.1, 0.1]
//		static int indexjj = 0;
//		if (enableSimContact)
//			for (auto i = 0; i < force.size(); ++i)
//				force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);
//
//		size_t jointInCollision = enableSimContact && !objectPointCloudPtr->empty() ? collisionPtr->simulate(desc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(robot->recvState().config), force) : 0;
//
//		//// writing data into a text file
//		auto strTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const bool contact) {
//			ostr << state.t << grasp::to<Data>(data)->sepField;
//			std::string c = contact ? "y" : "n";
//			ostr << c.c_str() << grasp::to<Data>(data)->sepField;
//			//grasp::RealSeq torques;
//			//torques.assign((size_t)robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
//			//robot->readFT(state, torques);
//			for (auto i = 0; i < forces.size(); ++i)
//				ostr << forces[i] << grasp::to<Data>(data)->sepField;
//		};
//		if (++indexjj % 10 == 0) 	{
//			robot->collectFTInp(state, force);
//			if (record) {
//				//breakPoint();
//				dataFTRaw << indexjj << grasp::to<Data>(data)->sepField;
//				strTorques(dataFTRaw, state, force, contact);
//				dataFTRaw << std::endl;
//				grasp::RealSeq f = robot->getFilterForce();
//				dataFTFiltered << indexjj << grasp::to<Data>(data)->sepField;
//				strTorques(dataFTFiltered, state, f, contact);
//				dataFTFiltered << std::endl;
//			}
//		}
//
//		if (!enableForceReading)
//			return;
//
//		triggeredGuards.clear();
//		std::vector<Configspace::Index> joints;
//		robot->guardsReader(robot->recvState().command, force, joints);
//		for (U32 i = 0; i != joints.size(); ++i) {
//			FTGuard guard(*manipulator);
//			guard.create(joints[i]);
//			const size_t k = joints[i] - robot->getStateHandInfo().getJoints().begin();
//			guard.force = force[k];
//			guard.threshold = robot->getFlimit(k);
//			triggeredGuards.push_back(guard);
//		}
//		if (!triggeredGuards.empty()) {
////			std::stringstream str;
//			for (FTGuard::Seq::const_iterator g = triggeredGuards.begin(); g != triggeredGuards.end(); ++g)
//				g->str();
//			if (force.size() >= 20) {
//				context.write("Forces: Thumb: [%3.3lf %3.3lf %3.3lf %3.3lf] Index [%3.3lf %3.3lf %3.3lf %3.3lf] Middle [%3.3lf %3.3lf %3.3lf %3.3lf] Ring [%3.3lf %3.3lf %3.3lf %3.3lf] Pinky [%3.3lf %3.3lf %3.3lf %3.3lf]\n",
//					force[0], force[1], force[2], force[3],
//					force[4], force[5], force[6], force[7], 
//					force[8], force[9], force[10], force[11], 
//					force[12], force[13], force[14], force[15],
//					force[16], force[17], force[18], force[19]);
//				grasp::RealSeq f = robot->getFilterForce();
//				context.write("filter: Thumb: [%3.3lf %3.3lf %3.3lf %3.3lf] Index [%3.3lf %3.3lf %3.3lf %3.3lf] Middle [%3.3lf %3.3lf %3.3lf %3.3lf] Ring [%3.3lf %3.3lf %3.3lf %3.3lf] Pinky [%3.3lf %3.3lf %3.3lf %3.3lf]\n",
//					f[0], f[1], f[2], f[3],
//					f[4], f[5], f[6], f[7],
//					f[8], f[9], f[10], f[11],
//					f[12], f[13], f[14], f[15],
//					f[16], f[17], f[18], f[19]);
//			}
//
//			throw Message(Message::LEVEL_NOTICE, "spam::Robot::handForceReader(): Triggered %d guard(s).\n", triggeredGuards.size());
//		}
//	}); // end robot->setHandForceReader
//
//	//robot->setHandContactDetector([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws
//	//	// writing data into a text file
//	//	auto strTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const bool contact) {
//	//		ostr << state.t << grasp::to<Data>(data)->sepField;
//	//		std::string c = contact ? "y" : "n";
//	//		ostr << c.c_str() << grasp::to<Data>(data)->sepField;
//	//		//grasp::RealSeq torques;
//	//		//torques.assign((size_t)robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
//	//		//robot->readFT(state, torques);
//	//		for (auto i = 0; i < forces.size(); ++i)
//	//			ostr << forces[i] << grasp::to<Data>(data)->sepField;
//	//	};
//	//	static int indexjj = 0;
//	//	//auto breakPoint = [&]() { if (waitKey(10) == ' ') contact = !contact; };
//	//	//auto recording = [&]() { if (waitKey(10) == 27) record = !record; };
//	//	//recording();
//	//	if (record && ++indexjj % 10 == 0) {
//	//		//breakPoint();
//	//		dataFTFiltered << indexjj << grasp::to<Data>(data)->sepField;
//	//		strTorques(dataFTFiltered, state, force, contact);
//	//		dataFTFiltered << std::endl;
//	//	}
//	//	if (!enableForceReading)
//	//		return;
//	//	std::vector<Configspace::Index> joints;
//	//	robot->guardsReader(robot->recvState().command, force, joints);
//	//	for (U32 i = 0; i != joints.size(); ++i) {
//	//		FTGuard guard(*manipulator);
//	//		guard.create(joints[i]);
//	//		guard.force = force[i];
//	//		guard.threshold = robot->getFlimit(i);
//	//		triggeredGuards.push_back(guard);
//	//	}
//	//	if (!triggeredGuards.empty()) {
//	//		for (FTGuard::Seq::const_iterator g = triggeredGuards.begin(); g != triggeredGuards.end(); ++g)
//	//			g->str();
//	//		if (force.size() >= 20) {
//	//			context.write("Forces: Thumb: [%3.3lf %3.3lf %3.3lf %3.3lf] Index [%3.3lf %3.3lf %3.3lf %3.3lf] Middle [%3.3lf %3.3lf %3.3lf %3.3lf] Ring [%3.3lf %3.3lf %3.3lf %3.3lf] Pinky [%3.3lf %3.3lf %3.3lf %3.3lf]\n",
//	//				force[0], force[1], force[2], force[3],
//	//				force[4], force[5], force[6], force[7],
//	//				force[8], force[9], force[10], force[11],
//	//				force[12], force[13], force[14], force[15],
//	//				force[16], force[17], force[18], force[19]);
//	//		}
//	//		throw Message(Message::LEVEL_NOTICE, "spam::Robot::handForceReader(): Triggered %d guard(s).\n", triggeredGuards.size());
//	//	}
//	//});
//
//	enableControllers = false;
//	// called after guards are triggered
//	robot->setEmergencyModeHandler([=]() {
////		context.debug("Emergency mode handler\n");
//		enableForceReading = false;
//		contactOccured = true;
//		//grasp::to<Data>(currentDataPtr)->replanning = true;
//		robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_DISABLED);
//		robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_DISABLED);
////		updateAndResample(currentDataPtr);
////		triggeredGuards.clear();
//		//robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
//		//robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
//	}); // end robot->setEmergencyModeHandler

	uncEnable = desc.uncEnable;
	singleGrasp = desc.singleGrasp;
	withdrawToHomePose = desc.withdrawToHomePose;
	posterior = true;

	triggeredGuards.clear();

	ragDesc = desc;

	handBounds.clear();
	robotStates.clear();

	isGrasping = false;

	//	collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
	//	objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));

	menuCmdMap.insert(std::make_pair("L", [=]() {
		auto select = [&](Strategy &strat) {
			switch (option("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
			case 'N':
			{
				strat = Strategy::NONE_STRATEGY;
				return;
			}
			case 'E':
			{
				strat = Strategy::ELEMENTARY;
				return;
			}
			case 'M':
			{
				strat = Strategy::MYCROFT;
				return;
			}
			case 'I':
			{
				strat = Strategy::IR3NE;
				return;
			}
			}
			//switch (strat) {
			//case NONE_STRATEGY:
			//	strat = Strategy::ELEMENTARY;
			//	break;
			//case ELEMENTARY:
			//	strat = Strategy::MYCROFT;
			//	break;
			//case MYCROFT:
			//	strat = Strategy::IR3NE;
			//	break;
			//case IR3NE:
			//	strat = Strategy::NONE_STRATEGY;
			//	break;
			//default:
			//	strat = Strategy::NONE_STRATEGY;
			//	break;
			//}
		};
		to<Data>(dataCurrentPtr)->createRender();
		// set the simulated object
		if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
			collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
		}

		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		data::Trajectory* trajectory = is<data::Trajectory>(item->second.get());
		// play
		Controller::State::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");
		
		for (;;) {
			if (!execute(dataCurrentPtr, inp))
				return;
			if (contactOccured) {
				//grasp::to<Data>(cdata->second)->replanning = false;
				contactOccured = false;
				updateAndResample(dataCurrentPtr);
				enableForceReading = false;
				continue;
			}
			// grasp
			enableForceReading = false;
			//grasp::to<Data>(dataPtr)->actionType = action::GRASP;
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataCurrentPtr)->actionType));
			if (!execute(dataCurrentPtr, inp))
				return;
					
			// lifting
			enableForceReading = false;
			//grasp::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataCurrentPtr)->actionType));
			if (execute(dataCurrentPtr, inp))
				return;
		
			break;
		}

	//finish
	context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("O", [=]() {
		record = !record;
		context.write("record ft %s\n", record ? "ON" : "OFF");
	}));

	menuCmdMap.insert(std::make_pair("X", [=]() {
		// setup initial variables
		const Controller::State home = lookupState();

		//--------------------------------------------------------------------//
		// UTILITY FUNCTIONS
		//--------------------------------------------------------------------//
		auto strategy = [&](const Strategy &strat) -> std::string {
			switch (strat) {
			case Strategy::NONE_STRATEGY:
				return "NONE";
			case Strategy::ELEMENTARY:
				return "ELEMENTARY";
			case Strategy::MYCROFT:
				return "MYCROFT";
			case Strategy::IR3NE:
				return "IR3NE";
			default:
				return "";
			}
		};
		auto select = [&](Strategy &strat) {
			//switch (option("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
			//case 'N':
			//{
			//	strat = Strategy::NONE_STRATEGY;
			//	return;
			//}
			//case 'E':
			//{
			//	strat = Strategy::ELEMENTARY;
			//	return;
			//}
			//case 'M':
			//{
			//	strat = Strategy::MYCROFT;
			//	return;
			//}
			//case 'I':
			//{
			//	strat = Strategy::IR3NE;
			//	return;
			//}
			//}
			switch (strat) {
			case NONE_STRATEGY:
				strat = Strategy::ELEMENTARY;
				break;
			case ELEMENTARY:
				strat = Strategy::MYCROFT;
				break;
			case MYCROFT:
				strat = Strategy::IR3NE;
				break;
			case IR3NE:
				strat = Strategy::NONE_STRATEGY;
				break;
			default:
				strat = Strategy::NONE_STRATEGY;
				break;
			}
		};
		auto executeCmd = [&](const std::string command) {
			MenuCmdMap::const_iterator cmd = menuCmdMap.find(command);
			if (cmd == menuCmdMap.end())
				throw Cancel(makeString("Error: impossible to execute command %s.", command.c_str()).c_str());
			cmd->second();
		};
		auto reset = [&]() {
			enableForceReading = false;
			pHeuristic->enableUnc = false;
			pHeuristic->setPointCloudCollision(false);
			Controller::State::Seq seq, out;
			findTrajectory(lookupState(), &home, nullptr, 0, seq);
			profile(this->trjDuration, seq, out, true);
			sendTrajectory(out);
			// repeat every send waypoint until trajectory end
			for (U32 i = 0; controller->waitForBegin(); ++i) {
				if (universe.interrupted())
					throw Exit();
				if (controller->waitForEnd(0))
					break;
			}
		};

		std::ofstream logFile("./data/boris/experiments/output.log");
		std::ofstream elementaryLog("./data/boris/experiments/elementary.log");
		std::ofstream mycroftLog("./data/boris/experiments/mycroft.log");
		std::ofstream ireneLog("./data/boris/experiments/irene.log");

		grasp::to<Data>(dataCurrentPtr)->stratType = Strategy::NONE_STRATEGY;
		grasp::to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
		grasp::to<Data>(dataCurrentPtr)->replanning = false;
		U32 modelViews = 0, queryViews = 0, trials = 5;
		const U32 maxFailures = 2, maxIterations = 3;
		// collects results as string
		std::string results;

		// command keys
		std::string createModelCmd("PM");
		std::string createQueryCmd("PQ");
		std::string itemTransCmd("IT");
		std::string itemConvCmd("IC");

		//--------------------------------------------------------------------//
		// CREATE A MODEL POINT CLOUD
		//--------------------------------------------------------------------//
		// object name
		std::string objectName("");
		readString("Enter object name: ", objectName);
		// create a model for the object
		context.write("Create a model...\n");
		executeCmd(createModelCmd);
		context.write("done.\n");

		//--------------------------------------------------------------------//
		// TRIALS
		//--------------------------------------------------------------------//
		for (U32 trial = 0; trial < trials; ++trial) {
			//--------------------------------------------------------------------//
			// TRIAL HEADER
			//--------------------------------------------------------------------//
			results = grasp::makeString("%u\t%u\t%u", modelViews, queryViews, trial);
			// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
			select(grasp::to<Data>(dataCurrentPtr)->stratType);
			context.write("execute %s trajectory (%s)\n", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), actionToString(grasp::to<Data>(dataCurrentPtr)->actionType));
			logFile << grasp::makeString("Strategy: %s\n", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str());
			// Setup trial data
			TrialData::Ptr tdata = createTrialData();
			tdata->grasped = false;
			tdata->silent = true;
			tdata->setup(context, rand);
			tdata->name = grasp::makeString("trial_%s_v%u_0%u", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), queryViews, trial); //"trial_v1_01"; 
			// object name
			tdata->object = objectName;
			tdata->path = grasp::makeString("%s%s/%s%s", this->trial->path.c_str(), tdata->object.c_str(), tdata->name.c_str(), this->trial->extTrial.c_str());
			context.write("Object name = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nModel view(s) = %u\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
				tdata->object.c_str(), tdata->name.c_str(), modelViews, queryViews, trial, tdata->path.c_str());
			logFile << grasp::makeString("Object name = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nModel view(s) = %u\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
				tdata->object.c_str(), tdata->name.c_str(), modelViews, queryViews, trial, tdata->path.c_str());
			trialPtr = getTrialData().insert(getTrialData().begin(), TrialData::Map::value_type(grasp::to<TrialData>(tdata)->path, tdata));

			//--------------------------------------------------------------------//
			// CREATE A QUERY POINT CLOUD
			//--------------------------------------------------------------------//
			// create a query for the object
			context.write("Create a query...\n");
			executeCmd(createQueryCmd);
			// set the simulated object
			if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
				collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
				objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
			}
			context.write("done.\n");

			context.write("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(dataCurrentPtr)->queryPoints.size(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
			logFile << grasp::makeString("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(dataCurrentPtr)->queryPoints.size(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);

			//--------------------------------------------------------------------//
			// CREATE A PREDICTIVE MODEL FOR THE GRASP
			//--------------------------------------------------------------------//
			// create a grasp
			context.write("Create a predictive contact model for the grasp...\n");
			dataItemLabel = modelGraspItem;
			executeCmd(itemTransCmd);
			context.write("done.\n");

			//--------------------------------------------------------------------//
			// CREATE A PREDICTIVE QUERY FOR THE GRASP
			//--------------------------------------------------------------------//
			// create a grasp
			context.write("Create a predictive contact query for the grasp...\n");
			// retrieve model contact
			grasp::data::Item::Map::iterator modelContactPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelGraspItem);
			if (modelContactPtr == to<Data>(dataCurrentPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", modelGraspItem.c_str());
			// retrieve query point cloud
			grasp::data::Item::Map::iterator queryPtr = to<Data>(dataCurrentPtr)->itemMap.find(queryItem);
			if (queryPtr == to<Data>(dataCurrentPtr)->itemMap.end())
				throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", queryItem.c_str());

			// generate features
			data::Transform* transform = is<data::Transform>(queryGraspHandler);
			if (!transform)
				throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", queryGraspHandler->getID().c_str());

			data::Item::List list;
			list.insert(list.end(), modelContactPtr);
			list.insert(list.end(), queryPtr);
			data::Item::Ptr queryGraspItemPtr = transform->transform(list);

			// insert processed object, remove old one
			grasp::data::Item::Map::iterator queryGraspPtr;
			RenderBlock renderBlock(*this);
			{
				golem::CriticalSectionWrapper cswData(getCS());
				to<Data>(dataCurrentPtr)->itemMap.erase(queryGraspItem);
				queryGraspPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(queryGraspItem, queryGraspItemPtr));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryGraspPtr, to<Data>(dataCurrentPtr)->getView());
			}
			to<Data>(dataCurrentPtr)->createRender();
			context.write("done.\n");

			//--------------------------------------------------------------------//
			// CREATE A GRASP TRAJECTORY
			//--------------------------------------------------------------------//
			context.write("Create grasp trajectory...\n");
			data::Convert* convert = is<data::Convert>(queryGraspPtr);
			if (!convert)
				throw Message(Message::LEVEL_ERROR, "Handler %s does not support Convert interface", queryHandlerTrj->getID().c_str());
			// convert
			data::Item::Ptr queryGraspTrj = convert->convert(*queryHandlerTrj);
			// insert processed object, remove old one
			grasp::data::Item::Map::iterator queryGraspTrjPtr;
			RenderBlock renderBlock2(*this);
			{
				golem::CriticalSectionWrapper cswData(getCS());
				to<Data>(dataCurrentPtr)->itemMap.erase(queryItemTrj);
				queryGraspTrjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(queryItemTrj, queryGraspTrj));
				Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryGraspTrjPtr, to<Data>(dataCurrentPtr)->getView());
			}
			to<Data>(dataCurrentPtr)->createRender();
			context.write("done.\n");

			data::Trajectory* trajectory = is<data::Trajectory>(queryGraspTrj.get());
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Handler %s does not support Trajectory interface", queryHandlerTrj->getID().c_str());
			Controller::State::Seq inp = trajectory->getWaypoints();
			if (inp.size() < 3)
				throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");


			//--------------------------------------------------------------------//
			// EXECUTE TRIAL
			//--------------------------------------------------------------------//
			U32 iteration = 1, failures = 0;
			RBCoord obj(simQueryFrame);
			for (; failures < maxFailures && iteration < maxIterations;) {
				grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_ON_QUERY;
				grasp::RBCoord queryPose(grasp::to<Data>(dataCurrentPtr)->queryFrame);
				grasp::RBDist error;
				error.lin = obj.p.distance(queryPose.p);
				error.ang = obj.q.distance(queryPose.q);
				context.write("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
				context.write("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
					obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
					queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
					error.lin, error.ang);
				logFile << grasp::makeString("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
				logFile << grasp::makeString("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
					obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
					queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
					error.lin, error.ang);
				if (iteration == 1 && failures == 0)
					results = grasp::makeString("%s\t%.2f\t%.6f\t%.6f", results.c_str(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100, error.lin, error.ang);

				context.debug("\n----------------------------------------------------------\n");
				if (!execute(dataCurrentPtr, inp)) { // if it fails to find a trajectory repeat 
					++failures;
					continue;
				}

				if (contactOccured && grasp::to<Data>(dataCurrentPtr)->stratType != Strategy::ELEMENTARY) {
					//grasp::to<Data>(cdata->second )->replanning = false;
					contactOccured = false;
					updateAndResample(dataCurrentPtr);
					enableForceReading = false;
					++iteration;
					//bool r = unlockContact();
					//context.write("unlock contact %s\n", r ? "TRUE" : "FALSE");
					continue;
				}
				// grasp
				//if (error.lin < 0.01) {
				grasp::to<Data>(dataCurrentPtr)->actionType = action::GRASP;
				context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
				logFile << grasp::makeString("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
				if (!execute(dataCurrentPtr, inp)) {// not successfully close fingers
					++failures;
					continue;
				}
				grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_LIFT;
				context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
				logFile << grasp::makeString("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
				if (!execute(dataCurrentPtr, inp)) {// not successfully close fingers
					++failures;
					continue;
				}
				if (option("YN", "Success? (Y/N)") == 'Y')
					to<TrialData>(trialPtr)->grasped = true;
				else 
					to<TrialData>(trialPtr)->grasped = false;
				context.write("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
				logFile << grasp::makeString("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
				results = grasp::makeString("%s\t%.6f\t%.6f\t%u\t%u\n", results.c_str(), error.lin, error.ang, grasp::to<TrialData>(trialPtr)->grasped ? 1 : 0, iteration);
				for (auto i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i) {
					context.write("%s\n", (*i).str().c_str());
					logFile << grasp::makeString("%s\n", (*i).str().c_str());
				}

				context.write("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
				logFile << grasp::makeString("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
				results = grasp::makeString("%s\t%.6f\t%.6f\t%u\t%u\n", results.c_str(), error.lin, error.ang, grasp::to<TrialData>(trialPtr)->grasped ? 1 : 0, iteration);
				break;
			}
			// save results
			context.write("%s", results.c_str());
			logFile << grasp::makeString("%s", results.c_str());
			switch (grasp::to<Data>(dataCurrentPtr)->stratType){
			case Strategy::ELEMENTARY:
				elementaryLog << grasp::makeString("%s", results.c_str());
				break;
			case Strategy::MYCROFT:
				mycroftLog << grasp::makeString("%s", results.c_str());
				break;
			case Strategy::IR3NE:
				ireneLog << grasp::makeString("%s", results.c_str());
				break;
			}
			results = "";
			context.debug("----------------------------------------------------------\n");
			context.debug("%s\n", grasp::to<TrialData>(trialPtr)->toString(*controller).c_str());
			context.debug("----------------------------------------------------------\n");
			context.debug("----------------------------------------------------------\n");
			if (failures < maxFailures) {
				context.write("Sava trial in %s\n", grasp::to<TrialData>(trialPtr)->path.c_str());
				logFile << grasp::makeString("Sava trial in %s\n", grasp::to<TrialData>(trialPtr)->path.c_str());
			}
			else {
				context.write("Failed trial\n");
				logFile << "Failed trial\n";
			}
			//grasp::to<TrialData>(trialPtr)->save();
			context.write("------------------END TRIAL----------------------------------------\n");
			logFile << "------------------END TRIAL----------------------------------------\n";
			// reset robot
			reset();

			//----------------------------------
			//if (!execute(dataCurrentPtr, inp))
			//	return;
			//if (contactOccured) {
			//	//grasp::to<Data>(cdata->second)->replanning = false;
			//	contactOccured = false;
			//	updateAndResample(dataCurrentPtr);
			//	enableForceReading = false;
			//	continue;
			//}
			//// grasp
			//enableForceReading = false;
			////grasp::to<Data>(dataPtr)->actionType = action::GRASP;
			//context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataCurrentPtr)->actionType));
			//if (!execute(dataCurrentPtr, inp))
			//	return;

			//// lifting
			//enableForceReading = false;
			////grasp::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
			//context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataCurrentPtr)->actionType));
			//if (execute(dataCurrentPtr, inp))
			//	return;

			//break;
		}

		//finish
		context.write("Done!\n");
	}));

	menuCmdMap.insert(std::make_pair("H", [=]() {
		// setup initial variables
		const Controller::State home = lookupState();
		//		std::ofstream logFile("./data/boris/trajectory/experiments.log");
		auto resetPlanning = [&]() {
			pHeuristic->testCollision = false; // debug collissions with point clouds
			pHeuristic->enableUnc = false; // plans trajectories with IG
			pHeuristic->setPointCloudCollision(false); // collision with mean pose hypothesis' point cloud
			isGrasping = false; // used in perform(). Prevent from enable forcereader while grasping
			enableForceReading = false; // enables/disable force reader
		};

		auto reset = [&]() {
			enableForceReading = false;
			pHeuristic->enableUnc = false;
			pHeuristic->setPointCloudCollision(false);
			Controller::State::Seq seq, out;
			findTrajectory(lookupState(), &home, nullptr, 0, seq);
			profile(this->trjDuration, seq, out, true);
			sendTrajectory(out);
			// repeat every send waypoint until trajectory end
			for (U32 i = 0; controller->waitForBegin(); ++i) {
				if (universe.interrupted())
					throw Exit();
				if (controller->waitForEnd(0))
					break;
			}
		};

		if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
			collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
		}

		// load trajectory
		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		data::Trajectory* trajectory = is<data::Trajectory>(item->second.get());
		// play
		Controller::State::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
		Controller::State cend = inp[1];
		resetPlanning();
		pHeuristic->setPointCloudCollision(true);
		Controller::State::Seq approach;
		findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		Controller::State::Seq seq;
		profile(this->trjDuration, approach, seq, true);

		for (U32 t = 0; t < 10; ++t) {
			// create trial data
			TrialData::Ptr tdata = createTrialData();
			tdata->grasped = false;
			tdata->silent = true;
			tdata->setup(context, rand);
			trialPtr = getTrialData().insert(getTrialData().begin(), TrialData::Map::value_type(grasp::to<TrialData>(tdata)->path, tdata));

			// transform trajectory
			Controller::State::Seq out;
			transformTrajectory(grasp::to<TrialData>(tdata)->objectPoseTrn.toMat34(), seq.begin(), seq.end(), out);
			//trajectory->setWaypoints(out);
			trajectory->createTrajectory(out);
			// done!
			createRender();

			// record activated
			brecord = true;

			// perform
			perform(dataCurrentPtr->first, item->first, out);

			reset(); // send the robot home
		}
	}));

	//		auto select = [&](Strategy &strat) {
	//			switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
	//			case 'N':
	//			{
	//				strat = Strategy::NONE_STRATEGY;
	//				return;
	//			}
	//			case 'E':
	//			{
	//				strat = Strategy::ELEMENTARY;
	//				return;
	//			}
	//			case 'M':
	//			{
	//				strat = Strategy::MYCROFT;
	//				return;
	//			}
	//			case 'I':
	//			{
	//				strat = Strategy::IR3NE;
	//				return;
	//			}
	//			}
	//			//switch (strat) {
	//			//case NONE_STRATEGY:
	//			//	strat = Strategy::ELEMENTARY;
	//			//	break;
	//			//case ELEMENTARY:
	//			//	strat = Strategy::MYCROFT;
	//			//	break;
	//			//case MYCROFT:
	//			//	strat = Strategy::IR3NE;
	//			//	break;
	//			//case IR3NE:
	//			//	strat = Strategy::NONE_STRATEGY;
	//			//	break;
	//			//default:
	//			//	strat = Strategy::NONE_STRATEGY;
	//			//	break;
	//			//}
	//		};
	//
	//		// set the simulated object
	//		if (!grasp::to<Data>(dataPtr)->simulateObjectPose.empty()) {
	//			collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
	//			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
	//		}
	//
	//		grasp::RobotState::Map::iterator trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
	//		if (trajectory->second.size() < 3)
	//			context.write("Error: the selected trajectory have not at least 3 waypoints.");
	//
	//		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
	//		select(grasp::to<Data>(dataPtr)->stratType);
	//		grasp::to<Data>(dataPtr)->actionType = action::NONE_ACTION;
	//		grasp::to<Data>(dataPtr)->replanning = false;
	//		Controller::State::Seq inp = grasp::RobotState::makeCommand(trajectory->second);
	//		// open the fingers for the pregrasp
	//		for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
	//			auto j = robot->getStateHandInfo().getJoints(robot->getStateHandInfo().getChains().begin()).begin() + 1;
	//			inp[1].cpos[j] = 0.0;
	//		}
	//		for (;;) {
	//			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
	//			if (!execute(dataPtr, inp))
	//				return;
	//			if (contactOccured) {
	//				//grasp::to<Data>(cdata->second)->replanning = false;
	//				contactOccured = false;
	//				updateAndResample(dataPtr);
	//				enableForceReading = false;
	//				continue;
	//			}
	//			// grasp
	//			enableForceReading = false;
	//			//grasp::to<Data>(dataPtr)->actionType = action::GRASP;
	//			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
	//			if (!execute(dataPtr, inp))
	//				return;
	//			
	//			// lifting
	//			enableForceReading = false;
	//			//grasp::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
	//			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
	//			if (execute(dataPtr, inp))
	//				return;
	//
	//			break;
	//		}
	//		context.write("Finish Experiment\n");
	//		//context.write("%s\n", grasp::to<TrialData>(trialPtr)->toString(*robot->getController()).c_str());
	//		return;
	//	}

	ragDesc = desc;

	return true;
}

void RagPlanner::render() const {
	PosePlanner::render();
//	handRenderer.reset();
	{
		golem::CriticalSectionWrapper csw(getCS());
		debugRenderer.render();
	}
}

//------------------------------------------------------------------------------

void RagPlanner::findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting) {
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

void RagPlanner::createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory) {
	if (!pcend && !pwend)
		throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): no target specified");

	// Trajectory from initial position to end position
	for (golem::U32 i = 0; i < trajectoryTrials; ++i) {
		if (universe.interrupted())
			throw Exit();
		context.debug("Player::findTrajectory(): Planning movement...\n");
		// All bounds are treated as obstacles
		uiPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);

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

grasp::RBDist RagPlanner::trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
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

grasp::RBDist RagPlanner::findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
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

//------------------------------------------------------------------------------

Real RagPlanner::distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
	Real dist = REAL_ZERO;
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		dist += distance[i] * Math::sqr(prev[i] - next[i]);
	return Math::sqrt(dist);
}

Real RagPlanner::distCoord(Real prev, Real next) const {
	return Math::abs(prev - next);
}

bool RagPlanner::distCoordEnabled(const Configspace::Index& index) const {
	return true;
}

//------------------------------------------------------------------------------

void RagPlanner::profile(golem::SecTmReal duration, const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& out, const bool silent) const {
	if (inp.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::profile(): At least two waypoints required");

	if (!silent)
		readNumber("Trajectory duration: ", duration);

	const golem::SecTmReal t = out.empty() ? context.getTimer().elapsed() : out.back().t;
	const Controller::State begin(inp.front(), t), end(inp.back(), begin.t + duration);

	golem::Controller::State::Seq::iterator ptr, tmpend;

	// create trajectory
	ptr = out.end();
	tmpend = out.end();
	pProfile->create(inp.begin(), inp.end(), begin, end, out, ptr, tmpend);
	// profile trajectory
	tmpend = out.end();
	pProfile->profile(out, ptr, tmpend);
}

void RagPlanner::perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	findTrajectory(lookupState(), &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

	// create trajectory item
	data::Item::Ptr itemTrajectory;
	data::Handler::Map::const_iterator handlerPtr = handlerMap.find(trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unknown default trajectory handler %s", trajectoryHandler.c_str());
	data::Handler* handler = is<data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): invalid default trajectory handler %s", trajectoryHandler.c_str());
	itemTrajectory = handler->create();
	data::Trajectory* trajectoryIf = is<data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to create trajectory using handler %s", trajectoryHandler.c_str());
	trajectoryIf->setWaypoints(completeTrajectory);

	// block displaying the current item
	RenderBlock renderBlock(*this);

	// test trajectory
	if (testTrajectory) {
		// insert trajectory to data with temporary name
		const std::string itemLabelTmp = item + dataDesc->sepName + makeString("%f", context.getTimer().elapsed());
		ScopeGuard removeItem([&]() {
			UI::removeCallback(*this, getCurrentHandler());
			{
				golem::CriticalSectionWrapper csw(getCS());
				to<Data>(dataCurrentPtr)->itemMap.erase(itemLabelTmp);
			}
			createRender();
		});
		{
			golem::CriticalSectionWrapper csw(getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemLabelTmp, itemTrajectory));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// enable GUI interaction and refresh
		UI::addCallback(*this, getCurrentHandler());
		createRender();
		// prompt user
		EnableKeyboardMouse enableKeyboardMouse(*this);
		option("\x0D", "Press <Enter> to accept trajectory...");
	}

	// go to initial state
	sendTrajectory(initTrajectory);
	// wait until the last trajectory segment is sent
	controller->waitForEnd();

	// start recording
	recordingStart(data, item, true);
	recordingWaitToStart();

	// send trajectory
	sendTrajectory(trajectory);

	record = brecord ? true : false;
	Controller::State::Seq robotPoses; robotPoses.clear();
	TwistSeq ftSeq; ftSeq.clear();
	std::vector<grasp::RealSeq> forceInpSensorSeq;
	// repeat every send waypoint until trajectory end
	for (U32 i = 0; controller->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		// print every 10th robot state
		if (i % 10 == 0) {
			context.write("State #%d (%s)\r", i, enableForceReading ? "Y" : "N");
			Controller::State s = lookupState();
			//RealSeq handTorques; handTorques.assign(dimensions(), REAL_ZERO);
			//if (armHandForce) armHandForce->getHandForce(handTorques);
			//forceInpSensorSeq.push_back(handTorques);
			//robotPoses.push_back(s);
			//Twist wrench; if (graspSensorForce) graspSensorForce->readSensor(wrench, s.t);
			//ftSeq.push_back(wrench);
			if (!isGrasping && i > 150)
				enableForceReading = expectedCollisions(s);
			//if (!isGrasping && !enableForceReading && i > 150 && expectedCollisions(s))
			//	enableForceReading = true;
		}
	}
	enableForceReading = false;
	if (record) {
		std::stringstream prefix;
		prefix << std::fixed << std::setprecision(3) << "./data/boris/experiments/" << item << "-" << context.getTimer().elapsed();


		auto strSimTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const U32 jointInCollision) {
			//ostr << state.t << "\t";
			std::string c = jointInCollision > 0 ? "y" : "n"; //contact ? "y" : "n";
			ostr << c.c_str() << "\t";
			for (auto i = 0; i < forces.size(); ++i)
				ostr << forces[i] << "\t";
		};
		// writing data into a text file, data conversion
		auto strMat34 = [=](std::ostream& ostr, const Mat34& mat) {
			Quat quat(mat.R);
			ostr << mat.p.x << "\t" << mat.p.y << "\t" << mat.p.z << "\t" << quat.x << "\t" << quat.y << "\t" << quat.z << "\t" << quat.w << "\t";
		};
		auto strMat34Desc = [=](std::ostream& ostr, const std::string& prefix, U32 index) {
			ostr << prefix << "x" << index << "\t" << prefix << "y" << index << "\t" << prefix << "z" << index << "\t" << prefix << "qx" << index << "\t" << prefix << "qy" << index << "\t" << prefix << "qz" << index << "\t" << prefix << "qw" << index << "\t";
		};
		auto strState = [=](std::ostream& ostr, const Controller::State& state, const Controller::State::Info& info) {
			ostr << state.t << "\t";
			WorkspaceJointCoord wjc;
			controller->jointForwardTransform(state.cpos, wjc);
			for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
				ostr << std::fixed << std::setprecision(6) << state.cpos[i] << "\t";
				//			strMat34(ostr, wjc[i]);
			}
		};
		auto strStateDesc = [=](std::ostream& ostr, const Controller::State::Info& info, const std::string& prefix) {
			ostr << prefix << "t" << "\t";
			for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
				ostr << prefix << "j" << *i << "\t";
				//			strMat34Desc(ostr, prefix, *i);
			}
		};
		auto strFT = [=](std::ostream& ostr, const Twist& twist) {
			ostr << twist.v.x << "\t" << twist.v.y << "\t" << twist.v.z << "\t" << twist.w.x << "\t" << twist.w.y << "\t" << twist.w.z << "\t";
		};
		auto strFTDesc = [=](std::ostream& ostr, const std::string& prefix) {
			ostr << prefix << "vx" << "\t" << prefix << "vy" << "\t" << prefix << "vz" << "\t" << prefix << "wx" << "\t" << prefix << "wy" << "\t" << prefix << "wz" << "\t";
		};
		auto strTorquesDesc = [=](std::ostream& ostr) {
			ostr << /*"timestamp" << "\t" << "contact" << "\t" <<*/ "thumb_0" << "\t" << "thumb_1" << "\t" << "thumb_2" << "\t" << "thumb_3" << "\t" <<
				"index_0" << "\t" << "index_1" << "\t" << "index_2" << "\t" << "index_3" << "\t" <<
				"middle_0" << "\t" << "middle_1" << "\t" << "middle_2" << "\t" << "middle_3" << "\t" <<
				"ring_0" << "\t" << "ring_1" << "\t" << "ring_2" << "\t" << "ring_3" << "\t" <<
				"pinky_0" << "\t" << "pinky_1" << "\t" << "pinky_2" << "\t" << "pinky_3" << "\t";
		};
		auto strTorques = [=](std::ostream& ostr, const grasp::RealSeq& forces) {
			//ostr << state.t << grasp::to<Data>(data)->sepField;
			//std::string c = contact ? "y" : "n";
			//ostr << c.c_str() << grasp::to<Data>(data)->sepField;
			for (auto i = 0; i < forces.size(); ++i)
				ostr << std::fixed << std::setprecision(6) << forces[i] << "\t";
		};
		auto strSimTorquesDesc = [=](std::ostream& ostr) {
			ostr << /*"timestamp" << "\t" <<*/ "contact" << "\t" << "thumb_0" << "\t" << "thumb_1" << "\t" << "thumb_2" << "\t" << "thumb_3" << "\t" <<
				"index_0" << "\t" << "index_1" << "\t" << "index_2" << "\t" << "index_3" << "\t" <<
				"middle_0" << "\t" << "middle_1" << "\t" << "middle_2" << "\t" << "middle_3" << "\t" <<
				"ring_0" << "\t" << "ring_1" << "\t" << "ring_2" << "\t" << "ring_3" << "\t" <<
				"pinky_0" << "\t" << "pinky_1" << "\t" << "pinky_2" << "\t" << "pinky_3" << "\t";
		};


		// writing data into a text file, open file
		const std::string dataIndexPath = grasp::makeString("%s.txt", prefix.str().c_str());
		std::ofstream dataIndex(dataIndexPath);

		// writing data into a text file, prepare headers
		dataIndex << "#index" << "\t";
		strStateDesc(dataIndex, handInfo, std::string("cfg_"));
		strFTDesc(dataIndex, std::string("ft_"));
		strTorquesDesc(dataIndex);
		strSimTorquesDesc(dataIndex);
		dataIndex << std::endl;

		// writing data into a text file
		U32 j = 0, c = 0;
		//for (grasp::RobotState::Seq::const_iterator i = robotStates.begin(); i != robotStates.end(); ++i) {
		//	dataIndex << ++j << "\t";
		//	strState(dataIndex, i->command, robot->getStateHandInfo());
		//	strState(dataIndex, i->config, robot->getStateHandInfo());
		//	strFT(dataIndex, i->getSensorData<Twist>(grasp::RobotState::SENSOR_WRENCH));
		//	strTorques(dataIndex, i->command, "n");
		//	dataIndex << std::endl;
		//}
		for (U32 indexjj = 0; indexjj < robotPoses.size(); ++indexjj) {
			//RealSeq links; links.assign(dimensions(), REAL_ZERO);
			//size_t jointInCollision = !objectPointCloudPtr->empty() ? collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(robotPoses[indexjj]), links, true) : 0;
			//dataSimContact << indexjj << "\t";
			//strTorques(dataSimContact, links, jointInCollision);
			//dataSimContact << std::endl;

			dataIndex << ++j << "\t";
			strState(dataIndex, robotPoses[indexjj], handInfo);
			strFT(dataIndex, ftSeq[indexjj]);
			strTorques(dataIndex, forceInpSensorSeq[indexjj]);
			RealSeq links; links.assign(dimensions(), REAL_ZERO);
			size_t jointInCollision = !objectPointCloudPtr->empty() ? collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(robotPoses[indexjj]), links, true) : 0;
			strSimTorques(dataIndex, robotPoses[indexjj], links, jointInCollision);
			dataIndex << std::endl;
		}
	}
	record = brecord = false;

	// stop recording
	recordingStop(trajectoryIdlePerf);
	recordingWaitToStop();

	// insert trajectory
	{
		golem::CriticalSectionWrapper csw(getCS());
		data::Data::Map::iterator data = dataMap.find(recorderData);
		if (data == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to find Data %s", recorderData.c_str());
		data->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f", dataDesc->sepName.c_str(), recorderStart), itemTrajectory));
	}

	context.write("Performance finished!\n");
}

//------------------------------------------------------------------------------

bool RagPlanner::execute(data::Data::Map::iterator dataPtr, golem::Controller::State::Seq& trajectory) {
	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
	bool silent = to<Data>(dataPtr)->actionType != action::NONE_ACTION;
//	context.debug("execute(): silen=%s, actionType=%s\n", silent ? "TRUE" : "FALSE", actionToString(grasp::to<Data>(dataPtr)->actionType));
	const int key = !silent ? option("MQTGU", "Press to (M)odel based or (Q)uery based grasp, (T)rajectory based planner, (G)rasp, (U)p lifting") :
		grasp::to<Data>(dataPtr)->actionType == action::IG_PLAN_M2Q ? 'M' :
		grasp::to<Data>(dataPtr)->actionType == action::IG_PLAN_ON_QUERY ? 'Q' :
		grasp::to<Data>(dataPtr)->actionType == action::IG_TRAJ_OPT ? 'T' :
		grasp::to<Data>(dataPtr)->actionType == action::GRASP ? 'G' : 'U';

	// lamda function to reset flags for planning
	auto resetPlanning = [&]() {
		pHeuristic->testCollision = false; // debug collissions with point clouds
		pHeuristic->enableUnc = false; // plans trajectories with IG
		pHeuristic->setPointCloudCollision(false); // collision with mean pose hypothesis' point cloud
		isGrasping = false; // used in perform(). Prevent from enable forcereader while grasping
		enableForceReading = false; // enables/disable force reader
	};
	grasp::RealSeq vl;
	vl.assign(20, REAL_ZERO);
	vl[0] = Real(0.22); vl[1] = Real(0.6); vl[2] = Real(0.2); vl[3] = Real(0.2);
	vl[4] = Real(0.0); vl[5] = Real(0.85); vl[6] = Real(0.2); vl[7] = Real(0.2);
	vl[8] = Real(0.0); vl[9] = Real(0.85); vl[10] = Real(0.2); vl[11] = Real(0.2);
	vl[12] = Real(0.0); vl[13] = Real(1.2); vl[14] = Real(1.0); vl[15] = Real(1.0);
	vl[16] = Real(0.0); vl[17] = Real(1.2); vl[18] = Real(1.0); vl[19] = Real(1.0);
	//grasp::RealSeq vv;
	//vv.assign(20, REAL_ZERO);
	//vv[0] = Real(0.0432042); vv[1] = Real(0.25618); vv[2] = Real(0.116704); vv[3] = Real(0.116704);
	//vv[12] = Real(0.0); vv[13] = Real(1.2); vv[14] = Real(1.0); vv[15] = Real(1.0);
	//vv[16] = Real(0.0); vv[17] = Real(1.2); vv[18] = Real(1.0); vv[19] = Real(1.0);

//	collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
//	objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
//	PosePlanner::TrialData::Iteration::Ptr iteration;
//	iteration.reset(new PosePlanner::TrialData::Iteration("", pBelief));

	switch (key){
	case 'M':
	{
		context.debug("Plan from Model to Query\n");
		std::string trjItemName("modelR2GTrj");
		resetPlanning();
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(true);
		isGrasping = false;

		Controller::State cend = lookupState();
		// transform w.r.t. query frame
		findTarget(grasp::to<Data>(dataPtr)->queryTransform, trajectory[1], cend);
		
		//auto i = handInfo.getChains().begin();
		//for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
		//	const size_t k = j - handInfo.getJoints().begin();
		//	cend.cpos[j] = vv[k];
		//}
		//i++; i++; i++;//move to the ring finger
		//for (; i != handInfo.getChains().end(); ++i) {
		//	for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
		//		const size_t k = j - handInfo.getJoints().begin();
		//		cend.cpos[j] = vv[k];
		//	}
		//}

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach;
		(void)findTrajectory(lookupState(), &cend, nullptr, 0, approach);

		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);

		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			data::Trajectory* trajectory = is<data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(out);
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, !silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}
//		printf("Finished perform\n");
		resetPlanning();
		return true;
	}
	case 'Q':
	{
		context.debug("Plan to Query\n");
		std::string trjItemName("queryR2GTrj");
		resetPlanning();
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		// pre-grasp pose w.r.t. query frame
		Controller::State cend = trajectory[1];
		pHeuristic->setPointCloudCollision(true);
		Controller::State::Seq approach;
		findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);

		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), data::Item::Map::value_type(trjItemName, queryHandlerTrj->create()));
			data::Trajectory* trajectory = is<data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(out);
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}
		resetPlanning();

		return true;
	}
	case 'T':
	{
		context.debug("Plan trajectory optimisation\n");
		std::string trjItemName("queryTrj");
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(false);
		isGrasping = false;
		// select index 
		U32 index = 1;
		selectIndex(trajectory, index, "waypoint");
		index -= 1;

		Controller::State cend = lookupState();
		cend.cpos = trajectory[index].cpos;

		// transform w.r.t. query frame
		findTarget(grasp::to<Data>(dataPtr)->queryTransform, trajectory[index], cend);

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach;
		try {
			findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		}
		catch (const Message &msg) { return false; }

		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);
		pHeuristic->setPointCloudCollision(true);
		
		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			data::Trajectory* trajectory = is<data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(out);
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}
		resetPlanning();

		return true;
	}
	case 'G':
	{
		context.debug("Plan grasping\n");
		std::string trjItemName("graspingTrj");
		resetPlanning();
		isGrasping = true;

		// current configuration (fingers opened)
		Controller::State cstart = lookupState();

		// select index 
		U32 index = 3;
		//selectIndex(trajectory, index, "waypoint");
		index -= 1;

		// grasp configuration (fingers closed)
		Controller::State cend = lookupState();
		for (auto i = handInfo.getJoints().begin(); i != handInfo.getJoints().end(); ++i)
			cend.cpos[i] = trajectory[index].cpos[i];

		for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
			for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
				const size_t k = j - handInfo.getJoints().begin();
				cend.cpos[j] = vl[k];
			}
			if (i == handInfo.getChains().begin() + 1)
				break;
		}

		Controller::State::Seq approach;
		approach.push_back(cstart);
		approach.push_back(cend);

		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);

		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			data::Trajectory* trajectory = is<data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(out);
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}

		// print distance to the targt configuration
		Controller::State cfg = lookupState();
		std::stringstream ss;
		for (auto i = handInfo.getJoints().begin(); i != handInfo.getJoints().end(); ++i) {
			const size_t k = i - handInfo.getJoints().begin();
			ss << "c=" << k << " [" << cend.cpos[i] - cfg.cpos[i] << "]/t";
		}
		context.write("Hand joints error:\n%s\n", ss.str().c_str());

		resetPlanning();
		return true;
	}
	case 'U':
	{
		context.debug("Plan lifting up\n");
		std::string trjItemName("liftingTrj");
		resetPlanning();
		isGrasping = true;

		Controller::State cend = lookupState();

		for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
			for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
				const size_t k = j - handInfo.getJoints().begin();
				cend.cpos[j] = vl[k];
			}
			if (i == handInfo.getChains().begin() + 1)
				break;
		}
		// transform w.r.t. query frame 
		findTarget(Mat34::identity(), cend, cend, true);

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(target), manipulator->getPose(target).toMat34());
		//renderHand(target, bounds, true);

		//for (auto j = handInfo.getJoints().begin() + 1; j != handInfo.getJoints().end(); ++j) {
		//	const size_t k = j - handInfo.getJoints().begin();
		//	target.cpos[j] = vl[k];
		//}

		Controller::State::Seq approach;
		findTrajectory(lookupState(), &cend, nullptr, 0, approach);

		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);

		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			data::Trajectory* trajectory = is<data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(out);
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform
		try {
			perform(dataPtr->first, ptr->first, out, silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}

		resetPlanning();
		return true;
	}
	}
	return false;
}


//------------------------------------------------------------------------------

//grasp::Cloud::PointSeqMap::iterator RagPlanner::getTrnPoints(Data::Map::iterator dataPtr, const Mat34 &trn) {
//	grasp::Cloud::PointSeqMap::iterator points;
//	if (!grasp::to<Data>(dataPtr)->ptrPoints || (points = grasp::to<Data>(dataPtr)->getPoints<grasp::Cloud::PointSeqMap::iterator>(grasp::to<Data>(dataPtr)->points)) == grasp::to<Data>(dataPtr)->points.end())
//		throw Message(Message::LEVEL_NOTICE, "Director::getPoints(): No points are selected!");
//
//	grasp::RBCoord m(trn);
//	context.debug("Transform processed points <%.4f %.4f %.4f> [%.4f %.4f %.4f %.4f]\n", m.p.x, m.p.y, m.p.z, m.q.w, m.q.x, m.q.y, m.q.z);
//	// transform points
//	grasp::Cloud::transform(trn, points->second, points->second);
//	
//	return points;
//}
//
//grasp::Cloud::RawPointSeqMultiMap::iterator RagPlanner::getTrnRawPoints(Data::Map::iterator dataPtr, const Mat34 &trn) {
//	grasp::Cloud::RawPointSeqMultiMap::iterator points;
//	if ((points = grasp::to<Data>(dataPtr)->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(grasp::to<Data>(dataPtr)->pointsRaw)) == grasp::to<Data>(dataPtr)->pointsRaw.end())
//		throw Message(Message::LEVEL_NOTICE, "Director::getPoints(): No points are selected!");
//
//	grasp::RBCoord m(trn);
//	context.debug("Transform raw points <%.4f %.4f %.4f> [%.4f %.4f %.4f %.4f]\n", m.p.x, m.p.y, m.p.z, m.q.w, m.q.x, m.q.y, m.q.z);
//	// transform points
//	grasp::Cloud::RawPointSeqVal val;
//	val.first = points->first;
//	grasp::Cloud::transform(trn, points->second, val.second);
//	return grasp::to<Data>(dataPtr)->insertPoints(val, grasp::to<Data>(dataPtr)->getLabelName(val.first));
//}

//------------------------------------------------------------------------------

Real RagPlanner::simContacts(const golem::Bounds::Seq::const_iterator &begin, const golem::Bounds::Seq::const_iterator &end, const golem::Mat34 pose) {
	// if the point cloud is empty simContacts return the default behavior of force reader.
	//if (objectPointCloudPtr->empty())
	//	return REAL_ZERO;

	//Rand rand;
	//bool intersect = false;
	//for (Bounds::Seq::const_iterator b = begin; b != end; ++b) {
	//	const size_t size = objectPointCloudPtr->size() < ragDesc.maxModelPoints ? objectPointCloudPtr->size() : ragDesc.maxModelPoints;
	//	for (size_t i = 0; i < size; ++i) {
	//		const Vec3 point = grasp::Cloud::getPoint(objectPointCloudPtr->at(objectPointCloudPtr->size() < ragDesc.maxModelPoints ? i : size_t(rand.next()) % size));
	//		//context.write("force reader\n");
	//		//context.write("bound pose <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
	//		if ((*b)->intersect(point)) {
	//			context.write("bound pose <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
	//			Vec3 v;
	//			Mat34 jointFrameInv;
	//			jointFrameInv.setInverse(pose);
	//			jointFrameInv.multiply(v, point);
	//			v.normalise();
	//			return v.z > REAL_ZERO ? -REAL_ONE : REAL_ONE;
	//		}
	//	}
	//}
	return REAL_ZERO;
}

//------------------------------------------------------------------------------

//void RagPlanner::renderData(Data::Map::const_iterator dataPtr) {
//	ShapePlanner::renderData(dataPtr);
//	{
//		golem::CriticalSectionWrapper csw(csDataRenderer);
//		debugRenderer.reset();
//		//if (!objectPointCloudPtr->empty()) {
//		//	grasp::Cloud::Point p = *objectPointCloudPtr->begin();
//		//	Mat34 m(Mat33::identity(), Vec3(p.x, p.y, p.z));
//		//	debugRenderer.addAxes(m, featureFrameSize * 10);
//		//}
//	}
//}
//
//void RagPlanner::renderContacts() {
//	context.write("render contacts...\n");
//	GenWorkspaceChainState gwcs;
//	robot->getController()->chainForwardTransform(robot->recvState().config.cpos, gwcs.wpos);
//	const size_t size = 10000;
//	for (grasp::Cloud::PointSeq::iterator i = grasp::to<Data>(currentDataPtr)->queryPoints.begin(); i != grasp::to<Data>(currentDataPtr)->queryPoints.end(); ++i) {
//		Real maxDist = REAL_MAX;
//		for (Chainspace::Index j = robot->getStateHandInfo().getChains().begin(); j != robot->getStateHandInfo().getChains().end(); ++j) {
//			const Real d = grasp::Cloud::getPoint(*i).distance(gwcs.wpos[j].p);
//			if (d < maxDist)
//				maxDist = d;
//		}
//		const Real lhd = pBelief->density(maxDist);
//		//context.write("lhd %f\n", lhd);
//		grasp::Cloud::setColour((lhd < 0.15) ? RGBA::BLUE : (lhd < 0.20) ? RGBA::YELLOW : (lhd < 0.25) ? RGBA::MAGENTA : RGBA::RED, *i);//RGBA(lhd*255, 0, 0, 0);
//	}
//	{
//		golem::CriticalSectionWrapper csw(csDataRenderer);
//		pointRenderer.reset();
//		data->appearance.draw(grasp::to<Data>(currentDataPtr)->queryPoints, pointRenderer);
//	}
//	context.write("Done.\n");
//}
//
//void RagPlanner::renderPose(const Mat34 &pose) {
////	testPose.reset();
//	testPose.addAxes(pose, featureFrameSize*10);
//}
//
//void RagPlanner::renderUpdate(const golem::Mat34 &pose, const grasp::RBPose::Sample::Seq &samples) {
//	testUpdate.reset();
//	renderPose(pose);
//	for (grasp::RBPose::Sample::Seq::const_iterator i = samples.begin(); i != samples.end(); ++i)
//		testUpdate.addAxes(Mat34(i->q, i->p), featureFrameSize*i->weight*10);
//}
//
//void RagPlanner::renderHand(const golem::Controller::State &state, const Bounds::Seq &bounds, bool clear) {
//	{		
//		golem::CriticalSectionWrapper csw(csDataRenderer);
//		debugRenderer.reset();
//		if (clear)
//			handBounds.clear();
//		if (!bounds.empty())
//			handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
//		debugRenderer.setColour(RGBA::BLACK);
//		debugRenderer.setLineWidth(Real(2.0));
//		debugRenderer.addWire(handBounds.begin(), handBounds.end());
//
//		Collision::Waypoint w;
//		w.points = 1000;
//		if (showIndices) {
//			//collision->draw(manipulator->getPose(state), debugRenderer);
//			//(*pBelief->getHypotheses().begin())->draw(debugRenderer, rand, manipulator->getPose(state));
//			Collision::FlannDesc flann;
//			flann.neighbours = 100;
//			flann.points = 1000;
//			flann.depthStdDev = Real(10);
//			flann.likelihood = Real(100);
//			std::vector<Configspace::Index> joints;
//			grasp::RealSeq forces;
//			(*pBelief->getHypotheses().begin())->draw(debugRenderer, manipulator->getPose(state), joints, forces, flann);
//		}
//			//(*pBelief->getHypotheses().begin())->draw(w, manipulator->getPose(state), debugRenderer);
//			//		handRenderer.renderWire(handBounds.begin(), handBounds.end());
//		//context.write("Hand bounds (size=%d)\n", handBounds.size());
//		//for (Bounds::Seq::const_iterator i = handBounds.begin(); i != handBounds.end(); ++i)
//		//	context.write("%s pose <%f %f %f>\n", (*i)->getName(), (*i)->getPose().p.x, (*i)->getPose().p.y, (*i)->getPose().p.z);
//	}
//
////	WorkspaceJointCoord jointPoses;
////	robot->getController()->jointForwardTransform(state.cpos, jointPoses);
////	{
////		golem::CriticalSectionWrapper csw(csDataRenderer);
////		handRenderer.reset();
////		Bounds::Desc::SeqPtr jointBounds;
////		Bounds::SeqPtr boundsSeq;
////		for (Configspace::Index i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i) {
////			Bounds::Desc::SeqPtr boundDescSeq = robot->getController()->getJoints()[i]->getBoundsDescSeq();
////			for (Bounds::Desc::Seq::iterator boundDesc = boundDescSeq->begin(); boundDesc != boundDescSeq->end(); ++boundDesc)
////				if ((*boundDesc) != NULL) { 
////					(*boundDesc)->pose = jointPoses[i];
////					jointBounds->push_back(*boundDesc);
////				}
//////			handRenderer.renderWire(boundDescSeq->begin(), boundDescSeq->end());
////		}
////		for (Bounds::Desc::Seq::iterator boundDesc = jointBounds->begin(); boundDesc != jointBounds->end(); ++boundDesc)
////			boundsSeq->push_back(boundDesc->get()->create());
////		handRenderer.renderWire(boundsSeq->begin(), boundsSeq->end());
////	}
//}
//
//void RagPlanner::renderWaypoints(const Bounds::Seq &bounds, bool clear) {
//		{
//			golem::CriticalSectionWrapper csw(csDataRenderer);
//			debugRenderer.reset();
//			if (clear)
//				waypointBounds.clear();
//			if (!bounds.empty())
//				waypointBounds.insert(waypointBounds.end(), bounds.begin(), bounds.end());
//			debugRenderer.setColour(RGBA::RED);
//			debugRenderer.setLineWidth(Real(2.0));
//			debugRenderer.addWire(waypointBounds.begin(), waypointBounds.end());
//			//		handRenderer.renderWire(handBounds.begin(), handBounds.end());
//			//context.write("Hand bounds (size=%d)\n", handBounds.size());
//			//for (Bounds::Seq::const_iterator i = handBounds.begin(); i != handBounds.end(); ++i)
//			//	context.write("%s pose <%f %f %f>\n", (*i)->getName(), (*i)->getPose().p.x, (*i)->getPose().p.y, (*i)->getPose().p.z);
//		}
//}

//------------------------------------------------------------------------------

void RagPlanner::updateAndResample(Data::Map::iterator dataPtr) {
	if (!pBelief || grasp::to<Data>(dataPtr)->queryPoints.empty())
		return;
	context.debug("RagPlanner::updateAndResample(): %d triggered guards:\n", /*grasp::to<Data>(dataPtr)->*/triggeredGuards.size());
	//for (std::vector<Configspace::Index>::const_iterator i = dataPtr->second.triggeredGuards.begin(); i != dataPtr->second.triggeredGuards.end(); ++i)
	//	context.write(" %d ", *i);
	//for (FTGuard::Seq::const_iterator i = /*grasp::to<Data>(dataPtr)->*/triggeredGuards.begin(); i != /*grasp::to<Data>(dataPtr)->*/triggeredGuards.end(); ++i)
	//	context.write(" %s ", i->str().c_str());
	//context.write("\n");

	//golem::Waypoint w(*robot->getController(), grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos);
	grasp::RealSeq force;
	force.assign(handInfo.getJoints().size(), REAL_ZERO);
	//force[]
////	printState(*grasp::to<Data>(dataPtr)->triggeredStates.begin(), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
//	robot->readFT(robot->recvState().config, force);
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
	golem::Waypoint w(*controller, lookupState().cpos/*grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos*/);
	context.write("update weights, triggered guards = %u\n", triggeredGuards.size());
	//pBelief->createUpdate(manipulator.get(), robot, w, /*grasp::to<Data>(dataPtr)->*/triggeredGuards, force);
	//for (auto i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i)
	//	context.write("%s\n", (*i).str().c_str());
	showDistrPoints = true;
	to<Data>(dataCurrentPtr)->createRender();
	::Sleep(1000);
	pBelief->createUpdate(collisionPtr, w, triggeredGuards, trialPtr != trialDataMap.end() ? grasp::to<TrialData>(trialPtr)->queryPointsTrn : grasp::RBCoord());
	::Sleep(1000);
	showDistrPoints = true;
	to<Data>(dataCurrentPtr)->createRender();

	//showSamplePoints = false; // shows hypotheses and mean pose
	//showDistrPoints = true;
	//if (screenCapture) universe.postScreenCaptureFrames(-1);
	//renderData(dataPtr);
	//if (screenCapture) universe.postScreenCaptureFrames(0);
	//::Sleep(10000);

	//for (grasp::RBPose::Sample::Seq::iterator sample = pBelief->getHypotheses().begin(); sample !=  pBelief->getHypotheses().end(); ++sample) {
	//	sample->weight = pHeuristic->evaluate(manipulator.get(), w, *sample, dataPtr->second.triggeredGuards, force, modelFrame);
	//	golem::kahanSum(norm, c, sample->weight);
	//}
//	context.write("normalising weights norm=%f (%.30f)\n", golem::REAL_ONE/norm, norm);
	// computes the normalised importance weight associated to each sample
	//dataPtr->second.normFac = norm = golem::REAL_ONE/norm;
	//for (grasp::RBPose::Sample::Seq::iterator sample = pBelief->getHypotheses().begin(); sample !=  pBelief->getHypotheses().end(); ++sample) 
	//	sample->weight *= norm;

	context.debug("resample (wheel algorithm)...\n");
	// resampling (wheel algorithm)
	pBelief->createResample();
	
	//dataPtr->second.samples.clear();
	//dataPtr->second.samples.reserve(K);

	// update the query frame
	context.debug("create hypotheses and update query frame...\n");
	grasp::RBPose::Sample mlFrame = pBelief->createHypotheses(modelPoints, modelFrame);
	grasp::to<Data>(dataPtr)->queryTransform/*actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame*/ = Mat34(mlFrame.q, mlFrame.p);
	//context.write("action frame poseData <%f %f %f> queryData <%f %f %f>\n", 
	//	grasp::to<Data>(dataPtr)->actionFrame.p.x, grasp::to<Data>(dataPtr)->actionFrame.p.y, grasp::to<Data>(dataPtr)->actionFrame.p.z,
	//	grasp::to<Data>(queryDataPtr)->actionFrame.p.x, grasp::to<Data>(queryDataPtr)->actionFrame.p.y, grasp::to<Data>(queryDataPtr)->actionFrame.p.z);

	// update query settings
	grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->queryTransform, modelFrame);
//	grasp::Cloud::transform(grasp::to<Data>(dataPtr)->queryTransform, modelPoints, grasp::to<Data>(dataPtr)->queryPoints);

	//grasp::to<Data>(dataPtr)->hypotheses.clear();
	//for (grasp::RBPose::Sample::Seq::const_iterator i = pBelief->getHypothesesToSample().begin(); i != pBelief->getHypothesesToSample().end(); ++i)
	//	grasp::to<Data>(dataPtr)->hypotheses.push_back(*i);
	
	//dataPtr->second.samples.push_back(mlFrame);
	//for (size_t i = 1; i < K; ++i)
	//	dataPtr->second.samples.push_back(pBelief->sampleHypothesis());
	//showQueryDistrib = true;
	//renderData(dataPtr);
	//showQueryDistrib = false;
	context.debug("render...\n");
//	showSamplePoints = true;
	//if (screenCapture) universe.postScreenCaptureFrames(-1);
	//::Sleep(50);
//	renderUncertainty(pBelief->getSamples());
	//showSamplePoints = true; // shows hypotheses and mean pose
	//showMeanHypothesis = false;
	//showDistrPoints = true;
//	if (screenCapture) universe.postScreenCaptureFrames(-1);
	pHeuristic->setHypothesisBounds();
	createRender();
	//::Sleep(100);
//	if (screenCapture) universe.postScreenCaptureFrames(0);

	//::Sleep(100);
	//if (screenCapture) universe.postScreenCaptureFrames(0);	
//	std::cout << "spam:update&resample 15\n";
}

//------------------------------------------------------------------------------

//void RagPlanner::function(Data::Map::iterator& dataPtr, int key) {
//	switch (key) {
//	case 'Z':
//	{
//		FTGuard::Seq triggerredGuards;
//		for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i) {
//			FTGuard g(*manipulator);
//			g.create(i);
//			(void)g.str();
//			triggerredGuards.push_back(g);
//		}
//		context.write("Guards:\n");
//		for (FTGuard::Seq::const_iterator g = triggeredGuards.begin(); g != triggeredGuards.end(); ++g)
//			context.write("%s\n", g->str().c_str());
//		context.write("done.\n");
//		return;
//		context.write("Load object ground truth pose\n");
//		grasp::Cloud::RawPointSeqMultiMap::const_iterator points;
//		try {
//			if (waitKey("YN", "Do you want to move the point cloud? (Y/N)...") == 'Y')
//				points = getTrnRawPoints(dataPtr, grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34());
//			//else
//			//	points = getPoints(dataPtr);
//
//			//objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(points->second));
//			renderData(dataPtr);
//		}
//		catch (const Message &msg) { context.notice("%s\n", msg.what()); }
//		return;
//	}
//	case 'B':
//	{
//		TrialData::Ptr tdata = createTrialData();
//		tdata->setup(context, rand);
//		switch (waitKey("NSLRIP", "Press a key to create (N)ew/(S)ave/(L)oad/(R)emove/(I)mport/(P)rocess data...")) {
//		case 'S':
//		{
//			//// save data
//			//const bool allData = getData().size() > 1 && waitKey("YN", "Save all data (Y/N)...") == 'Y';
//			//if (allData) {
//			//	for (Data::Map::iterator i = getData().begin(); i != getData().end(); ++i) {
//			//		context.write("Saving %s...\n", i->first.c_str());
//			//		i->second->save();
//			//	}
//			//}
//			//else {
//				const std::string path = tdata->path;
//				grasp::ScopeGuard guard([&]() {tdata->path = path; });
//				readPath("Enter data path to save: ", tdata->path, tdata->extTrial.c_str());
//				tdata->path = grasp::makeString("%s%s", tdata->getName().c_str(), tdata->extTrial.c_str());
//				tdata->save();
//			//}
//			context.write("Done!\n");
//			return;
//		}
//		}
//	}
////	case 'R':
////	{
////		if (poseDataPtr._Ptr == NULL) {
////			context.write("Unable to reset.\n");
////			return;
////		}
////		pBelief->reset();
////		//poseDataPtr->second.samples.clear();
////		//for (grasp::RBPose::Sample::Seq::const_iterator i = poseDataPtr->second.init.begin(); i != poseDataPtr->second.init.end(); ++i)
////		//	poseDataPtr->second.samples.push_back(*i);
////		showSampleColour = false;
////		showSampleFrame = false;
////		showSamplePoints = true;
////		showMLEFrame = true;
////		showMLEPoints = false;
//////		pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
//////		pHeuristic->setBelief(pBelief);
////		pHeuristic->enableUnc = uncEnable;	
////		grasp::to<Data>(dataPtr)->actionFrame = initActionFrame;
////		mlFrame = grasp::RBPose::Sample(grasp::to<Data>(dataPtr)->actionFrame);
////		grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->actionFrame, modelFrame);	
////		grasp::Cloud::transform(grasp::to<Data>(dataPtr)->actionFrame, grasp::to<Data>(dataPtr)->queryPoints, grasp::to<Data>(dataPtr)->queryPoints);
////		showModelPoints = false;
////		showModelFeatures = false;
////		showQueryDistrib = true;
////		showDistribution = false;
////		renderData(dataPtr);
////		renderTrialData(poseDataPtr);
////		return;
////	}
////	case 'B':
////	{
////		context.debug("testing active controller.\n");
////		robot->enableGuards();
////		Controller::State::Seq trj;
////		robot->createTrajectory(robot->recvState().config, NULL, &poseSeq[2], 0, trj);
////		
////		grasp::to<Data>(dataPtr)->action.clear();
////		//grasp::to<Data>(poseDataPtr)->action.clear();
////		profile(dataPtr, trj, trjApproachDuration, trjApproachIdle);
////		perform(dataPtr);
////		// retrieve robot pose from real justin
////		// print current robot joint position
////		//Controller::State state = robot->recvState().config;
////		//std::stringstream str;
////		//for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
////		//	str << " c" << (*i - *state.getInfo().getJoints().begin() + 1) << "=\"" << state.cpos[i] << "\"";
////		//context.write("<pose dim=\"%d\"%s/>\n", state.getInfo().getJoints().size(), str.str().c_str());
////		//golem::WorkspaceJointCoord wc;
////		//robot->getController()->jointForwardTransform(state.cpos, wc);
////		//for (golem::Chainspace::Index i = robot->getStateInfo().getChains().begin(); i < robot->getStateInfo().getChains().end(); ++i) {
////		//	context.write("Name: %s\n", robot->getController()->getChains()[i]->getName().c_str());
////		//	for (golem::Configspace::Index j = robot->getStateInfo().getJoints(i).begin(); j < robot->getStateInfo().getJoints(i).end(); ++j) {
////		//		const U32 k = U32(j - robot->getStateInfo().getJoints(i).begin());
////		//		const Mat34& m = wc[j];
////		//		context.write("Joint %d: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", k, m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
////		//	}
////		//}
////		break;
////		//if (poseDataPtr._Ptr == NULL) {
////		//	context.write("Unable to find the withdraw trajectory.\n");
////		//	return;
////		//}
////		//performWithdraw(poseDataPtr);
////		//return;
////		//if (modelPoints.empty() || queryDataPtr == getData().end() || grasp::to<Data>(queryDataPtr)->queryPoints.empty()) {
////		//	context.write("Unable to find a model or query. Please make sure you have generated a model and query.\n");
////		//	return;
////		//}
////		//context.write("Learning observational model\n");
////		//Controller::State::Seq seq;
////		//extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
////		//// compute the approach traectory in a new frame and its profile
////		//grasp::to<Data>(dataPtr)->action.clear();
////		//grasp::to<Data>(queryDataPtr)->action.clear();
////		//PosePlanner::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
////		//// copy the planned trajectory in my trial data
////		//grasp::to<Data>(poseDataPtr)->action.clear();
////		//const Controller::State pregrasp = *grasp::to<Data>(dataPtr)->action.begin();
////		//for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
////		//	grasp::to<Data>(poseDataPtr)->action.push_back(*i);
////		//
////		//golem::Controller::State::Seq initTrajectory;
////		//robot->createTrajectory(robot->recvState().command, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, initTrajectory);
////		//robot->enableGuards(false);
////		//robot->sendTrajectory(initTrajectory, true);
////		//robot->waitForEnd();
////
////		//Controller::State state = pregrasp;
////		//const Chainspace::Index i = robot->getStateHandInfo().getChains().begin() + 1;
////		//return;
////	}
//	case 'X':
//	{
//		if (getData().empty()) {
//			context.write("Error: no data is loaded.");
//			return;
//		}
//		auto strategy = [&] (const Strategy &strat) -> std::string {
//			switch (strat) {
//			case Strategy::NONE_STRATEGY:
//				return "NONE";
//			case Strategy::ELEMENTARY:
//				return "ELEMENTARY";
//			case Strategy::MYCROFT:
//				return "MYCROFT";
//			case Strategy::IR3NE:
//				return "IR3NE";
//			default:
//				return "";
//			}
//		};
//		auto select = [&](Strategy &strat) {
//			//switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
//			//	case 'N':
//			//	{
//			//		strat = Strategy::NONE_STRATEGY;
//			//		return;
//			//	}
//			//	case 'E':
//			//	{
//			//		strat = Strategy::ELEMENTARY;
//			//		return;
//			//	}
//			//	case 'M':
//			//	{
//			//		strat = Strategy::MYCROFT;
//			//		return;
//			//	}
//			//	case 'I':
//			//	{
//			//		strat = Strategy::IR3NE;
//			//		return;
//			//	}
//			//}
//			switch (strat) {
//			case NONE_STRATEGY:
//				strat = Strategy::ELEMENTARY;
//				break;
//			case ELEMENTARY:
//				strat = Strategy::MYCROFT;
//				break;
//			case MYCROFT:
//				strat = Strategy::IR3NE;
//				break;
//			case IR3NE:
//				strat = Strategy::NONE_STRATEGY;
//				break;
//			default:
//				strat = Strategy::NONE_STRATEGY;
//				break;
//			}
//		};
////		Strategy strat = Strategy::ELEMENTARY; //Strategy::NONE;
//		auto getNViews = [&](const size_t view) -> size_t {
//			switch (view) {
//			case 0:
//				return 1;
//			case 1:
//				return 3;
//			case 3:
//				return 5;
//			case 5:
//				return 7; 
//			default:
//				return 10;
//			}
//		};
//		const Controller::State home = robot->recvState().command;
//		auto reset = [&]() {
//			enableForceReading = false;
//			pHeuristic->enableUnc = false;
//			pHeuristic->setPointCloudCollision(false);
//			Controller::State::Seq seq, out;
//			robot->findTrajectory(robot->recvState().command, &home, nullptr, 0, seq);
//			profile(this->trjDuration, seq, out, true);
//			robot->sendTrajectory(out);
//			// repeat every send waypoint until trajectory end
//			for (U32 i = 0; robot->waitForBegin(); ++i) {
//				if (universe.interrupted())
//					throw grasp::Interrupted();
//				if (robot->waitForEnd(0))
//					break;
//			}
//
//		};
//		auto resetDataPtr = [&](Data::Map::iterator& dataPtr) {
//			grasp::to<Data>(dataPtr)->queryPoints.clear();
//			grasp::to<Data>(dataPtr)->queryTransform.setId();
//			grasp::to<Data>(dataPtr)->queryFrame.setId();
//			grasp::to<Data>(dataPtr)->poses.clear();
//			grasp::to<Data>(dataPtr)->hypotheses.clear();
//			grasp::to<Data>(dataPtr)->simulateObjectPose.clear();
//		};
//		auto strHypothesisDesc = [=](std::ostream& ostr) {
//			ostr << "px" << "\t" << "py" << "\t" << "pz" << "\t" << "qw" << "\t" <<
//				"qx" << "\t" << "qy" << "\t" << "qz" << std::endl;
//		};
//		auto strHypothesis = [=](std::ostream& ostr, const grasp::RBPose::Sample::Seq &poses) {
//			for (auto pose = poses.begin(); pose != poses.end(); ++pose)
//				ostr << (*pose).p.x << "\t" << (*pose).p.y << "\t" << (*pose).p.z << "\t" << (*pose).q.w << "\t" << (*pose).q.x << "\t" << (*pose).q.y << "\t" << (*pose).q.z << std::endl;
//		};
//		// save log file
////		std::ofstream hypothesisLog("./data/boris/experiments/hypotheses.log"); strHypothesisDesc(hypothesisLog);
//		std::ofstream logFile("./data/boris/experiments/mug4/output.log");
//		std::ofstream elementaryLog("./data/boris/experiments/mug4/elementary.log");
//		std::ofstream mycroftLog("./data/boris/experiments/mug4/mycroft.log");
//		std::ofstream ireneLog("./data/boris/experiments/mug4/irene.log");
//
//		context.write("================================================================================================================================\n");
//		logFile << "================================================================================================================================\n";
//		context.debug("Running analysis on:\n");
//		// iterate fort all the files loaded in data
//		grasp::Cloud::PointSeqMap::const_iterator points;
//		
//		// number of maximum failures allowed
//		const U32 maxFailures = 2, maxIterations = 10;
//		for (auto cdata = getData().begin(); cdata != getData().end(); ++cdata) {
//			//// load model points
//			//grasp::to<Data>(cdata->second)->ptrIndex = 0;
//			//points = getPoints(cdata);
//			//const std::string objectName = grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str();
//			//// LOAD MODEL POINTS
//			//context.debug("Creating %s model...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str());
//			//pBelief->createModel(points->second);
//			//modelFrame = Belief::createFrame(points->second);
//			//modelPoints = grasp::to<Data>(cdata->second)->modelPoints; // points->second;
//			//context.debug("Model point size %d, loaded point size %d\n", modelPoints.size(), grasp::to<Data>(cdata)->modelPoints.size());
//			//resetDataPointers();
//			//renderData(cdata);
//
//			U32 size = grasp::to<Data>(cdata->second)->pointsRaw.size();
//			std::vector<U32> indeces;
//			indeces.reserve(size);
//			for (auto i = 0; i < size; ++i) indeces.push_back(i);
//			// to retieve the newest processed points for the query
//			U32 indexProcessedPoints = 0;
//			for (auto trj = grasp::to<Data>(cdata)->trajectory.begin(); trj != grasp::to<Data>(cdata)->trajectory.end(); ++trj) {
//				context.debug("%s, trajectory %s\n", cdata->first.c_str(), trj->first.c_str());
//
//				grasp::to<Data>(cdata->second)->stratType = Strategy::NONE_STRATEGY;
//				// number of views selected for query
//				size_t VIEWS = 0;
//				for (; VIEWS < 7;) {
//					std::string results;
//					VIEWS = getNViews(VIEWS);
//					//readNumber("Enter views: ", VIEWS);
//					for (size_t TRIALS = 1; TRIALS <= 13; ++TRIALS) {
//						//readNumber("Enter trial: ", TRIALS);
//						for (;;) { // run through strategies
//							// View\tTrial\tCoverage\tlin\tang\tlin\tang\tgrasped\titerations\n
//							results = grasp::makeString("%u\t%u", VIEWS, TRIALS);
//							// reset query information
//							resetDataPtr(cdata);
//							// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
//							select(grasp::to<Data>(cdata->second)->stratType);
//							grasp::to<Data>(cdata->second)->stratType = Strategy::IR3NE;
//							// if strategy is NONE then we test a new trajectory
//							if (grasp::to<Data>(cdata->second)->stratType == Strategy::NONE_STRATEGY)
//								break;
//							context.write("Strategy: %s\n", strategy(grasp::to<Data>(cdata->second)->stratType).c_str());
//							logFile << grasp::makeString("Strategy: %s\n", strategy(grasp::to<Data>(cdata->second)->stratType).c_str());
//							// set up trial data
//							//this->trial->path = grasp::makeString("%s%s", this->trial->path.c_str(), cdata->first.c_str()/*, this->trial->extTrial.c_str()*/);
//							//context.write("path %s\n", this->trial->path.c_str());
//							//return;
//							grasp::to<Data>(cdata->second)->ptrIndex = 0;
//							points = getPoints(cdata);
//
//							TrialData::Ptr tdata = createTrialData();
//							tdata->grasped = false;
//							tdata->silent = true;
//							tdata->setup(context, rand);
//							tdata->name = grasp::makeString("trial_%s_v%s_0%s", strategy(grasp::to<Data>(cdata->second)->stratType).c_str(), boost::lexical_cast<std::string>(VIEWS).c_str(), boost::lexical_cast<std::string>(TRIALS).c_str()); //"trial_v1_01"; 
//							tdata->object = grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(); // objectName; //
//							tdata->path = grasp::makeString("%s%s/%s%s", this->trial->path.c_str(), tdata->object.c_str(), tdata->name.c_str(), this->trial->extTrial.c_str());
//							context.write("Object name = %s\nFile data = %s\nGrasp type = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
//								tdata->object.c_str(), grasp::to<Data>(cdata->second)->path.c_str(), trj->first.c_str(), tdata->name.c_str(), VIEWS, TRIALS, tdata->path.c_str());
//							logFile << grasp::makeString("Object name = %s\nFile data = %s\nGrasp type = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
//								tdata->object.c_str(), grasp::to<Data>(cdata->second)->path.c_str(), trj->first.c_str(), tdata->name.c_str(), VIEWS, TRIALS, tdata->path.c_str());
//							trialPtr = getTrialData().insert(getTrialData().begin(), TrialData::Map::value_type(grasp::to<TrialData>(tdata)->path, tdata));
//
//							// LOAD MODEL POINTS
//							context.debug("Creating %s model (%u points)...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), points->second.size());
//							pBelief->createModel(points->second);
//							modelFrame = Belief::createFrame(points->second);
//							modelPoints = grasp::to<Data>(cdata->second)->modelPoints; // points->second;
//							context.debug("Model point size %d, loaded point size %d\n", modelPoints.size(), grasp::to<Data>(cdata->second)->modelPoints.size());
//							resetDataPointers();
//							renderData(cdata);
//
//							// COMPUTE QUERY POINTS
//							grasp::to<Data>(cdata->second)->ptrPoints = false;
//							resetDataPointers();
//							//to<Data>(dataPtr)->ptrIndex = 0;
//							auto range = grasp::Cloud::find<grasp::Cloud::RawPointSeqMultiMap>(grasp::to<Data>(cdata->second)->pointsRaw, grasp::to<Data>(cdata->second)->ptrLabel);
//							if (range.first != range.second) {
//								Selection selection;
//								//U32 size = grasp::to<Data>(cdata->second)->pointsRaw.size();
//								//std::vector<U32> indeces;
//								//indeces.reserve(size);
//								//for (auto i = 0; i < size; ++i) indeces.push_back(i);
//								std::random_shuffle(indeces.begin(), indeces.end());
//								for (size_t pview = 0; pview < VIEWS; ++pview) {
//									grasp::to<Data>(cdata->second)->ptrIndex = indeces[pview];
//									grasp::Cloud::RawPointSeqMultiMap::const_iterator points = getTrnRawPoints(cdata, grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34());
//									context.debug("Query transform {(%f %f %f)}\n", grasp::to<TrialData>(trialPtr)->queryPointsTrn.p.x, grasp::to<TrialData>(trialPtr)->queryPointsTrn.p.y, grasp::to<TrialData>(trialPtr)->queryPointsTrn.p.z);
//									context.debug("Query: point cloud index=%d(%d), point size %d\n", indeces[pview], indeces.size(), points->second.size());
//									selection[grasp::to<Data>(cdata->second)->ptrIndex] = grasp::to<Data>(cdata->second)->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(grasp::to<Data>(cdata->second)->pointsRaw);
//								}
//								renderData(cdata);
//								try {
//									processPoints(cdata, selection, true);
//								}
//								catch (const Message& msg) {
//									context.write("%s\n", msg.str().c_str());
//									return;
//								}
//								grasp::to<Data>(cdata->second)->ptrIndex = grasp::to<Data>(cdata->second)->points.size() - 1;
//								auto range = grasp::Cloud::find<grasp::Cloud::PointSeqMap>(grasp::to<Data>(cdata->second)->points, grasp::to<Data>(cdata->second)->ptrIndex);
//								points = getPoints(cdata);
//								context.write("point name = %s, size = %d, ptrIndex = %d\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), grasp::to<Data>(cdata->second)->points.size(), grasp::to<Data>(cdata->second)->ptrIndex);
//								logFile << grasp::makeString("point name = %s, size = %d, ptrIndex = %d\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), grasp::to<Data>(cdata->second)->points.size(), grasp::to<Data>(cdata->second)->ptrIndex);
//								renderData(cdata);
//								//grasp::to<Data>(cdata->second)->ptrIndex = indexProcessedPoints++;
//								//for (grasp::to<Data>(cdata->second)->ptrIndex = 0; grasp::to<Data>(cdata->second)->ptrIndex < grasp::to<Data>(cdata->second)->points.size(); grasp::to<Data>(cdata->second)->ptrIndex++) {
//								//	points = getPoints(cdata);
//								//	//grasp::to<Data>(cdata->second)->ptrIndex = grasp::to<Data>(cdata->second)->points.size() - 1;
//								//	auto range = grasp::Cloud::find<grasp::Cloud::PointSeqMap>(grasp::to<Data>(cdata->second)->points, grasp::to<Data>(cdata->second)->ptrIndex);
//								//	context.write("point name = %s, size = %d, ptrIndex = %d\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), grasp::to<Data>(cdata->second)->points.size(), grasp::to<Data>(cdata->second)->ptrIndex);
//								//	renderData(cdata);
//								//	::Sleep(5000);
//								//}
//							//	for (;;) {
//							//		const int key = waitKey();
//							//		if (key == 13) { // enter
//							//			context.write("\n");
//							//			break;
//							//		}
//							//		else if (key == '[') {
//							//			if (grasp::to<Data>(cdata->second)->ptrIndex > 0) grasp::to<Data>(cdata->second)->ptrIndex--;
//							//		}
//							//		else if (key == ']') {
//							//			grasp::to<Data>(cdata->second)->ptrIndex++;
//							//		}
//							//		else if (key == 127) { // del
//							//			selection.clear();
//							//		}
//							//		else if (key == (GLUT_KEY_INSERT | Universe::KEY_SPECIAL)) { // ins
//							//			selection.clear();
//							//			size_t ptr = 0;
//							//			if (grasp::to<Data>(cdata->second)->ptrLabel == grasp::Cloud::LABEL_DEFAULT) {
//							//				for (grasp::Cloud::RawPointSeqMultiMap::iterator i = grasp::to<Data>(cdata->second)->pointsRaw.begin(); i != grasp::to<Data>(cdata->second)->pointsRaw.end(); ++i) selection[ptr++] = i;
//							//			}
//							//			else {
//							//				auto range = grasp::Cloud::find(grasp::to<Data>(cdata->second)->pointsRaw, grasp::to<Data>(cdata->second)->ptrLabel);
//							//				for (grasp::Cloud::RawPointSeqMultiMap::iterator i = range.first; i != range.second; ++i) selection[ptr++] = i;
//							//			}
//							//		}
//							//		else if (key == ' ') {  // <space>
//							//			auto ptr = selection.find(grasp::to<Data>(cdata->second)->ptrIndex);
//							//			if (selection.end() != ptr)
//							//				selection.erase(ptr);
//							//			else
//							//				selection[grasp::to<Data>(cdata->second)->ptrIndex] = grasp::to<Data>(cdata->second)->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(grasp::to<Data>(cdata->second)->pointsRaw);
//							//		}
//							//		else if (key == 27) {  // <Esc>
//							//			throw Cancel("\nCancelled");
//							//		}
//							//		else {
//							//			context.write(
//							//				"\n[]               change current point cloud\n"
//							//				"<Ins>/<Del>      select/clear all point clouds\n"
//							//				"<Space>          select/clear current point cloud\n"
//							//				"<Esc>            cancel\n"
//							//				"<Enter>          process\n"
//							//				);
//							//		}
//							//	}
//							//	if (selection.empty())
//							//		context.write("No point clouds to process\n");
//							//	else {
//							//		processPoints(cdata, selection);
//							//		renderData(cdata);
//							//		context.write("Done!\n");
//							//	}
//							//}
//							}
//							else {
//								context.write("Error: range.first != range.second\n");
//								return;
//							}
//
//							// QUERY CREATE
//							//points = getPoints(cdata);
//							grasp::to<Data>(cdata->second)->queryPoints = points->second;
//							context.write("Creating %s query...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str());
//							logFile << grasp::makeString("Creating %s query...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str());
//							pBelief->createQuery(points->second);
//							// compute transformation model -> query
//							const grasp::RBPose::Sample trn = pBelief->createHypotheses(modelPoints, modelFrame);
//							grasp::to<Data>(cdata->second)->queryTransform = trn.toMat34();
//							// update query settings
//							//resetDataPointers();
//							queryDataPtr = cdata;
//							grasp::to<Data>(cdata->second)->queryFrame.multiply(grasp::to<Data>(cdata->second)->queryTransform, modelFrame);
//							//grasp::Cloud::transform(grasp::to<Data>(cdata->second)->queryTransform, modelPoints, grasp::to<Data>(cdata->second)->queryPoints);
//							// copying belief
//							//grasp::to<Data>(cdata->second)->poses.clear();
//							grasp::to<Data>(cdata->second)->poses = pBelief->getSamples();
//							//grasp::to<Data>(cdata->second)->hypotheses.clear();
//							grasp::to<Data>(cdata->second)->hypotheses = pBelief->getHypothesesToSample();
//							context.debug("Belief poses = %d, hypotheses = %d\n", pBelief->getSamples().size(), pBelief->getHypotheses().size());
//
//							// set the simulated object as ground truth
//							//grasp::to<Data>(cdata->second)->simulateObjectPose.clear();
//							grasp::to<Data>(cdata->second)->simulateObjectPose.reserve(modelPoints.size());
//							for (U32 point = 0; point != modelPoints.size(); ++point) {
//								grasp::Cloud::Point p = modelPoints[point];
//								grasp::Cloud::setColour(golem::RGBA::MAGENTA, p);
//								grasp::to<Data>(cdata->second)->simulateObjectPose.push_back(p);
//							}
//							context.write("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(cdata->second)->queryPoints.size(), ((Real)grasp::to<Data>(cdata->second)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
//							logFile << grasp::makeString("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(cdata->second)->queryPoints.size(), ((Real)grasp::to<Data>(cdata->second)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
//							//grasp::Cloud::transform(grasp::to<Data>(cdata->second)->queryTransform, grasp::to<Data>(cdata->second)->simulateObjectPose, grasp::to<Data>(cdata->second)->simulateObjectPose);
//							Mat34 m = grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34();
//							m.p.y -= 0.005; //m.p.z += 0.01;
//							grasp::Cloud::transform(/*grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34()*/m, grasp::to<Data>(cdata->second)->simulateObjectPose, grasp::to<Data>(cdata->second)->simulateObjectPose);
//							enableForceReading = false; 
//							collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
//							objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
//
//							// render
//							showSamplePoints = true; // shows hypotheses and mean pose
//							showMeanHypothesis = false;
//							showDistrPoints = false;
//							showQueryPoints = false;
//							showQueryDistrib = true;
//							showObject = false;
//							// removes grey points from the model
//							showPoints = false;
//							//pointRenderer.reset();
//							renderData(cdata);
//
//							//strHypothesis(hypothesisLog, grasp::to<Data>(cdata->second)->poses);
//							//return;
//
//							// uncomment to interrupt the loop before planning thew trajectory
//							/*::Sleep(5000);
//							continue;*/
//
//							// PLAN TRAJECTORY
//							//currentDataPtr = queryDataPtr;
//
//							grasp::to<Data>(cdata->second)->replanning = false;
//							Controller::State::Seq inp = grasp::RobotState::makeCommand(trj->second);
//							// open the fingers for the pregrasp
//							for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
//								auto j = robot->getStateHandInfo().getJoints(robot->getStateHandInfo().getChains().begin()).begin() + 1;
//								inp[1].cpos[j] = 0.0;
//							}
//							Mat34 objectTransform;
//							objectTransform.multiply(grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34(), modelFrame);
//							grasp::RBCoord obj(objectTransform);
//							U32 iteration = 1, failures = 0;
//							for (; failures < maxFailures && iteration < maxIterations;) {
//								grasp::to<Data>(cdata->second)->actionType = action::IG_PLAN_M2Q;
//								grasp::RBCoord queryPose(grasp::to<Data>(cdata->second)->queryFrame);
//								grasp::RBDist error;
//								error.lin = obj.p.distance(queryPose.p);
//								error.ang = obj.q.distance(queryPose.q);
//								context.write("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
//								context.write("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
//									obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
//									queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
//									error.lin, error.ang);
//								logFile << grasp::makeString("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
//								logFile << grasp::makeString("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
//									obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
//									queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
//									error.lin, error.ang);
//								if (iteration == 1 && failures == 0)
//									results = grasp::makeString("%s\t%.2f\t%.6f\t%.6f", results.c_str(), ((Real)grasp::to<Data>(cdata->second)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100, error.lin, error.ang);
//
//								context.debug("\n----------------------------------------------------------\n");
//								if (!execute(cdata, inp)) { // if it fails to find a trajectory repeat 
//									++failures;
//									continue;
//								}
//
//								if (contactOccured && grasp::to<Data>(cdata->second)->stratType != Strategy::ELEMENTARY) {
//									//grasp::to<Data>(cdata->second )->replanning = false;
//									contactOccured = false;
//									updateAndResample(cdata);
//									enableForceReading = false;
//									++iteration;
//									//bool r = unlockContact();
//									//context.write("unlock contact %s\n", r ? "TRUE" : "FALSE");
//									continue;
//								}
//								// grasp
//								//if (error.lin < 0.01) {
//								grasp::to<Data>(cdata->second)->actionType = action::GRASP;
//								context.debug("\n----------------------------------------------------------\n");
//								context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
//								logFile << grasp::makeString("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
//								if (!execute(cdata, inp)) {// not successfully close fingers
//									++failures;
//									continue;
//								}
//								//enableForceReading = true;
//								//::Sleep(1000); // time to retrieve contacts
//								size_t collisions = collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(robot->recvState().config), triggeredGuards);
//								context.write("Attempt to grasp. Collisions=%d\n", collisions);
//								logFile << grasp::makeString("Attempt to grasp. Collisions=%d\n", collisions);
//								if (collisions > 1)
//									grasp::to<TrialData>(trialPtr)->grasped = true;
//								else if (grasp::to<Data>(cdata->second)->stratType != Strategy::ELEMENTARY) { // no contact while grasping then we need to keep iterating
//								//	if (waitKey("YN", "Want to replan? (Y/N)") == 'Y') {
//										contactOccured = false;
//										updateAndResample(cdata);
//										enableForceReading = false;
//										context.write("No contact retrieved while grasping. replanning.\n");
//										++iteration;
//										/*if (iteration > 15)
//											failures = maxFailures;*/
//										continue;
//								//	}
//								}
//
//								
//								//context.write("Iteration %u: execute trajectory (%s). triggered guards %u\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType), triggeredGuards.size());
//								//logFile << grasp::makeString("Iteration %u: execute trajectory (%s). triggered guards %u\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType), triggeredGuards.size());
//								for (auto i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i) {
//									context.write("%s\n", (*i).str().c_str());
//									logFile << grasp::makeString("%s\n", (*i).str().c_str());
//								}
//								
//								context.write("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
//								logFile << grasp::makeString("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
//								results = grasp::makeString("%s\t%.6f\t%.6f\t%u\t%u\n", results.c_str(), error.lin, error.ang, grasp::to<TrialData>(trialPtr)->grasped ? 1 : 0, iteration);
//								break;
//							}
//							// save results
//							context.write("%s", results.c_str());
//							logFile << grasp::makeString("%s", results.c_str());
//							switch (grasp::to<Data>(cdata->second)->stratType){
//							case Strategy::ELEMENTARY:
//								elementaryLog << grasp::makeString("%s", results.c_str());
//								break;
//							case Strategy::MYCROFT:
//								mycroftLog << grasp::makeString("%s", results.c_str());
//								break;
//							case Strategy::IR3NE:
//								ireneLog << grasp::makeString("%s", results.c_str());
//								break;
//							}
//							results = "";
//							context.debug("----------------------------------------------------------\n");
//							context.debug("%s\n", grasp::to<TrialData>(trialPtr)->toString(*robot->getController()).c_str());
//							context.debug("----------------------------------------------------------\n");
//							context.debug("----------------------------------------------------------\n");
//							if (failures < maxFailures) {
//								context.write("Sava trial in %s\n", grasp::to<TrialData>(trialPtr)->path.c_str());
//								logFile << grasp::makeString("Sava trial in %s\n", grasp::to<TrialData>(trialPtr)->path.c_str());
//							}
//							else {
//								context.write("Failed trial\n");
//								logFile << "Failed trial\n";
//							}
//							//grasp::to<TrialData>(trialPtr)->save();
//							context.write("------------------END TRIAL----------------------------------------\n");
//							logFile << "------------------END TRIAL----------------------------------------\n";
//							// reset robot
//							reset();
//							//return;
//							//createBeliefState();
//							//pHeuristic->setBelief(pBelief);
//						}
//					} // end Strategy
//				} // end TRIALS
//			} // end VIEWS
//		} // end for (auto cdata = getData().begin(); cdata != getData().end(); ++cdata)
//		context.write("================================================================================================================================\n");
//		logFile << "================================================================================================================================\n";
////		if (modelPoints.empty() || pBelief->getSamples().empty()) {
////			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
////			return;
////		}
////		// update pose settings
////		poseDataPtr = dataPtr;
////		grasp::to<Data>(dataPtr)->queryTransform = grasp::to<Data>(queryDataPtr)->queryTransform;
////		grasp::to<Data>(dataPtr)->queryFrame = grasp::to<Data>(queryDataPtr)->queryFrame;
////		//grasp::to<Data>(dataPtr)->actionApproach = grasp::to<Data>(queryDataPtr)->actionApproach;
////		//grasp::to<Data>(dataPtr)->actionManip = grasp::to<Data>(queryDataPtr)->actionManip;
////		//grasp::to<Data>(dataPtr)->action = grasp::to<Data>(queryDataPtr)->action;
////		grasp::to<Data>(dataPtr)->queryPoints = grasp::to<Data>(queryDataPtr)->queryPoints;
////		objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->queryPoints));
////		renderData(dataPtr);
////		//grasp::to<Data>(dataPtr)->actionWithdraw.clear();
////		const grasp::RobotState homeState = robot->recvState();
////		//		grasp::to<Data>(dataPtr)->actionWithdraw.push_back(robot->recvState());
////		//		grasp::to<Data>(dataPtr)->actionWithdraw.push_back(robot->recvState());
////
////		grasp::to<Data>(dataPtr)->replanning = false;
////
////		//if (grasp::to<Data>(dataPtr)->actionApproach.empty())
////		//	context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");
////
////		robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
////		robot->setCollisionDetection(true);
////
////	AGAIN:
////		context.debug("RagPlanner:function(X): performing approach trajectory\n");
////		try {
////			if (!singleGrasp) {
////				performApproach(dataPtr);
////			}
////			else {
////				context.write("Single grasp performs\n");
////				performSingleGrasp(dataPtr);
////				//				goto GRASP_QUALITY;
////			}
////		}
////		catch (const Message& msg) {
////			context.write("%s\n", msg.str().c_str());
////		}
////		context.write("perform approach is over.\n");
//////		return;
////		if (grasp::to<Data>(dataPtr)->replanning) {
////			//if (!executedTrajectory.empty()) {
////			//	context.write("Computing withdrawing pose.\n");
////			//	Real distMin(REAL_MAX);
////			//	Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
////			//	// skip the last element of the executed trajectory which is supposed to be the final state
////			//	for (Controller::State::Seq::const_iterator a = executedTrajectory.begin(), b = executedTrajectory.begin() + 1; b != executedTrajectory.end() - 1; ++a, ++b) {
////			//		Real dist(REAL_ZERO);
////			//		for (Configspace::Index i = robot->getStateInfo().getJoints().begin(); i != robot->getStateInfo().getJoints().end(); ++i)				
////			//			dist += Math::sqrt(Math::sqr(robotConfig.cpos[i] - (a->cpos[i] + (a->cpos[i] - b->cpos[i])/2))); //Math::sqrt(Math::sqr(robotConfig.cpos[i] - a->cpos[i]));
////			//		if (dist/* = Math::sqrt(dist)*/ < distMin) {
////			//			distMin = dist;
////			//			waypoint = *a;
////			//		}
////			//	}
////			//	grasp::RobotState w(*robot->getController());
////			//	w.command = w.config = waypoint;
////			//	while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2)
////			//		grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(w);
////			//}
////			//else {
////			//	context.write("Vanilla withdraw (home pose).\n");
////			//	grasp::to<Data>(dataPtr)->actionWithdraw.push_back(homeState);
////			//	grasp::to<Data>(dataPtr)->actionWithdraw.push_back(homeState);
////			//}
////			//performWithdraw(dataPtr);
////			const int key = waitKey("YN", "Do you want to repeat? (Y/N)...");
////			if (key == 'Y') {
////				goto AGAIN;
////			}
////			else if (key == 'N')
////				return;
////		}
////		else
////			performManip(dataPtr);
//		logFile.close();
//		elementaryLog.close();
//		mycroftLog.close();
//		ireneLog.close();
//		return;
//	}
//	case 'L':
//	{
//		auto select = [&](Strategy &strat) {
//			switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
//			case 'N':
//			{
//				strat = Strategy::NONE_STRATEGY;
//				return;
//			}
//			case 'E':
//			{
//				strat = Strategy::ELEMENTARY;
//				return;
//			}
//			case 'M':
//			{
//				strat = Strategy::MYCROFT;
//				return;
//			}
//			case 'I':
//			{
//				strat = Strategy::IR3NE;
//				return;
//			}
//			}
//			//switch (strat) {
//			//case NONE_STRATEGY:
//			//	strat = Strategy::ELEMENTARY;
//			//	break;
//			//case ELEMENTARY:
//			//	strat = Strategy::MYCROFT;
//			//	break;
//			//case MYCROFT:
//			//	strat = Strategy::IR3NE;
//			//	break;
//			//case IR3NE:
//			//	strat = Strategy::NONE_STRATEGY;
//			//	break;
//			//default:
//			//	strat = Strategy::NONE_STRATEGY;
//			//	break;
//			//}
//		};
//
//		// set the simulated object
//		if (!grasp::to<Data>(dataPtr)->simulateObjectPose.empty()) {
//			collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
//			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
//		}
//
//		grasp::RobotState::Map::iterator trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
//		if (trajectory->second.size() < 3)
//			context.write("Error: the selected trajectory have not at least 3 waypoints.");
//
//		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
//		select(grasp::to<Data>(dataPtr)->stratType);
//		grasp::to<Data>(dataPtr)->actionType = action::NONE_ACTION;
//		grasp::to<Data>(dataPtr)->replanning = false;
//		Controller::State::Seq inp = grasp::RobotState::makeCommand(trajectory->second);
//		// open the fingers for the pregrasp
//		for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
//			auto j = robot->getStateHandInfo().getJoints(robot->getStateHandInfo().getChains().begin()).begin() + 1;
//			inp[1].cpos[j] = 0.0;
//		}
//		for (;;) {
//			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
//			if (!execute(dataPtr, inp))
//				return;
//			if (contactOccured) {
//				//grasp::to<Data>(cdata->second)->replanning = false;
//				contactOccured = false;
//				updateAndResample(dataPtr);
//				enableForceReading = false;
//				continue;
//			}
//			// grasp
//			enableForceReading = false;
//			//grasp::to<Data>(dataPtr)->actionType = action::GRASP;
//			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
//			if (!execute(dataPtr, inp))
//				return;
//			
//			// lifting
//			enableForceReading = false;
//			//grasp::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
//			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
//			if (execute(dataPtr, inp))
//				return;
//
//			break;
//		}
//		context.write("Finish Experiment\n");
//		//context.write("%s\n", grasp::to<TrialData>(trialPtr)->toString(*robot->getController()).c_str());
//		return;
//	}
////
////
////		//grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
////		//// query create
////		//context.write("Creating %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
////		//pBelief->createQuery(points->second);
////		//// create hypotheses and return the action frame for this query
////		//actionFrameT = pBelief->maximum().toMat34();
////		//context.write("action frame TEACH <%f %f %f>\n", actionFrameT.p.x, actionFrameT.p.y, actionFrameT.p.z);
////
////		//try {
////		//	graspFrame = getGraspFrame(dataPtr);
////		//}
////		//catch (const Message& msg) {
////		//	context.write("%s\n", msg.str().c_str());
////		//}		
////		return;
////	}
////	case 'U':
////	{
////		if (modelPoints.empty() || pBelief->getSamples().empty()) {
////			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
////			return;
////		}
////		// update pose settings
////		poseDataPtr = dataPtr;
////		grasp::to<Data>(poseDataPtr)->actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame;
////		grasp::to<Data>(poseDataPtr)->queryFrame = grasp::to<Data>(queryDataPtr)->queryFrame;
////		grasp::to<Data>(poseDataPtr)->actionApproach = grasp::to<Data>(queryDataPtr)->actionApproach;
////		grasp::to<Data>(poseDataPtr)->actionManip = grasp::to<Data>(queryDataPtr)->actionManip;
////		grasp::to<Data>(poseDataPtr)->action = grasp::to<Data>(queryDataPtr)->action;
////		grasp::to<Data>(poseDataPtr)->queryPoints = grasp::to<Data>(queryDataPtr)->queryPoints;
////		//grasp::to<Data>(dataPtr)->queryPoints.clear();
////		//grasp::to<Data>(dataPtr)->queryPoints.reserve(grasp::to<Data>(queryDataPtr)->queryPoints.size());
////		//std::for_each(grasp::to<Data>(queryDataPtr)->queryPoints.begin(), grasp::to<Data>(queryDataPtr)->queryPoints.end(), [=] (grasp::Cloud::PointSeq::const_iterator p) {
////		//	grasp::to<Data>(dataPtr)->queryPoints.push_back(*p);
////		//});
////
////		// set home pose
////		//grasp::to<Data>(poseDataPtr)->homeStates.clear();
////		//std::cout << "spam:function: 7 bis\n";
////		//grasp::to<Data>(poseDataPtr)->homeStates.reserve(1);
////		//std::cout << "spam:function: 8 riceiving state\n";
////		//grasp::RobotState homeState = robot->recvState();
////		//std::cout << "spam:function: 8 bis\n";
////		//grasp::to<Data>(poseDataPtr)->homeStates.push_back(homeState);
////		homeStates.clear();
////		homeStates.push_back(robot->recvState());
////
////		bool conclude = false;
////		grasp::to<Data>(poseDataPtr)->replanning = false;
////		grasp::to<Data>(poseDataPtr)->release = false;
////		switch (waitKey("PE", "Press a key to (P)lan a reach-and-grasp trajectory, (E)nable/Disable planning with uncertainty...")) {
////		case 'P':
////		{			
////			if (grasp::to<Data>(poseDataPtr)->actionApproach.empty())
////				context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");
////
////			// approach action is the entire trajectory taught to the robot
////			// NOTE: states are memorised from the last (pre grasp = begin()) to the first (initial pose = end())
////			// Generate a new approach action formed by [pregrasp, corrent conf of the robot]
////			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
////			GenWorkspaceChainState gwcs;
////			gwcs.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
////			grasp::RobotState grasp = *grasp::to<Data>(poseDataPtr)->actionApproach.begin();
////			robot->getController()->chainForwardTransform(grasp.config.cpos, gwcs.wpos);
////			Mat34 poseFrameInv, graspFrame, graspFrameInv;
////			poseFrameInv.setInverse(gwcs.wpos[armChain]);
////			graspFrame.multiply(poseFrameInv, modelFrame/*grasp::to<Data>(poseDataPtr)->queryFrame*/);
////			graspFrameInv.setInverse(graspFrame);
//////				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
////			graspFrameM.multiply(modelFrame, graspFrameInv);
////			renderHand(grasp.config, true);
////
////			grasp::RobotState pregrasp = *(--grasp::to<Data>(poseDataPtr)->actionApproach.end());			
////			robot->getController()->chainForwardTransform(pregrasp.config.cpos, gwcs.wpos);
////			Mat34 preposeFrameInv, pregraspFrame, pregraspFrameInv;
////			preposeFrameInv.setInverse(gwcs.wpos[armChain]);
////			pregraspFrame.multiply(preposeFrameInv, modelFrame/*grasp::to<Data>(poseDataPtr)->queryFrame*/);
////			pregraspFrameInv.setInverse(pregraspFrame);
//////				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
////			pregraspFrameM.multiply(modelFrame, pregraspFrameInv);
////			renderHand(pregrasp.config);
////			showTest = false;
////			renderData(dataPtr);
////
////			trjApproachExtrapolFac = REAL_ZERO;
////			robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
////			robot->setCollisionDetection(true);		
//////			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
////			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
////			iterations = 1;
////
////REPLAN_TEST:
////			try {
////				if (!singleGrasp) {
////					testTransformation(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, grasp::to<Data>(dataPtr)->actionApproach);
////					performApproach(poseDataPtr);
////					context.write("perform approach is over.\n");
////				}
////				else {
////					context.write("Single grasp performs\n");
////					performSingleGrasp(poseDataPtr);
////					goto GRASP_QUALITY;
////				}
////			}
////			catch (const Message& msg) {
////				context.write("%s\n", msg.str().c_str());
////			}
////			
////			std::cout << "check for computing the grasp quality.\n";
////			if (grasp::to<Data>(poseDataPtr)->triggered > 0 && !grasp::to<Data>(poseDataPtr)->replanning) {
////				// todo: compute if grasped
////GRASP_QUALITY:
////				std::cout << "grasp quality: step 1\n";
////				context.write("Check the grasp quality (triggered=%s, replanning=%s)\n", grasp::to<Data>(poseDataPtr)->triggered > 0 ? "true" : "false", grasp::to<Data>(poseDataPtr)->replanning ? "true" : "false"); 
////
////				std::cout << "grasp quality: step 2\n";
////				grasp::RealSeq force;
////				//Real graspQuality = REAL_ZERO;
////				//try {
////				//	graspQuality = robot->analyseGrasp(force);
////				//}
////				//catch (const golem::Message &msg) {
////				//	context.write("%s\n", msg.str().c_str());
////				//}
////				//std::cout << "grasp quality: step 3\n";
////				//replanning = !(graspQuality > 0);
////				//context.write("Grasp %s. quality = %f\n", !replanning ? "succeess (no replanning)" : "failure (replanning)", graspQuality);
////				//if (!replanning) {
////					const int key = waitKey("YN", "Do you want to exit? (Y/N)...");
////					if (key == 'Y') {
////						performManip(poseDataPtr);
////						return;
////						conclude = true;
////					} 
////					else {
////						const int key = waitKey("YN", "Do you want to replan a trajectory? (Y/N)...");
////						if (key == 'Y')
////							grasp::to<Data>(poseDataPtr)->replanning = true;
////						else return;
////					}
////				goto MOVING_BACK;
////			}
////			else {
////MOVING_BACK:
////				trnEnable = false;
////				pHeuristic->enableUnc = false;
////				grasp::to<Data>(poseDataPtr)->actionWithdraw.clear();
////				// if replanning is not required the robot is moved back to the home pose
////				if (!grasp::to<Data>(poseDataPtr)->replanning || withdrawToHomePose) {
//////					if (!/*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.empty()) {
////						// set as first waypont the pregrasp pose or the home pose
////						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(grasp::to<Data>(poseDataPtr)->release && !grasp::to<Data>(poseDataPtr)->actionApproach.empty() ? *(--grasp::to<Data>(poseDataPtr)->actionApproach.end()) : homeStates.front());
////						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(homeStates.front());
//////					}
////					grasp::to<Data>(poseDataPtr)->release = false;
////					//while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2 && !grasp::to<Data>(poseDataPtr)->executedTrajectory.empty())
////					//	grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(grasp::to<Data>(poseDataPtr)->homeStates.front());
////				// otherwise the previous waypoint in the trajectory is withdrawn
////				} else {
////					Real distMin(REAL_MAX);
////					Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
////					for (Controller::State::Seq::const_iterator a = /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.begin(), b = /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.begin() + 1; b != /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.end(); ++a, ++b) {
////						Real dist(REAL_ZERO);
////						for (Configspace::Index i = robot->getStateInfo().getJoints().begin(); i != robot->getStateInfo().getJoints().end(); ++i)				
////							dist += Math::sqrt(Math::sqr(robotConfig.cpos[i] - (a->cpos[i] + (a->cpos[i] - b->cpos[i])/2))); //Math::sqrt(Math::sqr(robotConfig.cpos[i] - a->cpos[i]));
////						if (dist/* = Math::sqrt(dist)*/ < distMin) {
////							distMin = dist;
////							waypoint = *a;
////						}
////					}
////					grasp::RobotState w(*robot->getController());
////					w.command = waypoint; w.config = waypoint;
////					while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2)
////						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(w);
////				}
//////				renderTrialData(poseDataPtr);
////				// move back to home pose
////				performWithdraw(poseDataPtr);
////				goto REPLAN_TEST;
////			}
////			return;
////		}
////		case 'G':
////		{
////			golem::Waypoint w(*robot->getController(), robot->recvState().command.cpos);
////			Mat34 tcpFrameInv, trnFromTcpToObj, tcpFrame = w.wpos[robot->getStateInfo().getChains().begin()];
////			tcpFrameInv.setInverse(tcpFrame);
////			trnFromTcpToObj.multiply(tcpFrameInv, grasp::to<Data>(dataPtr)->queryFrame);
////			context.write("tcpFrame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n", 
////				tcpFrame.R.m11, tcpFrame.R.m12, tcpFrame.R.m13, tcpFrame.p.x, 
////				tcpFrame.R.m21, tcpFrame.R.m22, tcpFrame.R.m23, tcpFrame.p.y, 
////				tcpFrame.R.m31, tcpFrame.R.m32, tcpFrame.R.m33, tcpFrame.p.z);
////			context.write("obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n", 
////				grasp::to<Data>(dataPtr)->queryFrame.R.m11, grasp::to<Data>(dataPtr)->queryFrame.R.m12, grasp::to<Data>(dataPtr)->queryFrame.R.m13, grasp::to<Data>(dataPtr)->queryFrame.p.x, 
////				grasp::to<Data>(dataPtr)->queryFrame.R.m21, grasp::to<Data>(dataPtr)->queryFrame.R.m22, grasp::to<Data>(dataPtr)->queryFrame.R.m23, grasp::to<Data>(dataPtr)->queryFrame.p.y, 
////				grasp::to<Data>(dataPtr)->queryFrame.R.m31, grasp::to<Data>(dataPtr)->queryFrame.R.m32, grasp::to<Data>(dataPtr)->queryFrame.R.m33, grasp::to<Data>(dataPtr)->queryFrame.p.z);
////			context.write("Trn from tcp frame to obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
////				trnFromTcpToObj.R.m11, trnFromTcpToObj.R.m12, trnFromTcpToObj.R.m13, trnFromTcpToObj.p.x,
////				trnFromTcpToObj.R.m21, trnFromTcpToObj.R.m22, trnFromTcpToObj.R.m23, trnFromTcpToObj.p.y,
////				trnFromTcpToObj.R.m31, trnFromTcpToObj.R.m32, trnFromTcpToObj.R.m33, trnFromTcpToObj.p.z);
////			pBelief->setInitObjPose(tcpFrame);
////			Mat34 trn = pBelief->transform(grasp::to<Data>(dataPtr)->queryFrame);
////			context.write("BELIEF Trn from tcp frame to obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
////				trn.R.m11, trn.R.m12, trn.R.m13, trn.p.x,
////				trn.R.m21, trn.R.m22, trn.R.m23, trn.p.y,
////				trn.R.m31, trn.R.m32, trn.R.m33, trn.p.z);
////			trn.multiply(tcpFrame, trnFromTcpToObj);
////			context.write("BELIEF obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
////				trn.R.m11, trn.R.m12, trn.R.m13, trn.p.x,
////				trn.R.m21, trn.R.m22, trn.R.m23, trn.p.y,
////				trn.R.m31, trn.R.m32, trn.R.m33, trn.p.z);
////
////			Mat34 m = robot->getController()->getGlobalPose();
////			context.write("Robot global pose\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n det:%5.7f\n",
////				m.R.m11, m.R.m12, m.R.m13, m.p.x,
////				m.R.m21, m.R.m22, m.R.m23, m.p.y,
////				m.R.m31, m.R.m32, m.R.m33, m.p.z,
////				m.R.determinant());
////
////			Real distMax = golem::REAL_ZERO;
////			for (grasp::Cloud::PointSeq::const_iterator p = modelPoints.begin(); p != modelPoints.end(); ++p) {
////				// CHECK max magnitude of absolute coordinates of points is related to the object coords but NOT its size!
////				//const Real d = p->frame.p.magnitude();
////				const Real d = grasp::Cloud::getPoint(*p).magnitude();
////				if (d > distMax)
////					distMax = d;
////			}
////			context.write("Size of the object %5.7f\n", distMax);
////			// go to to predefined pose
////			//context.write("Goto Pose test\n");
////			//(void)gotoPose();
////			//std::vector<Configspace::Index> triggeredGuards;
////			//if (robot->getTriggeredGuards(triggeredGuards)) {
////			//	context.write("RagPlanner::Test triggered guards on Justin");
////			//	for (std::vector<Configspace::Index>::const_iterator i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i)
////			//		context.write(" %d", *i);
////			//	context.write("\n");
////			//}
////			return;
////		}
////		case 'T':
////		{
////			switch (waitKey("OU", "Press a key to test (O)bservations or (U)pdate..")) {
////			case 'O':
////			{			
////				context.write("test observation\n");
////				Mat34 pose;
////				pose.R.setId();
////				pose.R.rotX(-REAL_HALF*REAL_PI);
////				pose.p = Vec3(-0.1, 0.40, 0.1);
////				for (Real i = 0; i < 60; ++i) {
////					pose.p.x += 0.001*i;
////					pHeuristic->testObservations(pose, true);
////					renderPose(pose);
////					::Sleep(1000);
////				}
////				return;
////			}
////			case 'U':
////			{
////				context.write("test update\n");
////				grasp::RBPose::Sample::Seq samples;
////				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.80, -0.36, 0.01), Quat())));
////				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.65, -0.36, 0.01), Quat())));
////				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.50, -0.36, 0.01), Quat())));
////				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.30, -0.36, 0.01), Quat())));
////				Mat34 pose;
////				pose.R.setId();
////				pose.R.rotX(REAL_HALF*REAL_PI);
////				pose.p = Vec3(0.4, -0.45, 0.1);
////				for (size_t i = 0; i < 20; ++i) {
////					pose.p.x += 0.01*i;
////					context.write("Iteration n.%d pose <%f %f %f>\n", i + 1, pose.p.x, pose.p.y, pose.p.z);
////					//for (grasp::RBPose::Sample::Seq::iterator j = samples.begin(); j != samples.end(); ++j)
////					//	j->weight = pHeuristic->evaluate(grasp::RBCoord(pose), *j, -0.5, modelFrame);
////					context.write("\n");
////					renderUpdate(pose, samples);
////					::Sleep(1000);
////				}
////				return;
////			}
////			}
////		}
////		case 'E':
////			uncEnable = !uncEnable;
////			context.write("Planning with uncertainty %s\n", uncEnable ? "ON" : "OFF");
////			return;
////		}
////		break;
////	}
////	//case 'T':
////	//{
////	//	switch (waitKey("OU", "Press a key to test (O)bservations or (U)pdate..")) {
////	//	case 'O':
////	//	{			
////	//		context.write("test observation\n");
////	//		Mat34 pose;
////	//		pose.R.setId();
////	//		pose.R.rotX(-REAL_HALF*REAL_PI);
////	//		pose.p = Vec3(-0.4, 0.70, 0.15);
////	//		for (Real i = 0; i < 60; ++i) {
////	//			pose.p.x += 0.001*i;
////	//			pHeuristic->testObservations(pose, true);
////	//			renderPose(pose);
////	//			::Sleep(1000);
////	//		}
////	//		return;
////	//	}
////	//	case 'U':
////	//	{
////	//		context.write("test update\n");
////	//		grasp::RBPose::Sample::Seq samples;
////	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.80, -0.36, 0.01), Quat())));
////	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.65, -0.36, 0.01), Quat())));
////	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.50, -0.36, 0.01), Quat())));
////	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.30, -0.36, 0.01), Quat())));
////	//		Mat34 pose;
////	//		pose.R.setId();
////	//		pose.R.rotX(REAL_HALF*REAL_PI);
////	//		pose.p = Vec3(0.4, -0.45, 0.1);
////	//		for (size_t i = 0; i < 20; ++i) {
////	//			pose.p.x += 0.01*i;
////	//			context.write("Iteration n.%d pose <%f %f %f>\n", i + 1, pose.p.x, pose.p.y, pose.p.z);
////	//			//for (grasp::RBPose::Sample::Seq::iterator j = samples.begin(); j != samples.end(); ++j)
////	//			//	j->weight = pHeuristic->evaluate(grasp::RBCoord(pose), *j, -0.5, modelFrame);
////	//			context.write("\n");
////	//			renderUpdate(pose, samples);
////	//			::Sleep(1000);
////	//		}
////	//		return;
////	//	}
////	//	}
////	//}
////	case ',':
////	{
////		handBounds.clear();
////		return;
////	}
////	case '.':
////	{
////		renderHand(grasp::to<Data>(dataPtr)->actionApproach[grasp::to<Data>(dataPtr)->actionApproach.size()-1].config);
////		return;
//////			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//////			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
//////			GenWorkspaceChainState gwcs;
//////			robot->getController()->chainForwardTransform(robot->recvState().config.cpos, gwcs.wpos);
////////			gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
//////			renderPose(gwcs.wpos[armChain]);
//////			grasp::RBCoord b(gwcs.wpos[armChain]);
//////			context.write("render pose triggered state <%f %f %f> <%f %f %f %f>\n", b.p.x, b.p.y, b.p.z, b.q.w, b.q.x, b.q.y, b.q.z);
//////			{		
//////				golem::CriticalSectionWrapper csw(csDataRenderer);
//////				const Bounds::Seq bounds = manipulator->getBounds(manipulator->getPose(robot->recvState().config));
//////				handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
//////			}	
//////			return;
////	}
//	case 'I':
//	{		
//		auto select = [&](Strategy &strat) {
//			switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
//			case 'N':
//			{
//				strat = Strategy::NONE_STRATEGY;
//				return;
//			}
//			case 'E':
//			{
//				strat = Strategy::ELEMENTARY;
//				return;
//			}
//			case 'M':
//			{
//				strat = Strategy::MYCROFT;
//				return;
//			}
//			case 'I':
//			{
//				strat = Strategy::IR3NE;
//				return;
//			}
//			}
//			//switch (strat) {
//			//case NONE_STRATEGY:
//			//	strat = Strategy::ELEMENTARY;
//			//	break;
//			//case ELEMENTARY:
//			//	strat = Strategy::MYCROFT;
//			//	break;
//			//case MYCROFT:
//			//	strat = Strategy::IR3NE;
//			//	break;
//			//case IR3NE:
//			//	strat = Strategy::NONE_STRATEGY;
//			//	break;
//			//default:
//			//	strat = Strategy::NONE_STRATEGY;
//			//	break;
//			//}
//		};
//
//		grasp::RobotState::Map::iterator trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
//		if (trajectory->second.size() < 3)
//			context.write("Error: the selected trajectory have not at least 3 waypoints.\n");
//
//		Controller::State::Seq inp = grasp::RobotState::makeCommand(trajectory->second);
//		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
//		select(grasp::to<Data>(dataPtr)->stratType);
//		execute(dataPtr, inp);
//		return;
//	}
//	case 'N':
//	{
//		std::stringstream prefix;
//		prefix << std::fixed << std::setprecision(3) << "./data/boris/coke/ftsensors" << "-";
//		contact = false;
//		record = false;
//
//		auto strTorquesDesc = [=](std::ostream& ostr) {
//			ostr << "timestamp" << grasp::to<Data>(data)->sepField << "contact" << grasp::to<Data>(data)->sepField << "thumb_0" << grasp::to<Data>(data)->sepField << "thumb_1" << grasp::to<Data>(data)->sepField << "thumb_2" << grasp::to<Data>(data)->sepField << "thumb_3" << grasp::to<Data>(data)->sepField <<
//				"index_0" << grasp::to<Data>(data)->sepField << "index_1" << grasp::to<Data>(data)->sepField << "index_2" << grasp::to<Data>(data)->sepField << "index_3" << grasp::to<Data>(data)->sepField <<
//				"middle_0" << grasp::to<Data>(data)->sepField << "middle_1" << grasp::to<Data>(data)->sepField << "middle_2" << grasp::to<Data>(data)->sepField << "middle_3" << grasp::to<Data>(data)->sepField <<
//				"ring_0" << grasp::to<Data>(data)->sepField << "ring_1" << grasp::to<Data>(data)->sepField << "ring_2" << grasp::to<Data>(data)->sepField << "ring_3" << grasp::to<Data>(data)->sepField <<
//				"pinky_0" << grasp::to<Data>(data)->sepField << "pinky_1" << grasp::to<Data>(data)->sepField << "pinky_2" << grasp::to<Data>(data)->sepField << "pinky_3" << grasp::to<Data>(data)->sepField;
//		};
//
//		// assumption: the robot is in the final pose with all finger open (pre-grasp)
//		Controller::State s = robot->recvState().command;// trajectory[0];
//
//		// current configuration (fingers opened)
//		Controller::State pregrasp = robot->getController()->createState();
//		robot->getHandCtrl()->getController()->setToDefault(pregrasp);
//		pregrasp.cpos = s.cpos;
//
//		// compute starting pose
//		// arm chain and joints pointers
//		const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//		const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
//		const golem::Controller::State::Info handInfo = robot->getStateHandInfo();
//		// Compute a sequence of targets corresponding to the transformed arm end-effector
//		GenWorkspaceChainState gwcs;
//		robot->getController()->chainForwardTransform(pregrasp.cpos, gwcs.wpos);
//		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
//		gwcs.t = pregrasp.t;
//		//	gwcs.wpos[armChain].p.x += 0.045;
//		gwcs.wpos[armChain].p.y -= 0.07;
//
//		// initial pose
//		Controller::State cstart = pregrasp;
//		{
//			// lock controller
//			golem::CriticalSectionWrapper csw(robot->getControllerCS());
//			// Find initial target position
//			if (!robot->getPlanner()->findTarget(pregrasp, gwcs, cstart))
//				context.write("findTarget(): Unable to find initial target configuration");
//		}
//
//		// update arm configurations and compute average error
//		grasp::RBDist err;
//		WorkspaceChainCoord wcc;
//		robot->getController()->chainForwardTransform(cstart.cpos, wcc);
//		wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
//		err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(gwcs.wpos[armChain])));
//		context.write("findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
//
//		auto markContact = [&]() { if (waitKey(10) == ' ') contact = !contact; };
//
//		auto selectFinger = [&]() -> size_t {
//			context.write("[0] thumb\n[1] index\n[2] middle\n[3] ring\n[4] pinky\n");
//			I32 index = 0;
//			readNumber("Enter index: ", index);
//			return index;
//		};
//
//		auto closeFinger = [&](Controller::State &state, const golem::Chainspace::Index &index) {
////			context.write("closing finger");
//			for (auto i = handInfo.getJoints(index).begin() + 2; i != handInfo.getJoints(index).end(); ++i)
//				state.cpos[i] = 1.0;
//		};
//		auto openFinger = [&](Controller::State &state, const golem::Chainspace::Index &index) {
////			context.write("opening finger");
//			for (auto i = handInfo.getJoints(index).begin() + 2; i != handInfo.getJoints(index).end(); ++i)
//				state.cpos[i] = 0.01;
//		};
//
//		const SecTmReal dur = SecTmReal(15.0);
//		auto move2start = [&](bool &breakpoint) {
//			auto breaking = [&]() { if (waitKey(10) == 27) breakpoint = true; };
//			Controller::State::Seq seq, out;
////			robot->findTrajectory(robot->recvState().command, &cstart, nullptr, 0, seq);
//			seq.push_back(pregrasp);
//			seq.push_back(cstart);
//			profile(dur, seq, out, true);
//			robot->sendTrajectory(out);
//			// repeat every send waypoint until trajectory end
//			context.write("Press (ESC) - recording %s contact %s\r", record ? "Y" : "N", contact ? "Y" : "N");
//			for (U32 i = 0; robot->waitForBegin(); ++i) {
//				if (universe.interrupted())
//					throw grasp::Interrupted();
//				if (robot->waitForEnd(0))
//					break;
//				breaking();
//			}
//			context.write("                                                                               \r");
//		};
//		auto poke = [&]() {
//			Controller::State::Seq seq, out;
////			robot->findTrajectory(robot->recvState().command, &pregrasp, nullptr, 0, seq);
//			seq.push_back(cstart);
//			seq.push_back(pregrasp);
//			profile(SecTmReal(5.0), seq, out, true);
//			robot->sendTrajectory(out);
//			// repeat every send waypoint until trajectory end
//			for (U32 i = 0; robot->waitForBegin(); ++i) {
//				if (universe.interrupted())
//					throw grasp::Interrupted();
//				if (robot->waitForEnd(0))
//					break;
//				markContact();
//				if (i % 10 == 0) {
//					context.write("State #%d - recording %s contact %s\r", i, record ? "Y" : "N", contact ? "Y" : "N");
//				}
//			}
//			context.write(" \r");
//		};
//
//		auto stop = [&](I32 &counter) -> bool {
//			const int key = waitKey("QC", "Press to (Q)uit or to (C)ontinue");
//			if (key == 'Q')
//				return true;
//			counter = 0;
//			return false;
//		};
//		auto quit = [&]() -> bool {
//			const int key = waitKey("QC", "Press to (Q)uit the program or to (C)ontinue with another finger");
//			if (key == 'Q')
//				return true;
//			return false;
//		};
//
//		for (;;) {
//			I32 index = selectFinger();
//
//			I32 idx = 0;
//			for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
//				//			context.write("idx=%d index=%d\n", idx, index);
//				if (idx == index) {
//					openFinger(cstart, i);
//					openFinger(pregrasp, i);
//				}
//				else {
//					closeFinger(cstart, i);
//					closeFinger(pregrasp, i);
//				}
//				++idx;
//			}
//
////			Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cstart), manipulator->getPose(cstart).toMat34());
////			renderHand(cstart, bounds, true);
//			// writing data into a text file, open file
//			std::string name(prefix.str().c_str());
//			readString("Enter name file:", name);
//			const std::string dataIndexPathRaw = grasp::makeString("%s_raw.txt", name.c_str());
//			const std::string dataIndexPathFiltered = grasp::makeString("%s_filtered.txt", name.c_str());
//
//			//	if (enableSimContact) {
//			// raw data from drl hand FT
//			dataFTRaw.open(dataIndexPathRaw);
//			// writing data into a text file, prepare headers
//			dataFTRaw << "#index" << grasp::to<Data>(data)->sepField;
//			strTorquesDesc(dataFTRaw);
//			dataFTRaw << std::endl;
//			// filtered data from DLR hand FT
//			dataFTFiltered.open(dataIndexPathFiltered);
//			// writing data into a text file, prepare headers
//			dataFTFiltered << "#index" << grasp::to<Data>(data)->sepField;
//			strTorquesDesc(dataFTFiltered);
//			dataFTFiltered << std::endl;
//
//			bool breakPoint = false;
//			for (;;) {
//				move2start(breakPoint);
//				if (breakPoint)
//					break;
//				record = true;
//				poke();
//				record = false;
//				contact = false;
//			}
//			dataFTRaw.close();
//			dataFTFiltered.close();
//			if (quit())
//				break;
//		}
//
//		return;
//	}
//	case 'O':
//	{
//		brecord = !brecord;
//		context.write("Screencapture recording %s\n", brecord ? "ON" : "OFF");
//		universe.postScreenCaptureFrames(brecord  ? - 1 : 0);
//
//		//auto breakPoint = [&]() -> bool { if (waitKey(10) == 27) { record = !record;  return true; } return false; };
//		//torquesNoContact.open("./data/nocontact.log");
//		//torquesInContact.open("./data/contact.log");
//		//for (;;) {
//		//	switch (waitKey("CNB", "Press a key to select (N)oContact/(C)ontact/(B)reak...")) {
//		//	case 'N':
//		//		inContact = false;
//		//		break;
//		//	case 'C':
//		//		inContact = true;
//		//		break;
//		//	case 'B':
//		//		return;
//		//	}
//		//	for (; !breakPoint();){
//
//		//	}
//		//	
//		//}
//
////		const grasp::Cloud::PointSeq points = *objectPointCloudPtr.get();//getPoints(dataPtr)->second;
//		//Collision collision(context, *manipulator);
//		//grasp::Manipulator::Pose pose = manipulator->getPose(robot->recvState().config);
//		//Collision::Waypoint waypoint;
//		//waypoint.points = 1000000;
//		//std::vector<Configspace::Index> joints;
//		//context.debug("Evaluate on robot's bounds (point size=%d)\n", points.size());
//		//(void)collision.evaluate(robot, waypoint, points, rand, pose, joints, true);
//		//context.debug("Evaluate on collision bounds\n");
//		//(void)collision.evaluate(waypoint, points, rand, pose, true);
//		return;
//	}
//	//case '0':
//	//{
//	//	forcereadersilent = !forcereadersilent;
//	//	return;
//	//	//context.write("%s\n", trialData->toString(*robot->getController()).c_str());
//	//	Controller::State state = robot->getController()->createState();
//	//	robot->getController()->lookupState(SEC_TM_REAL_MAX, state);
//	//	printState(state, robot->getController()->getStateInfo().getJoints().begin(), robot->getController()->getStateInfo().getJoints().end(), "Current robot pose", true);
//	//	return;
//	//}
//	//case '!':
//	//{
//	//	WaypointGenerator::Seq generators = robot->getGlobalPathGenerators();
//	//	for (WaypointGenerator::Seq::const_iterator i = generators.begin(); i != generators.end(); ++i) {
//	//		Controller::State state = robot->getController()->createState();
//	//		state.cpos = i->mean;
//	//		Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(state), manipulator->getPose(state).toMat34());
//	//		renderWaypoints(bounds, false);
//	//	}
//
//	//	//singleGrasp = !singleGrasp;
//	//	//context.write("Single grasp attempt %s\n", singleGrasp ? "ON" : "OFF");
//	//	return;
//	//}
//	case '-':
//	{
//		if ((dynamic_cast<RagGraphPlanner&>(*robot->getPlanner())).getLocalPath().empty())
//			return;
//		Waypoint::Seq localPath = (dynamic_cast<RagGraphPlanner&>(*robot->getPlanner())).getLocalPath();
//		context.write("Graph size=%d)\n", localPath.size());
//
//		for (size_t i = 0; i < 50; ++i) {
//			Waypoint w = localPath[size_t(rand.next()) % localPath.size()];
//			Controller::State state = robot->getController()->createState();
//			state.cpos = w.cpos;
//			Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(state), manipulator->getPose(state).toMat34());
//			renderHand(state, bounds, (i == 0) ? true : false);
//		}
//		return;
//
//	}
//	} // switch
//	ShapePlanner::function(dataPtr, key);
//}

//------------------------------------------------------------------------------

#ifdef _SPAM_RAG_MAIN_
int main(int argc, char *argv[]) {
	return spam::RagPlanner::Desc().main(argc, argv);
}
#endif // _SPAM_RAG_MAIN_