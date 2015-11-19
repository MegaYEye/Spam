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
//#include <Golem/Phys/PhysScene.h>
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
	catch (const Message&) {
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
	enableSimContact = false;
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
	prefix << std::fixed << std::setprecision(3) << "./data/boris/experiments/ftsensors/ftsensors" << "-" << context.getTimer().elapsed();
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
		size_t jointInCollision = enableSimContact && !objectPointCloudPtr->empty() ? collisionPtr->simulate(desc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(lookupState()), /*links*/force, true) : 0;
		RealSeq filteredForces; filteredForces.assign(dimensions(), REAL_ZERO);
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
			filteredForces = getFilterForce();
			if (record) {
				//breakPoint();
				dataFTRaw << indexjj << "\t";
				strTorques(dataFTRaw, state, force, links/*contact*/);
				dataFTRaw << std::endl;
				dataFTFiltered << indexjj << "\t";
				strTorques(dataFTFiltered, state, filteredForces, links/*contact*/);
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
			guard.force = filteredForces[k]; //force[k];
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
//		size_t jointInCollision = enableSimContact && !objectPointCloudPtr->empty() ? collisionPtr->simulate(desc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(robot->recvState().config), force) : 0;
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

	auto executeCmd = [&](const std::string command) {
		MenuCmdMap::const_iterator cmd = menuCmdMap.find(command);
		if (cmd == menuCmdMap.end())
			throw Cancel(makeString("Error: impossible to execute command %s.", command.c_str()).c_str());
		cmd->second();
	};

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
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		//Controller::State::Seq inp = trajectory->getWaypoints();
		grasp::Waypoint::Seq inp = trajectory->getWaypoints();
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
			//	strat = Strategy::ELEMENTARY;
			//	break;
			//case ELEMENTARY:
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
		auto reset = [&]() {
			enableForceReading = false;
			pHeuristic->enableUnc = false;
			pHeuristic->setPointCloudCollision(false);
			Controller::State::Seq seq, out;
			findTrajectory(lookupState(), &home, nullptr, 0, seq);
			profile(this->trjDuration, seq, out, true);
			sendTrajectory(out);
			controller->waitForBegin();
			controller->waitForEnd();
			//// repeat every send waypoint until trajectory end
			//for (U32 i = 0; controller->waitForBegin(); ++i) {
			//	if (universe.interrupted())
			//		throw Exit();
			//	if (controller->waitForEnd(0))
			//		break;
			//}
		};
		auto resetDataPtr = [&](Data::Map::iterator& dataPtr) {
			to<Data>(dataPtr)->queryPoints.clear();
			to<Data>(dataPtr)->queryTransform.setId();
			to<Data>(dataPtr)->queryFrame.setId();
			to<Data>(dataPtr)->poses.clear();
			to<Data>(dataPtr)->hypotheses.clear();
			to<Data>(dataPtr)->simulateObjectPose.clear();
			};
		std::ofstream logFile("F:/data/boris/experiments/output.log");
		std::ofstream elementaryLog("F:/data/boris/experiments/elementary.log");
		std::ofstream mycroftLog("F:/data/boris/experiments/mycroft.log");
		std::ofstream ireneLog("F:/data/boris/experiments/irene.log");

		grasp::to<Data>(dataCurrentPtr)->stratType = Strategy::NONE_STRATEGY;
		grasp::to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
		grasp::to<Data>(dataCurrentPtr)->replanning = false;
		grasp::data::Item::Map::iterator modelContactPtr, modelPtr, queryGraspPtr, queryGraspTrjPtr;
		data::Item::Ptr queryGraspTrj;
		U32 modelViews = modelScanPoseSeq.size(), queryViews = queryScanPoseSeq.size(), trials = 5;
		const U32 maxFailures = 2, maxIterations = 5;
		// collects results as string
		std::string results;

		// command keys
		std::string createModelCmd("PM");
		std::string createQueryCmd("PQ");
		std::string itemTransCmd("IT");
		std::string itemConvCmd("IC");
		std::string dataSaveCmd("DS");
		std::string itemRemoveCmd("IR");

		context.write("================================================================================================================================\n");
		logFile << "================================================================================================================================\n";
		//--------------------------------------------------------------------//
		// CREATE A MODEL POINT CLOUD
		//--------------------------------------------------------------------//
		// object name
		std::string objectName("");
		readString("Enter object name: ", objectName);
		// create a model for the object
		context.write("Create a model...\n");
		for (; to<Data>(dataCurrentPtr)->modelPoints.empty();)
			executeCmd(createModelCmd); // move the robot to the home pose after scanning
		//reset();
		context.write("done.\n");

		//--------------------------------------------------------------------//
		// CREATE A PREDICTIVE MODEL FOR THE GRASP
		//--------------------------------------------------------------------//
		// create a grasp
		context.write("Create a predictive contact model for the grasp...\n");
		std::vector<std::string> modelGraspItemSeq; modelGraspItemSeq.clear();
		//modelGraspItemSeq.push_back("grasp-handle");
		//modelGraspItemSeq.push_back("grasp-pinch");
		//modelGraspItemSeq.push_back("grasp-pinchsupp");
		//modelGraspItemSeq.push_back("grasp-power");
		//modelGraspItemSeq.push_back("grasp-powertube");
		//modelGraspItemSeq.push_back("grasp-rim");
		//for (;;) {
			dataItemLabel = modelGraspItem;
			executeCmd(itemTransCmd);
			modelGraspItem = dataItemLabel;
			modelGraspItemSeq.push_back(modelGraspItem);
		//	if (option("YN", "Add another grasp type? (Y/N)") == 'Y')
		//		continue;
		//	break;
			context.write("done.\n");
		//}

		//--------------------------------------------------------------------//
		// TRIALS
		//--------------------------------------------------------------------//
		itemPerformedTrj.clear();
		for (U32 trial = 0; trial < trials; ++trial) {
			for (;;) {
				//--------------------------------------------------------------------//
				// TRIAL HEADER
				//--------------------------------------------------------------------//
				// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
				select(grasp::to<Data>(dataCurrentPtr)->stratType);
				// stop internal loop if all the strategies have been executed. increase trial
				if (to<Data>(dataCurrentPtr)->stratType == Strategy::NONE_STRATEGY)
					break;

				results = grasp::makeString("%u\t%u\t%u", modelViews, queryViews, trial + 1);
				context.write("execute %s trajectory (%s)\n", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), actionToString(grasp::to<Data>(dataCurrentPtr)->actionType));
				logFile << grasp::makeString("Strategy: %s\n", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str());
				// Setup trial data
				TrialData::Ptr tdata = createTrialData();
				tdata->grasped = false;
				tdata->silent = true;
				tdata->setup(context, rand);
				tdata->name = grasp::makeString("trial_%s_v%u_0%u", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), queryViews, trial+1); //"trial_v1_01"; 
				// object name
				tdata->object = objectName;
				tdata->dirPath = grasp::makeString("%s%s/%s/", this->trial->path.c_str(), tdata->object.c_str(), tdata->name.c_str());
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
				resetDataPtr(dataCurrentPtr);
				for (; to<Data>(dataCurrentPtr)->queryPoints.empty();)
					executeCmd(createQueryCmd);
				// set the simulated object
				if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
					collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
					objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
				}
				reset(); // move the robot to the home pose after scanning
				context.write("done.\n");

				context.write("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(dataCurrentPtr)->queryPoints.size(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
				logFile << grasp::makeString("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(dataCurrentPtr)->queryPoints.size(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);

				//--------------------------------------------------------------------//
				// CREATE A PREDICTIVE QUERY FOR THE GRASP
				//--------------------------------------------------------------------//
				// create a grasp
				// retrieve model contact
				if (modelGraspItemSeq.empty())
					throw Message(Message::LEVEL_ERROR, "RagPlanner::demo(): No model grasp have been created.");

				// generate features
				data::Transform* transform = is<data::Transform>(queryGraspHandler);
				if (!transform)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", queryGraspHandler->getID().c_str());


				data::Item::List list;
				for (auto i = modelGraspItemSeq.begin(); i != modelGraspItemSeq.end(); ++i) {
					data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(*i);
					if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
						throw Message(Message::LEVEL_ERROR, "RagPlanner::estimatePose(): Does not find %s.", (*i).c_str());
					list.insert(list.end(), ptr);
				}
				// retrieve model point cloud
				modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
				if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
					throw Message(Message::LEVEL_ERROR, "RagPlanner::estimatePose(): Does not find %s.", modelItem.c_str());
				list.insert(list.end(), modelPtr); // grasp on model
				data::Item::Ptr queryGraspItemPtr = transform->transform(list);

				// insert processed object, remove old one
				//grasp::data::Item::Map::iterator queryGraspPtr;
				RenderBlock renderBlock(*this);
				{
					golem::CriticalSectionWrapper cswData(getCS());
					to<Data>(dataCurrentPtr)->itemMap.erase(queryGraspItem);
					queryGraspPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(queryGraspItem, queryGraspItemPtr));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryGraspPtr, to<Data>(dataCurrentPtr)->getView());
				}
				context.write("Transform: handler %s, inputs %s, %s...\n", queryGraspHandler->getID().c_str(), queryGraspPtr->first.c_str(), modelItem.c_str());
//				to<Data>(dataCurrentPtr)->createRender();
//				context.write("done.\n");

				//--------------------------------------------------------------------//
				// CREATE A GRASP TRAJECTORY
				//--------------------------------------------------------------------//
				context.write("Convert [%s]: handler %s, input %s...\n", queryItemTrj.c_str(), queryHandlerTrj->getID().c_str(), queryGraspPtr->first.c_str());
				data::Convert* convert = is<data::Convert>(queryGraspPtr);
				if (!convert)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Convert interface", queryHandlerTrj->getID().c_str());

				// convert
				queryGraspTrj = convert->convert(*queryHandlerTrj);
				// insert processed object, remove old one
				//grasp::data::Item::Map::iterator queryGraspTrjPtr;
				RenderBlock renderBlock2(*this);
				{
					golem::CriticalSectionWrapper cswData(getCS());
					to<Data>(dataCurrentPtr)->itemMap.erase(queryItemTrj);
					queryGraspTrjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(queryItemTrj, queryGraspTrj));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryGraspTrjPtr, to<Data>(dataCurrentPtr)->getView());
				}
//				to<Data>(dataCurrentPtr)->createRender();
				context.write("done.\n");

				context.write("Transform: handler %s, input %s...\n", queryHandlerTrj->getID().c_str(), queryGraspTrjPtr->first.c_str());
				data::Trajectory* trajectory = is<data::Trajectory>(queryGraspTrjPtr);
				if (!trajectory)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Trajectory interface", queryHandlerTrj->getID().c_str());
				grasp::Waypoint::Seq inp = trajectory->getWaypoints();
				if (inp.size() < 3)
					throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

				// set render to show only mean hypothesis and ground truth
				resetDataPointers();
				showHypothesesPointClouds = true;
				showMeanHypothesisPointClouds = true;
				showGroundTruth = false;
				to<Data>(dataCurrentPtr)->createRender();

				//--------------------------------------------------------------------//
				// EXECUTE TRIAL
				//--------------------------------------------------------------------//
				U32 iteration = 1, failures = 0;
				const RBCoord obj(simQueryFrame * to<Data>(dataCurrentPtr)->modelFrame);
				for (; failures < maxFailures && iteration < maxIterations;) {
					grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_M2Q;
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
				std::string dataPath = to<TrialData>(trialPtr)->dirPath + "data.xml";
				//readPath("Enter data path to save: ", dataPath, dataExt.c_str());
				//if (getExt(dataPath).empty())
				//	dataPath += dataExt;
				to<Manager::Data>(dataCurrentPtr)->save(dataPath);
				//executeCmd(dataSaveCmd);
				context.write("------------------END TRIAL----------------------------------------\n");
				logFile << "------------------END TRIAL----------------------------------------\n";
				// reset robot
				reset();
			} // end strategy
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
		} // end trial
		context.write("================================================================================================================================\n");
		logFile << "================================================================================================================================\n";
		logFile.close();
		elementaryLog.close();
		mycroftLog.close();
		ireneLog.close();

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
		grasp::Waypoint::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
		Controller::State cend = inp[1].state;
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

	menuCmdMap.insert(std::make_pair("N", [=]() {
		context.write("Test collision detection\n");
		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		data::Trajectory* trajectory = is<data::Trajectory>(item->second.get());
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		grasp::Waypoint::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");
		pHeuristic->enableUnc = false;
		isGrasping = false;

		Controller::State cend = lookupState();
		// transform w.r.t. query frame
		findTarget(grasp::to<Data>(dataCurrentPtr)->queryTransform, grasp::to<Data>(dataCurrentPtr)->modelFrame, inp[1].state, cend);

		Controller::State::Seq approach;
		(void)findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		context.write("path dist delta %f\n", pHeuristic->getDesc().collisionDesc.pathDistDelta);
		Real delta = 0.0025;//pHeuristic->getDesc().collisionDesc.pathDistDelta;
		for (auto j = approach.begin(); j != approach.end(); ++j) {
//			const bool res = (*pBelief->getHypotheses().begin())->check(pHeuristic->ftDrivenDesc.checkDesc, rand, manipulator->getConfig(*i), true);
			if (j != approach.end() - 1) {
				golem::Waypoint w0, w1;
				w1.cpos = j->cpos; w0.cpos = (j + 1)->cpos;
				const Real dist = pHeuristic->getDist(w0, w1);
				const U32 size = (U32)Math::round(dist / delta) + 1;
				Real p[2];
				golem::Waypoint w;

				// test for collisions in the range (w0, w1) - excluding w0 and w1
				for (U32 i = 1; i < size;) {
					p[0] = Real(i) / Real(size);
					p[1] = REAL_ONE - p[0];

					// lineary interpolate coordinates
					for (Configspace::Index l = armInfo.getJoints().begin(); l < armInfo.getJoints().end(); ++l)
						w.cpos[l] = p[0] * w0.cpos[l] + p[1] * w1.cpos[l];
					for (Configspace::Index l = handInfo.getJoints().begin(); l < handInfo.getJoints().end(); ++l)
						w.cpos[l] = p[0] * w0.cpos[l] + p[1] * w1.cpos[l];

					// skip reference pose computation
					w.setup(*controller, false, true);
					const grasp::Manipulator::Config config(w.cpos);
					Bounds::Seq bounds = manipulator->getBounds(config.config, config.frame.toMat34());
					renderHand(*j, bounds, true);
					const bool res = (*pBelief->getHypotheses().begin())->check(pHeuristic->ftDrivenDesc.checkDesc, rand, config.config, true);
					if (option("PN", "(N)ext, (P)revious") == 'N') {
						if (i < size) ++i;
						else {
							if (option("YN", "Exit? (Y/N)") == 'Y')
								i = size;
						}
					}
					else { // previous
						if (i > 0) --i;
						else {
							if (option("YN", "Exit? (Y/N)") == 'Y')
								i = size;
						}

					}
				}
			}
		}
		//		cloudImport.generate(rand, vertices, triangles, [&](const Vec3& p, const Vec3& n) {
//			points.push_back(Cloud::Import::make<Point>(p, n, RGBA::WHITE));
//		});
//		Cloud::setSensorFrame(Mat34::identity(), points);
//
//		BoundingConvexMesh::Desc *pBoundingConvexMeshDesc;
//		pBoundingConvexMeshDesc->triangles = triangles;
//		pBoundingConvexMeshDesc->vertices = vertices;
//		pBoundingConvexMeshDesc->pose = Mat34::identity();
////		NxShapeDesc *pNxShapeDesc = getScene().createNxShapeDesc(pBoundingConvexMeshDesc);
//		// object body
//		BoundingCylinder::Desc pDesc = BoundingCylinder::Desc();
//		pDesc.radius = 0.1;
//		pDesc.length = 0.3;
//		Actor::Desc cylinderDesc;
//		cylinderDesc.setToDefault();
//		cylinderDesc.pose.p = Vec3(.49, -.37, .3);
//		// create object
//		Actor *pCylinder = dynamic_cast<Actor*>(getScene().createObject(cylinderDesc));
//		if (pCylinder == NULL)
//			throw Message(Message::LEVEL_CRIT, "Unable to create object");
//		to<Data>(dataCurrentPtr)->createRender();
		context.write("done.\n");
		return;
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

//------------------------------------------------------------------------------

void RagPlanner::render() const {
	PosePlanner::render();
//	handRenderer.reset();
	{
		golem::CriticalSectionWrapper csw(getCS());
		debugRenderer.render();
	}
}

void RagPlanner::renderHand(const golem::Controller::State &state, const Bounds::Seq &bounds, bool clear) {
	return;
	{
		golem::CriticalSectionWrapper csw(getCS());
		debugRenderer.reset();
		if (clear)
			handBounds.clear();
		if (!bounds.empty())
			handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
		debugRenderer.setColour(RGBA::BLACK);
		debugRenderer.setLineWidth(Real(2.0));
		debugRenderer.addWire(handBounds.begin(), handBounds.end());
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

void RagPlanner::findTarget(const golem::Mat34& queryTrn, const golem::Mat34& modelFrame, const golem::Controller::State& target, golem::Controller::State& cend, const bool lifting) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(target.cpos, gwcs.wpos);
	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1

	Mat34 poseFrameInv, graspFrame, graspFrameInv;
	poseFrameInv.setInverse(gwcs.wpos[armChain]);
	graspFrame.multiply(poseFrameInv, modelFrame);
	graspFrameInv.setInverse(graspFrame);
	gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
	gwcs.wpos[armChain].multiply(queryTrn, gwcs.wpos[armChain]);
	gwcs.t = target.t;
	gwcs.wpos[armChain].p.z -= 0.007;
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

	//golem::Controller::State::Seq initTrajectory;
	//findTrajectory(lookupState(), &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	//golem::Controller::State::Seq completeTrajectory = initTrajectory;
	//completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

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
	trajectoryIf->setWaypoints(/*completeTrajectory*/grasp::Waypoint::make(trajectory, trajectory));

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

	//// go to initial state
	//sendTrajectory(initTrajectory);
	//// wait until the last trajectory segment is sent
	//controller->waitForEnd();

	// start recording
	printf("Start recording (blocking call)\n");
	recordingStart(data, item, true);
	recordingWaitToStart();
	printf("Recording started\n");

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
			RealSeq handTorques; handTorques.assign(dimensions(), REAL_ZERO);
			if (armHandForce) armHandForce->getHandForce(handTorques);
			forceInpSensorSeq.push_back(handTorques);
			robotPoses.push_back(s);
			Twist wrench; if (graspSensorForce) graspSensorForce->readSensor(wrench, s.t);
			ftSeq.push_back(wrench);
			if (!isGrasping && i > 150)
				enableForceReading = expectedCollisions(s);
			//if (!isGrasping && !enableForceReading && i > 150 && expectedCollisions(s))
			//	enableForceReading = true;
		}
	}
	enableForceReading = false;
	if (record) {
		std::stringstream prefix;
		prefix << std::fixed << std::setprecision(3) << "./data/boris/experiments/" << trialPtr->second->object.c_str() << "/" << trialPtr->second->name <<  "/obs_model_data/" << item << "-" << context.getTimer().elapsed();

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
		golem::mkdir(dataIndexPath.c_str()); // make sure the directory exists
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
			//size_t jointInCollision = !objectPointCloudPtr->empty() ? collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(robotPoses[indexjj]), links, true) : 0;
			//dataSimContact << indexjj << "\t";
			//strTorques(dataSimContact, links, jointInCollision);
			//dataSimContact << std::endl;

			dataIndex << ++j << "\t";
			strState(dataIndex, robotPoses[indexjj], handInfo);
			strFT(dataIndex, ftSeq[indexjj]);
			strTorques(dataIndex, forceInpSensorSeq[indexjj]);
			RealSeq links; links.assign(dimensions(), REAL_ZERO);
			size_t jointInCollision = !objectPointCloudPtr->empty() ? collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(robotPoses[indexjj]), links, true) : 0;
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

bool RagPlanner::execute(data::Data::Map::iterator dataPtr, grasp::Waypoint::Seq& trajectory) {
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
	vl[0] = Real(0.22); vl[1] = Real(0.6); vl[2] = Real(0.1); vl[3] = Real(0.1);
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
		try {
			findTarget(grasp::to<Data>(dataPtr)->queryTransform, grasp::to<Data>(dataPtr)->modelFrame, trajectory[1].state, cend);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}
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

		Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend).config, manipulator->getConfig(cend).frame.toMat34());
		renderHand(cend, bounds, true);

		Controller::State::Seq approach;
		try {
			(void)findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}

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
			// add current states
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		handBounds.clear();
		debugRenderer.reset();
		//brecord = true;
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
		Controller::State cend = trajectory[1].state;
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
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
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
		resetPlanning();

		return true;
	}
	case 'T':
	{
		context.debug("Plan trajectory optimisation\n");
		std::string trjItemName("queryTrj");
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(true);
		isGrasping = false;
		// select index 
		U32 index = 1;
		selectIndex(trajectory, index, "waypoint");
		index -= 1;

		Controller::State cend = lookupState();
		cend.cpos = trajectory[index].state.cpos;

		// transform w.r.t. query frame
		findTarget(grasp::to<Data>(dataPtr)->queryTransform, trajectory[index].state, cend);

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getConfig(cend).toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach;
		try {
			findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		}
		catch (const Message&) { return false; }

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
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
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
			cend.cpos[i] = trajectory[index].state.cpos[i];

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
		//findTrajectory(lookupState(), &cend, nullptr, 0, approach);

		Controller::State::Seq out;
		profile(Real(5.0)/*this->trjDuration*/, approach, out, silent);

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
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}


		// disable active controller to apply enough force to the object to lift it.
		if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED) {
			armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
			armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
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
		Controller::State init = cend;
		// transform w.r.t. query frame 
		try {
			findTarget(Mat34::identity(), cend, cend, true);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(target), manipulator->getConfig(target).toMat34());
		//renderHand(target, bounds, true);

		//for (auto j = handInfo.getJoints().begin() + 1; j != handInfo.getJoints().end(); ++j) {
		//	const size_t k = j - handInfo.getJoints().begin();
		//	target.cpos[j] = vl[k];
		//}

		Controller::State::Seq approach;
		try {
			findTrajectory(init, &cend, nullptr, 0, approach);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}

		Controller::State::Seq out;
		profile(Real(5.0)/*this->trjDuration*/, approach, out, silent);

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
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// disable active controller to apply enough force to the object to lift it.
		if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED) {
			armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
			armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
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

		// open hand and release the object
		// pre-grasp pose w.r.t. query frame
		::Sleep(1000);
		Controller::State openfingers = lookupState();
		Controller::State cnow = lookupState();
		for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
			for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j)
				openfingers.cpos[j] = trajectory[1].state.cpos[j]; // pre-grasp fingers' pose
		}
		// transform w.r.t. query frame 
		findTarget(Mat34::identity(), openfingers, openfingers);
		
		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(openfingers), manipulator->getConfig(openfingers).toMat34());
		//renderHand(openfingers, bounds, true);
		//Bounds::Seq bounds2 = manipulator->getBounds(manipulator->getConfig(cnow), manipulator->getConfig(cnow).toMat34());
		//renderHand(openfingers, bounds2, false);

		Controller::State::Seq openTrj;
		openTrj.push_back(cnow);
		openTrj.push_back(openfingers);

		Controller::State::Seq out2;
		profile(Real(2.0), openTrj, out2, silent);
		std::string releaseTrjName = "releaseObjTrj";

		// add trajectory waypoint
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(releaseTrjName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), data::Item::Map::value_type(releaseTrjName, modelHandlerTrj->create()));
			data::Trajectory* trajectory = is<data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		// perform trajectory
		try {
			perform(dataPtr->first, ptr->first, out2, !silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}
		// disable active controller to apply enough force to the object to grasp.
		if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_ENABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_ENABLED) {
			armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_ENABLED);
			armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_ENABLED);
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
//			//collision->draw(manipulator->getConfig(state), debugRenderer);
//			//(*pBelief->getHypotheses().begin())->draw(debugRenderer, rand, manipulator->getConfig(state));
//			Collision::FlannDesc flann;
//			flann.neighbours = 100;
//			flann.points = 1000;
//			flann.depthStdDev = Real(10);
//			flann.likelihood = Real(100);
//			std::vector<Configspace::Index> joints;
//			grasp::RealSeq forces;
//			(*pBelief->getHypotheses().begin())->draw(debugRenderer, manipulator->getConfig(state), joints, forces, flann);
//		}
//			//(*pBelief->getHypotheses().begin())->draw(w, manipulator->getConfig(state), debugRenderer);
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
	if (!pBelief.get() || grasp::to<Data>(dataPtr)->queryPoints.empty())
		return;
	context.debug("RagPlanner::updateAndResample(): %d triggered guards:\n", /*grasp::to<Data>(dataPtr)->*/triggeredGuards.size());
	
	// update samples' weights
	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
	golem::Waypoint w(*controller, lookupState().cpos/*grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos*/);
	context.write("update weights, triggered guards = %u\n", triggeredGuards.size());

	// render uncertainty before belief update
	resetDataPointers();
	showQueryDistribPointClouds = true;
	to<Data>(dataCurrentPtr)->createRender();

	printf("Start recording (blocking call)\n");
	const std::string itemName = "belief-" + makeString("%f", context.getTimer().elapsed());
	recordingStart(dataPtr->first, itemName, true);
	recordingWaitToStart();
	printf("Recording started\n");

	::Sleep(1000);
	pBelief->createUpdate(collisionPtr, w, triggeredGuards, trialPtr != trialDataMap.end() ? grasp::to<TrialData>(trialPtr)->queryPointsTrn : grasp::RBCoord());

	// render the mismatch between estimate and ground truth before resampling
	to<Data>(dataCurrentPtr)->createRender();
	::Sleep(5000);


	context.debug("resample (wheel algorithm)...\n");
	// resampling (wheel algorithm)
	pBelief->createResample(/*manipulator->getConfig(getStateFrom(w))*/);

	// update the query frame
	context.debug("create hypotheses and update query frame...\n");
	grasp::RBPose::Sample mlFrame = pBelief->createHypotheses(modelPoints, modelFrame);
	Mat33 qq; mlFrame.q.toMat33(qq);
	grasp::to<Data>(dataPtr)->queryTransform/*actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame*/ = Mat34(qq, mlFrame.p);

	// update query settings
	grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->queryTransform, modelFrame);
	//grasp::RBPose::Sample cc(grasp::to<Data>(dataPtr)->queryFrame);
	//context.write("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t\n", cc.p.x, cc.p.y, cc.p.z, cc.q.w, cc.q.x, cc.q.y, cc.q.z);

	context.debug("render...\n");
	showQueryDistribPointClouds = false;
	showGroundTruth = true;
	showMeanHypothesisPointClouds = true;
	showHypothesesPointClouds = true;
	createRender();

//	showSamplePoints = true;
	//if (screenCapture) universe.postScreenCaptureFrames(-1);
	//::Sleep(50);
//	renderUncertainty(pBelief->getSamples());
	//showSamplePoints = true; // shows hypotheses and mean pose
	//showMeanHypothesis = false;
	//showDistrPoints = true;
//	if (screenCapture) universe.postScreenCaptureFrames(-1);
	pHeuristic->setHypothesisBounds();

	// stop recording
	recordingStop(trajectoryIdlePerf);
	recordingWaitToStop();

	// render the mismatch between estimate and ground truth after resempling
//	createRender();
	//::Sleep(100);
//	if (screenCapture) universe.postScreenCaptureFrames(0);

	//::Sleep(100);
	//if (screenCapture) universe.postScreenCaptureFrames(0);	
//	std::cout << "spam:update&resample 15\n";
}

//------------------------------------------------------------------------------

#ifdef _SPAM_RAG_MAIN_
int main(int argc, char *argv[]) {
	return spam::RagPlanner::Desc().main(argc, argv);
}
#endif // _SPAM_RAG_MAIN_