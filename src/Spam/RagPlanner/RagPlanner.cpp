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
//#define  GLUT_KEY_INSERT                    0x006C
//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::RagPlanner::Data::xmlData(golem::XMLContext* context, bool create) const {
	ShapePlanner::Data::xmlData(context, create);

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

grasp::Director::Data::Ptr RagPlanner::createData() const {
	return Data::Ptr(new Data(*grasp::to<Data>(data))); // assuming data has been initialised properly
}

//------------------------------------------------------------------------------

RagPlanner::RagPlanner(Scene &scene) : ShapePlanner(scene),/* pBelief(nullptr),*/ pHeuristic(nullptr) {
}

RagPlanner::~RagPlanner() {
}

bool RagPlanner::create(const Desc& desc) {
	ShapePlanner::create(desc); // throws

	robot = dynamic_cast<Robot*>(ShapePlanner::robot);
	if (robot == NULL)
		throw Message(Message::LEVEL_CRIT, "RagPlanner::create(): unable to cast to Robot");

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
	showIndices = false;

	collisionPtr = desc.objCollisionDescPtr->create(*manipulator);

	std::stringstream prefix;
	prefix << std::fixed << std::setprecision(3) << "./data/boris/coke/ftsensors" << "-" << context.getTimer().elapsed();
	contact = false;
	record = false;
	brecord = false;

	auto strTorquesDesc = [=](std::ostream& ostr) {
		ostr << "timestamp" << grasp::to<Data>(data)->sepField << "contact" << grasp::to<Data>(data)->sepField << "thumb_0" << grasp::to<Data>(data)->sepField << "thumb_1" << grasp::to<Data>(data)->sepField << "thumb_2" << grasp::to<Data>(data)->sepField << "thumb_3" << grasp::to<Data>(data)->sepField <<
			"index_0" << grasp::to<Data>(data)->sepField << "index_1" << grasp::to<Data>(data)->sepField << "index_2" << grasp::to<Data>(data)->sepField << "index_3" << grasp::to<Data>(data)->sepField <<
			"middle_0" << grasp::to<Data>(data)->sepField << "middle_1" << grasp::to<Data>(data)->sepField << "middle_2" << grasp::to<Data>(data)->sepField << "middle_3" << grasp::to<Data>(data)->sepField <<
			"ring_0" << grasp::to<Data>(data)->sepField << "ring_1" << grasp::to<Data>(data)->sepField << "ring_2" << grasp::to<Data>(data)->sepField << "ring_3" << grasp::to<Data>(data)->sepField <<
			"pinky_0" << grasp::to<Data>(data)->sepField << "pinky_1" << grasp::to<Data>(data)->sepField << "pinky_2" << grasp::to<Data>(data)->sepField << "pinky_3" << grasp::to<Data>(data)->sepField;
	};

	// writing data into a text file, open file
	const std::string dataIndexPathRaw = grasp::makeString("%s_raw.txt", prefix.str().c_str());
	const std::string dataIndexPathFiltered = grasp::makeString("%s_filtered.txt", prefix.str().c_str());

//	if (enableSimContact) {
	// raw data from drl hand FT
		dataFTRaw.open(dataIndexPathRaw);
		// writing data into a text file, prepare headers
		dataFTRaw << "#index" << grasp::to<Data>(data)->sepField;
		strTorquesDesc(dataFTRaw);
		dataFTRaw << std::endl;
		// filtered data from DLR hand FT
		dataFTFiltered.open(dataIndexPathFiltered);
		// writing data into a text file, prepare headers
		dataFTFiltered << "#index" << grasp::to<Data>(data)->sepField;
		strTorquesDesc(dataFTFiltered);
		dataFTFiltered << std::endl;
//	}

	// simulates contacts between robot's hand and the object's point cloud
	robot->setHandForceReader([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws		
		// associate random noise [-0.1, 0.1]
		static int indexjj = 0;
		if (enableSimContact)
			for (auto i = 0; i < force.size(); ++i)
				force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);

		size_t jointInCollision = enableSimContact && !objectPointCloudPtr->empty() ? collisionPtr->simulate(desc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(robot->recvState().config), force) : 0;

		//// writing data into a text file
		auto strTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const bool contact) {
			ostr << state.t << grasp::to<Data>(data)->sepField;
			std::string c = contact ? "y" : "n";
			ostr << c.c_str() << grasp::to<Data>(data)->sepField;
			//grasp::RealSeq torques;
			//torques.assign((size_t)robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
			//robot->readFT(state, torques);
			for (auto i = 0; i < forces.size(); ++i)
				ostr << forces[i] << grasp::to<Data>(data)->sepField;
		};
		if (++indexjj % 10 == 0) 	{
			robot->collectFTInp(state, force);
			if (record) {
				//breakPoint();
				dataFTRaw << indexjj << grasp::to<Data>(data)->sepField;
				strTorques(dataFTRaw, state, force, contact);
				dataFTRaw << std::endl;
				grasp::RealSeq f = robot->getFilterForce();
				dataFTFiltered << indexjj << grasp::to<Data>(data)->sepField;
				strTorques(dataFTFiltered, state, f, contact);
				dataFTFiltered << std::endl;
			}
		}

		if (!enableForceReading)
			return;

		triggeredGuards.clear();
		std::vector<Configspace::Index> joints;
		robot->guardsReader(robot->recvState().command, force, joints);
		for (U32 i = 0; i != joints.size(); ++i) {
			FTGuard guard(*manipulator);
			guard.create(joints[i]);
			const size_t k = joints[i] - robot->getStateHandInfo().getJoints().begin();
			guard.force = force[k];
			guard.threshold = robot->getFlimit(k);
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
				grasp::RealSeq f = robot->getFilterForce();
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

	//robot->setHandContactDetector([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws
	//	// writing data into a text file
	//	auto strTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const bool contact) {
	//		ostr << state.t << grasp::to<Data>(data)->sepField;
	//		std::string c = contact ? "y" : "n";
	//		ostr << c.c_str() << grasp::to<Data>(data)->sepField;
	//		//grasp::RealSeq torques;
	//		//torques.assign((size_t)robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
	//		//robot->readFT(state, torques);
	//		for (auto i = 0; i < forces.size(); ++i)
	//			ostr << forces[i] << grasp::to<Data>(data)->sepField;
	//	};
	//	static int indexjj = 0;
	//	//auto breakPoint = [&]() { if (waitKey(10) == ' ') contact = !contact; };
	//	//auto recording = [&]() { if (waitKey(10) == 27) record = !record; };
	//	//recording();
	//	if (record && ++indexjj % 10 == 0) {
	//		//breakPoint();
	//		dataFTFiltered << indexjj << grasp::to<Data>(data)->sepField;
	//		strTorques(dataFTFiltered, state, force, contact);
	//		dataFTFiltered << std::endl;
	//	}
	//	if (!enableForceReading)
	//		return;
	//	std::vector<Configspace::Index> joints;
	//	robot->guardsReader(robot->recvState().command, force, joints);
	//	for (U32 i = 0; i != joints.size(); ++i) {
	//		FTGuard guard(*manipulator);
	//		guard.create(joints[i]);
	//		guard.force = force[i];
	//		guard.threshold = robot->getFlimit(i);
	//		triggeredGuards.push_back(guard);
	//	}
	//	if (!triggeredGuards.empty()) {
	//		for (FTGuard::Seq::const_iterator g = triggeredGuards.begin(); g != triggeredGuards.end(); ++g)
	//			g->str();
	//		if (force.size() >= 20) {
	//			context.write("Forces: Thumb: [%3.3lf %3.3lf %3.3lf %3.3lf] Index [%3.3lf %3.3lf %3.3lf %3.3lf] Middle [%3.3lf %3.3lf %3.3lf %3.3lf] Ring [%3.3lf %3.3lf %3.3lf %3.3lf] Pinky [%3.3lf %3.3lf %3.3lf %3.3lf]\n",
	//				force[0], force[1], force[2], force[3],
	//				force[4], force[5], force[6], force[7],
	//				force[8], force[9], force[10], force[11],
	//				force[12], force[13], force[14], force[15],
	//				force[16], force[17], force[18], force[19]);
	//		}
	//		throw Message(Message::LEVEL_NOTICE, "spam::Robot::handForceReader(): Triggered %d guard(s).\n", triggeredGuards.size());
	//	}
	//});

	enableControllers = false;
	// called after guards are triggered
	robot->setEmergencyModeHandler([=]() {
//		context.debug("Emergency mode handler\n");
		enableForceReading = false;
		contactOccured = true;
		//grasp::to<Data>(currentDataPtr)->replanning = true;
		robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_DISABLED);
		robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_DISABLED);
//		updateAndResample(currentDataPtr);
//		triggeredGuards.clear();
		//robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
		//robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
	}); // end robot->setEmergencyModeHandler

//	poseDataPtr = getData().end();

//	pBelief = static_cast<Belief*>(pRBPose.get());
	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&robot->getPlanner()->getHeuristic()); 
	Bounds::Seq b = manipulator->getBounds(manipulator->getConfig(robot->recvState().command), manipulator->getPose(robot->recvState().command).toMat34());
//	context.write("RagPlanner::create(): bound size=%d\n", b.size());
	pHeuristic->setBelief(pBelief);
	pHeuristic->setManipulator(manipulator.get());
	//	robot->setManipulator(manipulator.get());

	uncEnable = desc.uncEnable;
	singleGrasp = desc.singleGrasp;
	withdrawToHomePose = desc.withdrawToHomePose;
	posterior = true;

//	queryPointsTrn.set(desc.queryPointsTrn.p, desc.queryPointsTrn.q);

	homeStates.clear();
	executedTrajectory.clear();
	triggeredGuards.clear();

	ragDesc = desc;

	handBounds.clear();
	robotStates.clear();

	isGrasping = false;

	return true;
}

void RagPlanner::render() {
	ShapePlanner::render();
//	handRenderer.reset();
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		//handRenderer.setWireColour(RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(255)));
		//handRenderer.setLineWidth(Real(2.0));
//		handRenderer.renderWire(handBounds.begin(), handBounds.end());
		testPose.render();
		testUpdate.render();
		debugRenderer.render();
	}
}

//------------------------------------------------------------------------------

grasp::Cloud::PointSeqMap::iterator RagPlanner::getTrnPoints(Data::Map::iterator dataPtr, const Mat34 &trn) {
	grasp::Cloud::PointSeqMap::iterator points;
	if (!grasp::to<Data>(dataPtr)->ptrPoints || (points = grasp::to<Data>(dataPtr)->getPoints<grasp::Cloud::PointSeqMap::iterator>(grasp::to<Data>(dataPtr)->points)) == grasp::to<Data>(dataPtr)->points.end())
		throw Message(Message::LEVEL_NOTICE, "Director::getPoints(): No points are selected!");

	grasp::RBCoord m(trn);
	context.debug("Transform processed points <%.4f %.4f %.4f> [%.4f %.4f %.4f %.4f]\n", m.p.x, m.p.y, m.p.z, m.q.w, m.q.x, m.q.y, m.q.z);
	// transform points
	grasp::Cloud::transform(trn, points->second, points->second);
	
	return points;
}

grasp::Cloud::RawPointSeqMultiMap::iterator RagPlanner::getTrnRawPoints(Data::Map::iterator dataPtr, const Mat34 &trn) {
	grasp::Cloud::RawPointSeqMultiMap::iterator points;
	if ((points = grasp::to<Data>(dataPtr)->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(grasp::to<Data>(dataPtr)->pointsRaw)) == grasp::to<Data>(dataPtr)->pointsRaw.end())
		throw Message(Message::LEVEL_NOTICE, "Director::getPoints(): No points are selected!");

	grasp::RBCoord m(trn);
	context.debug("Transform raw points <%.4f %.4f %.4f> [%.4f %.4f %.4f %.4f]\n", m.p.x, m.p.y, m.p.z, m.q.w, m.q.x, m.q.y, m.q.z);
	// transform points
	grasp::Cloud::RawPointSeqVal val;
	val.first = points->first;
	grasp::Cloud::transform(trn, points->second, val.second);
	return grasp::to<Data>(dataPtr)->insertPoints(val, grasp::to<Data>(dataPtr)->getLabelName(val.first));
}

//------------------------------------------------------------------------------

Real RagPlanner::simContacts(const golem::Bounds::Seq::const_iterator &begin, const golem::Bounds::Seq::const_iterator &end, const golem::Mat34 pose) {
	// if the point cloud is empty simContacts return the default behavior of force reader.
	if (objectPointCloudPtr->empty())
		return REAL_ZERO;

	Rand rand;
	bool intersect = false;
	for (Bounds::Seq::const_iterator b = begin; b != end; ++b) {
		const size_t size = objectPointCloudPtr->size() < ragDesc.maxModelPoints ? objectPointCloudPtr->size() : ragDesc.maxModelPoints;
		for (size_t i = 0; i < size; ++i) {
			const Vec3 point = grasp::Cloud::getPoint(objectPointCloudPtr->at(objectPointCloudPtr->size() < ragDesc.maxModelPoints ? i : size_t(rand.next()) % size));
			//context.write("force reader\n");
			//context.write("bound pose <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
			if ((*b)->intersect(point)) {
				context.write("bound pose <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
				Vec3 v;
				Mat34 jointFrameInv;
				jointFrameInv.setInverse(pose);
				jointFrameInv.multiply(v, point);
				v.normalise();
				return v.z > REAL_ZERO ? -REAL_ONE : REAL_ONE;
			}
		}
	}
	return REAL_ZERO;
}

//------------------------------------------------------------------------------

void RagPlanner::renderData(Data::Map::const_iterator dataPtr) {
	ShapePlanner::renderData(dataPtr);
	{
		golem::CriticalSectionWrapper csw(csDataRenderer);
		debugRenderer.reset();
		//if (!objectPointCloudPtr->empty()) {
		//	grasp::Cloud::Point p = *objectPointCloudPtr->begin();
		//	Mat34 m(Mat33::identity(), Vec3(p.x, p.y, p.z));
		//	debugRenderer.addAxes(m, featureFrameSize * 10);
		//}
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

void RagPlanner::renderHand(const golem::Controller::State &state, const Bounds::Seq &bounds, bool clear) {
	{		
		golem::CriticalSectionWrapper csw(csDataRenderer);
		debugRenderer.reset();
		if (clear)
			handBounds.clear();
		if (!bounds.empty())
			handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
		debugRenderer.setColour(RGBA::BLACK);
		debugRenderer.setLineWidth(Real(2.0));
		debugRenderer.addWire(handBounds.begin(), handBounds.end());

		Collision::Waypoint w;
		w.points = 1000;
		if (showIndices) {
			//collision->draw(manipulator->getPose(state), debugRenderer);
			//(*pBelief->getHypotheses().begin())->draw(debugRenderer, rand, manipulator->getPose(state));
			Collision::FlannDesc flann;
			flann.neighbours = 100;
			flann.points = 1000;
			flann.depthStdDev = Real(10);
			flann.likelihood = Real(100);
			std::vector<Configspace::Index> joints;
			grasp::RealSeq forces;
			(*pBelief->getHypotheses().begin())->draw(debugRenderer, manipulator->getPose(state), joints, forces, flann);
		}
			//(*pBelief->getHypotheses().begin())->draw(w, manipulator->getPose(state), debugRenderer);
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

void RagPlanner::renderWaypoints(const Bounds::Seq &bounds, bool clear) {
		{
			golem::CriticalSectionWrapper csw(csDataRenderer);
			debugRenderer.reset();
			if (clear)
				waypointBounds.clear();
			if (!bounds.empty())
				waypointBounds.insert(waypointBounds.end(), bounds.begin(), bounds.end());
			debugRenderer.setColour(RGBA::RED);
			debugRenderer.setLineWidth(Real(2.0));
			debugRenderer.addWire(waypointBounds.begin(), waypointBounds.end());
			//		handRenderer.renderWire(handBounds.begin(), handBounds.end());
			//context.write("Hand bounds (size=%d)\n", handBounds.size());
			//for (Bounds::Seq::const_iterator i = handBounds.begin(); i != handBounds.end(); ++i)
			//	context.write("%s pose <%f %f %f>\n", (*i)->getName(), (*i)->getPose().p.x, (*i)->getPose().p.y, (*i)->getPose().p.z);
		}
}

//------------------------------------------------------------------------------

/** Profile state sequence */
void RagPlanner::profile(golem::SecTmReal duration, const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& out, const bool silent) const {
	if (!silent)
		readNumber("Trajectory duration: ", duration);

	// velocity profiling
	Player::profile(duration, inp, out);
}

void RagPlanner::perform(const std::string& name, const golem::Controller::State::Seq& trajectory, bool silent) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	//context.debug("spam::RagPlanner::perform(): computing initial trajectory\n");
	//golem::Controller::State::Seq initTrajectory;
	//robot->findTrajectory(robot->recvState().command, &trajectory.front(), nullptr, 0, initTrajectory);

	//golem::Controller::State::Seq completeTrajectory = initTrajectory;
	//completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());
	//if (!silent && !testTrajectory(completeTrajectory.begin(), completeTrajectory.end()))
	//	return;

	// stop active controller before moving
//	robot->emergencyActiveController();
//	robot->startActiveController(); // set up offset

	std::stringstream prefix;
	prefix << std::fixed << std::setprecision(3) << name << "-" << context.getTimer().elapsed();

	//// start recording the approach trajectory if isRecordingApp() is set
	//for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
	//	if ((*i)->isRecordingApp()) (*i)->start(prefix.str().c_str());
	//// block untilthe  first image arrives
	//for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
	//	if ((*i)->isRecordingApp()) (*i)->wait();

	//// go to initial state
	//robot->sendTrajectory(initTrajectory);
	//// wait until the last trajectory segment is sent
	//robot->waitForEnd();

	auto breakPoint = [&]() { if (waitKey(10) == ' ') contact = !contact; };
	auto recording = [&]() { if (waitKey(10) == 27) record = !record; };

	// start recording the remaining trajectory part
	for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
		if (!(*i)->isRecordingApp()) (*i)->start(prefix.str().c_str());
	// block until the first image arrives
	for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
		if (!(*i)->isRecordingApp()) (*i)->wait();


	context.debug("spam::RagPlanner::perform(): sending grasp trajectory (forcereading %s)\n", enableForceReading ? "ON" : "OFF");
	// send trajectory
	printing = true;
	//robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
	//robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);

	robot->sendTrajectory(trajectory);
//	robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
//	robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);

	robotStates.clear();
	if (screenCapture) universe.postScreenCaptureFrames(-1);
	record = true;
	// repeat every send waypoint until trajectory end
	for (U32 i = 0; robot->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw grasp::Interrupted();
		if (robot->waitForEnd(0))
			break;

		grasp::RobotState s = robot->recvState();
		// print every 10th robot state
		if (i % 10 == 0) {
			context.write("State #%d - recording %s contact %s\r", i, record ? "Y" : "N", contact ? "Y" : "N");
			if (!isGrasping && !enableForceReading && i > 150 && robot->expectedCollisions(s.command)) { //
			//if (i > 250 && !enableForceReading) {
				// start active controller only if expected contacts
				//robot->startActiveController();
				enableForceReading = true;
			}
		}
//		printState(s.command, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
		recording();
		breakPoint();
		robotStates.push_back(s);
	}
	record = false;
	// stop force reading to quit printing
	enableForceReading = false;

	// insert the executed trajectory to the trial
	if (trialPtr != trialDataMap.end())
		(void)grasp::to<TrialData>(trialPtr)->insert(robotStates);

	// stop recording at the last frame
	// HACK: comment out for Justin! (stop manually due to faulty sync)
	for (grasp::Recorder::Seq::const_iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
		(*i)->stop(context.getTimer().elapsed() + trjPerfOff);//robotStates.back().config.t;

	context.write("----------------------------------------------------------------------\nPerformance finished!\n");
	if (screenCapture) universe.postScreenCaptureFrames(0);	

	// writing data into a text file, data conversion
	auto strMat34 = [=](std::ostream& ostr, const Mat34& mat) {
		Quat quat(mat.R);
		ostr << mat.p.x << grasp::to<Data>(data)->sepField << mat.p.y << grasp::to<Data>(data)->sepField << mat.p.z << grasp::to<Data>(data)->sepField << quat.x << grasp::to<Data>(data)->sepField << quat.y << grasp::to<Data>(data)->sepField << quat.z << grasp::to<Data>(data)->sepField << quat.w << grasp::to<Data>(data)->sepField;
	};
	auto strMat34Desc = [=](std::ostream& ostr, const std::string& prefix, U32 index) {
		ostr << prefix << "x" << index << grasp::to<Data>(data)->sepField << prefix << "y" << index << grasp::to<Data>(data)->sepField << prefix << "z" << index << grasp::to<Data>(data)->sepField << prefix << "qx" << index << grasp::to<Data>(data)->sepField << prefix << "qy" << index << grasp::to<Data>(data)->sepField << prefix << "qz" << index << grasp::to<Data>(data)->sepField << prefix << "qw" << index << grasp::to<Data>(data)->sepField;
	};
	auto strState = [=](std::ostream& ostr, const Controller::State& state, const Controller::State::Info& info) {
		ostr << state.t << grasp::to<Data>(data)->sepField;
		WorkspaceJointCoord wjc;
		robot->getController()->jointForwardTransform(state.cpos, wjc);
		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
			ostr << std::fixed << std::setprecision(6) << state.cpos[i] << grasp::to<Data>(data)->sepField;
//			strMat34(ostr, wjc[i]);
		}
	};
	auto strStateDesc = [=](std::ostream& ostr, const Controller::State::Info& info, const std::string& prefix) {
		ostr << prefix << "t" << grasp::to<Data>(data)->sepField;
		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
			ostr << prefix << "j" << *i << grasp::to<Data>(data)->sepField;
//			strMat34Desc(ostr, prefix, *i);
		}
	};
	auto strFT = [=](std::ostream& ostr, const Twist& twist) {
		ostr << twist.v.x << grasp::to<Data>(data)->sepField << twist.v.y << grasp::to<Data>(data)->sepField << twist.v.z << grasp::to<Data>(data)->sepField << twist.w.x << grasp::to<Data>(data)->sepField << twist.w.y << grasp::to<Data>(data)->sepField << twist.w.z << grasp::to<Data>(data)->sepField;
	};
	auto strFTDesc = [=](std::ostream& ostr, const std::string& prefix) {
		ostr << prefix << "vx" << grasp::to<Data>(data)->sepField << prefix << "vy" << grasp::to<Data>(data)->sepField << prefix << "vz" << grasp::to<Data>(data)->sepField << prefix << "wx" << grasp::to<Data>(data)->sepField << prefix << "wy" << grasp::to<Data>(data)->sepField << prefix << "wz" << grasp::to<Data>(data)->sepField;
	};
	auto strTorquesDesc = [=](std::ostream& ostr) {
		ostr << /*"timestamp" << grasp::to<Data>(data)->sepField << "contact" << grasp::to<Data>(data)->sepField <<*/ "thumb_0" << grasp::to<Data>(data)->sepField << "thumb_1" << grasp::to<Data>(data)->sepField << "thumb_2" << grasp::to<Data>(data)->sepField << "thumb_3" << grasp::to<Data>(data)->sepField <<
			"index_0" << grasp::to<Data>(data)->sepField << "index_1" << grasp::to<Data>(data)->sepField << "index_2" << grasp::to<Data>(data)->sepField << "index_3" << grasp::to<Data>(data)->sepField <<
			"middle_0" << grasp::to<Data>(data)->sepField << "middle_1" << grasp::to<Data>(data)->sepField << "middle_2" << grasp::to<Data>(data)->sepField << "middle_3" << grasp::to<Data>(data)->sepField <<
			"ring_0" << grasp::to<Data>(data)->sepField << "ring_1" << grasp::to<Data>(data)->sepField << "ring_2" << grasp::to<Data>(data)->sepField << "ring_3" << grasp::to<Data>(data)->sepField <<
			"pinky_0" << grasp::to<Data>(data)->sepField << "pinky_1" << grasp::to<Data>(data)->sepField << "pinky_2" << grasp::to<Data>(data)->sepField << "pinky_3" << grasp::to<Data>(data)->sepField;
	};
	auto strTorques = [=](std::ostream& ostr, const Controller::State& state, const bool contact) {
		//ostr << state.t << grasp::to<Data>(data)->sepField;
		//std::string c = contact ? "y" : "n";
		//ostr << c.c_str() << grasp::to<Data>(data)->sepField;
		grasp::RealSeq torques;
		torques.assign((size_t)robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
		robot->readFT(state, torques);
		for (auto i = 0; i < torques.size(); ++i)
			ostr << std::fixed << std::setprecision(6) << torques[i] << grasp::to<Data>(data)->sepField;
	};


	// writing data into a text file, open file
	const std::string dataIndexPath = grasp::makeString("%s.txt", prefix.str().c_str());
	std::ofstream dataIndex(dataIndexPath);

	// writing data into a text file, prepare headers
	dataIndex << "#index" << grasp::to<Data>(data)->sepField;
	strStateDesc(dataIndex, robot->getStateHandInfo(), std::string("cmd_"));
	strStateDesc(dataIndex, robot->getStateHandInfo(), std::string("cfg_"));
	strFTDesc(dataIndex, std::string("ft_"));
	strTorquesDesc(dataIndex);
	dataIndex << std::endl;

	// writing data into a text file
	U32 j = 0, c = 0;
	for (grasp::RobotState::Seq::const_iterator i = robotStates.begin(); i != robotStates.end(); ++i) {
		dataIndex << ++j << grasp::to<Data>(data)->sepField;
		strState(dataIndex, i->command, robot->getStateHandInfo());
		strState(dataIndex, i->config, robot->getStateHandInfo());
		strFT(dataIndex, i->getSensorData<Twist>(grasp::RobotState::SENSOR_WRENCH));
		strTorques(dataIndex, i->command, "n");
		dataIndex << std::endl;
	}
}

bool RagPlanner::unlockContact() {
	context.write("unlockContact(): robotStates.size()=%d\n", robotStates.size());
	if (robotStates.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::unlockContact: robotState empty.");
	
	Controller::State::Seq trj, out;
	Controller::State state = robot->recvState().command;
	trj.push_back(robot->recvState().command);
	for (auto i = robotStates.size() - 1; i >= 0; --i) {
		printState(robotStates[i].command, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());

		//if (Math::abs(collision->evaluate(w, *objectPointCloudPtr.get(), rand, manipulator->getPose(robotStates[i].command), true)) > REAL_ZERO)
		//	continue;
		
		for (auto j = robot->getStateInfo().getJoints().begin(); j != robot->getStateInfo().getJoints().end(); ++j)
			state.cpos[j] = robotStates[i].command.cpos[j];
		
		trj.push_back(state);
		robot->sendTrajectory(trj);
		// repeat every send waypoint until trajectory end
		for (U32 i = 0; robot->waitForBegin(); ++i) {
			if (universe.interrupted())
				throw grasp::Interrupted();
			if (robot->waitForEnd(0))
				break;

			// print every 10th robot state
			if (i % 10 == 0) {
				context.write("State #%d\r", i);
			}
			grasp::RobotState s = robot->recvState();
			printState(s.command, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
		}
		//robot->findTrajectory(robot->recvState().command, &state, nullptr, 0, trj);
		//profile(this->trjDuration, trj, out, true);
		//try {
		//	perform("unlockContact", out);
		//}
		//catch (const Message& msg) {
		//	context.write("%s\n", msg.str().c_str());
		//	return false;
		//}
		return true;
	}
	return false;
}

void RagPlanner::performApproach(Data::Map::iterator dataPtr) {
	if (grasp::to<Data>(dataPtr)->queryPoints.empty())
		throw Message(Message::LEVEL_ERROR, "RagPlanner::performApproach(): no query is present.");

	context.debug("RagPlanner::performApproach(): build and perform approach trajectory\n");
	// fixed properties of the planner
	pHeuristic->enableUnc = this->uncEnable;
//	pHeuristic->setCollisionDetection(Bounds::GROUP_ALL);
	pHeuristic->setPointCloudCollision(true);

	context.debug("RagPlanner::performApproach(): enable guards\n");
//	robot->enableGuards();
	grasp::to<Data>(dataPtr)->replanning = true;
	robot->setCollisionDetection(waitKey("YN", "Collision detection (Y/N)...") == 'Y' ? true : false);

	const int key = waitKey("PMQ", "(P)lay trajectory, planning from (M)odel-based grasp, planning to estimated (Q)uery-based grasp");
	const bool P = (key == 'P'), M = (key == 'M'), Q = (key == 'Q');
	Controller::State::Seq approach, grasp;
	grasp::RobotState::Map::iterator trajectory;
	std::string name = "QueryGrasp";
	if (P)	{
		trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
		name = trajectory->first.c_str();
		for (grasp::Robot::State::Seq::const_iterator s = trajectory->second.begin(); s != trajectory->second.end(); ++s)
			approach.push_back(s->command);
	}
	else if (M || Q) {
		bool accept = false;
		if (M) {
			trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
			Controller::State::Seq inp = grasp::RobotState::makeCommand(trajectory->second);
			/*grasp::Grasp::Config* config;
			if (config ==grasp::to<Data>(dataPtr)->getGraspConfig()->get()) {
				 = grasp::to<Data>(dataPtr)->getGraspConfig()->get();
				manipulator->getState(*config->path.begin());*/
			//context.write("Grasp config path size %d\n", (*grasp::to<Data>(dataPtr)->getGraspConfig())->path.size());
			//if ((*grasp::to<Data>(dataPtr)->getGraspConfig())->path.empty()) return;
			//for (grasp::Manipulator::Waypoint::Seq::const_iterator i = (*grasp::to<Data>(dataPtr)->getGraspConfig())->path.begin(); i != (*grasp::to<Data>(dataPtr)->getGraspConfig())->path.end(); ++i) {
			//	Controller::State state = manipulator->getState(*i);
			//	inp.push_back(state);
			//}
			// choose first with acceptable error
			const grasp::RBDist dist = robot->transformTrajectory(grasp::to<Data>(dataPtr)->queryTransform, inp.begin(), inp.end(), approach);
			golem::Real err = manipulator->getDesc().trajectoryErr.dot(dist);
			context.write("Trajectory error: lin=%.9f, ang=%.9f, total=%.9f\n", dist.lin, dist.ang, err);
			name = trajectory->first.c_str();
		}
		if (Q) {
			// available point cloud
			const grasp::Cloud::PointSeq& points = grasp::to<Data>(queryDataPtr)->queryPoints.empty() ? getPoints(dataPtr)->second : grasp::to<Data>(queryDataPtr)->queryPoints;
			// available grasp types
			grasp::StringSet availables;
			for (grasp::Grasp::Map::const_iterator i = classifier->getGraspMap().begin(); i != classifier->getGraspMap().end(); ++i)
				if (!i->second->getDataMap().empty())
					availables.insert(i->first);
			if (availables.empty())
				throw Message(Message::LEVEL_NOTICE, "No available grasp estimators with training data!");

			grasp::StringSet types;
			try {
				types.insert(*select(availables.begin(), availables.end(), "Select grasp type:\n  [0]  all types\n", [](grasp::StringSet::const_iterator ptr) -> const std::string&{ return *ptr; }, -1));
			}
			catch (const Message&) {}

			context.write("Estimating grasp...\n");
			classifier->find(points, grasp::to<Data>(dataPtr)->graspConfigs, &types);

			grasp::to<Data>(dataPtr)->resetDataPointers();
			grasp::to<Data>(dataPtr)->graspMode = MODE_CONFIG;

			if (types.empty())
				grasp::Cluster::findLikelihood(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters);
			else {
				grasp::Cluster::findType(clusterDesc, grasp::to<Data>(dataPtr)->graspConfigs, grasp::to<Data>(dataPtr)->graspClusters);
				while (grasp::to<Data>(dataPtr)->graspClusterPtr < grasp::to<Data>(dataPtr)->graspClusters.size() && grasp::to<Data>(dataPtr)->getGraspConfig()->get()->type != *types.begin()) ++grasp::to<Data>(dataPtr)->graspClusterPtr;
			}

			grasp::Grasp::Config* config = grasp::to<Data>(dataPtr)->getGraspConfig()->get();
			manipulator->find(config->path);
			manipulator->copy((*grasp::to<Data>(dataPtr)->getGraspConfig())->path/*config->path*/, approach);
		}
		context.write("Select target grasp.\n'[' previous, ']' next, <ENTER> to accept\n");
		// todo check if seq is empty
		Controller::State target = approach.front();
		Controller::State::Seq::iterator ptr = approach.begin();
		while (!accept) {
			Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(*ptr), manipulator->getPose(*ptr).toMat34());
			renderHand(*ptr, bounds, true);
			switch (waitKey()) {
			case '\x0D':
			{
				accept = true;
				target = *ptr;
				break;
			}
			case'[':
			{
				if (ptr == approach.begin()) break;
				ptr--;
				break;
			}
			case']':
			{
				if (ptr == approach.end() - 1) break;
				ptr++;
				break;
			}
			default:
				break;
			}
		}
		approach.clear();
		robot->findTrajectory(robot->recvState().command, &target, nullptr, /*&(*grasp::to<Data>(currentDataPtr)->getGraspConfigBegin())->path.front().toMat34(),*/ 0, approach);
	}
	readString("Enter target trajectory name: ", name);
	grasp::to<Data>(currentDataPtr)->trajectory[name] = grasp::RobotState::make(manipulator->getController(), approach); // update trajectory collection
	context.debug("spam::RagPlanner::performApproach(): create trajectory profile\n");
	golem::Controller::State::Seq out;
	// velocity profiling
	profile(this->trjDuration, approach, out);
	//// send trajectory
	//robot->sendTrajectory(out);

	//// repeat every send waypoint until trajectory end
	//for (U32 i = 0; robot->waitForBegin(); ++i) {
	//	if (universe.interrupted())
	//		throw grasp::Interrupted();
	//	if (robot->waitForEnd(0))
	//		break;

	//	// print every 10th robot state
	//	if (i % 10 == 0)
	//		context.write("State #%d\r", i);
	//}

	// find manipulation trajectory
	//golem::Controller::State::Seq trj2;
	//createWithManipulation(trj, trj2);

	// perform
	try {
		perform(grasp::to<Data>(dataPtr)->getName(), out);
	}
	catch (const Message& msg) {
		context.write("%s\n", msg.str().c_str());
		return;
	}

	//WorkspaceChainCoord chainPoses;
	//Controller::State state = robot->recvState().config;
	//robot->getController()->chainForwardTransform(state.cpos, chainPoses);
	//for (Chainspace::Index i = state.getInfo().getChains().begin(); i != state.getInfo().getChains().end(); ++i) {
	//	context.debug("Robot distance to Knear: chain %d distance %.4f\n", i - state.getInfo().getChains().begin(), (*pBelief->getHypotheses().begin())->dist2NearestKPoints(grasp::RBCoord(chainPoses[i])));
	//}

//	// handle the approaching action to generate a sequence of states 
//	//context.write("approaching trajetory (list)\n");
//	//printTrajectory(makeCommand(dataPtr->second.approachAction), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
////	createGrasp(dataPtr);
//	//context.write("approaching trajetory (list)\n");
//	//printTrajectory(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
//	Controller::State::Seq seq;
//	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
//
//	// compute the approach traectory in a new frame and its profile
//	context.debug("RagPlanner::performApproach(): Computing trajectory in a new frame and generating profile...\n");
//	Controller::State::Seq seqTrn, trjApproach, trjGrasp;
//	// compute trajectory to the pre-grasp pose with point cloud collision detection ON
//	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.begin(), seq.begin() + 1, seqTrn);
//
//	// debug: shows pregrasp and grasp poses of the hand
//	context.debug("RagPlanner::performApproach(): render pre-grasp pose.\n");
//	Controller::State pregrasp = seqTrn.front();
//	renderHand(pregrasp, true);
//	//Controller::State grasp = seqTrn.back();
//	//renderHand(grasp);
//	grasp::Manipulator::Pose pose1 = manipulator->getPose(pregrasp);
//	context.debug("RagPlanner::performApproach(): manipulator pose <%f %f %f>\n", pose1.p.x, pose1.p.y, pose1.p.z);
//
//	robot->createTrajectory(robot->recvState().config, &pregrasp, NULL, 0, trjApproach);
//	grasp::to<Data>(dataPtr)->action.clear();
////	trjApproach.insert(trjApproach.end(), seqTrn.begin(), seqTrn.end()-1);
//	context.debug("RagPlanner::performApproach(): compute profile.\n");
//	profile(dataPtr, trjApproach, trjApproachDuration, trjApproachIdle);
//
//	// perform approaching action
//	context.debug("RagPlanner::performApproach(): perform action.\n");
//	perform(dataPtr);
//
//	// set the current state of the robot after the trajectory is over
//	context.debug("RagPlanner::performApproach(): retrieve current state of the robot and check guards.\n");
//	Controller::State state = robot->recvState().config;
//	//renderHand(state, true);
//	grasp::Manipulator::Pose pose = manipulator->getPose(state);
//	context.debug("RagPlanner::performApproach(): manipulator pose <%f %f %f>\n", pose.p.x, pose.p.y, pose.p.z);
//	grasp::to<Data>(dataPtr)->triggered = robot->getTriggeredGuards(/*grasp::to<Data>(dataPtr)->*/triggeredGuards, state); 
//
//	// -1 means no contact between the object and the hand (most likely a contact with the table).
//	if (grasp::to<Data>(dataPtr)->triggered == -1)
//		return;
//
//	// triggered == 0 means no unexpected contact (we can grasp)
//	if (grasp::to<Data>(dataPtr)->triggered == 0) {
//		context.debug("RagPlanner::performApproach(): No unexpected contact:. we grasp! (disable guards)\n");
//		pHeuristic->setPointCloudCollision(false);
////		pHeuristic->setCollisionDetection(false);
//		// compute trajectory to the grasp pose with point cloud collision detection OFF
//		seqTrn.clear();
//		robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.end() - 1, seq.end(), seqTrn);
//		robot->createTrajectory(robot->recvState().command, &seqTrn.front(), NULL, 0, trjGrasp);
//		grasp::to<Data>(dataPtr)->action.clear();
//		profile(dataPtr, trjGrasp, SecTmReal(5.0), trjApproachIdle);
//		robot->enableGuards(false);
//		// perform grasping action
//		perform(dataPtr);
//		printTrajectory(grasp::to<Data>(dataPtr)->action, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
//		if (grasp::to<Data>(dataPtr)->triggered = robot->isGrasping(/*grasp::to<Data>(dataPtr)->*/triggeredGuards, robot->recvState().config) > 0) {
//			const int key = waitKey("YN", "Do you accept this grasp? (Y/N)...");
//			grasp::to<Data>(dataPtr)->replanning = key == 'Y'? false : true;
//		}
//	}
//	//else {
//	//	context.debug("RagPlanner::performApproach(): Contacts occurred: restore EMERGENCY MODE for BHAM robot\n");
//	//	// reset guards allowing the bham to move again
//	//	golem::RobotJustin *justin = robot->getRobotJustin();
//	//	if (!justin) {
//	//		Controller::State::Seq seq, out;
//	//		robot->initControlCycle();
//	//		grasp::RobotState::List tmp;
//	//		grasp::RobotState s = robot->recvState(context.getTimer().elapsed());
//	//		s.config = state;
//	//		s.command = state;
//	//		tmp.push_back(s);
//	//		tmp.push_back(s);
//
//	//		extrapolate(makeCommand(tmp), trjApproachExtrapolFac, false, seq);
//	//		// compute profile
//	//		golem::Controller::State::Seq::iterator ptr = out.end(), tmpend;
//	//		const golem::SecTmReal t = out.empty() ? context.getTimer().elapsed() : out.back().t;
//	//		const Controller::State begin(seq.front(), t), end(seq.back(), begin.t + SecTmReal(1.0));
//
//	//		// create trajectory
//	//		tmpend = out.end();
//	//		pProfile->create(seq.begin(), seq.end(), begin, end, out, ptr, tmpend);
//	//		// profile trajectory
//	//		tmpend = out.end();
//	//		pProfile->profile(out, ptr, tmpend);
//
//	//		robot->resumeGuards(out);
//	//	}
//	//}
//
//	// save executed trajectory
//	context.debug("RagPlanner::performApproach(): saving executed trajectory (length=%d)\n", grasp::to<Data>(dataPtr)->action.size());
//	executedTrajectory.clear();
//	executedTrajectory.reserve(grasp::to<Data>(dataPtr)->action.size());
//	for (Controller::State::Seq::const_iterator i = grasp::to<Data>(dataPtr)->action.begin(); i != grasp::to<Data>(dataPtr)->action.end(); ++i)
//		executedTrajectory.push_back(*i);
//
	context.debug("RagPlanner::performApproach(): update and resample belief\n");
	updateAndResample(dataPtr);
//
//	context.write("RagPlanner::performApproach(): return (triggered=%s, replanning=%s)\n", grasp::to<Data>(dataPtr)->triggered > 0 ? "true" : "false", grasp::to<Data>(dataPtr)->replanning ? "true" : "false"); 
//	robot->initControlCycle();
}

void RagPlanner::performWithdraw(Data::Map::iterator dataPtr) {
//	context.debug("RagPlanner::performWithdraw(): build and perform withdraw trajectory (vanilla version withdraw to initial state).\n");
//	// handle the approaching action to generate a sequence of states 
//	Controller::State::Seq seq, tmp, trjWithdraw;
////	robot->initControlCycle();
//
//	// fixed properties of the planner
//	robot->enableGuards(/*false*/);
//	pHeuristic->enableUnc = false;
//	pHeuristic->setPointCloudCollision(false);
//	robot->setCollisionDetection(true);
////	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionWithdraw), trjApproachExtrapolFac, false, seq);
//
//	//robot->resumeGuards();
//	//if (screenCapture) universe.postScreenCaptureFrames(-1);
//	//robot->set(grasp::to<Data>(dataPtr)->actionWithdraw.front());
//	//::Sleep(100);
//	//if (screenCapture) universe.postScreenCaptureFrames(0);	
//
//	// compute the approach traectory in a new frame and its profile
//	robot->createTrajectory(robot->recvState().config, &grasp::to<Data>(dataPtr)->actionWithdraw.front().command, NULL, 0, trjWithdraw);
//	
//	grasp::to<Data>(dataPtr)->action.clear();
//	profile(dataPtr, trjWithdraw, trjApproachDuration, trjApproachIdle);
//
//	// perform manip action
//	perform(dataPtr);
////
////	//grasp::to<Data>(dataPtr)->action.clear();
////	//golem::Controller::State::Seq trjWithdraw;
////	//tmp.push_back(robot->recvState().config);
////	//tmp.insert(tmp.end(), seq.begin(), seq.end());
////	//profile(dataPtr, trjWithdraw, trjApproachDuration, trjApproachIdle);
////	//robot->createTrajectory(robot->recvState().config, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, trjWithdraw);
//////	dataPtr->second.executedTrajectory = initTrajectory;
////	
////
////	if (!testTrajectory(trjWithdraw.begin(), trjWithdraw.end()))
////		return;
////
////	robot->resumeGuards();
////	if (screenCapture) universe.postScreenCaptureFrames(-1);
////	robot->sendTrajectory(trjWithdraw, true);
////	::Sleep(100);
////	if (screenCapture) universe.postScreenCaptureFrames(0);	
////	robot->waitForEnd();
}

void RagPlanner::performManip(Data::Map::iterator dataPtr) {
//	context.write("RagPlanner::performManip(): build and perform manipulative trajectory (vanilla version just lift the object up).\n");
//	// handle the approaching action to generate a sequence of states 
////	robot->initControlCycle();
//	// fixed properties of the planner
//	robot->enableGuards(false);
//	pHeuristic->enableUnc = false;
//	pHeuristic->setPointCloudCollision(false);
////	robot->setCollisionDetection(false);
//
//	Controller::State::Seq seq;
//	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionManip), trjApproachExtrapolFac, false, seq);
//
//		// compute the approach traectory in a new frame and its profile
//	context.debug("RagPlanner::performApproach(): Computing trajectory in a new frame and generating profile...\n");
//	Controller::State::Seq seqTrn, trjManip;
//	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.begin(), seq.end(), seqTrn);
//
//	robot->createTrajectory(robot->recvState().config, &seqTrn.front(), NULL, 0, trjManip);
//
//	// compute the approach traectory in a new frame and its profile
//	grasp::to<Data>(dataPtr)->action.clear();
//	profile(dataPtr, trjManip, trjManipDuration, trjManipIdle);
//
//	perform(dataPtr);
//	//dataPtr->second.action.clear();
//	//queryDataPtr->second.action.clear();
}


void RagPlanner::performSingleGrasp(Data::Map::iterator dataPtr) {
//	if (grasp::to<Data>(queryDataPtr)->queryPoints.empty())
//		throw Message(Message::LEVEL_ERROR, "RagPlanner::performSingleGrasp(): no query is present.");
//
//	context.write("RagPlanner::performSingleGrasp(): build and perform approach trajectory\n");
//	// fixed properties of the planner
//	pHeuristic->enableUnc = false;
//	pHeuristic->setCollisionDetection(Bounds::GROUP_ALL);
//	pHeuristic->setPointCloudCollision(true);
//
//	robot->enableGuards(false);
//	grasp::to<Data>(dataPtr)->replanning = false;
//	robot->setCollisionDetection(true);		
//
//	// handle the approaching action to generate a sequence of states 
//	//context.write("approaching trajetory (list)\n");
//	//printTrajectory(makeCommand(dataPtr->second.approachAction), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());		
//	Controller::State::Seq seq;
//	extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
//	Controller::State::Seq seqTrn, trjApproach, trjGrasp;
//	robot->trnTrajectory(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, seq.begin(), seq.end(), seqTrn);
//
//	// debug: shows pregrasp and grasp poses of the hand
//	context.debug("RagPlanner::performApproach(): render pre-grasp pose.\n");
//	Controller::State pregrasp = seqTrn.front();
//	renderHand(pregrasp, true);
//	//Controller::State grasp = seqTrn.back();
//	//renderHand(grasp);
//
//	robot->createTrajectory(robot->recvState().config, &pregrasp, NULL, 0, trjApproach);
//	// compute the approach traectory in a new frame and its profile
//	grasp::to<Data>(dataPtr)->action.clear();
//	//grasp::to<Data>(poseDataPtr)->action.clear();
//	profile(dataPtr, trjApproach, trjApproachDuration, trjApproachIdle);
//	//// copy the planned trajectory in my trial data
//	//grasp::to<Data>(poseDataPtr)->action.clear();
//	//for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
//	//	poseDataPtr->second.action.push_back(*i);
//
//	// perform action
//	perform(dataPtr);
//
//	::Sleep(2000);
//	robot->initControlCycle();
//	robot->enableGuards(false);
//	// clear states
////		dataPtr->second.robotStates.clear();
//	seq.clear();
//	Controller::State graspPose = robot->recvState().config;
//	// setting only the fingers to grasp pose
//	for (Configspace::Index j = robot->getStateHandInfo().getJoints().begin(); j != robot->getStateHandInfo().getJoints().end(); ++j)
//		graspPose.cpos[j] = grasp::to<Data>(dataPtr)->actionApproach.begin()->command.cpos[j];
//	seq.push_back(robot->recvState().config);
//	seq.push_back(graspPose);
//	// define the profile
//	grasp::to<Data>(dataPtr)->action.clear();
//	profile(dataPtr, seq, trjApproachDuration, trjApproachIdle);
//
//	// move robot
//	robot->sendTrajectory(grasp::to<Data>(dataPtr)->action, true);
//	// repeat every send waypoint until trajectory end
//	for (U32 i = 0; robot->waitForBegin(); ++i) {
//		if (universe.interrupted())
//			throw grasp::Interrupted();
//		if (robot->waitForEnd(0))
//			break;
//
//		// print every 10th robot state
//		if (i%10 == 0)
//			context.write("State #%d\r", i);
//	}
//	::Sleep(100);
//	context.debug("(Single grasp attempt) Performance finished!\n");
}

//void RagPlanner::run(const Demo::Map::value_type& demo) {
//#ifdef _GRASP_CAMERA_DEPTH
//	auto breakPoint = [&]() { if (waitKey(10) == 27) throw Cancel("Demo cancelled"); };
//
//	context.write("================================================================================================================================\nRunning demo %s\n", demo.first.c_str());
//
//	{
//		grasp::ScopeGuard cleanup([&]() {
//			grasp::to<Data>(currentDataPtr)->graspMode = MODE_DISABLED;
//			renderData(currentDataPtr);
//		});
//
//		grasp::Cloud::RawPointSeqVal val;
//		val.first = grasp::to<Data>(currentDataPtr)->labelMap.empty() ? grasp::Cloud::LABEL_OBJECT : grasp::to<Data>(currentDataPtr)->labelMap.rbegin()->first + 1;
//		const grasp::Cloud::RawPointSeqMultiMap::iterator points = grasp::to<Data>(currentDataPtr)->insertPoints(val, demo.first);
//		grasp::Cloud::RawPointSeq prev, next;
//		grasp::Cloud::RawPointSeqQueue rawPointSeqQueue(cloudDesc.filterDesc.window);
//
//		// capture depth image
//		for (Demo::ObjectDetection::Seq::const_iterator j = demo.second.objectDetectionSeq.begin(); j != demo.second.objectDetectionSeq.end(); ++j) {
//			// go to object detection pose
//			gotoPose(j->scanPose);
//			breakPoint();
//			grasp::RobotPose robotPose;
//			getPose(robotPose);
//
//			// find camera
//			grasp::Recorder* recorder = nullptr;
//			grasp::DepthCamera* camera = nullptr;
//			for (grasp::Recorder::Seq::iterator i = cameraSeq.begin(); i != cameraSeq.end(); ++i)
//				if ((*i)->getCamera()->getName() == j->cameraName) {
//				recorder = i->get();
//				camera = recorder->getDepthCamera();
//				break;
//				}
//			if (!camera)
//				throw Message(Message::LEVEL_ERROR, "ShapePlanner::run(): unknown camera %s", j->cameraName.c_str());
//			// set camera frame
//			robotPose.multiply(camera->getCalibration()->getDeformation(robotPose), robotPose);
//			Mat34 cameraFrame = camera->getCalibration()->getParameters().pose;
//			if (recorder->isHand()) cameraFrame.multiply(robotPose, cameraFrame);
//			camera->setFrame(cameraFrame);
//			// Object region in global coordinates
//			golem::Bounds::Seq objectRegion;
//			for (golem::Bounds::Desc::Seq::const_iterator i = cloudDesc.objectRegionDesc.begin(); i != cloudDesc.objectRegionDesc.end(); ++i)
//				objectRegion.push_back((*i)->create());
//			// Move bounds to camera frame
//			Mat34 cameraFrameInv;
//			cameraFrameInv.setInverse(cameraFrame);
//			for (golem::Bounds::Seq::const_iterator i = objectRegion.begin(); i != objectRegion.end(); ++i)
//				(*i)->multiplyPose(cameraFrameInv, (*i)->getPose());
//			// start camera
//			grasp::ScopeGuard guardCamera([&]() { camera->set(grasp::Camera::CMD_STOP); });
//			camera->setProperty(camera->getDepthProperty());
//			camera->set(grasp::Camera::CMD_VIDEO);
//			// detect changes
//			grasp::Image::Ptr image;
//			rawPointSeqQueue.clear();
//			prev.clear();
//			U32 prevSize = 0;
//			for (bool accept = false; !accept;) {
//				breakPoint();
//				// grab image
//				camera->peek_back(image);
//				grasp::Image::assertImageData(image->dataDepth);
//				try {
//					grasp::Cloud::copy(objectRegion, false, *image->dataDepth, next);
//				}
//				catch (const Message&) {}
//				grasp::Cloud::transform(cameraFrame, next, next);
//				grasp::Cloud::setSensorFrame(cameraFrame, next);
//				rawPointSeqQueue.push_back(next);
//				if (!rawPointSeqQueue.full())
//					continue;
//				grasp::Cloud::filter(context, cloudDesc.filterDesc, rawPointSeqQueue, next);
//				// render points
//				{
//					golem::CriticalSectionWrapper csw(csDataRenderer);
//					pointRenderer.reset();
//					grasp::to<Data>(currentDataPtr)->draw(next, pointRenderer);
//				}
//				// count points
//				U32 nextSize = 0;
//				for (grasp::Cloud::RawPointSeq::const_iterator i = next.begin(); i != next.end(); ++i)
//					if (!grasp::Cloud::isNanXYZ(*i))
//						++nextSize;
//				if (nextSize < j->minSize)
//					continue;
//				//context.debug("ShapePlanner::run(): object detected (size=%u)\n", nextSize);
//				if (prev.size() == next.size()) {
//					size_t sharedSize = 0;
//					Real deltaDepth = REAL_ZERO;
//					for (size_t i = 0; i < next.size(); ++i) {
//						const grasp::Cloud::RawPoint p = prev[i], n = next[i];
//						if (grasp::Cloud::isNanXYZ(p) || grasp::Cloud::isNanXYZ(n))
//							continue;
//						++sharedSize;
//						deltaDepth += Math::sqrt(Math::sqr(p.x - n.x) + Math::sqr(p.y - n.y) + Math::sqr(p.z - n.z));
//					}
//					if (nextSize - sharedSize < j->deltaSize && deltaDepth / sharedSize < j->deltaDepth) {
//						context.write("Object detected (size=%u, delta_size=%u, delta_depth=%f)\n", nextSize, nextSize - sharedSize, deltaDepth / sharedSize);
//						grasp::Cloud::nanRem(context, next, grasp::Cloud::isNanXYZ<grasp::Cloud::RawPoint>); // required by flann in linux
//						grasp::Cloud::normal(context, cloudDesc.normal, next, next);
//						grasp::Cloud::nanRem(context, next, grasp::Cloud::isNanXYZNormal<grasp::Cloud::RawPoint>);
//						points->second.insert(points->second.end(), next.begin(), next.end());
//						accept = true;
//						// render
//						renderData(currentDataPtr);
//					}
//				}
//				if (!accept) {
//					prev = next;
//					prevSize = nextSize;
//				}
//			}
//		}
//
//		breakPoint();
//
//		// find approach trajectory
//		golem::Real err = REAL_MAX;
//		golem::Controller::State::Seq approach;
//		if (demo.second.poseEstimation) {
//			// load model
//			grasp::Cloud::RawPointSeq modelPoints;
//			Data::pcdRead(demo.second.modelObject, demo.second.modelObject, modelPoints);
//			pRBPose->createModel(modelPoints);
//			// estimate pose
//			breakPoint();
//			pRBPose->createQuery(points->second);
//			const Mat34 queryTransform = pRBPose->maximum().toMat34();
//			grasp::Cloud::transform(queryTransform, modelPoints, modelPoints);
//			points->second.insert(points->second.end(), modelPoints.begin(), modelPoints.end());
//			// render
//			renderData(currentDataPtr);
//			// find feasible trajectory
//			for (grasp::StringSeq::const_iterator i = demo.second.modelTrajectories.begin(); i != demo.second.modelTrajectories.end() && err > REAL_ONE; ++i) {
//				breakPoint();
//				grasp::RobotState::Seq raw;
//				golem::FileReadStream frs(i->c_str());
//				frs.read(raw, raw.end(), grasp::RobotState(*robot->getController()));
//				golem::Controller::State::Seq inp = grasp::RobotState::makeCommand(raw), seq;
//				// choose first with acceptable error
//				approach.clear();
//				const grasp::RBDist dist = robot->transformTrajectory(queryTransform, seq.begin(), seq.end(), approach);
//				err = manipulator->getDesc().trajectoryErr.dot(dist);
//				context.write("Trajectory error: lin=%.9f, ang=%.9f, total=%.9f\n", dist.lin, dist.ang, err);
//			}
//		}
//		else {
//			grasp::Classifier::Desc::Map::const_iterator ptr = classifierDescMap.find(demo.second.classifierName);
//			if (ptr == classifierDescMap.end())
//				throw Message(Message::LEVEL_ERROR, "ShapePlanner::run(): unknown classifier %s", demo.second.classifierName.c_str());
//			classifier = load(*ptr->second);
//			// estimate curvatures
//			grasp::Cloud::PointSeq features;
//			grasp::Cloud::copy(Bounds::Seq(), false, points->second, features);
//			grasp::Cloud::curvature(context, cloudDesc.curvature, features);
//			grasp::Cloud::nanRem(context, features, grasp::Cloud::isNanXYZNormalCurvature<grasp::Cloud::Point>);
//			// estimate grasp
//			classifier->find(features, grasp::to<Data>(currentDataPtr)->graspConfigs);
//			grasp::Cluster::findLikelihood(clusterDesc, grasp::to<Data>(currentDataPtr)->graspConfigs, grasp::to<Data>(currentDataPtr)->graspClusters);
//			// display
//			grasp::to<Data>(currentDataPtr)->resetDataPointers();
//			grasp::to<Data>(currentDataPtr)->graspMode = MODE_CONFIG;
//			// trajectory
//			CollisionBounds collisionBounds(*this, true);
//			golem::U32 size = std::min(grasp::to<Data>(currentDataPtr)->getGraspConfigSize() - grasp::to<Data>(currentDataPtr)->graspConfigPtr, manipulator->getDesc().trajectoryClusterSize), j = grasp::to<Data>(currentDataPtr)->graspConfigPtr;
//			const std::pair<grasp::Grasp::Config::Seq::const_iterator, grasp::RBDistEx> val = manipulator->find<grasp::Grasp::Config::Seq::const_iterator>(grasp::to<Data>(currentDataPtr)->getGraspConfigBegin(), grasp::to<Data>(currentDataPtr)->getGraspConfigBegin() + size, [&](const grasp::Grasp::Config::Ptr& config) -> grasp::RBDistEx {
//				breakPoint();
//				grasp::to<Data>(currentDataPtr)->graspConfigPtr = j++;
//				renderData(currentDataPtr);
//				const grasp::RBDist dist(manipulator->find(config->path));
//				const grasp::RBDistEx distex(dist, manipulator->getDesc().trajectoryErr.collision && collisionBounds.collides(manipulator->getConfig(config->path.getApproach())));
//				err = manipulator->getDesc().trajectoryErr.dot(dist);
//				context.write("#%03u: Trajectory error: lin=%.9f, ang=%.9f, collision=%s, total=%f\n", j, distex.lin, distex.ang, distex.collision ? "yes" : "no", err);
//				return distex;
//			});
//			manipulator->copy((*val.first)->path, approach);
//			grasp::to<Data>(currentDataPtr)->graspConfigPtr = val.first - grasp::to<Data>(currentDataPtr)->getGraspConfigBegin();
//			context.write("#%03u: Best trajectory\n", grasp::to<Data>(currentDataPtr)->graspConfigPtr + 1);
//			renderData(currentDataPtr);
//		}
//		if (err > REAL_ONE) {
//			context.write("No feasible trajectory has been found\n");
//			return;
//		}
//		grasp::to<Data>(currentDataPtr)->trajectory[demo.first] = grasp::RobotState::make(manipulator->getController(), approach); // update trajectory collection
//
//		// overwrite not used joints
//		const grasp::RealSeq& cpos = demo.second.objectDetectionSeq.back().scanPose.c;
//		for (golem::Controller::State::Seq::iterator i = approach.begin(); i != approach.end(); ++i)
//			i->cpos.set(cpos.data() + manipulator->getJoints(), cpos.data() + cpos.size(), manipulator->getJoints());
//
//		// find manipulation trajectory
//		golem::Controller::State::Seq trajectory;
//		createWithManipulation(approach, trajectory, true);
//		breakPoint();
//
//		// collision bounds
//		CollisionBounds collisionBounds(*this, points->second);
//
//		// perform
//		try {
//			perform(demo.first, trajectory, true);
//		}
//		catch (const Message& msg) {
//			context.write("%s\n", msg.str().c_str());
//			return;
//		}
//		breakPoint();
//
//		// cleanup
//	}
//
//	// goto remaining poses
//	for (Robot::Pose::Seq::const_iterator i = demo.second.poses.begin(); i != demo.second.poses.end(); ++i) {
//		gotoPose(*i);
//		breakPoint();
//	}
//
//#endif
//}

//------------------------------------------------------------------------------

golem::Controller::State RagPlanner::lookupState(golem::SecTmReal t) const {
	Controller::State state = robot->getController()->createState();
	robot->getController()->lookupState(t, state);
	return state;
}

golem::Controller::State RagPlanner::lookupCommand(golem::SecTmReal t) const {
	Controller::State state = robot->getController()->createState();
	robot->getController()->lookupCommand(t, state);
	return state;
}


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
	force.assign(robot->getStateHandInfo().getJoints().size(), REAL_ZERO);
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
	golem::Waypoint w(*robot->getController(), robot->recvState().config.cpos/*grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos*/);
	context.write("update weights, triggered guards = %u\n", triggeredGuards.size());
	//pBelief->createUpdate(manipulator.get(), robot, w, /*grasp::to<Data>(dataPtr)->*/triggeredGuards, force);
	//for (auto i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i)
	//	context.write("%s\n", (*i).str().c_str());
	pBelief->createUpdate(collisionPtr, w, triggeredGuards, trialPtr != trialDataMap.end() ? grasp::to<TrialData>(trialPtr)->queryPointsTrn : grasp::RBCoord());
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
	if (screenCapture) universe.postScreenCaptureFrames(-1);
	renderData(dataPtr);
	//::Sleep(100);
	if (screenCapture) universe.postScreenCaptureFrames(0);

	//::Sleep(100);
	//if (screenCapture) universe.postScreenCaptureFrames(0);	
//	std::cout << "spam:update&resample 15\n";
}

//------------------------------------------------------------------------------

bool RagPlanner::execute(Data::Map::iterator dataPtr, golem::Controller::State::Seq& trajectory) {
//	Controller::State::Seq tmp,approach, out;
//	Bounds::Seq bounds;
//	pHeuristic->enableUnc = false;
//	pHeuristic->setPointCloudCollision(false);
//	robot->setCollisionDetection(/*waitKey("YN", "Collision detection (Y/N)...") == 'Y' ? true : */false);
//	for (Controller::State::Seq::const_iterator i = trajectory.begin(); i != trajectory.end(); ++i) {
//		bounds = manipulator->getBounds(manipulator->getConfig(*i), manipulator->getPose(*i).toMat34());
//		renderHand(*i, bounds, false);
//	}
//	robot->transformTrajectory(grasp::to<Data>(dataPtr)->queryTransform, trajectory.begin(), trajectory.end(), tmp);
////	robot->trnTrajectory(Mat34::identity(), modelFrame, grasp::to<Data>(dataPtr)->queryTransform, trajectory.begin(), trajectory.end(), tmp);
//	for (Controller::State::Seq::iterator i = ++tmp.begin(); i != tmp.end(); ++i) approach.push_back(*i);
//	bounds = manipulator->getBounds(manipulator->getConfig(tmp.at(0)), manipulator->getPose(tmp.at(0)).toMat34());
//	renderHand(tmp.at(0), bounds, false);
//	for (Controller::State::Seq::iterator i = approach.begin(); i != approach.end(); ++i) {
//		bounds = manipulator->getBounds(manipulator->getConfig(*i), manipulator->getPose(*i).toMat34());
//		renderHand(*i, bounds, false);
//	}
//	pHeuristic->enableUnc = this->uncEnable;
//	pHeuristic->setPointCloudCollision(waitKey("YN", "Point Cloud Collision detection (Y/N)...") == 'Y' ? true : false);
//	robot->findTrajectory(robot->recvState().command, &tmp.at(0), nullptr, /*&(*grasp::to<Data>(currentDataPtr)->getGraspConfigBegin())->path.front().toMat34(),*/ 0, approach);
//	if (!testTrajectory(approach.begin(), approach.end()))
//		return;
//	//for (Controller::State::Seq::const_iterator i = ++tmp.cbegin(); i != tmp.cend(); ++i) approach.push_back(*i);
//	profile(this->trjDuration, approach, out);
//	// perform
//	try {
//		perform(grasp::to<Data>(dataPtr)->getName(), out);
//	}
//	catch (const Message& msg) {
//		context.write("%s\n", msg.str().c_str());
//		return;
//	}
//
//	return;
	const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
	bool silent = grasp::to<Data>(dataPtr)->actionType != action::NONE_ACTION;
//	context.debug("execute(): silen=%s, actionType=%s\n", silent ? "TRUE" : "FALSE", actionToString(grasp::to<Data>(dataPtr)->actionType));
	const int key = !silent ? waitKey("MQTGUF", "Press to (M)odel based or (Q)uery based grasp, (T)rajectory based planner, (G)rasp, (U)p lifting, (F)ind target") :
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
	grasp::RealSeq vv;
	vv.assign(20, REAL_ZERO);
	vv[0] = Real(0.0432042); vv[1] = Real(0.25618); vv[2] = Real(0.116704); vv[3] = Real(0.116704);
	vv[12] = Real(0.0); vv[13] = Real(1.2); vv[14] = Real(1.0); vv[15] = Real(1.0);
	vv[16] = Real(0.0); vv[17] = Real(1.2); vv[18] = Real(1.0); vv[19] = Real(1.0);

//	collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
//	objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
//	PosePlanner::TrialData::Iteration::Ptr iteration;
//	iteration.reset(new PosePlanner::TrialData::Iteration("", pBelief));

	switch (key){
	case 'M':
	{
		context.debug("Plan from Model to Query\n");
//		pHeuristic->testCollision = true;
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(true);
		isGrasping = false;
		//pHeuristic->setPointCloudCollision(false);
		// pre-grasp pose w.r.t. model frame
//		Controller::State cend = trajectory[1];

		Controller::State cend = lookupState(); // robot->getController()->createState();
		//robot->getHandCtrl()->getController()->setToDefault(cend);
		//for (auto i = robot->getStateArmInfo().getJoints().begin(); i != robot->getStateArmInfo().getJoints().end(); ++i)
		//	cend.cpos[i] = trajectory[1].cpos[i];
		//for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		//	cend.cpos[i] = trajectory[1].cpos[i];

		// transform w.r.t. query frame
		try {
			robot->findTarget(grasp::to<Data>(dataPtr)->queryTransform, trajectory[1], cend);
		}
		catch (const Message &msg)
		{
			context.write("%s\n", msg.str().c_str());
			return false;
		}
		auto i = robot->getStateHandInfo().getChains().begin();
		for (auto j = robot->getStateHandInfo().getJoints(i).begin(); j != robot->getStateHandInfo().getJoints(i).end(); ++j) {
			const size_t k = j - robot->getStateHandInfo().getJoints().begin();
			cend.cpos[j] = vv[k];
		}
		i++; i++; i++;//move to the ring finger
		for (; i != robot->getStateHandInfo().getChains().end(); ++i) {
			for (auto j = robot->getStateHandInfo().getJoints(i).begin(); j != robot->getStateHandInfo().getJoints(i).end(); ++j) {
				const size_t k = j - robot->getStateHandInfo().getJoints().begin();
				cend.cpos[j] = vv[k];
			}
		}


		pHeuristic->setPointCloudCollision(true);
//		printState(cend, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end(), "Target", true);
		//Waypoint pregrasp;
		//pregrasp.cpos = cend.cpos;
		//pregrasp.setup(*robot->getController(), false, true);
		Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		renderHand(cend, bounds, true);
		//pHeuristic->testCollision = true;
		//context.debug("Check for collision (points=%d)\n", pHeuristic->getNumCollisionPoints());
		//pHeuristic->collides(pregrasp);
		Controller::State::Seq approach;
		try {
			robot->findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		}
		catch (const Message &msg)
		{
			return false;
		}
//		Bounds::Seq bounds;
//		//Collision collision(context, *manipulator);
//		//Collision::Waypoint waypoint; 
//		for (Controller::State::Seq::iterator i = approach.begin(); i != approach.end(); ++i) {
//			bounds = manipulator->getBounds(manipulator->getConfig(*i), manipulator->getPose(*i).toMat34());
//			//(void)collision.evaluate(waypoint, (*pBelief->getHypotheses().begin())->getCloud(), rand, manipulator->getPose(*i), true);
////			printState(*i, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end(), "Approach", true);
//			renderHand(*i, bounds, false);
//		}		
//		iteration->setTrajectory(/*grasp::to<Data>(dataPtr)->actionType, *robot->getController(),*/ approach);
//		grasp::to<TrialData>(trialPtr)->add(iteration);
		if (!silent)
			if (!testTrajectory(approach.begin(), approach.end()))
				return false;
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);
		// perform
		try {
			perform(grasp::to<Data>(dataPtr)->getName(), out);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			return false;
		}
//		printf("Finished perform\n");
		resetPlanning();
		return true;
	}
	case 'Q':
	{
		context.debug("Plan to Query\n");
//		pHeuristic->testCollision = true;
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		//pHeuristic->setPointCloudCollision(false);
		// pre-grasp pose w.r.t. query frame
		Controller::State cend = trajectory[1];
		//robot->findTarget(grasp::to<Data>(dataPtr)->queryTransform, trajectory[1], cend);
		//Waypoint pregrasp;
		//pregrasp.cpos = cend.cpos;
		//pregrasp.setup(*robot->getController(), false, true);
		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		//renderHand(cend, bounds, false);
		pHeuristic->setPointCloudCollision(true);
		//pHeuristic->testCollision = true;
		//context.debug("Check for collision (points=%d)\n", pHeuristic->getNumCollisionPoints());
		//pHeuristic->collides(pregrasp);
		Controller::State::Seq approach;
		robot->findTrajectory(lookupState(), &cend, nullptr, 0, approach);
//		iteration->setTrajectory(/*grasp::to<Data>(dataPtr)->actionType, *robot->getController(),*/ approach);
//		grasp::to<TrialData>(trialPtr)->add(iteration);
		if (!silent && !testTrajectory(approach.begin(), approach.end()))
			return false;
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);
		// perform
		try {
			perform(grasp::to<Data>(dataPtr)->getName(), out, silent);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			return false;
		}
		resetPlanning();

		return true;
	}
	case 'T':
	{
		context.debug("Plan trajectory optimisation\n");
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(false);
		isGrasping = false;
		// select index 
		U32 index = 1;
		selectIndex(trajectory, index, "waypoint");
		index -= 1;

		Controller::State cend = lookupState(); //robot->getController()->createState();
		//robot->getHandCtrl()->getController()->setToDefault(cend);
		cend.cpos = trajectory[index].cpos;

		// transform w.r.t. query frame
		try {
			robot->findTarget(grasp::to<Data>(dataPtr)->queryTransform, trajectory[index], cend);
		}
		catch (const Message &msg)
		{
			context.write("%s\n", msg.str().c_str());
			return false;
		}
		//for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		//	cend.cpos[i] = trajectory[index].cpos[i];

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		//renderHand(cend, bounds, true);
		Controller::State::Seq approach;
		try {
			robot->findTrajectory(lookupState(), &cend, nullptr, 0, approach);
		}
		catch (const Message &msg)
		{
			return false;
		}

		if (!silent)
			if (!testTrajectory(approach.begin(), approach.end()))
				return false;
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);
		pHeuristic->setPointCloudCollision(true);
		// perform
		try {
			perform(grasp::to<Data>(dataPtr)->getName(), out);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			return false;
		}
		resetPlanning();

////		pHeuristic->testCollision = true;
//		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
//		pHeuristic->setPointCloudCollision(true);
//		Controller::State::Seq approach, trj;
//		for (auto i = trajectory.rbegin(); i != --trajectory.rend(); ++i)
//			trj.push_back(*i);
//		robot->findTrnTrajectory(grasp::to<Data>(dataPtr)->queryTransform, robot->recvState().command, trj.begin(), trj.end(), approach);
////		iteration->setTrajectory(/*grasp::to<Data>(dataPtr)->actionType, *robot->getController(),*/ approach);
////		grasp::to<TrialData>(trialPtr)->add(iteration);
//		if (!silent)
//			if (!testTrajectory(approach.begin(), approach.end()))
//				return false;
//		Controller::State::Seq out;
//		profile(this->trjDuration, approach, out, silent);
//		// perform
//		try {
//			perform(grasp::to<Data>(dataPtr)->getName(), out, silent);
//		}
//		catch (const Message& msg) {
//			context.write("%s\n", msg.str().c_str());
//			return false;
//		}
//		resetPlanning();
		return true;
	}
	case 'G':
	{
//		printf("plan grasp\n");
		context.debug("Plan grasping\n");
//		enableForceReading = false;
////		objectPointCloudPtr.reset();
////		pHeuristic->testCollision = false;
//		pHeuristic->enableUnc = false;
//		pHeuristic->setPointCloudCollision(false);
		resetPlanning();
		isGrasping = true;

		//Controller::State pregrasp = robot->getController()->createState();
//		printf("recovering state..\n");
//		Controller::State pregrasp = robot->recvState().command;
//		printf("done\n");
//		Controller::State target = robot->recvState().command; 
		//Controller::State s = lookupCommand();// trajectory[0];
		// close fingers
		//for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		//	auto j = robot->getStateHandInfo().getJoints(robot->getStateHandInfo().getChains().begin()).begin() + 1;
		//	target.cpos[j] = 0.12;
		//}
//		robot->findTarget(Mat34::identity(), target, cend);

		// current configuration (fingers opened)
		Controller::State cstart = lookupState();// robot->getController()->createState();
		//robot->getHandCtrl()->getController()->setToDefault(cstart);
		//cstart.cpos = s.cpos;

		// select index 
		U32 index = 1;
		//selectIndex(trajectory, index, "waypoint");
		index -= 1;

		// grasp configuration (fingers closed)
		Controller::State cend = lookupState();/*robot->getController()->createState();
		robot->getHandCtrl()->getController()->setToDefault(cend);
		for (auto i = robot->getStateArmInfo().getJoints().begin(); i != robot->getStateArmInfo().getJoints().end(); ++i)
			cend.cpos[i] = s.cpos[i];*/
		for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
			cend.cpos[i] = trajectory[index].cpos[i];

		// visual grasp pose (fingers closed)
		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		//renderHand(cend, bounds, true);
		//for (auto i = robot->getStateArmInfo().getJoints().begin(); i != robot->getStateArmInfo().getJoints().end(); ++i)
		//	target.cpos[i] = pregrasp.cpos[i];
		// close the fingers for the pregrasp
		//for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		for (auto j = robot->getStateHandInfo().getJoints().begin() + 1; j != robot->getStateHandInfo().getJoints().end(); ++j) {
			const size_t k = j - robot->getStateHandInfo().getJoints().begin();
			cend.cpos[j] = vl[k];
		}
		//}
		Controller::State::Seq approach;
		approach.push_back(cstart);
		approach.push_back(cend);
//		robot->findTrajectory(robot->recvState().command, &cend, nullptr, 0, approach);

		//approach.push_back(pregrasp);
		//approach.push_back(target);
//		robot->findTrajectory(robot->recvState().command, &cend, nullptr, 0, approach);
//		iteration->setTrajectory(grasp::to<Data>(dataPtr)->actionType, *robot->getController(), approach);
//		grasp::to<TrialData>(trialPtr)->add(iteration);
//		grasp::to<TrialData>(trialPtr)->grasped = true;
		if (!silent)
			if (!testTrajectory(approach.begin(), approach.end()))
				return false;
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);

		// perform
		try {
			perform(grasp::to<Data>(dataPtr)->getName(), out);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			return false;
		}

		// print distance to the targt configuration
		Controller::State cfg = lookupState();
		std::stringstream ss;
		for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i) {
			const size_t k = i - robot->getStateHandInfo().getJoints().begin();
			ss << "c=" << k << " [" << cend.cpos[i] - cfg.cpos[i] << "]/t";
		}
		context.write("Hand joints error:\n%s\n", ss.str().c_str());
//		// perform
//		//robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
//		//robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
//
//		robot->sendTrajectory(out);
////		robot->getArmCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
////		robot->getHandCtrl()->setMode(grasp::ActiveCtrl::MODE_ENABLED);
//		// repeat every send waypoint until trajectory end
//		robotStates.clear();
//		if (screenCapture) universe.postScreenCaptureFrames(-1);
//		for (U32 i = 0; robot->waitForBegin(); ++i) {
//			if (universe.interrupted())
//				throw grasp::Interrupted();
//			if (robot->waitForEnd(0))
//				break;
//
//			// print every 10th robot state
//			if (i % 10 == 0) {
//				context.write("State #%d\r", i);
//			}
//			robotStates.push_back(robot->recvState());
////			printState(s.command, robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());
//		}
//		if (screenCapture) universe.postScreenCaptureFrames(0);
//		// insert the executed trajectory to the trial
//		if (trialPtr != trialDataMap.end())
//			(void)grasp::to<TrialData>(trialPtr)->insert(robotStates);
//
//		// enable force reading to check for collision with the object to be grasped
//		//enableForceReading = true;
//
//		//try {
//		//	perform(grasp::to<Data>(dataPtr)->getName(), out, silent);
//		//}
//		//catch (const Message& msg) {
//		//	context.write("%s\n", msg.str().c_str());
//		//	return;
//		//}
//
		resetPlanning();
		return true;
	}
	case 'U':
	{
		context.debug("Plan lifting up\n");
		enableForceReading = false;
		//		objectPointCloudPtr.reset();
//		pHeuristic->testCollision = false;
		pHeuristic->enableUnc = false;
		pHeuristic->setPointCloudCollision(false);
		isGrasping = true;

		Controller::State target = lookupState();//robot->recvState().command;
		//for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		//	target.cpos[i] = trajectory[0].cpos[i];
		//		Controller::State cend = trajectory[1];
		// transform w.r.t. query frame 
		robot->findTarget(Mat34::identity(), lookupState(), target, true);
		//for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		//	target.cpos[i] += 0.1;

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(target), manipulator->getPose(target).toMat34());
		//renderHand(target, bounds, true);

		for (auto j = robot->getStateHandInfo().getJoints().begin() + 1; j != robot->getStateHandInfo().getJoints().end(); ++j) {
			const size_t k = j - robot->getStateHandInfo().getJoints().begin();
			target.cpos[j] = vl[k];
		}
		//for (auto i = robot->getStateArmInfo().getJoints().begin(); i != robot->getStateArmInfo().getJoints().end(); ++i)
		//	target.cpos[i] = pregrasp.cpos[i];
		//for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
		//	target.cpos[i] += 0.1;
		Controller::State::Seq approach;
		robot->findTrajectory(lookupState(), &target, nullptr, 0, approach);
		//approach.push_back(pregrasp);
		//approach.push_back(target);
		//		robot->findTrajectory(robot->recvState().command, &cend, nullptr, 0, approach);
		//		iteration->setTrajectory(grasp::to<Data>(dataPtr)->actionType, *robot->getController(), approach);
		//		grasp::to<TrialData>(trialPtr)->add(iteration);
		//		grasp::to<TrialData>(trialPtr)->grasped = true;
		if (!silent)
			if (!testTrajectory(approach.begin(), approach.end()))
				return false;
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out, silent);

		// perform
		try {
			perform(grasp::to<Data>(dataPtr)->getName(), out);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			return false;
		}

		resetPlanning();
		return true;
	}
	case 'F':
	{	
		objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
		// show the pre grasp config
		Bounds::Seq bounds;// = manipulator->getBounds(manipulator->getConfig(trajectory[1]), manipulator->getPose(trajectory[1]).toMat34());
//		renderHand(trajectory.at(1), bounds, false);
		// check for collisions
		robot->setCollisionDetection(true);
		pHeuristic->enableUnc = true;
		pHeuristic->setPointCloudCollision(waitKey("YN", "Point Cloud Collision detection (Y/N)...") == 'Y' ? true : false);
		Controller::State cend = trajectory[1];
		Waypoint pregrasp;
		pregrasp.cpos = cend.cpos;
		pregrasp.setup(*robot->getController(), false, true);
		context.debug("Check for the pregrasp for collisions\n");
//		pHeuristic->testCollision = true;
		if (pHeuristic->collides(pregrasp)) {
			// find new target
			context.write("find a new free-collision pregrasp\n");
			cend = trajectory[1];
			pHeuristic->testCollision = false;
			robot->findTarget(Mat34::identity(), trajectory[1], cend);
			pregrasp.cpos = cend.cpos;
			pHeuristic->testCollision = true;
			context.write("check the new pregrasp for collisions\n");
			pHeuristic->collides(pregrasp);
		}
		pHeuristic->testCollision = false;
		context.debug("Render target configuration\n");
		bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getPose(cend).toMat34());
		renderHand(cend, bounds, false);
		Controller::State::Seq approach;
		//if (waitKey("YN", "Add waypoint generators? (Y/N)...") == 'Y')
		//	for (U32 i = 3; i > 0; --i)
		//		approach.push_back(trajectory[i]);

		context.debug("Find trajectory\n");
		robot->findTrajectory(robot->recvState().command, &cend, nullptr/*&robot->forwardTransform(trajectory.at(1))*/, /*&(*grasp::to<Data>(currentDataPtr)->getGraspConfigBegin())->path.front().toMat34(),*/ 0, approach);
		context.debug("done\n");
		if (!testTrajectory(approach.begin(), approach.end()))
			return false;
		Controller::State::Seq out;
		profile(this->trjDuration, approach, out);
		// perform
		try {
			perform(grasp::to<Data>(dataPtr)->getName(), out);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			return false;
		}
		return true;
	}
	}
	return false;
}

//------------------------------------------------------------------------------

void RagPlanner::function(Data::Map::iterator& dataPtr, int key) {
	switch (key) {
	case 'Z':
	{
		FTGuard::Seq triggerredGuards;
		for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i) {
			FTGuard g(*manipulator);
			g.create(i);
			(void)g.str();
			triggerredGuards.push_back(g);
		}
		context.write("Guards:\n");
		for (FTGuard::Seq::const_iterator g = triggeredGuards.begin(); g != triggeredGuards.end(); ++g)
			context.write("%s\n", g->str().c_str());
		context.write("done.\n");
		return;
		context.write("Load object ground truth pose\n");
		grasp::Cloud::RawPointSeqMultiMap::const_iterator points;
		try {
			if (waitKey("YN", "Do you want to move the point cloud? (Y/N)...") == 'Y')
				points = getTrnRawPoints(dataPtr, grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34());
			//else
			//	points = getPoints(dataPtr);

			//objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(points->second));
			renderData(dataPtr);
		}
		catch (const Message &msg) { context.notice("%s\n", msg.what()); }
		return;
	}
	case 'B':
	{
		TrialData::Ptr tdata = createTrialData();
		tdata->setup(context, rand);
		switch (waitKey("NSLRIP", "Press a key to create (N)ew/(S)ave/(L)oad/(R)emove/(I)mport/(P)rocess data...")) {
		case 'S':
		{
			//// save data
			//const bool allData = getData().size() > 1 && waitKey("YN", "Save all data (Y/N)...") == 'Y';
			//if (allData) {
			//	for (Data::Map::iterator i = getData().begin(); i != getData().end(); ++i) {
			//		context.write("Saving %s...\n", i->first.c_str());
			//		i->second->save();
			//	}
			//}
			//else {
				const std::string path = tdata->path;
				grasp::ScopeGuard guard([&]() {tdata->path = path; });
				readPath("Enter data path to save: ", tdata->path, tdata->extTrial.c_str());
				tdata->path = grasp::makeString("%s%s", tdata->getName().c_str(), tdata->extTrial.c_str());
				tdata->save();
			//}
			context.write("Done!\n");
			return;
		}
		}
	}
//	case 'R':
//	{
//		if (poseDataPtr._Ptr == NULL) {
//			context.write("Unable to reset.\n");
//			return;
//		}
//		pBelief->reset();
//		//poseDataPtr->second.samples.clear();
//		//for (grasp::RBPose::Sample::Seq::const_iterator i = poseDataPtr->second.init.begin(); i != poseDataPtr->second.init.end(); ++i)
//		//	poseDataPtr->second.samples.push_back(*i);
//		showSampleColour = false;
//		showSampleFrame = false;
//		showSamplePoints = true;
//		showMLEFrame = true;
//		showMLEPoints = false;
////		pHeuristic->setBeliefState(poseDataPtr->second.samples, modelFrame);
////		pHeuristic->setBelief(pBelief);
//		pHeuristic->enableUnc = uncEnable;	
//		grasp::to<Data>(dataPtr)->actionFrame = initActionFrame;
//		mlFrame = grasp::RBPose::Sample(grasp::to<Data>(dataPtr)->actionFrame);
//		grasp::to<Data>(dataPtr)->queryFrame.multiply(grasp::to<Data>(dataPtr)->actionFrame, modelFrame);	
//		grasp::Cloud::transform(grasp::to<Data>(dataPtr)->actionFrame, grasp::to<Data>(dataPtr)->queryPoints, grasp::to<Data>(dataPtr)->queryPoints);
//		showModelPoints = false;
//		showModelFeatures = false;
//		showQueryDistrib = true;
//		showDistribution = false;
//		renderData(dataPtr);
//		renderTrialData(poseDataPtr);
//		return;
//	}
//	case 'B':
//	{
//		context.debug("testing active controller.\n");
//		robot->enableGuards();
//		Controller::State::Seq trj;
//		robot->createTrajectory(robot->recvState().config, NULL, &poseSeq[2], 0, trj);
//		
//		grasp::to<Data>(dataPtr)->action.clear();
//		//grasp::to<Data>(poseDataPtr)->action.clear();
//		profile(dataPtr, trj, trjApproachDuration, trjApproachIdle);
//		perform(dataPtr);
//		// retrieve robot pose from real justin
//		// print current robot joint position
//		//Controller::State state = robot->recvState().config;
//		//std::stringstream str;
//		//for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
//		//	str << " c" << (*i - *state.getInfo().getJoints().begin() + 1) << "=\"" << state.cpos[i] << "\"";
//		//context.write("<pose dim=\"%d\"%s/>\n", state.getInfo().getJoints().size(), str.str().c_str());
//		//golem::WorkspaceJointCoord wc;
//		//robot->getController()->jointForwardTransform(state.cpos, wc);
//		//for (golem::Chainspace::Index i = robot->getStateInfo().getChains().begin(); i < robot->getStateInfo().getChains().end(); ++i) {
//		//	context.write("Name: %s\n", robot->getController()->getChains()[i]->getName().c_str());
//		//	for (golem::Configspace::Index j = robot->getStateInfo().getJoints(i).begin(); j < robot->getStateInfo().getJoints(i).end(); ++j) {
//		//		const U32 k = U32(j - robot->getStateInfo().getJoints(i).begin());
//		//		const Mat34& m = wc[j];
//		//		context.write("Joint %d: p={(%f, %f, %f)}, R={(%f, %f, %f), (%f, %f, %f), (%f, %f, %f)}\n", k, m.p.x, m.p.y, m.p.z, m.R.m11, m.R.m12, m.R.m13, m.R.m21, m.R.m22, m.R.m23, m.R.m31, m.R.m32, m.R.m33);
//		//	}
//		//}
//		break;
//		//if (poseDataPtr._Ptr == NULL) {
//		//	context.write("Unable to find the withdraw trajectory.\n");
//		//	return;
//		//}
//		//performWithdraw(poseDataPtr);
//		//return;
//		//if (modelPoints.empty() || queryDataPtr == getData().end() || grasp::to<Data>(queryDataPtr)->queryPoints.empty()) {
//		//	context.write("Unable to find a model or query. Please make sure you have generated a model and query.\n");
//		//	return;
//		//}
//		//context.write("Learning observational model\n");
//		//Controller::State::Seq seq;
//		//extrapolate(makeCommand(grasp::to<Data>(dataPtr)->actionApproach), trjApproachExtrapolFac, false, seq);
//		//// compute the approach traectory in a new frame and its profile
//		//grasp::to<Data>(dataPtr)->action.clear();
//		//grasp::to<Data>(queryDataPtr)->action.clear();
//		//PosePlanner::profile(queryDataPtr, seq, trjApproachDuration, trjApproachIdle);
//		//// copy the planned trajectory in my trial data
//		//grasp::to<Data>(poseDataPtr)->action.clear();
//		//const Controller::State pregrasp = *grasp::to<Data>(dataPtr)->action.begin();
//		//for (Controller::State::Seq::const_iterator i = grasp::to<Data>(queryDataPtr)->action.begin(); i != grasp::to<Data>(queryDataPtr)->action.end(); ++i)
//		//	grasp::to<Data>(poseDataPtr)->action.push_back(*i);
//		//
//		//golem::Controller::State::Seq initTrajectory;
//		//robot->createTrajectory(robot->recvState().command, &grasp::to<Data>(dataPtr)->action.front(), NULL, 0, initTrajectory);
//		//robot->enableGuards(false);
//		//robot->sendTrajectory(initTrajectory, true);
//		//robot->waitForEnd();
//
//		//Controller::State state = pregrasp;
//		//const Chainspace::Index i = robot->getStateHandInfo().getChains().begin() + 1;
//		//return;
//	}
	case 'X':
	{
		if (getData().empty()) {
			context.write("Error: no data is loaded.");
			return;
		}
		auto strategy = [&] (const Strategy &strat) -> std::string {
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
			//switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
			//	case 'N':
			//	{
			//		strat = Strategy::NONE_STRATEGY;
			//		return;
			//	}
			//	case 'E':
			//	{
			//		strat = Strategy::ELEMENTARY;
			//		return;
			//	}
			//	case 'M':
			//	{
			//		strat = Strategy::MYCROFT;
			//		return;
			//	}
			//	case 'I':
			//	{
			//		strat = Strategy::IR3NE;
			//		return;
			//	}
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
//		Strategy strat = Strategy::ELEMENTARY; //Strategy::NONE;
		auto getNViews = [&](const size_t view) -> size_t {
			switch (view) {
			case 0:
				return 1;
			case 1:
				return 3;
			case 3:
				return 5;
			case 5:
				return 7; 
			default:
				return 10;
			}
		};
		const Controller::State home = robot->recvState().command;
		auto reset = [&]() {
			enableForceReading = false;
			pHeuristic->enableUnc = false;
			pHeuristic->setPointCloudCollision(false);
			Controller::State::Seq seq, out;
			robot->findTrajectory(robot->recvState().command, &home, nullptr, 0, seq);
			profile(this->trjDuration, seq, out, true);
			robot->sendTrajectory(out);
			// repeat every send waypoint until trajectory end
			for (U32 i = 0; robot->waitForBegin(); ++i) {
				if (universe.interrupted())
					throw grasp::Interrupted();
				if (robot->waitForEnd(0))
					break;
			}

		};
		auto resetDataPtr = [&](Data::Map::iterator& dataPtr) {
			grasp::to<Data>(dataPtr)->queryPoints.clear();
			grasp::to<Data>(dataPtr)->queryTransform.setId();
			grasp::to<Data>(dataPtr)->queryFrame.setId();
			grasp::to<Data>(dataPtr)->poses.clear();
			grasp::to<Data>(dataPtr)->hypotheses.clear();
			grasp::to<Data>(dataPtr)->simulateObjectPose.clear();
		};
		auto strHypothesisDesc = [=](std::ostream& ostr) {
			ostr << "px" << "\t" << "py" << "\t" << "pz" << "\t" << "qw" << "\t" <<
				"qx" << "\t" << "qy" << "\t" << "qz" << std::endl;
		};
		auto strHypothesis = [=](std::ostream& ostr, const grasp::RBPose::Sample::Seq &poses) {
			for (auto pose = poses.begin(); pose != poses.end(); ++pose)
				ostr << (*pose).p.x << "\t" << (*pose).p.y << "\t" << (*pose).p.z << "\t" << (*pose).q.w << "\t" << (*pose).q.x << "\t" << (*pose).q.y << "\t" << (*pose).q.z << std::endl;
		};
		// save log file
//		std::ofstream hypothesisLog("./data/boris/experiments/hypotheses.log"); strHypothesisDesc(hypothesisLog);
		std::ofstream logFile("./data/boris/experiments/mug4/output.log");
		std::ofstream elementaryLog("./data/boris/experiments/mug4/elementary.log");
		std::ofstream mycroftLog("./data/boris/experiments/mug4/mycroft.log");
		std::ofstream ireneLog("./data/boris/experiments/mug4/irene.log");

		context.write("================================================================================================================================\n");
		logFile << "================================================================================================================================\n";
		context.debug("Running analysis on:\n");
		// iterate fort all the files loaded in data
		grasp::Cloud::PointSeqMap::const_iterator points;
		
		// number of maximum failures allowed
		const U32 maxFailures = 2, maxIterations = 10;
		for (auto cdata = getData().begin(); cdata != getData().end(); ++cdata) {
			//// load model points
			//grasp::to<Data>(cdata->second)->ptrIndex = 0;
			//points = getPoints(cdata);
			//const std::string objectName = grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str();
			//// LOAD MODEL POINTS
			//context.debug("Creating %s model...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str());
			//pBelief->createModel(points->second);
			//modelFrame = Belief::createFrame(points->second);
			//modelPoints = grasp::to<Data>(cdata->second)->modelPoints; // points->second;
			//context.debug("Model point size %d, loaded point size %d\n", modelPoints.size(), grasp::to<Data>(cdata)->modelPoints.size());
			//resetDataPointers();
			//renderData(cdata);

			U32 size = grasp::to<Data>(cdata->second)->pointsRaw.size();
			std::vector<U32> indeces;
			indeces.reserve(size);
			for (auto i = 0; i < size; ++i) indeces.push_back(i);
			// to retieve the newest processed points for the query
			U32 indexProcessedPoints = 0;
			for (auto trj = grasp::to<Data>(cdata)->trajectory.begin(); trj != grasp::to<Data>(cdata)->trajectory.end(); ++trj) {
				context.debug("%s, trajectory %s\n", cdata->first.c_str(), trj->first.c_str());

				grasp::to<Data>(cdata->second)->stratType = Strategy::NONE_STRATEGY;
				// number of views selected for query
				size_t VIEWS = 0;
				for (; VIEWS < 7;) {
					std::string results;
					VIEWS = getNViews(VIEWS);
					//readNumber("Enter views: ", VIEWS);
					for (size_t TRIALS = 1; TRIALS <= 13; ++TRIALS) {
						//readNumber("Enter trial: ", TRIALS);
						for (;;) { // run through strategies
							// View\tTrial\tCoverage\tlin\tang\tlin\tang\tgrasped\titerations\n
							results = grasp::makeString("%u\t%u", VIEWS, TRIALS);
							// reset query information
							resetDataPtr(cdata);
							// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
							select(grasp::to<Data>(cdata->second)->stratType);
							grasp::to<Data>(cdata->second)->stratType = Strategy::IR3NE;
							// if strategy is NONE then we test a new trajectory
							if (grasp::to<Data>(cdata->second)->stratType == Strategy::NONE_STRATEGY)
								break;
							context.write("Strategy: %s\n", strategy(grasp::to<Data>(cdata->second)->stratType).c_str());
							logFile << grasp::makeString("Strategy: %s\n", strategy(grasp::to<Data>(cdata->second)->stratType).c_str());
							// set up trial data
							//this->trial->path = grasp::makeString("%s%s", this->trial->path.c_str(), cdata->first.c_str()/*, this->trial->extTrial.c_str()*/);
							//context.write("path %s\n", this->trial->path.c_str());
							//return;
							grasp::to<Data>(cdata->second)->ptrIndex = 0;
							points = getPoints(cdata);

							TrialData::Ptr tdata = createTrialData();
							tdata->grasped = false;
							tdata->silent = true;
							tdata->setup(context, rand);
							tdata->name = grasp::makeString("trial_%s_v%s_0%s", strategy(grasp::to<Data>(cdata->second)->stratType).c_str(), boost::lexical_cast<std::string>(VIEWS).c_str(), boost::lexical_cast<std::string>(TRIALS).c_str()); //"trial_v1_01"; 
							tdata->object = grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(); // objectName; //
							tdata->path = grasp::makeString("%s%s/%s%s", this->trial->path.c_str(), tdata->object.c_str(), tdata->name.c_str(), this->trial->extTrial.c_str());
							context.write("Object name = %s\nFile data = %s\nGrasp type = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
								tdata->object.c_str(), grasp::to<Data>(cdata->second)->path.c_str(), trj->first.c_str(), tdata->name.c_str(), VIEWS, TRIALS, tdata->path.c_str());
							logFile << grasp::makeString("Object name = %s\nFile data = %s\nGrasp type = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
								tdata->object.c_str(), grasp::to<Data>(cdata->second)->path.c_str(), trj->first.c_str(), tdata->name.c_str(), VIEWS, TRIALS, tdata->path.c_str());
							trialPtr = getTrialData().insert(getTrialData().begin(), TrialData::Map::value_type(grasp::to<TrialData>(tdata)->path, tdata));

							// LOAD MODEL POINTS
							context.debug("Creating %s model (%u points)...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), points->second.size());
							pBelief->createModel(points->second);
							modelFrame = Belief::createFrame(points->second);
							modelPoints = grasp::to<Data>(cdata->second)->modelPoints; // points->second;
							context.debug("Model point size %d, loaded point size %d\n", modelPoints.size(), grasp::to<Data>(cdata->second)->modelPoints.size());
							resetDataPointers();
							renderData(cdata);

							// COMPUTE QUERY POINTS
							grasp::to<Data>(cdata->second)->ptrPoints = false;
							resetDataPointers();
							//to<Data>(dataPtr)->ptrIndex = 0;
							auto range = grasp::Cloud::find<grasp::Cloud::RawPointSeqMultiMap>(grasp::to<Data>(cdata->second)->pointsRaw, grasp::to<Data>(cdata->second)->ptrLabel);
							if (range.first != range.second) {
								Selection selection;
								//U32 size = grasp::to<Data>(cdata->second)->pointsRaw.size();
								//std::vector<U32> indeces;
								//indeces.reserve(size);
								//for (auto i = 0; i < size; ++i) indeces.push_back(i);
								std::random_shuffle(indeces.begin(), indeces.end());
								for (size_t pview = 0; pview < VIEWS; ++pview) {
									grasp::to<Data>(cdata->second)->ptrIndex = indeces[pview];
									grasp::Cloud::RawPointSeqMultiMap::const_iterator points = getTrnRawPoints(cdata, grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34());
									context.debug("Query transform {(%f %f %f)}\n", grasp::to<TrialData>(trialPtr)->queryPointsTrn.p.x, grasp::to<TrialData>(trialPtr)->queryPointsTrn.p.y, grasp::to<TrialData>(trialPtr)->queryPointsTrn.p.z);
									context.debug("Query: point cloud index=%d(%d), point size %d\n", indeces[pview], indeces.size(), points->second.size());
									selection[grasp::to<Data>(cdata->second)->ptrIndex] = grasp::to<Data>(cdata->second)->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(grasp::to<Data>(cdata->second)->pointsRaw);
								}
								renderData(cdata);
								try {
									processPoints(cdata, selection, true);
								}
								catch (const Message& msg) {
									context.write("%s\n", msg.str().c_str());
									return;
								}
								grasp::to<Data>(cdata->second)->ptrIndex = grasp::to<Data>(cdata->second)->points.size() - 1;
								auto range = grasp::Cloud::find<grasp::Cloud::PointSeqMap>(grasp::to<Data>(cdata->second)->points, grasp::to<Data>(cdata->second)->ptrIndex);
								points = getPoints(cdata);
								context.write("point name = %s, size = %d, ptrIndex = %d\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), grasp::to<Data>(cdata->second)->points.size(), grasp::to<Data>(cdata->second)->ptrIndex);
								logFile << grasp::makeString("point name = %s, size = %d, ptrIndex = %d\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), grasp::to<Data>(cdata->second)->points.size(), grasp::to<Data>(cdata->second)->ptrIndex);
								renderData(cdata);
								//grasp::to<Data>(cdata->second)->ptrIndex = indexProcessedPoints++;
								//for (grasp::to<Data>(cdata->second)->ptrIndex = 0; grasp::to<Data>(cdata->second)->ptrIndex < grasp::to<Data>(cdata->second)->points.size(); grasp::to<Data>(cdata->second)->ptrIndex++) {
								//	points = getPoints(cdata);
								//	//grasp::to<Data>(cdata->second)->ptrIndex = grasp::to<Data>(cdata->second)->points.size() - 1;
								//	auto range = grasp::Cloud::find<grasp::Cloud::PointSeqMap>(grasp::to<Data>(cdata->second)->points, grasp::to<Data>(cdata->second)->ptrIndex);
								//	context.write("point name = %s, size = %d, ptrIndex = %d\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str(), grasp::to<Data>(cdata->second)->points.size(), grasp::to<Data>(cdata->second)->ptrIndex);
								//	renderData(cdata);
								//	::Sleep(5000);
								//}
							//	for (;;) {
							//		const int key = waitKey();
							//		if (key == 13) { // enter
							//			context.write("\n");
							//			break;
							//		}
							//		else if (key == '[') {
							//			if (grasp::to<Data>(cdata->second)->ptrIndex > 0) grasp::to<Data>(cdata->second)->ptrIndex--;
							//		}
							//		else if (key == ']') {
							//			grasp::to<Data>(cdata->second)->ptrIndex++;
							//		}
							//		else if (key == 127) { // del
							//			selection.clear();
							//		}
							//		else if (key == (GLUT_KEY_INSERT | Universe::KEY_SPECIAL)) { // ins
							//			selection.clear();
							//			size_t ptr = 0;
							//			if (grasp::to<Data>(cdata->second)->ptrLabel == grasp::Cloud::LABEL_DEFAULT) {
							//				for (grasp::Cloud::RawPointSeqMultiMap::iterator i = grasp::to<Data>(cdata->second)->pointsRaw.begin(); i != grasp::to<Data>(cdata->second)->pointsRaw.end(); ++i) selection[ptr++] = i;
							//			}
							//			else {
							//				auto range = grasp::Cloud::find(grasp::to<Data>(cdata->second)->pointsRaw, grasp::to<Data>(cdata->second)->ptrLabel);
							//				for (grasp::Cloud::RawPointSeqMultiMap::iterator i = range.first; i != range.second; ++i) selection[ptr++] = i;
							//			}
							//		}
							//		else if (key == ' ') {  // <space>
							//			auto ptr = selection.find(grasp::to<Data>(cdata->second)->ptrIndex);
							//			if (selection.end() != ptr)
							//				selection.erase(ptr);
							//			else
							//				selection[grasp::to<Data>(cdata->second)->ptrIndex] = grasp::to<Data>(cdata->second)->getPoints<grasp::Cloud::RawPointSeqMultiMap::iterator>(grasp::to<Data>(cdata->second)->pointsRaw);
							//		}
							//		else if (key == 27) {  // <Esc>
							//			throw Cancel("\nCancelled");
							//		}
							//		else {
							//			context.write(
							//				"\n[]               change current point cloud\n"
							//				"<Ins>/<Del>      select/clear all point clouds\n"
							//				"<Space>          select/clear current point cloud\n"
							//				"<Esc>            cancel\n"
							//				"<Enter>          process\n"
							//				);
							//		}
							//	}
							//	if (selection.empty())
							//		context.write("No point clouds to process\n");
							//	else {
							//		processPoints(cdata, selection);
							//		renderData(cdata);
							//		context.write("Done!\n");
							//	}
							//}
							}
							else {
								context.write("Error: range.first != range.second\n");
								return;
							}

							// QUERY CREATE
							//points = getPoints(cdata);
							grasp::to<Data>(cdata->second)->queryPoints = points->second;
							context.write("Creating %s query...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str());
							logFile << grasp::makeString("Creating %s query...\n", grasp::to<Data>(cdata->second)->getLabelName(points->first).c_str());
							pBelief->createQuery(points->second);
							// compute transformation model -> query
							const grasp::RBPose::Sample trn = pBelief->createHypotheses(modelPoints, modelFrame);
							grasp::to<Data>(cdata->second)->queryTransform = trn.toMat34();
							// update query settings
							//resetDataPointers();
							queryDataPtr = cdata;
							grasp::to<Data>(cdata->second)->queryFrame.multiply(grasp::to<Data>(cdata->second)->queryTransform, modelFrame);
							//grasp::Cloud::transform(grasp::to<Data>(cdata->second)->queryTransform, modelPoints, grasp::to<Data>(cdata->second)->queryPoints);
							// copying belief
							//grasp::to<Data>(cdata->second)->poses.clear();
							grasp::to<Data>(cdata->second)->poses = pBelief->getSamples();
							//grasp::to<Data>(cdata->second)->hypotheses.clear();
							grasp::to<Data>(cdata->second)->hypotheses = pBelief->getHypothesesToSample();
							context.debug("Belief poses = %d, hypotheses = %d\n", pBelief->getSamples().size(), pBelief->getHypotheses().size());

							// set the simulated object as ground truth
							//grasp::to<Data>(cdata->second)->simulateObjectPose.clear();
							grasp::to<Data>(cdata->second)->simulateObjectPose.reserve(modelPoints.size());
							for (U32 point = 0; point != modelPoints.size(); ++point) {
								grasp::Cloud::Point p = modelPoints[point];
								grasp::Cloud::setColour(golem::RGBA::MAGENTA, p);
								grasp::to<Data>(cdata->second)->simulateObjectPose.push_back(p);
							}
							context.write("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(cdata->second)->queryPoints.size(), ((Real)grasp::to<Data>(cdata->second)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
							logFile << grasp::makeString("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(cdata->second)->queryPoints.size(), ((Real)grasp::to<Data>(cdata->second)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
							//grasp::Cloud::transform(grasp::to<Data>(cdata->second)->queryTransform, grasp::to<Data>(cdata->second)->simulateObjectPose, grasp::to<Data>(cdata->second)->simulateObjectPose);
							Mat34 m = grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34();
							m.p.y -= 0.005; //m.p.z += 0.01;
							grasp::Cloud::transform(/*grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34()*/m, grasp::to<Data>(cdata->second)->simulateObjectPose, grasp::to<Data>(cdata->second)->simulateObjectPose);
							enableForceReading = false; 
							collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
							objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));

							// render
							showSamplePoints = true; // shows hypotheses and mean pose
							showMeanHypothesis = false;
							showDistrPoints = false;
							showQueryPoints = false;
							showQueryDistrib = true;
							showObject = false;
							// removes grey points from the model
							showPoints = false;
							//pointRenderer.reset();
							renderData(cdata);

							//strHypothesis(hypothesisLog, grasp::to<Data>(cdata->second)->poses);
							//return;

							// uncomment to interrupt the loop before planning thew trajectory
							/*::Sleep(5000);
							continue;*/

							// PLAN TRAJECTORY
							//currentDataPtr = queryDataPtr;

							grasp::to<Data>(cdata->second)->replanning = false;
							Controller::State::Seq inp = grasp::RobotState::makeCommand(trj->second);
							// open the fingers for the pregrasp
							for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
								auto j = robot->getStateHandInfo().getJoints(robot->getStateHandInfo().getChains().begin()).begin() + 1;
								inp[1].cpos[j] = 0.0;
							}
							Mat34 objectTransform;
							objectTransform.multiply(grasp::to<TrialData>(trialPtr)->queryPointsTrn.toMat34(), modelFrame);
							grasp::RBCoord obj(objectTransform);
							U32 iteration = 1, failures = 0;
							for (; failures < maxFailures && iteration < maxIterations;) {
								grasp::to<Data>(cdata->second)->actionType = action::IG_PLAN_M2Q;
								grasp::RBCoord queryPose(grasp::to<Data>(cdata->second)->queryFrame);
								grasp::RBDist error;
								error.lin = obj.p.distance(queryPose.p);
								error.ang = obj.q.distance(queryPose.q);
								context.write("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
								context.write("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
									obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
									queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
									error.lin, error.ang);
								logFile << grasp::makeString("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
								logFile << grasp::makeString("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
									obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
									queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
									error.lin, error.ang);
								if (iteration == 1 && failures == 0)
									results = grasp::makeString("%s\t%.2f\t%.6f\t%.6f", results.c_str(), ((Real)grasp::to<Data>(cdata->second)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100, error.lin, error.ang);

								context.debug("\n----------------------------------------------------------\n");
								if (!execute(cdata, inp)) { // if it fails to find a trajectory repeat 
									++failures;
									continue;
								}

								if (contactOccured && grasp::to<Data>(cdata->second)->stratType != Strategy::ELEMENTARY) {
									//grasp::to<Data>(cdata->second )->replanning = false;
									contactOccured = false;
									updateAndResample(cdata);
									enableForceReading = false;
									++iteration;
									//bool r = unlockContact();
									//context.write("unlock contact %s\n", r ? "TRUE" : "FALSE");
									continue;
								}
								// grasp
								//if (error.lin < 0.01) {
								grasp::to<Data>(cdata->second)->actionType = action::GRASP;
								context.debug("\n----------------------------------------------------------\n");
								context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
								logFile << grasp::makeString("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType));
								if (!execute(cdata, inp)) {// not successfully close fingers
									++failures;
									continue;
								}
								//enableForceReading = true;
								//::Sleep(1000); // time to retrieve contacts
								size_t collisions = collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getPose(robot->recvState().config), triggeredGuards);
								context.write("Attempt to grasp. Collisions=%d\n", collisions);
								logFile << grasp::makeString("Attempt to grasp. Collisions=%d\n", collisions);
								if (collisions > 1)
									grasp::to<TrialData>(trialPtr)->grasped = true;
								else if (grasp::to<Data>(cdata->second)->stratType != Strategy::ELEMENTARY) { // no contact while grasping then we need to keep iterating
								//	if (waitKey("YN", "Want to replan? (Y/N)") == 'Y') {
										contactOccured = false;
										updateAndResample(cdata);
										enableForceReading = false;
										context.write("No contact retrieved while grasping. replanning.\n");
										++iteration;
										/*if (iteration > 15)
											failures = maxFailures;*/
										continue;
								//	}
								}

								
								//context.write("Iteration %u: execute trajectory (%s). triggered guards %u\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType), triggeredGuards.size());
								//logFile << grasp::makeString("Iteration %u: execute trajectory (%s). triggered guards %u\n", iteration, actionToString(grasp::to<Data>(cdata->second)->actionType), triggeredGuards.size());
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
							switch (grasp::to<Data>(cdata->second)->stratType){
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
							context.debug("%s\n", grasp::to<TrialData>(trialPtr)->toString(*robot->getController()).c_str());
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
							//return;
							//createBeliefState();
							//pHeuristic->setBelief(pBelief);
						}
					} // end Strategy
				} // end TRIALS
			} // end VIEWS
		} // end for (auto cdata = getData().begin(); cdata != getData().end(); ++cdata)
		context.write("================================================================================================================================\n");
		logFile << "================================================================================================================================\n";
//		if (modelPoints.empty() || pBelief->getSamples().empty()) {
//			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
//			return;
//		}
//		// update pose settings
//		poseDataPtr = dataPtr;
//		grasp::to<Data>(dataPtr)->queryTransform = grasp::to<Data>(queryDataPtr)->queryTransform;
//		grasp::to<Data>(dataPtr)->queryFrame = grasp::to<Data>(queryDataPtr)->queryFrame;
//		//grasp::to<Data>(dataPtr)->actionApproach = grasp::to<Data>(queryDataPtr)->actionApproach;
//		//grasp::to<Data>(dataPtr)->actionManip = grasp::to<Data>(queryDataPtr)->actionManip;
//		//grasp::to<Data>(dataPtr)->action = grasp::to<Data>(queryDataPtr)->action;
//		grasp::to<Data>(dataPtr)->queryPoints = grasp::to<Data>(queryDataPtr)->queryPoints;
//		objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->queryPoints));
//		renderData(dataPtr);
//		//grasp::to<Data>(dataPtr)->actionWithdraw.clear();
//		const grasp::RobotState homeState = robot->recvState();
//		//		grasp::to<Data>(dataPtr)->actionWithdraw.push_back(robot->recvState());
//		//		grasp::to<Data>(dataPtr)->actionWithdraw.push_back(robot->recvState());
//
//		grasp::to<Data>(dataPtr)->replanning = false;
//
//		//if (grasp::to<Data>(dataPtr)->actionApproach.empty())
//		//	context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");
//
//		robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
//		robot->setCollisionDetection(true);
//
//	AGAIN:
//		context.debug("RagPlanner:function(X): performing approach trajectory\n");
//		try {
//			if (!singleGrasp) {
//				performApproach(dataPtr);
//			}
//			else {
//				context.write("Single grasp performs\n");
//				performSingleGrasp(dataPtr);
//				//				goto GRASP_QUALITY;
//			}
//		}
//		catch (const Message& msg) {
//			context.write("%s\n", msg.str().c_str());
//		}
//		context.write("perform approach is over.\n");
////		return;
//		if (grasp::to<Data>(dataPtr)->replanning) {
//			//if (!executedTrajectory.empty()) {
//			//	context.write("Computing withdrawing pose.\n");
//			//	Real distMin(REAL_MAX);
//			//	Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
//			//	// skip the last element of the executed trajectory which is supposed to be the final state
//			//	for (Controller::State::Seq::const_iterator a = executedTrajectory.begin(), b = executedTrajectory.begin() + 1; b != executedTrajectory.end() - 1; ++a, ++b) {
//			//		Real dist(REAL_ZERO);
//			//		for (Configspace::Index i = robot->getStateInfo().getJoints().begin(); i != robot->getStateInfo().getJoints().end(); ++i)				
//			//			dist += Math::sqrt(Math::sqr(robotConfig.cpos[i] - (a->cpos[i] + (a->cpos[i] - b->cpos[i])/2))); //Math::sqrt(Math::sqr(robotConfig.cpos[i] - a->cpos[i]));
//			//		if (dist/* = Math::sqrt(dist)*/ < distMin) {
//			//			distMin = dist;
//			//			waypoint = *a;
//			//		}
//			//	}
//			//	grasp::RobotState w(*robot->getController());
//			//	w.command = w.config = waypoint;
//			//	while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2)
//			//		grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(w);
//			//}
//			//else {
//			//	context.write("Vanilla withdraw (home pose).\n");
//			//	grasp::to<Data>(dataPtr)->actionWithdraw.push_back(homeState);
//			//	grasp::to<Data>(dataPtr)->actionWithdraw.push_back(homeState);
//			//}
//			//performWithdraw(dataPtr);
//			const int key = waitKey("YN", "Do you want to repeat? (Y/N)...");
//			if (key == 'Y') {
//				goto AGAIN;
//			}
//			else if (key == 'N')
//				return;
//		}
//		else
//			performManip(dataPtr);
		logFile.close();
		elementaryLog.close();
		mycroftLog.close();
		ireneLog.close();
		return;
	}
	case 'L':
	{
		auto select = [&](Strategy &strat) {
			switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
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

		// set the simulated object
		if (!grasp::to<Data>(dataPtr)->simulateObjectPose.empty()) {
			collisionPtr->create(rand, grasp::to<Data>(dataPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataPtr)->simulateObjectPose));
		}

		grasp::RobotState::Map::iterator trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
		if (trajectory->second.size() < 3)
			context.write("Error: the selected trajectory have not at least 3 waypoints.");

		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
		select(grasp::to<Data>(dataPtr)->stratType);
		grasp::to<Data>(dataPtr)->actionType = action::NONE_ACTION;
		grasp::to<Data>(dataPtr)->replanning = false;
		Controller::State::Seq inp = grasp::RobotState::makeCommand(trajectory->second);
		// open the fingers for the pregrasp
		for (auto i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) { //	for (auto i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i)
			auto j = robot->getStateHandInfo().getJoints(robot->getStateHandInfo().getChains().begin()).begin() + 1;
			inp[1].cpos[j] = 0.0;
		}
		for (;;) {
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
			if (!execute(dataPtr, inp))
				return;
			if (contactOccured) {
				//grasp::to<Data>(cdata->second)->replanning = false;
				contactOccured = false;
				updateAndResample(dataPtr);
				enableForceReading = false;
				continue;
			}
			// grasp
			enableForceReading = false;
			//grasp::to<Data>(dataPtr)->actionType = action::GRASP;
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
			if (!execute(dataPtr, inp))
				return;
			
			// lifting
			enableForceReading = false;
			//grasp::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataPtr)->actionType));
			if (execute(dataPtr, inp))
				return;

			break;
		}
		context.write("Finish Experiment\n");
		//context.write("%s\n", grasp::to<TrialData>(trialPtr)->toString(*robot->getController()).c_str());
		return;
	}
//
//
//		//grasp::Cloud::PointSeqMap::const_iterator points = getPoints(dataPtr);
//		//// query create
//		//context.write("Creating %s query...\n", grasp::to<Data>(dataPtr)->getLabelName(points->first).c_str());
//		//pBelief->createQuery(points->second);
//		//// create hypotheses and return the action frame for this query
//		//actionFrameT = pBelief->maximum().toMat34();
//		//context.write("action frame TEACH <%f %f %f>\n", actionFrameT.p.x, actionFrameT.p.y, actionFrameT.p.z);
//
//		//try {
//		//	graspFrame = getGraspFrame(dataPtr);
//		//}
//		//catch (const Message& msg) {
//		//	context.write("%s\n", msg.str().c_str());
//		//}		
//		return;
//	}
//	case 'U':
//	{
//		if (modelPoints.empty() || pBelief->getSamples().empty()) {
//			context.write("Unable to find a model or query. Please make sure you have generated a model and a query.\n");
//			return;
//		}
//		// update pose settings
//		poseDataPtr = dataPtr;
//		grasp::to<Data>(poseDataPtr)->actionFrame = grasp::to<Data>(queryDataPtr)->actionFrame;
//		grasp::to<Data>(poseDataPtr)->queryFrame = grasp::to<Data>(queryDataPtr)->queryFrame;
//		grasp::to<Data>(poseDataPtr)->actionApproach = grasp::to<Data>(queryDataPtr)->actionApproach;
//		grasp::to<Data>(poseDataPtr)->actionManip = grasp::to<Data>(queryDataPtr)->actionManip;
//		grasp::to<Data>(poseDataPtr)->action = grasp::to<Data>(queryDataPtr)->action;
//		grasp::to<Data>(poseDataPtr)->queryPoints = grasp::to<Data>(queryDataPtr)->queryPoints;
//		//grasp::to<Data>(dataPtr)->queryPoints.clear();
//		//grasp::to<Data>(dataPtr)->queryPoints.reserve(grasp::to<Data>(queryDataPtr)->queryPoints.size());
//		//std::for_each(grasp::to<Data>(queryDataPtr)->queryPoints.begin(), grasp::to<Data>(queryDataPtr)->queryPoints.end(), [=] (grasp::Cloud::PointSeq::const_iterator p) {
//		//	grasp::to<Data>(dataPtr)->queryPoints.push_back(*p);
//		//});
//
//		// set home pose
//		//grasp::to<Data>(poseDataPtr)->homeStates.clear();
//		//std::cout << "spam:function: 7 bis\n";
//		//grasp::to<Data>(poseDataPtr)->homeStates.reserve(1);
//		//std::cout << "spam:function: 8 riceiving state\n";
//		//grasp::RobotState homeState = robot->recvState();
//		//std::cout << "spam:function: 8 bis\n";
//		//grasp::to<Data>(poseDataPtr)->homeStates.push_back(homeState);
//		homeStates.clear();
//		homeStates.push_back(robot->recvState());
//
//		bool conclude = false;
//		grasp::to<Data>(poseDataPtr)->replanning = false;
//		grasp::to<Data>(poseDataPtr)->release = false;
//		switch (waitKey("PE", "Press a key to (P)lan a reach-and-grasp trajectory, (E)nable/Disable planning with uncertainty...")) {
//		case 'P':
//		{			
//			if (grasp::to<Data>(poseDataPtr)->actionApproach.empty())
//				context.write("RagPlanner(): Unable to find a pre-grasp pose of the robot.\n");
//
//			// approach action is the entire trajectory taught to the robot
//			// NOTE: states are memorised from the last (pre grasp = begin()) to the first (initial pose = end())
//			// Generate a new approach action formed by [pregrasp, corrent conf of the robot]
//			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//			GenWorkspaceChainState gwcs;
//			gwcs.setToDefault(robot->getStateInfo().getChains().begin(), robot->getStateInfo().getChains().end()); // all used chains
//			grasp::RobotState grasp = *grasp::to<Data>(poseDataPtr)->actionApproach.begin();
//			robot->getController()->chainForwardTransform(grasp.config.cpos, gwcs.wpos);
//			Mat34 poseFrameInv, graspFrame, graspFrameInv;
//			poseFrameInv.setInverse(gwcs.wpos[armChain]);
//			graspFrame.multiply(poseFrameInv, modelFrame/*grasp::to<Data>(poseDataPtr)->queryFrame*/);
//			graspFrameInv.setInverse(graspFrame);
////				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
//			graspFrameM.multiply(modelFrame, graspFrameInv);
//			renderHand(grasp.config, true);
//
//			grasp::RobotState pregrasp = *(--grasp::to<Data>(poseDataPtr)->actionApproach.end());			
//			robot->getController()->chainForwardTransform(pregrasp.config.cpos, gwcs.wpos);
//			Mat34 preposeFrameInv, pregraspFrame, pregraspFrameInv;
//			preposeFrameInv.setInverse(gwcs.wpos[armChain]);
//			pregraspFrame.multiply(preposeFrameInv, modelFrame/*grasp::to<Data>(poseDataPtr)->queryFrame*/);
//			pregraspFrameInv.setInverse(pregraspFrame);
////				gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], modelFrameInv); // new waypoint frame
//			pregraspFrameM.multiply(modelFrame, pregraspFrameInv);
//			renderHand(pregrasp.config);
//			showTest = false;
//			renderData(dataPtr);
//
//			trjApproachExtrapolFac = REAL_ZERO;
//			robot->setCollisionBoundsGroup(Bounds::GROUP_ALL);
//			robot->setCollisionDetection(true);		
////			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
//			iterations = 1;
//
//REPLAN_TEST:
//			try {
//				if (!singleGrasp) {
//					testTransformation(actionFrameT, modelFrame, grasp::to<Data>(dataPtr)->actionFrame, grasp::to<Data>(dataPtr)->actionApproach);
//					performApproach(poseDataPtr);
//					context.write("perform approach is over.\n");
//				}
//				else {
//					context.write("Single grasp performs\n");
//					performSingleGrasp(poseDataPtr);
//					goto GRASP_QUALITY;
//				}
//			}
//			catch (const Message& msg) {
//				context.write("%s\n", msg.str().c_str());
//			}
//			
//			std::cout << "check for computing the grasp quality.\n";
//			if (grasp::to<Data>(poseDataPtr)->triggered > 0 && !grasp::to<Data>(poseDataPtr)->replanning) {
//				// todo: compute if grasped
//GRASP_QUALITY:
//				std::cout << "grasp quality: step 1\n";
//				context.write("Check the grasp quality (triggered=%s, replanning=%s)\n", grasp::to<Data>(poseDataPtr)->triggered > 0 ? "true" : "false", grasp::to<Data>(poseDataPtr)->replanning ? "true" : "false"); 
//
//				std::cout << "grasp quality: step 2\n";
//				grasp::RealSeq force;
//				//Real graspQuality = REAL_ZERO;
//				//try {
//				//	graspQuality = robot->analyseGrasp(force);
//				//}
//				//catch (const golem::Message &msg) {
//				//	context.write("%s\n", msg.str().c_str());
//				//}
//				//std::cout << "grasp quality: step 3\n";
//				//replanning = !(graspQuality > 0);
//				//context.write("Grasp %s. quality = %f\n", !replanning ? "succeess (no replanning)" : "failure (replanning)", graspQuality);
//				//if (!replanning) {
//					const int key = waitKey("YN", "Do you want to exit? (Y/N)...");
//					if (key == 'Y') {
//						performManip(poseDataPtr);
//						return;
//						conclude = true;
//					} 
//					else {
//						const int key = waitKey("YN", "Do you want to replan a trajectory? (Y/N)...");
//						if (key == 'Y')
//							grasp::to<Data>(poseDataPtr)->replanning = true;
//						else return;
//					}
//				goto MOVING_BACK;
//			}
//			else {
//MOVING_BACK:
//				trnEnable = false;
//				pHeuristic->enableUnc = false;
//				grasp::to<Data>(poseDataPtr)->actionWithdraw.clear();
//				// if replanning is not required the robot is moved back to the home pose
//				if (!grasp::to<Data>(poseDataPtr)->replanning || withdrawToHomePose) {
////					if (!/*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.empty()) {
//						// set as first waypont the pregrasp pose or the home pose
//						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(grasp::to<Data>(poseDataPtr)->release && !grasp::to<Data>(poseDataPtr)->actionApproach.empty() ? *(--grasp::to<Data>(poseDataPtr)->actionApproach.end()) : homeStates.front());
//						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(homeStates.front());
////					}
//					grasp::to<Data>(poseDataPtr)->release = false;
//					//while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2 && !grasp::to<Data>(poseDataPtr)->executedTrajectory.empty())
//					//	grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(grasp::to<Data>(poseDataPtr)->homeStates.front());
//				// otherwise the previous waypoint in the trajectory is withdrawn
//				} else {
//					Real distMin(REAL_MAX);
//					Controller::State robotConfig = robot->recvState().config, waypoint = robot->recvState().config;
//					for (Controller::State::Seq::const_iterator a = /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.begin(), b = /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.begin() + 1; b != /*grasp::to<Data>(poseDataPtr)->*/executedTrajectory.end(); ++a, ++b) {
//						Real dist(REAL_ZERO);
//						for (Configspace::Index i = robot->getStateInfo().getJoints().begin(); i != robot->getStateInfo().getJoints().end(); ++i)				
//							dist += Math::sqrt(Math::sqr(robotConfig.cpos[i] - (a->cpos[i] + (a->cpos[i] - b->cpos[i])/2))); //Math::sqrt(Math::sqr(robotConfig.cpos[i] - a->cpos[i]));
//						if (dist/* = Math::sqrt(dist)*/ < distMin) {
//							distMin = dist;
//							waypoint = *a;
//						}
//					}
//					grasp::RobotState w(*robot->getController());
//					w.command = waypoint; w.config = waypoint;
//					while (grasp::to<Data>(poseDataPtr)->actionWithdraw.size() < 2)
//						grasp::to<Data>(poseDataPtr)->actionWithdraw.push_back(w);
//				}
////				renderTrialData(poseDataPtr);
//				// move back to home pose
//				performWithdraw(poseDataPtr);
//				goto REPLAN_TEST;
//			}
//			return;
//		}
//		case 'G':
//		{
//			golem::Waypoint w(*robot->getController(), robot->recvState().command.cpos);
//			Mat34 tcpFrameInv, trnFromTcpToObj, tcpFrame = w.wpos[robot->getStateInfo().getChains().begin()];
//			tcpFrameInv.setInverse(tcpFrame);
//			trnFromTcpToObj.multiply(tcpFrameInv, grasp::to<Data>(dataPtr)->queryFrame);
//			context.write("tcpFrame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n", 
//				tcpFrame.R.m11, tcpFrame.R.m12, tcpFrame.R.m13, tcpFrame.p.x, 
//				tcpFrame.R.m21, tcpFrame.R.m22, tcpFrame.R.m23, tcpFrame.p.y, 
//				tcpFrame.R.m31, tcpFrame.R.m32, tcpFrame.R.m33, tcpFrame.p.z);
//			context.write("obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n", 
//				grasp::to<Data>(dataPtr)->queryFrame.R.m11, grasp::to<Data>(dataPtr)->queryFrame.R.m12, grasp::to<Data>(dataPtr)->queryFrame.R.m13, grasp::to<Data>(dataPtr)->queryFrame.p.x, 
//				grasp::to<Data>(dataPtr)->queryFrame.R.m21, grasp::to<Data>(dataPtr)->queryFrame.R.m22, grasp::to<Data>(dataPtr)->queryFrame.R.m23, grasp::to<Data>(dataPtr)->queryFrame.p.y, 
//				grasp::to<Data>(dataPtr)->queryFrame.R.m31, grasp::to<Data>(dataPtr)->queryFrame.R.m32, grasp::to<Data>(dataPtr)->queryFrame.R.m33, grasp::to<Data>(dataPtr)->queryFrame.p.z);
//			context.write("Trn from tcp frame to obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
//				trnFromTcpToObj.R.m11, trnFromTcpToObj.R.m12, trnFromTcpToObj.R.m13, trnFromTcpToObj.p.x,
//				trnFromTcpToObj.R.m21, trnFromTcpToObj.R.m22, trnFromTcpToObj.R.m23, trnFromTcpToObj.p.y,
//				trnFromTcpToObj.R.m31, trnFromTcpToObj.R.m32, trnFromTcpToObj.R.m33, trnFromTcpToObj.p.z);
//			pBelief->setInitObjPose(tcpFrame);
//			Mat34 trn = pBelief->transform(grasp::to<Data>(dataPtr)->queryFrame);
//			context.write("BELIEF Trn from tcp frame to obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
//				trn.R.m11, trn.R.m12, trn.R.m13, trn.p.x,
//				trn.R.m21, trn.R.m22, trn.R.m23, trn.p.y,
//				trn.R.m31, trn.R.m32, trn.R.m33, trn.p.z);
//			trn.multiply(tcpFrame, trnFromTcpToObj);
//			context.write("BELIEF obj frame\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n",
//				trn.R.m11, trn.R.m12, trn.R.m13, trn.p.x,
//				trn.R.m21, trn.R.m22, trn.R.m23, trn.p.y,
//				trn.R.m31, trn.R.m32, trn.R.m33, trn.p.z);
//
//			Mat34 m = robot->getController()->getGlobalPose();
//			context.write("Robot global pose\n[[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n[%5.7f %5.7f %5.7f %5.7f]\n det:%5.7f\n",
//				m.R.m11, m.R.m12, m.R.m13, m.p.x,
//				m.R.m21, m.R.m22, m.R.m23, m.p.y,
//				m.R.m31, m.R.m32, m.R.m33, m.p.z,
//				m.R.determinant());
//
//			Real distMax = golem::REAL_ZERO;
//			for (grasp::Cloud::PointSeq::const_iterator p = modelPoints.begin(); p != modelPoints.end(); ++p) {
//				// CHECK max magnitude of absolute coordinates of points is related to the object coords but NOT its size!
//				//const Real d = p->frame.p.magnitude();
//				const Real d = grasp::Cloud::getPoint(*p).magnitude();
//				if (d > distMax)
//					distMax = d;
//			}
//			context.write("Size of the object %5.7f\n", distMax);
//			// go to to predefined pose
//			//context.write("Goto Pose test\n");
//			//(void)gotoPose();
//			//std::vector<Configspace::Index> triggeredGuards;
//			//if (robot->getTriggeredGuards(triggeredGuards)) {
//			//	context.write("RagPlanner::Test triggered guards on Justin");
//			//	for (std::vector<Configspace::Index>::const_iterator i = triggeredGuards.begin(); i != triggeredGuards.end(); ++i)
//			//		context.write(" %d", *i);
//			//	context.write("\n");
//			//}
//			return;
//		}
//		case 'T':
//		{
//			switch (waitKey("OU", "Press a key to test (O)bservations or (U)pdate..")) {
//			case 'O':
//			{			
//				context.write("test observation\n");
//				Mat34 pose;
//				pose.R.setId();
//				pose.R.rotX(-REAL_HALF*REAL_PI);
//				pose.p = Vec3(-0.1, 0.40, 0.1);
//				for (Real i = 0; i < 60; ++i) {
//					pose.p.x += 0.001*i;
//					pHeuristic->testObservations(pose, true);
//					renderPose(pose);
//					::Sleep(1000);
//				}
//				return;
//			}
//			case 'U':
//			{
//				context.write("test update\n");
//				grasp::RBPose::Sample::Seq samples;
//				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.80, -0.36, 0.01), Quat())));
//				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.65, -0.36, 0.01), Quat())));
//				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.50, -0.36, 0.01), Quat())));
//				samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.30, -0.36, 0.01), Quat())));
//				Mat34 pose;
//				pose.R.setId();
//				pose.R.rotX(REAL_HALF*REAL_PI);
//				pose.p = Vec3(0.4, -0.45, 0.1);
//				for (size_t i = 0; i < 20; ++i) {
//					pose.p.x += 0.01*i;
//					context.write("Iteration n.%d pose <%f %f %f>\n", i + 1, pose.p.x, pose.p.y, pose.p.z);
//					//for (grasp::RBPose::Sample::Seq::iterator j = samples.begin(); j != samples.end(); ++j)
//					//	j->weight = pHeuristic->evaluate(grasp::RBCoord(pose), *j, -0.5, modelFrame);
//					context.write("\n");
//					renderUpdate(pose, samples);
//					::Sleep(1000);
//				}
//				return;
//			}
//			}
//		}
//		case 'E':
//			uncEnable = !uncEnable;
//			context.write("Planning with uncertainty %s\n", uncEnable ? "ON" : "OFF");
//			return;
//		}
//		break;
//	}
//	//case 'T':
//	//{
//	//	switch (waitKey("OU", "Press a key to test (O)bservations or (U)pdate..")) {
//	//	case 'O':
//	//	{			
//	//		context.write("test observation\n");
//	//		Mat34 pose;
//	//		pose.R.setId();
//	//		pose.R.rotX(-REAL_HALF*REAL_PI);
//	//		pose.p = Vec3(-0.4, 0.70, 0.15);
//	//		for (Real i = 0; i < 60; ++i) {
//	//			pose.p.x += 0.001*i;
//	//			pHeuristic->testObservations(pose, true);
//	//			renderPose(pose);
//	//			::Sleep(1000);
//	//		}
//	//		return;
//	//	}
//	//	case 'U':
//	//	{
//	//		context.write("test update\n");
//	//		grasp::RBPose::Sample::Seq samples;
//	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.80, -0.36, 0.01), Quat())));
//	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.65, -0.36, 0.01), Quat())));
//	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.50, -0.36, 0.01), Quat())));
//	//		samples.push_back(grasp::RBPose::Sample(grasp::RBCoord(Vec3(0.30, -0.36, 0.01), Quat())));
//	//		Mat34 pose;
//	//		pose.R.setId();
//	//		pose.R.rotX(REAL_HALF*REAL_PI);
//	//		pose.p = Vec3(0.4, -0.45, 0.1);
//	//		for (size_t i = 0; i < 20; ++i) {
//	//			pose.p.x += 0.01*i;
//	//			context.write("Iteration n.%d pose <%f %f %f>\n", i + 1, pose.p.x, pose.p.y, pose.p.z);
//	//			//for (grasp::RBPose::Sample::Seq::iterator j = samples.begin(); j != samples.end(); ++j)
//	//			//	j->weight = pHeuristic->evaluate(grasp::RBCoord(pose), *j, -0.5, modelFrame);
//	//			context.write("\n");
//	//			renderUpdate(pose, samples);
//	//			::Sleep(1000);
//	//		}
//	//		return;
//	//	}
//	//	}
//	//}
//	case ',':
//	{
//		handBounds.clear();
//		return;
//	}
//	case '.':
//	{
//		renderHand(grasp::to<Data>(dataPtr)->actionApproach[grasp::to<Data>(dataPtr)->actionApproach.size()-1].config);
//		return;
////			const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
////			const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
////			GenWorkspaceChainState gwcs;
////			robot->getController()->chainForwardTransform(robot->recvState().config.cpos, gwcs.wpos);
//////			gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
////			renderPose(gwcs.wpos[armChain]);
////			grasp::RBCoord b(gwcs.wpos[armChain]);
////			context.write("render pose triggered state <%f %f %f> <%f %f %f %f>\n", b.p.x, b.p.y, b.p.z, b.q.w, b.q.x, b.q.y, b.q.z);
////			{		
////				golem::CriticalSectionWrapper csw(csDataRenderer);
////				const Bounds::Seq bounds = manipulator->getBounds(manipulator->getPose(robot->recvState().config));
////				handBounds.insert(handBounds.end(), bounds.begin(), bounds.end());
////			}	
////			return;
//	}
	case 'I':
	{		
		auto select = [&](Strategy &strat) {
			switch (waitKey("NEMI", "Press a key to select (N)one/(E)lementary/(M)ycroft/(I)r3ne...")) {
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

		grasp::RobotState::Map::iterator trajectory = selectTrajectory(grasp::to<Data>(dataPtr)->trajectory.begin(), grasp::to<Data>(dataPtr)->trajectory.end(), "Select trajectory:\n");
		if (trajectory->second.size() < 3)
			context.write("Error: the selected trajectory have not at least 3 waypoints.\n");

		Controller::State::Seq inp = grasp::RobotState::makeCommand(trajectory->second);
		// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
		select(grasp::to<Data>(dataPtr)->stratType);
		execute(dataPtr, inp);
		return;
	}
	case 'N':
	{
		std::stringstream prefix;
		prefix << std::fixed << std::setprecision(3) << "./data/boris/coke/ftsensors" << "-";
		contact = false;
		record = false;

		auto strTorquesDesc = [=](std::ostream& ostr) {
			ostr << "timestamp" << grasp::to<Data>(data)->sepField << "contact" << grasp::to<Data>(data)->sepField << "thumb_0" << grasp::to<Data>(data)->sepField << "thumb_1" << grasp::to<Data>(data)->sepField << "thumb_2" << grasp::to<Data>(data)->sepField << "thumb_3" << grasp::to<Data>(data)->sepField <<
				"index_0" << grasp::to<Data>(data)->sepField << "index_1" << grasp::to<Data>(data)->sepField << "index_2" << grasp::to<Data>(data)->sepField << "index_3" << grasp::to<Data>(data)->sepField <<
				"middle_0" << grasp::to<Data>(data)->sepField << "middle_1" << grasp::to<Data>(data)->sepField << "middle_2" << grasp::to<Data>(data)->sepField << "middle_3" << grasp::to<Data>(data)->sepField <<
				"ring_0" << grasp::to<Data>(data)->sepField << "ring_1" << grasp::to<Data>(data)->sepField << "ring_2" << grasp::to<Data>(data)->sepField << "ring_3" << grasp::to<Data>(data)->sepField <<
				"pinky_0" << grasp::to<Data>(data)->sepField << "pinky_1" << grasp::to<Data>(data)->sepField << "pinky_2" << grasp::to<Data>(data)->sepField << "pinky_3" << grasp::to<Data>(data)->sepField;
		};

		// assumption: the robot is in the final pose with all finger open (pre-grasp)
		Controller::State s = robot->recvState().command;// trajectory[0];

		// current configuration (fingers opened)
		Controller::State pregrasp = robot->getController()->createState();
		robot->getHandCtrl()->getController()->setToDefault(pregrasp);
		pregrasp.cpos = s.cpos;

		// compute starting pose
		// arm chain and joints pointers
		const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
		const golem::Configspace::Range armJoints = robot->getStateArmInfo().getJoints();
		const golem::Controller::State::Info handInfo = robot->getStateHandInfo();
		// Compute a sequence of targets corresponding to the transformed arm end-effector
		GenWorkspaceChainState gwcs;
		robot->getController()->chainForwardTransform(pregrasp.cpos, gwcs.wpos);
		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
		gwcs.t = pregrasp.t;
		//	gwcs.wpos[armChain].p.x += 0.045;
		gwcs.wpos[armChain].p.y -= 0.07;

		// initial pose
		Controller::State cstart = pregrasp;
		{
			// lock controller
			golem::CriticalSectionWrapper csw(robot->getControllerCS());
			// Find initial target position
			if (!robot->getPlanner()->findTarget(pregrasp, gwcs, cstart))
				context.write("findTarget(): Unable to find initial target configuration");
		}

		// update arm configurations and compute average error
		grasp::RBDist err;
		WorkspaceChainCoord wcc;
		robot->getController()->chainForwardTransform(cstart.cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], robot->getController()->getChains()[armChain]->getReferencePose());
		err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(gwcs.wpos[armChain])));
		context.write("findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);

		auto markContact = [&]() { if (waitKey(10) == ' ') contact = !contact; };

		auto selectFinger = [&]() -> size_t {
			context.write("[0] thumb\n[1] index\n[2] middle\n[3] ring\n[4] pinky\n");
			I32 index = 0;
			readNumber("Enter index: ", index);
			return index;
		};

		auto closeFinger = [&](Controller::State &state, const golem::Chainspace::Index &index) {
//			context.write("closing finger");
			for (auto i = handInfo.getJoints(index).begin() + 2; i != handInfo.getJoints(index).end(); ++i)
				state.cpos[i] = 1.0;
		};
		auto openFinger = [&](Controller::State &state, const golem::Chainspace::Index &index) {
//			context.write("opening finger");
			for (auto i = handInfo.getJoints(index).begin() + 2; i != handInfo.getJoints(index).end(); ++i)
				state.cpos[i] = 0.01;
		};

		const SecTmReal dur = SecTmReal(15.0);
		auto move2start = [&](bool &breakpoint) {
			auto breaking = [&]() { if (waitKey(10) == 27) breakpoint = true; };
			Controller::State::Seq seq, out;
//			robot->findTrajectory(robot->recvState().command, &cstart, nullptr, 0, seq);
			seq.push_back(pregrasp);
			seq.push_back(cstart);
			profile(dur, seq, out, true);
			robot->sendTrajectory(out);
			// repeat every send waypoint until trajectory end
			context.write("Press (ESC) - recording %s contact %s\r", record ? "Y" : "N", contact ? "Y" : "N");
			for (U32 i = 0; robot->waitForBegin(); ++i) {
				if (universe.interrupted())
					throw grasp::Interrupted();
				if (robot->waitForEnd(0))
					break;
				breaking();
			}
			context.write("                                                                               \r");
		};
		auto poke = [&]() {
			Controller::State::Seq seq, out;
//			robot->findTrajectory(robot->recvState().command, &pregrasp, nullptr, 0, seq);
			seq.push_back(cstart);
			seq.push_back(pregrasp);
			profile(SecTmReal(5.0), seq, out, true);
			robot->sendTrajectory(out);
			// repeat every send waypoint until trajectory end
			for (U32 i = 0; robot->waitForBegin(); ++i) {
				if (universe.interrupted())
					throw grasp::Interrupted();
				if (robot->waitForEnd(0))
					break;
				markContact();
				if (i % 10 == 0) {
					context.write("State #%d - recording %s contact %s\r", i, record ? "Y" : "N", contact ? "Y" : "N");
				}
			}
			context.write(" \r");
		};

		auto stop = [&](I32 &counter) -> bool {
			const int key = waitKey("QC", "Press to (Q)uit or to (C)ontinue");
			if (key == 'Q')
				return true;
			counter = 0;
			return false;
		};
		auto quit = [&]() -> bool {
			const int key = waitKey("QC", "Press to (Q)uit the program or to (C)ontinue with another finger");
			if (key == 'Q')
				return true;
			return false;
		};

		for (;;) {
			I32 index = selectFinger();

			I32 idx = 0;
			for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
				//			context.write("idx=%d index=%d\n", idx, index);
				if (idx == index) {
					openFinger(cstart, i);
					openFinger(pregrasp, i);
				}
				else {
					closeFinger(cstart, i);
					closeFinger(pregrasp, i);
				}
				++idx;
			}

//			Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cstart), manipulator->getPose(cstart).toMat34());
//			renderHand(cstart, bounds, true);
			// writing data into a text file, open file
			std::string name(prefix.str().c_str());
			readString("Enter name file:", name);
			const std::string dataIndexPathRaw = grasp::makeString("%s_raw.txt", name.c_str());
			const std::string dataIndexPathFiltered = grasp::makeString("%s_filtered.txt", name.c_str());

			//	if (enableSimContact) {
			// raw data from drl hand FT
			dataFTRaw.open(dataIndexPathRaw);
			// writing data into a text file, prepare headers
			dataFTRaw << "#index" << grasp::to<Data>(data)->sepField;
			strTorquesDesc(dataFTRaw);
			dataFTRaw << std::endl;
			// filtered data from DLR hand FT
			dataFTFiltered.open(dataIndexPathFiltered);
			// writing data into a text file, prepare headers
			dataFTFiltered << "#index" << grasp::to<Data>(data)->sepField;
			strTorquesDesc(dataFTFiltered);
			dataFTFiltered << std::endl;

			bool breakPoint = false;
			for (;;) {
				move2start(breakPoint);
				if (breakPoint)
					break;
				record = true;
				poke();
				record = false;
				contact = false;
			}
			dataFTRaw.close();
			dataFTFiltered.close();
			if (quit())
				break;
		}

		return;
	}
	case 'O':
	{
		brecord = !brecord;
		context.write("Screencapture recording %s\n", brecord ? "ON" : "OFF");
		universe.postScreenCaptureFrames(brecord  ? - 1 : 0);

		//auto breakPoint = [&]() -> bool { if (waitKey(10) == 27) { record = !record;  return true; } return false; };
		//torquesNoContact.open("./data/nocontact.log");
		//torquesInContact.open("./data/contact.log");
		//for (;;) {
		//	switch (waitKey("CNB", "Press a key to select (N)oContact/(C)ontact/(B)reak...")) {
		//	case 'N':
		//		inContact = false;
		//		break;
		//	case 'C':
		//		inContact = true;
		//		break;
		//	case 'B':
		//		return;
		//	}
		//	for (; !breakPoint();){

		//	}
		//	
		//}

//		const grasp::Cloud::PointSeq points = *objectPointCloudPtr.get();//getPoints(dataPtr)->second;
		//Collision collision(context, *manipulator);
		//grasp::Manipulator::Pose pose = manipulator->getPose(robot->recvState().config);
		//Collision::Waypoint waypoint;
		//waypoint.points = 1000000;
		//std::vector<Configspace::Index> joints;
		//context.debug("Evaluate on robot's bounds (point size=%d)\n", points.size());
		//(void)collision.evaluate(robot, waypoint, points, rand, pose, joints, true);
		//context.debug("Evaluate on collision bounds\n");
		//(void)collision.evaluate(waypoint, points, rand, pose, true);
		return;
	}
	//case '0':
	//{
	//	forcereadersilent = !forcereadersilent;
	//	return;
	//	//context.write("%s\n", trialData->toString(*robot->getController()).c_str());
	//	Controller::State state = robot->getController()->createState();
	//	robot->getController()->lookupState(SEC_TM_REAL_MAX, state);
	//	printState(state, robot->getController()->getStateInfo().getJoints().begin(), robot->getController()->getStateInfo().getJoints().end(), "Current robot pose", true);
	//	return;
	//}
	//case '!':
	//{
	//	WaypointGenerator::Seq generators = robot->getGlobalPathGenerators();
	//	for (WaypointGenerator::Seq::const_iterator i = generators.begin(); i != generators.end(); ++i) {
	//		Controller::State state = robot->getController()->createState();
	//		state.cpos = i->mean;
	//		Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(state), manipulator->getPose(state).toMat34());
	//		renderWaypoints(bounds, false);
	//	}

	//	//singleGrasp = !singleGrasp;
	//	//context.write("Single grasp attempt %s\n", singleGrasp ? "ON" : "OFF");
	//	return;
	//}
	case '-':
	{
		if ((dynamic_cast<RagGraphPlanner&>(*robot->getPlanner())).getLocalPath().empty())
			return;
		Waypoint::Seq localPath = (dynamic_cast<RagGraphPlanner&>(*robot->getPlanner())).getLocalPath();
		context.write("Graph size=%d)\n", localPath.size());

		for (size_t i = 0; i < 50; ++i) {
			Waypoint w = localPath[size_t(rand.next()) % localPath.size()];
			Controller::State state = robot->getController()->createState();
			state.cpos = w.cpos;
			Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(state), manipulator->getPose(state).toMat34());
			renderHand(state, bounds, (i == 0) ? true : false);
		}
		return;

	}
	} // switch
	ShapePlanner::function(dataPtr, key);
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
	context.write("%s torques <", label.c_str());
	for (Configspace::Index i = begin; i != end; ++i) {
		const size_t idx = k - begin;
		context.write("%f ", forces[idx]);
	}
	context.write(">\n");

}

//void RagPlanner::testTransformation(const golem::Mat34 &teachFrame, const golem::Mat34 &modelFrame, const golem::Mat34 &queryFrame, const grasp::RobotState::List &action) {
//	const golem::Chainspace::Index armChain = robot->getStateArmInfo().getChains().begin();
//	context.write("RagPlanner::testTransformation(): approaching trajetory (list)\n");
//	printTrajectory(makeCommand(action), robot->getStateInfo().getJoints().begin(), robot->getStateInfo().getJoints().end());	
//	for (grasp::RobotState::List::const_iterator i = action.begin(); i != action.end(); ++i) {
//		GenWorkspaceChainState gwcs;
//		robot->getController()->chainForwardTransform(i->config.cpos, gwcs.wpos);
//		gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], robot->getController()->getChains()[armChain]->getReferencePose()); // 1:1
//		// define the grasp frame
//		Mat34 poseFrameInv, graspFrame, graspFrameInv;
//		poseFrameInv.setInverse(gwcs.wpos[armChain]);
//		graspFrame.multiply(poseFrameInv, teachFrame * modelFrame);
//		graspFrameInv.setInverse(graspFrame);
//		gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
//		context.write("trnTrajectory(): grasp frame at model <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
//		gwcs.wpos[armChain].multiply(queryFrame, gwcs.wpos[armChain]); // new waypoint frame
//		context.write("trnTrajectory(): grasp frame at new query <%f %f %f>\n", gwcs.wpos[armChain].p.x, gwcs.wpos[armChain].p.y, gwcs.wpos[armChain].p.z);
//		
//		if (i == action.begin())
//			graspFrameQ = gwcs.wpos[armChain];
//		else
//		pregraspFrameQ = gwcs.wpos[armChain];
//	}
//	handBounds.clear();
//	renderData(currentDataPtr);
//}


//------------------------------------------------------------------------------

void spam::XMLData(RagPlanner::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	// load ShapePlanner description
	spam::XMLData((ShapePlanner::Desc&)val, context, xmlcontext, create);

	// Load spam::Robot
	spam::XMLData((Robot::Desc&)*val.robotDesc, context, xmlcontext, create);

	// Load RagPlanner
	xmlcontext = xmlcontext->getContextFirst("rag_planner");

	golem::XMLData("planning_uncertainty", val.uncEnable, xmlcontext);
	golem::XMLData("single_grasp_attempt", val.singleGrasp, xmlcontext);
	golem::XMLData("withdraw_to_home_pose", val.withdrawToHomePose, xmlcontext);
	val.sampleAppearance.xmlData(xmlcontext->getContextFirst("point_appearance"), create);

	spam::XMLData(*val.objCollisionDescPtr, xmlcontext->getContextFirst("collision"), create);

//	try {
////		XMLData(val.queryPointsTrn, xmlcontext->getContextFirst("query_points_trn"), create);
//	}
	//catch (const golem::MsgXMLParser &msg) { context->notice("%s\n", msg.what()); }
	//Belief::Desc* pBeliefDesc(new Belief::Desc);
	//(grasp::RBPose::Desc&)*pBeliefDesc = *((PosePlanner::Desc&)val).pRBPoseDesc;
	//val.pRBPoseDesc.reset(pBeliefDesc);
	//spam::XMLData((Belief::Desc&)*val.pRBPoseDesc, xmlcontext, create);
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
#endif // _SPAM_RAG_MAIN_