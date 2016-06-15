/** @file RAGPlanner.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Planner/Data.h>
#include <Golem/Planner/GraphPlanner/Data.h>
//#include <Golem/Phys/PhysScene.h>
#include <Spam/App/R2GPlanner/R2GPlanner.h>
#include <algorithm>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>
#include <boost/tokenizer.hpp>
#include <Grasp/Contact/OptimisationSA.h>

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

grasp::data::Data::Ptr spam::R2GPlanner::Data::Desc::create(golem::Context &context) const {
	grasp::data::Data::Ptr data(new R2GPlanner::Data(context));
	static_cast<R2GPlanner::Data*>(data.get())->create(*this);
	return data;
}

spam::R2GPlanner::Data::Data(golem::Context &context) : PosePlanner::Data(context), owner(nullptr) {
}

void spam::R2GPlanner::Data::create(const Desc& desc) {
	PosePlanner::Data::create(desc);

	triggered = 0;
	replanning = false;
	release = false;
}

spam::R2GPlanner::Data* spam::R2GPlanner::Data::clone() const {
	return new Data(*this);
}


void spam::R2GPlanner::Data::setOwner(grasp::Manager* owner) {
	PosePlanner::Data::setOwner(owner);
	this->owner = grasp::is<R2GPlanner>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "pacman::Demo::Data::setOwner(): unknown data owner");
}


void spam::R2GPlanner::Data::createRender() {
	PosePlanner::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->getCS());
		owner->debugRenderer.render();
		owner->sampleRenderer.render();
	}
}

void spam::R2GPlanner::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap) {
	PosePlanner::Data::load(prefix, xmlcontext, handlerMap);
}

void spam::R2GPlanner::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	PosePlanner::Data::save(prefix, xmlcontext);
}

//------------------------------------------------------------------------------

void R2GPlanner::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
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

	//sensorBundleDesc->load(context, xmlcontext);
	try {
		golem::XMLData("active_ctrl", activeCtrlStr, const_cast<golem::XMLContext*>(xmlcontext));
	}
	catch (const Message& msg) {
		context.info("Active control non selected. Used default %s", activeCtrlStr.c_str());
	}
	//sensorBundleDesc.load(context, xmlcontext);
	spam::XMLData(*objCollisionDescPtr, xmlcontext->getContextFirst("sensor_bundle collision"));
}

//------------------------------------------------------------------------------

R2GPlanner::R2GPlanner(Scene &scene) : PosePlanner(scene) {
}

bool R2GPlanner::create(const Desc& desc) {
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
	initTrjDur = desc.initTrjDur;
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
	contactOccured = false;

	contact = false;
	record = false;
	brecord = false;

	// ACTIVE CONTROLLER
//	const grasp::ActiveCtrl::Map::const_iterator armHandCtrlPtr = activectrlMap.find("ArmHandForce+ArmHandForce");
	const grasp::ActiveCtrl::Map::const_iterator armHandCtrlPtr = activectrlMap.find(desc.activeCtrlStr);
	if (armHandCtrlPtr == activectrlMap.end())
		throw Message(Message::LEVEL_ERROR, "R2GPlanner::create(): active ctrl %s not found", desc.activeCtrlStr.c_str());
	//	armHandForce = dynamic_cast<ArmHandForce*>(&*armHandCtrlPtr->second);
	armHandForce = dynamic_cast<ActiveTouchCtrl*>(&*armHandCtrlPtr->second);
	if (!armHandForce)
		throw Message(Message::LEVEL_ERROR, "R2GPlanner::create(): active ctrl %s is invalid", desc.activeCtrlStr.c_str());
	armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::Mode::MODE_DISABLED);
	armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::Mode::MODE_ENABLED);
	armMode = armHandForce->getArmCtrl()->getMode();
	handMode = armHandForce->getHandCtrl()->getMode();
	context.write("Active ctrl %s mode [arm/hand]: %s/%s\n", desc.activeCtrlStr.c_str(), ActiveCtrlForce::ModeName[armMode], ActiveCtrlForce::ModeName[handMode]);

	// SET FT SENSORS AND GUARDS
	fLimit = desc.fLimit;
	context.write("Thumb limits [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
		fLimit[0], fLimit[1], fLimit[2], fLimit[3], fLimit[4], fLimit[5]);
	context.write("Index limits [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
		fLimit[6], fLimit[7], fLimit[8], fLimit[9], fLimit[10], fLimit[11]);
	context.write("Middle limits [%.3f %.3f %.3f %.3f %.3f %.3f]\n",
		fLimit[12], fLimit[13], fLimit[14], fLimit[15], fLimit[16], fLimit[17]);
	
	collisionPtr.reset();
	collisionPtr = desc.objCollisionDescPtr->create(*this->manipulator.get());

	Sensor::Seq sensorSeq = armHandForce->getSensorSeq();
	// NOTE: skips the sensor at the wrist
	for (auto i = sensorSeq.begin(); i != sensorSeq.end(); ++i) {
		//		if (i == sensorSeq.begin()) continue;
		FT* sensor = grasp::is<FT>(*i);
		if (sensor)
		if (i == sensorSeq.begin())
			wristFTSensor = sensor;
		else
			ftSensorSeq.push_back(sensor);
		else
			context.write("FT sensor is not available\n");
	}
	// set FT guard for the thumb
	Chainspace::Index chain = getPlanner().handInfo.getChains().begin();
	FTGuard::Desc thumbFTDesc = FTGuard::Desc();
	thumbFTDesc.chain = HandChain::THUMB;
	thumbFTDesc.jointIdx = getPlanner().handInfo.getJoints(chain++).end() - 1;
	thumbFTDesc.setLimits(&fLimit[0]);
	ftGuards.push_back(thumbFTDesc.create());
	// set FT guard for the index
	FTGuard::Desc indexFTDesc = FTGuard::Desc();
	indexFTDesc.chain = HandChain::INDEX;
	indexFTDesc.jointIdx = getPlanner().handInfo.getJoints(chain++).end() - 1;
	indexFTDesc.setLimits(&fLimit[6]);
	ftGuards.push_back(indexFTDesc.create());
	// set FT guard for the thumb
	FTGuard::Desc middleFTDesc = FTGuard::Desc();
	middleFTDesc.chain = HandChain::MIDDLE;
	middleFTDesc.jointIdx = getPlanner().handInfo.getJoints(chain++).end() - 1;
	middleFTDesc.setLimits(&fLimit[12]);
	ftGuards.push_back(middleFTDesc.create());

	bool simulatedForces = true, ftsensors = false;
	armHandForce->setSensorForceReader([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws
		if (!enableForceReading)
			return;

		if (simulatedForces && !collisionPtr->getPoints().empty()) {
			for (auto i = 0; i < force.size(); ++i)
				force[i] = collisionPtr->getFTBaseSensor().ftMedian[i] + (2 * rand.nextUniform<Real>()*collisionPtr->getFTBaseSensor().ftStd[i] - collisionPtr->getFTBaseSensor().ftStd[i]);

			golem::Controller::State dflt = manipulator->getController().createState();
			manipulator->getController().setToDefault(dflt);

			manipulator->getController().lookupState(golem::SEC_TM_REAL_MAX, dflt);
			dflt.cvel.setToDefault(manipulator->getController().getStateInfo().getJoints());
			dflt.cacc.setToDefault(manipulator->getController().getStateInfo().getJoints());

			(void)collisionPtr->simulateFT(debugRenderer, desc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(dflt), force, false);
		}
		// read from the state variable (if supported)
		else {
			if (ftsensors) {
				size_t k = 0;
				for (auto i = ftSensorSeq.begin(); i < ftSensorSeq.end(); ++i) {
					grasp::FT::Data data;
					(*i)->read(data);
					data.wrench.v.getColumn3(&force[k]);
					data.wrench.w.getColumn3(&force[k + 3]);
					k += 6;
				}

			}
			else {
				const ptrdiff_t forceOffset = armHandForce->getHand()->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
				if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
					for (Configspace::Index j = state.getInfo().getJoints().begin(); j < state.getInfo().getJoints().end(); ++j) {
						const size_t k = j - state.getInfo().getJoints().begin();
						force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
					}
				}
			}
		}
	}); // end robot->setSimHandForceReader

	armHandForce->setHandForceReader([&](const golem::Controller::State& state, grasp::RealSeq& force) { // throws
		if (!enableForceReading)
			return;

		size_t k = 0;
		U32 contacts = golem::numeric_const<U32>::ZERO;
		handFilteredForce = force; // sensorBundlePtr->getFilteredForces(); // sensorBundlePtr.get() ? sensorBundlePtr->getFilteredForces() : handFilteredForce;

		for (auto i = ftGuards.begin(); i < ftGuards.end(); ++i) {
			(*i)->setColumn6(&handFilteredForce[k]);
			if ((*i)->checkContacts())
				++contacts;
//			(*i)->str(context);
			k += 6;
		}

		if (contacts > 0) {
			for (auto i = ftGuards.begin(); i < ftGuards.end(); ++i)
				(*i)->str(context);
			std::stringstream ss;
			for (auto i = ftSensorSeq.begin(); i < ftSensorSeq.end(); ++i) {
				FT::Data data;
				(*i)->read(data);
				Twist wrench;
				SecTmReal t;
				(*i)->readSensor(wrench, t);
				
				ss << "FT sensor " << (*i)->getID().c_str() << "data.wrench [" << data.wrench.v.x << " " << data.wrench.v.y << " " << data.wrench.v.z << " " << data.wrench.w.x << " " << data.wrench.w.y << " " << data.wrench.w.z <<
					"]\nRaw wrench [" << wrench.v.x << " " << wrench.v.y << " " << wrench.v.z << " " << wrench.w.x << " " << wrench.w.y << " " << wrench.w.z << "]\n";
			}
			//context.write("%s\n", ftGuards[i].str().c_str());
			throw Message(Message::LEVEL_NOTICE, "handForceReader(): Triggered guard(s) %d.\n%s", contacts, ss.str().c_str());
		}
	}); // end robot->setHandForceReader

	armHandForce->setEmergencyModeHandler([=]() {
//		handForces = getHandForceVec();
		enableForceReading = false;
		contactOccured = true;
		armHandForce->getArmCtrl()->setMode(armMode);
		armHandForce->getHandCtrl()->setMode(handMode);
	}); // end robot->setEmergencyModeHandler

	uncEnable = desc.uncEnable;
	singleGrasp = desc.singleGrasp;
	withdrawToHomePose = desc.withdrawToHomePose;
	posterior = true;

	ragDesc = desc;

	handBounds.clear();
	robotStates.clear();

	isGrasping = false;

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
		};
		to<Data>(dataCurrentPtr)->createRender();
		// set the simulated object
		if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
			//sensorBundlePtr->getCollisionPtr()->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
		}

		grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(item->second.get());
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		//Controller::State::Seq inp = trajectory->getWaypoints();
		grasp::Waypoint::Seq inp = trajectory->getWaypoints();
		if (inp.size() < 3)
			throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");
		
		to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
		select(to<Data>(dataCurrentPtr)->stratType);
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
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
			if (!execute(dataCurrentPtr, inp))
				return;
					
			// lifting
			enableForceReading = false;
			//grasp::to<Data>(dataPtr)->actionType = action::IG_PLAN_LIFT;
			context.write("execute trajectory (%s)\n", actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
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

//	menuCmdMap.insert(std::make_pair("X", [=]() {
//		// setup initial variables
//		const Controller::State home = lookupState();
//
//		//--------------------------------------------------------------------//
//		// UTILITY FUNCTIONS
//		//--------------------------------------------------------------------//
//		auto strategy = [&](const Strategy &strat) -> std::string {
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
//			switch (strat) {
//			case NONE_STRATEGY:
//			//	strat = Strategy::ELEMENTARY;
//			//	break;
//			//case ELEMENTARY:
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
//		auto reset = [&]() {
//			enableForceReading = false;
//			pHeuristic->enableUnc = false;
//			pHeuristic->setPointCloudCollision(false);
//			gotoConfig(home);
//		};
//		auto resetDataPtr = [&](Data::Map::iterator& dataPtr) {
//			to<Data>(dataPtr)->queryPoints.clear();
//			to<Data>(dataPtr)->queryTransform.setId();
//			to<Data>(dataPtr)->queryFrame.setId();
//			to<Data>(dataPtr)->poses.clear();
//			to<Data>(dataPtr)->hypotheses.clear();
//			to<Data>(dataPtr)->simulateObjectPose.clear();
//			};
//		std::ofstream logFile("./data/boris/experiments/output.log");
//		std::ofstream elementaryLog("./data/boris/experiments/elementary.log");
//		std::ofstream mycroftLog("./data/boris/experiments/mycroft.log");
//		std::ofstream ireneLog("./data/boris/experiments/irene.log");
//
//		grasp::to<Data>(dataCurrentPtr)->stratType = Strategy::NONE_STRATEGY;
//		grasp::to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
//		grasp::to<Data>(dataCurrentPtr)->replanning = false;
//		grasp::data::Item::Map::iterator modelContactPtr, modelPtr, queryGraspPtr, queryGraspTrjPtr;
//		grasp::data::Item::Ptr queryGraspTrj;
//		U32 modelViews = modelScanPoseSeq.size(), queryViews = queryScanPoseSeq.size(), trials = 5;
//		const U32 maxFailures = 2, maxIterations = 5;
//		// collects results as string
//		std::string results;
//
//		// command keys
//		std::string createModelCmd("PM");
//		std::string createQueryCmd("PQ");
//		std::string itemTransCmd("IT");
//		std::string itemConvCmd("IC");
//		std::string dataSaveCmd("DS");
//		std::string itemRemoveCmd("IR");
//
//		context.write("================================================================================================================================\n");
//		logFile << "================================================================================================================================\n";
//		//--------------------------------------------------------------------//
//		// CREATE A MODEL POINT CLOUD
//		//--------------------------------------------------------------------//
//		// object name
//		std::string objectName;
//		readString("Enter object name: ", objectName);
//		// create a model for the object
//		context.write("Create a model...\n");
//		for (; to<Data>(dataCurrentPtr)->modelPoints.empty();)
//			executeCmd(createModelCmd); // move the robot to the home pose after scanning
//		//reset();
//		context.write("done.\n");
//
//		//--------------------------------------------------------------------//
//		// CREATE A PREDICTIVE MODEL FOR THE GRASP
//		//--------------------------------------------------------------------//
//		context.write("Create a predictive contact model for the grasp [%s]...\n", modelGraspItem.c_str());
//		if (option("YN", "Create a new contact model? (Y/N)") == 'Y') {
//			dataItemLabel = modelGraspItem;
//			executeCmd(itemTransCmd);
//			modelGraspItem = dataItemLabel;
//		}
//		context.write("done.\n");
//		//--------------------------------------------------------------------//
//		// TRIALS
//		//--------------------------------------------------------------------//
//		itemPerformedTrj.clear();
//		for (U32 trial = 0; trial < trials; ++trial) {
//			for (;;) {
//				//--------------------------------------------------------------------//
//				// TRIAL HEADER
//				//--------------------------------------------------------------------//
//				// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
//				select(grasp::to<Data>(dataCurrentPtr)->stratType);
//				// stop internal loop if all the strategies have been executed. increase trial
//				if (to<Data>(dataCurrentPtr)->stratType == Strategy::NONE_STRATEGY)
//					break;
//
//				results = grasp::makeString("%u\t%u\t%u", modelViews, queryViews, trial + 1);
//				context.write("execute %s trajectory (%s)\n", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//				logFile << grasp::makeString("Strategy: %s\n", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str());
//				// Setup trial data
//				TrialData::Ptr tdata = createTrialData();
//				tdata->grasped = false;
//				tdata->silent = true;
//				tdata->setup(context, rand);
//				tdata->name = grasp::makeString("trial_%s_v%u_0%u", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), queryViews, trial+1); //"trial_v1_01"; 
//				// object name
//				tdata->object = objectName;
//				tdata->dirPath = grasp::makeString("%s%s/%s/", this->trial->path.c_str(), tdata->object.c_str(), tdata->name.c_str());
//				tdata->path = grasp::makeString("%s%s/%s%s", this->trial->path.c_str(), tdata->object.c_str(), tdata->name.c_str(), this->trial->extTrial.c_str());
//				context.write("Object name = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nModel view(s) = %u\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
//					tdata->object.c_str(), tdata->name.c_str(), modelViews, queryViews, trial, tdata->path.c_str());
//				logFile << grasp::makeString("Object name = %s\n-----------------START TRIAL-----------------------------------------\nTrial name = %s\nModel view(s) = %u\nQuery view(s) = %u\nTrial number = %u\nTrial path = %s\n\n",
//					tdata->object.c_str(), tdata->name.c_str(), modelViews, queryViews, trial, tdata->path.c_str());
//				trialPtr = getTrialData().insert(getTrialData().begin(), TrialData::Map::value_type(grasp::to<TrialData>(tdata)->path, tdata));
//
//				//--------------------------------------------------------------------//
//				// CREATE A QUERY POINT CLOUD
//				//--------------------------------------------------------------------//
//				// create a query for the object
//				context.write("Create a query...\n");
//				resetDataPtr(dataCurrentPtr);
//				for (; to<Data>(dataCurrentPtr)->queryPoints.empty();)
//					executeCmd(createQueryCmd);
//				// set the simulated object
//				if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
//					//sensorBundlePtr->getCollisionPtr()->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
//					collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
//					objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
//				}
//				reset(); // move the robot to the home pose after scanning
//				context.write("done.\n");
//
//				context.write("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(dataCurrentPtr)->queryPoints.size(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
//				logFile << grasp::makeString("\nModel points = %u, query points = %u, coverage = %.2f (percent)\n", modelPoints.size(), grasp::to<Data>(dataCurrentPtr)->queryPoints.size(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100);
//
//				//--------------------------------------------------------------------//
//				// CREATE A PREDICTIVE QUERY FOR THE GRASP
//				//--------------------------------------------------------------------//
//				// generate features
//				grasp::data::Transform* transform = is<grasp::data::Transform>(queryGraspHandler);
//				if (!transform)
//					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", queryGraspHandler->getID().c_str());
//
//
//				grasp::data::Item::List list;
//				//for (auto i = modelGraspItemSeq.begin(); i != modelGraspItemSeq.end(); ++i) {
//				grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelGraspItem);
//				if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
//					throw Message(Message::LEVEL_ERROR, "R2GPlanner::estimatePose(): Does not find %s.", modelGraspItem.c_str());
//				list.insert(list.end(), ptr);
//				//}
//				// retrieve model point cloud
//				modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
//				if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
//					throw Message(Message::LEVEL_ERROR, "R2GPlanner::estimatePose(): Does not find %s.", modelItem.c_str());
//				list.insert(list.end(), modelPtr); // grasp on model
//				grasp::data::Item::Ptr queryGraspItemPtr = transform->transform(list);
//
//				// insert processed object, remove old one
//				//grasp::data::Item::Map::iterator queryGraspPtr;
//				RenderBlock renderBlock(*this);
//				{
//					golem::CriticalSectionWrapper cswData(getCS());
//					to<Data>(dataCurrentPtr)->itemMap.erase(queryGraspItem);
//					queryGraspPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(queryGraspItem, queryGraspItemPtr));
//					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryGraspPtr, to<Data>(dataCurrentPtr)->getView());
//				}
//				to<Data>(dataCurrentPtr)->createRender();
//				context.write("Transform: handler %s, inputs %s, %s...\n", queryGraspHandler->getID().c_str(), queryGraspPtr->first.c_str(), modelItem.c_str());
//				// calculate the scor eof the desired grasp
//				grasp::data::ContactQuery *cq = is<grasp::data::ContactQuery>(queryGraspPtr);
//				const golem::Real dGraspLik = cq->getData().configs[0]->likelihood.value;
//				context.info("Desired grasp quality (active lik) %f\n", dGraspLik);
////				context.write("done.\n");
//				//--------------------------------------------------------------------//
//				// CREATE A GRASP TRAJECTORY
//				//--------------------------------------------------------------------//
//				context.write("Convert [%s]: handler %s, input %s...\n", queryItemTrj.c_str(), queryHandlerTrj->getID().c_str(), queryGraspPtr->first.c_str());
//				grasp::data::Convert* convert = is<grasp::data::Convert>(queryGraspPtr);
//				if (!convert)
//					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Convert interface", queryHandlerTrj->getID().c_str());
//
//				// convert
//				queryGraspTrj = convert->convert(*queryHandlerTrj);
//				// insert processed object, remove old one
//				//grasp::data::Item::Map::iterator queryGraspTrjPtr;
//				RenderBlock renderBlock2(*this);
//				{
//					golem::CriticalSectionWrapper cswData(getCS());
//					to<Data>(dataCurrentPtr)->itemMap.erase(queryItemTrj);
//					queryGraspTrjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(queryItemTrj, queryGraspTrj));
//					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryGraspTrjPtr, to<Data>(dataCurrentPtr)->getView());
//				}
////				to<Data>(dataCurrentPtr)->createRender();
//				context.write("done.\n");
//
//				context.write("Transform: handler %s, input %s...\n", queryHandlerTrj->getID().c_str(), queryGraspTrjPtr->first.c_str());
//				grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(queryGraspTrjPtr);
//				if (!trajectory)
//					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Trajectory interface", queryHandlerTrj->getID().c_str());
//				grasp::Waypoint::Seq inp = trajectory->getWaypoints();
//				if (inp.size() < 3)
//					throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");
//
//				// set render to show only mean hypothesis and ground truth
//				resetDataPointers();
//				showHypothesesPointClouds = true;
//				showMeanHypothesisPointClouds = true;
//				showGroundTruth = true;
//				to<Data>(dataCurrentPtr)->createRender();
//
//				//--------------------------------------------------------------------//
//				// EXECUTE TRIAL
//				//--------------------------------------------------------------------//
//				U32 iteration = 1, failures = 0;
//				const RBCoord obj(simQueryFrame * to<Data>(dataCurrentPtr)->modelFrame);
//				for (; failures < maxFailures && iteration < maxIterations;) {
//					grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_M2Q;
//					grasp::RBCoord queryPose(grasp::to<Data>(dataCurrentPtr)->queryFrame);
//					grasp::RBDist error;
//					error.lin = obj.p.distance(queryPose.p);
//					error.ang = obj.q.distance(queryPose.q);
//					context.write("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//					context.write("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
//						obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
//						queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
//						error.lin, error.ang);
//					logFile << grasp::makeString("\nIteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//					logFile << grasp::makeString("Object pose = {(%f %f %f), (%f %f %f %f)}\nEstimate pose = {(%f %f %f), (%f %f %f %f)}\nError {lin, ang} = {%f, %f}\n",
//						obj.p.x, obj.p.y, obj.p.z, obj.q.w, obj.q.x, obj.q.y, obj.q.z,
//						queryPose.p.x, queryPose.p.y, queryPose.p.z, queryPose.q.w, queryPose.q.x, queryPose.q.y, queryPose.q.z,
//						error.lin, error.ang);
//					if (iteration == 1 && failures == 0)
//						results = grasp::makeString("%s\t%.2f\t%.6f\t%.6f", results.c_str(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100, error.lin, error.ang);
//
//					context.debug("\n----------------------------------------------------------\n");
//					if (!execute(dataCurrentPtr, inp)) { // if it fails to find a trajectory repeat 
//						++failures;
//						continue;
//					}
//
//					if (contactOccured && grasp::to<Data>(dataCurrentPtr)->stratType != Strategy::ELEMENTARY) {
//						// updates belief
//						contactOccured = false;
//						updateAndResample(dataCurrentPtr);
//						enableForceReading = false;
//						continue;
//					}
//
//					grasp::to<Data>(dataCurrentPtr)->actionType = action::GRASP;
//					context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//					logFile << grasp::makeString("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//					if (!execute(dataCurrentPtr, inp)) {// not successfully close fingers
//						++failures;
//						continue;
//					}
//					grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_LIFT;
//					context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//					logFile << grasp::makeString("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
//					if (!execute(dataCurrentPtr, inp)) {// not successfully close fingers
//						++failures;
//						continue;
//					}
//					if (option("YN", "Success? (Y/N)") == 'Y')
//						to<TrialData>(trialPtr)->grasped = true;
//					else
//						to<TrialData>(trialPtr)->grasped = false;
//
//					for (auto i = ftGuards.begin(); i != ftGuards.end(); ++i) {
//						//context.write("%s\n", (*i).str().c_str());
//						//logFile << grasp::makeString("%s\n", (*i).str().c_str());
//					}
//
//					context.write("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
//					logFile << grasp::makeString("grasped = %s, final error [lin, ang] = [%f, %f]\n", grasp::to<TrialData>(trialPtr)->grasped ? "YES" : "NO", error.lin, error.ang);
//					results = grasp::makeString("%s\t%.6f\t%.6f\t%u\t%u\n", results.c_str(), error.lin, error.ang, grasp::to<TrialData>(trialPtr)->grasped ? 1 : 0, iteration);
//					break;
//				}
//				// save results
//				context.write("%s", results.c_str());
//				logFile << grasp::makeString("%s", results.c_str());
//				switch (grasp::to<Data>(dataCurrentPtr)->stratType){
//				case Strategy::ELEMENTARY:
//					elementaryLog << grasp::makeString("%s", results.c_str());
//					break;
//				case Strategy::MYCROFT:
//					mycroftLog << grasp::makeString("%s", results.c_str());
//					break;
//				case Strategy::IR3NE:
//					ireneLog << grasp::makeString("%s", results.c_str());
//					break;
//				}
//				results = "";
//				context.debug("----------------------------------------------------------\n");
//				context.debug("%s\n", grasp::to<TrialData>(trialPtr)->toString(*controller).c_str());
//				context.debug("----------------------------------------------------------\n");
//				context.debug("----------------------------------------------------------\n");
//				if (failures < maxFailures) {
//					context.write("Sava trial in %s\n", grasp::to<TrialData>(trialPtr)->path.c_str());
//					logFile << grasp::makeString("Sava trial in %s\n", grasp::to<TrialData>(trialPtr)->path.c_str());
//				}
//				else {
//					context.write("Failed trial\n");
//					logFile << "Failed trial\n";
//				}
//				//grasp::to<TrialData>(trialPtr)->save();
//				std::string dataPath = to<TrialData>(trialPtr)->dirPath + "data.xml";
//				readPath("Enter data path to save: ", dataPath, dataExt.c_str());
//				if (getExt(dataPath).empty())
//					dataPath += dataExt;
//				to<Manager::Data>(dataCurrentPtr)->save(dataPath);
//				//executeCmd(dataSaveCmd);
//				context.write("------------------END TRIAL----------------------------------------\n");
//				logFile << "------------------END TRIAL----------------------------------------\n";
//				// reset robot
//				reset();
//			} // end strategy
//			//----------------------------------
//		} // end trial
//		context.write("================================================================================================================================\n");
//		logFile << "================================================================================================================================\n";
//		logFile.close();
//		elementaryLog.close();
//		mycroftLog.close();
//		ireneLog.close();
//
//		//finish
//		context.write("Done!\n");
//	}));

	menuCmdMap.insert(std::make_pair("H", [=]() {
		// setup initial variables
		const Controller::State home = grasp::Waypoint::lookup(*controller).command;
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
			findTrajectory(grasp::Waypoint::lookup(*controller).command, &home, nullptr, 0, seq);
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
			//sensorBundlePtr->getCollisionPtr()->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
		}

		// load trajectory
		grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(item->second.get());
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
		grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(item->second.get());
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
		findTarget(grasp::to<Data>(dataCurrentPtr)->queryTransform, grasp::to<Data>(dataCurrentPtr)->modelFrame, inp[2].state, cend);


		grasp::data::Handler* handler = modelHandler;

		// item name
		//	readString("Enter item name: ", itemName);
		grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", modelItem.c_str());

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

		//sensorBundlePtr->getCollisionPtr()->create(rand, points);
		collisionPtr->create(rand, points);
		objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(points));

		Collision::FlannDesc waypointDesc;
		Collision::Desc::Ptr cloudDesc;
		cloudDesc.reset(new Collision::Desc());
		Collision::Ptr cloud = cloudDesc->create(*manipulator);
		waypointDesc.depthStdDev = 0.01/*0.0005*/; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
		waypointDesc.radius = REAL_ZERO;

		grasp::Manipulator::Config config(inp[2].state.cpos, manipulator->getBaseFrame(inp[2].state.cpos));
		debugRenderer.reset();
//		debugAppearance.draw(points, debugRenderer);
		//RealSeq ff = {0., 0., -0.1, 0., 0., 0.};
		//ftGuards[1]->setColumn6(&ff[0]);
		//ftGuards[1]->checkContacts();
		cloud->create(rand, points);
		(void)cloud->evaluateFT(debugRenderer, waypointDesc, config, ftGuards, true);

		if (option("YN", "Resample Next? (Y/N)") == 'Y')
			return;


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
					for (Configspace::Index l = getPlanner().armInfo.getJoints().begin(); l < getPlanner().armInfo.getJoints().end(); ++l)
						w.cpos[l] = p[0] * w0.cpos[l] + p[1] * w1.cpos[l];
					for (Configspace::Index l = getPlanner().handInfo.getJoints().begin(); l < getPlanner().handInfo.getJoints().end(); ++l)
						w.cpos[l] = p[0] * w0.cpos[l] + p[1] * w1.cpos[l];

					// skip reference pose computation
					w.setup(*controller, false, true);
					const grasp::Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
					Bounds::Seq bounds = manipulator->getBounds(config.config, config.frame.toMat34());
					renderHand(*j, bounds, true);
					const bool res = (*pBelief->getHypotheses().begin())->checkNN(pHeuristic->ftDrivenDesc.checkDesc, config.config, true);
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
		context.write("done.\n");
		return;
	}));

	ragDesc = desc;

	return true;
}

//------------------------------------------------------------------------------

void R2GPlanner::render() const {
	PosePlanner::render();
//	handRenderer.reset();
	{
		golem::CriticalSectionWrapper csw(getCS());
		debugRenderer.render();
		sampleRenderer.render();
		//sensorBundlePtr->render();
	}
}

void R2GPlanner::renderHand(const golem::Controller::State &state, const Bounds::Seq &bounds, bool clear) {
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

void R2GPlanner::findTarget(const golem::Mat34 &trn, const golem::Controller::State &target, golem::Controller::State &cend, const bool lifting) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
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
		if (!getPlanner().planner->findTarget(target, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "spam::Robot::findTarget(): Unable to find initial target configuration");
	}
	//	context.write(">\n"); //std::cout << ">\n"; //context.write(">\n");
	// set fingers pose
	for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i)
		cend.cpos[i] = target.cpos[i];

	// update arm configurations and compute average error
	grasp::RBDist err;
	WorkspaceChainCoord wcc;
	controller->chainForwardTransform(cend.cpos, wcc);
	wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
	err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(gwcs.wpos[armChain])));
	context.write("Robot::findTarget(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
}

void R2GPlanner::findTarget(const golem::Mat34& queryTrn, const golem::Mat34& modelFrame, const golem::Controller::State& target, golem::Controller::State& cend, const bool lifting) {
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState gwcs;
	controller->chainForwardTransform(target.cpos, gwcs.wpos);
	gwcs.wpos[armChain].multiply(gwcs.wpos[armChain], controller->getChains()[armChain]->getReferencePose()); // 1:1

	Mat34 poseFrameInv, graspFrame, graspFrameInv;
	poseFrameInv.setInverse(gwcs.wpos[armChain]);
	graspFrame.multiply(poseFrameInv, modelFrame);
	graspFrameInv.setInverse(graspFrame);
	//gwcs.wpos[armChain].multiply(queryTrn, graspFrameInv);
	gwcs.wpos[armChain].multiply(modelFrame, graspFrameInv);
	gwcs.wpos[armChain].multiply(queryTrn, gwcs.wpos[armChain]);
	gwcs.t = target.t;
//	gwcs.wpos[armChain].p.z -= 0.007;
	if (lifting) gwcs.wpos[armChain].p.z += 0.07;

	cend = target;
	{
		// Find initial target position
		if (!getPlanner().planner->findTarget(target, gwcs, cend))
			throw Message(Message::LEVEL_ERROR, "spam::Robot::findTarget(): Unable to find initial target configuration");
	}
	//	context.write(">\n"); //std::cout << ">\n"; //context.write(">\n");
	// set fingers pose
	for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i)
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

void R2GPlanner::createTrajectory(const golem::Controller::State& begin, const golem::Controller::State* pcend, const golem::Mat34* pwend, golem::SecTmReal t, const golem::Controller::State::Seq& waypoints, golem::Controller::State::Seq& trajectory) {
	if (!pcend && !pwend)
		throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): no target specified");

	// Trajectory from initial position to end position
	for (golem::U32 i = 0; i < getPlanner().trajectoryTrials; ++i) {
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
			wend.wpos[getPlanner().armInfo.getChains().begin()] = *pwend;
			// planner debug
			//context.verbose("%s\n", plannerWorkspaceDebug(*planner, &wend.wpos).c_str());
			if (!getPlanner().planner->findTarget(begin, wend, cend))
				continue;
			// update configspace coords of the hand
			if (pcend) cend.cpos.set(getPlanner().handInfo.getJoints(), pcend->cpos);
			// error
			WorkspaceChainCoord wcc;
			controller->chainForwardTransform(cend.cpos, wcc);
			wcc[getPlanner().armInfo.getChains().begin()].multiply(wcc[getPlanner().armInfo.getChains().begin()], controller->getChains()[getPlanner().armInfo.getChains().begin()]->getReferencePose());
			grasp::RBDist err;
			err.set(grasp::RBCoord(*pwend), grasp::RBCoord(wcc[getPlanner().armInfo.getChains().begin()]));
			context.debug("Robot::findTrajectory(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
		}

		// planner debug
		//context.verbose("%s\n", plannerConfigspaceDebug(*planner, &cend.cpos).c_str());
		// Find collision-free trajectory and wait until the device is ready for new commands
		cend.t = begin.t + (t > SEC_TM_REAL_ZERO ? t : getPlanner().trajectoryDuration);
		if (getPlanner().planner->findGlobalTrajectory(begin, cend, trajectory, trajectory.begin()))
			return;// success
	}

	throw Message(Message::LEVEL_ERROR, "Robot::findTrajectory(): unable to find trajectory");
}

grasp::RBDist R2GPlanner::trnTrajectory(const golem::Mat34& actionFrame, const golem::Mat34& modelFrame, const golem::Mat34& trn, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
	// Compute a sequence of targets corresponding to the transformed arm end-effector
	GenWorkspaceChainState::Seq seq;

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
		if (!getPlanner().planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find initial target configuration");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !getPlanner().planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::createTrajectory(2): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	grasp::RBDist err;
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(getPlanner().armInfo.getJoints().begin(), getPlanner().armInfo.getJoints().end(), ctrajectory[i]);
		// error
		WorkspaceChainCoord wcc;
		controller->chainForwardTransform(trajectory.back().cpos, wcc);
		wcc[armChain].multiply(wcc[armChain], controller->getChains()[armChain]->getReferencePose());
		err.add(err, grasp::RBDist(grasp::RBCoord(wcc[armChain]), grasp::RBCoord(seq[i].wpos[armChain])));
	}
	context.debug("Robot::createTrajectory(2): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);

	return err;
}

grasp::RBDist R2GPlanner::findTrnTrajectory(const golem::Mat34& trn, const golem::Controller::State& startPose, golem::Controller::State::Seq::const_iterator begin, golem::Controller::State::Seq::const_iterator end, golem::Controller::State::Seq& trajectory) {
	if (begin == end)
		throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Empty input trajectory");
	// arm chain and joints pointers
	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
	const golem::Configspace::Range armJoints = getPlanner().armInfo.getJoints();
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
		if (!getPlanner().planner->findTarget(*begin, seq[0], cend))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find initial target configuration");
		// compute the initial trajectory to move thee robot from current pose to the beginning of the approach trajectory
		if (!getPlanner().planner->findGlobalTrajectory(startPose, cend, initTrajectory, initTrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find initial trajectory");
		// Find remaining position sequence
		if (seq.size() == 1)
			ctrajectory.push_back(cend);
		else if (seq.size() > 1 && !getPlanner().planner->findLocalTrajectory(cend, ++seq.begin(), seq.end(), ctrajectory, ctrajectory.end()))
			throw Message(Message::LEVEL_ERROR, "Robot::transformTrajectory(): Unable to find trajectory");
	}
	// update arm configurations and compute average error
	trajectory.insert(trajectory.end(), initTrajectory.begin(), initTrajectory.end());
	grasp::RBDist err;
	Controller::State::Seq::const_iterator j = begin;
	for (size_t i = 0; i < ctrajectory.size(); ++i, ++j) {
		// copy config
		trajectory.push_back(*j);
		trajectory.back().set(getPlanner().armInfo.getJoints().begin(), getPlanner().armInfo.getJoints().end(), ctrajectory[i]);
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

Real R2GPlanner::distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
	Real dist = REAL_ZERO;
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		dist += distance[i] * Math::sqr(prev[i] - next[i]);
	return Math::sqrt(dist);
}

Real R2GPlanner::distCoord(Real prev, Real next) const {
	return Math::abs(prev - next);
}

bool R2GPlanner::distCoordEnabled(const Configspace::Index& index) const {
	return true;
}

//------------------------------------------------------------------------------

void R2GPlanner::profile(golem::SecTmReal duration, const golem::Controller::State::Seq& inp, golem::Controller::State::Seq& out, const bool silent) const {
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

void R2GPlanner::perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	//golem::Controller::State::Seq initTrajectory;
	//findTrajectory(lookupState(), &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	//golem::Controller::State::Seq out = initTrajectory;
	//out.insert(out.end(), trajectory.begin(), trajectory.end());

	//golem::Controller::State::Seq completeTrajectory;
	//profile(initTrjDur, out, completeTrajectory, true);

	context.write("R2GPlanner::perform()");
	// create trajectory item
	grasp::data::Item::Ptr itemTrajectory;
	grasp::data::Handler::Map::const_iterator handlerPtr = handlerMap.find(getPlanner().trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unknown default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	grasp::data::Handler* handler = is<grasp::data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): invalid default trajectory handler %s", getPlanner().trajectoryHandler.c_str());
	itemTrajectory = handler->create();
	grasp::data::Trajectory* trajectoryIf = is<grasp::data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to create trajectory using handler %s", getPlanner().trajectoryHandler.c_str());
	trajectoryIf->setWaypoints(grasp::Waypoint::make(trajectory, trajectory)/*grasp::Waypoint::make(trajectory, trajectory)*/);

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
			const grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(itemLabelTmp, itemTrajectory));
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
	recordingStart(data, item, true);
	recordingWaitToStart();
	const SecTmReal initRecTime = context.getTimer().elapsed();

	// send trajectory
	sendTrajectory(trajectory);

	bool record2 = brecord ? true : false;
	//Controller::State::Seq robotPoses; robotPoses.clear();
	//TwistSeq ftSeq; ftSeq.clear();
	//std::vector<grasp::RealSeq> forceInpSensorSeq;
	//// repeat every send waypoint until trajectory end
	//for (U32 i = 0; controller->waitForBegin(); ++i) {
	//	if (universe.interrupted())
	//		throw Exit();
	//	if (controller->waitForEnd(0))
	//		break;

	//	// print every 10th robot state
	//	if (i % 10 == 0) {
	//		context.write("State #%d (%s)\r", i, enableForceReading ? "Y" : "N");
	//		Controller::State s = lookupState();
	//		RealSeq handTorques; handTorques.assign(dimensions(), REAL_ZERO);
	//		if (armHandForce) armHandForce->getHandForce(handTorques);
	//		forceInpSensorSeq.push_back(handTorques);
	//		robotPoses.push_back(s);
	//		Twist wrench; if (graspSensorForce) graspSensorForce->readSensor(wrench, s.t);
	//		ftSeq.push_back(wrench);
	//		if (!isGrasping && i > 150) {
	//			enableForceReading = expectedCollisions(s);
	//			//record = true;
	//		}
	//		//if (!isGrasping && !enableForceReading && i > 150 && expectedCollisions(s))
	//		//	enableForceReading = true;
	//	}
	//}
//	enableForceReading = false;

	Controller::State::Seq robotPoses; robotPoses.clear();
	TwistSeq ftSeq; ftSeq.clear();
	std::vector<grasp::RealSeq> forceInpSensorSeq;
	TwistSeq thumbFT, indexFT, wristFT;
	TwistSeq rawThumbFT, rawIndexFT, rawWristFT;
	FT::Data thumbData, indexData, wristData;
	grasp::RealSeq force; force.assign(18, golem::REAL_ZERO);
	// repeat every send waypoint until trajectory end
	for (U32 i = 0; controller->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		Controller::State state = lookupState();
		robotPoses.push_back(state);
		if (wristFTSensor) {
			wristFTSensor->read(wristData);
			wristFT.push_back(wristData.wrench);
			Twist wwrench; SecTmReal wt;
			wristFTSensor->readSensor(wwrench, wt);
			rawWristFT.push_back(wwrench);

			wristData.wrench.v.getColumn3(&force[0]);
			wristData.wrench.w.getColumn3(&force[3]);

			ftSensorSeq[0]->read(thumbData);
			thumbFT.push_back(thumbData.wrench);
			Twist wthumb; SecTmReal tt;
			ftSensorSeq[0]->readSensor(wthumb, tt);
			rawThumbFT.push_back(wthumb);

			thumbData.wrench.v.getColumn3(&force[6]);
			thumbData.wrench.w.getColumn3(&force[9]);

			ftSensorSeq[1]->read(indexData);
			indexFT.push_back(indexData.wrench);
			Twist iwrench; SecTmReal it;
			ftSensorSeq[1]->readSensor(iwrench, it);
			rawIndexFT.push_back(iwrench);

			indexData.wrench.v.getColumn3(&force[12]);
			indexData.wrench.w.getColumn3(&force[15]);
		}

		// print every 10th robot state
		if (i % 10 == 0) {
			context.write("State #%d (%s)\r", i, enableForceReading ? "Y" : "N");

			//RealSeq handTorques; handTorques.assign(dimensions(), REAL_ZERO);
			//if (armHandForce) armHandForce->getHandForce(handTorques);
			//forceInpSensorSeq.push_back(handTorques);
			//robotPoses.push_back(s);
			//Twist wrench; if (graspSensorForce) graspSensorForce->readSensor(wrench, s.t);
			//ftSeq.push_back(wrench);

			if (!isGrasping && i > 500) {
				enableForceReading = expectedCollisions(state);
				//if (enableForceReading)
				//	sensorBundlePtr->enable();
				//enableForceReading = sensorBundlePtr->start2read = expectedCollisions(state);
				//record = true;
			}
			//if (!isGrasping && !enableForceReading && i > 150 && expectedCollisions(s))
			//	enableForceReading = true;
		}
	}
	enableForceReading = /*sensorBundlePtr->start2read =*/ false;
	//sensorBundlePtr->disable();

	std::stringstream prefixWrist;
	prefixWrist << std::fixed << std::setprecision(3) << "./data/boris/experiments/ft_readings/wrist" << "-" << context.getTimer().elapsed();
	std::stringstream prefixThumb;
	prefixThumb << std::fixed << std::setprecision(3) << "./data/boris/experiments/ft_readings/thumb" << "-" << context.getTimer().elapsed();
	std::stringstream prefixIndex;
	prefixIndex << std::fixed << std::setprecision(3) << "./data/boris/experiments/ft_readings/index" << "-" << context.getTimer().elapsed();

	std::stringstream prefixRawWrist;
	prefixRawWrist << std::fixed << std::setprecision(3) << "./data/boris/experiments/ft_readings/raw_wrist" << "-" << context.getTimer().elapsed();
	std::stringstream prefixRawThumb;
	prefixRawThumb << std::fixed << std::setprecision(3) << "./data/boris/experiments/ft_readings/raw_thumb" << "-" << context.getTimer().elapsed();
	std::stringstream prefixRawIndex;
	prefixRawIndex << std::fixed << std::setprecision(3) << "./data/boris/experiments/ft_readings/raw_index" << "-" << context.getTimer().elapsed();

	auto strFT = [=](std::ostream& ostr, const Twist& twist, const golem::SecTmReal tAbs, const golem::SecTmReal tRel) {
		ostr << tAbs << "\t" << tRel << "\t" << twist.v.x << "\t" << twist.v.y << "\t" << twist.v.z << "\t" << twist.w.x << "\t" << twist.w.y << "\t" << twist.w.z << std::endl;
	};
	auto strFTDesc = [=](std::ostream& ostr, const std::string& prefix) {
		ostr << "tAbs" << "tRel" << prefix << "vx" << "\t" << prefix << "vy" << "\t" << prefix << "vz" << "\t" << prefix << "wx" << "\t" << prefix << "wy" << "\t" << prefix << "wz" << std::endl;
	};

	// writing data into a text file, open file
	const std::string wristPath = grasp::makeString("%s.txt", prefixWrist.str().c_str());
	golem::mkdir(wristPath.c_str()); // make sure the directory exists
	std::ofstream wristSS(wristPath);
	const std::string rawWristPath = grasp::makeString("%s.txt", prefixRawWrist.str().c_str());
	golem::mkdir(rawWristPath.c_str()); // make sure the directory exists
	std::ofstream rawWristSS(rawWristPath);
	const std::string thumbPath = grasp::makeString("%s.txt", prefixThumb.str().c_str());
	golem::mkdir(thumbPath.c_str()); // make sure the directory exists
	std::ofstream thumbSS(thumbPath);
	const std::string rawThumbPath = grasp::makeString("%s.txt", prefixRawThumb.str().c_str());
	golem::mkdir(rawThumbPath.c_str()); // make sure the directory exists
	std::ofstream rawThumbSS(rawThumbPath);
	const std::string indexPath = grasp::makeString("%s.txt", prefixIndex.str().c_str());
	golem::mkdir(indexPath.c_str()); // make sure the directory exists
	std::ofstream indexSS(indexPath);
	const std::string rawIndexPath = grasp::makeString("%s.txt", prefixRawIndex.str().c_str());
	golem::mkdir(rawIndexPath.c_str()); // make sure the directory exists
	std::ofstream rawIndexSS(rawIndexPath);

	//// writing data into a text file, prepare headers
	strFTDesc(wristSS, std::string("ft_"));
	strFTDesc(rawWristSS, std::string("ft_"));
	strFTDesc(thumbSS, std::string("ft_"));
	strFTDesc(rawThumbSS, std::string("ft_"));
	strFTDesc(indexSS, std::string("ft_"));
	strFTDesc(rawIndexSS, std::string("ft_"));

	for (U32 indexjj = 0; indexjj < thumbFT.size(); ++indexjj) {
		const SecTmReal t = robotPoses[indexjj].t - initRecTime;
		strFT(wristSS, wristFT[indexjj], robotPoses[indexjj].t, t);
		strFT(rawWristSS, rawWristFT[indexjj], robotPoses[indexjj].t, t);
		strFT(thumbSS, thumbFT[indexjj], robotPoses[indexjj].t, t);
		strFT(rawThumbSS, rawThumbFT[indexjj], robotPoses[indexjj].t, t);
		strFT(indexSS, indexFT[indexjj], robotPoses[indexjj].t, t);
		strFT(rawIndexSS, rawIndexFT[indexjj], robotPoses[indexjj].t, t);
	}

	//if (record2) {
	//	std::stringstream prefix;
	//	prefix << std::fixed << std::setprecision(3) << "./data/boris/experiments/" << trialPtr->second->object.c_str() << "/" << trialPtr->second->name <<  "/obs_model_data/" << item << "-" << context.getTimer().elapsed();

	//	auto strSimTorques = [=](std::ostream& ostr, const Controller::State& state, const grasp::RealSeq& forces, const U32 jointInCollision) {
	//		//ostr << state.t << "\t";
	//		std::string c = jointInCollision > 0 ? "y" : "n"; //contact ? "y" : "n";
	//		ostr << c.c_str() << "\t";
	//		for (auto i = 0; i < forces.size(); ++i)
	//			ostr << forces[i] << "\t";
	//	};
	//	// writing data into a text file, data conversion
	//	auto strMat34 = [=](std::ostream& ostr, const Mat34& mat) {
	//		Quat quat(mat.R);
	//		ostr << mat.p.x << "\t" << mat.p.y << "\t" << mat.p.z << "\t" << quat.x << "\t" << quat.y << "\t" << quat.z << "\t" << quat.w << "\t";
	//	};
	//	auto strMat34Desc = [=](std::ostream& ostr, const std::string& prefix, U32 index) {
	//		ostr << prefix << "x" << index << "\t" << prefix << "y" << index << "\t" << prefix << "z" << index << "\t" << prefix << "qx" << index << "\t" << prefix << "qy" << index << "\t" << prefix << "qz" << index << "\t" << prefix << "qw" << index << "\t";
	//	};
	//	auto strState = [=](std::ostream& ostr, const Controller::State& state, const Controller::State::Info& info) {
	//		ostr << state.t << "\t";
	//		WorkspaceJointCoord wjc;
	//		controller->jointForwardTransform(state.cpos, wjc);
	//		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
	//			ostr << std::fixed << std::setprecision(6) << state.cpos[i] << "\t";
	//			//			strMat34(ostr, wjc[i]);
	//		}
	//	};
	//	auto strStateDesc = [=](std::ostream& ostr, const Controller::State::Info& info, const std::string& prefix) {
	//		ostr << prefix << "t" << "\t";
	//		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
	//			ostr << prefix << "j" << *i << "\t";
	//			//			strMat34Desc(ostr, prefix, *i);
	//		}
	//	};
	//	auto strFT = [=](std::ostream& ostr, const Twist& twist) {
	//		ostr << twist.v.x << "\t" << twist.v.y << "\t" << twist.v.z << "\t" << twist.w.x << "\t" << twist.w.y << "\t" << twist.w.z << "\t";
	//	};
	//	auto strFTDesc = [=](std::ostream& ostr, const std::string& prefix) {
	//		ostr << prefix << "vx" << "\t" << prefix << "vy" << "\t" << prefix << "vz" << "\t" << prefix << "wx" << "\t" << prefix << "wy" << "\t" << prefix << "wz" << "\t";
	//	};
	//	auto strTorquesDesc = [=](std::ostream& ostr) {
	//		ostr << /*"timestamp" << "\t" << "contact" << "\t" <<*/ "thumb_0" << "\t" << "thumb_1" << "\t" << "thumb_2" << "\t" << "thumb_3" << "\t" <<
	//			"index_0" << "\t" << "index_1" << "\t" << "index_2" << "\t" << "index_3" << "\t" <<
	//			"middle_0" << "\t" << "middle_1" << "\t" << "middle_2" << "\t" << "middle_3" << "\t" <<
	//			"ring_0" << "\t" << "ring_1" << "\t" << "ring_2" << "\t" << "ring_3" << "\t" <<
	//			"pinky_0" << "\t" << "pinky_1" << "\t" << "pinky_2" << "\t" << "pinky_3" << "\t";
	//	};
	//	auto strTorques = [=](std::ostream& ostr, const grasp::RealSeq& forces) {
	//		//ostr << state.t << grasp::to<Data>(data)->sepField;
	//		//std::string c = contact ? "y" : "n";
	//		//ostr << c.c_str() << grasp::to<Data>(data)->sepField;
	//		for (auto i = 0; i < forces.size(); ++i)
	//			ostr << std::fixed << std::setprecision(6) << forces[i] << "\t";
	//	};
	//	auto strSimTorquesDesc = [=](std::ostream& ostr) {
	//		ostr << /*"timestamp" << "\t" <<*/ "contact" << "\t" << "thumb_0" << "\t" << "thumb_1" << "\t" << "thumb_2" << "\t" << "thumb_3" << "\t" <<
	//			"index_0" << "\t" << "index_1" << "\t" << "index_2" << "\t" << "index_3" << "\t" <<
	//			"middle_0" << "\t" << "middle_1" << "\t" << "middle_2" << "\t" << "middle_3" << "\t" <<
	//			"ring_0" << "\t" << "ring_1" << "\t" << "ring_2" << "\t" << "ring_3" << "\t" <<
	//			"pinky_0" << "\t" << "pinky_1" << "\t" << "pinky_2" << "\t" << "pinky_3" << "\t";
	//	};


	//	// writing data into a text file, open file
	//	const std::string dataIndexPath = grasp::makeString("%s.txt", prefix.str().c_str());
	//	golem::mkdir(dataIndexPath.c_str()); // make sure the directory exists
	//	std::ofstream dataIndex(dataIndexPath);

	//	// writing data into a text file, prepare headers
	//	dataIndex << "#index" << "\t";
	//	strStateDesc(dataIndex, handInfo, std::string("cfg_"));
	//	strFTDesc(dataIndex, std::string("ft_"));
	//	strTorquesDesc(dataIndex);
	//	strSimTorquesDesc(dataIndex);
	//	dataIndex << std::endl;

	//	// writing data into a text file
	//	U32 j = 0, c = 0;
	//	//for (grasp::RobotState::Seq::const_iterator i = robotStates.begin(); i != robotStates.end(); ++i) {
	//	//	dataIndex << ++j << "\t";
	//	//	strState(dataIndex, i->command, robot->getStateHandInfo());
	//	//	strState(dataIndex, i->config, robot->getStateHandInfo());
	//	//	strFT(dataIndex, i->getSensorData<Twist>(grasp::RobotState::SENSOR_WRENCH));
	//	//	strTorques(dataIndex, i->command, "n");
	//	//	dataIndex << std::endl;
	//	//}
	//	for (U32 indexjj = 0; indexjj < robotPoses.size(); ++indexjj) {
	//		//RealSeq links; links.assign(dimensions(), REAL_ZERO);
	//		//size_t jointInCollision = !objectPointCloudPtr->empty() ? collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(robotPoses[indexjj]), links, true) : 0;
	//		//dataSimContact << indexjj << "\t";
	//		//strTorques(dataSimContact, links, jointInCollision);
	//		//dataSimContact << std::endl;

	//		dataIndex << ++j << "\t";
	//		strState(dataIndex, robotPoses[indexjj], handInfo);
	//		strFT(dataIndex, ftSeq[indexjj]);
	//		strTorques(dataIndex, forceInpSensorSeq[indexjj]);
	//		RealSeq links; links.assign(dimensions(), REAL_ZERO);
	//		size_t jointInCollision = !objectPointCloudPtr->empty() ? collisionPtr->simulate(ragDesc.objCollisionDescPtr->flannDesc, rand, manipulator->getConfig(robotPoses[indexjj]), links, true) : 0;
	//		strSimTorques(dataIndex, robotPoses[indexjj], links, jointInCollision);
	//		dataIndex << std::endl;
	//	}
	//}
	record = brecord = false;

	// stop recording
	recordingStop(getPlanner().trajectoryIdlePerf);
	recordingWaitToStop();

	// insert trajectory
	{
		golem::CriticalSectionWrapper csw(getCS());
		grasp::data::Data::Map::iterator data = dataMap.find(recorderData);
		if (data == dataMap.end())
			throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to find Data %s", recorderData.c_str());
		data->second->itemMap.insert(std::make_pair(recorderItem + makeString("%s%.3f", dataDesc->sepName.c_str(), recorderStart), itemTrajectory));
	}
	context.write("Performance finished!\n");
}

//------------------------------------------------------------------------------

bool R2GPlanner::execute(grasp::data::Data::Map::iterator dataPtr, grasp::Waypoint::Seq& trajectory) {
	if (trajectory.size() < 3)
		throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

	const golem::Chainspace::Index armChain = getPlanner().armInfo.getChains().begin();
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
	const U32 initIdx = 0, pregraspIdx = 1, graspIdx = 2;
	// init has the wrist pose of the 2nd element in the trajectory
	Controller::State init = trajectory[pregraspIdx].state;
	// and the fingers opened as the 1st element in the trajectory
	for (auto j = getPlanner().handInfo.getJoints().begin(); j != getPlanner().handInfo.getJoints().end(); ++j)
		init.cpos[j] = trajectory[initIdx].state.cpos[j];
	// pregrasp has the wrist pose of the 2nd element in the trajectory
	Controller::State pregrasp = trajectory[graspIdx].state;
	// and the fingers opened as the 1st element in the trajectory
	for (auto j = getPlanner().handInfo.getJoints().begin(); j != getPlanner().handInfo.getJoints().end(); ++j)
		pregrasp.cpos[j] = trajectory[initIdx].state.cpos[j];
	// grasp pose
	Controller::State grasp = trajectory[graspIdx].state;

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
			findTarget(grasp::to<Data>(dataPtr)->queryTransform, grasp::to<Data>(dataPtr)->modelFrame, trajectory.front().command, cend);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}
		//grasp::Manipulator::Config config(cend.cpos, manipulator->getBaseFrame(cend.cpos));
		//Bounds::Seq bounds = manipulator->getBounds(config.config, config.frame.toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach;
		try {
			(void)findTrajectory(grasp::Waypoint::lookup(*controller).command, &cend, nullptr, 0, approach);
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
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
			if (!trajectory)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current states
			trajectory->setWaypoints(grasp::Waypoint::make(out, out));
			Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
		}

		//handBounds.clear();
		//debugRenderer.reset();
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
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, queryHandlerTrj->create()));
			grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
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

		Controller::State cstart = init, cend = pregrasp, cInit = lookupState();

		// transform w.r.t. query frame
		findTarget(grasp::to<Data>(dataPtr)->queryTransform, init, cstart);
		findTarget(grasp::to<Data>(dataPtr)->queryTransform, pregrasp, cend);

		//Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(cend), manipulator->getConfig(cend).toMat34());
		//renderHand(cend, bounds, true);

		Controller::State::Seq approach, seq;
		findTrajectory(cstart, &cend, nullptr, 0, approach);
		findTrajectory(cInit, &approach.front(), nullptr, 0, approach);
//		approach.insert(approach.end(), seq.begin(), seq.end());

		Controller::State::Seq out = approach;
		profile(this->trjDuration, approach, out, silent);
		pHeuristic->setPointCloudCollision(true);
		
		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
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
	case 'G':
	{
		context.debug("Plan grasping\n");
		std::string trjItemName("graspingTrj");
		resetPlanning();
		isGrasping = true;

		// current configuration (fingers opened)
		Controller::State cstart = lookupState();

		// grasp configuration (fingers closed)
		Controller::State cend = lookupState();
		for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i)
			cend.cpos[i] = grasp.cpos[i];

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
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, queryHandlerTrj->create()));
			grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
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
		for (auto i = getPlanner().handInfo.getJoints().begin(); i != getPlanner().handInfo.getJoints().end(); ++i) {
			const size_t k = i - getPlanner().handInfo.getJoints().begin();
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

		Controller::State cstart = lookupState(), cend = lookupState();

		// transform w.r.t. query frame 
		try {
			findTarget(Mat34::identity(), cend, cend, true);
		}
		catch (const Message& msg) {
			context.write("%s\n", msg.str().c_str());
			resetPlanning();
			return false;
		}

		Controller::State::Seq approach;
		try {
			findTrajectory(cstart, &cend, nullptr, 0, approach);
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
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
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
		//::sleep(1000);
		Controller::State openfingers = lookupState();
		Controller::State cnow = lookupState();
		for (auto i = getPlanner().handInfo.getChains().begin(); i != getPlanner().handInfo.getChains().end(); ++i) {
			for (auto j = getPlanner().handInfo.getJoints(i).begin(); j != getPlanner().handInfo.getJoints(i).end(); ++j)
				openfingers.cpos[j] = init.cpos[j]; // pre-grasp fingers' pose
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
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(releaseTrjName, modelHandlerTrj->create()));
			grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
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
			armHandForce->getArmCtrl()->setMode(armMode/*ActiveCtrlForce::MODE_ENABLED*/);
			armHandForce->getHandCtrl()->setMode(handMode/*ActiveCtrlForce::MODE_ENABLED*/);
		}


		resetPlanning();
		return true;
	}
	}
	return false;
}

//------------------------------------------------------------------------------

//grasp::Cloud::PointSeqMap::iterator R2GPlanner::getTrnPoints(Data::Map::iterator dataPtr, const Mat34 &trn) {
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
//grasp::Cloud::RawPointSeqMultiMap::iterator R2GPlanner::getTrnRawPoints(Data::Map::iterator dataPtr, const Mat34 &trn) {
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

Real R2GPlanner::simContacts(const golem::Bounds::Seq::const_iterator &begin, const golem::Bounds::Seq::const_iterator &end, const golem::Mat34 pose) {
	return REAL_ZERO;
}

//------------------------------------------------------------------------------

void R2GPlanner::updateAndResample(Data::Map::iterator dataPtr) {
	if (!pBelief.get() || grasp::to<Data>(dataPtr)->queryPoints.empty())
		return;
	context.debug("R2GPlanner::updateAndResample(): %d triggered guards:\n", /*grasp::to<Data>(dataPtr)->*/ftGuards.size());
	
	// update samples' weights
//	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
	golem::Waypoint w(*controller, lookupState().cpos/*grasp::to<Data>(dataPtr)->triggeredStates.begin()->cpos*/);
	context.write("update weights, triggered guards size = %u\n", ftGuards.size());

	for (auto g = ftGuards.begin(); g != ftGuards.end(); ++g)
		(*g)->str(context);
	// render uncertainty before belief update
	resetDataPointers();
	showQueryDistribPointClouds = false;
	showGroundTruth = false;
	to<Data>(dataCurrentPtr)->createRender();

	const std::string itemName = "belief-" + makeString("%f", context.getTimer().elapsed());
	recordingStart(dataPtr->first, itemName, true);
	recordingWaitToStart();

	//::sleep(1000);
	//Collision::FlannDesc waypointDesc;
	//Collision::Desc::Ptr cloudDesc;
	//cloudDesc.reset(new Collision::Desc());
	//Collision::Ptr cloud = cloudDesc->create(*manipulator);
	//waypointDesc.depthStdDev = 0.01/*0.0005*/; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
	//waypointDesc.radius = REAL_ZERO;

	//golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO, cdf = golem::REAL_ZERO;
	//pBelief->normaliseFac = REAL_ZERO;
	//grasp::Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
	//for (grasp::RBPose::Sample::Seq::iterator sampledPose = pBelief->getSamples().begin(); sampledPose != pBelief->getSamples().end();) {
	//	grasp::Cloud::PointSeq points;
	//	grasp::Cloud::transform(sampledPose->toMat34(), modelPoints, points);
	//	debugRenderer.reset();
	//	debugAppearance.draw(points, debugRenderer);
	//	cloud->create(rand, points);
	//	sampledPose->weight = cloud->evaluateFT(debugRenderer, waypointDesc, config, ftGuards, true);
	//	golem::kahanSum(pBelief->normaliseFac, c, sampledPose->weight);
	//	context.write("sample.weight = %f,\n", sampledPose->weight);
	//	if (option("YN", "Next? (Y/N)") == 'Y')
	//		++sampledPose;
	//}
	//c = golem::REAL_ZERO;
	//for (grasp::RBPose::Sample::Seq::iterator sampledPose = pBelief->getSamples().begin(); sampledPose != pBelief->getSamples().end(); ++sampledPose) {
	//	golem::kahanSum(cdf, c, sampledPose->weight);
	//	sampledPose->cdf = cdf;
	//}

	pBelief->createUpdate(debugRenderer, w, ftGuards, trialPtr != trialDataMap.end() ? grasp::to<TrialData>(trialPtr)->queryPointsTrn : grasp::RBCoord());
	
	// render the mismatch between estimate and ground truth before resampling
	to<Data>(dataCurrentPtr)->createRender();
	//::sleep(5000);


	context.write("resample (wheel algorithm)...\n");
	// resampling (wheel algorithm)
//	pBelief->createResample(/*manipulator->getConfig(getStateFrom(w))*/);
//	size_t N = pBelief->getSamples().size(), index = rand.nextUniform<size_t>(0, N);
//	Real beta = golem::REAL_ZERO;
//	grasp::RBPose::Sample::Seq newPoses;
//	newPoses.reserve(N);
//	for (size_t i = 0; i < N; ++i) {
//		beta += rand.nextUniform<golem::Real>() * 2 * pBelief->maxWeight();
//		//		context.write("spam::RBPose::createResampling(): beta=%4.6f\n", beta);
//		while (beta > pBelief->getSamples()[index].weight) {
//			beta -= pBelief->getSamples()[index].weight;
//			index = (index + 1) % N;
//		}
//		context.write("Resample[%d] = Sample[%d].weight=%f\n", i, index, pBelief->getSamples()[index].weight);
//		newPoses.push_back(pBelief->getSamples().at(index));
//	}
//
//	// add noise to the resampled elements and overwrite poses
//	pBelief->getSamples().clear();
//	pBelief->getSamples().reserve(N);
//
//	// generate new (noisy) samples out of selected subset of poses 
//	for (size_t i = 0; i < N;) {
//		//mfsePoses.push_back(Sample(newPoses[i], REAL_ONE, i*REAL_ONE));
//		//continue;
//		grasp::RBCoord c = newPoses[i];
////		rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &(newPoses[i])[0], &pBelief->pose.covarianceSqrt[0]); // normalised multivariate Gaussian
//		grasp::Cloud::PointSeq points;
//		grasp::Cloud::transform(newPoses[i].toMat34(), modelPoints, points);
//		debugRenderer.reset();
//		debugAppearance.draw(points, debugRenderer);
//		grasp::Cloud::PointSeq resample;
//		grasp::Cloud::transform(c.toMat34(), modelPoints, resample);
//		resampleAppeareance.draw(resample, debugRenderer);
//
//		pBelief->getSamples().push_back(grasp::RBPose::Sample(c, REAL_ONE, i*REAL_ONE));
//		if (option("YN", "Resample Next? (Y/N)") == 'Y')
//			++i;
//	}
//	pBelief->normaliseFac = REAL_ZERO;
//
//	// compute mean and covariance
//	if (!pBelief->pose.create<golem::Ref1, grasp::RBPose::Sample::Ref>(grasp::RBCoord::N, pBelief->desc.covariance, pBelief->getSamples()))
//		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createResample(): Unable to create mean and covariance for the high dimensional representation");
//
//	context.write("spam::Belief::createResample(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", 
//		pBelief->pose.covariance[0], pBelief->pose.covariance[1], pBelief->pose.covariance[2], 
//		pBelief->pose.covariance[3], pBelief->pose.covariance[4], pBelief->pose.covariance[5], pBelief->pose.covariance[6]);

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
	for (auto g = ftGuards.begin(); g != ftGuards.end(); ++g)
		(*g)->unlock();

	// stop recording
	recordingStop(getPlanner().trajectoryIdlePerf);
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
