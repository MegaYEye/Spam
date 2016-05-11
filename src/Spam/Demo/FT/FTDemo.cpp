/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Spam/Demo/FT/FTDemo.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Planner/Data.h>
#include <Golem/Planner/GraphPlanner/Data.h>
//#include <Golem/Phys/PhysScene.h>
#include <algorithm>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>
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

//------------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;

//------------------------------------------------------------------------------

void FTDemo::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	R2GPlanner::Desc::load(context, xmlcontext);

	try {
		xmlcontext = xmlcontext->getContextFirst("demo");
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
}

//------------------------------------------------------------------------------

FTDemo::FTDemo(Scene &scene) : R2GPlanner(scene) {
}

FTDemo::~FTDemo() {
}

void FTDemo::create(const Desc& desc) {
	desc.assertValid(Assert::Context("FTDemo::Desc."));

	// create object
	R2GPlanner::create(desc); // throws

	// command keys
	std::string playCmd("TP");
	std::string moveCmd("TE");
	ftpath = desc.path;
	sepDir = desc.sepDir;
	iteration = 1;

	auto executeCmd = [&](const std::string command) {
		MenuCmdMap::const_iterator cmd = menuCmdMap.find(command);
		if (cmd == menuCmdMap.end())
			throw Cancel(makeString("Error: impossible to execute command %s.", command.c_str()).c_str());
		cmd->second();
	};

	menuCtrlMap.insert(std::make_pair("Z", [=](MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: (D)ata gathering/(B)elief update test/e(X)ecute demo/(P)lay trajectory...";
	}));
	menuCmdMap.insert(std::make_pair("ZD", [=]() {
		// item name
		readString("Enter item name: ", object);
		// item name
		readNumber("Enter iteration: ", iteration);
	}));
	menuCmdMap.insert(std::make_pair("ZB", [=]() {
		grasp::to<Data>(dataCurrentPtr)->stratType = Strategy::MYCROFT;
		grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_M2Q;
		// command keys
		std::string createModelCmd("PM");
		std::string createQueryCmd("PQ");
		std::string trjPlayCmd("TP");
		// create a model for the object
		context.write("Create a model...\n");
		executeCmd(createModelCmd); // move the robot to the home pose after scanning
		context.write("Create a query...\n");
		executeCmd(createQueryCmd);

		// set the simulated object
		if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
			sensorBundlePtr->getCollisionPtr()->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
			objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
		}
		//executeCmd(trjPlayCmd);


		////grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		////grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(item->second.get());
		////if (!trajectory)
		////	throw Cancel("Error: no trajectory selected.");
		////// play
		//////Controller::State::Seq inp = trajectory->getWaypoints();
		////grasp::Waypoint::Seq inp = trajectory->getWaypoints();
		////if (inp.size() < 3)
		////	throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

		//for (;;) {
		//	//if (!execute(dataCurrentPtr, inp))
		//	//	return;
		//	if (contactOccured) {
		//		//grasp::to<Data>(cdata->second)->replanning = false;
		//		contactOccured = false;
		//		updateAndResample(dataCurrentPtr);
		//		enableForceReading = false;
		//		continue;
		//	}
		//}

	}));
	menuCmdMap.insert(std::make_pair("ZX", [=]() {
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
				strat = Strategy::IR3NE;
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
			gotoConfig(home);
			//Controller::State::Seq seq, out;
			//findTrajectory(lookupState(), &home, nullptr, 0, seq);
			//profile(this->trjDuration, seq, out, true);
			//sendTrajectory(out);
			//controller->waitForBegin();
			//controller->waitForEnd();
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

		grasp::to<Data>(dataCurrentPtr)->stratType = Strategy::NONE_STRATEGY;
		grasp::to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
		grasp::data::Item::Map::iterator modelContact, modelPtr, queryContactPtr, queryTrjPtr;
		grasp::data::Item::Ptr queryGraspTrj;
		U32 modelViews = modelScanPoseSeq.size(), queryViews = queryScanPoseSeq.size(), trials = 5;
		const U32 maxFailures = 2, maxIterations = 5;

		// command keys
		std::string createModelCmd("PM");
		std::string createQueryCmd("PQ");
		std::string itemTransCmd("IT");
		std::string itemConvCmd("IC");
		std::string dataSaveCmd("DS");
		std::string itemRemoveCmd("IR");

		//--------------------------------------------------------------------//
		// CREATE A MODEL POINT CLOUD
		//--------------------------------------------------------------------//
		readString("Enter object name: ", object);
		// item name
		readNumber("Enter iteration: ", iteration);
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
		context.write("Create a predictive contact model for the grasp [%s]...\n", modelGraspItem.c_str());
		if (option("YN", "Create a new contact model? (Y/N)") == 'Y') {
			dataItemLabel = modelGraspItem;
			executeCmd(itemTransCmd);
			modelGraspItem = dataItemLabel;
		}
		context.write("done.\n");

		//--------------------------------------------------------------------//
		// CREATE LOG
		//--------------------------------------------------------------------//
		std::string dirLog = makeString("./data/boris/experiments/%s/", object.c_str());
		golem::mkdir(dirLog.c_str());
		std::stringstream prefixLog;
		prefixLog << std::fixed << std::setprecision(3) << dirLog << "output" << "-" << context.getTimer().elapsed();
		const std::string outputLog = makeString("%s.log", prefixLog.str().c_str());
		std::ofstream logFile(outputLog);
		std::stringstream prefixMicroft;
		prefixMicroft << std::fixed << std::setprecision(3) << dirLog << "MICROFT" << "-" << context.getTimer().elapsed();
		const std::string outputMicroft = makeString("%s.log", prefixLog.str().c_str());
		std::ofstream logMicroft(outputMicroft);
		std::stringstream prefixIrene;
		prefixIrene << std::fixed << std::setprecision(3) << dirLog << "IRENE" << "-" << context.getTimer().elapsed();
		const std::string outputIrene = makeString("%s.log", prefixLog.str().c_str());
		std::ofstream logIrene(outputIrene);

		Data::Ptr data = createData();
		data.reset(to<Data>(dataCurrentPtr));

		//--------------------------------------------------------------------//
		// TRIALS
		//--------------------------------------------------------------------//
		for (U32 trial = iteration; trial <= trials; ++trial) {
			for (;;) {
				// select strategy in order: ELEMENTARY, MYCROFT, IR3NE, NONE. 
				select(grasp::to<Data>(dataCurrentPtr)->stratType);
				// stop internal loop if all the strategies have been executed. increase trial
				if (to<Data>(dataCurrentPtr)->stratType == Strategy::NONE_STRATEGY)
					break;

				//--------------------------------------------------------------------//
				// CREATE DATA TRIAL
				//--------------------------------------------------------------------//
				std::string dataTrialPath = makeString("./data/boris/experiments/%s/trial0%d/%s/data.xml", object.c_str(), trial, strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str());
				Data::Ptr dataTrial = createData();
				dataTrial.reset(to<Data>(data));
				RenderBlock renderBlock(*this);
				scene.getOpenGL(to<Data>(dataCurrentPtr)->getView().openGL); // set current view
				scene.getOpenGL(to<Data>(dataTrial)->getView().openGL); // set view of the new data
				{
					golem::CriticalSectionWrapper cswData(scene.getCS());
					dataMap.erase(dataPath);
					dataCurrentPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(dataTrialPath, dataTrial));
				}

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
					sensorBundlePtr->getCollisionPtr()->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
					objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
				}
				reset(); // move the robot to the home pose after scanning
				context.write("done.\n");

				//--------------------------------------------------------------------//
				// CREATE A PREDICTIVE QUERY FOR THE GRASP
				//--------------------------------------------------------------------//
				// generate features
				grasp::data::Transform* transform = is<grasp::data::Transform>(queryGraspHandler);
				if (!transform)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", queryGraspHandler->getID().c_str());


				grasp::data::Item::List list;
				grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(modelGraspItem);
				if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
					throw Message(Message::LEVEL_ERROR, "R2GPlanner::estimatePose(): Does not find %s.", modelGraspItem.c_str());
				list.insert(list.end(), ptr);
				
				// retrieve model point cloud
				modelPtr = to<Data>(dataCurrentPtr)->itemMap.find(modelItem);
				if (modelPtr == to<Data>(dataCurrentPtr)->itemMap.end())
					throw Message(Message::LEVEL_ERROR, "R2GPlanner::estimatePose(): Does not find %s.", modelItem.c_str());
				list.insert(list.end(), modelPtr); // grasp on model
				grasp::data::Item::Ptr queryGraspItemPtr = transform->transform(list);

				// insert processed object, remove old one
				RenderBlock renderBlock0(*this);
				{
					golem::CriticalSectionWrapper cswData(getCS());
					to<Data>(dataCurrentPtr)->itemMap.erase(queryGraspItem);
					queryContactPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(queryGraspItem, queryGraspItemPtr));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryContactPtr, to<Data>(dataCurrentPtr)->getView());
				}
				context.write("Transform: handler %s, inputs %s, %s...\n", queryGraspHandler->getID().c_str(), queryContactPtr->first.c_str(), modelItem.c_str());
				// calculate the scor eof the desired grasp
				grasp::data::ContactQuery *cq = is<grasp::data::ContactQuery>(queryContactPtr);
				const golem::Real dGraspLik = cq->getData().configs[0]->likelihood.value;
				context.info("Desired grasp quality (active lik) %f\n", dGraspLik);

				//--------------------------------------------------------------------//
				// CREATE A GRASP TRAJECTORY
				//--------------------------------------------------------------------//
				context.write("Convert [%s]: handler %s, input %s...\n", queryItemTrj.c_str(), queryHandlerTrj->getID().c_str(), queryContactPtr->first.c_str());
				grasp::data::Convert* convert = is<grasp::data::Convert>(queryContactPtr);
				if (!convert)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Convert interface", queryHandlerTrj->getID().c_str());

				// convert
				queryGraspTrj = convert->convert(*queryHandlerTrj);
				// insert processed object, remove old one
				RenderBlock renderBlock2(*this);
				{
					golem::CriticalSectionWrapper cswData(getCS());
					to<Data>(dataCurrentPtr)->itemMap.erase(queryItemTrj);
					queryTrjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(queryItemTrj, queryGraspTrj));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryTrjPtr, to<Data>(dataCurrentPtr)->getView());
				}
				context.write("done.\n");

				context.write("Transform: handler %s, input %s...\n", queryHandlerTrj->getID().c_str(), queryTrjPtr->first.c_str());
//				spam::data::R2GTrajectory* trajectory = is<spam::data::R2GTrajectory>(queryTrjPtr);
				grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(queryTrjPtr);
				if (!trajectory)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Trajectory interface", queryHandlerTrj->getID().c_str());
				grasp::Waypoint::Seq inp = trajectory->getWaypoints();
				if (inp.size() < 3)
					throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

				// set render to show only mean hypothesis and ground truth
				resetDataPointers();
				showHypothesesPointClouds = true;
				showMeanHypothesisPointClouds = true;
				showGroundTruth = true;
				to<Data>(dataCurrentPtr)->createRender();

				//--------------------------------------------------------------------//
				// EXECUTE TRIAL
				//--------------------------------------------------------------------//
				U32 iteration = 1, failures = 0;
				const RBCoord obj(simQueryFrame * to<Data>(dataCurrentPtr)->modelFrame);
				for (; failures < maxFailures && iteration < maxIterations;) {
					grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_M2Q;
					if (!execute(dataCurrentPtr, inp)) { // if it fails to find a trajectory repeat 
						++failures;
						continue;
					}

					/*if (contactOccured && grasp::to<Data>(dataCurrentPtr)->stratType != Strategy::ELEMENTARY)*/ {
						// decide if attempt to grasp if a contact occured
						//cq->getData().configs[0]->getContact()->getOptimisation()->;
						grasp::OptimisationSA::Desc optimisationDesc;
						grasp::Contact c = *cq->getData().configs[0]->getContact();
						const OptimisationSA* optimisation = dynamic_cast<const OptimisationSA*>(cq->getData().configs[0]->getContact()->getOptimisation());
						if (!optimisation)
							continue;
						
						Controller::State cinit = lookupState(), cend = lookupState();
						findTarget(grasp::to<Data>(dataCurrentPtr)->queryTransform, inp[2].state, cend);
						grasp::Manipulator::Config config(cend.cpos, manipulator->getBaseFrame(cend.cpos));

						Bounds::Seq bounds = manipulator->getBounds(config.config, config.frame.toMat34());
						renderHand(cend, bounds, true);

						Quat q; manipulator->getBaseFrame(cend.cpos).R.toQuat(q);
						grasp::RBCoord cc(manipulator->getBaseFrame(cend.cpos).p, q);
						grasp::Manipulator::Waypoint waypoint(cend.cpos, cc);
						grasp::Contact::Likelihood likelihood;
						optimisation->evaluate(waypoint, likelihood);
						grasp::Contact::Config::Seq queryConfigs = cq->getData().configs;
						grasp::Contact::Config::Seq::iterator it = c.find(queryConfigs, queryConfigs.begin());
						
						const Real loss = dGraspLik != REAL_ZERO ? 1 - likelihood.value / dGraspLik : REAL_ZERO;
						context.write("Loss %f -> lik.value = %f\n", loss, likelihood.value);
						// if loss is greater than 0.25, then we replan -> go for a grasp!
						if (loss > 0.25) {
							//grasp::to<Data>(cdata->second)->replanning = false;
							contactOccured = false;
							updateAndResample(dataCurrentPtr);
							enableForceReading = false;
							++iteration;
							//bool r = unlockContact();
							//context.write("unlock contact %s\n", r ? "TRUE" : "FALSE");
							continue;
						}
					}
					grasp::to<Data>(dataCurrentPtr)->actionType = action::GRASP;
					context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
					if (!execute(dataCurrentPtr, inp)) {// not successfully close fingers
						++failures;
						continue;
					}
					grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_LIFT;
					context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
					if (!execute(dataCurrentPtr, inp)) {// not successfully close fingers
						++failures;
						continue;
					}
					if (option("YN", "Success? (Y/N)") == 'Y')
						to<TrialData>(trialPtr)->grasped = true;
					else
						to<TrialData>(trialPtr)->grasped = false;

					break;
				}
				// reset robot
				reset();
			} // end strategy
		} // end trial
		//finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("ZP", [=]() {
		grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		data::R2GTrajectory* trajectory = is<data::R2GTrajectory>(item->second.get());
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		Controller::State::Seq seq;
		trajectory->createTrajectory(seq);
		// select collision object
		CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
		// perform
		perform(dataCurrentPtr->first, item->first, seq);
		// done!
		createRender();
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("ZA", [=]() {
		grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		data::R2GTrajectory* trajectory = is<data::R2GTrajectory>(item->second.get());
		if (!trajectory)
			throw Cancel("Error: no trajectory selected.");
		// play
		Controller::State::Seq seq;
		trajectory->createAction(seq);
		// select collision object
		CollisionBounds::Ptr collisionBounds = selectCollisionBounds();
		// perform
		perform(dataCurrentPtr->first, item->first, seq);
		// done!
		createRender();
		context.write("Done!\n");
	}));

}

//------------------------------------------------------------------------------

void FTDemo::render() const {
	R2GPlanner::render();
}

void FTDemo::perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

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

	// start recording
	recordingStart(data, item, true);
	recordingWaitToStart();

	// send trajectory
	sendTrajectory(trajectory);

	Controller::State::Seq robotPoses; robotPoses.clear();
	//TwistSeq ftSeq; ftSeq.clear();
	//std::vector<grasp::RealSeq> forceInpSensorSeq;
	//TwistSeq thumbFT, indexFT, wristFT;
	//TwistSeq rawThumbFT, rawIndexFT, rawWristFT;
	//FT::Data thumbData, indexData, wristData;
	//grasp::RealSeq force; force.assign(18, golem::REAL_ZERO);
	//sensorBundlePtr->start2read = true;
	sensorBundlePtr->enable();
	// repeat every send waypoint until trajectory end
	for (U32 i = 0; controller->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		Controller::State state = lookupState();
		robotPoses.push_back(state);
		//sensorBundlePtr->increment();

		/*
		//if (wristFTSensor) {
		//	wristFTSensor->read(wristData);
		//	wristFT.push_back(wristData.wrench);
		//	Twist wwrench; SecTmReal wt;
		//	wristFTSensor->readSensor(wwrench, wt);
		//	rawWristFT.push_back(wwrench);

		//	wristData.wrench.v.getColumn3(&force[0]);
		//	wristData.wrench.w.getColumn3(&force[3]);

		//	ftSensorSeq[0]->read(thumbData);
		//	thumbFT.push_back(thumbData.wrench);
		//	Twist wthumb; SecTmReal tt;
		//	ftSensorSeq[0]->readSensor(wthumb, tt);
		//	rawThumbFT.push_back(wthumb);

		//	thumbData.wrench.v.getColumn3(&force[6]);
		//	thumbData.wrench.w.getColumn3(&force[9]);

		//	ftSensorSeq[1]->read(indexData);
		//	indexFT.push_back(indexData.wrench);
		//	Twist iwrench; SecTmReal it;
		//	ftSensorSeq[1]->readSensor(iwrench, it);
		//	rawIndexFT.push_back(iwrench);

		//	indexData.wrench.v.getColumn3(&force[12]);
		//	indexData.wrench.w.getColumn3(&force[15]);
		//}*/

		// print every 10th robot state
		if (i % 10 == 0)
			context.write("State #%d (%s)\r", i, enableForceReading ? "Y" : "N");
		if (!isGrasping && i > 600) {
			enableForceReading = expectedCollisions(state);
		}
	}
	enableForceReading = false;
	sensorBundlePtr->disable();
	//sensorBundlePtr->start2read = false;
	/*
	//std::string dir = makeString("%s%s/%s/trial0%d/", ftpath.c_str(), object.c_str(), item.c_str(), iteration++);
	//golem::mkdir(dir.c_str());
	//std::stringstream prefixWrist;
	//prefixWrist << std::fixed << std::setprecision(3) << dir << "wrist" << "-" << context.getTimer().elapsed();
	//std::stringstream prefixThumb;
	//prefixThumb << std::fixed << std::setprecision(3) << dir << "thumb" << " - " << context.getTimer().elapsed();
	//std::stringstream prefixIndex;
	//prefixIndex << std::fixed << std::setprecision(3) << dir << "index" << " - " << context.getTimer().elapsed();

	//std::stringstream prefixRawWrist;
	//prefixRawWrist << std::fixed << std::setprecision(3) << dir << "raw_wrist" << " - " << context.getTimer().elapsed();
	//std::stringstream prefixRawThumb;
	//prefixRawThumb << std::fixed << std::setprecision(3) << dir << "raw_thumb" << "-" << context.getTimer().elapsed();
	//std::stringstream prefixRawIndex;
	//prefixRawIndex << std::fixed << std::setprecision(3) << dir << "raw_index" << "-" << context.getTimer().elapsed();

	//auto strFT = [=](std::ostream& ostr, const Twist& twist, const golem::SecTmReal tAbs, const golem::SecTmReal tRel) {
	//	ostr << tAbs << "\t" << tRel << "\t" << twist.v.x << "\t" << twist.v.y << "\t" << twist.v.z << "\t" << twist.w.x << "\t" << twist.w.y << "\t" << twist.w.z << std::endl;
	//};
	//auto strFTDesc = [=](std::ostream& ostr, const std::string& prefix) {
	//	ostr << "tAbs" << "tRel" << prefix << "vx" << "\t" << prefix << "vy" << "\t" << prefix << "vz" << "\t" << prefix << "wx" << "\t" << prefix << "wy" << "\t" << prefix << "wz" << std::endl;
	//};

	//// writing data into a text file, open file
	//const std::string wristPath = grasp::makeString("%s.txt", prefixWrist.str().c_str());
	//golem::mkdir(wristPath.c_str()); // make sure the directory exists
	//std::ofstream wristSS(wristPath);
	//const std::string rawWristPath = grasp::makeString("%s.txt", prefixRawWrist.str().c_str());
	//golem::mkdir(rawWristPath.c_str()); // make sure the directory exists
	//std::ofstream rawWristSS(rawWristPath);
	//const std::string thumbPath = grasp::makeString("%s.txt", prefixThumb.str().c_str());
	//golem::mkdir(thumbPath.c_str()); // make sure the directory exists
	//std::ofstream thumbSS(thumbPath);
	//const std::string rawThumbPath = grasp::makeString("%s.txt", prefixRawThumb.str().c_str());
	//golem::mkdir(rawThumbPath.c_str()); // make sure the directory exists
	//std::ofstream rawThumbSS(rawThumbPath);
	//const std::string indexPath = grasp::makeString("%s.txt", prefixIndex.str().c_str());
	//golem::mkdir(indexPath.c_str()); // make sure the directory exists
	//std::ofstream indexSS(indexPath);
	//const std::string rawIndexPath = grasp::makeString("%s.txt", prefixRawIndex.str().c_str());
	//golem::mkdir(rawIndexPath.c_str()); // make sure the directory exists
	//std::ofstream rawIndexSS(rawIndexPath);

	////// writing data into a text file, prepare headers
	//strFTDesc(wristSS, std::string("ft_"));
	//strFTDesc(rawWristSS, std::string("ft_"));
	//strFTDesc(thumbSS, std::string("ft_"));
	//strFTDesc(rawThumbSS, std::string("ft_"));
	//strFTDesc(indexSS, std::string("ft_"));
	//strFTDesc(rawIndexSS, std::string("ft_"));

	//for (U32 indexjj = 0; indexjj < thumbFT.size(); ++indexjj) {
	//	const SecTmReal t = robotPoses[indexjj].t - recorderStart;
	//	strFT(wristSS, wristFT[indexjj], robotPoses[indexjj].t, t);
	//	strFT(rawWristSS, rawWristFT[indexjj], robotPoses[indexjj].t, t);
	//	strFT(thumbSS, thumbFT[indexjj], robotPoses[indexjj].t, t);
	//	strFT(rawThumbSS, rawThumbFT[indexjj], robotPoses[indexjj].t, t);
	//	strFT(indexSS, indexFT[indexjj], robotPoses[indexjj].t, t);
	//	strFT(rawIndexSS, rawIndexFT[indexjj], robotPoses[indexjj].t, t);
	//}*/

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

bool FTDemo::execute2(grasp::data::Data::Map::iterator dataPtr, spam::data::R2GTrajectory& trajectory) {
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


	switch (key){
	case 'M':
	{
		return false;
	}
	case 'Q':
	{
		return false;
	}
	case 'T':
	{
		context.debug("Plan trajectory optimisation\n");
		std::string trjItemName("queryTrj");
		pHeuristic->enableUnc = grasp::to<Data>(dataPtr)->stratType == Strategy::IR3NE ? true : false;
		pHeuristic->setPointCloudCollision(true);
		isGrasping = false;

		Controller::State::Seq seq;
		trajectory.createTrajectory(seq);
		// transform w.r.t. query frame
		Controller::State cstart = lookupState(), cend = lookupState();

		// transform w.r.t. query frame
		//findTarget(grasp::to<Data>(dataPtr)->queryTransform, seq[0], cstart);
		findTarget(grasp::to<Data>(dataPtr)->queryTransform, seq[1], cend);

		Controller::State::Seq approach;
		findTrajectory(cstart, &cend, nullptr, 0, approach);

		Controller::State::Seq out = approach;
//		profile(this->trjDuration, approach, out, silent);
		pHeuristic->setPointCloudCollision(true);

		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			spam::data::R2GTrajectory* trj = is<spam::data::R2GTrajectory>(ptr);
			if (!trj)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trj->setWaypoints(grasp::Waypoint::make(out, out));
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
		Controller::State::Seq seq, trnSeq;
		trajectory.createAction(seq);
		// transform w.r.t. query frame
		transformTrajectory(grasp::to<Data>(dataPtr)->queryTransform, seq.begin(), seq.end(), trnSeq);

		context.write("Seq size %d pregrasp %d\n", seq.size(), trajectory.pregraspIdx);
		Controller::State::Seq approach;
		findTrajectory(trnSeq[0], &trnSeq.back(), nullptr, 0, approach);

		Controller::State::Seq out = approach;
		//		profile(this->trjDuration, approach, out, silent);

		// add trajectory waypoint
		grasp::data::Item::Map::iterator ptr;
		{
			RenderBlock renderBlock(*this);
			golem::CriticalSectionWrapper cswData(getCS());
			to<Data>(dataPtr)->itemMap.erase(trjItemName);
			ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
			spam::data::R2GTrajectory* trj = is<spam::data::R2GTrajectory>(ptr);
			if (!trj)
				throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
			// add current state
			trj->setWaypoints(grasp::Waypoint::make(out, out));
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

				//// select index 
				//U32 index = 3;
				////selectIndex(trajectory, index, "waypoint");
				//index -= 1;

				//// grasp configuration (fingers closed)
				//Controller::State cend = lookupState();
				//for (auto i = handInfo.getJoints().begin(); i != handInfo.getJoints().end(); ++i)
				//	cend.cpos[i] = trajectory[index].state.cpos[i];

				//for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
				//	for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
				//		const size_t k = j - handInfo.getJoints().begin();
				//		cend.cpos[j] = vl[k];
				//	}
				//	if (i == handInfo.getChains().begin() + 1)
				//		break;
				//}

				//Controller::State::Seq approach;
				//approach.push_back(cstart);
				//approach.push_back(cend);
				////findTrajectory(lookupState(), &cend, nullptr, 0, approach);

				//Controller::State::Seq out;
				//profile(Real(5.0)/*this->trjDuration*/, approach, out, silent);

				//// add trajectory waypoint
				//grasp::data::Item::Map::iterator ptr;
				//{
				//	RenderBlock renderBlock(*this);
				//	golem::CriticalSectionWrapper cswData(getCS());
				//	to<Data>(dataPtr)->itemMap.erase(trjItemName);
				//	ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, queryHandlerTrj->create()));
				//	grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
				//	if (!trajectory)
				//		throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
				//	// add current state
				//	trajectory->setWaypoints(grasp::Waypoint::make(out, out));
				//	Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
				//}


				//// disable active controller to apply enough force to the object to lift it.
				//if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED) {
				//	armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
				//	armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
				//}
				//// perform
				//try {
				//	perform(dataPtr->first, ptr->first, out, !silent);
				//}
				//catch (const Message& msg) {
				//	context.write("%s\n", msg.str().c_str());
				//	resetPlanning();
				//	return false;
				//}

				//// print distance to the targt configuration
				//Controller::State cfg = lookupState();
				//std::stringstream ss;
				//for (auto i = handInfo.getJoints().begin(); i != handInfo.getJoints().end(); ++i) {
				//	const size_t k = i - handInfo.getJoints().begin();
				//	ss << "c=" << k << " [" << cend.cpos[i] - cfg.cpos[i] << "]/t";
				//}
				//context.write("Hand joints error:\n%s\n", ss.str().c_str());

				//resetPlanning();
				return true;
	}
	case 'U':
	{
				//context.debug("Plan lifting up\n");
				//std::string trjItemName("liftingTrj");
				//resetPlanning();
				//isGrasping = true;

				//Controller::State cend = lookupState();

				//for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
				//	for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
				//		const size_t k = j - handInfo.getJoints().begin();
				//		cend.cpos[j] = vl[k];
				//	}
				//	if (i == handInfo.getChains().begin() + 1)
				//		break;
				//}
				//Controller::State init = cend;
				//// transform w.r.t. query frame 
				//try {
				//	findTarget(Mat34::identity(), cend, cend, true);
				//}
				//catch (const Message& msg) {
				//	context.write("%s\n", msg.str().c_str());
				//	resetPlanning();
				//	return false;
				//}

				////Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(target), manipulator->getConfig(target).toMat34());
				////renderHand(target, bounds, true);

				////for (auto j = handInfo.getJoints().begin() + 1; j != handInfo.getJoints().end(); ++j) {
				////	const size_t k = j - handInfo.getJoints().begin();
				////	target.cpos[j] = vl[k];
				////}

				//Controller::State::Seq approach;
				//try {
				//	findTrajectory(init, &cend, nullptr, 0, approach);
				//}
				//catch (const Message& msg) {
				//	context.write("%s\n", msg.str().c_str());
				//	resetPlanning();
				//	return false;
				//}

				//Controller::State::Seq out;
				//profile(Real(5.0)/*this->trjDuration*/, approach, out, silent);

				//// add trajectory waypoint
				//grasp::data::Item::Map::iterator ptr;
				//{
				//	RenderBlock renderBlock(*this);
				//	golem::CriticalSectionWrapper cswData(getCS());
				//	to<Data>(dataPtr)->itemMap.erase(trjItemName);
				//	ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trjItemName, modelHandlerTrj->create()));
				//	grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
				//	if (!trajectory)
				//		throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
				//	// add current state
				//	trajectory->setWaypoints(grasp::Waypoint::make(out, out));
				//	Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
				//}

				//// disable active controller to apply enough force to the object to lift it.
				//if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_DISABLED) {
				//	armHandForce->getArmCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
				//	armHandForce->getHandCtrl()->setMode(ActiveCtrlForce::MODE_DISABLED);
				//}
				//// perform
				//try {
				//	perform(dataPtr->first, ptr->first, out, !silent);
				//}
				//catch (const Message& msg) {
				//	context.write("%s\n", msg.str().c_str());
				//	resetPlanning();
				//	return false;
				//}

				//// open hand and release the object
				//// pre-grasp pose w.r.t. query frame
				////::sleep(1000);
				//Controller::State openfingers = lookupState();
				//Controller::State cnow = lookupState();
				//for (auto i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
				//	for (auto j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j)
				//		openfingers.cpos[j] = trajectory[0].state.cpos[j]; // pre-grasp fingers' pose
				//}
				//// transform w.r.t. query frame 
				//findTarget(Mat34::identity(), openfingers, openfingers);

				////Bounds::Seq bounds = manipulator->getBounds(manipulator->getConfig(openfingers), manipulator->getConfig(openfingers).toMat34());
				////renderHand(openfingers, bounds, true);
				////Bounds::Seq bounds2 = manipulator->getBounds(manipulator->getConfig(cnow), manipulator->getConfig(cnow).toMat34());
				////renderHand(openfingers, bounds2, false);

				//Controller::State::Seq openTrj;
				//openTrj.push_back(cnow);
				//openTrj.push_back(openfingers);

				//Controller::State::Seq out2;
				//profile(Real(2.0), openTrj, out2, silent);
				//std::string releaseTrjName = "releaseObjTrj";

				//// add trajectory waypoint
				//{
				//	RenderBlock renderBlock(*this);
				//	golem::CriticalSectionWrapper cswData(getCS());
				//	to<Data>(dataPtr)->itemMap.erase(releaseTrjName);
				//	ptr = to<Data>(dataPtr)->itemMap.insert(to<Data>(dataPtr)->itemMap.end(), grasp::data::Item::Map::value_type(releaseTrjName, modelHandlerTrj->create()));
				//	grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(ptr);
				//	if (!trajectory)
				//		throw Message(Message::LEVEL_ERROR, "Demo::selectTrajectory(): Trajectory handler does not implement data::Trajectory");
				//	// add current state
				//	trajectory->setWaypoints(grasp::Waypoint::make(out, out));
				//	Data::View::setItem(to<Data>(dataPtr)->itemMap, ptr, to<Data>(dataPtr)->getView());
				//}

				//// perform trajectory
				//try {
				//	perform(dataPtr->first, ptr->first, out2, !silent);
				//}
				//catch (const Message& msg) {
				//	context.write("%s\n", msg.str().c_str());
				//	resetPlanning();
				//	return false;
				//}
				//// disable active controller to apply enough force to the object to grasp.
				//if (armHandForce->getArmCtrl()->getMode() != ActiveCtrlForce::MODE_ENABLED || armHandForce->getHandCtrl()->getMode() != ActiveCtrlForce::MODE_ENABLED) {
				//	armHandForce->getArmCtrl()->setMode(armMode/*ActiveCtrlForce::MODE_ENABLED*/);
				//	armHandForce->getHandCtrl()->setMode(handMode/*ActiveCtrlForce::MODE_ENABLED*/);
				//}


				//resetPlanning();
				return true;
	}
	}
	return false;
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return FTDemo::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
