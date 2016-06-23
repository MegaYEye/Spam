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

	menuCmdMap.insert(std::make_pair("ZX", [=]() {
		// debug mode
		const bool stopAtBreakPoint = option("YN", "Debug mode (Y/N)...") == 'Y';
		const auto breakPoint = [=](const char* str) {
			if (!stopAtBreakPoint && str)
				context.write("%s....\n", str);
			if (stopAtBreakPoint && str ? option("YN", makeString("%s: Continue (Y/N)...", str).c_str()) != 'Y' : waitKey(5) == 27)
				throw Cancel("Demo cancelled");
		};

		// setup initial variables
		const Controller::State home = grasp::Waypoint::lookup(*controller).command;

		std::string r2gHandlerItemName = "SpamDataR2GTrajectory+SpamDataR2GTrajectoryDemoR2G";
		grasp::data::Handler::Map::const_iterator r2gTrjHandlerPtr = handlerMap.find(r2gHandlerItemName);
		grasp::data::Handler* r2gTrjHandler = r2gTrjHandlerPtr != handlerMap.end() ? r2gTrjHandlerPtr->second.get() : nullptr;
		if (!r2gTrjHandler)
			throw Message(Message::LEVEL_CRIT, "spam::PosePlanner::create(): unknown model (scan) data handler: %s", r2gHandlerItemName.c_str());

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
			gotoConfig(home);
		};
		auto resetDataPtr = [&](Data::Map::iterator& dataPtr) {
			to<Data>(dataPtr)->queryPoints.clear();
			to<Data>(dataPtr)->queryTransform.setId();
			to<Data>(dataPtr)->queryFrame.setId();
			to<Data>(dataPtr)->simulateObjectPose.clear();
		};

		grasp::to<Data>(dataCurrentPtr)->stratType = Strategy::NONE_STRATEGY;
		grasp::to<Data>(dataCurrentPtr)->actionType = action::NONE_ACTION;
		grasp::data::Item::Map::iterator modelContact, modelPtr, queryContactPtr, queryTrjPtr, trjPtr;
		grasp::data::Item::Ptr queryGraspTrj;
		U32 modelViews = modelScanPoseSeq.size(), /*queryViews = queryScanPoseSeq.size(),*/ trials = 5;
		const U32 maxFailures = 2, maxIterations = 5;
		bool grasped = false;

		// command keys
		std::string loadBeliefCmd("BL");
		std::string createModelCmd("PM");
		std::string createQueryCmd("PQ");
		std::string itemTransCmd("IT");
		std::string itemConvCmd("IC");
		std::string dataSaveCmd("DS");
		std::string itemRemoveCmd("IR");
		std::string trajectoryItem("trj");

		//--------------------------------------------------------------------//
		// CREATE A MODEL POINT CLOUD
		//--------------------------------------------------------------------//
		if (stopAtBreakPoint) {
			readString("Enter object name: ", object);
			// item name
			readNumber("Enter iteration: ", iteration);
		}
		else 
			object = "debug";
		
		const int loadbelief = stopAtBreakPoint ? option("YN", "Load a belief state? (Y/N)") : 'Y';
		if (loadbelief == 'Y')
			executeCmd(loadBeliefCmd);
		else {
			// create a model for the object
			context.write("Create a model...\n");
			for (; to<Data>(dataCurrentPtr)->modelPoints.empty();)
				executeCmd(createModelCmd); // move the robot to the home pose after scanning
		}//reset();

		//--------------------------------------------------------------------//
		// CREATE A PREDICTIVE MODEL FOR THE GRASP
		//--------------------------------------------------------------------//
		// create a grasp
		context.write("Create a predictive contact model for the grasp [%s]...\n", modelGraspItem.c_str());
		const int createModel = stopAtBreakPoint ? option("YN", "Create a new contact model? (Y/N)") : 'N';
		if (createModel == 'Y') {
			dataItemLabel = modelGraspItem;
			executeCmd(itemTransCmd);
			modelGraspItem = dataItemLabel;
		}

		//--------------------------------------------------------------------//
		// CREATE LOG
		//--------------------------------------------------------------------//
		std::string dirLog = makeString("./data/boris/experiments/%s/", object.c_str());
		golem::mkdir(dirLog.c_str());
		std::stringstream prefixLog;
		prefixLog << std::fixed << std::setprecision(3) << dirLog << "output";// << "-" << context.getTimer().elapsed();
		const std::string outputLog = makeString("%s.log", prefixLog.str().c_str());
		std::ofstream logFile(outputLog);
		std::stringstream prefixMicroft;
		prefixMicroft << std::fixed << std::setprecision(3) << dirLog << "MICROFT";// << "-" << context.getTimer().elapsed();
		const std::string outputMicroft = makeString("%s.log", prefixMicroft.str().c_str());
		std::ofstream logMicroft(outputMicroft);
		std::stringstream prefixIrene;
		prefixIrene << std::fixed << std::setprecision(3) << dirLog << "IRENE";// << "-" << context.getTimer().elapsed();
		const std::string outputIrene = makeString("%s.log", prefixIrene.str().c_str());
		std::ofstream logIrene(outputIrene);
		// collects results as string
		std::string results;

		// copy original data
		Data::Ptr testData = createData();
		testData.reset(to<Data>(dataCurrentPtr)->clone());

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
				context.write("TRIAL: %d Strategy: %s\n", trial, strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str());

				//--------------------------------------------------------------------//
				// CREATE DATA TRIAL
				//--------------------------------------------------------------------//
				std::string dataTrialPath = makeString("./data/boris/experiments/%s/trial0%d/%s/data.xml", object.c_str(), trial, strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str());
				const Strategy tmp = grasp::to<Data>(dataCurrentPtr)->stratType;
				Data::Ptr dataTrial = createData();
				dataTrial.reset(to<Data>(testData)->clone());
				// overwrite strategy
				to<Data>(dataTrial)->stratType = tmp;
				{
					golem::CriticalSectionWrapper cswData(scene.getCS());
					dataMap.erase(dataPath);
					dataCurrentPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(dataTrialPath, dataTrial));
					setCurrentDataPtr(dataCurrentPtr);
					// reset current view on the belief state
					currentBeliefPtr = to<Data>(dataCurrentPtr)->itemMap.find(currentBeliefItem);
					if (currentBeliefPtr != to<Data>(dataCurrentPtr)->itemMap.end())
						Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, currentBeliefPtr, to<Data>(dataCurrentPtr)->getView());
					else
						context.write("%s is not available\n", currentBeliefItem.c_str());
				}
				// force to draw the belief state
				drawBeliefState = true;
				to<Data>(dataCurrentPtr)->createRender();

				//--------------------------------------------------------------------//
				// CREATE A QUERY POINT CLOUD
				//--------------------------------------------------------------------//
				// create a query for the object
				context.write("Create a query...\n");
				//resetDataPtr(dataCurrentPtr);
				if (loadbelief == 'N')
					for (; to<Data>(dataCurrentPtr)->queryPoints.empty();)
						executeCmd(createQueryCmd);
				// set the simulated object
				if (!to<Data>(dataCurrentPtr)->simulateObjectPose.empty()) {
					//sensorBundlePtr->getCollisionPtr()->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
					collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
					objectPointCloudPtr.reset(new grasp::Cloud::PointSeq(grasp::to<Data>(dataCurrentPtr)->simulateObjectPose));
				}
				reset(); // move the robot to the home pose after scanning

				// write results
				results = grasp::makeString("%u\t%u\t%u", modelViews, queryViews, trial + 1);

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
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, currentBeliefPtr, to<Data>(dataCurrentPtr)->getView());
					//Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryContactPtr, to<Data>(dataCurrentPtr)->getView());
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
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, currentBeliefPtr, to<Data>(dataCurrentPtr)->getView());
					//Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, queryTrjPtr, to<Data>(dataCurrentPtr)->getView());
				}
				to<Data>(dataCurrentPtr)->createRender();
				context.write("done.\n");

				context.write("Transform: handler %s, input %s...\n", queryHandlerTrj->getID().c_str(), queryTrjPtr->first.c_str());
				//spam::data::R2GTrajectory* trajectory = is<spam::data::R2GTrajectory>(queryTrjPtr);
				grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(queryTrjPtr);
				if (!trajectory)
					throw Message(Message::LEVEL_ERROR, "Handler %s does not support Trajectory interface", queryHandlerTrj->getID().c_str());
				grasp::Waypoint::Seq inp = trajectory->getWaypoints();
				if (inp.size() < 3)
					throw Cancel("Error: the selected trajectory have not at least 3 waypoints.");

				// set render to show only mean hypothesis and ground truth
				resetDataPointers();
				to<Data>(dataCurrentPtr)->createRender();

				//--------------------------------------------------------------------//
				// EXECUTE TRIAL
				//--------------------------------------------------------------------//
				U32 iteration = 1, failures = 0;
				const RBCoord obj(simQueryFrame * to<Data>(dataCurrentPtr)->modelFrame);
				for (; failures < maxFailures && iteration < maxIterations;) {
					grasp::to<Data>(dataCurrentPtr)->actionType = action::IG_PLAN_M2Q;

					// compute misalignment
					grasp::RBCoord queryPose(grasp::to<Data>(dataCurrentPtr)->queryFrame);
					grasp::RBDist error;
					error.lin = obj.p.distance(queryPose.p);
					error.ang = obj.q.distance(queryPose.q);

					if (iteration == 1 && failures == 0)
						results = grasp::makeString("%s\t%.2f\t%.6f\t%.6f", results.c_str(), ((Real)grasp::to<Data>(dataCurrentPtr)->queryPoints.size() / (Real)modelPoints.size()) * (Real)100, error.lin, error.ang);

					//--------------------------------------------------------------------//
					// TRANSFORM GRASPING TRAJECTORY TO THE QUERY POSE
					//--------------------------------------------------------------------//
					// generate features
					grasp::data::Transform* btransform = is<grasp::data::Transform>(r2gTrjHandler);
					if (!btransform)
						throw Message(Message::LEVEL_ERROR, "Handler %s does not support Transform interface", r2gTrjHandler->getID().c_str());

					// retrive belief state
					list.clear();
					currentBeliefPtr = to<Data>(dataCurrentPtr)->itemMap.find(currentBeliefItem);
					if (currentBeliefPtr == to<Data>(dataCurrentPtr)->itemMap.end())
						throw Message(Message::LEVEL_ERROR, "FTDemo: Does not support belief state handler %s.", currentBeliefItem.c_str());
					list.insert(list.end(), currentBeliefPtr);

					// insert trajectory
					list.insert(list.end(), queryTrjPtr); // grasp on model
					grasp::data::Item::Ptr queryR2GTrajectoryPtr = btransform->transform(list);
					// insert processed object, remove old one
					RenderBlock renderBlock0(*this);
					{
						golem::CriticalSectionWrapper cswData(getCS());
						to<Data>(dataCurrentPtr)->itemMap.erase(trajectoryItem);
						trjPtr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), grasp::data::Item::Map::value_type(trajectoryItem, queryR2GTrajectoryPtr));
//						Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, currentBeliefPtr, to<Data>(dataCurrentPtr)->getView());
					}
					to<Data>(dataCurrentPtr)->createRender();

					spam::data::R2GTrajectory* r2gtrajectory = is<spam::data::R2GTrajectory>(trjPtr);
					if (!r2gtrajectory)
						throw Message(Message::LEVEL_ERROR, "Handler %s does not support R2GTrajectory interface", beliefHandler->getID().c_str());

					Controller::State::Seq seq;
					if (grasp::to<Data>(dataCurrentPtr)->stratType != Strategy::IR3NE)
						r2gtrajectory->createTrajectory(grasp::Waypoint::lookup(*controller).command, seq);
					else
						r2gtrajectory->createIGTrajectory(grasp::Waypoint::lookup(*controller).command, seq);
					
					// find global trajectory & perform
					R2GPlanner::perform(dataCurrentPtr->first, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str(), seq, false);

					if (contactOccured && grasp::to<Data>(dataCurrentPtr)->stratType != Strategy::ELEMENTARY) {
						contactOccured = false;
						updateAndResample(dataCurrentPtr);
						enableForceReading = false;
						++iteration;
						continue;
						//}
					}
					grasp::to<Data>(dataCurrentPtr)->actionType = action::GRASP;
					seq.clear();
					r2gtrajectory->createTrajectory(seq);
					// find global trajectory & perform
					R2GPlanner::perform(dataCurrentPtr->first, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str(), seq, false);
					context.write("Iteration %u: execute trajectory (%s)\n", iteration, actionToString(grasp::to<Data>(dataCurrentPtr)->actionType).c_str());
					if (option("YN", "Success? (Y/N)") == 'Y')
						grasped = true;
					else
						grasped = false;

					results = grasp::makeString("%s\t%.6f\t%.6f\t%u\t%u\n", results.c_str(), error.lin, error.ang, grasped ? 1 : 0, iteration);
					break;
				}
				// save results
				switch (grasp::to<Data>(dataCurrentPtr)->stratType){
					case Strategy::MYCROFT:
						context.write("strategy: %s\n%s", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), results.c_str());
						logMicroft << grasp::makeString("%s", results.c_str());
						logMicroft.flush();
						break;
					case Strategy::IR3NE:
						context.write("strategy: %s\n%s", strategy(grasp::to<Data>(dataCurrentPtr)->stratType).c_str(), results.c_str());
						logIrene << grasp::makeString("%s", results.c_str());
						logIrene.flush();
						break;
				}
				results = "";
				readPath("Enter data path to save: ", dataTrialPath, dataExt.c_str());
				if (getExt(dataTrialPath).empty())
					dataTrialPath += dataExt;
				to<Manager::Data>(dataCurrentPtr)->save(dataTrialPath);
				// reset robot
				reset();
			} // end strategy
		} // end trial
		//finish
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("ZP", [=]() {
		grasp::data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<grasp::data::Item::Map::const_iterator>(true);
		grasp::data::Trajectory* trajectory = is<grasp::data::Trajectory>(item->second.get());
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

}

//------------------------------------------------------------------------------

void FTDemo::render() const {
	R2GPlanner::render();
}

void FTDemo::perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	findTrajectory(grasp::Waypoint::lookup(*controller).command, &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

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
	trajectoryIf->setWaypoints(grasp::Waypoint::make(completeTrajectory, completeTrajectory)/*grasp::Waypoint::make(trajectory, trajectory)*/);

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

	// go to initial state
	sendTrajectory(initTrajectory);
	// wait until the last trajectory segment is sent
	controller->waitForEnd();

	Controller::State::Seq robotPoses; robotPoses.clear();
	TwistSeq ftSeq; ftSeq.clear();
	std::vector<grasp::RealSeq> forceInpSensorSeq;
	TwistSeq thumbFT, indexFT, wristFT;
	TwistSeq rawThumbFT, rawIndexFT, rawWristFT;
	FT::Data thumbData, indexData, wristData;
	grasp::RealSeq force; force.assign(18, golem::REAL_ZERO);

	// start recording
	recordingStart(data, item, true);
	recordingWaitToStart();

	// send trajectory
	sendTrajectory(trajectory);

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
		if (i % 10 == 0)
			context.write("State #%d (%s)\r", i, enableForceReading ? "Y" : "N");
		if (grasp::to<Data>(dataCurrentPtr)->actionType != action::GRASP && i > 600) {
			enableForceReading = expectedCollisions(state);
		}
	}
	enableForceReading = false;

	// stop recording
	recordingStop(getPlanner().trajectoryIdlePerf);
	recordingWaitToStop();

	std::string dir = makeString("%s%s/%s/trial0%d/", ftpath.c_str(), object.c_str(), item.c_str(), iteration++);
	golem::mkdir(dir.c_str());
	std::stringstream prefixWrist;
	prefixWrist << std::fixed << std::setprecision(3) << dir << "wrist" << "-" << context.getTimer().elapsed();
	std::stringstream prefixThumb;
	prefixThumb << std::fixed << std::setprecision(3) << dir << "thumb" << " - " << context.getTimer().elapsed();
	std::stringstream prefixIndex;
	prefixIndex << std::fixed << std::setprecision(3) << dir << "index" << " - " << context.getTimer().elapsed();

	std::stringstream prefixRawWrist;
	prefixRawWrist << std::fixed << std::setprecision(3) << dir << "raw_wrist" << " - " << context.getTimer().elapsed();
	std::stringstream prefixRawThumb;
	prefixRawThumb << std::fixed << std::setprecision(3) << dir << "raw_thumb" << "-" << context.getTimer().elapsed();
	std::stringstream prefixRawIndex;
	prefixRawIndex << std::fixed << std::setprecision(3) << dir << "raw_index" << "-" << context.getTimer().elapsed();

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

	// writing data into a text file, prepare headers
	strFTDesc(wristSS, std::string("ft_"));
	strFTDesc(rawWristSS, std::string("ft_"));
	strFTDesc(thumbSS, std::string("ft_"));
	strFTDesc(rawThumbSS, std::string("ft_"));
	strFTDesc(indexSS, std::string("ft_"));
	strFTDesc(rawIndexSS, std::string("ft_"));

	for (U32 indexjj = 0; indexjj < thumbFT.size(); ++indexjj) {
		const SecTmReal t = robotPoses[indexjj].t - recorderStart;
		strFT(wristSS, wristFT[indexjj], robotPoses[indexjj].t, t);
		strFT(rawWristSS, rawWristFT[indexjj], robotPoses[indexjj].t, t);
		strFT(thumbSS, thumbFT[indexjj], robotPoses[indexjj].t, t);
		strFT(rawThumbSS, rawThumbFT[indexjj], robotPoses[indexjj].t, t);
		strFT(indexSS, indexFT[indexjj], robotPoses[indexjj].t, t);
		strFT(rawIndexSS, rawIndexFT[indexjj], robotPoses[indexjj].t, t);
	}

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

int main(int argc, char *argv[]) {
	return FTDemo::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
