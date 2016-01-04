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
#include <Golem/Plan/Data.h>
//#include <Golem/Phys/PhysScene.h>
#include <algorithm>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>

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
		desc = "Press a key to: (D)ata gathering/(B)elief update test...";
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
			sensorBundlePtr->collisionPtr->create(rand, grasp::to<Data>(dataCurrentPtr)->simulateObjectPose);
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

}

//------------------------------------------------------------------------------

void FTDemo::render() const {
	R2GPlanner::render();
}

void FTDemo::perform(const std::string& data, const std::string& item, const golem::Controller::State::Seq& trajectory, bool testTrajectory) {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): At least two waypoints required");

	golem::Controller::State::Seq initTrajectory;
	findTrajectory(lookupState(), &trajectory.front(), nullptr, SEC_TM_REAL_ZERO, initTrajectory);

	golem::Controller::State::Seq completeTrajectory = initTrajectory;
	completeTrajectory.insert(completeTrajectory.end(), trajectory.begin(), trajectory.end());

	// create trajectory item
	grasp::data::Item::Ptr itemTrajectory;
	grasp::data::Handler::Map::const_iterator handlerPtr = handlerMap.find(trajectoryHandler);
	if (handlerPtr == handlerMap.end())
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unknown default trajectory handler %s", trajectoryHandler.c_str());
	grasp::data::Handler* handler = is<grasp::data::Handler>(handlerPtr);
	if (!handler)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): invalid default trajectory handler %s", trajectoryHandler.c_str());
	itemTrajectory = handler->create();
	grasp::data::Trajectory* trajectoryIf = is<grasp::data::Trajectory>(itemTrajectory.get());
	if (!trajectoryIf)
		throw Message(Message::LEVEL_ERROR, "Player::perform(): unable to create trajectory using handler %s", trajectoryHandler.c_str());
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

	// start recording
	recordingStart(data, item, true);
	recordingWaitToStart();

	// send trajectory
	sendTrajectory(trajectory);

	Controller::State::Seq robotPoses; robotPoses.clear();
	TwistSeq ftSeq; ftSeq.clear();
	std::vector<grasp::RealSeq> forceInpSensorSeq;
	TwistSeq thumbFT, indexFT, wristFT;
	TwistSeq rawThumbFT, rawIndexFT, rawWristFT;
	FT::Data thumbData, indexData, wristData;
	grasp::RealSeq force; force.assign(18, golem::REAL_ZERO);
	sensorBundlePtr->start2read = true;
	// repeat every send waypoint until trajectory end
	for (U32 i = 0; controller->waitForBegin(); ++i) {
		if (universe.interrupted())
			throw Exit();
		if (controller->waitForEnd(0))
			break;

		Controller::State state = lookupState();
		robotPoses.push_back(state);
		sensorBundlePtr->increment();

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
		if (!isGrasping && i > 200) {
			enableForceReading = expectedCollisions(state);
		}
	}
	enableForceReading = false;
	sensorBundlePtr->start2read = false;

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

	//// writing data into a text file, prepare headers
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

	// stop recording
	recordingStop(trajectoryIdlePerf);
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

int main(int argc, char *argv[]) {
	return FTDemo::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
