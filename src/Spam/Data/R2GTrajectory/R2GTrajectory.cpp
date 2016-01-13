/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Spam/Data/R2GTrajectory/R2GTrajectory.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Plan/Data.h>
#include <Golem/UI/Data.h>
#include <Golem/UICtrl/Data.h>
#include <Grasp/Core/Import.h>
#include <Grasp/Core/Ctrl.h>
#include <Grasp/Contact/Manipulator.h>
#include <Grasp/Contact/Model.h>
#include <Grasp/Data/Image/Image.h>
#include <boost/algorithm/string.hpp>
#include <pcl/io/pcd_io.h>

//-----------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace grasp::data;
using namespace spam;
using namespace spam::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* graspDescLoader() {
	// Create description
	return new HandlerR2GTrajectory::Desc();
}

//------------------------------------------------------------------------------

spam::data::ItemR2GTrajectory::ItemR2GTrajectory(HandlerR2GTrajectory& handler) : ItemTrajectory(handler), handler(handler), robotHandPoseFile(handler.file), objectPoseFile(handler.file), objectPointFile(handler.file), jointsPoseFile(handler.file) {
}

Item::Ptr spam::data::ItemR2GTrajectory::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemR2GTrajectory::clone(): not implemented");
}

void spam::data::ItemR2GTrajectory::createRender() {
	handler.createRender(*this);
}

void spam::data::ItemR2GTrajectory::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	ItemTrajectory::load(prefix, xmlcontext);

	// robotHand pose
	std::string robotHandPoseSuffix;
	golem::XMLData("robot_hand_pose", robotHandPoseSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (robotHandPoseSuffix.length() > 0) {
		robotHandPoseFile.load(prefix + robotHandPoseSuffix, [&](const std::string& path) {
			robotHandPoses.clear();
			FileReadStream(path.c_str()).read(robotHandPoses, robotHandPoses.end());
		});
	}

	// joints pose
	std::string jointsPoseSuffix;
	golem::XMLData("joints_pose", jointsPoseSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (jointsPoseSuffix.length() > 0) {
		jointsPoseFile.load(prefix + jointsPoseSuffix, [&](const std::string& path) {
			jointsPoses.clear();
			FileReadStream(path.c_str()).read(jointsPoses, jointsPoses.end());
		});
	}
}

void spam::data::ItemR2GTrajectory::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	ItemTrajectory::save(prefix, xmlcontext);

	// pose xml
	std::string robotHandPoseSuffix = !robotHandPoses.empty() ? "hand" + handler.poseSuffix : "";
	golem::XMLData("robot_hand_pose", robotHandPoseSuffix, xmlcontext, true);
	// pose binary
	if (robotHandPoseSuffix.length() > 0) {
		robotHandPoseFile.save(prefix + robotHandPoseSuffix, [=](const std::string& path) {
			FileWriteStream(path.c_str()).write(robotHandPoses.begin(), robotHandPoses.end());
		});
	}
	else
		robotHandPoseFile.remove();

	// pose xml
	std::string jointsPoseSuffix = !jointsPoses.empty() ? "joints" + handler.poseSuffix : "";
	golem::XMLData("joints_pose", jointsPoseSuffix, xmlcontext, true);
	// pose binary
	if (jointsPoseSuffix.length() > 0) {
		jointsPoseFile.save(prefix + jointsPoseSuffix, [=](const std::string& path) {
			FileWriteStream(path.c_str()).write(jointsPoses.begin(), jointsPoses.end());
		});
	}
	else
		jointsPoseFile.remove();
}

void spam::data::ItemR2GTrajectory::setWaypoints(const grasp::Waypoint::Seq& waypoints) {
	ItemTrajectory::setWaypoints(waypoints);
}

void spam::data::ItemR2GTrajectory::createTrajectory(golem::Controller::State::Seq& trajectory) {
	handler.createTrajectory(*this, trajectory);
}

//------------------------------------------------------------------------------

Item::Ptr spam::data::ItemR2GTrajectory::convert(const Handler& handler) {
	return this->handler.convert(*this, handler);
}

const StringSeq& spam::data::ItemR2GTrajectory::getConvertInterfaces() const {
	return this->handler.convertInterfaces;
}

bool spam::data::ItemR2GTrajectory::isConvertSupported(const Handler& handler) const {
	return this->handler.isConvertSupported(handler);
}

//------------------------------------------------------------------------------

void spam::data::ItemR2GTrajectory::exportt(const std::string& path) const {
	handler.exportt(*this, path);
}

const StringSeq& spam::data::ItemR2GTrajectory::getExportFileTypes() const {
	return handler.getExportFileTypes();
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::data::HandlerTrajectory::Desc::load(context, xmlcontext);

	manipulatorDesc->load(xmlcontext->getContextFirst("manipulator"));

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("trajectory", false);

	golem::XMLData("pose_suffix", poseSuffix, pxmlcontext, false);

	importRobotHandPose.load(pxmlcontext->getContextFirst("import robot_hand_pose"));
	importObjectPose.load(pxmlcontext->getContextFirst("import object_pose"));
	importJointsPose.load(pxmlcontext->getContextFirst("import joints_pose"));

	Mat34Seq seq;
	golem::XMLData(seq, seq.max_size(), pxmlcontext->getContextFirst("import robot_trj", false), "transform");
	importRobotTrjTransform.setId();
	for (Mat34Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
		importRobotTrjTransform.multiply(importRobotTrjTransform, *i);

	golem::XMLData("robot_hand_pose", importHDF5RobotHandPose, pxmlcontext->getContextFirst("import hdf5", false), false);
	golem::XMLData("object_pose", importHDF5ObjectPose, pxmlcontext->getContextFirst("import hdf5", false), false);

	try {
		importHDF5JointsPose.clear();
		XMLGetValue(importHDF5JointsPose, "joints_pose", "name", pxmlcontext->getContextFirst("import hdf5", false));
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}

	golem::XMLData(appearanceHandFrameSize, pxmlcontext->getContextFirst("appearance hand_frame_size"), false);
	golem::XMLData(appearanceJointsFrameSize, pxmlcontext->getContextFirst("appearance joints_frame_size"), false);
}

grasp::data::Handler::Ptr spam::data::HandlerR2GTrajectory::Desc::create(golem::Context &context) const {
	Handler::Ptr handler(new HandlerR2GTrajectory(context));
	to<HandlerR2GTrajectory>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

spam::data::HandlerR2GTrajectory::HandlerR2GTrajectory(golem::Context &context) : HandlerTrajectory(context) {
}

void spam::data::HandlerR2GTrajectory::create(const Desc& desc) {
	grasp::data::HandlerTrajectory::create(desc);

	// shallow copy, TODO clone member function
	manipulatorDesc = desc.manipulatorDesc;

	poseSuffix = desc.poseSuffix;

	importRobotHandPose = desc.importRobotHandPose;
	importObjectPose = desc.importObjectPose;
	importJointsPose = desc.importJointsPose;
	importRobotTrjTransform = desc.importRobotTrjTransform;
	importHDF5RobotHandPose = desc.importHDF5RobotHandPose;
	importHDF5ObjectPose = desc.importHDF5ObjectPose;
	importHDF5JointsPose = desc.importHDF5JointsPose;

	appearanceHandFrameSize = desc.appearanceHandFrameSize;
	appearanceJointsFrameSize = desc.appearanceJointsFrameSize;

	convertInterfaces = {
		"ItemImage",
	};

	exportTypes = {
		".txt",
	};
}

void spam::data::HandlerR2GTrajectory::set(golem::Planner& planner, const StringSeq& controllerIDSeq) {
	grasp::data::HandlerTrajectory::set(planner, controllerIDSeq);
	manipulator = manipulatorDesc->create(planner, controllerIDSeq);
	pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner.getHeuristic());
}

void spam::data::HandlerR2GTrajectory::create(const golem::ConfigspaceCoord& delta, golem::Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::create(): At least two waypoints required");

	Controller::State state = trajectory.back();

	// last two waypoint used to extrapolate coordinates
	const Controller::State* s[2] = { &trajectory[trajectory.size() - 1], &trajectory[trajectory.size() - 2] };
	// linear extrapolation
	for (Configspace::Index i = state.getInfo().getJoints().begin(); i != state.getInfo().getJoints().end(); ++i)
		state.cpos[i] = Math::clamp(s[0]->cpos[i] + delta[i] * (s[0]->cpos[i] - s[1]->cpos[i]), controller->getMin().cpos[i], controller->getMax().cpos[i]);

	trajectory.push_back(state);
}

void spam::data::HandlerR2GTrajectory::profile(golem::SecTmReal duration, golem::Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::profile(): At least two waypoints required");

	trajectory.front().t = golem::SEC_TM_REAL_ZERO;
	trajectory.back().t = duration;
	pProfile->profile(trajectory);
}

void spam::data::HandlerR2GTrajectory::createTrajectory(const ItemR2GTrajectory& item, golem::Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): planner and controller are not set");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): At least two waypoints required");

	Real trjExtrapolation = this->trjExtrapolation;
	Real trjDuration = this->trjDuration;
	bool inverse = false;
	bool createAction = arm && !action.empty();

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
		if (createAction) createAction = menu.option("YN", "Create action (Y/N)... ") == 'Y';
	}

	// make trajectory
	golem::Controller::State::Seq states = grasp::Waypoint::make(item.getWaypoints(), false), commands = grasp::Waypoint::make(item.getWaypoints(), true);

	// extrapolation delta
	golem::ConfigspaceCoord delta;
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
		delta[i] = trjExtrapolation*extrapolation[i];

	// extrapolation
	const bool extrapol = trjExtrapolation > REAL_EPS;
	if (extrapol) {
		create(delta, states);
		create(delta, commands);
	}

	const size_t size = commands.size();
	{
		// waypoint pruning
		ScopeGuard guard([&]() {distRemovedCallback = nullptr; });
		distRemovedCallback = [&](size_t index) {
			//context.debug("data::HandlerTrajectory::import(): removing index #%u/%u\n", U32(index + 1), states.size());
			states.erase(states.begin() + index);
			commands.erase(commands.begin() + index);
		};
		// trajectory profiling
		trajectory = commands;
		for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
		if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
			// replace with states
			for (size_t j = 0; j < trajectory.size(); ++j) {
				trajectory[j].cpos[i] = states[j].cpos[i];
				trajectory[j].cvel[i] = states[j].cvel[i];
				trajectory[j].cacc[i] = states[j].cacc[i];
			}
		}
		profile(trjDuration, trajectory);
	}

	// assign if command[i] > 0
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
	if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
		// replace with commands
		for (size_t j = 0; j < trajectory.size(); ++j) {
			trajectory[j].cpos[i] = commands[j].cpos[i];
			trajectory[j].cvel[i] = commands[j].cvel[i];
			trajectory[j].cacc[i] = commands[j].cacc[i];
		}
		if (extrapol)
			trajectory.back().cpos[i] = command[i];
	}

	// debug
	context.debug("HandlerTrajectory::createTrajectory(): Trajectory size %u -> %u\n", (U32)size, (U32)trajectory.size());

	// stationary waypoint in the end
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));

	// action
	if (createAction) {
		GenWorkspaceChainState gwcc;
		controller->chainForwardTransform(trajectory.back().cpos, gwcc.wpos);
		for (Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i)
			gwcc.wpos[i].multiply(gwcc.wpos[i], controller->getChains()[i]->getReferencePose()); // reference pose
		gwcc.t = trajectory.back().t;

		const Controller::State::Info armInfo = arm->getStateInfo();
		GenWorkspaceChainState::Seq wAction;
		wAction.push_back(gwcc);
		for (Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i) {
			GenWorkspaceChainState gwccTrn = gwcc;
			gwccTrn.wpos[armInfo.getChains().begin()].multiply(*i, gwcc.wpos[armInfo.getChains().begin()]);
			gwccTrn.t += trjDuration;
			wAction.push_back(gwccTrn);
		}
		// find trajectory
		golem::Controller::State::Seq cAction;
		if (!const_cast<Planner*>(planner)->findLocalTrajectory(trajectory.back(), wAction.begin(), wAction.end(), cAction, cAction.end()))
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::createTrajectory(): Unable to find action trajectory");
		// add
		trajectory.insert(trajectory.end(), ++cAction.begin(), cAction.end());
		trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
	}
}

//------------------------------------------------------------------------------

std::string spam::data::HandlerR2GTrajectory::getFileExtPose() {
	return std::string(".pose");
}

grasp::data::Item::Ptr spam::data::HandlerR2GTrajectory::create() const {
	return grasp::data::Item::Ptr(new spam::data::ItemR2GTrajectory(*const_cast<spam::data::HandlerR2GTrajectory*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

Item::Ptr spam::data::HandlerR2GTrajectory::import(const std::string& path) {
	// check extension
	if (std::find(importTypes.begin(), importTypes.end(), getExt(path)) == importTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): unknown file type %s", getExt(path).c_str());

	RenderBlock renderBlock(*this); // do not show cache

	Item::Ptr item(create());
	ItemR2GTrajectory* itemTrajectory = to<ItemR2GTrajectory>(item.get());
	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);

	// HDF5 format
	if (path.rfind(importHDF5FileExt) != std::string::npos) {
		//There are two stages of trajectory import :
		//1) Import of data points from HDF5 datasets is performed using definitions specified in: grasp::Import::HDF5DumpRealSeqMap map;
		//   this also include subsampling, all configurable in GraspDataTrajectoryUA.xml.
		//2) Trajectory construction which involve:
		//   a) Computing trajectory in the robot configuration space using wrist pose (hdf5 provides only hand configuration):  const RBDist err = manipulator->find(seq);
		//   b) Re-profile trajectory with additional waypoint pruning: profile(trjDuration, trajectory);

		Controller::State state = controller->createState(), command = state;

		if (menu != nullptr) {
			menu->readString("Enter robot trajectory dataset name: ", importHDF5RobotTrj);
			menu->readString("Enter robot hand pose dataset name: ", importHDF5RobotHandPose);
			menu->readString("Enter object pose dataset name: ", importHDF5ObjectPose);
		}

		// defines datasets import routines
		grasp::Import::HDF5DumpRealSeqMap map;

		// hand configurations and motor commands
		map.insert(std::make_pair(importHDF5RobotTrj, [&](const std::string& name, size_t index, const grasp::RealSeq& seq) {
			// subsampling in range <begin, end)
			if (index >= importRobotTrj.begin && index < importRobotTrj.end && (index - importRobotTrj.begin) % importRobotTrj.subsampling == 0) {
				// debug message
				if (index == importRobotTrj.begin)
					context.debug("HandlerR2GTrajectory::import(): importing HDF5 dataset \"%s\"\n", name.c_str());

				// state: reset
				controller->setToDefault(state);
				// state: increment time stamp by default
				if (!itemTrajectory->waypoints.empty()) state.t = itemTrajectory->waypoints.back().state.t + importRobotTrj.interval;
				// state: update using custom map
				ImportState::update(importRobotTrj.stateMap, seq, state);

				// command: reset
				controller->setToDefault(command);
				// command: increment time stamp by default
				if (!itemTrajectory->waypoints.empty()) command.t = itemTrajectory->waypoints.back().command.t + importRobotTrj.interval;
				// command: update using custom map
				ImportState::update(importRobotTrj.commandMap, seq, command);

				// update waypoints
				itemTrajectory->waypoints.push_back(grasp::Waypoint(state, command));
			}
		}));

		// wrist Cartesian poses
		map.insert(std::make_pair(importHDF5RobotHandPose, [&](const std::string& name, size_t index, const grasp::RealSeq& seq) {
			// subsampling in range <begin, end)
			if (index >= importRobotTrj.begin && index < importRobotTrj.end && (index - importRobotTrj.begin) % importRobotTrj.subsampling == 0) {
				//printf("import index=%u\n", index);
				// debug message
				if (index == importRobotTrj.begin)
					context.debug("HandlerR2GTrajectory::import(): importing HDF5 dataset \"%s\"\n", name.c_str());
				// pose
				Mat34 pose = Mat34::identity();
				// state: update using custom map
				importRobotHandPose.update(seq, pose);
				// update poses
				itemTrajectory->robotHandPoses.push_back(importRobotTrjTransform * pose); // transformation
			}
		}));

		// object poses
		map.insert(std::make_pair(importHDF5ObjectPose, [&](const std::string& name, size_t index, const grasp::RealSeq& seq) {
			// subsampling in range <begin, end)
			if (index >= importRobotTrj.begin && index < importRobotTrj.end && (index - importRobotTrj.begin) % importRobotTrj.subsampling == 0) {
				// debug message
				if (index == importRobotTrj.begin)
					context.debug("HandlerR2GTrajectory::import(): importing HDF5 dataset \"%s\"\n", name.c_str());
			}
		}));

		// Cartesian poses of all hand joints (for visualisation/debuging only)
		for (StringSeq::const_iterator i = importHDF5JointsPose.begin(); i != importHDF5JointsPose.end(); ++i) {
			map.insert(std::make_pair(*i, [&](const std::string& name, size_t index, const grasp::RealSeq& seq) {
				// subsampling in range <begin, end)
				if (index >= importRobotTrj.begin && index < importRobotTrj.end && (index - importRobotTrj.begin) % importRobotTrj.subsampling == 0) {
					// debug message
					if (index == importRobotTrj.begin)
						context.debug("HandlerR2GTrajectory::import(): importing HDF5 dataset \"%s\"\n", name.c_str());
					// pose
					Mat34 pose = Mat34::identity();
					// state: update using custom map
					importJointsPose.update(seq, pose);
					// update poses
					itemTrajectory->jointsPoses[name].push_back(importRobotTrjTransform * pose); // transformation
				}
			}));
		}

		// read datasets from file
		grasp::Import().hdf5DumpRealSeq(path, map);
		// assert data validity
		if (itemTrajectory->waypoints.empty())
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): empty robot trajectory");
		if (itemTrajectory->robotHandPoses.empty())
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): empty hand poses");

		// Make sure sizes of all data streams are the same
		const size_t size = std::min(itemTrajectory->waypoints.size(), itemTrajectory->robotHandPoses.size());
		itemTrajectory->waypoints.resize(size, grasp::Waypoint::create(*controller));
		itemTrajectory->robotHandPoses.resize(size);
		for (Mat34MapSeq::iterator i = itemTrajectory->jointsPoses.begin(); i != itemTrajectory->jointsPoses.end(); ++i) {
			if (i->second.empty())
				throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): HDF5 dataset \"%s\" is empty", i->first.c_str());
			i->second.resize(size, i->second.back()); // if smaller than size then fill in with the last pose
		}

		// Update robot arm configuration
		Mat34 localFrame = manipulator->getLocalFrame();
		localFrame.setInverse(localFrame);
		Manipulator::Waypoint::Seq seq;
		for (size_t j = 0; j < size; ++j)
			seq.push_back(Manipulator::Waypoint(itemTrajectory->waypoints[j].state.cpos, RBCoord(itemTrajectory->robotHandPoses[j]) * localFrame));

		// compute trajectory in the configuration space of the entire robot - pruning disabled!
		const RBDist err = manipulator->find(seq);
		context.debug("HandlerTrajectory::import(): Pose error: lin=%.9f, ang=%.9f\n", err.lin, err.ang);
		for (size_t j = 0; j < seq.size(); ++j) {
			itemTrajectory->waypoints[j].state.cpos.set(arm->getStateInfo().getJoints(), seq[j].config);
			itemTrajectory->waypoints[j].command.cpos.set(arm->getStateInfo().getJoints(), seq[j].config);
		}

		// trajectories
		golem::Controller::State::Seq states = grasp::Waypoint::make(itemTrajectory->waypoints, false), trajectory; // use states for profiling

		{
			// waypoint pruning - distRemovedCallback is called for every removed index
			ScopeGuard guard([&]() {distRemovedCallback = nullptr; });
			distRemovedCallback = [&](size_t index) {
				// remove waypoints with index in other datasets to be consistent with the main hand trajectory
				//context.debug("data::HandlerR2GTrajectory::import(): removing index #%u/%u\n", U32(index + 1), itemTrajectory->waypoints.size());
				itemTrajectory->waypoints.erase(itemTrajectory->waypoints.begin() + index);
				itemTrajectory->robotHandPoses.erase(itemTrajectory->robotHandPoses.begin() + index);
				for (Mat34MapSeq::iterator i = itemTrajectory->jointsPoses.begin(); i != itemTrajectory->jointsPoses.end(); ++i)
					i->second.erase(i->second.begin() + index);
			};
			// trajectory profiling - for details see golem::Profile::profile(). It consists of 4 stages:
			// 1) pruning and time profiling (events are handled by distRemovedCallback): golem::Profile::profileTime()
			// 2) computing velocities at each waypoint: golem::Profile::profileVelocity()
			// 3) velocity optimisation: golem::Profile::optimise()
			// 4) trajectory duration rescaling: golem::Profile::rescale()
			trajectory = states;
			profile(trjDuration, trajectory);
		}

		// assert size, this should never happen
		if (trajectory.size() != itemTrajectory->waypoints.size())
			throw Message(Message::LEVEL_ERROR, "HandlerTrajectory::import(): profile trajectory does not match input %u != !u", trajectory.size(), itemTrajectory->waypoints.size());
		// update time stamps
		for (size_t i = 0; i < trajectory.size(); ++i) {
			// time stamps are the same for states and corresponding commands
			itemTrajectory->waypoints[i].command.t = itemTrajectory->waypoints[i].state.t = trajectory[i].t;

			// uncomment for debuging:
			//std::string str;
			//str.reserve(golem::Message::MAX_SIZE);
			//// state
			//for (golem::Configspace::Index j = info.getJoints().begin(); j != info.getJoints().end(); ++j)
			//	str += ' ' + std::to_string(itemTrajectory->waypoints[i].state.cpos[j]);
			//// command
			//for (golem::Configspace::Index j = info.getJoints().begin(); j != info.getJoints().end(); ++j)
			//	str += ' ' + std::to_string(itemTrajectory->waypoints[i].command.cpos[j]);
			//// print in easy-to-export format
			//context.write("%f%s\n", trajectory[i].t, str.c_str());
		}
		// debug
		context.debug("data::HandlerR2GTrajectory::import(): Trajectory size %u -> %u\n", U32(size), U32(itemTrajectory->waypoints.size()));

		//// import points
		//const StringSeq objectTypes = { grasp::Import::FILE_EXT_CLOUD_PLY };
		//std::string objectPath = grasp::getDir(path);
		//menu->readPath("Enter object file path: ", objectPath, objectTypes);
		//Cloud::Import().pointCloudPly(context, objectPath, itemTrajectory->objectPoints);
	}

	itemTrajectory->waypointFile.setModified(true);
	itemTrajectory->robotHandPoseFile.setModified(true);
	itemTrajectory->objectPoseFile.setModified(true);
	itemTrajectory->objectPointFile.setModified(true);
	itemTrajectory->jointsPoseFile.setModified(true);

	return item;
}

//------------------------------------------------------------------------------

Item::Ptr spam::data::HandlerR2GTrajectory::convert(ItemR2GTrajectory& item, const Handler& handler) {
	// only one type of output
	Item::Ptr pItem(handler.create());

	return pItem;
}

bool spam::data::HandlerR2GTrajectory::isConvertSupported(const Handler& handler) const {
	return handler.isItem<const grasp::data::ItemImage>();
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::exportt(const ItemR2GTrajectory& item, const std::string& path) const {
	// check extension
	if (std::find(exportTypes.begin(), exportTypes.end(), getExt(path)) == exportTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::export(): unknown file type %s", getExt(path).c_str());

	// compute size
	const size_t size = std::min(item.waypoints.size(), item.robotHandPoses.size());
	if (size <= 0)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::export(): no data available");

	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, const_cast<UICallback&>(*getUICallback())) : nullptr);

	// common parameters
	const char separator = '\t';

	// contact model
	grasp::Model::Desc modelDesc;
	modelDesc.contact3DType = Contact3D::TYPE_POINT; // simplest possible

	modelDesc.contact3DDesc.distance = Real(0.01); // receptive field extension from the link surface
	if (menu != nullptr) do menu->readNumber("Receptive field extension: ", modelDesc.contact3DDesc.distance); while (modelDesc.contact3DDesc.distance < REAL_EPS);

	modelDesc.contact3DDesc.lambda = Real(100.0); // meta-parameter which controls "likelihood" of a particular contact at given point ~ exp(-lambda*distance)
	Real rfStdDev = modelDesc.contact3DDesc.lambda * modelDesc.contact3DDesc.distance;
	if (menu != nullptr) menu->readNumber("Receptive field std_dev: ", rfStdDev);
	modelDesc.contact3DDesc.lambda = rfStdDev / modelDesc.contact3DDesc.distance;

	modelDesc.contact3DDesc.minNum = 1; // min num of points in the receptive field
	grasp::Model::Ptr model = modelDesc.create(context, "HandlerR2GTrajectory::export()");

	// contact helper function
	Contact3D::Seq contacts;
	Vec3 location;
	Real weight, distance;
	//auto contact = [&](const data::Point3D& points, const Mat34& frame, const Bounds::Seq& bounds, std::ostream& str) {
	//	contacts.clear();
	//	(void)model->create(points, frame, bounds, contacts);
	//	location.setZero();
	//	distance = REAL_ZERO;
	//	weight = REAL_ZERO;
	//	for (auto &i : contacts) {
	//		location += i.frame.p;// *i.weight; // weighted "point mass" location
	//		distance += std::max(REAL_ZERO, i.distance);// *i.weight; // weighted distance (i.distance < 0 if inside robot's body)
	//		weight += i.weight;
	//	}
	//	if (!contacts.empty()) {
	//		location /= contacts.size();
	//		distance /= contacts.size();
	//	}
	//	str << location.x << separator << location.y << separator << location.z << separator;
	//	str << distance << separator << weight << separator;
	//};

	//// point cloud interface
	//struct Points3D : public data::Point3D {
	//	Points3D(const grasp::Cloud::PointSeq& objectPoints) : objectPoints(objectPoints) {}
	//	virtual size_t getNumOfPoints() const { return objectPoints.size(); }
	//	virtual size_t samplePoint(golem::Rand& rand) const { return size_t(rand.next()) % objectPoints.size(); }
	//	virtual Point getPoint(size_t index) const { return Point(Cloud::getPoint<Point3D::Real>(objectPoints[index])); }
	//	virtual void transform(const golem::Mat34& trn) {} // not supported
	//	const grasp::Cloud::PointSeq& objectPoints;
	//};

	// reference frame of the entire sequence with respect to the last waypoint by default
	size_t refWaypoint = size;
	if (menu != nullptr) do menu->readNumber(makeString("Reference waypoint index [1..%u]: ", size).c_str(), refWaypoint); while (refWaypoint < 1 || refWaypoint > size);
	refWaypoint -= 1;

	// hand reference frame, compute from robot arm configuration
	const Mat34 baseFrameRef = manipulator->getBaseFrame(item.waypoints[refWaypoint].state.cpos);
	Mat34 baseFrameInvRef;
	baseFrameInvRef.setInverse(baseFrameRef);

	// object reference frame
	//const Mat34 objectFrameRef = item.objectPoses[refWaypoint];
	//Mat34 objectFrameInvRef;
	//objectFrameInvRef.setInverse(objectFrameRef);

	// open file
	std::ofstream ofs(path, std::ios::trunc); // overwrite
	if (!ofs)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::export(): unable to create %s", path.c_str());

	// header
	for (golem::Configspace::Index j = hand->getStateInfo().getJoints().begin(); j != hand->getStateInfo().getJoints().end(); ++j)
		ofs << "hand_j" << (j - hand->getStateInfo().getJoints().begin()) << separator;

	ofs << "hand_s0" << separator;

	ofs << "hand_px" << separator << "hand_py" << separator << "hand_pz" << separator;
	ofs << "hand_qx" << separator << "hand_qy" << separator << "hand_qz" << separator << "hand_qw" << separator;

	ofs << "object_px" << separator << "object_py" << separator << "object_pz" << separator;
	ofs << "object_qx" << separator << "object_qy" << separator << "object_qz" << separator << "object_qw" << separator;

	auto contactHeader = [=](const std::string& name, std::ostream& str) {
		str << name << "_lx" << separator << name << "_ly" << separator << name << "_lz" << separator;
		str << name << "_dist" << separator << name << "_wgh" << separator;
	};
	contactHeader("hand", ofs);
	for (golem::Configspace::Index j = hand->getStateInfo().getJoints().begin(); j != hand->getStateInfo().getJoints().end(); ++j)
		contactHeader(std::string("hand_j") + std::to_string(j - hand->getStateInfo().getJoints().begin()), ofs);

	ofs << std::endl;

	// iterate through all waypoints
	Quat qBaseFrameLocal;
	Quat qObjectFrameLocal;
	bool begin = true;
	for (size_t i = 0; i < size; ++i) {
		context.debug("HandlerR2GTrajectory::export(): waypoint #%u/%u...\n", U32(i + 1), U32(size));

		ofs << std::setprecision(7);

		// configuration (state)
		for (golem::Configspace::Index j = hand->getStateInfo().getJoints().begin(); j != hand->getStateInfo().getJoints().end(); ++j)
			ofs << item.waypoints[i].state.cpos[j] << separator;

		// synergy (mapped onto 1-st hand joint of command)
		ofs << item.waypoints[i].command.cpos[hand->getStateInfo().getJoints().begin()] << separator;

		// hand pose from forward kinematics
		const Mat34 baseFrame = manipulator->getBaseFrame(item.waypoints[i].state.cpos);
		const Mat34 baseFrameLocal = baseFrame * baseFrameInvRef;
		RBCoord rbBaseFrameLocal(baseFrameLocal);
		if (!begin)
			rbBaseFrameLocal.q = RBCoord::getQuatMin(qBaseFrameLocal, rbBaseFrameLocal.q); // account for quaternion duality - minimise angular distance in a sequence
		qBaseFrameLocal = rbBaseFrameLocal.q;

		ofs << rbBaseFrameLocal.p.x << separator << rbBaseFrameLocal.p.y << separator << rbBaseFrameLocal.p.z << separator;
		ofs << rbBaseFrameLocal.q.x << separator << rbBaseFrameLocal.q.y << separator << rbBaseFrameLocal.q.z << separator << rbBaseFrameLocal.q.w << separator;

		// object pose
		//const Mat34 objectFrameLocal = item.objectPoses[i] * objectFrameInvRef;
		//RBCoord rbObjectFrameLocal(objectFrameLocal);
		//if (!begin)
		//	rbObjectFrameLocal.q = RBCoord::getQuatMin(qObjectFrameLocal, rbObjectFrameLocal.q); // account for quaternion duality - minimise angular distance in a sequence
		//qObjectFrameLocal = rbObjectFrameLocal.q;

		//ofs << rbObjectFrameLocal.p.x << separator << rbObjectFrameLocal.p.y << separator << rbObjectFrameLocal.p.z << separator;
		//ofs << rbObjectFrameLocal.q.x << separator << rbObjectFrameLocal.q.y << separator << rbObjectFrameLocal.q.z << separator << rbObjectFrameLocal.q.w << separator;

		// points
		//grasp::Cloud::PointSeq objectPoints;
		//Cloud::transform(item.objectPoses[i], item.objectPoints, objectPoints);

		//// base
		//Bounds::Seq baseBounds;
		//manipulator->getBaseBounds(baseFrame, baseBounds);
		//contact(Points3D(objectPoints), baseFrame, baseBounds, ofs);

		// joints
		golem::WorkspaceJointCoord joints;
		manipulator->getJointFrames(item.waypoints[i].state.cpos, baseFrame, joints);
		for (golem::Configspace::Index j = hand->getStateInfo().getJoints().begin(); j != hand->getStateInfo().getJoints().end(); ++j) {
			Bounds::Seq jointBounds;
			manipulator->getJointBounds(j, joints[j], jointBounds);
//			contact(Points3D(objectPoints), joints[j], jointBounds, ofs);
		}

		ofs << std::endl;
		begin = false;
	}
}

const StringSeq& spam::data::HandlerR2GTrajectory::getExportFileTypes() const {
	return exportTypes;
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::createRender(const ItemR2GTrajectory& item) {
	HandlerTrajectory::createRender(item);

	renderer.reset();

	// robot hand
	if (!item.robotHandPoses.empty()) {
		RBCoord coord;
		coord.interpolate(RBCoord(item.robotHandPoses[item.pathWaypoint]), RBCoord(item.robotHandPoses[std::min(item.pathWaypoint + 1, item.robotHandPoses.size() - 1)]), item.pathInterpol);
		renderer.addAxes3D(coord.toMat34(), appearanceHandFrameSize);
	}

	//// object
	//if (!item.objectPoses.empty()) {
	//	RBCoord coord;
	//	coord.interpolate(RBCoord(item.objectPoses[item.pathWaypoint]), RBCoord(item.objectPoses[std::min(item.pathWaypoint + 1, item.objectPoses.size() - 1)]), item.pathInterpol);
	//	const Mat34 trn(coord.toMat34());
	//	renderer.addAxes3D(trn, appearanceHandFrameSize);
	//	//context.verbose("<transform v1=\"%.8f\" v2=\"%.8f\" v3=\"%.8f\" q0=\"%.8f\" q1=\"%.8f\" q2=\"%.8f\" q3=\"%.8f\"/>\n", coord.p.v1, coord.p.v2, coord.p.v3, coord.q.q0, coord.q.q1, coord.q.q2, coord.q.q3);
	//	appearancePoints.drawPoints(item.objectPoints, renderer, &trn);
	//}

	// joints
	for (Mat34MapSeq::const_iterator i = item.jointsPoses.begin(); i != item.jointsPoses.end(); ++i) {
		RBCoord coord;
		coord.interpolate(RBCoord(i->second[item.pathWaypoint]), RBCoord(i->second[std::min(item.pathWaypoint + 1, i->second.size() - 1)]), item.pathInterpol);
		renderer.addAxes3D(coord.toMat34(), appearanceJointsFrameSize);
	}
}

void spam::data::HandlerR2GTrajectory::render() const {
	HandlerTrajectory::render();

	if (hasRenderBlock()) return;

	renderer.render();
}

void spam::data::HandlerR2GTrajectory::customRender() const {
	HandlerTrajectory::customRender();
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::mouseHandler(int button, int state, int x, int y) {
	HandlerTrajectory::mouseHandler(button, state, x, y);

	if (hasRenderBlock()) return;

}

void spam::data::HandlerR2GTrajectory::motionHandler(int x, int y) {
	HandlerTrajectory::motionHandler(x, y);

}

void spam::data::HandlerR2GTrajectory::keyboardHandler(int key, int x, int y) {
	HandlerTrajectory::keyboardHandler(key, x, y);

	if (hasRenderBlock()) return;
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(spam::data::Mat34MapSeq::value_type& value) const {
	read(const_cast<std::string&>(value.first));
	read(value.second, value.second.begin());
}

template <> void golem::Stream::write(const spam::data::Mat34MapSeq::value_type& value) {
	write(value.first);
	write(value.second.begin(), value.second.end());
}

//------------------------------------------------------------------------------