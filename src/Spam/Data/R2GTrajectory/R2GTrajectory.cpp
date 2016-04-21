/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Spam/Data/R2GTrajectory/R2GTrajectory.h>
#include <Grasp/App/Player/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Planner/GraphPlanner/Data.h>
#include <Golem/UI/Data.h>
#include <Grasp/Core/Import.h>
#include <Grasp/Core/Ctrl.h>
#include <Grasp/Contact/Manipulator.h>
#include <boost/algorithm/string.hpp>

//-----------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;
using namespace spam::data;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* graspDescLoader() {
	// Create description
	return new HandlerR2GTrajectory::Desc();
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::ImportState::load(const golem::XMLContext* xmlcontext) {
	spam::XMLData("type", type, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("inp", inpStr, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("out", outStr, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("offset", offset, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("scale", scale, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void spam::data::HandlerR2GTrajectory::ImportState::update(const grasp::RealSeq& data, golem::Controller::State& state) const {
	// ignore if input pointer is out of range
	if (data.size() <= inp)
		return;
	// transform input
	const Real val = offset + scale*data[inp];
	// update
	if (type == TYPE_TIME)
		state.t = val; // assign
	else if (type == TYPE_POSITION)
		state.cpos.data()[out] += val; // add
	else if (type == TYPE_VELOCITY)
		state.cvel.data()[out] += val; // add
	else if (type == TYPE_ACCELERATION)
		state.cacc.data()[out] += val; // add
	else
		state.get<Real>(out) += val; // add
}

void spam::data::HandlerR2GTrajectory::ImportState::update(const Map& map, const grasp::RealSeq& data, golem::Controller::State& state) {
	for (HandlerR2GTrajectory::ImportState::Map::const_iterator i = map.begin(); i != map.end(); ++i)
		i->update(data, state);
}

void spam::data::HandlerR2GTrajectory::ImportState::extract(const std::string& str, grasp::U32Seq& seq) {
	// pointers
	std::stringstream spointers(str + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream ipointers(spointers, " -,");
	U32Seq pointers;
	try {
		while (!ipointers.eos())
			pointers.push_back(ipointers.nextInt<U32>());
	}
	catch (...) {
		// no numbers
		seq.push_back(golem::U32(-1));
		return;
	}
	// operations
	std::stringstream soperations(str + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
	IStream ioperations(soperations, " 0123456789");
	BoolSeq operations;
	try {
		while (!ioperations.eos())
			operations.push_back(std::strcmp(ioperations.next<const char*>(), "-") == 0); // true for a range of indices, false for a single index
	}
	catch (...) {
		// no operations
	}
	// test length for validity
	if (operations.size() + 1 != pointers.size())
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory::ImportState::extract(): invalid pointer range formatting");
	// extract
	if (operations.empty()) {
		seq.push_back(pointers.front());
		return;
	}
	U32Seq::const_iterator j = pointers.begin(), k = ++pointers.begin();
	for (BoolSeq::const_iterator i = operations.begin(); i != operations.end(); ++i, ++j, ++k) {
		// range of indices
		if (*i) {
			for (U32 index = *j; index <= *k; ++index)
				seq.push_back(index);
			continue;
		}
		// a single index
		// at the beginning or if there was no range before
		if (i == operations.begin() || !*(i - 1))
			seq.push_back(*j);
		// at the end
		if (i + 1 == operations.end())
			seq.push_back(*k);
	}
}

void spam::data::HandlerR2GTrajectory::ImportState::extract(Map& map) const {
	// input pointer
	U32Seq inpPtr;
	extract(inpStr, inpPtr);
	// output pointer
	U32Seq outPtr;
	extract(outStr, outPtr);
	// create all pairs
	map.reserve(map.size() + inpPtr.size()*outPtr.size());
	for (U32Seq::const_iterator ip = inpPtr.begin(); ip != inpPtr.end(); ++ip)
	for (U32Seq::const_iterator op = outPtr.begin(); op != outPtr.end(); ++op)
		map.push_back(ImportState(type, *ip < golem::U32(-1) ? *ip : *op, *op < golem::U32(-1) ? *op : *ip, offset, scale));
}

void spam::data::HandlerR2GTrajectory::ImportState::extract(const Map& inp, Map& out) {
	Map buf;
	for (Map::const_iterator i = inp.begin(); i != inp.end(); ++i) {
		buf.clear();
		i->extract(buf);
		out.insert(out.end(), buf.begin(), buf.end());
	}
	//for (auto &i : out)
	//	printf("%i: %i->%i, off=%f, s=%f\n", (int)i.type, i.inp, i.out, i.offset, i.scale);
}

void spam::data::HandlerR2GTrajectory::ImportFrame::load(const golem::XMLContext* xmlcontext) {
	spam::XMLData("type", type, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("lin", lin, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("ang", ang, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void spam::data::HandlerR2GTrajectory::ImportFrame::update(const grasp::RealSeq& data, golem::Mat34& trn) const {
	if (size_t(lin + 3) >= data.size())
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory::ImportFrame::update(): linear pointer %lu not in range <0, %lu)", lin + 3, data.size());
	const size_t size = type == TYPE_QUAT ? 4 : 3;
	if (size_t(ang + size) >= data.size())
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory::ImportFrame::update(): angular pointer %lu not in range <0, %lu)", ang + size, data.size());
	// linear component
	trn.p.setColumn3(data.data() + lin);
	// angular component
	if (type == TYPE_QUAT) {
		Quat quat;
		quat.x = data[ang + 0];
		quat.y = data[ang + 1];
		quat.z = data[ang + 2];
		quat.w = data[ang + 3]; // w last
		//quat.w = data[ang + 0]; // w first
		//quat.x = data[ang + 1];
		//quat.y = data[ang + 2];
		//quat.z = data[ang + 3];
		trn.R.fromQuat(quat);
	}
	else if (type == TYPE_EULER)
		trn.R.fromEuler(data[ang + 0], data[ang + 1], data[ang + 2]);
	else {
		Vec3 axis(data[ang + 0], data[ang + 1], data[ang + 2]);
		const Real angle = axis.magnitude();
		if (angle < REAL_EPS)
			trn.R.setId();
		else {
			axis.normalise();
			trn.R.fromAngleAxis(angle, axis);
		}
	}
}

//------------------------------------------------------------------------------

spam::data::ItemR2GTrajectory::ItemR2GTrajectory(HandlerR2GTrajectory& handler) : grasp::data::Item(handler), handler(handler), waypointFile(handler.file), pathPosition(golem::REAL_ZERO), pathWaypoint(0), pathInterpol(golem::REAL_ZERO), contactPathWaypoint(-1), contactPathInterpol(golem::REAL_ONE) {
	pregraspIdx = 0;
}

grasp::data::Item::Ptr spam::data::ItemR2GTrajectory::clone() const {
	throw Message(Message::LEVEL_ERROR, "ItemR2GTrajectory::clone(): not implemented");
}

void spam::data::ItemR2GTrajectory::createRender() {
	handler.createRender(*this);
}

void spam::data::ItemR2GTrajectory::load(const std::string& prefix, const golem::XMLContext* xmlcontext) {
	// waypoint
	std::string waypointSuffix;
	golem::XMLData("waypoint", waypointSuffix, const_cast<golem::XMLContext*>(xmlcontext), false);
	if (waypointSuffix.length() > 0) {
		waypointFile.load(prefix + waypointSuffix, [&](const std::string& path) {
			if (!handler.controller)
				throw Message(Message::LEVEL_ERROR, "ItemR2GTrajectory::load(): controller is not set");

			waypoints.clear();
			try {
				FileReadStream(path.c_str()).read(waypoints, waypoints.end(), grasp::Waypoint::create(*handler.controller));
			}
			catch (const golem::Message& msg) {
				// try legacy format
				golem::Controller::State::Seq trajectory;
				try {
					FileReadStream(path.c_str()).read(trajectory, trajectory.end(), handler.controller->createState());
				}
				catch (...) {
					throw msg;
				}
				// commands = states
				waypoints = grasp::Waypoint::make(trajectory, trajectory);
			}
		});
	}

	// path settings
	golem::XMLData("path_position", pathPosition, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("path_waypoint", contactPathWaypoint, xmlcontext->getContextFirst("contact", false), false);
		golem::XMLData("path_interpol", contactPathInterpol, xmlcontext->getContextFirst("contact", false), false);
	}
	catch (const golem::MsgXMLParser&) {
	}
}

void spam::data::ItemR2GTrajectory::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	// waypoint xml
	std::string waypointSuffix = !waypoints.empty() ? handler.waypointSuffix : "";
	golem::XMLData("waypoint", waypointSuffix, xmlcontext, true);

	// waypoint binary
	if (waypointSuffix.length() > 0) {
		waypointFile.save(prefix + waypointSuffix, [=](const std::string& path) {
			FileWriteStream(path.c_str()).write(waypoints.begin(), waypoints.end());
		});
	}
	else
		waypointFile.remove();

	// path settings
	golem::XMLData("path_position", const_cast<Real&>(pathPosition), xmlcontext, true);
	golem::XMLData("path_waypoint", const_cast<size_t&>(contactPathWaypoint), xmlcontext->getContextFirst("contact", true), true);
	golem::XMLData("path_interpol", const_cast<Real&>(contactPathInterpol), xmlcontext->getContextFirst("contact", true), true);
}

void spam::data::ItemR2GTrajectory::setWaypoints(const grasp::Waypoint::Seq& waypoints) {
	this->waypoints = waypoints;
	waypointFile.setModified(true);
}

static const char* TypeName[] = {
	"None",
	"Approach",
	"Action",
};

void spam::data::ItemR2GTrajectory::setWaypoints(const grasp::Waypoint::Seq& waypoints, const R2GTrajectory::Type type) {
	const WaypointMap::value_type val(type, waypoints);
	waypointMap.insert(val);
}

const grasp::Waypoint::Seq& spam::data::ItemR2GTrajectory::getWaypoints() const {
	return waypoints;
}

const grasp::Waypoint::Seq& spam::data::ItemR2GTrajectory::getWaypoints(const R2GTrajectory::Type type) const {
	WaypointMap::const_iterator i = waypointMap.find(type);
	if (i == waypointMap.end())
		throw Message(Message::LEVEL_ERROR, "Error: no waypoint of type %s", TypeName[type]);
	return i->second;
}

void spam::data::ItemR2GTrajectory::createTrajectory(golem::Controller::State::Seq& trajectory) {
	handler.convert(*this, handler);
	handler.createTrajectory(*this, trajectory);
}

void spam::data::ItemR2GTrajectory::createAction(golem::Controller::State::Seq& trajectory) {
	handler.createTrajectory(*this, trajectory);
}

//------------------------------------------------------------------------------

grasp::data::Item::Ptr spam::data::ItemR2GTrajectory::convert(const grasp::data::Handler& handler) {
	return this->handler.convert(*this, handler);
}

const grasp::StringSeq& spam::data::ItemR2GTrajectory::getConvertInterfaces() const {
	return this->handler.convertInterfaces;
}

bool spam::data::ItemR2GTrajectory::isConvertSupported(const grasp::data::Handler& handler) const {
	return this->handler.isConvertSupported(handler);
}


//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::FactorDesc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::XMLData("arm", arm, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("hand", hand, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("other", other, const_cast<golem::XMLContext*>(xmlcontext), false);
}

void spam::data::HandlerR2GTrajectory::ImportRobotTrjDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("interval", interval, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("begin", begin, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("end", end, const_cast<golem::XMLContext*>(xmlcontext), false);
	golem::XMLData("subsampling", subsampling, const_cast<golem::XMLContext*>(xmlcontext), false);
	stateMap.clear();
	golem::XMLData(stateMap, stateMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "state");
	commandMap.clear();
	golem::XMLData(commandMap, commandMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "command");
}

void spam::data::HandlerR2GTrajectory::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	grasp::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("trajectory", false);

	golem::XMLData("waypoint_suffix", waypointSuffix, pxmlcontext, false);

	golem::XMLData(*profileDesc, pxmlcontext->getContextFirst("profile"), false);

	golem::XMLDataSeq(distance, "c", pxmlcontext->getContextFirst("profile distance"), false, golem::REAL_ZERO);
	golem::XMLDataSeq(extrapolation, "c", pxmlcontext->getContextFirst("profile extrapolation"), false, golem::REAL_ZERO);
	golem::XMLDataSeq(command, "c", pxmlcontext->getContextFirst("profile command"), false, golem::REAL_ZERO);

	velFac.load(context, pxmlcontext->getContextFirst("profile velocity"));
	accFac.load(context, pxmlcontext->getContextFirst("profile acceleration"));
	disFac.load(context, pxmlcontext->getContextFirst("profile distance"));
	extFac.load(context, pxmlcontext->getContextFirst("profile extrapolation"));
	cmdFac.load(context, pxmlcontext->getContextFirst("profile command"));

	golem::XMLData("extrapolation", trjExtrapolation, pxmlcontext->getContextFirst("profile"), false);
	golem::XMLData("duration", trjDuration, pxmlcontext->getContextFirst("profile"), false);
	golem::XMLData("idle", trjIdle, pxmlcontext->getContextFirst("profile"), false);

	action.clear();
	try {
		golem::XMLData(action, action.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "action");
	}
	catch (const golem::MsgXMLParser&) {}

	golem::XMLData(boundsSolidColour, pxmlcontext->getContextFirst("appearance bounds solid_colour"), false);
	golem::XMLData(pathRenderer, pxmlcontext->getContextFirst("appearance path"), false);
	golem::XMLData("inc_large", pathIncLarge, pxmlcontext->getContextFirst("appearance path"), false);
	golem::XMLData("inc_small", pathIncSmall, pxmlcontext->getContextFirst("appearance path"), false);

	importRobotTrj.load(pxmlcontext->getContextFirst("import robot_trj"));
	golem::XMLData("file_ext", importHDF5FileExt, pxmlcontext->getContextFirst("import hdf5", false), false);
	golem::XMLData("robot_trj", importHDF5RobotTrj, pxmlcontext->getContextFirst("import hdf5", false), false);
}

grasp::data::Handler::Ptr spam::data::HandlerR2GTrajectory::Desc::create(golem::Context &context) const {
	grasp::data::Handler::Ptr handler(new HandlerR2GTrajectory(context));
	to<HandlerR2GTrajectory>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

spam::data::HandlerR2GTrajectory::HandlerR2GTrajectory(golem::Context &context) : 
	Handler(context), planner(nullptr), controller(nullptr), 
	arm(nullptr), hand(nullptr), pathPositionInc(golem::REAL_ZERO), 
	boundsShow(false), contactPathRequest(false) {
}

void spam::data::HandlerR2GTrajectory::create(const Desc& desc) {
	grasp::data::Handler::create(desc);

	waypointSuffix = desc.waypointSuffix;

	plannerIndex = desc.plannerIndex;

	profileDesc = desc.profileDesc; // shallow copy
	profileDesc->pCallbackDist = this;

	desc.profileDesc->velocity.resize(Configspace::DIM, golem::REAL_ONE);
	velocityFac.set(desc.profileDesc->velocity.begin(), desc.profileDesc->velocity.end());

	desc.profileDesc->acceleration.resize(Configspace::DIM, golem::REAL_ONE);
	accelerationFac.set(desc.profileDesc->acceleration.begin(), desc.profileDesc->acceleration.end());

	distance.fill(golem::REAL_ONE);
	distance.set(desc.distance.data(), desc.distance.data() + std::min(desc.distance.size(), (size_t)Configspace::DIM));
	distanceFac = distance; // backup

	extrapolation.fill(golem::REAL_ZERO);
	extrapolation.set(desc.extrapolation.data(), desc.extrapolation.data() + std::min(desc.extrapolation.size(), (size_t)Configspace::DIM));
	extrapolationFac = extrapolation; // backup

	command.fill(golem::REAL_ZERO);
	command.set(desc.command.data(), desc.command.data() + std::min(desc.command.size(), (size_t)Configspace::DIM));
	commandFac = command; // backup

	// multipliers
	velFac = desc.velFac;
	accFac = desc.accFac;
	disFac = desc.disFac;
	extFac = desc.extFac;
	cmdFac = desc.cmdFac;

	trjExtrapolation = desc.trjExtrapolation;
	trjDuration = desc.trjDuration;
	trjIdle = desc.trjIdle;

	action = desc.action;

	boundsSolidColour = desc.boundsSolidColour;
	pathRenderer = desc.pathRenderer;
	pathIncLarge = desc.pathIncLarge;
	pathIncSmall = desc.pathIncSmall;

	showCommands = desc.showCommands;

	importRobotTrj = desc.importRobotTrj;
	importRobotTrj.stateMap.clear();
	importRobotTrj.commandMap.clear();
	ImportState::extract(desc.importRobotTrj.stateMap, importRobotTrj.stateMap);
	ImportState::extract(desc.importRobotTrj.commandMap, importRobotTrj.commandMap);
	importRobotTrj.assertValid(Assert::Context("HandlerR2GTrajectory::create(): importRobotTrj."));

	importHDF5FileExt = desc.importHDF5FileExt;
	importHDF5RobotTrj = desc.importHDF5RobotTrj;

	importTypes = {
		importHDF5FileExt,
	};

	// convert from
	convertInterfaces = {
		"Trajectory",
	};

	handPregraspPose = desc.handPregraspPose;
	handGraspPose = desc.handGraspPose;
}

//------------------------------------------------------------------------------

std::string spam::data::HandlerR2GTrajectory::getFileExtWaypoint() {
	return std::string(".wp");
}

grasp::data::Item::Ptr spam::data::HandlerR2GTrajectory::create() const {
	return grasp::data::Item::Ptr(new ItemR2GTrajectory(*const_cast<HandlerR2GTrajectory*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

golem::U32 spam::data::HandlerR2GTrajectory::getPlannerIndex() const {
	return plannerIndex;
}

void spam::data::HandlerR2GTrajectory::set(const golem::Planner& planner, const grasp::ControllerId::Seq& controllerIDSeq) {
	this->planner = &planner;
	this->controller = &planner.getController();

	if (controllerIDSeq.size() < 2)
		throw Message(Message::LEVEL_CRIT, "HandlerR2GTrajectory(): arm and hand are required");

	// joint and chain info
	info = controller->getStateInfo();
	armInfo = controllerIDSeq[0].findInfo(*const_cast<golem::Controller*>(controller));
	handInfo = controllerIDSeq[1].findInfo(*const_cast<golem::Controller*>(controller));

	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i) {
		const bool isArm = armInfo.getJoints().contains(i);
		const bool isHand = handInfo.getJoints().contains(i);
		const size_t j = i - info.getJoints().begin();

		// velocity
		profileDesc->velocity[j] = velocityFac[i] * (isArm ? velFac.arm : isHand ? velFac.hand : velFac.other);
		// accleleration
		profileDesc->acceleration[j] = accelerationFac[i] * (isArm ? accFac.arm : isHand ? accFac.hand : accFac.other);
		// distance
		distance[i] = distanceFac[i] * (isArm ? disFac.arm : isHand ? disFac.hand : disFac.other);
		// extrapolation
		extrapolation[i] = extrapolationFac[i] * (isArm ? extFac.arm : isHand ? extFac.hand : extFac.other);
		// command
		command[i] = commandFac[i] * (isArm ? cmdFac.arm : isHand ? cmdFac.hand : cmdFac.other);
	}

	pProfile = profileDesc->create(*controller); // throws

	boundsSeq.clear();
	// chain bounds
	for (Chainspace::Index i = info.getChains().begin(); i != info.getChains().end(); ++i) {
		const Chain* chain = controller->getChains()[i];
		chainBoundsSeq[i].clear();
		chainMat34Seq[i].clear();
		const Bounds::Desc::SeqPtr seq = chain->getBoundsDescSeq();
		for (Bounds::Desc::Seq::const_iterator j = seq->begin(); j != seq->end(); ++j)
		if (*j != nullptr) {
			chainBoundsSeq[i].push_back(j->get()->create());
			chainMat34Seq[i].push_back(chainBoundsSeq[i].back()->getPose());
			boundsSeq.push_back(chainBoundsSeq[i].back().get());
		}
	}
	// joint bounds
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
		const Joint* joint = controller->getJoints()[i];
		jointBoundsSeq[i].clear();
		jointMat34Seq[i].clear();
		const Bounds::Desc::SeqPtr seq = joint->getBoundsDescSeq();
		for (Bounds::Desc::Seq::const_iterator j = seq->begin(); j != seq->end(); ++j)
		if (*j != nullptr) {
			jointBoundsSeq[i].push_back(j->get()->create());
			jointMat34Seq[i].push_back(jointBoundsSeq[i].back()->getPose());
			boundsSeq.push_back(jointBoundsSeq[i].back().get());
		}
	}
}

void spam::data::HandlerR2GTrajectory::create(const golem::ConfigspaceCoord& delta, golem::Controller::State::Seq& trajectory) const {
	if (trajectory.size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::create(): At least two waypoints required");

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

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
//		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
	}

	// make trajectory
	golem::Controller::State::Seq states, commands;
	try {
		states = grasp::Waypoint::make(item.getWaypoints(R2GTrajectory::Type::APPROACH), false);
		commands = grasp::Waypoint::make(item.getWaypoints(R2GTrajectory::Type::APPROACH), true);
	}
	catch (const Message& msg) {
		context.write("%s\n", msg.what());
		return;
	}
	// extrapolation delta
	//golem::ConfigspaceCoord delta;
	//for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
	//	delta[i] = trjExtrapolation*extrapolation[i];

	// extrapolation
	//const bool extrapol = trjExtrapolation > REAL_EPS;
	//if (extrapol) {
	//	create(delta, states);
	//	create(delta, commands);
	//}

	// don't process the last waypoint (used for grasping)
	const size_t size = commands.size();
	
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

	// assign if command[i] > 0
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
	if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
		// replace with commands
		for (size_t j = 0; j < trajectory.size(); ++j) {
			trajectory[j].cpos[i] = commands[j].cpos[i];
			trajectory[j].cvel[i] = commands[j].cvel[i];
			trajectory[j].cacc[i] = commands[j].cacc[i];
		}
		//if (extrapol)
		//	trajectory.back().cpos[i] = command[i];
	}

	// debug
	context.debug("HandlerR2GTrajectory::createTrajectory(): Trajectory size %u -> %u\n", (U32)size, (U32)trajectory.size());

	// stationary waypoint in the end
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
}

void spam::data::HandlerR2GTrajectory::createAction(const ItemR2GTrajectory& item, golem::Controller::State::Seq& trajectory) {
	if (!planner || !controller)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): planner and controller are not set");
	if (item.getWaypoints().size() < 2)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): At least two waypoints required");

	Real trjExtrapolation = this->trjExtrapolation;
	Real trjDuration = this->trjDuration;
	bool inverse = false;

	if (getUICallback() && getUICallback()->hasInputEnabled()) {
		Menu menu(context, *getUICallback());
		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
		menu.readNumber("Trajectory duration: ", trjDuration);
		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
	}

	// make trajectory
	golem::Controller::State::Seq states = grasp::Waypoint::make(item.getWaypoints(R2GTrajectory::Type::ACTION), false), commands = grasp::Waypoint::make(item.getWaypoints(R2GTrajectory::Type::ACTION), true);
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

	// don't process the last waypoint (used for grasping)
	const size_t size = commands.size();
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
	context.debug("HandlerR2GTrajectory::createTrajectory(): Trajectory size %u -> %u\n", (U32)size, (U32)trajectory.size());

	// stationary waypoint in the end
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));

	// liftin up action
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
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): Unable to find action trajectory");
	// add
	trajectory.insert(trajectory.end(), ++cAction.begin(), cAction.end());
	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
}

//void spam::data::HandlerR2GTrajectory::createTrajectory(ItemR2GTrajectory& item, golem::Controller::State::Seq& trajectory) {
//	if (!planner || !controller)
//		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): planner and controller are not set");
//	if (item.getWaypoints().size() < 2)
//		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): At least two waypoints required");
//
//	Real trjExtrapolation = this->trjExtrapolation;
//	Real trjDuration = this->trjDuration;
//	bool inverse = false;
//	bool createAction = arm && !action.empty();
//
//	if (getUICallback() && getUICallback()->hasInputEnabled()) {
//		Menu menu(context, *getUICallback());
//		//inverse = menu.option("FI", "Use (F)orward/(I)nverse trajectory... ") == 'I';
//		menu.readNumber("Trajectory duration: ", trjDuration);
//		menu.readNumber("Trajectory extrapolation: ", trjExtrapolation);
//	}
//
//	// make trajectory
//	golem::Controller::State::Seq states = grasp::Waypoint::make(item.getWaypoints(R2GTrajectory::Type::APPROACH), false), commands = grasp::Waypoint::make(item.getWaypoints(R2GTrajectory::Type::APPROACH), true);
//	// extrapolation delta
//	golem::ConfigspaceCoord delta;
//	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
//		delta[i] = trjExtrapolation*extrapolation[i];
//
//	// extrapolation
//	const bool extrapol = trjExtrapolation > REAL_EPS;
//	if (extrapol) {
//		create(delta, states);
//		create(delta, commands);
//	}
//
//	// don't process the last waypoint (used for grasping)
//	const size_t size = commands.size();
//	// trajectory profiling
//
//	trajectory = commands;
//	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
//	if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
//		// replace with states
//		for (size_t j = 0; j < trajectory.size(); ++j) {
//			trajectory[j].cpos[i] = states[j].cpos[i];
//			trajectory[j].cvel[i] = states[j].cvel[i];
//			trajectory[j].cacc[i] = states[j].cacc[i];
//		}
//	}
//	profile(trjDuration, trajectory);
//
//	// assign if command[i] > 0
//	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i)
//	if (Math::abs(command[i]) > REAL_EPS) { // no impedance control
//		// replace with commands
//		for (size_t j = 0; j < trajectory.size(); ++j) {
//			trajectory[j].cpos[i] = commands[j].cpos[i];
//			trajectory[j].cvel[i] = commands[j].cvel[i];
//			trajectory[j].cacc[i] = commands[j].cacc[i];
//		}
//		if (extrapol)
//			trajectory.back().cpos[i] = command[i];
//	}
//
//	// debug
//	context.debug("HandlerR2GTrajectory::createTrajectory(): Trajectory size %u -> %u\n", (U32)size, (U32)trajectory.size());
//
//	// stationary waypoint in the end
//	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
//	item.pregraspIdx = size - 1;
//
//	// grasp
//
//
//	// liftin up action
//	GenWorkspaceChainState gwcc;
//	controller->chainForwardTransform(trajectory.back().cpos, gwcc.wpos);
//	for (Chainspace::Index i = info.getChains().begin(); i < info.getChains().end(); ++i)
//		gwcc.wpos[i].multiply(gwcc.wpos[i], controller->getChains()[i]->getReferencePose()); // reference pose
//	gwcc.t = trajectory.back().t;
//
//	const Controller::State::Info armInfo = arm->getStateInfo();
//	GenWorkspaceChainState::Seq wAction;
//	wAction.push_back(gwcc);
//	for (Mat34Seq::const_iterator i = action.begin(); i != action.end(); ++i) {
//		GenWorkspaceChainState gwccTrn = gwcc;
//		gwccTrn.wpos[armInfo.getChains().begin()].multiply(*i, gwcc.wpos[armInfo.getChains().begin()]);
//		gwccTrn.t += trjDuration;
//		wAction.push_back(gwccTrn);
//	}
//	// find trajectory
//	golem::Controller::State::Seq cAction;
//	if (!const_cast<Planner*>(planner)->findLocalTrajectory(trajectory.back(), wAction.begin(), wAction.end(), cAction, cAction.end()))
//		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::createTrajectory(): Unable to find action trajectory");
//	// add
//	trajectory.insert(trajectory.end(), ++cAction.begin(), cAction.end());
//	trajectory.push_back(Controller::State(trajectory.back(), trajectory.back().t + this->trjIdle));
//}

//------------------------------------------------------------------------------

//grasp::data::Item::Ptr spam::data::HandlerR2GTrajectory::transform(const grasp::data::Item::List& input) {
//	grasp::data::Item::Ptr item(create());
//	ItemR2GTrajectory* itemR2GTrajectory = to<ItemR2GTrajectory>(item.get());
//
//	// block item rendering (hasRenderBlock()=true)
//	RenderBlock renderBlock(*this);
//
//	const spam::data::R2GTrajectory* trajectory = nullptr;
//	grasp::Waypoint::Seq seq;
//
//	// collect data
//	for (grasp::data::Item::List::const_iterator i = input.begin(); i != input.end(); ++i) {
//		trajectory = is<const spam::data::R2GTrajectory>(*i);
//		if (!trajectory)
//			continue;
//		seq.insert(seq.end, trajectory->getWaypoints().begin(), trajectory->getWaypoints().end());
//	}
//	if (seq.empty())
//		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::transform(): the trajectory has no waypoint");
//	if (seq.size() < 3)
//		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::transform(): the trajectory has no enough waypoint");
//
//	// seq[0] = initial pose; seq[1] = pregrasp pose; seq[2] = grasp pose
//	grasp::Waypoint* seq0 = &seq[0];
//	grasp::Waypoint* seq1 = &seq[1];
//	grasp::Waypoint* seq2 = &seq[2];
//
//	for (auto j = handInfo.getJoints().begin(); j != handInfo.getJoints().end(); ++j) {
//		const size_t k = j - handInfo.getJoints().begin();
//		seq0->command.cpos[j] = handPregraspPose[k];
//		seq1->command.cpos[j] = handPregraspPose[k];
//		seq2->command.cpos[j] = handGraspPose[k];
//	}
//	seq0->state.cpos = seq0->command.cpos;
//	seq1->state.cpos = seq1->command.cpos;
//	seq2->state.cpos = seq2->command.cpos;
//
//	itemR2GTrajectory->setWaypoints(seq);
//
//	return item;
//}

//const grasp::StringSeq& spam::data::HandlerR2GTrajectory::getTransformInterfaces() const {
//	return transformInterfaces;
//}
//
//bool spam::data::HandlerR2GTrajectory::isTransformSupported(const grasp::data::Item& item) const {
//	return is<const spam::data::ItemR2GTrajectory>(&item);
//}

//------------------------------------------------------------------------------

grasp::data::Item::Ptr spam::data::HandlerR2GTrajectory::convert(ItemR2GTrajectory& item, const grasp::data::Handler& handler) {
	// only one type of output
	grasp::data::Item::Ptr pItem(handler.create());
	//R2GTrajectory* trajectory = is<R2GTrajectory>(pItem.get());
	//if (!trajectory)
	//	throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::convert(): Item %s does not support %s interface", handler.getType().c_str(), pItem.get()->getHandler().getID().c_str());

	grasp::Waypoint::Seq seq;

	seq.insert(seq.end(), item.getWaypoints().begin(), item.getWaypoints().end());

	if (seq.empty())
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::convert(): the trajectory has no waypoint");
	if (seq.size() < 3)
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::convert(): the trajectory has no enough waypoint");

	// seq[0] = initial pose; seq[1] = pregrasp pose; seq[2] = grasp pose
	grasp::Waypoint* seq0 = &seq[0];
	grasp::Waypoint* seq1 = &seq[1];
	grasp::Waypoint* seq2 = &seq[2];

	for (auto j = handInfo.getJoints().begin(); j != handInfo.getJoints().end(); ++j) {
		seq1->command.cpos[j] = seq0->command.cpos[j];
		//const size_t k = j - handInfo.getJoints().begin();
		//seq0->command.cpos[j] = handPregraspPose[k];
		//seq1->command.cpos[j] = handPregraspPose[k];
		//seq2->command.cpos[j] = handGraspPose[k];
	}
	seq0->state.cpos = seq0->command.cpos;
	seq1->state.cpos = seq1->command.cpos;
	seq2->state.cpos = seq2->command.cpos;

	grasp::Waypoint::Seq approach, action;
	approach.push_back(seq[0]);
	approach.push_back(seq[1]);
	approach.push_back(seq[1]);
	action.push_back(seq[1]);
	action.push_back(seq[2]);
	action.push_back(seq[2]);
	item.setWaypoints(approach, R2GTrajectory::Type::APPROACH);
	item.setWaypoints(action, R2GTrajectory::Type::ACTION);

	item.setWaypoints(seq);

	return pItem;
}

bool spam::data::HandlerR2GTrajectory::isConvertSupported(const grasp::data::Handler& handler) const {
	return handler.isItem<const spam::data::ItemR2GTrajectory>();
}

//------------------------------------------------------------------------------

Real spam::data::HandlerR2GTrajectory::distConfigspaceCoord(const ConfigspaceCoord& prev, const ConfigspaceCoord& next) const {
	Real dist = REAL_ZERO;
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i)
		dist += distance[i] * Math::sqr(prev[i] - next[i]);
	return Math::sqrt(dist);
}

Real spam::data::HandlerR2GTrajectory::distCoord(Real prev, Real next) const {
	return Math::abs(prev - next);
}

bool spam::data::HandlerR2GTrajectory::distCoordEnabled(const Configspace::Index& index) const {
	return (arm->getStateInfo().getJoints().contains(index) || hand->getStateInfo().getJoints().contains(index));
}

bool spam::data::HandlerR2GTrajectory::distCoordInterpolate(const Configspace::Index& index) const {
	return !arm->getStateInfo().getJoints().contains(index) && !hand->getStateInfo().getJoints().contains(index);
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::distRemoved(size_t index) const {
	if (distRemovedCallback) distRemovedCallback(index);
}

//------------------------------------------------------------------------------

grasp::data::Item::Ptr spam::data::HandlerR2GTrajectory::import(const std::string& path) {
	// check extension
	if (std::find(importTypes.begin(), importTypes.end(), getExt(path)) == importTypes.end())
		throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::import(): unknown file type %s", getExt(path).c_str());

	RenderBlock renderBlock(*this); // do not show cache

	grasp::data::Item::Ptr item(create());
	ItemR2GTrajectory* itemTrajectory = to<ItemR2GTrajectory>(item.get());
	Menu::Ptr menu(getUICallback() && getUICallback()->hasInputEnabled() ? new Menu(context, *getUICallback()) : nullptr);

	// HDF5 format
	if (path.rfind(importHDF5FileExt) != std::string::npos) {
		Controller::State state = controller->createState(), command = state;

		grasp::Import::HDF5DumpRealSeqMap map;
		if (menu != nullptr) {
			menu->readString("Enter robot trajectory dataset name: ", importHDF5RobotTrj);
		}

		map.insert(std::make_pair(importHDF5RobotTrj, [&](const std::string& name, size_t index, const grasp::RealSeq& seq) {
			// subsampling in range <begin, end)
			if (index >= importRobotTrj.begin && index < importRobotTrj.end && (index - importRobotTrj.begin) % importRobotTrj.subsampling == 0) {
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

		// read datasets from file
		grasp::Import().hdf5DumpRealSeq(path, map);
		// assert data validity
		if (itemTrajectory->waypoints.empty())
			throw Message(Message::LEVEL_ERROR, "HandlerR2GTrajectory::import(): empty robot trajectory");
	}

	itemTrajectory->waypointFile.setModified(true);

	return item;
}

const grasp::StringSeq& spam::data::HandlerR2GTrajectory::getImportFileTypes() const {
	return importTypes;
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::createRender(const ItemR2GTrajectory& item) {
	UI::CriticalSectionWrapper cs(getUICallback());

	boundsShow = false;
	pathRenderer.reset();
	if (!controller || item.waypoints.empty())
		return;

	// commands/states
	const Controller::State::Seq seq = grasp::Waypoint::make(item.waypoints, showCommands);

	// path
	if (pathRenderer.show)
		pathRenderer.fromConfigspace(*controller, seq.begin(), seq.end());

	// bounds interpolation
	ConfigspaceCoord cc;
	if (seq.size() > 1) {
		// compute distances
		RealSeq distance;
		distance.push_back(REAL_ZERO);
		for (Controller::State::Seq::const_iterator j = seq.begin(), i = j++; j != seq.end(); ++i, ++j) {
			Real d = REAL_ZERO;
			for (Configspace::Index k = info.getJoints().begin(); k != info.getJoints().end(); ++k)
				d += Math::sqr(i->cpos[k] - j->cpos[k]);
			distance.push_back(distance.back() + Math::sqrt(d));
		}
		// find neighbours
		const_cast<Real&>(item.pathPosition) = Math::clamp(item.pathPosition + pathPositionInc, REAL_ZERO, REAL_ONE);
		pathPositionInc = REAL_ZERO;
		const Real pos = item.pathPosition * distance.back();
		size_t i = 1;
		for (; i < distance.size() - 1 && pos > distance[i]; ++i);
		const_cast<size_t&>(item.pathWaypoint) = i;
		const Real d = distance[i] - distance[i - 1];
		const Real s = d > REAL_EPS ? (pos - distance[i - 1]) / d : REAL_ZERO;
		const_cast<Real&>(item.pathInterpol) = s;
		// interpolate between neighbours
		const ConfigspaceCoord prev = seq[i - 1].cpos, next = seq[i].cpos;
		for (Configspace::Index k = info.getJoints().begin(); k != info.getJoints().end(); ++k)
			cc[k] = (REAL_ONE - s)*prev[k] + (s)*next[k];
	}
	else {
		cc = seq.front().cpos;
		const_cast<Real&>(item.pathPosition) = REAL_ZERO;
		const_cast<size_t&>(item.pathWaypoint) = 0;
		const_cast<Real&>(item.pathInterpol) = REAL_ZERO;
	}

	if (contactPathRequest) {
		contactPathRequest = false;
		const_cast<size_t&>(item.contactPathWaypoint) = item.pathWaypoint;
		const_cast<Real&>(item.contactPathInterpol) = item.pathInterpol;
	}

	WorkspaceJointCoord wjc;
	controller->jointForwardTransform(cc, wjc);

	// chain bounds
	for (Chainspace::Index i = info.getChains().begin(); i != info.getChains().end(); ++i) {
		const Bounds::Seq& boundsSeq = chainBoundsSeq[i];
		const Mat34Seq& mat34Seq = chainMat34Seq[i];
		if (boundsSeq.empty())
			continue;
		const Chain* chain = controller->getChains()[i];
		const Chainspace::Index linkedChainIndex = chain->getLinkedChainIndex();
		Mat34 pose = linkedChainIndex < i ? wjc[info.getJoints(linkedChainIndex).end() - 1] : controller->getGlobalPose();
		pose.multiply(pose, chain->getLocalPose());
		for (size_t j = 0; j < boundsSeq.size(); ++j)
			boundsSeq[j]->multiplyPose(pose, mat34Seq[j]);
	}
	// joint bounds
	for (Configspace::Index i = info.getJoints().begin(); i != info.getJoints().end(); ++i) {
		const Bounds::Seq& boundsSeq = jointBoundsSeq[i];
		const Mat34Seq& mat34Seq = jointMat34Seq[i];
		const Mat34 pose = wjc[i];
		for (size_t j = 0; j < boundsSeq.size(); ++j)
			boundsSeq[j]->multiplyPose(pose, mat34Seq[j]);
	}

	boundsShow = true;
}

void spam::data::HandlerR2GTrajectory::render() const {
	if (hasRenderBlock()) return;

	// path
	pathRenderer.render();
	// bounds
	if (boundsShow) {
		boundsRenderer.setSolidColour(boundsSolidColour);
		boundsRenderer.renderSolid(boundsSeq.begin(), boundsSeq.end());
	}
}

void spam::data::HandlerR2GTrajectory::customRender() const {
}

//------------------------------------------------------------------------------

void spam::data::HandlerR2GTrajectory::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;

	if (state == 0) {
		pathPositionInc +=
			button == 3 ? +pathIncLarge : button == 19 ? +pathIncSmall :
			button == 4 ? -pathIncLarge : button == 20 ? -pathIncSmall :
			button == 1 ? -REAL_MAX : REAL_ZERO;
		requestRender();
	}
}

void spam::data::HandlerR2GTrajectory::motionHandler(int x, int y) {
}

void spam::data::HandlerR2GTrajectory::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;

	switch (key) {
	case '3':
		showCommands = !showCommands;
		context.write("Show %s\n", showCommands ? "commands" : "states");
		requestRender();
		break;
	case '4':
		pathRenderer.show = !pathRenderer.show;
		context.write("Show path %s\n", pathRenderer.show ? "ON" : "OFF");
		requestRender();
		break;
	case '5':
		context.write("Setting contact trajectory position\n");
		contactPathRequest = true;
		requestRender();
		break;
	};
}

//------------------------------------------------------------------------------

void spam::XMLData(const std::string &attr, data::HandlerR2GTrajectory::ImportState::Type& val, golem::XMLContext* xmlcontext, bool create) {
	std::string type;
	XMLData(attr, type, xmlcontext, create);
	transform(type.begin(), type.end(), type.begin(), tolower);
	val =
		type == "t" || type == "time" ? data::HandlerR2GTrajectory::ImportState::TYPE_TIME :
		type == "pos" || type == "position" ? data::HandlerR2GTrajectory::ImportState::TYPE_POSITION :
		type == "vel" || type == "velocity" ? data::HandlerR2GTrajectory::ImportState::TYPE_VELOCITY :
		type == "acc" || type == "acceleration" ? data::HandlerR2GTrajectory::ImportState::TYPE_VELOCITY :
		data::HandlerR2GTrajectory::ImportState::TYPE_RESERVED;
}

void spam::XMLData(const std::string &attr, data::HandlerR2GTrajectory::ImportFrame::Type& val, golem::XMLContext* xmlcontext, bool create) {
	std::string type;
	XMLData(attr, type, xmlcontext, create);
	transform(type.begin(), type.end(), type.begin(), tolower);
	val =
		type == "euler" ? data::HandlerR2GTrajectory::ImportFrame::TYPE_EULER :
		type == "axis" ? data::HandlerR2GTrajectory::ImportFrame::TYPE_AXIS :
		data::HandlerR2GTrajectory::ImportFrame::TYPE_QUAT;
}

void spam::data::XMLData(spam::data::HandlerR2GTrajectory::ImportState::Map::value_type& val, golem::XMLContext* xmlcontext, bool create) {
	val.load(xmlcontext);
}

//------------------------------------------------------------------------------