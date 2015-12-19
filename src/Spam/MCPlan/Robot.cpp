/** @file Robot.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Spinta/Planner/Robot.h>
#include <Golem/Device/MultiCtrl/MultiCtrl.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Plan/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/PhysCtrl/Data.h>

#ifdef _GRASP_HITHANDII_DEBUG
#include <Golem/Device/DLR/DLRHitHandII.h>
#endif // _GRASP_HITHANDII_DEBUG

using namespace golem;
using namespace spinta;

//------------------------------------------------------------------------------

Robot::Robot(Scene &scene) : Object(scene), groundPlane(NULL), physPlanner(NULL), planner(NULL), controller(NULL), arm(NULL), hand(NULL) {
}
	
bool Robot::create(const Desc& desc) {
	Object::create(desc); // throws

	// create ground plane
	Actor::Desc groundPlaneDesc;
	groundPlaneDesc.nxActorDesc.shapes.pushBack(scene.createNxShapeDesc(Bounds::Desc::Ptr(new BoundingPlane::Desc)));
	groundPlane = scene.createObject(groundPlaneDesc);

	physPlanner = dynamic_cast<PhysPlanner*>(scene.createObject(*desc.physPlannerDesc));
	if (physPlanner == NULL)
		throw Message(Message::LEVEL_CRIT, "Robot::create(): unable to cast to PhysPlanner");
	planner = &physPlanner->getPlanner();
	heuristic = &planner->getHeuristic();
	
	controller = &physPlanner->getController();
	info = controller->getStateInfo();

	MultiCtrl* multiCtrl = dynamic_cast<MultiCtrl*>(controller);
	SingleCtrl *singleCtrl;
	if (multiCtrl == NULL || multiCtrl->getControllers().size() != 2) {
		context.getMessageStream()->write(Message::LEVEL_DEBUG, "Robot::create(): No valid MultiCtrl.");
		singleCtrl =  dynamic_cast<SingleCtrl*>(controller);
		if (singleCtrl == NULL)
			throw Message(Message::LEVEL_CRIT, "Robot::create(): Robot requires MultiCtrl with two connected devices or a SingleCtrl");
	}
	isMultiCtrl = singleCtrl == NULL;

	arm = isMultiCtrl ? dynamic_cast<SingleCtrl*>(multiCtrl->getControllers()[0]) : singleCtrl;
	hand = isMultiCtrl ? dynamic_cast<SingleCtrl*>(multiCtrl->getControllers()[1]) : NULL;
	if (arm == NULL || (isMultiCtrl && hand == NULL))
		throw Message(Message::LEVEL_CRIT, "Robot::create(): Robot requires SingleCtrl");	

	armInfo = arm->getStateInfo();
	if (isMultiCtrl) handInfo = hand->getStateInfo();

	// add bounds to the tool
	armToolBounds = desc.armToolBounds;
	heuristic->addJointBounds(armToolBounds.begin(), armToolBounds.end(), armInfo.getJoints().end() - 1);
	heuristic->syncJointBounds();

	trajectoryDuration = desc.trajectoryDuration;
	trajectoryIdle = desc.trajectoryIdle;


	try {
		ftSensor = desc.ftSensorDesc->create();
		ftSensorMode = true;
	}
	catch (const std::exception& ex) {
		context.debug("%s\n", ex.what());
		context.warning("Robot::create(): F/T sensor disabled\n", ex.what());
		ftSensorMode = false;
	}
	ftFrame = desc.ftFrame;
	ftFrameInv.setInverse(desc.ftFrame);
	ftFrameSize = desc.ftFrameSize;
	ftGain = desc.ftGain;
	ftLimit = desc.ftLimit;

	// TODO check size of all chains
	if (std::min(desc.impedStiffMin.size(), desc.impedDampMin.size()) < (isMultiCtrl ? (size_t)handInfo.getJoints(handInfo.getChains().begin()).size() : 1))
		throw Message(Message::LEVEL_CRIT, "Robot::create(): Invalid number of impedance control parameters");
	impedStiffMin = desc.impedStiffMin;
	impedStiffMax = desc.impedStiffMax;
	impedStiffSteps = desc.impedStiffSteps;
	impedStiffStep = desc.impedStiffStepInit;
	impedStiff = impedStiffMin;
	impedDampMin = desc.impedDampMin;
	impedDampMax = desc.impedDampMax;
	impedDampSteps = desc.impedDampSteps;
	impedDampStep = desc.impedDampStepInit;
	impedDamp = impedDampMin;

	// update Hand impedance parameters
	level("stiffness", impedStiffMin, impedStiffMax, impedStiffSteps, impedStiffStep, impedStiff);
	level("damping", impedDampMin, impedDampMax, impedDampSteps, impedDampStep, impedDamp);

	simForceGain = desc.simForceGain;
	simArmForceMode = FORCE_MODE_DISABLED;
	simHandForceMode = FORCE_MODE_DISABLED;
	simForceModeOvr = false;
	simForceVec.setZero();

	forceDispScale = desc.forceDispScale;

	armForceController = desc.armForceController;
	if (isMultiCtrl) handForceController = desc.handForceController;

	offsetIdx = 0;

	desc.armCtrlDesc->forceReader = [=] (const golem::Controller::State&, grasp::RealSeq& force) {
		force.assign(6, REAL_ZERO);
		Twist* wrench = reinterpret_cast<Twist*>(force.data());

		if (ftSensorMode && armCtrl->getMode() != grasp::ActiveCtrl::MODE_EMERGENCY) {
			// try to read from F/T sensor
			ftSensor->read(force.data());
			// throw exception when F/T threshold limit is reached
			for (size_t i = 0; i < 3; ++i)
				if (Math::abs(wrench->v[i]) >= ftLimit.v[i] || Math::abs(wrench->w[i]) >= ftLimit.w[i])
					throw Message(Message::LEVEL_NOTICE, "forceReader(): F/T limit: Fx=%6.2lf, Fy=%6.2lf, Fz=%6.2lf, Tx=%6.2lf, Ty=%6.2lf, Tz=%6.2lf", wrench->v.x, wrench->v.y, wrench->v.z, wrench->w.x, wrench->w.y, wrench->w.z);
			// multiply by gain
			wrench->arrayMultiply(*wrench, ftGain);
		}

		if (simArmForceMode != FORCE_MODE_DISABLED) {
			if (simArmForceMode == FORCE_MODE_TORQUE)
				wrench->w.arrayMultiply(simForceVec, simForceGain);
			if (simArmForceMode == FORCE_MODE_FORCE)
				wrench->v.arrayMultiply(simForceVec, simForceGain);
		}
		
		{
			golem::CriticalSectionWrapper csw(csData);
			armForceRaw = force;
		}

		wrench->adjointTransposedTransform(*wrench, ftFrameInv);
	};
	desc.armCtrlDesc->actionFilter = [=] (const golem::Controller::State& prev, golem::Controller::State& next, bool bSendPrev, bool bSendNext) {
		armForceController(armForceRaw, prev, next, bSendPrev, bSendNext);
	};
	armCtrl = desc.armCtrlDesc->create(*arm); // create and install callback (throws)
	
	if (isMultiCtrl) {
		desc.handCtrlDesc->forceReader = [=] (const golem::Controller::State& state, grasp::RealSeq& force) {
			// no force by default
			force.assign(state.getInfo().getJoints().size(), REAL_ZERO);
		
			// read from state if available
			const ptrdiff_t forceOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_FORCE_TORQUE);
			if (forceOffset != Controller::ReservedOffset::UNAVAILABLE) {
				for (Configspace::Index j = state.getInfo().getJoints().begin(); j < state.getInfo().getJoints().end(); ++j) {
					const size_t k = j - state.getInfo().getJoints().begin();
					force[k] = state.get<ConfigspaceCoord>(forceOffset)[j];
				}
			}
			// overwrite 
			if (simHandForceMode != FORCE_MODE_DISABLED) {
				const Vec3 simForce(simForceVec.z*simForceGain.z, simForceVec.x*simForceGain.x, simForceVec.y*simForceGain.y);

				Chainspace::Index i = state.getInfo().getChains().begin() + simHandForceMode - 1; // simHandForceMode corresponds to chain
				for (Configspace::Index j = state.getInfo().getJoints(i).begin(); j < state.getInfo().getJoints(i).end(); ++j) {
					const size_t k = j - state.getInfo().getJoints().begin(); // from the first joint
					const size_t l = j - state.getInfo().getJoints(i).begin(); // from the first joint in the chain
					force[k] = simForce[std::min(size_t(2), l)];
				}
			}

			handForceRaw = force;
		};
	
		desc.handCtrlDesc->actionFilter = [=] (const golem::Controller::State& prev, golem::Controller::State& next, bool bSendPrev, bool bSendNext) {
			// only if there are no awaiting waypoints
			if (!bSendNext) {
				golem::CriticalSectionWrapper csw(csData);

				const ptrdiff_t stiffnessOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_STIFFNESS);
				const ptrdiff_t dampingOffset = hand->getReservedOffset(Controller::RESERVED_INDEX_DAMPING);

				for (Chainspace::Index i = next.getInfo().getChains().begin(); i < next.getInfo().getChains().end(); ++i)
					for (Configspace::Index j = next.getInfo().getJoints(i).begin(); j < next.getInfo().getJoints(i).end(); ++j) {
						const size_t k = j - next.getInfo().getJoints(i).begin(); // from the first joint in current chain
						if (stiffnessOffset != Controller::ReservedOffset::UNAVAILABLE)
							next.get<ConfigspaceCoord>(stiffnessOffset)[j] = impedStiff[k];
						if (dampingOffset != Controller::ReservedOffset::UNAVAILABLE)
							next.get<ConfigspaceCoord>(dampingOffset)[j] = impedDamp[k];
					}
			}

			handForceController(handForceRaw, prev, next, bSendPrev, bSendNext);
		};
		handCtrl = desc.handCtrlDesc->create(*hand); // create and install callback (throws)
	}
	return true;
}

void Robot::release() {
	if (physPlanner != NULL) scene.releaseObject(*physPlanner);
	if (groundPlane != NULL) scene.releaseObject(*groundPlane);
}

//------------------------------------------------------------------------------

void Robot::render() {
	const bool isActiveArm = armCtrl->getMode() != grasp::ActiveCtrl::MODE_DISABLED;
	const bool isActiveHand = isMultiCtrl ? handCtrl->getMode() != grasp::ActiveCtrl::MODE_DISABLED : false;

	// current state
	golem::Controller::State state = controller->createState();
	controller->lookupState(context.getTimer().elapsed(), state);
	// workspace coords of each joint
	golem::WorkspaceJointCoord wjc;
	controller->jointForwardTransform(state.cpos, wjc);
	
	// display forces/torques
	forceRenderer.reset();
	for (Configspace::Index i = info.getJoints().begin(); i < info.getJoints().end(); ++i) {
		const Mat34 frame = wjc[i];
		Real torque = REAL_ZERO;
		
		if (isActiveArm && armInfo.getJoints().contains(i)) {
			const idx_t j = i - armInfo.getJoints().begin();
			armCtrl->getForceOut(armJointTorques);
			torque = armJointTorques[j];
		}
		if (isActiveHand && handInfo.getJoints().contains(i)) {
			const idx_t j = i - handInfo.getJoints().begin();
			handCtrl->getForceOut(handJointTorques);
			torque = handJointTorques[j];
		}

		if (torque != REAL_ZERO) {
			Vec3 anchor, axis;
			controller->getJoints()[i]->getTrn().twist.toScrew(anchor, axis);
			axis.setMagnitude(forceDispScale*torque);
			frame.R.multiply(axis, axis);
			const Vec3 p[2] = {frame.p, frame.p + axis};
			forceRenderer.addLine(p[0], p[1], RGBA::MAGENTA);
		}
	}

	Mat34 ftFrame;
	ftFrame.multiply(wjc[armInfo.getJoints().end() - 1], this->ftFrame);
	forceRenderer.addAxes(ftFrame, ftFrameSize);

	forceRenderer.render();
}
	
void Robot::keyboardHandler(unsigned char key, int x, int y) {
	grasp::ActiveCtrl* activeCtrl = NULL;

	switch (key) {
	case 1: // F1
		armCtrl->start();
		activeCtrl = armCtrl.get();
		simHandForceMode = FORCE_MODE_DISABLED;
		simArmForceMode = FORCE_MODE_DISABLED;
		break;
	case 2: // F2
		handCtrl->start();
		activeCtrl = handCtrl.get();
		simHandForceMode = FORCE_MODE_DISABLED;
		simArmForceMode = FORCE_MODE_DISABLED;
		break;
	case 3: // F3
		armCtrl->start();
		simForceVec.setZero();
		simArmForceMode = simArmForceMode >= FORCE_MODE_FORCE ? FORCE_MODE_TORQUE : simArmForceMode + 1;
		simHandForceMode = FORCE_MODE_DISABLED;
		context.write("%s: simulated %s ON\n", arm->getName().c_str(), simArmForceMode == FORCE_MODE_FORCE ? "force" : "torque");
		return;
	case 4: // F4
		handCtrl->start();
		simForceVec.setZero();
		simHandForceMode = simHandForceMode >= (U32)handInfo.getChains().size() ? 1 : simHandForceMode + 1;
		simArmForceMode = FORCE_MODE_DISABLED;
		context.write("%s: simulated %s ON\n", hand->getName().c_str(), hand->getChains()[handInfo.getChains().begin() + simHandForceMode - 1]->getName().c_str());
		return;
	case 5: // F5
		armCtrl->stop();
		activeCtrl = armCtrl.get();
		break;
	case 6: // F6
		handCtrl->stop();
		activeCtrl = handCtrl.get();
		break;
	case 9: // F9
		if (impedStiffStep > 0) --impedStiffStep;
		level("stiffness", impedStiffMin, impedStiffMax, impedStiffSteps, impedStiffStep, impedStiff);
		return;
	case 10: // F10
		if (impedStiffStep < impedStiffSteps) ++impedStiffStep;
		level("stiffness", impedStiffMin, impedStiffMax, impedStiffSteps, impedStiffStep, impedStiff);
		return;
	case 11: // F11
		if (impedDampStep > 0) --impedDampStep;
		level("damping", impedDampMin, impedDampMax, impedDampSteps, impedDampStep, impedDamp);
		return;
	case 12: // F12
		if (impedDampStep < impedDampSteps) ++impedDampStep;
		level("damping", impedDampMin, impedDampMax, impedDampSteps, impedDampStep, impedDamp);
		return;
#ifdef _GRASP_HITHANDII_DEBUG
	case '[':
		offsetIdx = offsetIdx <= 0 ? (U32)handInfo.getJoints().size() - 1 : offsetIdx - 1;
		context.write("Offset joint: %d\n", offsetIdx + 1);
		return;
	case ']':
		offsetIdx = offsetIdx >= (U32)handInfo.getJoints().size() - 1 ? 0 : offsetIdx + 1;
		context.write("Offset joint: %d\n", offsetIdx + 1);
		return;
	case '{':
	{
		DLRHitHandII* pDLRHitHandII = dynamic_cast<DLRHitHandII*>(hand);
		if (pDLRHitHandII) {
			Real o[golem::DLRHitHandII::NUM_JOINTS];
			pDLRHitHandII->getOffset(o);
			o[offsetIdx] -= 0.005*REAL_PI;
			pDLRHitHandII->setOffset(o);
			context.write("<offset c1=\"%f\" c2=\"%f\" c3=\"%f\" c4=\"%f\" c5=\"%f\" c6=\"%f\" c7=\"%f\" c8=\"%f\" c9=\"%f\" c10=\"%f\" c11=\"%f\" c12=\"%f\" c13=\"%f\" c14=\"%f\" c15=\"%f\" c16=\"%f\" c17=\"%f\" c18=\"%f\" c19=\"%f\" c20=\"%f\"/>\n", o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7], o[8], o[9], o[10], o[11], o[12], o[13], o[14], o[15], o[16], o[17], o[18], o[19]);
		}
		return;
	}
	case '}':
	{
		DLRHitHandII* pDLRHitHandII = dynamic_cast<DLRHitHandII*>(hand);
		if (pDLRHitHandII) {
			Real o[golem::DLRHitHandII::NUM_JOINTS];
			pDLRHitHandII->getOffset(o);
			o[offsetIdx] += 0.005*REAL_PI;
			pDLRHitHandII->setOffset(o);
			context.write("<offset c1=\"%f\" c2=\"%f\" c3=\"%f\" c4=\"%f\" c5=\"%f\" c6=\"%f\" c7=\"%f\" c8=\"%f\" c9=\"%f\" c10=\"%f\" c11=\"%f\" c12=\"%f\" c13=\"%f\" c14=\"%f\" c15=\"%f\" c16=\"%f\" c17=\"%f\" c18=\"%f\" c19=\"%f\" c20=\"%f\"/>\n", o[0], o[1], o[2], o[3], o[4], o[5], o[6], o[7], o[8], o[9], o[10], o[11], o[12], o[13], o[14], o[15], o[16], o[17], o[18], o[19]);
		}
		return;
	}
#endif // _GRASP_HITHANDII_DEBUG
	default:
		return;
	}

	if (activeCtrl != NULL) {
		const char* str [] = {"active control OFF", "active control ON", "emergency control ON", };
		context.write("%s: %s\n", activeCtrl->getController()->getName().c_str(), str[(size_t)activeCtrl->getMode()]);
	}
};

void Robot::mouseHandler(int button, int state, int x, int y) {
	if (state == 0 && button == 2) {
		simForceModeOvr = true;
		simX = x;
		simY = y;
		simForceVec.setZero();
	}
	else if (state == 1 && button != 7 && button != 8) {
		simForceModeOvr = false;
		simForceVec.setZero();
	}

	if (simForceModeOvr) {
		simForceVec.z += button == 7 ? +Real(2.0) : button == 8 ? -Real(2.0) : Real(0.0);
	}
}

void Robot::motionHandler(int x, int y) {
	if (simForceModeOvr) {
		simForceVec.x = Real(simX - x);
		simForceVec.y = Real(y - simY); // (0, 0) at the upper left corner!
	}
}

//------------------------------------------------------------------------------

void Robot::level(const char* name, const grasp::RealSeq& min, const grasp::RealSeq& max, golem::U32 steps, golem::U32 step, grasp::RealSeq& val) {
	std::stringstream str;
	str << std::setprecision(2) << std::fixed;
	{
		golem::CriticalSectionWrapper csw(csData);
		for (size_t i = 0; i < val.size(); ++i) {
			val[i] = step == 0 ? REAL_ZERO : min[i]*Math::pow(max[i]/min[i], (step - REAL_ONE)/(steps - REAL_ONE));
			str << val[i] << (i < val.size() - 1 ? ", " : "");
		}
	}
	context.write("%s: %s = (%s)\n", (isMultiCtrl ? hand->getName().c_str() : "finger"), name, str.str().c_str());
}

//------------------------------------------------------------------------------

grasp::State Robot::recvState() const {
	std::printf("Robot::recvState() #1\n");
	const SecTmReal t = context.getTimer().elapsed();
	
	std::printf("Robot::recvState() #2\n");
	grasp::State state(*arm);
	// system command
	std::printf("Robot::recvState() #3\n");
	arm->lookupCommand(t, state.command);
	// system state
	std::printf("Robot::recvState() #4\n");
	arm->lookupState(t, state.config);

	std::printf("Robot::recvState() #5\n");
	// F/T sensor at wrist
	{
		golem::CriticalSectionWrapper csw(csData);
		state.ftSensor.set(armForceRaw.data());
	}
	std::printf("Robot::recvState() #6\n");

	return state;
}

void Robot::createTrajectory(const golem::Controller::State& begin, const golem::Controller::State& end, golem::Controller::State::Seq& trajectory) {
	// All bounds are treated as obstacles
	physPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
	// Trajectory from initial position to target position
	for (;;) {
		if (universe.interrupted())
			throw grasp::Interrupted();
		context.debug("Robot::createTrajectory(): Planning movement...\n");
		// Setup target position
		Controller::State cend = end; 
		cend.t = begin.t + trajectoryDuration;
		// Find collision-free trajectory and wait until the device is ready for new commands
		if (planner->findGlobalTrajectory(begin, cend, trajectory, trajectory.begin()))
			break;
	}
}

void Robot::createArmTrajectory(const golem::Controller::State& begin, const golem::Mat34& end, golem::Controller::State::Seq& trajectory) {
	std::printf("Robot::createArmTrajectory() #1\n");
	// All bounds are treated as obstacles
	physPlanner->setCollisionBoundsGroup(Bounds::GROUP_ALL);
	// Setup target position
	GenWorkspaceChainState wend;
	wend.setToDefault(info.getChains().begin(), info.getChains().end()); // all used chains
	wend.wpos[armInfo.getChains().begin()] = end;
	// Trajectory from initial position to end position
	for (;;) {
		if (universe.interrupted())
			throw grasp::Interrupted();
		context.debug("Robot::createArmTrajectory(): Planning movement...\n");
		// Find end position
		Controller::State cend = begin;
		if (!planner->findTarget(begin, wend, cend))
			continue;
		// Find collision-free trajectory and wait until the device is ready for new commands
		cend.t = begin.t + trajectoryDuration;
		if (planner->findGlobalTrajectory(begin, cend, trajectory, trajectory.begin()))
			continue;
		// success
		break;
	}
	std::printf("Robot::createArmTrajectory() #2\n");
}

void Robot::sendTrajectory(const golem::Controller::State::Seq& trajectory) {
	if (trajectory.empty())
		throw Message(Message::LEVEL_ERROR, "Robot::sendTrajectory(): empty trajectory!");
	context.debug("Robot::sendTrajectory(): Moving robot...\n");
	golem::Controller::State::Seq trj = trajectory;
	// shift trajectory in time to avoid initial high hand joints accelerations (due to soft fingers)
	golem::SecTmReal dt = context.getTimer().elapsed() + trajectoryIdle - trj.begin()->t;
	for (Controller::Trajectory::iterator i = trj.begin(); i != trj.end(); ++i)
		i->t += dt;
	// Move the robot
	(void)controller->send(&*trj.begin(), &*trj.end());
}

bool Robot::waitForBegin(golem::MSecTmU32 timeWait) {
	return controller->waitForBegin(timeWait);
}	

bool Robot::waitForEnd(golem::MSecTmU32 timeWait) {
	return controller->waitForEnd(timeWait);
}

//------------------------------------------------------------------------------

void spinta::XMLData(Robot::Desc &val, Context* context, XMLContext* xmlcontext, bool create) {
	golem::XMLData(*val.physPlannerDesc, context, xmlcontext, create);

	xmlcontext = xmlcontext->getContextFirst("robot");

	try {
		val.armToolBounds.clear();
		golem::XMLData(val.armToolBounds, val.armToolBounds.max_size(), xmlcontext, "arm_tool_bounds", create);
	}
	catch (const golem::MsgXMLParserNameNotFound&) {
	}

	golem::XMLData("trajectory_duration", val.trajectoryDuration, xmlcontext, create);
	golem::XMLData("trajectory_idle", val.trajectoryIdle, xmlcontext, create);


	golem::XMLData("force_disp_scale", val.forceDispScale, xmlcontext, create);

#ifdef _GRASP_FTSENSOR_DAQ
	golem::XMLData("use_ft_sensor_client", val.useFTSensorClient, xmlcontext->getContextFirst("ft_sensor"), create);
	if (!val.useFTSensorClient) {
		DAQFTSensor::Desc* desc = new DAQFTSensor::Desc;
		val.ftSensorDesc.reset(desc);
		golem::XMLData("calibration_file", desc->calibrationFile, xmlcontext->getContextFirst("ft_sensor daq"), create);
		golem::XMLData("sampling_rate", desc->samplingRate, xmlcontext->getContextFirst("ft_sensor daq"), create);
		golem::XMLData("avg_window_size", desc->avgWindowSize, xmlcontext->getContextFirst("ft_sensor daq"), create);
	}
	else
#endif
	{
		grasp::FTSensorClient::Desc* desc = new grasp::FTSensorClient::Desc;
		val.ftSensorDesc.reset(desc);
		golem::XMLData("host", desc->host, xmlcontext->getContextFirst("ft_sensor client"), create);
		golem::XMLData("port", desc->port, xmlcontext->getContextFirst("ft_sensor client"), create);
	}
	
	golem::XMLData(val.ftFrame, xmlcontext->getContextFirst("ft_sensor frame"), create);
	golem::XMLData(val.ftFrameSize, xmlcontext->getContextFirst("ft_sensor frame_size"), create);
	golem::XMLData(val.ftGain, xmlcontext->getContextFirst("ft_sensor gain"), create);
	golem::XMLData(val.ftLimit, xmlcontext->getContextFirst("ft_sensor limit"), create);

	golem::XMLData(val.impedStiffMin, xmlcontext->getContextFirst("impedance stiff_min"), create);
	golem::XMLData(val.impedStiffMax, xmlcontext->getContextFirst("impedance stiff_max"), create);
	golem::XMLData("stiff_steps", val.impedStiffSteps, xmlcontext->getContextFirst("impedance"), create);
	golem::XMLData("stiff_step_init", val.impedStiffStepInit, xmlcontext->getContextFirst("impedance"), create);
	golem::XMLData(val.impedDampMin, xmlcontext->getContextFirst("impedance damp_min"), create);
	golem::XMLData(val.impedDampMax, xmlcontext->getContextFirst("impedance damp_max"), create);
	golem::XMLData("damp_steps", val.impedDampSteps, xmlcontext->getContextFirst("impedance"), create);
	golem::XMLData("damp_step_init", val.impedDampStepInit, xmlcontext->getContextFirst("impedance"), create);

	grasp::XMLData(dynamic_cast<grasp::ActiveCtrlFTSensor::Desc&>(*val.armCtrlDesc), xmlcontext->getContextFirst("active_ctrl_arm"), create);
	grasp::XMLData(dynamic_cast<grasp::ActiveCtrl::Desc&>(*val.handCtrlDesc), xmlcontext->getContextFirst("active_ctrl_hand"), create);
}

//------------------------------------------------------------------------------
