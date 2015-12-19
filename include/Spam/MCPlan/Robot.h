/** @file Robot.h
 * 
 * @author	Marek Kopicki
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _SPINTA_ROBOT_H_
#define _SPINTA_ROBOT_H_

//------------------------------------------------------------------------------

#include <Golem/PhysCtrl/PhysPlanner.h>
#include <Grasp/Grasp/Data.h>
#include <Grasp/Grasp/ActiveCtrl.h>
#include <Grasp/Grasp/FTSensor.h>

//------------------------------------------------------------------------------

namespace golem {
	class XMLContext;
};

//------------------------------------------------------------------------------

namespace spinta {

//------------------------------------------------------------------------------

/** Robot is the interafce to a robot (right arm + hand), sensing devices and objects. */
class Robot : public golem::Object {
public:
	typedef golem::shared_ptr<Desc> Ptr;

	/** Force controller */
	typedef std::function<void(const grasp::RealSeq&, const golem::Controller::State&, golem::Controller::State&, bool, bool)> ForceController;

	/** Simulated arm/hand force mode */
	enum ForceMode {
		/** Inactive */
		FORCE_MODE_DISABLED = 0,
		/** Torque */
		FORCE_MODE_TORQUE,
		/** Force */
		FORCE_MODE_FORCE,
	};
	
	/** Robot factory */
	class Desc : public golem::Object::Desc {
	protected:
		CREATE_FROM_OBJECT_DESC1(Robot, golem::Object::Ptr, golem::Scene&)

	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Arm and hand + planner. */
		golem::PhysPlanner::Desc::Ptr physPlannerDesc;
		/** Arm tool bounds */
		golem::Bounds::Desc::Seq armToolBounds;

		/** Min trajectory duration */
		golem::SecTmReal trajectoryDuration;
		/** Trajectory idle time */
		golem::SecTmReal trajectoryIdle;

		/** Active arm controller. */
		grasp::ActiveCtrl::Desc::Ptr armCtrlDesc;
		/** Active hand controller. */
		grasp::ActiveCtrl::Desc::Ptr handCtrlDesc;
		
		/** FT sensor */
		grasp::FTSensor::Desc::Ptr ftSensorDesc;
		/** FT sensor client */
		bool useFTSensorClient;
		/** FT sensor frame with respect to the tool frame */
		golem::Mat34 ftFrame;
		/** FT sensor frame size */
		golem::Vec3 ftFrameSize;
		/** FT sensor gain */
		golem::Twist ftGain;
		/** FT sensor force/torque limit */
		golem::Twist ftLimit;

		/** Simulated force gains */
		golem::Vec3 simForceGain;
		/** Force display scaling factor */
		golem::Real forceDispScale;

		/** Hand impedance control, min and max stiffness levels */
		grasp::RealSeq impedStiffMin, impedStiffMax;
		/** Hand impedance control, stiffness num of steps */
		golem::U32 impedStiffSteps;
		/** Hand impedance control, stiffness initial step */
		golem::U32 impedStiffStepInit;
		/** Hand impedance control, min and max damping levels */
		grasp::RealSeq impedDampMin, impedDampMax;
		/** Hand impedance control, damping num of steps */
		golem::U32 impedDampSteps;
		/** Hand impedance control, damping initial step */
		golem::U32 impedDampStepInit;

		/** Arm force controller */
		ForceController armForceController;
		/** Hand force controller */
		ForceController handForceController;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			physPlannerDesc.reset(new golem::PhysPlanner::Desc);
			armToolBounds.clear();
			trajectoryDuration = golem::SecTmReal(1.0);
			trajectoryIdle = golem::SecTmReal(0.5);

			armCtrlDesc.reset(new grasp::ActiveCtrlFTSensor::Desc);
			handCtrlDesc.reset(new grasp::ActiveCtrl::Desc);

			useFTSensorClient = false;
#ifdef _GRASP_FTSENSOR_DAQ
			ftSensorDesc.reset(new grasp::DAQFTSensor::Desc);
#else
			ftSensorDesc.reset(new grasp::FTSensorClient::Desc);
#endif
			ftFrame.setId();
			ftFrameSize.set(0.1);
			ftGain.set(golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0), golem::Real(-1.0));
			ftLimit.set(golem::Real(10.0), golem::Real(10.0), golem::Real(10.0), golem::Real(2.0), golem::Real(2.0), golem::Real(2.0));

			simForceGain.set(golem::Real(0.02), golem::Real(0.02), golem::Real(0.02));
			forceDispScale = golem::Real(0.2);

			impedStiffMin.assign(4, golem::Real(0.0));
			impedStiffMax.assign(4, golem::Real(100.0));
			impedStiffSteps = 10;
			impedStiffStepInit = 2;
			impedDampMin.assign(4, golem::Real(0.0));
			impedDampMax.assign(4, golem::Real(100.0));
			impedDampSteps = 10;
			impedDampStepInit = 2;

			armForceController = [] (const grasp::RealSeq&, const golem::Controller::State&, golem::Controller::State&, bool, bool) {};
			handForceController = [] (const grasp::RealSeq&, const golem::Controller::State&, golem::Controller::State&, bool, bool) {};
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Object::Desc::isValid())
				return false;

			if (physPlannerDesc == NULL || !physPlannerDesc->isValid())
				return false;
			for (golem::Bounds::Desc::Seq::const_iterator i = armToolBounds.begin(); i != armToolBounds.end(); ++i)
				if (!(*i)->isValid())
					return false;
			if (trajectoryDuration <= golem::SecTmReal(0.0) || trajectoryIdle <= golem::SecTmReal(0.0))
				return false;

			if (armCtrlDesc == NULL || !armCtrlDesc->isValid())
				return false;
			if (handCtrlDesc == NULL || !handCtrlDesc->isValid())
				return false;

			if (ftSensorDesc == NULL || !ftSensorDesc->isValid())
				return false;
			if (!ftFrame.isValid() || !ftGain.isValid() || !ftLimit.isPositive())
				return false;

			if (!simForceGain.isValid() || !ftFrameSize.isValid())
				return false;

			if (forceDispScale <= golem::REAL_ZERO)
				return false;

			if (impedStiffMin.empty() || impedStiffMin.size() != impedStiffMax.size() || impedStiffSteps < 2 || impedStiffStepInit > impedStiffSteps)
				return false;
			for (size_t i = 0; i < impedStiffMin.size(); ++i)
				if (impedStiffMin[i] < golem::REAL_ZERO || impedStiffMin[i] + golem::REAL_EPS > impedStiffMax[i])
					return false;
			if (impedDampMin.empty() || impedDampMin.size() != impedDampMax.size() || impedDampSteps < 2 || impedDampStepInit > impedDampSteps)
				return false;
			for (size_t i = 0; i < impedDampMin.size(); ++i)
				if (impedDampMin[i] < golem::REAL_ZERO || impedDampMin[i] + golem::REAL_EPS > impedDampMax[i])
					return false;

			if (armForceController == NULL/* || handForceController == NULL*/)
				return false;

			return true;
		}
	};

	/** Defines data access member */
	#define GRASP_ENVIRONMENT_GET(TYPE, NAME, POINTER)\
		inline const TYPE get##NAME() const {\
			return this->POINTER;\
		}\
		inline TYPE get##NAME() {\
			return this->POINTER;\
		}

	GRASP_ENVIRONMENT_GET(golem::Planner*, Planner, planner)
	GRASP_ENVIRONMENT_GET(golem::Controller*, Controller, controller)
	GRASP_ENVIRONMENT_GET(golem::SingleCtrl*, Arm, arm)
	GRASP_ENVIRONMENT_GET(golem::SingleCtrl*, Hand, hand)

	#undef GRASP_ENVIRONMENT_GET

	/** Controller state info */
	inline const golem::Controller::State::Info& getStateInfo() const {
		return info;
	}

	/** Receive robot state */
	virtual grasp::State recvState() const;
	/** Configspace trajectory */
	virtual void createTrajectory(const golem::Controller::State& begin, const golem::Controller::State& end, golem::Controller::State::Seq& trajectory);
	/** Workspace trajectory */
	virtual void createArmTrajectory(const golem::Controller::State& begin, const golem::Mat34& end, golem::Controller::State::Seq& trajectory);
	/** Send robot trajectory */
	virtual void sendTrajectory(const golem::Controller::State::Seq& trajectory);
	/** Controller has just sent new trajectory segment */
	virtual bool waitForBegin(golem::MSecTmU32 timeWait = golem::MSEC_TM_U32_INF);
	/** Controller has just sent the last trajectory segment */
	virtual bool waitForEnd(golem::MSecTmU32 timeWait = golem::MSEC_TM_U32_INF);

protected:
	/** ground plane */
	golem::Object* groundPlane;
	/** PhysPlanner */
	golem::PhysPlanner* physPlanner;
	/** Planner */
	golem::Planner* planner;
	/** Robot controller heuristic */
	golem::Heuristic* heuristic;
	/** Robot controller */
	golem::Controller* controller;
	/** Robot right arm */
	golem::SingleCtrl *arm;
	/** Robot right hand */
	golem::SingleCtrl *hand;
	/** Flag for multi controllers */
	bool isMultiCtrl;

	/** Controller state info */
	golem::Controller::State::Info info;
	/** Controller state info */
	golem::Controller::State::Info armInfo;
	/** Controller state info */
	golem::Controller::State::Info handInfo;
	
	/** Arm tool bounds */
	golem::Bounds::Desc::Seq armToolBounds;

	/** Min trajectory duration */
	golem::SecTmReal trajectoryDuration;
	/** Trajectory idle time */
	golem::SecTmReal trajectoryIdle;

	/** Force access cs */
	mutable golem::CriticalSection csData;

	/** F/T sensor */
	grasp::FTSensor::Ptr ftSensor;
	/** F/T sensor mode */
	bool ftSensorMode;

	/** Simulated arm force mode */
	golem::U32 simArmForceMode;
	/** Simulated hand force mode */
	golem::U32 simHandForceMode;
	/** Simulated force mode overlay */
	bool simForceModeOvr;
	/** Simulated force */
	golem::Vec3 simForceVec;
	/** Simulated force gains */
	golem::Vec3 simForceGain;
	/** Simulated force screen coords */
	int simX, simY;

	/** Active arm controller. */
	grasp::ActiveCtrl::Ptr armCtrl;
	/** Active hand controller. */
	grasp::ActiveCtrl::Ptr handCtrl;

	/** FT sensor frame with respect to the tool frame */
	golem::Mat34 ftFrame, ftFrameInv;
	/** FT sensor frame size */
	golem::Vec3 ftFrameSize;
	/** FT sensor gain */
	golem::Twist ftGain;
	/** FT sensor force/torque limit */
	golem::Twist ftLimit;

	/** Hand impedance control, min and max stiffness levels */
	grasp::RealSeq impedStiffMin, impedStiffMax;
	/** Hand impedance control, stiffness level */
	grasp::RealSeq impedStiff;
	/** Hand impedance control, stiffness num of steps */
	golem::U32 impedStiffSteps;
	/** Hand impedance control, stiffness step */
	golem::U32 impedStiffStep;
	/** Hand impedance control, min and max damping levels */
	grasp::RealSeq impedDampMin, impedDampMax;
	/** Hand impedance control, damping level */
	grasp::RealSeq impedDamp;
	/** Hand impedance control, damping num of steps */
	golem::U32 impedDampSteps;
	/** Hand impedance control, damping step */
	golem::U32 impedDampStep;

	/** Arm force */
	grasp::RealSeq armForceRaw;
	/** Hand force */
	grasp::RealSeq handForceRaw;
	/** Arm force controller */
	ForceController armForceController;
	/** Hand force controller */
	ForceController handForceController;

	/** Hand offset */
	golem::U32 offsetIdx;
	/** Force display scaling factor */
	golem::Real forceDispScale;
	/** Arm joint torques */
	grasp::RealSeq armJointTorques;
	/** Hand joint torques */
	grasp::RealSeq handJointTorques;

	/** Render data */
	golem::DebugRenderer forceRenderer;

	/** Compute step-th value in <min, max> assuming logarythmic scale */
	virtual void level(const char* name, const grasp::RealSeq& min, const grasp::RealSeq& max, golem::U32 steps, golem::U32 step, grasp::RealSeq& val);

	// golem::Object interface
	virtual void render();
	virtual void keyboardHandler(unsigned char key, int x, int y);
	virtual void mouseHandler(int button, int state, int x, int y);
	virtual void motionHandler(int x, int y);

	// construction
	Robot(golem::Scene &scene);
	bool create(const Desc& desc);
	virtual void release();
};

/** Reads/writes object from/to a given XML context */
void XMLData(Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_SPINTA_ROBOT_H_*/
