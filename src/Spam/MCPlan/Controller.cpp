#include <Spinta/Planner/Controller.h>

//-------------------------------------------------------------------------

using namespace golem;
using namespace spinta;

//-------------------------------------------------------------------------

PushController::PushController(Robot *robot) : pRobot(robot) {
}

PushController::~PushController() {}

bool PushController::create(const Desc &desc) {
	std::printf("Controller::create()\n");
	if (!desc.isValid())
		throw MsgCtrlInvalidDesc(Message::LEVEL_CRIT, "Controller::create(): Invalid description");	
	return true;
}

void spinta::XMLData(PushController::Desc &val, golem::XMLContext* context, bool create) {
	ASSERT(context)
}

//-------------------------------------------------------------------------
