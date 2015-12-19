#include <Spinta/Planner/PushPlanner.h>

//-------------------------------------------------------------------------

using namespace golem;
using namespace spinta;

//-------------------------------------------------------------------------

PushPlanner::PushPlanner(Model::Ptr model, Robot *robot) : pModel(model), context(&model->context), rand(pModel->context.getRandSeed()), pRobot(robot) {
}

PushPlanner::~PushPlanner() {}

bool PushPlanner::create(const Desc &desc) {
	context->getMessageStream()->write(Message::LEVEL_DEBUG, "PushPlanner::create()\n");
	if (!desc.isValid())
		throw MsgPushPlannerInvalidDesc(Message::LEVEL_CRIT, "PushPlanner::create(): Invalid description");

	numDirections = desc.numDirections;
	numPushes = desc.numPushes;
	deltaS = desc.deltaS;
	nonGaussianPerturbation = desc.nonGaussianPerturbation;
	numMaxFails = desc.numMaxFails;
	numMaxIters = desc.numMaxIters;

	offset = desc.offset;
	accToTarget = desc.accToTarget;

	return true;
}

void PushPlanner::reset() {
	// TODO
}

//-------------------------------------------------------------------------
/* Select push actions over a set of directions, default 4 (north, south, 
east, west). For each direction a set of stochastic perturbated trajectories
have been computed, default only one trajcetory per direction. 
The perturbation is over x,y,z and can be drawn from a zero-Gaussian distribution
or an uniform distribution. */
PushAction::Seq PushPlanner::selectPushActions() {
	context->getMessageStream()->write(Message::LEVEL_DEBUG, "PushPlanner::selectPushAction()\n");
	PushAction::Seq inputs;
	Mat34 trajStartPose, trajEndPose, objPose;
	objPose = pModel->pManipulandum->getPose();

	Mat34Seq offsetStartDir, offsetEndDir;
	Mat33 rotId; rotId.setId();
	offsetStartDir.reserve(numDirections);
	offsetStartDir.push_back(Mat34(rotId, Vec3(-offset[0], .0, .0)));
	offsetStartDir.push_back(Mat34(rotId, Vec3(+offset[0], .0, .0)));
	offsetStartDir.push_back(Mat34(rotId, Vec3(.0, -offset[0], .0)));
	offsetStartDir.push_back(Mat34(rotId, Vec3(.0, +offset[0], .0)));

	offsetEndDir.reserve(numDirections);
	offsetEndDir.push_back(Mat34(rotId, Vec3(+offset[0], .0, .0)));
	offsetEndDir.push_back(Mat34(rotId, Vec3(-offset[0], .0, .0)));
	offsetEndDir.push_back(Mat34(rotId, Vec3(.0, +offset[0], .0)));
	offsetEndDir.push_back(Mat34(rotId, Vec3(.0, -offset[0], .0)));

	Vec3 startPoseNoise, endPoseNoise;
	for (size_t direction = 0; direction < numDirections; ++direction) {
		trajStartPose.multiply(objPose, offsetStartDir[direction]);
		trajEndPose.multiply(objPose, offsetEndDir[direction]);

		for (size_t perturbation = 0; perturbation < numPushes; ++perturbation) {
			if (nonGaussianPerturbation) {
				startPoseNoise.set(rand.nextUniform(-offset[0], offset[0]), 
					rand.nextUniform(-offset[1], offset[1]), rand.nextUniform(-offset[2], offset[2]));
				endPoseNoise.set(rand.nextUniform(-offset[0], offset[0]), 
					rand.nextUniform(-offset[1], offset[1]), rand.nextUniform(-offset[2], offset[2]));
			} else {
				startPoseNoise.set(rand.nextGaussian(0.0, offset[0]), 
					rand.nextGaussian(0.0, offset[1]), rand.nextGaussian(0.0, offset[2]));
				endPoseNoise.set(rand.nextGaussian(0.0, offset[0]), 
					rand.nextGaussian(0.0, offset[1]), rand.nextGaussian(0.0, offset[2]));

			}
				
			trajStartPose.R.rotX(Real(-0.5)*REAL_PI);
			trajStartPose.p += startPoseNoise;
			trajEndPose.R.rotX(Real(-0.5)*REAL_PI);
			trajEndPose.p += endPoseNoise;
			PushAction input(trajStartPose, trajEndPose);
			inputs.push_back(input);
		}
	}
	return inputs;
}

bool PushPlanner::satisfied(const Mat34 &x) {
	// TODO
	return true;
}

Mat34 PushPlanner::randomState() {
	Mat34 s;
	//s.setId();
	//for (size_t i=0; i<3; ++i)
	//	s.p[i] = rand.next()*(linUpBound[i] - linLowBound[i]);
	//s.R.rotZ(rand.next()*(angUpBound[3] - angLowBound[3]));
	return s;
}

void PushPlanner::generateTrajectorySim(const Mat34 &target) {
	std::printf("PushPlanner::generateTrajectorySim() #1\n"); std::fflush(stdout);
	const golem::Controller::State begin = pRobot->recvState().command;
	std::printf("PushPlanner::generateTrajectorySim() #2\n"); std::fflush(stdout);
	golem::Controller::State::Seq trajectory;
	std::printf("PushPlanner::generateTrajectorySim() #3\n"); std::fflush(stdout);
	pRobot->createArmTrajectory(begin, target, trajectory);
	std::printf("PushPlanner::generateTrajectorySim() #4\n"); std::fflush(stdout);
	pRobot->sendTrajectory(trajectory);
}

Mat34 PushPlanner::integrate(const Mat34 &current, const Mat34 &target, PushAction *action) {
	context->getMessageStream()->write(Message::LEVEL_DEBUG, "PushPlanner::integrate()\n");
	Real costToGoal = pModel->cost(current, target);
	// moves the robot to the initial pose
	std::printf("PushPlanner::integrate() #1\n");
	generateTrajectorySim(action->initPose);
	std::printf("PushPlanner::integrate() #2\n");
	pRobot->waitForEnd();
	std::printf("PushPlanner::integrate() #3\n");
	// computes the pushing direction
	action->deltaS = deltaS;
	Vec3 delta = pModel->stateTransitionEquation(action->deltaS, action->direction());
	Mat34 nx(current);
	std::printf("PushPlanner::integrate() #4\n");
	// previous and current pose of the manipulandum
	Mat34 prevObjPose, currObjPose;
	prevObjPose = currObjPose = pModel->getObjPose();
	std::printf("PushPlanner::integrate() #5\n");
	size_t iter = 0;
	do {
		iter++;
		action->manipulandumPoses.push_back(currObjPose);
		prevObjPose.R.set(currObjPose.R);
		prevObjPose.p.set(currObjPose.p);
		nx.p += delta;
		generateTrajectorySim(nx);
		pRobot->waitForEnd();
		currObjPose = pModel->getObjPose();
	} while(satisfied(prevObjPose, currObjPose, target) && (iter < numPushes));
	std::printf("PushPlanner::integrate() #6\n");
	// restore the initial pose of the robot in order to avoid that the obj is still in contact
	generateTrajectorySim(action->initPose);
	std::printf("PushPlanner::integrate() #7\n");
	pRobot->waitForEnd();
	std::printf("PushPlanner::integrate() #8\n");
	currObjPose = pModel->getObjPose();

	if (!prevObjPose.equals(currObjPose, golem::REAL_EPS)) {
		prevObjPose.R.set(currObjPose.R);
		prevObjPose.p.set(currObjPose.p);
		action->t = iter;
	}
	else action->t = iter - 1;

	action->manipulandumPoses.push_back(prevObjPose);
	return prevObjPose;
}


PushAction::Seq PushPlanner::getInput(const golem::Mat34 &xNear, const golem::Mat34 &xRand, golem::Mat34 *objPose, bool &success) {
	context->getMessageStream()->write(Message::LEVEL_DEBUG, "PushPlanner::getInput()\n");
	// PRECOND: no dynamic object in the scene 
	if(pModel->pManipulandum != NULL)
		pModel->removeManipulandum();
	
	bool found_local_minima = false;
	success = false;
	size_t iter=1, nFailure=0;

	PushAction::Seq policy;
	PushAction selectedAction;
	Mat34 currentObjPose, tempObjPose, bestObjPose;
	Real costToTarget = pModel->cost(xNear, xRand), tempCostToTarget;

	currentObjPose = xNear;
//	cout << " SELECT INPUT init_cost:" << c_goal << endl;
	while(!satisfied(currentObjPose, xRand) && (iter < numMaxIters) && (nFailure < numMaxFails)) {
		pModel->createManipulandum(currentObjPose);
		PushAction::Seq local_trajectories = selectPushActions();

		for (PushAction::Seq::iterator action = local_trajectories.begin(); action != local_trajectories.end(); ++action) {
			if(pModel->pManipulandum == NULL) 
				pModel->createManipulandum(currentObjPose);

			tempObjPose = integrate(currentObjPose, xRand, &*action);
			tempCostToTarget = pModel->cost(tempObjPose, xRand);
			if(tempCostToTarget < costToTarget){
				costToTarget = tempCostToTarget;
				selectedAction = *action;
				bestObjPose = tempObjPose;
				found_local_minima = true;
			}
//#ifndef SIMULATE
//			moveArmHome();
//#endif
			pModel->removeManipulandum();
			// swap the above instructions to remove the obj before move the arm to the home position
		}
		if (found_local_minima) {
			currentObjPose = bestObjPose;
			policy.push_back(selectedAction);
			success = true;
			nFailure = 0;
		}
		else nFailure++;
		
		found_local_minima = false;
		iter++;
	}

	if (success) {
		*objPose = bestObjPose; 
	}
	
	return policy;
}

//-------------------------------------------------------------------------

void spinta::XMLData(PushPlanner::Desc &val, Context *context, XMLContext *xmlcontext, bool create) {
	ASSERT(xmlcontext)

	XMLData("num_directions", val.numDirections, xmlcontext->getContextFirst("push_planner"), create);
	XMLData("num_pushes", val.numPushes, xmlcontext->getContextFirst("push_planner"), create);
	XMLData("non_gaussian_perturbation", val.nonGaussianPerturbation, xmlcontext->getContextFirst("push_planner"), create);
	XMLData("num_max_fails", val.numMaxFails, xmlcontext->getContextFirst("push_planner"), create);
	XMLData("num_max_iters", val.numMaxIters, xmlcontext->getContextFirst("push_planner"), create);
	XMLData("accuracy_to_target", val.accToTarget, xmlcontext->getContextFirst("push_planner"), create);
	XMLData(val.deltaS, xmlcontext->getContextFirst("push_planner spatial_delta"), create);
	XMLData(val.offset, xmlcontext->getContextFirst("push_planner offset"), create);

//	XMLData(*val.pControllerDesc, xmlcontext->getContextFirst("icub_controller"), create);
}

////-------------------------------------------------------------------------
//
//ICubPushPlanner::ICubPushPlanner(ICubModel::Ptr model) : pModel(model) {
//}
//
//ICubPushPlanner::~ICubPushPlanner() {}
//
//bool ICubPushPlanner::create(const Desc &desc) { 
//	std::printf("ICubPushPlanner::create()\n");
//	if (!desc.isValid())
//		return false;
//
//	pController = desc.pControllerDesc->create();
//
//	return true;
//}
//
//PushAction ICubPushPlanner::getInput(const golem::Mat34 &xNear, const golem::Mat34 &xRand, golem::Mat34 *objPos) {
//	PushAction u(xNear.p, xRand.p);
//	objPos->p.set(xRand.p);
//	objPos->R.set(xRand.R);
//	return u;
//}
//
////-------------------------------------------------------------------------
//
//void spinta::XMLData(ICubPushPlanner::Desc &val, Context *context, XMLContext *xmlcontext, bool create) {
//	ASSERT(xmlcontext)
//
//	XMLData("num_pushes", val.numPushes, xmlcontext->getContextFirst("icub_push_planner"), create);
//	XMLData("num_max_fails", val.numMaxFails, xmlcontext->getContextFirst("icub_push_planner"), create);
//	XMLData("num_max_iters", val.numMaxIters, xmlcontext->getContextFirst("icub_push_planner"), create);
//
//	XMLData(*val.pControllerDesc, xmlcontext->getContextFirst("icub_controller"), create);
//}