/** @file GraphPlanner.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Spam/Spam/GraphPlanner.h>
#include <Golem/Device/MultiCtrl/MultiCtrl.h>
#include <Golem/Device/SingleCtrl/SingleCtrl.h>
#include <Golem/Plan/Data.h>
#include <Spam/Spam/Heuristic.h>
#include <iomanip> // std::setprecision
//#include <Spam/Spam/Data.h>

//------------------------------------------------------------------------------

using namespace spam;
using namespace golem;

//------------------------------------------------------------------------------

std::string spam::plannerDebug(golem::Planner& planner) {
	std::stringstream str;

	const Heuristic& heuristic = planner.getHeuristic();
	FTDrivenHeuristic *pHeuristic = dynamic_cast<FTDrivenHeuristic*>(&planner.getHeuristic());
	const Controller& controller = planner.getController();
	const Heuristic::JointDesc::JointSeq& jointDesc = heuristic.getJointDesc();
	const Chainspace::Range chains = controller.getStateInfo().getChains();
	golem::U32 enabled = 0;
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		const Configspace::Range joints = controller.getStateInfo().getJoints(i);
		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j)
			if (jointDesc[j]->enabled) ++enabled;
	}

	str << controller.getName() << ": chains=" << controller.getStateInfo().getChains().size() << ", joints=" << controller.getStateInfo().getJoints().size() << "(enabled=" << enabled << "), collisions=" << (heuristic.getDesc().collisionDesc.enabled ? "ENABLED" : "DISABLED") << ", non-Euclidian metrics=" << (pHeuristic && pHeuristic->enableUnc ? "ENABLE" : "DISABLE") << ", point cloud collisions=" << (pHeuristic && pHeuristic->getPointCloudCollision() ? "ENABLE" : "DISABLE");

	return str.str();
}

std::string spam::plannerConfigspaceDebug(golem::Planner& planner, const golem::ConfigspaceCoord* c) {
	std::stringstream str;

	const Heuristic& heuristic = planner.getHeuristic();
	const Controller& controller = planner.getController();
	const Heuristic::JointDesc::JointSeq& jointDesc = heuristic.getJointDesc();
	const Chainspace::Range chains = controller.getStateInfo().getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		str << std::setprecision(3) << "{" << controller.getChains()[i]->getName() << ": ";
		const Configspace::Range joints = controller.getStateInfo().getJoints(i);
		for (Configspace::Index j = joints.begin(); j < joints.end(); ++j) {
			str << "(" << j - controller.getStateInfo().getJoints().begin() << ", ";
			str << (jointDesc[j]->enabled ? "Y" : "N") << ", ";
			if (c != nullptr) str << (*c)[j] << "/";
			str << jointDesc[j]->dfltPos << (j < joints.end() - 1 ? "), " : ")");
		}
		str << (i < chains.end() - 1 ? "}, " : "}");
	}

	return str.str();
}

std::string spam::plannerWorkspaceDebug(golem::Planner& planner, const golem::WorkspaceChainCoord* w) {
	std::stringstream str;

	const Heuristic& heuristic = planner.getHeuristic();
	const Controller& controller = planner.getController();
	const Heuristic::ChainDesc::ChainSeq& chainDesc = heuristic.getChainDesc();
	const Chainspace::Range chains = controller.getStateInfo().getChains();
	for (Chainspace::Index i = chains.begin(); i < chains.end(); ++i) {
		str << std::setprecision(3) << "{" << controller.getChains()[i]->getName() << ": ";
		if (w != nullptr) {
			const Vec3 p((*w)[i].p);
			const Quat q((*w)[i].R);
			chainDesc[i]->enabledLin ? str << "lin=(" << p.x << ", " << p.y << ", " << p.z << "), " : str << "lin=N, ";
			chainDesc[i]->enabledAng ? str << "ang=(" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << "), " : str << "ang=N";
		}
		else {
			chainDesc[i]->enabledLin ? str << "lin=Y, " : str << "lin=N, ";
			chainDesc[i]->enabledAng ? str << "ang=Y, " : str << "ang=N";
		}

		str << (i < chains.end() - 1 ? "}, " : "}");
	}

	return str.str();
}

//------------------------------------------------------------------------------

Planner::Desc::Ptr RagGraphPlanner::Desc::load(Context* context, const std::string& libraryPath, const std::string& configPath) {
	Planner::Desc::Ptr pDesc;
	
	context->debug("RagPlanner::Desc::load(): loading library %s and config %s.xml...\n", libraryPath.c_str(), configPath.c_str());
	
	// TODO load library rather than explicitly create GraphPlanner description
	RagGraphPlanner::Desc* pGraphPlannerDesc = new RagGraphPlanner::Desc;
	pDesc.reset(pGraphPlannerDesc);

	// load config
	XMLData(*pGraphPlannerDesc, XMLParser::load(configPath + ".xml")->getContextRoot()->getContextFirst("golem planner"));

	return pDesc;
}

Planner::Desc::Ptr RagGraphPlanner::Desc::load(Context* context, XMLContext* xmlcontext) {
	// driver and config paths must be specified in xmlcontext
	std::string libraryPath, configPath;
	XMLData("library_path", libraryPath, xmlcontext);
	XMLData("config_path", configPath, xmlcontext);
	// load driver and config
	return load(context, libraryPath, configPath);
}

//------------------------------------------------------------------------------

RagGraphPlanner::RagGraphPlanner(Controller& controller) :	GraphPlanner(controller) {
}

bool RagGraphPlanner::create(const Desc& desc) {
//	context.write("RagGraphPlanner::create()\n");
	GraphPlanner::create(desc); // throws	

	MultiCtrl* multiCtrl = dynamic_cast<MultiCtrl*>(&controller);
	if (multiCtrl == NULL || multiCtrl->getControllers().size() < 2)
		throw Message(Message::LEVEL_CRIT, "Robot::create(): Robot requires MultiCtrl at least two connected devices");	
	armInfo = dynamic_cast<SingleCtrl*>(multiCtrl->getControllers()[0])->getStateInfo();
	handInfo = dynamic_cast<SingleCtrl*>(multiCtrl->getControllers()[1])->getStateInfo();
	
	for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
		jointDescSeq.push_back(pHeuristic->getJointDesc()[j]->enabled);
	//context.write("JOINT DESC (cre): <");
	//for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
	//	context.write("%s ", pHeuristic->getJointDesc()[j]->enabled ? "ON" : "OFF");	
	//context.write(">\n");
	//context.write("JOINT DESC (seq): <");
	//for (size_t j = 0; j < Configspace::DIM; ++j)
	//	context.write("%s ", jointDescSeq[j] ? "ON" : "OFF");	
	//context.write(">\n");
//	disableHandPlanning();
	context.debug("RagGraphPlanner::create: %s\n", spam::plannerDebug(*this).c_str());
	return true;
}

//------------------------------------------------------------------------------

FTDrivenHeuristic* RagGraphPlanner::getFTDrivenHeuristic() const {
	 return dynamic_cast<FTDrivenHeuristic*>(pHeuristic.get()); 
}

void RagGraphPlanner::disableHandPlanning() {
	Chainspace::Index hand = stateInfo.getChains().begin() + 1;
	for (Configspace::Index j = stateInfo.getJoints(hand).begin(); j < stateInfo.getJoints().end(); ++j) {
		pHeuristic->getJointDesc()[j]->enabled = false;	
		pLocalPathFinder->getHeuristic().getJointDesc()[j]->enabled = false;
	}
	// debug printing
	//context.write("JOINT DESC (dis): <");
	//for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
	//	context.write("%s ", pHeuristic->getJointDesc()[j]->enabled ? "ON" : "OFF");	
	//context.write(">\n");
}

void RagGraphPlanner::enableHandPlanning() {
	size_t i = 0;
	for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j, ++i) {
		pHeuristic->getJointDesc()[j]->enabled = jointDescSeq[i];
		pLocalPathFinder->getHeuristic().getJointDesc()[j]->enabled = jointDescSeq[i];
	}
	// debug printing
	//context.write("JOINT DESC (ena): <");
	//for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
	//	context.write("%s ", pHeuristic->getJointDesc()[j]->enabled ? "ON" : "OFF");	
	//context.write(">\n");
	
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::localFind(const ConfigspaceCoord &begin, const ConfigspaceCoord &end, Waypoint::Seq &localPath) {
	enableHandPlanning();
	context.debug("localFind: %s\n", plannerDebug(*this).c_str());
//	context.write("RagGraphPlanner::localFind: %s\n", plannerConfigspaceDebug(*this).c_str());
//	context.write("RagGraphPlanner::localFind: %s\n", grasp::plannerWorkspaceDebug(*this).c_str());

//	context.verbose("Rag::LocalFind: %s\n", grasp::plannerDebug(*this).c_str());
	Real scale = REAL_ONE;
	for (U32 i = 0; i < pathFinderDesc.numOfIterations; ++i) {
		// scale maximum distance between waypoints
		scale *= pathFinderDesc.distScaleFac;
		pHeuristic->setScale(scale);

		// set graph generators delta
		ConfigspaceCoord delta;
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
			delta[j] = pathFinderDesc.rangeFac*pHeuristic->getJointDesc()[j]->distMax;

		// create online graph generators
		WaypointGenerator::Desc::Seq generators;
		for (Waypoint::Seq::const_iterator j = localPath.begin(); j != localPath.end(); ++j) {
			WaypointGenerator::Desc::Ptr generator(new WaypointGenerator::Desc);
			generator->name = "local";
			generator->delta = delta;
			generator->mean = j->cpos;
			generator->seed = j == localPath.begin() ? WaypointGenerator::SEED_ROOT : j == --localPath.end() ? WaypointGenerator::SEED_GOAL : WaypointGenerator::SEED_USER;
			generators.push_back(generator);
		}

		// allocate and generate local graph
		pLocalPathFinder->allocateGraph((U32)(scale*localPath.size()*pLocalPathFinder->getOnlineGraphSize()), pLocalPathFinder->getOnlineGraphSize(), pLocalPathFinder->getGraphNeighbours());
		pLocalPathFinder->generateOnlineGraph(begin, end, &generators);

		// find node path on local graph
		localPath.clear();
		if (!pLocalPathFinder->findPath(end, localPath, localPath.begin()))
			return false;
	}
	//	disableHandPlanning();
	return true;
}

//#define SA_PERFMON

//bool GraphPlanner::optimize(const GenConfigspaceState &begin, const GenConfigspaceState &end, Waypoint::Seq &optimisedPath) const {
//	// purge the path
//	if (!purge(optimisedPath))
//		return false;
//	
//	// setup cost function
//	PARAMETER_GUARD(Heuristic, Heuristic::CostDesc, CostDesc, *pHeuristic);
//	Heuristic::CostDesc costDesc = pHeuristic->getCostDesc();
//	costDesc.distGoalFac = REAL_ZERO; // disable goal cost
//	pHeuristic->setCostDesc(costDesc);
//
//#ifdef SA_PERFMON
//	U32 n = 0, SA[2] = {0};
//	static U32 nn = 0, id = 0;
//	PerfTimer timer;
//	SecTmReal t;
//	static SecTmReal tt = SEC_TM_REAL_ZERO;
//#endif //SA_PERFMON
//
//	const U32 numOfIterations = optimisedPath.size() <= 2 ? 0 : (U32)optimisedPath.size()*pathOptimisationDesc.numOfIterations;
//	for (U32 s = 0; s < numOfIterations; ++s) {
//		// select random waypoint
//		const U32 index = 1 + rand.next()%((U32)optimisedPath.size() - 2);
//		// neighbouring waypoints
//		Waypoint* const curr = &optimisedPath[index];
//		Waypoint* const prev[2] = {curr - 1, &optimisedPath.front() < curr - 1 ? curr - 2 : NULL};
//		Waypoint* const next[2] = {curr + 1, &optimisedPath. back() > curr + 1 ? curr + 2 : NULL}; 		
//		// Backup existing waypoint
//		Waypoint tmp = *curr;
//
//		// temperature cooling schedule
//		const Real T = pathOptimisationDesc.Tfinal + (pathOptimisationDesc.Tinit - pathOptimisationDesc.Tfinal)*(numOfIterations - s)/numOfIterations;
//		
//		// generate a candidate solution
//		const idx_t crossPoint = rand.next()%stateInfo.getJoints().size();
//		for (idx_t j = 0; j < stateInfo.getJoints().size(); ++j) {
//			const Configspace::Index i = stateInfo.getJoints().begin() + (crossPoint + j)%stateInfo.getJoints().size();
//			
//			if (pHeuristic->getJointDesc()[i]->enabled) {
//				// local variation range 
//				const Real range = Real(2.0)*(Math::abs(curr->cpos[i] - prev[0]->cpos[i]) + Math::abs(curr->cpos[i] - next[0]->cpos[i])) +
//					(prev[1] != NULL ? Real(1.0)*Math::abs(prev[0]->cpos[i] - prev[1]->cpos[i]) : REAL_ZERO) +
//					(next[1] != NULL ? Real(1.0)*Math::abs(next[0]->cpos[i] - next[1]->cpos[i]) : REAL_ZERO);
//
//				// generate new coordinate
//				curr->cpos[i] = Math::clamp(curr->cpos[i] + range*rand.nextUniform(-T, +T), controller.getMin().cpos[i], controller.getMax().cpos[i]);
//				
//				if (pathOptimisationDesc.crossProb < rand.nextUniform<Real>())
//					break;
//			}
//		}
//
//		// setup waypoint
//		curr->setup(controller, true, true);
//
//		{
//			// modification of a single waypoint on path affects neighbours as well
//			const Real cCurr = cost(index, optimisedPath);
//			if (cCurr >= Node::COST_INF)
//				goto REDO;
//			const Real cPrev = cost(index - 1, optimisedPath);
//			if (cPrev >= Node::COST_INF)
//				goto REDO;
//			const Real cNext = cost(index + 1, optimisedPath);
//			if (cNext >= Node::COST_INF)
//				goto REDO;
//
//			// new cost
//			const Real cNew = cPrev + cCurr + cNext;
//			// old cost
//			const Real cOld = prev[0]->cost + curr->cost + next[0]->cost;
//		
//			// SA parameters
//			const Real dE = pathOptimisationDesc.Enorm*(cNew - cOld);
//			const bool b1 = dE < REAL_ZERO;
//			const bool b2 = !b1 && Math::exp(-dE) > rand.nextUniform<Real>();
//#ifdef SA_PERFMON
//			if (b1) ++SA[0];
//			if (b2) ++SA[1];
//#endif //SA_PERFMON
//			// update old cost if the new cost is lower and there are no collisions
//			if ((b1 || b2) && !collides(index, optimisedPath)) {
//				curr->cost = cCurr;
//				prev[0]->cost = cPrev;
//				next[0]->cost = cNext;
//				// purge the path
//				if (!purge(optimisedPath))
//					return false;
//#ifdef SA_PERFMON
//				++n;
//#endif //SA_PERFMON
//				continue;
//			}
//		}
//
//		REDO:
//		*curr = tmp;
//	}
//
//#ifdef SA_PERFMON
//	nn += n;
//	t = timer.elapsed();
//	tt+=t;
//
//	Real c = REAL_ZERO;
//	for (Waypoint::Seq::const_iterator i = optimisedPath.begin(); i != optimisedPath.end(); ++i) c += i->cost;
//	static Real cc = REAL_ZERO;
//	cc+=c;
//	context.debug("#%d (ni=%d, ti=%.3f, tf=%.3f, cp=%.3f): l=%d, t=%.3f(%.3f), n=%d(%d), c=%.4f(%.4f), SA=(%d, %d)\n",
//		++id, pathOptimisationDesc.numOfIterations, pathOptimisationDesc.Tinit, pathOptimisationDesc.Tfinal, pathOptimisationDesc.crossProb,
//		optimisedPath.size(), t, tt, n, nn, c, cc, SA[0], SA[1]
//	);
//#endif //SA_PERFMON
//
//	return true;
//}

//------------------------------------------------------------------------------

bool RagGraphPlanner::findTarget(const golem::GenConfigspaceState &begin, const GenWorkspaceChainState& wend, GenConfigspaceState &cend) {
//	context.write("RagGraphPlanner::find target\n");
//	return GraphPlanner::findTarget(begin, wend, cend);
	bool enable = false;
	spam::FTDrivenHeuristic *heuristic = getFTDrivenHeuristic();
	if (heuristic) {
		enable = heuristic->enableUnc;
		heuristic->enableUnc = false;
		context.debug("RagGraphPlanner::findTarget(): enable unc %s\n", heuristic->enableUnc ? "ON" : "OFF");
	}
	// TODO: Find why the pre-grasp pose returns with close fingers
	disableHandPlanning();
	context.debug("findTarget: %s\n", plannerDebug(*this).c_str());
	//context.write("RagGraphPlanner::findTarget: %s\n", grasp::plannerConfigspaceDebug(*this).c_str());
	//context.write("RagGraphPlanner::findTarget: %s\n", grasp::plannerWorkspaceDebug(*this).c_str());

#ifdef _HEURISTIC_PERFMON
	heuristic->resetLog();
	heuristic->getCollision()->resetLog();
#endif
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif

	getCallbackDataSync()->syncCollisionBounds();

	// generate graph
	pGlobalPathFinder->generateOnlineGraph(begin.cpos, wend.wpos);

	// create waypoints pointers
	const Waypoint::Seq& graph = pGlobalPathFinder->getGraph();
	WaypointPtr::Seq waypointPtrGraph;
	waypointPtrGraph.reserve(graph.size());
	for (U32 i = 0; i < graph.size(); ++i)
		waypointPtrGraph.push_back(WaypointPtr(&graph[i]));

	// Create waypoint population
	const U32 populationSize = std::min(U32(pKinematics->getDesc().populationSize), (U32)waypointPtrGraph.size());

	// sort waypoints (pointers) from the lowest to the highest cost
	std::partial_sort(waypointPtrGraph.begin(), waypointPtrGraph.begin() + populationSize, waypointPtrGraph.end(), WaypointPtr::cost_less());

	// create initial population for kinematics solver
	ConfigspaceCoord::Seq population;
	population.reserve(populationSize);
	for (WaypointPtr::Seq::const_iterator i = waypointPtrGraph.begin(); population.size() < populationSize && i != waypointPtrGraph.end(); ++i)
		population.push_back((*i)->cpos);
	pKinematics->setPopulation(&population);

	GenConfigspaceState root;
	root.setToDefault(controller.getStateInfo().getJoints().begin(), controller.getStateInfo().getJoints().end());
	root.cpos = graph[Node::IDX_ROOT].cpos;

	// find the goal state
	if (!pKinematics->findGoal(root, wend, cend)) {
		printf("failure\n");
		context.error("GraphPlanner::findTarget(): unable to find target\n");
		return false;
	}

	cend.t = wend.t;
	cend.cvel.fill(REAL_ZERO);
	cend.cacc.fill(REAL_ZERO);

#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GraphPlanner::findTarget(): time_elapsed = %f [sec]\n", t.elapsed()
		);
#endif
#ifdef _HEURISTIC_PERFMON
	heuristic->writeLog(context, "GraphPlanner::findTarget()");
	heuristic->getCollision()->writeLog(context, "GraphPlanner::findTarget()");;
#endif

	if (heuristic)
		heuristic->enableUnc = enable;

	enableHandPlanning();

//	context.write("RagGraphPlanner::findTarget(): done.\n");
	return true;
}

//------------------------------------------------------------------------------

//bool RagGraphPlanner::generateWaypoints(const Controller::Trajectory &trajectory, golem::WaypointGenerator::Seq &generators, golem::U32 steps) const {
//	if (trajectory.size() < 2)
//		return false;
//
//	//ConfigspaceCoord delta;
//	//for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j) 
//	//	delta[j] = pathFinderDesc.rangeFac*pHeuristic->getJointDesc()[j]->distMax*0.8;
//	for (Controller::State::Seq::const_iterator w = trajectory.begin(); w != trajectory.end(); ++w) {
//		Waypoint wi;
//		wi.cpos = w->cpos;
//		wi.setup(controller, false, true); 
//		if (!getFTDrivenHeuristic()->collides(wi))
//			generators.push_back(WaypointGenerator(w->cpos, pLocalPathFinder->getGeneratorRoot().delta));
//
//	}
//	//for (Controller::State::Seq::const_iterator w0 = trajectory.begin(), w1 = ++trajectory.begin(); w1 != trajectory.end(); ++w0, ++w1) {
//	//	// lineary interpolate coordinates
//	//	generators.push_back(WaypointGenerator(w0->cpos, pLocalPathFinder->getGeneratorRoot().delta));
//	//	for (U32 i = 1; i < steps; ++i) {
//	//		Waypoint w;
//	//		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j) 
//	//			w.cpos[j] = w1->cpos[j] - (w1->cpos[j] - w0->cpos[j])*Real(i) / Real(steps);
//
//	//		// skip reference pose computation
//	//		w.setup(controller, false, true);
//
//	//		if (!getFTDrivenHeuristic()->collides(w))
//	//			generators.push_back(WaypointGenerator(w.cpos, pLocalPathFinder->getGeneratorRoot().delta));
//	//	}
//
//	//}
////	generators.push_back(WaypointGenerator(trajectory.back().cpos, pLocalPathFinder->getGeneratorRoot().delta));
//	return true;
//}


bool RagGraphPlanner::findGlobalTrajectory(const golem::Controller::State &begin, const golem::Controller::State &end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, const GenWorkspaceChainState* wend) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	FTDrivenHeuristic::resetLog();
	Collision::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
#endif

	getCallbackDataSync()->syncCollisionBounds();

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	// generate global graph only for the arm
	context.debug("GraphPlanner::findGlobalTrajectory(): Enabled Uncertainty %s. disable hand planning...\n", getFTDrivenHeuristic()->enableUnc ? "ON" : "OFF");
	disableHandPlanning();
	context.debug("findGlobalTrajectory(): %s\n", plannerDebug(*this).c_str());
	//context.write("GraphPlanner::findGlobalTrajectory(): %s\n", grasp::plannerConfigspaceDebug(*this).c_str());
	//context.write("GraphPlanner::findGlobalTrajectory(): %s\n", grasp::plannerWorkspaceDebug(*this).c_str());

	// generate global graph
	pGlobalPathFinder->generateOnlineGraph(begin.cpos, end.cpos);
	// find node path on global graph
	globalPath.clear();
	if (!pGlobalPathFinder->findPath(end.cpos, globalPath, globalPath.begin())) {
		context.error("GlobalPathFinder::findPath(): unable to find global path\n");
		return false;
	}
#ifdef _GRAPHPLANNER_PERFMON
	context.write(
		"GlobalPathFinder::findPath(): time_elapsed = %f [sec], len = %d\n",
		t.elapsed(), globalPath.size()
		);
#endif

	if (pLocalPathFinder != NULL) {
#ifdef _GRAPHPLANNER_PERFMON
		t.reset();
#endif
		PARAMETER_GUARD(Heuristic, Real, Scale, *pHeuristic);

		for (U32 i = 0;;) {
			localPath = globalPath;
			if (localFind(begin.cpos, end.cpos, localPath))
				break;
			else if (++i > pathFinderDesc.numOfTrials) {
				context.error("LocalPathFinder::findPath(): unable to find local path\n");
				return false;
			}
		}
#ifdef _GRAPHPLANNER_PERFMON
		context.write(
			"LocalPathFinder::findPath(): time_elapsed = %f [sec], len = %d\n",
			t.elapsed(), localPath.size()
			);
#endif
		// copy localPath
		optimisedPath = localPath;
	}
	else {
		// copy globalPath
		optimisedPath = globalPath;
	}

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
//	context.debug("Enabled Uncertainty %s\n", getFTDrivenHeuristic()->enableUnc ? "ON" : "OFF");
	FTDrivenHeuristic::writeLog(context, "PathFinder::find()");
	Collision::writeLog(context, "PathFinder::find()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "PathFinder::find()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	FTDrivenHeuristic::resetLog();
	Collision::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
	t.reset();
#endif
	optimize(optimisedPath);
#ifdef _GRAPHPLANNER_PERFMON
	context.write(
		"GraphPlanner::optimize(): time_elapsed = %f [sec], len = %d\n", t.elapsed(), optimisedPath.size()
		);
#ifdef _HEURISTIC_PERFMON
	FTDrivenHeuristic::writeLog(context, "GraphPlanner::optimize()");
	Collision::writeLog(context, "GraphPlanner::optimize()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "GraphPlanner::optimize()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	Controller::Trajectory::iterator iend = iter;
	pProfile->create(optimisedPath.begin(), optimisedPath.end(), begin, end, trajectory, iter, iend);
	pProfile->profile(trajectory, iter, iend);
#ifdef _GRAPHPLANNER_PERFMON
	context.write(
		"GraphPlanner::profile(): time_elapsed = %f [sec], len = %d\n",
		t.elapsed(), trajectory.size()
		);
#endif

	getCallbackDataSync()->syncFindTrajectory(trajectory.begin(), trajectory.end(), wend);
//	enableHandPlanning();

	return true;
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::findLocalTrajectory(const Controller::State &cbegin, GenWorkspaceChainState::Seq::const_iterator wbegin, GenWorkspaceChainState::Seq::const_iterator wend, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, MSecTmU32 timeOut) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#ifdef _HEURISTIC_PERFMON
	FTDrivenHeuristic::resetLog();
	Collision::resetLog();
#endif
#endif
	context.debug("findLocalTrajectory(): %s\n", plannerDebug(*this).c_str());
	//context.write("RagGraphPlanner::findLocalTrajectory: %s\n", grasp::plannerConfigspaceDebug(*this).c_str());
	//context.write("RagGraphPlanner::findLocalTrajectory: %s\n", grasp::plannerWorkspaceDebug(*this).c_str());

	spam::FTDrivenHeuristic *heuristic = getFTDrivenHeuristic();
	// trajectory size
	const size_t size = 1 + (size_t)(wend - wbegin);
	// check initial size
	if (size < 2) {
		context.error("GraphPlanner::findLocalTrajectory(): Invalid workspace sequence size\n");
		return false;
	}
	// time out
	const MSecTmU32 segTimeOut = timeOut == MSEC_TM_U32_INF ? MSEC_TM_U32_INF : timeOut / MSecTmU32(size - 1);
	// fill trajectory with cbegin
	const Controller::State cinit = cbegin; // backup
	Controller::Trajectory::iterator end = ++trajectory.insert(iter, cinit);
	for (GenWorkspaceChainState::Seq::const_iterator i = wbegin; i != wend; ++i)
		end = ++trajectory.insert(end, cinit);
	Controller::Trajectory::iterator begin = end - size;

	getCallbackDataSync()->syncCollisionBounds();
	optimisedPath.resize(size - 1);
	pKinematics->setPopulation();

	// find configspace trajectory
	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Min, *pHeuristic);
	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Max, *pHeuristic);
	for (size_t i = 1; i < size; ++i) {
		// pointers
		const Controller::Trajectory::iterator c[2] = { begin + i - 1, begin + i };
		const GenWorkspaceChainState::Seq::const_iterator w = wbegin + i - 1;

		// setup search limits
		GenCoordConfigspace min = pHeuristic->getMin();
		GenCoordConfigspace max = pHeuristic->getMin();
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j) {
			const idx_t k = j - stateInfo.getJoints().begin();
			min[j].pos = c[0]->cpos[j] - localFinderDesc.range[k];
			max[j].pos = c[0]->cpos[j] + localFinderDesc.range[k];
		}
		pHeuristic->setMin(min);
		pHeuristic->setMax(max);

		// and search for a solution
		if (!pKinematics->findGoal(*c[0], *w, *c[1], segTimeOut)) {
			context.error("GraphPlanner::findLocalTrajectory(): unable to solve inverse kinematics\n");
			return false;
		}

		// visualisation
		optimisedPath[i - 1].cpos = c[1]->cpos;
		optimisedPath[i - 1].wpos = w->wpos;
	}

	// profile configspace trajectory
	pProfile->profile(trajectory, begin, end);

	getCallbackDataSync()->syncFindTrajectory(begin, end, &*(wend - 1));

#ifdef _GRAPHPLANNER_PERFMON
	context.write("GraphPlanner::findLocalTrajectory(): time_elapsed = %f [sec], len = %d\n", t.elapsed(), size);
#ifdef _HEURISTIC_PERFMON
	if (heuristic) {
		context.write("Enabled Uncertainty %s\n", heuristic->enableUnc ? "ON" : "OFF");
		heuristic->writeLog(context, "GraphPlanner::findTarget()");
		heuristic->getCollision()->writeLog(context, "GraphPlanner::findTarget()");;
	}
#endif
#endif

	return true;
}

//------------------------------------------------------------------------------

void spam::XMLData(RagGraphPlanner::Desc &val, golem::XMLContext* context, bool create) {
	golem::XMLData((golem::GraphPlanner::Desc&)val, context, create);
}

//void spam::XMLData(RagPathFinder::Desc &val, golem::XMLContext* context, bool create) {
//	golem::XMLData((golem::GraphPlanner::Desc&)val, context, create);
//}
//
//void spam::XMLData(RagGraphPlanner::RagPathFinderDesc &val, golem::XMLContext* context, bool create) {
//	golem::XMLData((golem::GraphPlanner::PathFinderDesc&)val, context, create);
//}

//------------------------------------------------------------------------------