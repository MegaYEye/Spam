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
#include <Spam/Spam/Data.h>

//------------------------------------------------------------------------------

using namespace spam;
using namespace golem;

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
	context.write("RagGraphPlanner::create()\n");
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
	disableHandPlanning();

	
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
	context.verbose("Rag::LocalFind: %s\n", grasp::plannerDebug(*this).c_str());
	Real scale = REAL_ONE;
	for (U32 i = 0; i < pathFinderDesc.numOfIterations; ++i) {
		// scale maximum distance between waypoints
		scale *= pathFinderDesc.distScaleFac;
		pHeuristic->setScale(scale);

		// set graph generators delta
		ConfigspaceCoord delta;
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
			delta[j] = pathFinderDesc.rangeFac*pHeuristic->getJointDesc()[j]->distMax;
		
		// setup graph generators
		pLocalPathFinder->getGeneratorRoot().delta = pLocalPathFinder->getGeneratorGoal().delta = delta;
		pLocalPathFinder->getGenerators().clear();
		std::stringstream str;
		str << "global path\n";
		for (Waypoint::Seq::const_iterator j = localPath.begin(); j != localPath.end(); ++j) {
			if (j != localPath.begin() && j != localPath.end()) // do not double root and goal nodes
				pLocalPathFinder->getGenerators().push_back(WaypointGenerator(j->cpos, delta));
			str << "<";
			for (Configspace::Index i = stateInfo.getJoints().begin(); i != stateInfo.getJoints().end(); ++i)
				str << j->cpos[i] << " ";
			str << ">\n";
		}
		//context.write("ragGraphPlanner::local path\n");
		//for (Waypoint::Seq::const_iterator j = localPath.begin(); j != localPath.end(); ++j) {
		//	context.write("<");
		//	for (Configspace::Index i = controller.getStateInfo().getJoints().begin(); i != controller.getStateInfo().getJoints().end(); ++i)
		//		context.write("%f ", j->cpos[i]);
		//	context.write("\n");
		//}

		//Waypoint::Seq::const_iterator j = localPath.size() > 2 ? localPath.end() - 2 : localPath.begin();
		//for (WaypointGenerator::Seq::iterator g = pLocalPathFinder->getGenerators().begin(); g != pLocalPathFinder->getGenerators().end(); ++g) {
		//	if (j != localPath.begin()) { // do not double root and goal nodes
		//		g->mean = j->cpos;
		//		j--;
		//	}
		//}
		//context.write("ragGraphPlanner::generators\n");
		//for (WaypointGenerator::Seq::iterator j = pLocalPathFinder->getGenerators().begin(); j != pLocalPathFinder->getGenerators().end(); ++j) {
		//	context.write("<");
		//	for (Configspace::Index i = controller.getStateInfo().getJoints().begin(); i != controller.getStateInfo().getJoints().end(); ++i)
		//		context.write("%f ", j->mean[i]);
		//	context.write("\n");
		//}

		// and generate local graph

		if (!pLocalPathFinder->generateGraph(begin, end)) {
			context.error("GraphPlanner::localFind(): unable to generate graph\n");
			return false;
		}

		// find node path on local graph
		localPath.clear();
		if (!pLocalPathFinder->findPath(end, localPath, localPath.begin())) {
			context.error("GraphPlanner::localFind(): unable to find path\n");
			return false;
		}
		str << "local path\n";
		for (Waypoint::Seq::const_iterator j = localPath.begin(); j != localPath.end(); ++j) {
			str << "<";
			for (Configspace::Index i = stateInfo.getJoints().begin(); i != stateInfo.getJoints().end(); ++i)
				str << j->cpos[i] << " ";
			str << ">\n";
		}
//		std::printf("%s\n", str.str().c_str());
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
	context.verbose("RagGraphPlanner::findTarget: %s\n", grasp::plannerDebug(*this).c_str());

	bool enable = false;
	spam::FTDrivenHeuristic *heuristic = getFTDrivenHeuristic();
	if (heuristic) {
		enable = heuristic->enableUnc;
		heuristic->enableUnc = false;
		context.write("RagGraphPlanner::findTarget(): enable unc %s\n", heuristic->enableUnc ? "ON" : "OFF");
	}

#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif
	getCallbackDataSync()->syncCollisionBounds();

#ifdef _GRAPHPLANNER_PERFMON
	t.reset();
#endif
	// generate graph
//	disableHandPlanning();
//	context.write("RagGraphPlanner::findTarget(): generating graph...\n");
	if (!pGlobalPathFinder->generateGraph(begin.cpos, wend.wpos)) {
		context.error("RagGraphPlanner::findTarget(): unable to generate global graph\n");
		return false;
	}
//	context.write("done.\n");

	// find target
//	context.write("RagGraphPlanner::findTarget(): seeking for a target...\n");
	if (!pGlobalPathFinder->findGoal(cend.cpos)) {
		context.error("RagGraphPlanner::findTarget(): unable to find global path\n");
		return false;
	}
//	context.write("done.\n");


	cend.t = wend.t;
	cend.cvel.fill(REAL_ZERO);
	cend.cacc.fill(REAL_ZERO);

#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GlobalPathFinder::findTarget(): time elapsed = %f [sec], len = %d\n",
		t.elapsed(), globalPath.size()
	);
#endif
	if (heuristic)
		heuristic->enableUnc = enable;

	context.write("RagGraphPlanner::findTarget(): done.\n");
	return true;
}

//------------------------------------------------------------------------------

bool RagGraphPlanner::findGlobalTrajectory(const golem::Controller::State &begin, const golem::Controller::State &end, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, const GenWorkspaceChainState* wend) {
#ifdef _GRAPHPLANNER_PERFMON
	PerfTimer t;
#endif
	
#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::resetLog();
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
//	context.write("RagGraphPlanner::findGlobalTrajectory(): disable hand planning...\n");
	disableHandPlanning();
//	context.write("RagGraphPlanner::findGlobalTrajectory(): generating graph...\n");
	if (!pGlobalPathFinder->generateGraph(begin.cpos, end.cpos)) {
		context.error("GraphPlanner::findTrajectory(): unable to generate global graph\n");
		return false;
	}
//	context.write("done.\n");

	// find node path on global graph
	globalPath.clear();
//	context.write("RagGraphPlanner::findGlobalTrajectory(): seeking for a path...\n");
	if (!pGlobalPathFinder->findPath(end.cpos, globalPath, globalPath.begin())) {
		context.error("GraphPlanner::findTrajectory(): unable to find global path\n");
		return false;
	}
//	context.write("done.\n");
#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GlobalPathFinder::findPath(): time elapsed = %f [sec], len = %d\n",
		t.elapsed(), globalPath.size()
	);
#endif

	if (pLocalPathFinder != NULL) {
#ifdef _GRAPHPLANNER_PERFMON
		t.reset();
#endif
		PARAMETER_GUARD(Heuristic, Real, Scale, *pHeuristic);

		localPath = globalPath;
//		context.write("RagGraphPlanner::findGlobalTrajectory(): seeking for local path...\n");
		if (!localFind(begin.cpos, end.cpos, localPath))
			return false;
//		context.write("done...\n");
#ifdef _GRAPHPLANNER_PERFMON
		context.debug(
			"GraphPlanner::localFind(): time elapsed = %f [sec], len = %d\n",
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
	Heuristic::writeLog(context, "PathFinder::find()");
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::writeLog(context, "PathFinder::find()");
#endif
#endif

#ifdef _GRAPHPLANNER_PERFMON
#ifdef _HEURISTIC_PERFMON
	Heuristic::resetLog();
#endif
#ifdef _BOUNDS_PERFMON
	Bounds::resetLog();
#endif
	t.reset();
#endif
	if (!optimize(begin, end, optimisedPath))
		return false;
#ifdef _GRAPHPLANNER_PERFMON
	context.debug(
		"GraphPlanner::optimize(): time elapsed = %f [sec], len = %d\n", t.elapsed(), optimisedPath.size()
	);
#ifdef _HEURISTIC_PERFMON
	Heuristic::writeLog(context, "GraphPlanner::optimize()");
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
	context.debug(
		"GraphPlanner::profile(): time elapsed = %f [sec], len = %d\n",
		t.elapsed(), trajectory.size()
	);
#endif

	getCallbackDataSync()->syncFindTrajectory(trajectory.begin(), trajectory.end(), wend);
	enableHandPlanning();
	
	context.write("RagGraphPlanner::findGlobalTrajectory(): done.\n");
	return true;
}

//------------------------------------------------------------------------------

//bool GraphPlanner::findLocalTrajectory(const Controller::State &cbegin, GenWorkspaceChainState::Seq::const_iterator wbegin, GenWorkspaceChainState::Seq::const_iterator wend, Controller::Trajectory &trajectory, Controller::Trajectory::iterator iter, MSecTmU32 timeOut) {
//#ifdef _GRAPHPLANNER_PERFMON
//	PerfTimer t;
//#endif
//
//	// trajectory size
//	const size_t size = 1 + (size_t)(wend - wbegin);
//	// check initial size
//	if (size < 2) {
//		context.error("GraphPlanner::findLocalTrajectory(): Invalid workspace sequence size\n");
//		return false;
//	}
//	// time out
//	const MSecTmU32 segTimeOut = timeOut == MSEC_TM_U32_INF ? MSEC_TM_U32_INF : timeOut/(size - 1);
//	// fill trajectory with cbegin
//	const Controller::State cinit = cbegin; // backup
//	Controller::Trajectory::iterator end = ++trajectory.insert(iter, cinit);
//	for (GenWorkspaceChainState::Seq::const_iterator i = wbegin; i != wend; ++i)
//		end = ++trajectory.insert(end, cinit);
//	Controller::Trajectory::iterator begin = end - size;
//
//	getCallbackDataSync()->syncCollisionBounds();
//	optimisedPath.resize(size - 1);
//
//	// find configspace trajectory
//	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Min, *pHeuristic);
//	PARAMETER_GUARD(Heuristic, GenCoordConfigspace, Max, *pHeuristic);
//	for (size_t i = 1; i < size; ++i) {
//		// pointers
//		const Controller::Trajectory::iterator c[2] = {begin + i - 1, begin + i};
//		const GenWorkspaceChainState::Seq::const_iterator w = wbegin + i - 1;
//		
//		// setup search limits
//		GenCoordConfigspace min = pHeuristic->getMin();
//		GenCoordConfigspace max = pHeuristic->getMin();
//		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j) {
//			const idx_t k = j - stateInfo.getJoints().begin();
//			min[j].pos = c[0]->cpos[j] - localFinderDesc.range[k];
//			max[j].pos = c[0]->cpos[j] + localFinderDesc.range[k];
//		}
//		pHeuristic->setMin(min);
//		pHeuristic->setMax(max);
//		
//		// and search for a solution
//		if (!pKinematics->findGoal(*c[0], *w, *c[1], segTimeOut)) {
//			context.error("GraphPlanner::findLocalTrajectory(): unable to solve inverse kinematics\n");
//			return false;
//		}
//		
//		// visualisation
//		optimisedPath[i - 1].cpos = c[1]->cpos;
//		optimisedPath[i - 1].wpos = w->wpos;
//	}
//
//	// profile configspace trajectory
//	pProfile->profile(trajectory, begin, end);
//
//	getCallbackDataSync()->syncFindTrajectory(begin, end, &*(wend - 1));
//
//#ifdef _GRAPHPLANNER_PERFMON
//	context.debug("GraphPlanner::findLocalTrajectory(): time elapsed = %f [sec], len = %d\n", t.elapsed(), size);
//#endif
//	
//	return true;
//}

//------------------------------------------------------------------------------