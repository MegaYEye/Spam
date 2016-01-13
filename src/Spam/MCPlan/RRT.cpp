#include <Spam/MCPlan/RRT.h>
#include <Spam/MCPlan/Defs.h>

//-------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//-------------------------------------------------------------------------

const Real Node::COST_ZERO = REAL_ZERO;
const Real Node::COST_INF = REAL_MAX;// numeric_const<Real>::MAX;

//-------------------------------------------------------------------------

RRT::RRT(const golem::Context& context) : IncrementalPlanner(context) {
}

RRT::~RRT() {}

bool RRT::create(const Desc &desc) {
	std::printf("RRT::create()\n");
	if (!desc.isValid())
		return false;

	IncrementalPlanner::create(desc);
	
	bestState = initialState;

	return true;
}

void RRT::reset(void){
	IncrementalPlanner::reset();

	numNodes = 10000;

	/* TODO  check if the goal pose is known from the beginning for testing purposes */
	satisfiedCount = 0;
//	goalDist = pModel->metric(initialState.p, goalState.p);
//	goalAngularDist = pModel->metric(initialState.R, goalState.R);
	bestState = initialState;
}

void RRT::reset(Vec3& newInitState){
	// TODO set the new state as initial state 
	IncrementalPlanner::reset();
}

//-------------------------------------------------------------------------

Node3d* RRT::selectNode(const Vec3& x, Tree3d* t, bool forward) const {
	Real d = REAL_ZERO, d_min = REAL_MAX;
	Node3d* node;

	const Node3d::SeqPtr nl = t->getNodes();
	for (Node3d::SeqPtr::const_iterator n = nl.begin(); n != nl.end(); ++n) {
		d = forward ? modelPtr->cost((*n)->data, x) : modelPtr->cost(x, (*n)->data);
		if (d < d_min) {
			d_min = d;
			node = *n;
		}
	} 

	
	return node;// t->findNode(node);
}

Vec3 RRT::selectInput(const Vec3& xNear, const Vec3& xRand) const {
	const Vec3 u_best = modelPtr->selectInput(xNear, xRand);
	return modelPtr->integrate(xNear, u_best, plannerDeltaT);
	//int satisfiedCount = 0;
	//Vec3 u_best, nx;
	//Vec3 inputs; //list<Vec3>::iterator u;
	//double d_min;
	//success = true;
	//d_min = (forward) ? modelPtr->cost(xNear, xRand) : modelPtr->cost(xRand, xNear);
}

Node3d* RRT::extend(const Vec3& x, Tree3d* t, bool forward = true) {
//	context.write("extend towards x [%f %f %f]\n", x.x, x.y, x.z);
	Node3d* n_best = selectNode(x, t, forward);
	const Vec3 next = selectInput(n_best->data, x);
	//context.write("Extend node[%d] [%f %f %f] to node[%d] [%f %f %f] (desired [%f %f %f])\n", 
	//	n_best->index, n_best->data.x, n_best->data.y, n_best->data.z,
	//	nn->index, nn->data.x, nn->data.y, nn->data.z,
	//	next.x, next.y, next.z);

	return t->extend(n_best, next);
}

bool RRT::connect(const Vec3 &x, Tree3d *t, Node3d *&nn, bool forward = true) {
//	Node3d *nn_prev, *n_best;
//	Vec3 nx, nx_prev;
//	Vec3 u_best;
	bool success = false;
//	double d, d_prev, clock;
//	int steps;
//  
//	n_best = selectNode(x,t,forward);
//	u_best = selectInput(n_best->getState(), x, nx, success, forward); 
//	steps = 0;
//			// nx gets next state
//	if (success) {   // If a collision-free input was found
//		d = 0.01;// pModel->metric(nx.p, x.p);
//		d_prev = d;
//		nx_prev = nx; // Initialize
//		nn = n_best;
//		clock = plannerDeltaT;
//		while (modelPtr->satisfied(nx) && (d <= d_prev)){
////			satisfiedCount++;
//			steps++; // Number of steps made in connecting
//			nx_prev = nx;
//			d_prev = d; nn_prev = nn;
//			// Uncomment line below to select best action each time
//			u_best = selectInput(nn->getState(), x, nx, success, forward); 
////			nx = P->integrate(nx_prev,u_best,plannerDeltaT);
//			d = .01;//pModel->metric(nx.p,x.p);
//			clock += plannerDeltaT;
//			// Uncomment the subsequent two lines to
//			//   make each intermediate node added
//			//nn = g.new_node(nx_prev); // Make a new node
//			//g.new_edge(nn_prev,nn,u_best);
//		}
//		nn = t->extend(n_best, nx_prev, u_best, steps*plannerDeltaT);
//	}
//
	return success;
}

bool RRT::plan()
{
	context.write("RRT::plan() - start <%.2f, %.2f, %.2f> goal <%.2f, %.2f, %.2f>...\n",
		initialState.x, initialState.y, initialState.z, goalState.x, goalState.y, goalState.z);
	int extendFail=0;
	Real d = REAL_ZERO, d_min = REAL_MAX, time = REAL_ZERO;

	Node3d::SeqPtr path;

	// Keep track of time
	float t = claudio_used_time();

	// Make the root node of G
	if (!pRRTree)
		pRRTree = new Tree3d(initialState);

	Node3d *n = pRRTree->getRoot();
	Node3d *nGoal = pRRTree->getRoot();

	size_t i = 0;
	goalDist = modelPtr->cost(n->data, goalState);
	context.write("RRT::plan() - Node[%d] goal distance %.f\n", n->index, goalDist);
	while ((i < numNodes) && (!gapSatisfied(n->data, goalState))) {
		if ((n = extend(getNextState(n->data), pRRTree)) != nullptr) {
			n->cost = modelPtr->cost(n->data, goalState);
			if (n->cost < d_min) {
				d_min = n->cost;
				nGoal = n;
			}
//			context.write("RRT::plan(): %d nodes, distance=%f\n", pRRTree->getNodes().size(), d_min);
		}
		i++;
	}

	double cumulativePlanningTime = ((double)claudio_used_time(t));
	context.write("RRT::plan(): time %.2f %d nodes\n", cumulativePlanningTime, pRRTree->getNodes().size());

	// Get the solution path
	if (gapSatisfied(nGoal->data, goalState)) {
		path = pRRTree->pathToRoot(nGoal);
		recordSolution(path); // Write to Path and Policy
		context.write("RRT::plan() - success.\nbye.\n");
		return true;
	}
	context.write("RRT::plan() - failure.\nbye.\n");
	return false;
}

//-------------------------------------------------------------------------

//void spinta::XMLData(RRT::Desc &val, Context *context, XMLContext* xmlcontext, bool create) {
//	ASSERT(xmlcontext)
//	
//	xmlcontext = xmlcontext->getContextFirst("rrt_planner");
//
//	XMLData("num_nodes", val.numNodes, xmlcontext, create);
//	XMLData("num_max_fails", val.nMaxFails, xmlcontext, create);
//	XMLData("bidirectional", val.bidirectional, xmlcontext, create);
////	XMLData("satisfied_count", val.satisfiedCount, context, create);
//
//	XMLData(val.linError, xmlcontext->getContextFirst("lin_error"), create);
//	XMLData(val.angError, xmlcontext->getContextFirst("ang_error"), create);
//
//	// does not load the push planner as default 
////	XMLData((ICubPushPlanner::Desc &)val, xmlcontext->getContextFirst("push_planner"), create);
//}

//-------------------------------------------------------------------------

AtlasRRT::AtlasRRT(const golem::Context& context) : RRT(context) {
}
	
AtlasRRT::~AtlasRRT() {}
	
bool AtlasRRT::create(const Desc &desc) {
	context.getMessageStream()->write("RRTGoalBias::create()\n");
	if (!desc.isValid())
		return false;
	
	gpPtr.reset();

	RRT::create(desc);

	desc.pModelDesc->nextState = [&](const Vec3& x) -> Vec3 {
		Vec3 v;
		v.next(rand);
		v.normalise();
		return x + v;
	};

	desc.pModelDesc->nextInput = [&](const Vec3& node, const Vec3& state) -> Vec3 {
		Vec3 v = state - node;
		v.normalise();
		return v;
	};

	modelPtr = desc.pModelDesc->create(context);
	
	return true;
}

//-------------------------------------------------------------------------

//RRTGoalBias::RRTGoalBias(const golem::Context& context) : RRT(context) {
//}
//
//RRTGoalBias::~RRTGoalBias() {}
//
//bool RRTGoalBias::create(const Desc &desc) {
//	context.getMessageStream()->write(Message::LEVEL_DEBUG, "RRTGoalBias::create()\n");
//	if (!desc.isValid())
//		return false;
//
//	goalProb = desc.goalProb;
//	RRT::create(desc);
//
//	return true;
//}
//
//Vec3 RRTGoalBias::chooseState()
//{
//  double rv;
//
//  rv = rand.nextUniform(0.0, 1.0);
//  if (rv > goalProb)
//    return randomState();
//  else{
//	  std::printf("RRTGoalBias::chooseState() <%.2f, %.2f, %.2f>\n", goalState.p.x, goalState.p.y, goalState.p.z);
//	  return goalState;
//  }
//}

////-------------------------------------------------------------------------
//
//TwoLevelRRT::TwoLevelRRT(Model::Ptr model, Robot *robot) : RRTGoalBias(model, robot) {
//}
//
//TwoLevelRRT::~TwoLevelRRT() {}
//
//bool TwoLevelRRT::create(const Desc &desc) {
//	context->getMessageStream()->write(Message::LEVEL_DEBUG, "TwoLevelRRT::create()\n");
//	if (!desc.isValid())
//		return false;
//	
//	RRTGoalBias::create(desc);
//	pPushPlanner = desc.pPushPlannerDesc->create(pModel, pRobot);
//	if (pPushPlanner.get() == NULL)
//		throw MsgPlanner(Message::LEVEL_CRIT, "TwoLevelRRT::create(): Invalid smart pointer for the push planner");
//	
//	return true;
//}
//
//// Return the best new state in nx_best
//// success will be false if no action is collision free
//PushAction::Seq TwoLevelRRT::selectInput(const Vec3 &xNear, const Vec3 &xRand, 
//			   Vec3 &nx_best, bool &success,
//			   bool forward = true)
//{
//	context->getMessageStream()->write(Message::LEVEL_DEBUG, "TwoLevelRRT::selectInput()\n");
//	int satisfiedCount = 0;
//	Vec3 u_best, nx;
//	PushAction::Seq actions; //list<Vec3>::iterator u;
//	double d_min;
//	success = false;
//	d_min = (forward) ? pModel->metric(xNear.p,xRand.p) : pModel->metric(xRand.p,xNear.p);
//	
//	//if (pPushPlanner.get() == NULL)
//	//	throw MsgPlanner(Message::LEVEL_CRIT, "RRT::selectInput(): Invalid pointer");
//
//	actions = pPushPlanner->getInput(xNear, xRand, &nx_best, success);
//
//	//PushAction::Seq policy;
//	//policy.push_back(u);
//	return actions;
//}
//
//
////-------------------------------------------------------------------------
//
//void spinta::XMLData(TwoLevelRRT::Desc &val, Context *context, golem::XMLContext* xmlcontext, bool create) {
//	ASSERT(xmlcontext)
//
////	xmlcontext = xmlcontext->getContextFirst("rrt_planner");
//
//	XMLData("goal_prob", val.goalProb, xmlcontext, create);	
//	XMLData("num_nodes", val.numNodes, xmlcontext, create);
//	XMLData("num_max_fails", val.nMaxFails, xmlcontext, create);
//	XMLData("bidirectional", val.bidirectional, xmlcontext, create);
////	XMLData("satisfied_count", val.satisfiedCount, context, create);
//
//	XMLData(val.linError, xmlcontext->getContextFirst("lin_error"), create);
//	XMLData(val.angError, xmlcontext->getContextFirst("ang_error"), create);
//
//	XMLData(val.initialState, xmlcontext->getContextFirst("initial_state"), create);
//	XMLData(val.goalState, xmlcontext->getContextFirst("goal_state"), create);
//
//	std::printf("xmldata icub push planner::desc\n");
//	//ICubPushPlanner::Desc* pICubPushPlannerDesc(new ICubPushPlanner::Desc);
//	//(PushPlanner::Desc&)*pICubPushPlannerDesc = *val.pPushPlannerDesc;
//	//val.pPushPlannerDesc.reset(pICubPushPlannerDesc);
//	XMLData(*val.pPushPlannerDesc, context, xmlcontext, create);
//}

//-------------------------------------------------------------------------

//POMCRRT::POMCRRT(Model::Ptr model) : RRTGoalBias(model){
////	reset();
//}
//
//// Return the best new state in nx_best
//// success will be false if no action is collision free
//PushAction POMCRRT::selectInput(const Vec3 &xNear, const Vec3 &xRand, 
//			   Vec3 &nx_best, bool &success,
//			   bool forward = true)
//{
//	int satisfiedCount = 0;
//	Vec3 u_best, nx, nend;
//	PushAction u;
//	PushAction prova;
//	double d, d_min;
//	success = false;
//	d_min = (forward) ? pModel->metric(xNear.p, xRand.p) : pModel->metric(xRand.p, xNear.p);
//	u = pPushPlanner->getInput(xNear, xRand, &nx_best/*, success*/);
//	// the extention of the tree is made only if the object has been moved
////	if(!u.isNull())
////		success = true;
//
//	return u;
//}

//-------------------------------------------------------------------------
//
//#include <stdio.h>
//#include <GL/glut.h>

//-------------------------------------------------------------------------