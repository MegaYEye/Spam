#include <Spam/MCPlan/Planner.h>

//-------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//-------------------------------------------------------------------------

spam::Planner::Planner(const golem::Context& context) : rand(context.getRandSeed()), context(context) {
}

spam::Planner::~Planner() {}

bool spam::Planner::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

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

	error = desc.error;
	nMaxFails = desc.nMaxFails;
	numNodes = desc.numNodes;
	satisfiedCount = 0;
	bidirectional = desc.bidirectional;
	plannerDeltaT = desc.plannerDeltaT;
	
	pRRTree = nullptr;
//	pRRTree2 = NULL;

	initialState = desc.initialState;
	goalState = desc.goalState;

	return true;
}

void spam::Planner::reset() {
	if (pRRTree) delete pRRTree;
//	if (pRRTree2) delete pRRTree2;
	
	pRRTree = nullptr;
//	pRRTree2 = NULL;
}

//-------------------------------------------------------------------------

Vec3 spam::Planner::getNextState() {
	Vec3 v;
	v.next(rand);
	return v;
}

Vec3 spam::Planner::getNextState(const Vec3& x) {
	return modelPtr->getNextState(x);
}


bool spam::Planner::gapSatisfied(const Vec3& x1, const Vec3& x2) {
	if (x1.distanceSqr(x2) > error)
		return false;
	return true;
}

//-------------------------------------------------------------------------

IncrementalPlanner::IncrementalPlanner(const golem::Context& context) : spam::Planner(context) {
}

IncrementalPlanner::~IncrementalPlanner() {}

bool IncrementalPlanner::create(const Desc &desc) { 
	std::printf("IncrementalPlanner::create()\n");
	if (!desc.isValid())
		return false;

	return Planner::create(desc);
}

void IncrementalPlanner::recordSolution(const Node3d::SeqPtr& glist, const Node3d::SeqPtr& g2list) {
	//Node3d::SeqPtr::const_iterator nfirst, nlast;
	//double ptime;

	Node3d::SeqPtr path;

	SecTmReal ptime(.0); 

	for (Node3d::SeqPtr::const_iterator n = glist.begin(); n != glist.end(); ++n)
		path.push_back(*n);

	context.write("Solution:\n");
	for (Node3d::SeqPtr::const_iterator it = path.begin(); it != path.end(); it++)
		context.write("Node[%d] cost=%f state=[%.2f, %.2f, %.2f]\n", (*it)->index, (*it)->cost, (*it)->data.x, 
		(*it)->data.y, (*it)->data.z);
	std::printf("\n");
}

void IncrementalPlanner::recordSolution(const Node3d::SeqPtr& glist)
{
	Node3d::SeqPtr emptylist;
	emptylist.clear(); // Make sure it is clear

	recordSolution(glist, emptylist);
}

void IncrementalPlanner::writeGraphs(std::ofstream &fout)
{
  //if (pRRTree)
  //  fout << pRRTree << "\n\n\n";
  //if (pRRTree2)
  //  fout << pRRTree;
}



void IncrementalPlanner::readGraphs(std::ifstream &fin)
{
  if (pRRTree)
	  delete pRRTree;
  //if (pRRTree2)
	 // delete pRRTree2;

  pRRTree = new Tree3d();
//  pRRTree2 = new Tree3d();

  //fin >> pRRTree;
  //std::cout << "T \n" << pRRTree << std::endl;
  //fin >> pRRTree2;
}

//-------------------------------------------------------------------------