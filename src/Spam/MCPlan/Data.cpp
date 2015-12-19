#include <Spinta/Planner/Data.h>

//-------------------------------------------------------------------------

using namespace std;
using namespace golem;
using namespace spinta;

//-------------------------------------------------------------------------

//std::ostream& operator<< (std::ostream& out, const golem::Vec3 &x);
//std::ostream& operator<< (std::ostream& out, const golem::Mat34 &x);
//std::ostream& operator<< (std::ostream& out, const golem::Vec3 &x) {
//	out << "<" << x.x << "," << x.y << "," << x.z << ">";
//	return out;
//};
//
//std::ostream& operator<< (std::ostream& out, const golem::Mat34 &x) {
//	golem::Quat q(x.R);
//	out << "[" << q.w << "," << q.x << "," << q.y << "," << q.z << "]<" << x.p.x << "," << x.p.y << "," << x.p.z << ">";
//	return out;
//};

//-------------------------------------------------------------------------

Action::Action() {
	setToDefault();
}

Action::Action(const Mat34 &initPose, const Mat34 &targetPose) {
	setToDefault();
	this->initPose = initPose;
	this->targetPose = targetPose;
}

void Action::setToDefault() {
	initPose.setToDefault();
	targetPose.setToDefault();
	deltaS.setZero();
	t = 0;
}

bool Action::isValid() const {
	if (initPose.equals(targetPose, golem::REAL_EPS))
		return false;

	return true;
}

Action::~Action() {
}

//-------------------------------------------------------------------------

PushAction::PushAction() : Action() {
}

PushAction::PushAction(const Mat34 &initPose, const Mat34 &targetPose) : Action(initPose, targetPose) {
}

PushAction::~PushAction() {
}

//-------------------------------------------------------------------------

ostream& operator<< (ostream& out, const RRTNode &n)
{
	out << n.id;
	if (n.parent) 
		out << " " << n.parent->id;
	else
		out << " -1";
	out << " " << n.state.p.x << " " << n.state.p.y << " " << n.state.p.z << std::endl;
	//  out << " " << n.state;// << " " << n.input;
	out << endl;
	out.flush();

	return out;
}


ostream& operator<< (ostream& out, const RRTNode::Seq &L)
{
	RRTNode::Seq::iterator x; 
	RRTNode::Seq vl;
	vl = L;
	for (x = vl.begin(); x != vl.end(); x++) 
		out << " " << *x;
	return out;
}


RRTNode::RRTNode() {
}


RRTNode::RRTNode(void* pninfo) { 
	info = pninfo;
}


RRTNode::RRTNode(RRTNode *pn, const Mat34 &x, const PushAction::Seq &u) {
	state = x;
	inputs = u;
	parent = pn;
	time = 1.0; // Make up a default
	cost = 0.0;
	shapes.clear();
	setBounds();
}


RRTNode::RRTNode(RRTNode *pn, const Mat34 &x, const PushAction::Seq &u, double t) {
	state = x;
	inputs = u;
	parent = pn;
	time = t;
	cost = 0.0;
	shapes.clear();
	setBounds();
}

RRTNode::RRTNode(RRTNode *pn, const Mat34 &x, const PushAction::Seq &u, double t, void* pninfo) {
	state = x;
	inputs = u;
	parent = pn;
	time = t;
	cost = 0.0;
	info = pninfo;
	shapes.clear();
	setBounds();
}

//-------------------------------------------------------------------------

//void RRTNode::render() {
//	Bounds::Desc::Seq::const_iterator shape;
//	CriticalSectionWrapper csw(csRenderer);
//	renderer.reset();
//	forall(shape, shapes) {
//		renderer.renderWire(*(*shape)->create());
//	}
//}

void RRTNode::setBounds() {
	BoundingBox::Desc nodeDesc;
	nodeDesc.dimensions = Vec3(0.05, 0.15, 0.05);
//	nodeDesc.pose = state;
	shapes.push_back(Bounds::Desc::Ptr(&nodeDesc));
}

//-------------------------------------------------------------------------

ostream& operator<< (ostream& os, const RRTree& T) {
	RRTNode::Seq::iterator x;
	RRTNode::Seq vl;
	vl = T.nodes;
	os << "Tree size:" << T.size << "\n";
	//  os << T.size << "\n";
	for (x = vl.begin(); x != vl.end(); x++)
		os << *x;
	os.flush();
	return os;
}



istream& operator>> (istream& is, RRTree &T) {
	int i, nid, pid, tsize;
	Mat34 x;
	PushAction::Seq u;
	RRTNode *pnode, *n; // Parent node

	T.clear();

	is >> tsize;
	cout << "Loading a tree that has " << tsize << " nodes\n";
	for (i = 0; i < tsize; i++) {
		//    is >> nid >> pid >> x >> u;
		pnode = T.findNode(pid);
		if (pnode) {
			n = T.extend(pnode,x,u);
			n->setID(nid);
		}
		else
			T.makeRoot(x);
	}

	return is; 
}

//-------------------------------------------------------------------------

RRTree::RRTree() { 
	root = NULL;
	size = 0;
}


RRTree::RRTree(const Mat34 &x) { 
	PushAction::Seq u;
	u.clear();

	root = new RRTNode(NULL,x,u,0.0);
	root->id = 0;
	nodes.push_back(root);
	size = 1;
}


RRTree::RRTree(const Mat34 &x, void* nodeinfo) { 
	PushAction::Seq u;
	u.clear();

	root = new RRTNode(NULL, x, u, 0.0, nodeinfo);
	root->id = 0;
	nodes.push_back(root);
	size = 1;
}


RRTree::~RRTree() {
	clear();
}

//-------------------------------------------------------------------------

void RRTree::render() {
	RRTNode::Seq::const_iterator node;
	CriticalSectionWrapper csw(csRenderer);
	forall(node, nodes) {
//		(*node)->render();
	}
}

//-------------------------------------------------------------------------

void RRTree::makeRoot(const Mat34 &x) {
	PushAction::Seq u;
	u.clear();
  
	if (!root) {
		root = new RRTNode(NULL, x, u, 0.0);
		root->id = 0;
		nodes.push_back(root);
	}
	else
		cout << "Root already made.  MakeRoot has no effect.\n";
	size = 1;
}


RRTNode* RRTree::extend(RRTNode* parent, const Mat34 &x, const PushAction::Seq &u) {
	RRTNode *nn;

	nn = new RRTNode(parent, x, u);
	nn->id = size;  
	nodes.push_back(nn);
	parent->addChild(nn);

	size++;

	return nn;
}



RRTNode* RRTree::extend(RRTNode* parent, const Mat34 &x, 
			 const PushAction::Seq &u, double time) {
	RRTNode *nn;

	nn = new RRTNode(parent, x, u, time);
	nn->id = size;
	nodes.push_back(nn);
	parent->addChild(nn);
	size++;

	return nn;
}


RRTNode* RRTree::extend(RRTNode* parent, const Mat34 &x, 
			 const PushAction::Seq &u, double time, void* pninfo) {
	RRTNode *nn;

	nn = new RRTNode(parent, x, u, time, pninfo);
	nn->id = size;
	nodes.push_back(nn);
    parent->addChild(nn);

	size++;

	return nn;
}



RRTNode* RRTree::findNode(int nid) {
  RRTNode::Seq::iterator ni;

  for(ni = nodes.begin(); ni != nodes.end(); ni++) {
    if ((*ni)->id == nid)
      return *ni;
  }

  return NULL; // Indicates failure
}



RRTNode::Seq RRTree::pathToRoot(RRTNode *n) {
  RRTNode::Seq nl;
  RRTNode *ni;
  
  ni = n;
  while (ni != root) {
    nl.push_back(ni);
    ni = ni->getParent();
  }

  nl.push_back(root);
 
  return nl;
}


void RRTree::clear() {
  RRTNode::Seq::iterator n; 
  for (n = nodes.begin(); n != nodes.end(); n++)
	  delete *n;
  nodes.clear();
  *root = NULL;
}