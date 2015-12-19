#include <Spinta/Planner/RRTree.h>

//-------------------------------------------------------------------------

using namespace std;
using namespace golem;
using namespace spinta;

//-------------------------------------------------------------------------

ostream& operator<<(ostream& out, const RRTNode& n)
{
  out << n.id;
  if (n.parent) 
    out << " " << n.parent->id;
  else
    out << " -1";
  out << " " << n.state;
//  out << " " << n.state;// << " " << n.input;
  out << endl;
  out.flush();

  return out;
}


ostream& operator<<(ostream& out, const list<RRTNode*>& L)
{
  list<RRTNode*>::iterator x; 
  list<RRTNode*> vl;
  vl = L;
  for (x = vl.begin(); x != vl.end(); x++) 
    out << " " << **x;
  return out;
}


RRTNode::RRTNode() { 
}


RRTNode::RRTNode(void* pninfo) { 
  info = pninfo;
}


RRTNode::RRTNode(RRTNode::Ptr pn, const Mat34 &x, const PushAction &u) {
  state = x;
  inputs = u;
  parent = pn;
  time = 1.0; // Make up a default
  cost = 0.0;
}


RRTNode::RRTNode(RRTNode::Ptr pn, const Mat34 &x, const PushAction &u, double t) {
  state = x;
  inputs = u;
  parent = pn;
  time = t;
  cost = 0.0;
}

RRTNode::RRTNode(RRTNode::Ptr pn, const Mat34 &x, const PushAction &u, double t, void* pninfo) {
  state = x;
  inputs = u;
  parent = pn;
  time = t;
  cost = 0.0;

  info = pninfo;
}


// *********************************************************************
// CLASS:     RRTree class
// 
// *********************************************************************

ostream& operator<< (ostream& os, const RRTree& T) {
  RRTNode::Seq::iterator x;
  RRTNode::Seq vl;
  vl = T.nodes;
  os << "Tree size:" << T.size << "\n";
//  os << T.size << "\n";
  for (x = vl.begin(); x != vl.end(); x++)
    os << **x;
  os.flush();
  return os;
}



istream& operator>> (istream& is, RRTree & T) {
  int i,nid,pid,tsize;
  Mat34 x;
  PushAction u;
  RRTNode::Ptr pnode, n; // Parent node

  T.clear();

  is >> tsize;
  cout << "Loading a tree that has " << tsize << " nodes\n";
  for (i = 0; i < tsize; i++) {
//    is >> nid >> pid >> x >> u;
    pnode = T.findNode(pid);
    if (pnode.get()) {
      n = T.extend(pnode,x,u);
      n->setID(nid);
    }
    else
      T.makeRoot(x);
  }

  return is; 
}


RRTree::RRTree() { 
  *root = NULL;
  size = 0;
}


RRTree::RRTree(const Mat34 &x) { 
  PushAction u;

  *root = new RRTNode(RRTNode::Ptr(NULL),x,u,0.0);
  root->id = 0;
  nodes.push_back(root);
  size = 1;
}


RRTree::RRTree(const Mat34 &x, void* nodeinfo) { 
  PushAction u;

  *root = new RRTNode(RRTNode::Ptr(NULL),x,u,0.0,nodeinfo);
  root->id = 0;
  nodes.push_back(root);
  size = 1;
}


RRTree::~RRTree() {
  clear();
}


void RRTree::makeRoot(const Mat34 &x) {
  PushAction u;
  
  if (!root.get()) {
    *root = new RRTNode(RRTNode::Ptr(NULL),x,u,0.0);
    root->id = 0;
    nodes.push_back(root);
  }
  else
    cout << "Root already made.  MakeRoot has no effect.\n";
  size = 1;
}


RRTNode::Ptr RRTree::extend(RRTNode::Ptr parent, const Mat34 &x, const PushAction &u) {
  RRTNode *nn;

  nn = new RRTNode(parent, x, u);
  nn->id = size;
  RRTNode::Ptr ptr = RRTNode::Ptr(nn);
  nodes.push_back(ptr);
  size++;

  return ptr;
}



RRTNode::Ptr RRTree::extend(RRTNode::Ptr parent, const Mat34 &x, 
			 const spinta::PushAction &u,
			 double time) {
  RRTNode *nn;

  nn = new RRTNode(parent, x, u, time);
  nn->id = size;
  RRTNode::Ptr ptr = RRTNode::Ptr(nn);
  nodes.push_back(ptr);
  size++;

  return ptr;
}


RRTNode::Ptr RRTree::extend(RRTNode::Ptr parent, const Mat34 &x, 
			 const spinta::PushAction &u,
			 double time, void* pninfo) {
  RRTNode *nn;

  nn = new RRTNode(parent, x, u, time, pninfo);
  nn->id = size;
  RRTNode::Ptr ptr = RRTNode::Ptr(nn);
  nodes.push_back(ptr);
  size++;

  return ptr;
}



RRTNode::Ptr RRTree::findNode(int nid) {
  RRTNode::Seq::iterator ni;

  for(ni = nodes.begin(); ni != nodes.end(); ni++) {
    if ((*ni)->id == nid)
      return *ni;
  }

  return RRTNode::Ptr(NULL); // Indicates failure
}



RRTNode::Seq RRTree::pathToRoot(RRTNode::Ptr n) {
  RRTNode::Seq nl;
  RRTNode::Ptr ni;
  
  *ni = *n;
  while (ni.get() != root.get()) {
    nl.push_back(ni);
    ni = ni->getParent();
  }

  nl.push_back(root);
 
  return nl;
}


void RRTree::clear() {
  RRTNode::Seq::iterator n; 
  for (n = nodes.begin(); n != nodes.end(); n++)
	  delete n->get();
  nodes.clear();
  *root = NULL;
}

