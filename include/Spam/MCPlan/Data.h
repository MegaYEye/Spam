//-------------------------------------------------------------------------
//                  Single Finger Pushing Tool (SPinTa)
//-------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal with the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Single Finger Pushing Tool (SPinTa), 
//       University of Birmingham, nor the names of its contributors may be 
//       used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind,
// express or implied, including but not limited to the warranties of
// merchantability, fitness for a particular purpose and
// noninfringement.  In no event shall the contributors or copyright
// holders be liable for any claim, damages or other liability, whether
// in an action of contract, tort or otherwise, arising from, out of or
// in connection with the software or the use of other dealings with the
// software.
//
//-------------------------------------------------------------------------
// @Author:   Claudio Zito
// @Date:     19/07/2012
//-------------------------------------------------------------------------
#ifndef _SPINTA_PLANNER_DATA_H_
#define _SPINTA_PLANNER_DATA_H_

//-------------------------------------------------------------------------

#include <Golem/Tools/Data.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/XMLParser.h>
#include <Golem/Tools/Message.h>
#include <Golem/Defs/Defs.h>
#include <Golem/Phys/Data.h>
#include <Golem/Ctrl/Data.h>
#include <Spinta/Planner/Robot.h>
//#include <Grasp/Grasp/Robot.h>

//-------------------------------------------------------------------------

#define INFINITY 1.0e40

// Convenient list iterator
#define forall(x,S)\
for (x = S.begin(); x != S.end(); x++)\

//-------------------------------------------------------------------------

template<class T> std::ostream& operator<< (std::ostream& out, const std::vector<T>& L);
template<class T> std::istream& operator>> (std::istream& in, std::vector<T>& L);
//std::ostream& operator<< (std::ostream& out, const golem::Vec3 &x);
//std::ostream& operator<< (std::ostream& out, const golem::Mat34 &x);

template<class T> std::ofstream& operator<< (std::ofstream& out, const std::vector<T>& L)
{
	using namespace std;
	typename vector<T>::iterator x; 
	vector<T> vl;
	vl = L;
	for (x = vl.begin(); x != vl.end(); x++) 
		out << " " << *x;
	return out;
}


template<class T> std::ifstream& operator>> (std::ifstream& in, std::vector<T>& L)
{ 
	using namespace std;
	L.clear();
	T x;
	for(;;)
	{ 
		char c;
		while (in.get(c) && isspace(c));
		if (!in) break;
		in.putback(c);
		x = T(); in >> x; L.push_back(x);
	}
	return in;
}

//-------------------------------------------------------------------------

namespace spinta {

//-------------------------------------------------------------------------

typedef std::vector<golem::Mat34> Mat34Seq;
class RRTree;

//-------------------------------------------------------------------------

class Action {
public:
	/** Pointer */
	typedef golem::shared_ptr<Action> Ptr;
	/** Sequence of actions */
	typedef std::vector<Action> Seq;

	/** Initial pose in workspace */
	golem::Mat34 initPose;
	/** Target pose in workspace */
	golem::Mat34 targetPose;
	
	/** Spatial step in 3D */
	golem::Vec3 deltaS;
	/** Number of iteration */
	size_t t;
	/** Sequence of manipulandum poses */
	Mat34Seq manipulandumPoses;

	/** Default Constructor */
	Action();
	/** Constructor */
	Action(const golem::Mat34 &initPose, const golem::Mat34 &targetPose);
	/** Sets parameters to their default values */
	void setToDefault();
	/** Checks for validity */
	bool isValid() const;
	/** Destructor */
	~Action();
};

class PushAction : public Action {
public:
	/** Pointer */
	typedef golem::shared_ptr<PushAction> Ptr;
	/** Sequence of actions */
	typedef std::vector<PushAction> Seq;

	/** Direction of pushing */
	inline golem::Vec3 direction() const {
		return (targetPose.p - initPose.p).normalise();
	}

	/** Default constructor */
	PushAction();
	/** Constructor */
	PushAction(const golem::Mat34 &initPose, const golem::Mat34 &targetPose);
	/** Destructor */
	~PushAction();
};

//-------------------------------------------------------------------------

/** Basic class for RRTNode. */
class RRTNode {
public:
//	typedef golem::shared_ptr<RRTNode> Ptr;
	typedef std::vector<RRTNode*> Seq;

 //protected:
	golem::Mat34 state;
	PushAction::Seq inputs;
	RRTNode *parent;
	RRTNode::Seq children;
	double time;
	double cost;
	int id;

	void* info;

	/** Shapes */
	golem::Bounds::Desc::Seq shapes;
	/** Colour of the rendered node */
	golem::RGBA colour;
	/** Renderer */
	golem::BoundsRenderer renderer;
	/** Access render in cs */
	golem::CriticalSection csRenderer;


 public:
	//! The state to which this node corresponds
	golem::Mat34 getState() const { return state; };

	//! The state to which this node corresponds
	golem::Vec3 getStateVec() const { return state.p; };

	//! The input vector that leads to this state from the parent
	virtual inline PushAction::Seq getInputs() const { return inputs; };

	inline RRTNode* getParent() { return parent; };
	inline RRTNode::Seq const getChildren() { return children; };
  
	//! The time required to reach this node from the parent
	inline double getTime() const { return time; };

	//! A cost value, useful in some algorithms
	inline double getCost() const { return cost; };

	//! A cost value, useful in some algorithms
	inline void setCost(const double &x) { cost = x; };

	//! Change the node ID
	inline void setID(const int &i) { id = i; };

	//! Get the node ID
	inline int getID() const { return id; };

	//! Get the information 
	void* getInfo() { return info; };
  
	//! Set the information
	void setInfo(void* in) { info = in; };

	//! Clear the memory for the information
	//void ClearInfo() {if (!info) delete (RRTNodeInfo *) info; };
	//NOTE: Above incorrect design - shouldn't type the void* here

	RRTNode();
	RRTNode(void* pninfo);
	RRTNode(RRTNode *pn, const golem::Mat34 &x, const PushAction::Seq &u);
	RRTNode(RRTNode *pn, const golem::Mat34 &x, const PushAction::Seq &u, double t);
	RRTNode(RRTNode *pn, const golem::Mat34 &x, const PushAction::Seq &u, double t, void* pninfo);
	~RRTNode() { children.clear(); /*ClearInfo();*/ };

	inline void addChild(RRTNode *cn) { children.push_back(cn); }

	/** Render */
//	void render();
	/** Set bound */
	void setBounds();

	friend std::istream& operator>> (std::istream& is, RRTNode &n);
	friend std::ostream& operator<< (std::ostream& os, const RRTNode &n);
	friend std::istream& operator>> (std::istream& is, RRTNode::Seq &nl);
	friend std::ostream& operator<< (std::ostream& os, const RRTNode::Seq &nl);

	friend class RRTree;
};

//-------------------------------------------------------------------------

//! This is a comparison object to be used for STL-based sorting
class RRTNodeLess {
	public:
		bool operator() (RRTNode *p, RRTNode *q) const {
			return p->getCost() < q->getCost();
		}
};


//! This is a comparison object to be used for STL-based sorting
class RRTNodeGreater {
	public:
		bool operator() (RRTNode *p, RRTNode *q) const {
			return p->getCost() > q->getCost();
		}
};

//-------------------------------------------------------------------------

/** Basic class for a RRTree. */
class RRTree {
public:
	typedef golem::shared_ptr<RRTree> Ptr;

//protected:
	RRTNode::Seq nodes;
	RRTNode *root;
	int size;

	/** Renderer */
	golem::BoundsRenderer renderer;
	/** Access render in cs */
	golem::CriticalSection csRenderer;

public:

	RRTree();
	RRTree(const golem::Mat34 &x); // Argument is state of root node
	RRTree(const golem::Mat34 &x, void* nodeinfo);
	~RRTree();

	void makeRoot(const golem::Mat34 &x);
	RRTNode* extend(RRTNode *parent, const golem::Mat34 &x, const PushAction::Seq &u);
	RRTNode* extend(RRTNode *parent, const golem::Mat34 &x, const PushAction::Seq &u, 
			double time);
	RRTNode* extend(RRTNode *parent, const golem::Mat34 &x, const PushAction::Seq &u, 
			double time, void* pninfo);

	RRTNode::Seq pathToRoot(RRTNode *n);
	RRTNode* findNode(int nid);
	inline RRTNode::Seq getNodes() const { return nodes; };
	inline RRTNode* getRoot() { return root; };
	inline int getSize() { return size;}

	void clear();

	/** Renderer */
	void render();

	friend std::istream& operator>> (std::istream& is, RRTree& n);
	friend std::ostream& operator<< (std::ostream& os, const RRTree& n);
};

//-------------------------------------------------------------------------

/** Trial data for a single demonstration and/or test trial in a binary format (stored on disk) */
class RRTTrialData {
public:
	typedef std::map<golem::U32, RRTTrialData> Map;
	friend class golem::Stream;

	/** Header name */
	static const char headerName [];
	/** Header version */
	static const golem::U32 headerVersion;

	/** Complete trial state sequence */
	RRTNode::Seq rrtNodeSeq;

	/** Complete push action sequence */
	PushAction::Seq inputSeq;

	/** Constructor */
	RRTTrialData() {
		setToDefault();
	}
	/** Sets the parameters to the default values */
	void setToDefault() {
		rrtNodeSeq.clear();
		inputSeq.clear();
	}
	/** Checks if the description is valid. */
	bool isValid() const {
		if (rrtNodeSeq.empty())
			return false;
		if (inputSeq.empty())
			return false;
		
		return true;
	}
};

//-------------------------------------------------------------------------

/** Trial data for a single demonstration and/or test trial in a binary format (stored on disk) */
class TrialData {
public:
	typedef std::map<golem::U32, TrialData> Map;
	friend class golem::Stream;

	/** Header name */
	static const char headerName [];
	/** Header version */
	static const golem::U32 headerVersion;

	/** (Local) kinect data label */
	golem::U32 label;

	/** Constructor */
	TrialData(const golem::Controller& controller) : controller(&controller) {
	}
	/** Sets the parameters to the default values */
	void setToDefault() {

	}
	/** Checks if the description is valid. */
	bool isValid() const {
		return true;
	}

protected:
	/** Controller */
	const golem::Controller* controller;
};

//-------------------------------------------------------------------------

}; /* namespace */

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_DATA_H_ */