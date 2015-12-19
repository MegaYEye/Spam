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
// @Date:     06/04/2011
//-------------------------------------------------------------------------
/** @file Graph.h
*
*
* @author	Claudio Zito
*
* @copyright  Copyright (C) 2015 Claudio, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*
*/
#ifndef __PLAN_GRAPH_H__
#define __PLAN_GRAPH_H__
#define _USE_MATH_DEFINES

//------------------------------------------------------------------------------

#include <cstdio>
#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <boost/shared_ptr.hpp>
#include <Golem/Math/Math.h>
#include <Golem/Math/Vec3.h>
#include <Golem/Tools/Context.h>

//-------------------------------------------------------------------------

namespace spam {

//-------------------------------------------------------------------------

/** Node base class.
*/
class Node {
public:
	/** Node sequence */
	typedef std::vector<Node> Seq;

	/** Index */
	typedef golem::U32 Index;
	/** Index pair */
	typedef golem::U64 IndexPair;

	/** Goal node index */
//	static const golem::U32 IDX_GOAL = 0;
	/** Root node index */
	static const golem::U32 IDX_ROOT = 0;
	/** Uninitialised node index */
	static const golem::U32 IDX_UINI = golem::numeric_const<golem::U32>::MAX;

	/** Zero/uninitialised cost */
	static const golem::Real COST_ZERO;
	/** Infinite cost (node unreachable) */
	static const golem::Real COST_INF;

	/** Index comparator */
	struct index_less {
		inline bool operator () (const Node &left, const Node &right) const {
			return left.index < right.index;
		}
	};

	/** Cost comparator */
	struct cost_less {
		inline bool operator () (const Node &left, const Node &right) const {
			return left.cost < right.cost;
		}
	};

	/** Make pair */
	static inline golem::U64 makeIndexPair(golem::U32 left, golem::U32 right) {
		return (golem::U64)right << 32 | left;
	}
	/** Make pair */
	static inline golem::U64 makeIndexPair(const Node& left, const Node& right) {
		return makeIndexPair(left.index, right.index);
	}

	/** Node index, index E <0, size - 1> */
	golem::U32 index;
	/** (Minimal) cost of reaching the goal node, cost E <COST_ZERO, COST_INF> */
	golem::Real cost;
	/** Collision */
	bool collides;

	Node(golem::U32 index = IDX_UINI, golem::Real cost = COST_ZERO, bool collides = false) : index(index), cost(cost), collides(false) {
	}
	Node(const Node& node) : index(node.index), cost(node.cost), collides(node.collides) {
	}
};

//-------------------------------------------------------------------------

template <typename Type>
class RRTNode : public Node {
public:
	typedef std::vector<RRTNode> Seq;
	typedef std::vector<RRTNode*> SeqPtr;
	typedef std::vector<golem::U32> U32Seq;

	/** Data cointained in this node */
	Type data;
	/** Parent node */
	RRTNode *parent;
	/** Sequence of children */
//	U32Seq children;

	RRTNode(golem::U32 index = Node::IDX_UINI, golem::Real cost = Node::COST_ZERO) : Node(index, cost), parent(nullptr) {}

	RRTNode(const Type& data, RRTNode* pn, golem::U32 index = Node::IDX_UINI, golem::Real cost = Node::COST_ZERO) : Node(index, cost), parent(pn) {
		this->data = data;
	}

	golem::I32 getParentIndex() const {
		return parent ? parent->index : golem::I32(-1);
	}
	//void addChild(RRTNode* n) {
	//	children.push_back(n);
	//}

	//const Seq& getChildren() const { return children; }
};

//-------------------------------------------------------------------------

typedef RRTNode<golem::Vec3> Node3d;

//-------------------------------------------------------------------------

template <typename Type>
class Tree {
public:
	typedef RRTNode<Type> TNode;
	typedef std::vector<TNode*> TNodeSeq;

	Tree() {
		nodes.clear();
		size = 0;
	}
	Tree(const Type& data) {
		root = new TNode(data, nullptr, 0);
		nodes.push_back(root);
		size = 1;
	}

	bool makeRoot(const Type& data) {
		if (!nodes.empty())
			return false;
		root = new TNode(data, nullptr, 0);
		nodes.push_back(root);
		size = 1;
		return true;
	}

	TNode* extend(TNode* parent, const Type& data) {
		TNode *nn = new TNode(data, parent, size++);
		nodes.push_back(nn);
		return nodes.back();
	}

	TNodeSeq pathToRoot(TNode* n) const {
		TNodeSeq path;

		TNode *nn = n;
		while (nn != root) {
			path.push_back(nn);
			nn = nn->parent;
			if (!nn)
				throw golem::Message("Tree::pathToRoot(): Invalid pointer.");
		}
		// add the root
		path.push_back(nn);
		return path;
	}

	TNode* findNode(const golem::U32 idx) {
		//if (idx > nodes.size()) 
		//	throw golem::Message("findNode(): Invalid index\n");
		//
		//return &nodes[idx];
		for (TNodeSeq::iterator i = nodes.begin(); i != nodes.end(); ++i)
			if (i->index == idx)
				return &*i;

		return nullptr;
	}

	golem::U32 getSize() const { return size; }
	golem::U32 getNodesSize() const { return nodes.size(); }

	const TNodeSeq& getNodes() const { return nodes; }
	TNode* getRoot() const { return root; }
	void clear()  { 
		for (size_t i = nodes.size() - 1; i >= 0; --i)
			delete nodes[i];
		nodes.clear();
		delete root;
	}

	void print() const {
		printf("RRTree: size [%d]\n", nodes.size());
		for (auto i = nodes.begin(); i != nodes.end(); ++i)
			printf("Node[%d] -> parent[%d] cost=%f\n", (*i)->index, (*i)->getParentIndex(), (*i)->cost);
	}

	~Tree() {
		clear();
	}

private:
	TNodeSeq nodes;
	TNode* root;
	golem::U32 size;
};

//-------------------------------------------------------------------------

typedef Tree<golem::Vec3> Tree3d;

//-------------------------------------------------------------------------

//template <typename Type>
//class Tree
//
//
//template <typename _Type> class Tree;
//
////-------------------------------------------------------------------------
//
///** Basic class for Node. */
//template <typename _Real>
//class _Node {
//public:
//	typedef _Real Real;
//	typedef golem::_Vec3<_Real> Vec3;
//	typedef std::vector<Vec3> Vec3Seq;
//
//	typedef boost::shared_ptr<_Node> Ptr;
//	typedef std::vector<_Node> Seq;
//	typedef std::vector<_Node*> SeqPtr;
//
//private:
//	Vec3 state;
//	_Node* parent;
//	SeqPtr children;
//	double time;
//	double cost;
//	int id;
//
//	void* info;
//
//public:
//	//! The state to which this node corresponds
//	virtual inline Vec3 getState() const { return state; };
//
//	inline Ptr getParent() { return parent; };
//	inline SeqPtr const getChildren() { return children; };
//
//	//! The time required to reach this node from the parent
//	inline double getTime() const { return time; };
//
//	//! A cost value, useful in some algorithms
//	inline double getCost() const { return cost; };
//
//	//! A cost value, useful in some algorithms
//	inline void setCost(const double &x) { cost = x; };
//
//	//! Change the node ID
//	inline void setID(const int &i) { id = i; };
//
//	//! Get the node ID
//	inline int getID() const { return id; };
//
//	//! Get the information 
//	void* getInfo() { return info; };
//
//	//! Set the information
//	void setInfo(void* in) { info = in; };
//
//	_Node() : parent(nullptr) {};
//
//	_Node(void* pninfo) : parent(nullptr) {
//		info = pninfo;
//	};
//	_Node(_Node* pn, const Vec3& x) : parent(pn) {
//		state = x;
//		time = 1.0; // Make up a default
//		cost = 0.0;
//	}
//	_Node(_Node* pn, const Vec3& x, double t) : parent(pn) {
//		state = x;
//		time = t; // Make up a default
//		cost = 0.0;
//	}
//	_Node(_Node* pn, const Vec3& x, const Vec3& u, double t, void* pninfo) : parent(pn) {
//		state = x;
//		time = t; // Make up a default
//		cost = 0.0;
//		info = pninfo;
//	}
//
//	~_Node() { children.clear(); /*ClearInfo();*/ };
//
//	inline void addChild(_Node* cn) { children.push_back(cn); }
//
//	friend std::istream& operator>> (std::istream& is, _Node& n) {};
//	friend std::ostream& operator<< (std::ostream& os, const _Node& n) {
//		out << n.id;
//		if (n.parent)
//			out << " " << n.parent->id;
//		else
//			out << " -1";
//		out << " " << n.state;
//		//  out << " " << n.state;// << " " << n.input;
//		out << endl;
//		out.flush();
//
//		return out;
//	}
//	friend std::istream& operator>> (std::istream& is, Seq & nl) {};
//	friend std::ostream& operator<< (std::ostream& os, const Seq & nl) {
//		list<Node*>::iterator x;
//		list<Node*> vl;
//		vl = L;
//		for (x = vl.begin(); x != vl.end(); x++)
//			out << " " << **x;
//		return out;
//	}
//
//	friend class Tree<_Real>;
//};
//
////-------------------------------------------------------------------------
//
//typedef _Node<golem::Real> Node3d;
//
////-------------------------------------------------------------------------
//
////! This is a comparison object to be used for STL-based sorting
//template <typename _Real>
//class NodeLess {
//public:
//	bool operator() (_Node* p, _Node* q) const {
//		return p->getCost() < q->getCost();
//	}
//};
//
//
////! This is a comparison object to be used for STL-based sorting
//template <typename _Real>
//class NodeGreater {
//public:
//	bool operator() (_Node* p, _Node* q) const {
//		return p->getCost() > q->getCost();
//	}
//};
//
////-------------------------------------------------------------------------
//
///**
//* Basic class for a Tree.
//*/
//template <typename _Real>
//class Tree {
//public:
//	typedef _Real Real;
//	typedef golem::_Vec3<_Real> Vec3;
//	typedef std::vector<Vec3> Vec3Seq;
//	typedef boost::shared_ptr<Tree> Ptr;
//	typedef _Node<Real> Node;
//	typedef std::vector<Node> NodeSeq;
//	typedef boost::shared_ptr<Node> NodePtr;
//
//
//private:
//	NodeSeq nodes;
//	NodePtr root;
//
//public:
//
//	Tree() {
//		root.reset();
//		nodes.clear();
//	};
//	/** Argument is state of root node */
//	Tree(const Vec3& x) {
//		nodes.clear();
//		Node nn(nullptr, x, Vec3(), 0.0);
//		nn.id = 0;
//		nodes.push_back(nn);
//		root.reset(&nodes.back());
//	}
//	Tree(const Vec3& x, void* nodeinfo) {
//		nodes.clear();
//		Node nn(nullptr, x, Vec3(), 0.0, nodeinfo);
//		nn.id = 0;
//		nodes.push_back(nn);
//		root.reset(&nodes.back());
//	}
//
//	~Tree() {
//	}
//
//	void makeRoot(const Vec3& x) {
//		Vec3 u;
//
//		if (nodes.empty()) {
//			Node nn(nullptr, x, Vec3(), 0.0);
//			nn.id = 0;
//			nodes.push_back(nn);
//			root.reset(&nodes.back());
//		}
//		else
//			cout << "Root already made.  MakeRoot has no effect.\n";
//		size = 1;
//	}
//
//	Node* extend(Node* parent, const Vec3& x, const Vec3& u) {
//		nodes.push_back(Node(parent, x, u));
//		nodes.back()->id = size;
//		size++;
//
//		return *nodes.back();
//	}
//
//	Node* extend(Node* parent, const Vec3& x, const Vec3& u, double time) {
//		nodes.push_back(new Node(parent, x, u, time));
//		nodes.back()->id = size;
//		size++;
//
//		return nodes.back();
//	}
//
//	Node* extend(Node* parent, const Vec3& x, const Vec3& u, double time, void* pninfo) {
//		nodes.push_back(new Node(parent, x, u, time, pninfo));
//		nodes.back()->id = nodes.size() - 1;
//
//		return &nodes.back();
//	}
//
//	NodeSeqPtr pathToRoot(Node* n) const {
//		NodeSeq nl;
//		NodePtr ni;
//
//		ni.reset(n);
//		while (ni.get() != root.get()) {
//			nl.push_back(ni.get());
//			ni = ni->getParent();
//		}
//
//		nl.push_back(root.get());
//
//		return nl;
//	}
//	
//	Node* findNode(int nid) const {
//		for (NodeSeqPtr::const_iterator ni = nodes.begin(); ni != nodes.end(); ni++) {
//			if ((*ni)->id == nid)
//				return *ni;
//		}
//
//		return nullptr; // Indicates failure
//	}
//	
//	inline NodeSeqPtr getNodes() const {
//		return nodes; 
//	}
//	inline Node* getRoot() const { 
//		return root.get(); 
//	}
//	inline int getSize() const { 
//		return size; 
//	}
//
//	void clear() {
//		for (NodeSeqPtr::iterator n = nodes.begin(); n != nodes.end(); n++)
//			delete *n;
//		nodes.clear();
//		root.reset();
//	}
//
//	std::string str() {
//		std::string ss;
//		//for (NodeSeqPtr::const_iterator n = nodes.begin(); n != nodes.end(); ++n)
//		//	ss += "Node["(*n)->id + "] cost=" + (*n)->cost + "]\n";
//		return ss;
//	}
//
//	friend std::istream& operator>> (std::istream& is, Tree& n) {
//		int i, nid, pid, tsize;
//		//Mat34 x;
//		//PushAction u;
//		NodePtr pnode, n; // Parent node
//
//		T.clear();
//
//		is >> tsize;
//		cout << "Loading a tree that has " << tsize << " nodes\n";
//		for (i = 0; i < tsize; i++) {
//			//    is >> nid >> pid >> x >> u;
//			pnode = T.findNode(pid);
//			if (pnode.get()) {
//				n = T.extend(pnode, x, u);
//				n->setID(nid);
//			}
//			else
//				T.makeRoot(x);
//		}
//
//		return is;
//	}
//	friend std::ostream& operator<< (std::ostream& os, const Tree& n) {
//		NodeSeqPtr::iterator x;
//		NodeSeqPtr vl;
//		vl = T.nodes;
//		os << "Tree size:" << T.size << "\n";
//		//  os << T.size << "\n";
//		for (x = vl.begin(); x != vl.end(); x++)
//			os << **x;
//		os.flush();
//		return os;
//	}
//};
//
////-------------------------------------------------------------------------
//
//typedef Tree<golem::Real> Tree3d;

//-------------------------------------------------------------------------

}; /* namespace */

//------------------------------------------------------------------------------

#endif /* __PLAN_GRAPH_H__ *//