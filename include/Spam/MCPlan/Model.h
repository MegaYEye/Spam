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
//! @Author:   Claudio Zito
//! @Date:     21/07/2012
//-------------------------------------------------------------------------
#pragma once
#ifndef _SPINTA_PLANNER_MODEL_H_
#define _SPINTA_PLANNER_MODEL_H_

//-------------------------------------------------------------------------

#include <Spam/MCPlan/Heuristic.h>
//#include <Golem/Math/Sample.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <Eigen/StdVector>

//-------------------------------------------------------------------------

namespace spam {

//-------------------------------------------------------------------------

class Model {
public:
	typedef boost::shared_ptr<Model> Ptr;

	/** Choose state */
	typedef std::function<golem::Vec3(const golem::Vec3&)> NextState;
	/** Choose state */
	typedef std::function<golem::Vec3(const golem::Vec3&, const golem::Vec3&)> NextInput;

	/** Model description */
	class Desc {
	public:
		/** Pointer */
		typedef boost::shared_ptr<Desc> Ptr;
		/** Pointer to heuristic desc */
		Heuristic::Desc::Ptr pHeuristicDesc;

		/** Customised callback to choose next random state */
		NextState nextState;
		/** Customised callback to choose next action */
		NextInput nextInput;

		/** Program name */
		std::string name;

		/** The time interval to use for numerical integration (affects accuracy) */
		golem::Real modelDeltaT;
		/** The space interval to use for numerical integration (affects accuracy) */
		golem::Real modelDeltaS;

		/** The dimension of the state space */
		int stateDim;

		Desc() {
			setToDefault();
		}

		void setToDefault() {
			pHeuristicDesc.reset(new spam::Heuristic::Desc);

			nextState = nullptr;
			nextInput = nullptr;

			name = "Model";
			
			modelDeltaT = 0.1;
			modelDeltaS = 0.1;
			
			stateDim = 3;
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			return true;
		}
		
		/** Creates the object from the description. */
		Model::Ptr create(const golem::Context& context) const {
			Model* pObject = new Model(context);
			Model::Ptr pointer(pObject);
			pObject->create(*this);
			return pointer;
		}
	};

	/** Select random state */
	virtual golem::Vec3 getNextState(const golem::Vec3& x) const;
	/** Select input */
	virtual golem::Vec3 selectInput(const golem::Vec3& node, const golem::Vec3& state) const;
	/** Perform integration from state x, using input u, over time step h */
	virtual golem::Vec3 integrate(const golem::Vec3& x, const golem::Vec3& u, const double h);
	/** Test whether global state-space constraints are satisfied */ 
	virtual inline bool satisfied(const golem::Vec3& x);

	/** Computes cost function as linear and angular distance between two points */
	inline golem::Real cost(const golem::Vec3& x1, const golem::Vec3& x2) const { return pHeuristic->cost(x1, x2); };
	/** Computes cost of a state */
	inline golem::Real cost(const golem::Vec3& x) const { return pHeuristic->cost(x); };

	inline golem::Context& getContext() {
		return context;
	}

	inline const golem::Context& getContext() const {
		return context;
	}

	virtual ~Model() {};

protected:
	/** Context */
	golem::Context context;
	/** Pointer to the heuristic */
	Heuristic::Ptr pHeuristic;

	/** Default callback to choose next random state */
	NextState defaultNextState;
	/** Callback to choose next random state (optional) */
	NextState nextState;

	/** Default callback to choose next random state */
	NextInput defaultNextInput;
	/** Callback to choose next random state (optional) */
	NextInput nextInput;

	/** Program name */
	std::string name;
	/** The time interval to use for numerical integration (affects accuracy) */
	golem::Real modelDeltaT;
	/** The space interval to use for numerical integration (affects accuracy) */
	golem::Real modelDeltaS;
	/** The dimension of the state space */
	int stateDim;

	/** Generator of pseudo random numbers */
	golem::Rand rand;

	/** The state transition equation, or equations of motion, xdot=f(x,u) */
	virtual golem::Vec3 stateTransitionEquation(const golem::Vec3 &x, const golem::Vec3 &u);
	/** Runge-Kutta integration */
	golem::Vec3 rungeKuttaIntegration(const golem::Vec3 &x, const golem::Vec3 &u, const double &h);

	/** Render sequence of nodes */
	virtual void render(const Node3d::SeqPtr& nodes);
	/** Render */
	virtual void render();

	/** Create from the description */
	bool create(const Desc &desc);
	/** Contructor */
	Model(const golem::Context& context);
};

//-------------------------------------------------------------------------

class Atlas : public Model {
public:
	typedef boost::shared_ptr<Atlas> Ptr;

	/** Chart */
	class Chart {
	public:
		/** Smart pointer */
		typedef boost::shared_ptr<Chart> Ptr;
		/** Sequence of charts */
		typedef std::vector<Chart> Seq;
		/** Sequence of chart pointers */
		typedef std::vector<Chart*> SeqPtr;

		/** Chart sample */
		//class Sample : public golem::Sample<golem::Real> {
		//public:
		//	typedef std::vector<Sample> Seq;

		//	/** Dereferencing template */
		//	struct Ref {
		//		template <typename _Ptr> static inline const Chart& get(_Ptr& ptr) {
		//			return ptr; // nothing to do
		//		}
		//	};

		//	Sample() {}
		//	Sample(const Chart& c, golem::Real weight = golem::REAL_ONE, golem::Real cdf = -golem::REAL_ONE) : golem::Sample<golem::Real>(weight, cdf) {}
		//};

		/** Default C'tor */
		Chart() {
			origin = normal = golem::Vec3::zero();
			Tx = Ty = Eigen::Vector3d();
			radius = 0;
		}
		/** Copy c'tor */
		Chart(const Chart& c) {
			origin = c.origin;
			normal = c.normal;
			Tx = c.Tx;
			Ty = c.Ty;

			radius = c.radius;
		}
		/** C'tor */
		Chart(const golem::Vec3& origin, const golem::Vec3& normal) {
			this->origin = origin;
			this->normal = normal;
		}

		golem::Vec3 computeTangentBasis() {
			golem::Vec3 n(normal);
			n.normalise();
			Eigen::Vector3d N = convertToEigen(n);
			Eigen::Matrix3d TProj = Eigen::Matrix3d::Identity() - N*N.transpose();
			Eigen::JacobiSVD<Eigen::Matrix3d> svd(TProj, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Tx = svd.matrixU().col(0);
			Ty = svd.matrixU().col(1);
		}

		static Eigen::Vector3d convertToEigen(const golem::Vec3& v) {
			return Eigen::Map<Eigen::VectorXd>((double *)v.data(), 3);
		}

		/** Origin point on the gp */
		golem::Vec3 origin;
		/** Normal to the tangental plane */
		golem::Vec3 normal;
		/** Basic vector x for the tangental plane */
		Eigen::Vector3d Tx;
		/** Basic vector y for the tangental plane */
		Eigen::Vector3d Ty;

		/** Radius of the manifold */
		golem::Real radius;
	};

	/** Atlas description */
	class Desc : public Model::Desc {
	public:
		/** Pointer */
		typedef boost::shared_ptr<Desc> Ptr;

		Desc() {
			setToDefault();
		}

		void setToDefault() {
			Model::Desc::setToDefault();
		}

		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!Model::Desc::isValid())
				return false;
			return true;
		}

		/** Creates the object from the description. */
		Model::Ptr create(const golem::Context& context) const {
			Atlas* pObject = new Atlas(context);
			Model::Ptr pointer(pObject);
			pObject->create(*this);
			return pointer;
		}
	};

	virtual ~Atlas() {};

protected:
	/** Create from the description */
	bool create(const Desc &desc);
	/** Contructor */
	Atlas(const golem::Context& context);
};


};

//-------------------------------------------------------------------------

#endif /* _SPINTA_PLANNER_MODEL_H_ */