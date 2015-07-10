//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     27/10/2012
//------------------------------------------------------------------------------
#pragma once
#ifndef _SPAM_SPAM_H_
#define _SPAM_SPAM_H_

//------------------------------------------------------------------------------

#include <Grasp/Contact/Contact.h>
#include <Grasp/Core/Cloud.h>
#include <Grasp/Core/RBPose.h>
#include <Golem/Plan/GraphPlanner.h>
#include <Golem/Tools/XMLParser.h>
#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>

//------------------------------------------------------------------------------

namespace flann {
	template <typename T> struct L2_Simple;
};

namespace pcl {
	struct PointXYZ;
	template <typename T, typename Dist> class KdTreeFLANN;
	struct PolygonMesh;
};

namespace grasp {
	class Manipulator;
};

namespace spam {
	class FTDrivenHeuristic;
};

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Performance monitor */
#define _COLLISION_PERFMON

//------------------------------------------------------------------------------

/** Forward declaration */
class Robot;
class Belief;
class Hypothesis;

//------------------------------------------------------------------------------

/** Type of guards for Justin */
enum FTGuardTypes {
	/** Absolute values */
	FTGUARD_ABS = 0,
	/** Less than */
	FTGUARD_LESSTHAN,
	/**Greater than */
	FTGUARD_GREATERTHAN,
};
/** HandChains */
enum HandChains {
	UNKNOWN = 0,
	THUMB,
	INDEX,
	MIDDLE,
	RING,
	PINKY,
};
/** Force/torque guards for Justin and BHAM robots */
class FTGuard {
public:
	typedef std::vector<FTGuard> Seq;
	 
	/** Name of the guard {right,left}_{tcp,thumb,tip,middle,ring} */
	std::string name;
	/** Type of guard {|>,<,>} */
	FTGuardTypes type;
	/** Force/torque threshold for the guard */
	golem::Real threshold;
	/** Force/torque measured */
	golem::Real force;

	/** Last arm's joint index */
	golem::Configspace::Index armIdx;
	/** Joint index */
	golem::Configspace::Index jointIdx;
	/** Number of chains in the hand */
	golem::U32 handChains;
	/** Number of joints per fingers */
	golem::U32 fingerJoints;

	/** C'ctor */
	FTGuard(const grasp::Manipulator &manipulator);

	/** D'ctor */
	virtual ~FTGuard() {};

	/** Prints the guard in the format for Justin.
	Example: {right,left}_{tcp,thumb,tip,middle,ring} {0,1,2,[3,4]} {|>,<,>} value **/
	std::string str() const;

	/** Returns the chain index to the finger [1, 5] */
	inline golem::U32 getHandChain() {
		return ((golem::U32)(jointIdx - armIdx) / fingerJoints) + 1;
	}
	/** Returns the chain index to the finger [1, 5] */
	inline const golem::U32 getHandChain() const {
		return ((golem::U32)(jointIdx - armIdx) / fingerJoints) + 1;
	}
	/** Returns the index of the joint per finger [0,3] */
	inline golem::U32 getHandJoint() {
		return (golem::U32)(jointIdx - armIdx) % fingerJoints;
	}
	/** Returns the index of the joint per finger [0,3] */
	inline const golem::U32 getHandJoint() const {
		return (golem::U32)(jointIdx - armIdx) % fingerJoints;
	}

	/** Sets the chain and joint iterators */
	void create(golem::Configspace::Index& joint);
};
/** Reads/writes guards from/to a given XML context */
void XMLData(FTGuard &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** Grasp collision model */
class Collision {
public:
	friend class Hypothesis;
	typedef golem::shared_ptr<Collision> Ptr;
	typedef std::vector<Collision::Ptr> Seq;
	typedef std::vector<grasp::NNSearch::Ptr> NNSearchPtrSeq;

	class Feature {
	public:
		typedef std::vector<Feature> Seq;

		/** As real vector: two surface normals, distance between points, rgb colour */
		static const size_t N = 3;

		/** Feature distance metric */
		class FlannDist {
		public:
			typedef bool is_kdtree_distance;
			typedef golem::Real ElementType;
			typedef golem::Real ResultType;

			//			static const size_t LIN_N = 3;

			/** constructor */
			FlannDist() {
			}

			template <typename _Iter1, typename _Iter2> golem::Real operator() (_Iter1 a, _Iter2 b, size_t size = Feature::N, golem::Real worst_dist = -golem::numeric_const<golem::Real>::ONE) const {
				//printf("a=<%f>, b=<%f>, d=%f\n", a[0], b[0], golem::Math::sqrt(golem::Math::sqr(a[0] - b[0])));
				//return golem::Math::sqrt(golem::Math::sqr(a[0] - b[0]));
//				printf("a=<%f, %f, %f>, b=<%f, %f, %f>, d=%f\n", a[0], a[1], a[2], b[0], b[1], b[2], golem::Math::sqrt(golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2])));
				return golem::Math::sqrt(golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2]));
			}
			inline golem::Real accum_dist(const golem::Real& a, const golem::Real& b, int) const {
				return golem::Math::sqr(a - b);
			}
		};
		/** Feature normals: points' normals in a local frame  */
		golem::Vec3 point;

		/** No member init by default */
		//Feature() {}

		/** Constructor */
		Feature(const golem::Vec3 &data) {
			this->point.set(data);
		}
		/** Access to data as a single vector */
		inline golem::Real* data() {
			return (golem::Real*)&point.v;
		}
		/** Access to data as a single vector */
		inline const golem::Real* data() const {
			return (const golem::Real*)&point.v;
		}

		/** Access to data as a 3D point */
		inline golem::Vec3 getPoint() {
			return point;
		}
		/** Access to data as a 3D point */
		inline const golem::Vec3 getPoint() const {
			return (const golem::Vec3)point;
		}

	};

	/** Bounds */
	template <typename _Real, typename _RealEval> class _Bounds {
	public:
		typedef _Real Real;
		typedef _RealEval RealEval;
		typedef golem::_Vec3<_Real> Vec3;
		typedef golem::_Mat33<_Real> Mat33;
		typedef golem::_Mat34<_Real> Mat34;
		typedef std::vector<Vec3> Vec3Seq;

		typedef std::vector<_Bounds> Seq;

		/** Surface */
		struct Surface {
			typedef std::vector<Surface> Seq;
			typedef std::vector<Seq> SeqSeq;
			Vec3 point;
			Vec3 normal;
		};
		/** Triangle */
		struct Triangle : public Surface {
			typedef std::vector<Triangle> Seq;
			typedef std::vector<Seq> SeqSeq;
			Real distance;
		};

		/** Create bounds from convex meshes */
		inline void create(const golem::Bounds::Seq& bounds) {
			for (size_t i = 0; i < bounds.size(); ++i) {
				const golem::BoundingConvexMesh* mesh = dynamic_cast<const golem::BoundingConvexMesh*>(bounds[i].get());
				if (mesh != nullptr) {
					surfaces.resize(surfaces.size() + 1);
					triangles.resize(triangles.size() + 1);
					surfaces.back().resize(mesh->getTriangles().size());
					triangles.back().resize(mesh->getTriangles().size());
					for (size_t j = 0; j < mesh->getTriangles().size(); ++j) {
						triangles.back()[j].normal = surfaces.back()[j].normal = Vec3(mesh->getNormals()[j]);
						surfaces.back()[j].point = Vec3(mesh->getVertices()[mesh->getTriangles()[j].t1]); // e.g. first triangle
						//surfaces.back()[j].point = (mesh->getVertices()[mesh->getTriangles()[j].t1] + mesh->getVertices()[mesh->getTriangles()[j].t2] + mesh->getVertices()[mesh->getTriangles()[j].t3])/Real(3.0); // centroid
						triangles.back()[j].distance = Real(mesh->getDistances()[j]);
					}
				}
			}
		}

		/** Pose */
		static inline void setPose(const Mat34& pose, const Surface& surface, Triangle& triangle) {
			pose.multiply(triangle.point, surface.point);
			pose.R.multiply(triangle.normal, surface.normal);
			triangle.distance = triangle.normal.dot(triangle.point);
		}
		/** Pose */
		static inline void setPose(const Mat34& pose, const typename Surface::Seq& surfaces, typename Triangle::Seq& triangles) {
			triangles.resize(surfaces.size());
			for (size_t i = 0; i < triangles.size(); ++i)
				setPose(pose, surfaces[i], triangles[i]);
		}
		/** Pose */
		inline void setPose(const Mat34& pose) {
			for (size_t i = 0; i < triangles.size(); ++i)
				setPose(pose, surfaces[i], triangles[i]);
		}

		/** Penetration depth of a given point */
		static inline Real getDepth(const typename Triangle::Seq& triangles, const Vec3& point) {
			Real depth = golem::numeric_const<Real>::MAX;
			for (typename Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = i->distance - i->normal.dot(point);
				if (d < golem::numeric_const<Real>::ZERO) // no collision
					return d;
				if (depth > d) // search for minimum
					depth = d;
			}
			return depth;
		}
		/** Penetration depth of a given point, zero if none */
		inline Real getDepth(const Vec3& point, const bool distance = false) const {
			// if distance true, and there are not collisions, then returns the closest point outside the bounds (negative values) 
			Real depth = distance ? -golem::numeric_const<Real>::MAX : golem::numeric_const<Real>::ZERO; // if no bounds or collisions, no effect
			for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = getDepth(*i, point);
				if (depth < d) // search for maximum depth (can collide only with one mesh within bounds)
					depth = d;
			}
			//if (depth < golem::numeric_const<Real>::ZERO) printf("getdepth=%f\n", depth);
			return depth;
		}

		inline Real getDepth(const Feature &data, const bool distance = false) const {
			return getDepth((Bounds::Vec3)data.getPoint(), distance);
		}

		/** Collision likelihood model. No evaluation for points outside the bounds. */
		template <typename _Ptr> inline _RealEval evaluate(_Ptr begin, _Ptr end, _RealEval depthStdDev, size_t& collisions) const {
			_RealEval eval = golem::numeric_const<_RealEval>::ZERO, c = golem::numeric_const<_RealEval>::ZERO;
			for (_Ptr i = begin; i < end; ++i) {
				const Real depth = getDepth(*i);
				if (depth > golem::numeric_const<Real>::ZERO) {
					const _RealEval pointEval = golem::Math::exp(-depthStdDev*Real(depth));
					golem::kahanSum(eval, c, pointEval);
					++collisions;
				}
			}
			return eval;
		}

		/** Collision likelihood model. Evaluate also the likelihood of collision for point outside the bounds. */
		template <typename _Ptr> inline _RealEval evaluate(_Ptr begin, _Ptr end, _RealEval depthStdDev, _RealEval distanceStdDev, bool observation, const golem::Mat34 &pose, const golem::Real &force, size_t& collisions) const {
			_RealEval eval = golem::numeric_const<_RealEval>::EPS, c = golem::numeric_const<_RealEval>::ZERO;
			Real norm = golem::numeric_const<Real>::ONE / (depthStdDev*Math::sqrt(2 * golem::numeric_const<Real>::PI));
			for (_Ptr i = begin; i < end; ++i) {
				const Real depth = getDepth(*i, true); // penetration depth (positive values) or distance from the closest surfaces (negative values)
				const bool direction = match(pose, *i, force);
				// this hypothesis generate a contact that does not match the observation
				if (!observation && depth > golem::numeric_const<Real>::ZERO) {
//					printf("wrong direction\n");
					return golem::numeric_const<_RealEval>::ZERO;
				}
				// compute likelihood only for bounds were a contact occurred
				if (observation) {
					const _RealEval pointEval = norm*golem::Math::exp(-.5*Math::sqr(Real(depth)/Real(depthStdDev))); // gaussian 
					golem::kahanSum(eval, c, pointEval);
					// penalise concats not on the bound's surface
					if (depth >= 0.0008 || !direction) {
						const _RealEval penalise = -.5*pointEval;  //-golem::Math::exp(-Real(depth)); //-.1*pointEval;
						golem::kahanSum(eval, c, penalise);
					}
					++collisions;
				}
			}
			//if (observation) printf("eval = %f\n", eval);
			return eval;
		}

		inline bool match(const golem::Mat34& pose, const Feature& point, const golem::Real &force) const {
			golem::Mat34 inverse; inverse.setInverse(pose); // compute the inverse of the joint frame
			golem::Vec3 v; inverse.multiply(v, point.getPoint()); //v.normalise(); // compute the point in the joint's reference frame

			return ((v.z < 0 && force <= 0) || (v.z > 0 && force >= 0)) ? false : true; // return false if the the point does not match the observation
		}


		/** Penetration depth of a given point, zero if none */
		inline Real getDistance(const golem::Mat34& pose, const Vec3& point, const Real maxDist) const {
			golem::Mat34 inverse; inverse.setInverse(pose); // compute the inverse of the joint frame
			golem::Vec3 v; inverse.multiply(v, point); v.normalise(); // compute the point in the joint's reference frame

			return v.z < REAL_ZERO ? pose.p.distance(point) : maxDist; // compute the distance only for point in front of the finger tip
		}

		/** Penetration depth of a given point, zero if none */
		inline Real getDistance(const golem::Mat34& pose, const Feature& feature, const Real maxDist) const {
			return getDistance(pose, (Bounds::Vec3)feature.getPoint(), maxDist);
		}

		/** Expected collision likelihood model */
		template <typename _Ptr> inline _RealEval estimate(golem::Mat34& pose, _Ptr begin, _Ptr end, _RealEval depthStdDev, size_t& collisions, const _RealEval maxDist = golem::numeric_const<_RealEval>::MAX) const {
			_RealEval eval = golem::numeric_const<_RealEval>::ZERO, c = golem::numeric_const<_RealEval>::ZERO;
			for (_Ptr i = begin; i < end; ++i) {
				const Real distance = getDistance(pose, *i, maxDist);
				if (distance > golem::numeric_const<Real>::ZERO && distance < maxDist) {
					const _RealEval pointEval = golem::Math::exp(-depthStdDev*Real(distance));
					golem::kahanSum(eval, c, pointEval);
					++collisions;
				}
			}
			return eval;
		}


		/** Empty */
		inline bool empty() const {
			return surfaces.empty();
		}

		/** Triangles */
		inline const typename Triangle::SeqSeq& getTriangles() const {
			return triangles;
		}
		/** Surfaces */
		inline const typename Surface::SeqSeq& getSurfaces() const {
			return surfaces;
		}

	private:
		/** Triangles */
		typename Triangle::SeqSeq triangles;
		/** Surfaces */
		typename Surface::SeqSeq surfaces;
	};

	/** Collision waypoint */
	class Waypoint {
	public:
		typedef std::vector<Waypoint> Seq;

		/** Path distance */
		golem::Real pathDist;
		/** Number of points */
		golem::U32 points;
		/** Distance standard deviation */
		golem::Real depthStdDev;
		/** Likelihood multiplier */
		golem::Real likelihood;

		/** Constructs description object */
		Waypoint() {
			Waypoint::setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			pathDist = golem::Real(0.0);
			points = 1000;
			depthStdDev = golem::Real(1000.0);
			likelihood = golem::Real(1000.0);
		}
		/** Checks if the parameters are valid. */
		bool isValid() const {
			if (!golem::Math::isFinite(pathDist) || depthStdDev < golem::REAL_EPS || likelihood < golem::REAL_ZERO)
				return false;
			return true;
		}
	};

	/** Bounds */
	typedef _Bounds<golem::F32, golem::F32> Bounds;

	/** Flann description */
	class FlannDesc {
	public:
		/** Neighbour points during a query */
		golem::U32 neighbours;
		/** MAx neighbouring points selected for collision detection */
		golem::U32 points;
		/** Distance standard deviation */
		golem::Real depthStdDev;
		/** Likelihood multiplier */
		golem::Real likelihood;

		/** Contructor */
		FlannDesc() {
			setToDefault();
		}
		/** Nothing to do here */
		virtual ~FlannDesc() {}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			neighbours = 1000;
			points = 250;
			depthStdDev = golem::Real(1000.0);
			likelihood = golem::Real(1000.0);
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (neighbours <= 0 || points < 0)
				return false;
			if (depthStdDev < golem::REAL_EPS || likelihood < golem::REAL_ZERO)
				return false;
			return true;
		}

	};
	
	/** Model for the FT sensor in DLR Hit II */
	class FTSensorDesc {
	public:
		/** Force sensor limit */
		grasp::RealSeq ftMedian;
		/** Force sensor limit */
		grasp::RealSeq ftStd;

		FTSensorDesc() {
			FTSensorDesc::setToDefault();
		}
		void setToDefault() {
			ftMedian.clear();
			ftStd.clear();
		}
		bool isValid() const {
			for (size_t i = 0; i < ftMedian.size(); ++i)
				if (!golem::Math::isFinite(ftMedian[i]))
					return false;
			for (size_t i = 0; i < ftStd.size(); ++i)
				if (!golem::Math::isPositive(ftStd[i]))
					return false;
			return true;
		}
	};

	/** Collision description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Switch between kdtrees and random selected points */
		bool kdtree;
		FlannDesc flannDesc;

		/** Force sensor reading in free space */
		FTSensorDesc ftBase;
		/** Force sensor limit */
		FTSensorDesc ftContact;

		/** Collision waypoints */
		Waypoint::Seq waypoints;

		/** KDTree decription*/
		grasp::KDTreeDesc nnSearchDesc;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Collision::Ptr create(const grasp::Manipulator& manipulator) const {
			return Collision::Ptr(new Collision(manipulator, *this));
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			waypoints.clear();
			waypoints.push_back(Waypoint());
			nnSearchDesc.setToDefault();
			kdtree = true;
			flannDesc.setToDefault();
			ftBase.setToDefault();
			ftContact.setToDefault();
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			//if (waypoints.empty())
			//	return false;
			if (!flannDesc.isValid())
				return false;
			if (!ftBase.isValid() || !ftContact.isValid())
				return false;
			//for (Waypoint::Seq::const_iterator i = waypoints.begin(); i != waypoints.end(); ++i)
			//	if (!i->isValid())
			//		return false;
			return true;
		}
	};

	/** Create */
	virtual void create(golem::Rand& rand, const grasp::Cloud::PointSeq& points);

	/** Collision detection at a given waypoint */
	virtual bool check(const Waypoint& waypoint, const grasp::Manipulator::Pose& pose, bool debug = false) const;
	/** Collision detection using kdtree */
	virtual bool check(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, bool debug = false) const;

	/** Collision detection for the observational model durin hypopthesis-based planning */
	virtual golem::Real estimate(const FlannDesc& desc, const grasp::Manipulator::Pose& pose, golem::Real maxDist = golem::REAL_MAX, bool debug = false) const;
	/** Collision detection using kdtree */
//	virtual bool estimate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, bool debug = false) const;

	/** Collision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, std::vector<golem::Configspace::Index>& joints, grasp::RealSeq& forces, bool debug = false) const;
	/** Collision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, FTGuard::Seq& joints, bool debug = false) const;
	/** Collision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, grasp::RealSeq& forces, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint */
	virtual golem::Real evaluate(const Waypoint& waypoint, const grasp::Manipulator::Pose& pose, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint for belief update */
	virtual golem::Real evaluate(const Waypoint& waypoint, const grasp::Manipulator::Pose& pose, const FTGuard::Seq& triggeredGuards, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint for belief update */
	virtual golem::Real evaluate(const FlannDesc& desc, const grasp::Manipulator::Pose& pose, FTGuard::Seq& triggeredGuards, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint */
	virtual golem::Real evaluate(const FlannDesc& desc, const grasp::Manipulator::Pose& pose, bool debug = false) const;

	/** Collision likelihood estimation on a given path */
	virtual golem::Real evaluate(const grasp::Manipulator::Waypoint::Seq& path, bool debug = false) const;

	/** Bounds */
	inline const Bounds::Seq& getBounds() const {
		return bounds;
	}
	/** Points */
	//inline const Bounds::Vec3Seq& getPoints() const {
	//	return points;
	//}
	inline const Feature::Seq& getPoints() const {
		return points;
	}


	/** Draw collisions */
	void draw(const Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, golem::DebugRenderer& renderer) const;
	/** Draw collisions with kdtree */
	void draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, const Collision::FlannDesc& desc) const;
	/** Draw estimate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Pose& rbpose, const Collision::FlannDesc& desc) const;
	/** Draw simulate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Pose& rbpose, std::vector<golem::Configspace::Index> &joints, grasp::RealSeq &forces, const Collision::FlannDesc& desc) const;
	/** Collision likelihood estimation at a given waypoint for belief update */
	void draw(golem::DebugRenderer& renderer, const Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, const FTGuard::Seq &triggeredGuards, bool debug = false) const;

	/** Returns ft_sensor desc in free space */
	FTSensorDesc getFTBaseSensor() {
		return desc.ftBase;
	}
	/** Returns ft_sensor desc in contact */
	FTSensorDesc getFTContactSensor() {
		return desc.ftContact;
	}

#ifdef _COLLISION_PERFMON
	static golem::U32 perfEvalPoints, perfCheckNN, perfEstimate, perfSimulate;
	static golem::SecTmReal tperfEvalPoints, tperfCheckNN, tperfEstimate, tperfSimulate;

	static void resetLog();
	static void writeLog(golem::Context &context, const char *str);

#endif

protected:
	/** Manipulator */
	const grasp::Manipulator& manipulator;
	/** Description */
	const Desc desc;

	/** Joints + base */
	Bounds::Seq bounds;
	/** Points */
	Feature::Seq points;
	//	Bounds::Vec3Seq points;

	/** KD tree pointer */
	grasp::NNSearch::Ptr nnSearch;
	/** Create */
	Collision(const grasp::Manipulator& manipulator, const Desc& desc);
};

void XMLData(Collision::Waypoint& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(Collision::FlannDesc& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(Collision::FTSensorDesc& val, golem::XMLContext* xmlcontext, bool create = false);
void XMLData(Collision::Desc& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** Hypothesis over object poses */
class Hypothesis {
public:
	friend class FTDrivenHeuristic;
	friend class Belief;
	typedef golem::shared_ptr<Hypothesis> Ptr;
	typedef std::map<golem::U32, Ptr> Map;
	typedef std::vector<Ptr> Seq;

	/** Bounds Appearance */
	class BoundsAppearance {
	public:
		/** Show bounds solid */
		bool showSolid;
		/** Show bounds wire frames */
		bool showWire;
		/** Bounds solid colour */
		golem::RGBA solidColour;
		/** Bounds wire colour */
		golem::RGBA wireColour;
		/** Bounds wireframe thickness */
		golem::Real wireWidth;

		/** Constructs from description object */
		BoundsAppearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showSolid = false;
			showWire = true;
			solidColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireColour = golem::RGBA(golem::U8(255), golem::U8(255), golem::U8(0), golem::U8(127));
			wireWidth = golem::Real(1.0);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (wireWidth <= golem::REAL_ZERO)
				return false;
			return true;
		}

		/** Draw bounds */
		void draw(const golem::Bounds::Seq& bounds, golem::DebugRenderer& renderer) const;
	};

	/** Appearance */
	class Appearance {
	public:
		/** Show frame */
		bool showFrames;
		/** Show point cloud */
		bool showPoints;
		/** Frame size of the sample */
		golem::Vec3 frameSize;
		/** clolour of the point cloud */
		golem::RGBA colour;

		/** Bounds colour */
		BoundsAppearance bounds;

		/** Constructs from description object */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showFrames = true;
			showPoints = true;
			frameSize.set(golem::Real(0.02));
			colour = golem::RGBA::MAGENTA;
			bounds.setToDefault();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!bounds.isValid())
				return false;
			if (!frameSize.isPositive())
				return false;
			return true;
		}
	};

	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		Appearance appearance;
		Collision::Desc::Ptr collisionDescPtr;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Hypothesis::Ptr create(const grasp::Manipulator& manipulator) const {
			return Hypothesis::Ptr(new Hypothesis(manipulator, *this));
		}
		/** Sets description to default values */
		void setToDefault() {
			appearance.setToDefault();
			collisionDescPtr.reset(new Collision::Desc());
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!appearance.isValid())
				return false;
			if (collisionDescPtr != nullptr && !collisionDescPtr->isValid())
				return false;
			return true;
		}

	};

	/** Create */
	virtual void create(const golem::U32 idx, const golem::Mat34 &trn, const grasp::RBPose::Sample &s, golem::Rand& rand, const grasp::Cloud::PointSeq& points);

	/** Complete constructor */
	//Hypothesis(const golem::U32 idx, const golem::Mat34 &trn, const grasp::RBPose::Sample &s, grasp::Cloud::PointSeq &p) {
	//	index = idx;
	//	modelFrame = trn;
	//	sample = s;
	//	for (grasp::Cloud::PointSeq::const_iterator i = p.begin(); i != p.end(); ++i)
	//		points.push_back(*i);
	//	appearance.setToDefault();
	//	boundsDesc.setToDefault();
	//	build();
	//	//buildMesh();
	//}
	///** Destrutor */
	//~Hypothesis() {
	//	pTree.release();
	//	pTriangles.release();
	//}


	/** Returns this sample in model frame **/
	inline grasp::RBPose::Sample toRBPoseSample() const { return sample; };
	/** Returns this sample in global frame (default: robot frame) **/
	inline grasp::RBPose::Sample toRBPoseSampleGF() const { return grasp::RBPose::Sample(sample.toMat34() * modelFrame, sample.weight, sample.cdf); };
	/** Returns the point cloud in global frame */
	inline grasp::Cloud::PointSeq getCloud() const { return points; };

	/** Collision detection at a given waypoint */
	inline bool check(const Collision::Waypoint& waypoint, const grasp::Manipulator::Pose& pose, bool debug = false) const {
		return collisionPtr->check(waypoint, pose, debug);
	};
	/** Collision detection at a given waypoint */
	inline bool check(const Collision::FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, bool debug = false) const {
		return collisionPtr->check(desc, rand, pose, debug);
	}

	/** Collision detection at a given waypoint */
	inline virtual golem::Real estimate(const Collision::FlannDesc& desc, const grasp::Manipulator::Pose& pose, golem::Real maxDist = golem::REAL_MAX, bool debug = false) const {
		return collisionPtr->estimate(desc, pose, maxDist, debug);
	}

	/** Collision likelihood estimation at a given waypoint */
	inline golem::Real evaluate(const Collision::Waypoint& waypoint, const grasp::Manipulator::Pose& pose, bool debug = false) const {
		return collisionPtr->evaluate(waypoint, pose, debug);
	}
	/** Collision likelihood estimation at a given waypoint */
	inline golem::Real evaluate(const Collision::FlannDesc& desc, const grasp::Manipulator::Pose& pose, bool debug = false) const {
		return collisionPtr->evaluate(desc, pose, debug);
	}

	/** Return seq of bounds */
	golem::Bounds::Seq bounds();

	/** Prints global pose of the hypothesis */
	std::string str() const;

	/** Draw hypotheses */
	void draw(golem::DebugRenderer &renderer) const;

	/** Draw collisions */
	void draw(const Collision::Waypoint &waypoint, const grasp::Manipulator::Pose& rbpose, golem::DebugRenderer& renderer) const;
	/** Draw collision using kdtree */
	void draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose) const;
	/** Draw estimate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Pose& rbpose, const Collision::FlannDesc& desc) const {
		collisionPtr->draw(renderer, rbpose, desc);
	}
	/** Draw simulate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Pose& rbpose, std::vector<golem::Configspace::Index> &joints, grasp::RealSeq &forces, const Collision::FlannDesc& desc) const {
		collisionPtr->draw(renderer, rbpose, joints, forces, desc);
	}
	Appearance appearance;

protected:
	/** Identifier */
	golem::U32 index;
	/** Model frame **/
	golem::Mat34 modelFrame;
	/** Hypothesis. NOTE: contains the query (or sample) frame w.r.t model frame **/
	grasp::RBPose::Sample sample;
	/** Point cloud */
	grasp::Cloud::PointSeq points;

	/** Bounding box desc for the object */
	golem::BoundingBox::Desc boundsDesc;


	/** Manipulator */
	const grasp::Manipulator& manipulator;
	/** Description */
	const Desc desc;

	/** Collision detection pointer */
	Collision::Ptr collisionPtr;

	/** Create */
	Hypothesis(const grasp::Manipulator& manipulator, const Desc& desc);
};

//------------------------------------------------------------------------------

void XMLData(Hypothesis::Desc& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _SPAM_SPAM_H_ */