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
#ifndef _SPAM_COLLISION_H_
#define _SPAM_COLLISION_H_

//------------------------------------------------------------------------------

#include <Grasp/Core/Cloud.h>
#include <Grasp/Core/Search.h>
#include <Golem/Plan/GraphPlanner.h>
#include <Spam/Core/JContact.h>

//------------------------------------------------------------------------------

namespace flann {
	template <typename T> struct L2_Simple;
};

namespace pcl {
	struct PointXYZ;
	template <typename T, typename Dist> class KdTreeFLANN;
	struct PolygonMesh;
};

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Performance monitor */
#define _COLLISION_PERFMON

//------------------------------------------------------------------------------

/** Forward declaration */
class Hypothesis;

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

		//typedef std::vector<_Bounds> Seq;
		typedef golem::ScalarCoord<_Bounds, golem::Configspace> Coord;

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
			//static size_t jj = 0;
			// if distance true, and there are not collisions, then returns the closest point outside the bounds (negative values) 
			Real depth = distance ? -golem::numeric_const<Real>::MAX : golem::numeric_const<Real>::ZERO; // if no bounds or collisions, no effect
			for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = getDepth(*i, point);
				if (depth < d) // search for maximum depth (can collide only with one mesh within bounds)
					depth = d;
			}
			//if (jj % 100 == 0) printf("getdepth=%f\n", depth);
			return depth;
		}

		inline Real getDepth(const Feature &data, const bool distance = false) const {
			return getDepth((Bounds::Vec3)data.getPoint(), distance);
		}

		/** Get surface distance */
		static inline Real getSurfaceDistance(const typename Triangle::Seq& triangles, const Vec3& point) {
			Real distance = golem::numeric_const<Real>::MAX;
			Real penetration = golem::numeric_const<Real>::ONE;
			for (typename Triangle::Seq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d2 = i->point.distance(point);
				if (distance > d2) // search for minimum distance
					distance = d2;
				// check for penetration
				if (penetration > golem::numeric_const<Real>::ZERO) {
					const Real d = i->distance - i->normal.dot(point);
					if (d < golem::numeric_const<Real>::ZERO) // no collision
						penetration = -golem::numeric_const<Real>::ONE;
				}
			}
			return penetration*distance;
		}
		/** Get surface distance */
		inline Real getSurfaceDistance(const Vec3& point) const {
			// penetration value is positive if there is a penetration
//			const Real penetration = getDepth(point) > REAL_ZERO ? REAL_ONE : -REAL_ONE;
			// distance to the closest surface of this bound
			Real distance = -golem::numeric_const<Real>::MAX;
			for (typename Triangle::SeqSeq::const_iterator i = triangles.begin(); i != triangles.end(); ++i) {
				const Real d = getSurfaceDistance(*i, point);
				if (distance < d) // search for minimum distance
					distance = d;
			}
			//if (jj % 100 == 0) printf("getdepth=%f\n", depth);
			return /*penetration**/distance;
		}
		
		/** Get surface distance: negative is outside the bounds */
		inline Real getSurfaceDistance(const Feature &data) const {
			return getSurfaceDistance((Bounds::Vec3)data.getPoint());
		}

		/** Computes the contribution of each point to calculare the median frame */
		inline Real frameWeight(const Real distance) const {
			return golem::Math::exp(-distance);
		}

		/** Compute minimal distance or avarage penetration */
		template <typename _Ptr> inline _RealEval distance(_Ptr begin, _Ptr end, golem::Vec3& frame, golem::U32& collisions) const {
			golem::Vec3 inFrame, outFrame; inFrame.setZero(); outFrame.setZero();
			_RealEval eval = golem::numeric_const<_RealEval>::ZERO, c = golem::numeric_const<_RealEval>::ZERO, minDist = -golem::numeric_const<Real>::MAX;
			for (_Ptr i = begin; i < end; ++i) {
				const Real distance = getSurfaceDistance(*i);
				if (distance > golem::numeric_const<Real>::ZERO) {
					golem::kahanSum(eval, c, distance);
					inFrame += (*i).getPoint()*frameWeight(distance);
					++collisions;
				}
				else if (minDist < distance) {
					minDist = distance;
					outFrame = (*i).getPoint();
				}
			}
			frame = collisions > 0 ? inFrame : outFrame;
			return collisions > 0 ? eval/collisions : minDist;
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
			Real norm = golem::numeric_const<Real>::ONE / (depthStdDev*golem::Math::sqrt(2 * golem::numeric_const<Real>::PI));
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
					const _RealEval pointEval = norm*golem::Math::exp(-.5*golem::Math::sqr(Real(depth)/Real(depthStdDev))); // gaussian 
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

			return v.z < golem::REAL_ZERO ? pose.p.distance(point) : maxDist; // compute the distance only for point in front of the finger tip
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

		/** Consider a point as a sphere to increase accuracy */
		golem::Real radius;

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

			radius = golem::REAL_ZERO;
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
//	virtual bool check(const Waypoint& waypoint, const grasp::Manipulator::Config& config, bool debug = false) const;
	/** Collision detection using kdtree */
	virtual bool checkNN(const FlannDesc& desc, const grasp::Manipulator::Config& config, bool debug = false) const;

	/** Collision detection for the observational model durin hypopthesis-based planning */
	virtual golem::Real estimate(const FlannDesc& desc, const grasp::Manipulator::Config& config, golem::Real maxDist = golem::REAL_MAX, bool debug = false) const;
	/** Collision detection using kdtree */
//	virtual bool estimate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& pose, bool debug = false) const;

	/** Collision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, std::vector<golem::Configspace::Index>& joints, grasp::RealSeq& forces, bool debug = false) const;
	/** Collision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, FTGuard::Seq& joints, bool debug = false) const;
	/** Collision detection to simulate contact at execution time */
	virtual size_t simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, grasp::RealSeq& forces, bool debug = false) const;
	/** Collision detection to simulate contact at execution time */
	virtual size_t simulateFT(golem::DebugRenderer& renderer, const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, grasp::RealSeq& forces, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint */
	virtual golem::Real evaluate(const Waypoint& waypoint, const grasp::Manipulator::Config& config, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint for belief update */
	virtual golem::Real evaluate(const Waypoint& waypoint, const grasp::Manipulator::Config& config, const FTGuard::Seq& triggeredGuards, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint for belief update */
	virtual golem::Real evaluate(const FlannDesc& desc, const grasp::Manipulator::Config& config, FTGuard::Seq& triggeredGuards, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint for belief update */
	virtual golem::Real evaluateFT(golem::DebugRenderer& renderer, const FlannDesc& desc, const grasp::Manipulator::Config& config, FTGuard::SeqPtr& triggeredGuards, bool debug = false) const;

	/** Collision likelihood estimation at a given waypoint */
	virtual golem::Real evaluate(const FlannDesc& desc, const grasp::Manipulator::Config& config, bool debug = false) const;

	/** Collision likelihood estimation on a given path */
	virtual golem::Real evaluate(const grasp::Manipulator::Waypoint::Seq& path, bool debug = false) const;

	/** Joints bounds */
	inline const Bounds::Coord& getJointBounds() const {
		return jointBounds;
	}
	/** Base bounds */
	inline const Bounds& getBaseBounds() const {
		return baseBounds;
	}
	/** Points */
	//inline const Bounds::Vec3Seq& getPoints() const {
	//	return points;
	//}
	inline const Feature::Seq& getPoints() const {
		return points;
	}


	/** Draw collisions */
	void draw(const Waypoint& waypoint, const grasp::Manipulator::Config& config, golem::DebugRenderer& renderer) const;
	/** Draw collisions with kdtree */
	void draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Config& config, const Collision::FlannDesc& desc) const;
	/** Draw estimate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Config& config, const Collision::FlannDesc& desc) const;
	/** Draw simulate */
	void draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Config& config, std::vector<golem::Configspace::Index> &joints, grasp::RealSeq &forces, const Collision::FlannDesc& desc) const;
	/** Collision likelihood estimation at a given waypoint for belief update */
	void draw(golem::DebugRenderer& renderer, const Waypoint& waypoint, const grasp::Manipulator::Config& config, const FTGuard::Seq &triggeredGuards, bool debug = false) const;

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

	/** Joints with FTs */
	std::vector<golem::Configspace::Index> ftJoints;
	/** Joints */
	Bounds::Coord jointBounds;
	/** FTs */
	Bounds::Coord ftBounds;
	/** Base */
	Bounds baseBounds;
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

}; /** namespace */

#endif /** _SPAM_COLLISION_H_ */
