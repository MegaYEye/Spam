/** @file Data.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Spam/Core/Collision.h>

#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4291 4244 4996 4305)
#endif
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <flann/flann.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#ifdef WIN32
#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

#ifdef _COLLISION_PERFMON
U32 Collision::perfEvalPoints;
U32 Collision::perfCheckNN;
U32 Collision::perfEstimate;
U32 Collision::perfSimulate;
SecTmReal Collision::tperfEvalPoints;
SecTmReal Collision::tperfCheckNN;
SecTmReal Collision::tperfEstimate;
SecTmReal Collision::tperfSimulate;
//U32 Collision::tperfEvalPoints;
//U32 Collision::tperfEvalBelief;
//U32 Collision::tperfEvalRobot;

void Collision::resetLog() {
	perfEvalPoints = 0;
	perfCheckNN = 0;
	perfEstimate = 0;
	perfSimulate = 0;
	tperfEvalPoints = SEC_TM_REAL_ZERO;
	tperfCheckNN = SEC_TM_REAL_ZERO;
	tperfEstimate = SEC_TM_REAL_ZERO;
	tperfSimulate = SEC_TM_REAL_ZERO;
}

void Collision::writeLog(Context &context, const char *str) {
	context.write(
		"%s: Collision::collision_{point cloud, [sec], checkNN, [sec], estimate, [sec], simulate, [sec]} = {%u, %f, %u, %f, %u, %f, %u, %f}\n", str, perfEvalPoints, tperfEvalPoints / (perfEvalPoints > 0 ? perfEvalPoints : 1), 
		perfCheckNN, tperfCheckNN / (perfCheckNN > 0 ? perfCheckNN : 1), 
		perfEstimate, tperfEstimate / (perfEstimate > 0 ? perfEstimate : 1), 
		perfSimulate, tperfSimulate / (perfSimulate > 0 ? perfSimulate : 1)
		);
}

#endif

//------------------------------------------------------------------------------

static Vec3 getRelativeFrame(const golem::Mat34 &reference, const golem::Vec3 &point) {
	Vec3 v;
	Mat34 inverse;
	inverse.setInverse(reference);
	inverse.multiply(v, point);
	return v;// .normalise();
}

//------------------------------------------------------------------------------

Collision::Collision(const grasp::Manipulator& manipulator, const Desc& desc) : manipulator(manipulator), desc(desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "Collision(): invalid description");

	nnSearch.reset();

	// joints - hand only
	for (golem::Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i)
		jointBounds[i].create(manipulator.getJointBounds(i));

	// ft only
	for (golem::Chainspace::Index i = manipulator.getHandInfo().getChains().begin(); i != manipulator.getHandInfo().getChains().begin(); ++i) {
		const golem::Configspace::Index j = manipulator.getHandInfo().getJoints(i).end() - 1;
		ftBounds[j].create(manipulator.getJointBounds(j));
		ftJoints.push_back(j);
	}

	// base
	baseBounds.create(manipulator.getBaseBounds());
	// bounds
	/*bounds.resize(grasp::Manipulator::JOINTS + 1);
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i)
		bounds[i].create(i < manipulator.getJoints() ? manipulator.getJointBounds(i) : manipulator.getBaseBounds());*/
}

void Collision::create(golem::Rand& rand, const grasp::Cloud::PointSeq& points) {
	// points
	this->points.clear();
	this->points.reserve(points.size());

	for (grasp::Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i)
		this->points.push_back(Feature(grasp::Cloud::getPoint<Real>(*i)));

	std::random_shuffle(this->points.begin(), this->points.end(), rand);

	flann::SearchParams search;
	flann::KDTreeSingleIndexParams index;
	search.checks = 32;
	search.max_neighbors = 10;
	desc.nnSearchDesc.getKDTreeSingleIndex(search, index);
		
	typedef grasp::KDTree<golem::Real, Feature::FlannDist, flann::SearchParams> KDTree;
	nnSearch.reset(new KDTree(search, index, this->points, Feature::N, Feature::FlannDist()));
	
	//if (desc.kdtree) {
	//	flann::SearchParams search;
	//	flann::KDTreeSingleIndexParams index;
	//	search.checks = 32;
	//	search.max_neighbors = 10;
	//	desc.nnSearchDesc.getKDTreeSingleIndex(search, index);
	//	typedef grasp::KDTree<golem::Real, Feature::FlannDist, flann::SearchParams> KDTree;
	//	nnSearch.reset(new KDTree(search, index, this->points, Feature::N, Feature::FlannDist()));
	//}
	//else {
	//	std::random_shuffle(this->points.begin(), this->points.end(), rand);
	//}
}

//------------------------------------------------------------------------------

size_t Collision::simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, std::vector<Configspace::Index> &joints, grasp::RealSeq &forces, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif
	joints.clear();
	forces.clear();

	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord wjoints;
	manipulator.getJointFrames(config.config, base, wjoints);
	/*const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);*/

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		bounds.setPose(Bounds::Mat34(wjoints[i]));
		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(wjoints[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);

			if (depth > REAL_ZERO) {
#ifdef _COLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().debug(
					"Collision::check(kd-tree): time_elapsed = %f [sec], collision=yes, neighbours=%u, points=%u\n",
					t_end, desc.neighbours, desc.points
					);
#endif
				force += depth;
				frame.add(frame, getRelativeFrame(wjoints[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");

		}
		if (collisions > 0) {
			joints.push_back(Configspace::Index(i));
			force /= collisions;
			frame /= collisions;
			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force * 1000);
		}
	}

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif

	return joints.size();
}

size_t Collision::simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, FTGuard::Seq &joints, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif
	joints.clear();

	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		bounds.setPose(Bounds::Mat34(poses[i]));
		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);

			if (depth > REAL_ZERO) {
#ifdef _COLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().write(
					"Collision::check(kd-tree): time_elapsed = %f [sec], collision=yes, depth=%f, neighbours=%u, points=%u\n",
					t_end, depth, desc.neighbours, desc.points
					);
#endif
				if (depth > maxDepth) maxDepth = depth;
				force += depth;
				frame.add(frame, getRelativeFrame(poses[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");
		}
		if (collisions > 0) {
			const size_t k = i - manipulator.getHandInfo().getJoints().begin();
			FTGuard guard(manipulator);
			guard.create(Configspace::Index(i));
			force /= collisions;
			frame /= collisions;
			guard.force = Math::sign(REAL_ONE, -frame.z) * (this->desc.ftContact.ftMedian[k] + (2 * rand.nextUniform<Real>()*this->desc.ftContact.ftStd[k] - this->desc.ftContact.ftStd[k]));//* force * 0.1/* * 1000*/;
			joints.push_back(guard);
//			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force * 1000);
		}
	}

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().write("Collision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif
	if (maxDepth > REAL_ZERO) manipulator.getContext().write("Simulate contact max depth = %f\n", maxDepth);
	return joints.size();
}

size_t Collision::simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, grasp::RealSeq &forces, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif

	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	size_t collided = 0;
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);

			if (depth > REAL_ZERO) {
#ifdef _COLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug && false/*&& eval < REAL_ONE*/)
					manipulator.getContext().write(
					"Collision::check(kd-tree): time_elapsed = %f [sec], collision=yes, depth=%f, neighbours=%u, points=%u\n",
					t_end, depth, desc.neighbours, desc.points
					);
#endif
				if (depth > maxDepth) maxDepth = depth;
				force += depth;
				frame.add(frame, getRelativeFrame(poses[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");
		}
		if (collisions > 0) {
//			FTGuard guard(manipulator);
//			guard.create(Configspace::Index(i));
//			force /= collisions;
//			frame /= collisions;
			const size_t k = i - manipulator.getHandInfo().getJoints().begin();
			forces[k] = debug ? (Math::sign(REAL_ONE, -frame.z) * force / collisions)*10000 : Math::sign(REAL_ONE, -frame.z) * (this->desc.ftContact.ftMedian[k] + (2 * rand.nextUniform<Real>()*this->desc.ftContact.ftStd[k] - this->desc.ftContact.ftStd[k]));//* force * 0.1/* * 1000*/;
			//if (debug) manipulator.getContext().write("Collision joint=%d finger=%d link=%d force=%3.3f\n", i, U32(k/4)+1, U32(k%4), forces[k]);
			++collided;
//			joints.push_back(guard);
			//			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force * 1000);
		}
	}

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfSimulate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug && false/*&& eval < REAL_ONE*/)
		manipulator.getContext().write("Collision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif
//	if (maxDepth > REAL_ZERO) manipulator.getContext().write("Simulate contact max depth = %f\n", maxDepth);
	return collided;// joints.size();
}

//------------------------------------------------------------------------------

bool Collision::check(const Collision::Waypoint& waypoint, const grasp::Manipulator::Config& config, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
//	++perfCheckPoints;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));

		for (size_t j = 0; j < size; ++j) {
			const Real depth = bounds.getDepth(points[j]/*.getPoint()*/);
			if (depth > REAL_ZERO) {
#ifdef _COLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
//				tperfCheckPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().debug(
					"Collision::check(Waypoint): time_elapsed = %f [sec], points=%u, collision=yes\n",
					t_end, size
					);
#endif
				return true;
			}
		}
	}

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
//	tperfCheckPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, collision=no\n", size);
#endif

	return false;
}

bool Collision::check(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Config& config, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return false;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfCheckNN;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;
	// Collision consider a point as a sphere
//	const Real radius = -0.005; //REAL_ZERO; // -Real(0.0001);
////	manipulator.getContext().write("radius %f\n", radius);

	const size_t size = /*desc.points < desc.neighbours ? desc.points : */desc.neighbours;
	Real maxDepth = -golem::numeric_const<Real>::MAX, maxDistance = -golem::numeric_const<Real>::MAX;
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
				
		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);
		
		for (size_t j = 0; j < size; ++j) {
			const Feature f = /*desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : */points[indices[j]];
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");
			const Real depth = REAL_ZERO; // bounds.getDepth(f, true);
			const Real distance = bounds.getSurfaceDistance(f);//bounds.getDepth(f, true);
//			if (debug && maxDepth < depth) maxDepth = depth;
			if (debug && maxDistance < distance) maxDistance = distance;
			if (distance > -desc.radius) {
#ifdef _COLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfCheckNN += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().write(
					"Collision::check(kd-tree): time_elapsed = %f [sec], collision=yes, depth=%.7f, distance=%.7f, neighbours=%u, points=%u\n",
					t_end, depth, distance, desc.neighbours, desc.points
					);
#endif
				return true;
			}
		}
	}

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfCheckNN += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().write("Collision::evaluate(): depth=%.7f, distance=%f, neighbours=%u, points=%u, collision=no\n", maxDepth, maxDistance, desc.neighbours, desc.points);
#endif

	return false;
}

//------------------------------------------------------------------------------

golem::Real Collision::estimate(const FlannDesc& desc, const grasp::Manipulator::Config& config, golem::Real maxDist, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEstimate;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < desc.points ? points.size() : desc.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	// iterates only for finger tips
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		eval += bounds.estimate(poses[i], points.data(), points.data() + size, (Bounds::RealEval)desc.depthStdDev, collisions, maxDist);
		checks += size;
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEstimate += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::estimate(): points=%u, checks=%u, collisions=%u, eval=%f, likelihhod=%f\n", size, checks, collisions, eval, likelihood);
#endif

	return likelihood;
}

//------------------------------------------------------------------------------

golem::Real Collision::evaluate(const Waypoint& waypoint, const grasp::Manipulator::Config& config, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		eval += bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, collisions);
		checks += size;
	}

	const Real likelihood = waypoint.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", size, checks, collisions, likelihood);
#endif

	return likelihood;
}

golem::Real Collision::evaluate(const Waypoint& waypoint, const grasp::Manipulator::Config& config, const FTGuard::Seq &triggeredGuards, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;
//	manipulator.getContext().write("Collision::evaluate():\n");
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));

		Real force = REAL_ZERO;
		const bool triggered = [&]() -> const bool {
			for (auto guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
				if (i == guard->jointIdx) {
					force = guard->force;
					return true;
				}
			}
			return false;
		}();
		const Real val = bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, (Bounds::RealEval)waypoint.depthStdDev, triggered, poses[i], force, collisions);
//		manipulator.getContext().write("hand link %u, triggered = %u, eval = %f\n", i, triggered, val);
		eval += val;
		checks += size;
	}

	const Real likelihood = waypoint.likelihood*eval/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", size, checks, collisions, likelihood);
#endif

	return likelihood;
}

//golem::Real Collision::evaluate(const FlannDesc& desc, const grasp::Manipulator::Pose& rbpose, FTGuard::Seq &triggeredGuards, bool debug) const {
//	if (!this->desc.kdtree) {
//		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
//		// todo: check this is a valid value to return in case of faliure
//		return REAL_ZERO;
//	}
//#ifdef _COLLISION_PERFMON
//	PerfTimer t;
//	++perfEvalPoints;
//#endif
//	const golem::Mat34 pose(rbpose.toMat34());
//	golem::Mat34 poses[grasp::Manipulator::JOINTS];
//	manipulator.getPoses(rbpose, pose, poses);
//
//	grasp::NNSearch::IndexSeq indices;
//	grasp::NNSearch::DistanceF64Seq distances;
//
//	golem::Real eval = REAL_ZERO;
//	size_t collisions = 0, checks = 0;
//
//	for (U32 i = manipulator.getArmJoints()+3; i < manipulator.getJoints();) {
//		Bounds bounds = this->bounds[i];
//		if (bounds.empty())
//			continue;
//
//		bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));
//		Real force = REAL_ZERO;
//		const bool triggered = [&]() -> const bool {
//			for (auto guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
//				if (/*i == guard->jointIdx*/guard->jointIdx > i - 3 && guard->jointIdx < i) {
//					force = guard->force;
//					return true;
//				}
//			}
//			return false;
//		}();
//
//		Feature query(poses[i].p);
//		nnSearch->knnSearch(query, desc.neighbours, indices, distances);
//
//		Feature::Seq seq;
//		seq.reserve(indices.size());
//		for (size_t j = 0; j < indices.size(); ++j)
//			seq.push_back(points[indices[j]]);
//
//		eval += bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)0.0008, (Bounds::RealEval)desc.depthStdDev * 100, triggered, i < manipulator.getJoints() ? poses[i] : pose, force, collisions);
//		checks += triggered ? indices.size() : 0;
//		i += 4;
//	}
//
//	const Real likelihood = desc.likelihood*eval/collisions/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;
//
//#ifdef _COLLISION_PERFMON
//	SecTmReal t_end = t.elapsed();
//	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
//	if (debug /*&& eval < REAL_ONE*/)
//		manipulator.getContext().write("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f, eval=%f\n", indices.size(), checks, collisions, likelihood, eval/collisions);
//#endif
//
//	return likelihood;
//}

golem::Real Collision::evaluate(const FlannDesc& desc, const grasp::Manipulator::Config& config, FTGuard::Seq &triggeredGuards, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	golem::Real eval = REAL_ZERO, c = REAL_ZERO;
	const Real norm = golem::numeric_const<Real>::ONE / ((desc.depthStdDev * 100) * Math::sqrt(2 * golem::numeric_const<Real>::PI));
	size_t collisions = 0, checks = 0;
	static size_t debugjj = 0;

	const U32 jointPerFinger = manipulator.getController().getChains()[manipulator.getHandInfo().getChains().begin()]->getJoints().size();
	const U32 fingers = manipulator.getHandInfo().getChains().size();
	enum jointIndex {
		abductor = 0,
		joint1,
		joint2
	};
	// Utilities
	std::vector<bool> triggeredFingers; triggeredFingers.assign(fingers, false);
	std::vector<grasp::RealSeq> fingerGuardSeq;
	fingerGuardSeq.resize(fingers);
	for (std::vector<grasp::RealSeq>::iterator i = fingerGuardSeq.begin(); i != fingerGuardSeq.end(); ++i)
		i->assign(jointPerFinger, REAL_ZERO);
	//typedef std::map<U32, FTGuard> GuardMap; GuardMap guardMap;
	typedef std::map<U32, grasp::RealSeq> GuardMap; GuardMap guardMap;
	// retrive trigguered guards
	for (FTGuard::Seq::iterator guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
		triggeredFingers[guard->getHandChain() - 1] = true;
		const U32 k = U32(0 - guard->jointIdx);
		fingerGuardSeq[guard->getHandChain() - 1][guard->getHandJoint()] = guard->force;
		//manipulator.getContext().write("guard map key %d\n", -k);
//		guardMap.insert(guardMap.begin(), GuardMap::value_type(-k, *guard));
	}
//	if (debugjj % 100 == 0) manipulator.getContext().write("triggered finger [%s,%s,%s,%s,%s], guardMap size %d\n", triggeredFingers[0] ? "T" : "F", triggeredFingers[1] ? "T" : "F", triggeredFingers[2] ? "T" : "F", triggeredFingers[3] ? "T" : "F", triggeredFingers[4] ? "T" : "F", guardMap.size());

	// loops over fingers: thumb = 0,..., pinky = 4
	for (Chainspace::Index j = manipulator.getHandInfo().getChains().begin(); j < manipulator.getHandInfo().getChains().end(); ++j) {
		// thumb: begin=7, end=10
		const Configspace::Index begin = manipulator.getInfo().getJoints(j).begin(), end = manipulator.getInfo().getJoints(j).end();
		const U32 finger = j - manipulator.getHandInfo().getChains().begin();
		// finger frame
		Mat34 fingerFrameInv; fingerFrameInv.setInverse(poses[end - 1]);
		// abductor has no bounds. Retrive contact there at the beginning
		const bool triggeredAbductor = Math::abs(fingerGuardSeq[finger][jointIndex::abductor]) > REAL_ZERO;//false;
		const bool triggeredFlex = Math::abs(fingerGuardSeq[finger][jointIndex::joint1]) > REAL_ZERO || Math::abs(fingerGuardSeq[finger][jointIndex::joint2]) > REAL_ZERO;
		//GuardMap::const_iterator abductor = guardMap.find(begin);
		//if (abductor != guardMap.end())
		//	triggeredAbductor = true;
//		if (debugjj % 100 == 0) manipulator.getContext().write("abductor %d\n", triggeredAbductor);

		for (Configspace::Index i = begin; i < end; ++i) {
			Bounds bounds = jointBounds[i];
			if (bounds.empty())
				continue;

			bounds.setPose(Bounds::Mat34(poses[i]));
			
			// extract the closest point to the joint's bounds as feature
			Feature query(poses[i].p);
			nnSearch->knnSearch(query, desc.neighbours, indices, distances);

			Feature::Seq seq;
			seq.reserve(indices.size());
			for (size_t j = 0; j < indices.size(); ++j)
				seq.push_back(points[indices[j]]);
			
			Real maxDepth = -golem::numeric_const<Real>::MAX;
			U32 boundCollisions = golem::numeric_const<U32>::ZERO;
//			Real force = REAL_ZERO;
//			GuardMap::const_iterator jointGuard = guardMap.find(i);
//			if (jointGuard != guardMap.end()) force = jointGuard->second.force;
////			if (debugjj % 100 == 0) manipulator.getContext().write("joint %d, triggered %s, torque %d\n", i, jointGuard != guardMap.end() ? "T" : "F", force);

			//golem::Mat34 inverse; inverse.setInverse(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose)); // compute the inverse of the joint frame
			Vec3 median;
			median.setZero();
			//for (size_t j = 0; j < seq.size(); ++j) {
			//	const Feature f = seq[j];
			//	const Real depth = bounds.getSurfaceDistance(f);// bounds.getDepth(f, true);
			//	median += f.getPoint();
			//	if (depth > REAL_ZERO) ++boundCollisions; // number of collisions with this bound
			//	if (depth > maxDepth) {
			//		maxDepth = depth; // record max depth
			//	}
			//}
			maxDepth = bounds.distance(seq.data(), seq.data() + seq.size(), median, boundCollisions);

			if (!triggeredFingers[finger] && maxDepth > REAL_ZERO) // the hypothesis intersects a finger that has no contact retrieved
				return REAL_ZERO;
			// if no contact is retrieve and there is no intersection, don't change eval

			// if contact is retrieved
			Real pointEval = REAL_ZERO;
			if (triggeredFingers[finger]) {
				//median /= seq.size();
				Vec3 patchPose; fingerFrameInv.multiply(patchPose, median);
				// abd direction is true if:
				//   1. patchpose is the y-axis side to produce the observed direction of force. (e.g. y>0 && force<0)
				//   2. there is no observed direction (so the patch could be anywhere)
				const bool abdDirection = triggeredAbductor ? !((patchPose.x > REAL_ZERO && fingerGuardSeq[finger][jointIndex::abductor] > REAL_ZERO) || (patchPose.x < REAL_ZERO && fingerGuardSeq[finger][jointIndex::abductor] < REAL_ZERO)) : true;
				const bool flexDirection = triggeredFlex ? !((patchPose.z > REAL_ZERO && (fingerGuardSeq[finger][jointIndex::joint1] > REAL_ZERO || fingerGuardSeq[finger][jointIndex::joint2] > REAL_ZERO)) || (patchPose.z < REAL_ZERO && (fingerGuardSeq[finger][jointIndex::joint1] < REAL_ZERO || fingerGuardSeq[finger][jointIndex::joint2] < REAL_ZERO))) : true;
				const Real scalingFac = abdDirection && flexDirection ? 1 : 0.01;
				//// computes the direction of contact
				//golem::Vec3 v; inverse.multiply(v, median); //v.normalise(); // compute the point in the joint's reference frame
				//const bool adbDirection = triggeredAbductor ? !((v.y > 0 && abductor->second.force >= 0) || (v.y < 0 && abductor->second.force <= 0)) : true;
				//const bool flexDirection = Math::abs(force) > REAL_ZERO ? !((v.z > 0 && force > 0) || (v.z < 0 && force < 0)) : true;
				//const Real coef = Math::abs(maxDepth) >= 1 ? 1 / Math::abs(maxDepth) : Math::abs(maxDepth);
				////if (debugjj % 100 == 0) manipulator.getContext().write("coef %f\n", coef);
				//const Real direction = adbDirection && flexDirection ? 1 /*Math::exp(Real(maxDepth))*/ : Math::exp(Real(maxDepth - 0.01));
				////if (debugjj % 100 == 0) manipulator.getContext().write("direction %f\n", direction);

				//// there is penetration
				//if (maxDepth > REAL_ZERO)
				//	pointEval = Math::exp(-Real(desc.depthStdDev/* * 100*/)*(Real(maxDepth)));
				//else // maxDepth here has to be <= 0. no penetration.
				pointEval = Math::abs(maxDepth) > desc.depthStdDev*3 ? REAL_ZERO : scalingFac*norm*golem::Math::exp(-.5*Math::sqr(Real(maxDepth) / Real(desc.depthStdDev/* * 100*/))); // gaussian 
				//if (debugjj++ % 100 == 0) manipulator.getContext().write("PointEval %f adbDirection %s, flexDirection %s, direction %f\n", pointEval, adbDirection ? "T" : "F", flexDirection ? "T" : "F", direction);
				golem::kahanSum(eval, c, pointEval);
			}
			//if (debugjj++ % 100 == 0)  
			//	manipulator.getContext().write("joint %d [%s, %d] depth %f, eval %f, Lx %f, x %f, collisions %d\n", i, jointGuard != guardMap.end() ? "T" : "F", force, maxDepth, pointEval, 
			//	maxDepth > REAL_ZERO ? -Real(desc.depthStdDev * 100)*(maxDepth) : -.5*Math::sqr(Real(maxDepth) / Real(desc.depthStdDev * 100)), Real(maxDepth), boundCollisions);
			collisions += boundCollisions;
		}
	}

	const Real likelihood = desc.likelihood*eval;// / collisions/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	//if (debug /*&& eval < REAL_ONE*/)
//	if (debugjj++ % 100 == 0)	manipulator.getContext().write("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f, eval=%f\n", indices.size(), checks, collisions, likelihood, eval / collisions);
#endif

	return likelihood;
}

golem::Real Collision::evaluateFT(const FlannDesc& desc, const grasp::Manipulator::Config& config, FTGuard::Seq &triggeredGuards, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (auto i = ftJoints.begin(); i < ftJoints.end(); ++i) {
		if (*i != triggeredGuards[0].jointIdx)
			continue;

		Bounds bounds = jointBounds[*i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[*i]));

		Feature query(poses[*i].p);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Feature::Seq seq;
		seq.reserve(indices.size());
		for (size_t j = 0; j < indices.size(); ++j)
			seq.push_back(points[indices[j]]);

		eval += bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)desc.depthStdDev, collisions);
		checks += indices.size();
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", indices.size(), checks, collisions, likelihood);
#endif

	return likelihood;
}

golem::Real Collision::evaluate(const FlannDesc& desc, const grasp::Manipulator::Config& config, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));

		Feature query(poses[i].p);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Feature::Seq seq;
		seq.reserve(indices.size());
		for (size_t j = 0; j < indices.size(); ++j)
			seq.push_back(points[indices[j]]);

		eval += bounds.evaluate(seq.data(), seq.data() + seq.size(), (Bounds::RealEval)desc.depthStdDev, collisions);
		checks += indices.size();
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", indices.size(), checks, collisions, likelihood);
#endif

	return likelihood;
}

golem::Real Collision::evaluate(const grasp::Manipulator::Waypoint::Seq& path, bool debug) const {
	golem::Real eval = REAL_ZERO;
	for (Waypoint::Seq::const_iterator i = desc.waypoints.begin(); i != desc.waypoints.end(); ++i)
		eval += evaluate(*i, manipulator.interpolate(path, i->pathDist), debug);
	return eval;
}

//------------------------------------------------------------------------------

void Collision::draw(const Waypoint& waypoint, const grasp::Manipulator::Config& config, golem::DebugRenderer& renderer) const {
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	renderer.setColour(RGBA::BLACK);

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));
		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		for (size_t j = 0; j < size; ++j) {
			renderer.addPoint(points[j].getPoint());
			if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> distance=%f collision=%s\n", j, points[j].getPoint().x, points[j].getPoint().y, points[j].getPoint().z, bounds.getDepth(points[j]), bounds.getDepth(points[j]) > REAL_ZERO ? "YES" : "NO"
				/*indices[j], distances[j], poses[i].p.distance(points[indices[j]].getPoint())*/);
		}
		// debug: run only for one joint.
		break;
	}

}

void Collision::draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Config& config, const Collision::FlannDesc& desc) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return;
	}
	PerfTimer t;

	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	renderer.setColour(RGBA::BLACK);
	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));
		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);
//		manipulator.getContext().write("neighbours=%d, points=%d\n", desc.neighbours, desc.points);
		Feature query(poses[i].p);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

//		nnSearch->knnSearch((const Real*)&poses[i].p, neighbours, indices, distances);

//		for (size_t j = 0; j < indices.size(); ++j) {
		for (size_t j = 0; j < size; ++j) {
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			const Real depth = bounds.getDepth(f);
			if (depth > REAL_ZERO) renderer.addPoint(f.getPoint());
			manipulator.getContext().debug("depth=%f\n", depth);
			if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
				indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(points[j]) > REAL_ZERO ? "YES" : "NO");
		}

		// debug: run only for one joint.
//		if (i == manipulator.getArmJoints() + 3)
			break;
	}
	SecTmReal t_end = t.elapsed();
	manipulator.getContext().write("Collision::draw(): elapse_time=%f neighbours=%u points=%u\n", t_end, desc.neighbours, desc.points);
}

void Collision::draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Config& config, const Collision::FlannDesc& desc) const {
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < desc.points ? points.size() : desc.points;

	renderer.setColour(RGBA::BLACK);

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	// iterates only for finger tips
	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));
		eval += bounds.estimate(poses[i], points.data(), points.data() + size, (Bounds::RealEval)desc.depthStdDev, collisions, 0.5);
		checks += size;
		
		for (auto k = 0; k < size; ++k)
			renderer.addPoint(points[k].getPoint());
		break;
	}

	const Real likelihood = desc.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);
	manipulator.getContext().write("Collision::draw(): neighbours=%u points=%u\n", desc.neighbours, desc.points);
}

void Collision::draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Config& config, std::vector<golem::Configspace::Index> &joints, grasp::RealSeq &forces, const Collision::FlannDesc& desc) const {
	joints.clear();
	forces.clear();

	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;
	renderer.setColour(RGBA::BLACK);

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));

		//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);

		Real force = REAL_ZERO;
		Vec3 frame(.0, .0, .0);
		size_t collisions = 0;
		for (size_t j = 0; j < size; ++j) {
			//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = points[indices[j]];
			const Real depth = bounds.getDepth(f);
			manipulator.getContext().debug("depth=%f\n", depth);
			if (depth > REAL_ZERO) {
				renderer.addPoint(f.getPoint());
				force += depth;
				frame.add(frame, getRelativeFrame(poses[i], f.point));
				//Vec3 w; poses[i].multiply(w, frame);
				//const Vec3 p[2] = { poses[i].p, w };
				//renderer.addLine(p[0], p[1], RGBA::MAGENTA);
				++collisions;
			}
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");

		}
		if (collisions > 0) {
			joints.push_back(Configspace::Index(i));
			force /= collisions;
			frame /= collisions;
			forces.push_back(Math::sign(REAL_ONE, -frame.z)*force*1000);
			Vec3 w; poses[i].multiply(w, frame);
			const Vec3 p[2] = { poses[i].p, w };
			renderer.addLine(p[0], p[1], RGBA::MAGENTA);
		}
		manipulator.getContext().debug("Collision::evaluate(): neighbours=%u, points=%u, collision=%u, force=%f (%f), dir=<%f, %f, %f>\n", desc.neighbours, desc.points, collisions,
			Math::sign(REAL_ONE, -frame.z)*force*100, force, frame.x, frame.y, frame.z);
		break;
	}


}

void Collision::draw(golem::DebugRenderer& renderer, const Waypoint& waypoint, const grasp::Manipulator::Config& config, const FTGuard::Seq &triggeredGuards, bool debug) const {
	const golem::Mat34 base(config.frame.toMat34());
	golem::WorkspaceJointCoord poses;
	manipulator.getJointFrames(config.config, base, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (Configspace::Index i = manipulator.getHandInfo().getJoints().begin(); i < manipulator.getHandInfo().getJoints().end(); ++i) {
		Bounds bounds = jointBounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(poses[i]));
		renderer.addAxes(poses[i], Vec3(.05, .05, .05));

		Real force = REAL_ZERO;
		const bool triggered = [&]() -> const bool {
			for (auto guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard)
				if (i == (0 - guard->jointIdx)) {
					force = guard->force;
					return true;
				}
			return false;
		}();
		const Real val = bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, (Bounds::RealEval)waypoint.depthStdDev, triggered, poses[i], force, collisions);
		eval += val;
		checks += size;
	}

	const Real likelihood = waypoint.likelihood*((eval + (checks - collisions)) / checks - REAL_ONE);

	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f\n", size, checks, collisions, likelihood);

}

//------------------------------------------------------------------------------

void spam::XMLData(spam::Collision::Waypoint& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("path_dist", val.pathDist, xmlcontext, create);
	golem::XMLData("points", val.points, xmlcontext, create);
	golem::XMLData("depth_stddev", val.depthStdDev, xmlcontext, create);
	golem::XMLData("likelihood", val.likelihood, xmlcontext, create);
}

void spam::XMLData(spam::Collision::FlannDesc& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("neighbours", val.neighbours, xmlcontext, create);
	golem::XMLData("points", val.points, xmlcontext, create);
	golem::XMLData("depth_stddev", val.depthStdDev, xmlcontext, create);
	golem::XMLData("likelihood", val.likelihood, xmlcontext, create);
	
	try {
		golem::XMLData("radius", val.points, xmlcontext, create);
	}
	catch (const Message&) {}
}

void spam::XMLData(spam::Collision::FTSensorDesc& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData(val.ftMedian, xmlcontext->getContextFirst("ft_median"), create);
	golem::XMLData(val.ftStd, xmlcontext->getContextFirst("ft_std"), create);
}

void spam::XMLData(spam::Collision::Desc& val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData(val.waypoints, val.waypoints.max_size(), xmlcontext, "waypoint", create);
	spam::XMLData(val.flannDesc, xmlcontext->getContextFirst("kdtree"), create);
	try {
		spam::XMLData(val.ftBase, xmlcontext->getContextFirst("ft_base"), create);
		spam::XMLData(val.ftContact, xmlcontext->getContextFirst("ft_contact"), create);
	}
	catch (const golem::MsgXMLParser& msg) { }
}