/** @file Data.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Spam/Spam/Spam.h>
//#include <Spam/Spam/Robot.h>
#include <Spam/Spam/Belief.h>
#include <Golem/Ctrl/Data.h>

//#ifdef WIN32
//	#pragma warning (push)
//	#pragma warning (disable : 4244)
//	#pragma warning (disable : 4996)
//#endif
//#include <pcl/point_types.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#ifdef WIN32
//	#pragma warning (pop)
//#endif
//
//#include <boost/lexical_cast.hpp>

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

static std::string getGuardName(const U32 finger) {
	if (finger == HandChains::UNKNOWN) return "UNKNOWN";
	if (finger == HandChains::THUMB) return "THUMB";
	if (finger == HandChains::INDEX) return "INDEX";
	if (finger == HandChains::MIDDLE) return "MIDDLE";
	if (finger == HandChains::RING) return "RING";
	if (finger == HandChains::PINKY) return "PINKY";
	return "";
}

static HandChains bound2Chain(const U32 finger) {
	if (7 < finger && finger < 10)
		return HandChains::THUMB;
	if (11 < finger && finger < 14)
		return HandChains::INDEX;
	if (15 < finger && finger < 18)
		return HandChains::MIDDLE;
	if (19 < finger && finger < 22)
		return HandChains::RING;
	if (22 < finger && finger < 25)
		return HandChains::PINKY;
	return HandChains::UNKNOWN;

}
//------------------------------------------------------------------------------

FTGuard::FTGuard(const grasp::Manipulator& manipulator) {
	// arm joint indices [0, 6]
	armIdx = Configspace::Index(manipulator.getArmJoints());
	handChains = manipulator.getHandChains();
	fingerJoints = manipulator.getHandJoints() / handChains;
//	manipulator.getContext().write("FTGuard armJoints=%u, handChains=%u\n", armIdx, handChains);
}

std::string FTGuard::str() const {
	std::string ss = getGuardName(getHandChain()) + " joint=" + boost::lexical_cast<std::string>(getHandJoint()) + " measured_force=" + boost::lexical_cast<std::string>(force) + (type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + boost::lexical_cast<std::string>(threshold);
	printf("%s\n", ss.c_str());
	return ss;
}

void FTGuard::create(golem::Configspace::Index& i) {
	jointIdx = i;
	const size_t handIdx = i - armIdx;
	threshold = 0.2;
	force = REAL_ZERO;
	type = FTGUARD_ABS;
//	printf("FTGuard(armJoints=%u, handChains=%u, fingerJoints=%u) chain=%u, chain joint=%u, joint=%u\n", armIdx, handChains, fingerJoints, (handIdx / fingerJoints) + 1, handIdx % fingerJoints, i);
}

void XMLData(FTGuard &val, XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", val.name, xmlcontext, create);
//	golem::XMLData("joint", val.joint, xmlcontext, create);
//	golem::XMLData("type", val.type, xmlcontext, create);
//	golem::XMLData("value", val.value, xmlcontext, create);
}

//------------------------------------------------------------------------------

Collision::Collision(const grasp::Manipulator& manipulator, const Desc& desc) : manipulator(manipulator), desc(desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "Collision(): invalid description");

	nnSearch.reset();

	// bounds
	bounds.resize(grasp::Manipulator::JOINTS + 1);
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i)
		bounds[i].create(i < manipulator.getJoints() ? manipulator.getJointBounds(i) : manipulator.getBaseBounds());
}

void Collision::create(golem::Rand& rand, const grasp::Cloud::PointSeq& points) {
	// points
	this->points.clear();
	this->points.reserve(points.size());

	for (grasp::Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i)
		this->points.push_back(Feature(grasp::Cloud::getPoint<Real>(*i)));

	std::random_shuffle(this->points.begin(), this->points.end(), rand);

	//if (desc.kdtree) {
	flann::SearchParams search;
		flann::KDTreeSingleIndexParams index;
//		desc.nnSearchDesc.searchKDTrees = 4;
		search.checks = 32;
		search.max_neighbors = 10;
		desc.nnSearchDesc.getKDTreeSingleIndex(search, index);
		typedef grasp::KDTree<golem::Real, Feature::FlannDist, flann::SearchParams> KDTree;

//		nnSearch.reset(new KDTree(search, index, (Real*)&this->points.front(), this->points.size(), sizeof(golem::Real), Collision::FlannDist::LIN_N, Collision::FlannDist()));
		nnSearch.reset(new KDTree(search, index, this->points, Feature::N, Feature::FlannDist()));
		//}
	//else {
	//	std::random_shuffle(this->points.begin(), this->points.end(), rand);
	//}
}

//------------------------------------------------------------------------------

size_t Collision::simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, std::vector<Configspace::Index> &joints, grasp::RealSeq &forces, bool debug) const {
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

	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints(); ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
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
					manipulator.getContext().debug(
					"Collision::check(kd-tree): time_elapsed = %f [sec], collision=yes, neighbours=%u, points=%u\n",
					t_end, desc.neighbours, desc.points
					);
#endif
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

size_t Collision::simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, FTGuard::Seq &joints, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif
	joints.clear();

	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints(); ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
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
			const size_t k = i - manipulator.getArmJoints();
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

size_t Collision::simulate(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, grasp::RealSeq &forces, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfSimulate;
#endif

	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	Real maxDepth = REAL_ZERO;
	size_t collided = 0;
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints(); ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
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
			const size_t k = i - manipulator.getArmJoints();
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

bool Collision::check(const Collision::Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
//	++perfCheckPoints;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty()) {
//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);

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

bool Collision::check(const FlannDesc& desc, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return false;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfCheckNN;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty()) {
//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
//		manipulator.getContext().write("-----------------------------\nBound pose <%f, %f, %f> (<%f, %f, %f>)\n", poses[i].p.x, poses[i].p.y, poses[i].p.z, poses[i].p.v[0], poses[i].p.v[1], poses[i].p.v[2]);
				
		Feature query(poses[i].p);
		//nnSearch->knnSearch(/*(const Real*)&*/poses[i].p/*.x*//*.v*/, neighbours, indices, distances);
		nnSearch->knnSearch(query, desc.neighbours, indices, distances);
		
		// if true, only a subset of neighbours are used to eval the collision
		//if (desc.points < desc.neighbours)
		//	std::random_shuffle(indices.begin(), indices.end(), rand);

		for (size_t j = 0; j < size; ++j) {
//			const Real depth = Math::abs(bounds.getDepth(points[indices[j]]/*.getPoint()*/));
			const Feature f = desc.points < desc.neighbours ? points[indices[size_t(rand.next()) % indices.size()]] : points[indices[j]];
			//if (j % 10 == 0) manipulator.getContext().write("point=%d <%f, %f, %f> index=%d distance=%f depth=%f collision=%s\n", j, f.getPoint().x, f.getPoint().y, f.getPoint().z,
			//	indices[j], distances[j], bounds.getDepth(points[j]), bounds.getDepth(f) > REAL_ZERO ? "YES" : "NO");
			if (bounds.getDepth(f) > REAL_ZERO) {
#ifdef _COLLISION_PERFMON
				SecTmReal t_end = t.elapsed();
				tperfCheckNN += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
				if (debug /*&& eval < REAL_ONE*/)
					manipulator.getContext().debug(
					"Collision::check(kd-tree): time_elapsed = %f [sec], collision=yes, neighbours=%u, points=%u\n",
					t_end, desc.neighbours, desc.points
					);
#endif
				return true;
			}
		}
		//break;
	}

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfCheckNN += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().debug("Collision::evaluate(): neighbours=%u, , points=%u, collision=no\n", desc.neighbours, desc.points);
#endif

	return false;
}

//------------------------------------------------------------------------------

golem::Real Collision::estimate(const FlannDesc& desc, const grasp::Manipulator::Pose& rbpose, golem::Real maxDist, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEstimate;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < desc.points ? points.size() : desc.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	// iterates only for finger tips
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints(); ++i) {
		Bounds bounds = this->bounds[i];
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

golem::Real Collision::evaluate(const Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));
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

golem::Real Collision::evaluate(const Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, const FTGuard::Seq &triggeredGuards, bool debug) const {
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;
//	manipulator.getContext().write("Collision::evaluate():\n");
	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
	//for (auto guard = triggeredGuards.begin(); guard < triggeredGuards.end(); ++guard) {
		//const U32 i = 0 - guard->jointIdx;
		Bounds bounds = this->bounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));
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
		const Real val = bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, (Bounds::RealEval)waypoint.depthStdDev, triggered, i < manipulator.getJoints() ? poses[i] : pose, force, collisions);
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

golem::Real Collision::evaluate(const FlannDesc& desc, const grasp::Manipulator::Pose& rbpose, FTGuard::Seq &triggeredGuards, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	golem::Real eval = REAL_ZERO, c = REAL_ZERO;
	const Real norm = golem::numeric_const<Real>::ONE / ((desc.depthStdDev * 100) * Math::sqrt(2 * golem::numeric_const<Real>::PI));
	size_t collisions = 0, checks = 0;

	const U32 jointPerFinger = 4;
	const U32 fingers = 5;
	// Utilities
	std::vector<bool> triggeredFingers; triggeredFingers.assign(fingers, false);
	typedef std::map<U32, FTGuard> GuardMap; GuardMap guardMap;
	// retrive trigguered guards
	for (FTGuard::Seq::iterator guard = triggeredGuards.begin(); guard != triggeredGuards.end(); ++guard) {
		triggeredFingers[guard->getHandChain() - 1] = true;
		const U32 k = U32(0 - guard->jointIdx);
		manipulator.getContext().write("guard map key %d\n", -k);
		guardMap.insert(guardMap.begin(), GuardMap::value_type(-k, *guard));
	}
	manipulator.getContext().write("triggered finger [%s,%s,%s,%s,%s], guard %d\n", triggeredFingers[0] ? "T" : "F", triggeredFingers[1] ? "T" : "F", triggeredFingers[2] ? "T" : "F", triggeredFingers[3] ? "T" : "F", triggeredFingers[4] ? "T" : "F", guardMap.size());

	// loops over fingers: thumb = 0,..., pinky = 4
	for (U32 finger = 0; finger < fingers; ++finger) {
		// thumb: begin=7, end=10
		const U32 begin = manipulator.getArmJoints() + (finger*jointPerFinger), end = begin + jointPerFinger;
		// abductor has no bounds. Retrive contact there at the beginning
		bool triggeredAbductor = false;
		GuardMap::const_iterator abductor = guardMap.find(begin);
		if (abductor != guardMap.end())
			triggeredAbductor = true;
		//manipulator.getContext().write("abductor %d\n", triggeredAbductor);

		for (U32 i = begin; i < end; ++i) {
			Bounds bounds = this->bounds[i];
			if (bounds.empty())
				continue;

			// set current pose to the bounds
			bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));

			// extract the closest point to the joint's bounds as feature
			Feature query(poses[i].p);
			nnSearch->knnSearch(query, desc.neighbours, indices, distances);

			Feature::Seq seq;
			seq.reserve(indices.size());
			for (size_t j = 0; j < indices.size(); ++j)
				seq.push_back(points[indices[j]]);
			
			Real maxDepth = -golem::numeric_const<Real>::MAX;
			U32 boundCollisions = golem::numeric_const<U32>::ZERO;
			Real force = REAL_ZERO;
			GuardMap::const_iterator jointGuard = guardMap.find(i);
			if (jointGuard != guardMap.end()) force = jointGuard->second.force;
			//manipulator.getContext().write("joint %d, triggered %s, torque %d\n", i, jointGuard != guardMap.end() ? "T" : "F", force);

			golem::Mat34 inverse; inverse.setInverse(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose)); // compute the inverse of the joint frame

			for (size_t j = 0; j < seq.size(); ++j) {
				const Feature f = seq[j];
				const Real depth = bounds.getDepth(f, true);
				if (depth > REAL_ZERO) ++boundCollisions; // number of collisions with this bound
				if (depth > maxDepth) {
					// computes the direction of contact
					golem::Vec3 v; inverse.multiply(v, f.getPoint()); //v.normalise(); // compute the point in the joint's reference frame
					const bool adbDirection = triggeredAbductor ? !((v.y > 0 && abductor->second.force >= 0) || (v.y < 0 && abductor->second.force <= 0)) : true;
					const bool flexDirection = !((v.z > 0 && force >= 0) || (v.z < 0 && force <= 0));
					//if (j % 100 == 0) manipulator.getContext().write("adbDirection %s, flexDirection %s\n", adbDirection ? "T" : "F", flexDirection ? "T" : "F");
					if (adbDirection && flexDirection)
						maxDepth = depth; // record max depth
				}
			}
			if (maxDepth > -golem::numeric_const<Real>::MAX) manipulator.getContext().write("max depth %f\n", maxDepth);

			if (!triggeredFingers[finger] && maxDepth > REAL_ZERO) // the hypothesis intersects a finger that has no contact retrieved
				return REAL_EPS;
			// if no contact is retrieve and there is no intersection, don't change eval

			// if contact is retrieved
			if (triggeredFingers[finger]) {
				Real pointEval = REAL_ZERO;
				// there is penetration
				if (maxDepth > REAL_ZERO)
					pointEval = Math::exp(-Real(desc.depthStdDev * 100)*(maxDepth / boundCollisions));
				else // maxDepth here has to be <= 0. no penetration.
					pointEval = norm*golem::Math::exp(-.5*Math::sqr(Real(maxDepth) / Real(desc.depthStdDev * 100))); // gaussian 
				golem::kahanSum(eval, c, pointEval);
				collisions += boundCollisions;
			}
		}
	}

	const Real likelihood = desc.likelihood*eval / collisions/*((eval + (checks - collisions)) / checks - REAL_ONE)*/;

#ifdef _COLLISION_PERFMON
	SecTmReal t_end = t.elapsed();
	tperfEvalPoints += t_end /*tperfEvalPoints < t_end ? t_end : tperfEvalPoints*/;
	//if (debug /*&& eval < REAL_ONE*/)
		manipulator.getContext().write("Collision::evaluate(): points=%u, checks=%u, collisions=%u, likelihhod=%f, eval=%f\n", indices.size(), checks, collisions, likelihood, eval / collisions);
#endif

	return likelihood;
}

golem::Real Collision::evaluate(const FlannDesc& desc, const grasp::Manipulator::Pose& rbpose, bool debug) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		// todo: check this is a valid value to return in case of faliure
		return REAL_ZERO;
	}
#ifdef _COLLISION_PERFMON
	PerfTimer t;
	++perfEvalPoints;
#endif
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));

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

void Collision::draw(const Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, golem::DebugRenderer& renderer) const {
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	renderer.setColour(RGBA::BLACK);

	for (U32 i = manipulator.getArmJoints() + 3; i < manipulator.getJoints() + 1; ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty())
			continue;

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
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

void Collision::draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, const Collision::FlannDesc& desc) const {
	if (!this->desc.kdtree) {
		manipulator.getContext().info("Collision::Check(): No KD-Tree\n");
		return;
	}
	PerfTimer t;

	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	renderer.setColour(RGBA::BLACK);
	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;

	for (U32 i = manipulator.getArmJoints() + 3; i < manipulator.getJoints() + 1; ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty())
			continue;

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
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

void Collision::draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Pose& rbpose, const Collision::FlannDesc& desc) const {
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < desc.points ? points.size() : desc.points;

	renderer.setColour(RGBA::BLACK);

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	// iterates only for finger tips
	for (U32 i = manipulator.getArmJoints() + 3; i < manipulator.getJoints(); ++i) {
		Bounds bounds = this->bounds[i];
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

void Collision::draw(golem::DebugRenderer& renderer, const grasp::Manipulator::Pose& rbpose, std::vector<golem::Configspace::Index> &joints, grasp::RealSeq &forces, const Collision::FlannDesc& desc) const {
	joints.clear();
	forces.clear();

	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	grasp::NNSearch::IndexSeq indices;
	grasp::NNSearch::DistanceF64Seq distances;

	const size_t size = desc.points < desc.neighbours ? desc.points : desc.neighbours;
	renderer.setColour(RGBA::BLACK);

	for (U32 i = manipulator.getArmJoints() + 3; i < manipulator.getJoints(); ++i) {
		Bounds bounds = this->bounds[i];
		if (bounds.empty()) {
			//			manipulator.getContext().write("Collision::checkNN(): No bounds.\n");
			continue;
		}

		const Mat34 boundsPose = i < manipulator.getJoints() ? poses[i] : pose;
		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
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

void Collision::draw(golem::DebugRenderer& renderer, const Waypoint& waypoint, const grasp::Manipulator::Pose& rbpose, const FTGuard::Seq &triggeredGuards, bool debug) const {
	const golem::Mat34 pose(rbpose.toMat34());
	golem::Mat34 poses[grasp::Manipulator::JOINTS];
	manipulator.getPoses(rbpose, pose, poses);

	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;

	golem::Real eval = REAL_ZERO;
	size_t collisions = 0, checks = 0;

	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints(); ++i) {
		//for (auto guard = triggeredGuards.begin(); guard < triggeredGuards.end(); ++guard) {
		//const U32 i = 0 - guard->jointIdx;
		Bounds bounds = this->bounds[i];
		if (bounds.empty())
			continue;

		bounds.setPose(Bounds::Mat34(i < manipulator.getJoints() ? poses[i] : pose));
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
		const Real val = bounds.evaluate(points.data(), points.data() + size, (Bounds::RealEval)waypoint.depthStdDev, (Bounds::RealEval)waypoint.depthStdDev, triggered, i < manipulator.getJoints() ? poses[i] : pose, force, collisions);
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
	catch (const golem::MsgXMLParser&) {  }
}

//------------------------------------------------------------------------------

void Hypothesis::BoundsAppearance::draw(const golem::Bounds::Seq& bounds, golem::DebugRenderer& renderer) const {
	if (showSolid) {
		renderer.setColour(solidColour);
		renderer.addSolid(bounds.begin(), bounds.end());
	}
	if (showWire) {
		renderer.setColour(wireColour);
		renderer.setLineWidth(wireWidth);
		renderer.addWire(bounds.begin(), bounds.end());
	}
}

Bounds::Seq Hypothesis::bounds() {
	Bounds::Seq bounds;
	bounds.push_back(boundsDesc.create());
	return bounds;
}

//------------------------------------------------------------------------------

Hypothesis::Hypothesis(const grasp::Manipulator& manipulator, const Desc& desc) : manipulator(manipulator), desc(desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "Hypothesis(): invalid description");
	collisionPtr = desc.collisionDescPtr->create(manipulator);
}

//------------------------------------------------------------------------------

void Hypothesis::create(const golem::U32 idx, const golem::Mat34& trn, const grasp::RBPose::Sample& s, golem::Rand& rand, const grasp::Cloud::PointSeq& points) {
	collisionPtr->create(rand, points);
	index = idx;
	modelFrame = trn;
	sample = s;
	Real x = REAL_ZERO, y = REAL_ZERO, z = REAL_ZERO;
	Vec3 sampleFrame = toRBPoseSampleGF().p;
	for (grasp::Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i) {
		this->points.push_back(*i);
		// calculate max x, y, z values for bounding box
		if (x < Math::abs(sampleFrame.x - (float)i->x))
			x = Math::abs(sampleFrame.x - (float)i->x);
		if (y < Math::abs(sampleFrame.y - (float)i->y))
			y = Math::abs(sampleFrame.y - (float)i->y);
		if (z < Math::abs(sampleFrame.z - (float)i->z))
			z = Math::abs(sampleFrame.z - (float)i->z);
	}
//	printf("Bounding box dim x=%f, y=%f, z=%f\n", x, y, z);
	// set dimension of the bounding box
	boundsDesc.dimensions = Vec3(x, y, z);
	boundsDesc.pose.p = toRBPoseSampleGF().p;
//	printf("Bounding box dim x=%f, y=%f, z=%f, pose=<%f, %f, %f>\n", boundsDesc.dimensions.x, boundsDesc.dimensions.y, boundsDesc.dimensions.z, boundsDesc.pose.p.x, boundsDesc.pose.p.y, boundsDesc.pose.p.z);
}

void Hypothesis::draw(DebugRenderer &renderer) const {
	printf("Belief::Hypothesis::draw(showFrame=%s, showPoints=%s)\n", appearance.showFrames ? "ON" : "OFF", appearance.showPoints ? "ON" : "OFF");
	if (appearance.showFrames || true)
		renderer.addAxes(sample.toMat34() * modelFrame, appearance.frameSize);

	size_t t = 0;
	if (appearance.showPoints || true) {
		for (grasp::Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i) {
			grasp::Cloud::Point point = *i;
			if (++t < 10) printf("Belief::Hypothesis point %d <%.4f %.4f %.4f>\n", t, point.x, point.y, point.z);
			grasp::Cloud::setColour(/*appearance.colour*/RGBA::RED, point);
			renderer.addPoint(grasp::Cloud::getPoint<Real>(point));
		}
	}
}

//------------------------------------------------------------------------------

std::string Hypothesis::str() const {
	std::stringstream ss; 
	const grasp::RBPose::Sample pose = toRBPoseSampleGF();
	ss << pose.p.x << "\t" << pose.p.y << "\t" << pose.p.z << "\t" << pose.q.w << "\t" << pose.q.x << "\t" << pose.q.y << "\t" << pose.q.z;
	return ss.str();
}

//------------------------------------------------------------------------------

void Hypothesis::draw(const Collision::Waypoint &waypoint, const grasp::Manipulator::Pose& rbpose, golem::DebugRenderer& renderer) const {
	collisionPtr->draw(waypoint, rbpose, renderer);
}

void Hypothesis::draw(golem::DebugRenderer& renderer, const golem::Rand& rand, const grasp::Manipulator::Pose& rbpose) const {
	collisionPtr->draw(renderer, rand, rbpose, desc.collisionDescPtr->flannDesc);
}

//------------------------------------------------------------------------------

void spam::XMLData(spam::Hypothesis::Desc& val, golem::XMLContext* xmlcontext, bool create) {
	spam::XMLData(*val.collisionDescPtr, xmlcontext->getContextFirst("collision"), create);
}

//------------------------------------------------------------------------------