/** @file Data.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Spam/Spam/Data.h>
#include <Golem/Plan/Data.h>

#ifdef WIN32
	#pragma warning (push)
	#pragma warning (disable : 4244)
	#pragma warning (disable : 4996)
#endif
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#ifdef WIN32
	#pragma warning (pop)
#endif

#include <boost/lexical_cast.hpp>

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;
//
////------------------------------------------------------------------------------
//
//void spam::XMLData(FTDrivenHeuristic::FTModelDesc& val, golem::XMLContext* context, bool create) {
//	golem::XMLData("dist_max", val.distMax, context, create);
//	golem::XMLData("cone_theta_orizontal_axis", val.coneTheta1, context, create);
//	golem::XMLData("cone_theta_vertical_axis", val.coneTheta2, context, create);
//	golem::XMLData("num_nearest_points", val.k, context, create);
//	golem::XMLData("num_points", val.points, context, create);
//	golem::XMLData("mahalanobis_fac", val.mahalanobisDistFac, context, create);
//	//golem::XMLData("enable_arm_chain", val.enabledArmChain, context, create);
//	//golem::XMLData("enable_hand_joints", val.enabledHandJoints, context, create);
////	golem::XMLData("enabled_steps", val.enabledSteps, context, create);
//	golem::XMLData("enabled_likelihood", val.enabledLikelihood, context, create);
//	golem::XMLData("intrinsic_exp_parameter", val.lambda, context, create);
//	golem::XMLData(val.jointFac, context->getContextFirst("joint_fac"), create);
//}
//
//void spam::XMLData(FTDrivenHeuristic::Desc& val, golem::XMLContext* context, bool create) {
////	golem::XMLData((golem::Heuristic::Desc)val, context, create);
////	spam::XMLData(val.beliefDesc, context->getContextFirst("belief_space"), create);
//	golem::XMLData("contact_fac", val.contactFac, context, create);
//	golem::XMLData("non_contact_fac", val.nonContactFac, context, create);
//	golem::XMLData("max_surface_points_inkd", val.maxSurfacePoints, context, create);
//	spam::XMLData(val.ftModelDesc, context->getContextFirst("ftmodel"), create);
//	golem::XMLData(&val.covariance[0], &val.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);
//}

//------------------------------------------------------------------------------

//void spam::XMLData(Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create) {
////	grasp::XMLData((grasp::Robot::Desc&)val, context, xmlcontext, create);
//	RagGraphPlanner::Desc* pRagGraphPlanner(new RagGraphPlanner::Desc);
//	(golem::Planner::Desc&)*pRagGraphPlanner = *val.physPlannerDesc->pPlannerDesc;
//	val.physPlannerDesc->pPlannerDesc.reset(pRagGraphPlanner);
//	val.physPlannerDesc->pPlannerDesc = RagGraphPlanner::Desc::load(context, xmlcontext->getContextFirst("planner"));
//	
//	FTDrivenHeuristic::Desc* pFTDrivenHeuristic(new FTDrivenHeuristic::Desc);
//	(golem::Heuristic::Desc&)*pFTDrivenHeuristic = *val.physPlannerDesc->pPlannerDesc->pHeuristicDesc;
//	val.physPlannerDesc->pPlannerDesc->pHeuristicDesc.reset(pFTDrivenHeuristic);
//	spam::XMLData((FTDrivenHeuristic::Desc&)*val.physPlannerDesc->pPlannerDesc->pHeuristicDesc, xmlcontext->getContextFirst("rag_planner heuristic"), create);
//}

//------------------------------------------------------------------------------

//void spam::XMLData(RagGraphPlanner::Desc &val, golem::XMLContext* context, bool create) {
//	golem::XMLData((golem::GraphPlanner::Desc&)val, context, create);
//}

//------------------------------------------------------------------------------

const char spam::TrialData::headerName [] = "spam::TrialData";
const golem::U32 spam::TrialData::headerVersion = 1;

template <> void golem::Stream::read(spam::TrialData& trialData) const {
	char name[sizeof(trialData.headerName)];
	read(name, sizeof(trialData.headerName));
	name[sizeof(trialData.headerName) - 1] = '\0';
	if (strncmp(trialData.headerName, name, sizeof(trialData.headerName)) != 0)
		throw Message(Message::LEVEL_CRIT, "Stream::read(spam::TrialData&): Unknown file name: %s", name);

	golem::U32 version;
	*this >> version;
	if (version != trialData.headerVersion)
		throw Message(Message::LEVEL_CRIT, "Stream::read(spam::TrialData&): Unknown file version: %d", version);

	//trialData.approachAction.clear();
	//read(trialData.approachAction, trialData.approachAction.begin(), grasp::RobotState(trialData.controller));
	//trialData.manipAction.clear();
	//read(trialData.manipAction, trialData.manipAction.begin(), grasp::RobotState(trialData.controller));

	//trialData.approachWithdraw.clear();
	//read(trialData.approachWithdraw, trialData.approachWithdraw.begin(), grasp::RobotState(trialData.controller));
	////trialData.action.clear();
	////read(trialData.action, trialData.action.begin(), trialData.controller.createState());

	//trialData.density.clear();
	//read(trialData.density, trialData.density.begin());

	//trialData.hypotheses.clear();
	//read(trialData.hypotheses, trialData.hypotheses.begin());
}

template <> void golem::Stream::write(const spam::TrialData& trialData) {
	*this << trialData.headerName << trialData.headerVersion;

	//write(trialData.approachAction.begin(), trialData.approachAction.end());
	//write(trialData.manipAction.begin(), trialData.manipAction.end());

	//write(trialData.approachWithdraw.begin(), trialData.approachWithdraw.end());
	////write(trialData.action.begin(), trialData.action.end());
	//write(trialData.density.begin(), trialData.density.end());
	//write(trialData.hypotheses.begin(), trialData.hypotheses.end());
}

////------------------------------------------------------------------------------
//
//void Collision::Bounds::create(const golem::Bounds::Seq& bounds) {
//	for (size_t i = 0; i < bounds.size(); ++i) {
//		const golem::BoundingConvexMesh* mesh = dynamic_cast<const golem::BoundingConvexMesh*>(bounds[i].get());
//		if (mesh != nullptr) {
//			surfaces.resize(surfaces.size() + 1);
//			triangles.resize(triangles.size() + 1);
//			surfaces.back().resize(mesh->getTriangles().size());
//			triangles.back().resize(mesh->getTriangles().size());
//			for (size_t j = 0; j < mesh->getTriangles().size(); ++j) {
//				triangles.back()[j].normal = surfaces.back()[j].normal = mesh->getNormals()[j];
//				surfaces.back()[j].point = mesh->getVertices()[mesh->getTriangles()[j].t1]; // e.g. first triangle
//				//surfaces.back()[j].point = (mesh->getVertices()[mesh->getTriangles()[j].t1] + mesh->getVertices()[mesh->getTriangles()[j].t2] + mesh->getVertices()[mesh->getTriangles()[j].t3])/Real(3.0); // centroid
//				triangles.back()[j].distance = mesh->getDistances()[j];
//			}
//		}
//	}
//}
//
//Collision::Collision(const grasp::Manipulator& manipulator) : manipulator(manipulator) {
//	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i)
//		bounds[i].create(i < manipulator.getJoints() ? manipulator.getJointBounds(i) : manipulator.getBaseBounds());
//}
//
//golem::Real Collision::evaluate(const Waypoint& waypoint, const grasp::Cloud::PointSeq& points, golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, bool debug) {
//	const Mat34 pose(rbpose.toMat34());
//	Mat34 poses[grasp::Manipulator::JOINTS];
//	manipulator.getPoses(rbpose, pose, poses);
//
//	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;
//	Real norm = REAL_ZERO;
//	Real eval = REAL_ZERO;
//	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
//		Bounds& bounds = this->bounds[i];
//		if (bounds.empty())
//			continue;
//
//		bounds.setPose(i < manipulator.getJoints() ? poses[i] : pose);
//		for (size_t j = 0; j < size; ++j) {
//			const grasp::Cloud::Point& point = size < points.size() ? points[size_t(rand.next()) % points.size()] : points[j];
//			const Vec3 p = grasp::Cloud::getPoint(point);
//			const Real depth = Math::abs(bounds.getDepth(p));
//			//if (depth > REAL_EPS)
//			//	printf("depth");
//			eval += Math::exp(-waypoint.depthStdDev*depth);
//		}
//		norm += REAL_ONE;
//	}
//	norm = REAL_ONE / (size*norm);
//	eval *= norm;
//
//	if (debug)
//		manipulator.getContext().debug("Collision::evaluate(): points=%d, eval=%f, likelihhod=%f\n", size, eval, waypoint.likelihood*(eval - REAL_ONE));
//
//	return waypoint.likelihood*(eval - REAL_ONE);
//}
//
//
//golem::Real Collision::evaluate(const Robot *robot, const Waypoint& waypoint, const grasp::Cloud::PointSeq& points, golem::Rand& rand, const grasp::Manipulator::Pose& rbpose, std::vector<golem::Configspace::Index> &joints, bool debug) {
//	if (points.empty())
//		return REAL_ZERO;
//
//	joints.clear();
//	const Mat34 pose(rbpose.toMat34());
//	Mat34 poses[grasp::Manipulator::JOINTS];
//	manipulator.getPoses(rbpose, pose, poses);
//	
//	const size_t size = points.size() < waypoint.points ? points.size() : waypoint.points;
//	Real norm = REAL_ZERO;
//	Real eval = REAL_ZERO;
////	for (U32 i = manipulator.getArmJoints(); i < manipulator.getJoints() + 1; ++i) {
//	for (Configspace::Index i = robot->getStateHandInfo().getJoints().begin(); i != robot->getStateHandInfo().getJoints().end(); ++i) {
//		size_t k = i - robot->getStateArmInfo().getJoints().begin();
//		Bounds& bounds = this->bounds[k];
//		if (bounds.empty())
//			continue;
//
//		bounds.setPose(i < manipulator.getJoints() ? poses[k] : pose);
//		for (size_t j = 0; j < size; ++j) {
//			const grasp::Cloud::Point& point = size < points.size() ? points[size_t(rand.next()) % points.size()] : points[j];
//			const Vec3 p = grasp::Cloud::getPoint(point);
//			const Real depth = Math::abs(bounds.getDepth(p));
//			eval += Math::exp(-waypoint.depthStdDev*depth);
//			if (depth > REAL_ZERO) {
//				//				printf("collision\n");
//				joints.push_back(i);
//				Vec3 v;
//				Mat34 jointFrameInv;
//				jointFrameInv.setInverse(pose);
//				jointFrameInv.multiply(v, Vec3(point.x, point.y, point.z));
//				v.normalise();
//				return v.z > REAL_ZERO ? -REAL_ONE : REAL_ONE;
//			}
//		}
//		norm += REAL_ONE;
//	}
//	norm = REAL_ONE / (size*norm);
//	eval *= norm;
//
//	if (debug)
//		manipulator.getContext().debug("Collision::evaluate2(): points=%d, eval=%f, likelihhod=%f joints=%d\n", size, eval, waypoint.likelihood*(eval - REAL_ONE), joints.size());
//
//	return waypoint.likelihood*(eval - REAL_ONE);
//}
//
////------------------------------------------------------------------------------
