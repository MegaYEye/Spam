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

using namespace spam;

//------------------------------------------------------------------------------

void spam::XMLData(FTDrivenHeuristic::FTModelDesc& val, golem::XMLContext* context, bool create) {
	golem::XMLData("dist_max", val.distMax, context, create);
	golem::XMLData("cone_theta_orizontal_axis", val.coneTheta1, context, create);
	golem::XMLData("cone_theta_vertical_axis", val.coneTheta2, context, create);
	golem::XMLData("num_nearest_points", val.k, context, create);
	golem::XMLData("num_points", val.points, context, create);
	golem::XMLData("mahalanobis_fac", val.mahalanobisDistFac, context, create);
	//golem::XMLData("enable_arm_chain", val.enabledArmChain, context, create);
	//golem::XMLData("enable_hand_joints", val.enabledHandJoints, context, create);
//	golem::XMLData("enabled_steps", val.enabledSteps, context, create);
	golem::XMLData("enabled_likelihood", val.enabledLikelihood, context, create);
	golem::XMLData("intrinsic_exp_parameter", val.lambda, context, create);
	golem::XMLData(val.jointFac, context->getContextFirst("joint_fac"), create);
}

void spam::XMLData(FTDrivenHeuristic::Desc& val, golem::XMLContext* context, bool create) {
//	golem::XMLData((golem::Heuristic::Desc)val, context, create);
//	spam::XMLData(val.beliefDesc, context->getContextFirst("belief_space"), create);
	golem::XMLData("contact_fac", val.contactFac, context, create);
	golem::XMLData("non_contact_fac", val.nonContactFac, context, create);
	golem::XMLData("max_surface_points_inkd", val.maxSurfacePoints, context, create);
	spam::XMLData(val.ftModelDesc, context->getContextFirst("ftmodel"), create);
	golem::XMLData(&val.covariance[0], &val.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);
}

//------------------------------------------------------------------------------

void spam::XMLData(Robot::Desc &val, golem::Context* context, golem::XMLContext* xmlcontext, bool create) {
//	grasp::XMLData((grasp::Robot::Desc&)val, context, xmlcontext, create);
	RagGraphPlanner::Desc* pRagGraphPlanner(new RagGraphPlanner::Desc);
	(golem::Planner::Desc&)*pRagGraphPlanner = *val.physPlannerDesc->pPlannerDesc;
	val.physPlannerDesc->pPlannerDesc.reset(pRagGraphPlanner);
	val.physPlannerDesc->pPlannerDesc = RagGraphPlanner::Desc::load(context, xmlcontext->getContextFirst("planner"));
	
	FTDrivenHeuristic::Desc* pFTDrivenHeuristic(new FTDrivenHeuristic::Desc);
	(golem::Heuristic::Desc&)*pFTDrivenHeuristic = *val.physPlannerDesc->pPlannerDesc->pHeuristicDesc;
	val.physPlannerDesc->pPlannerDesc->pHeuristicDesc.reset(pFTDrivenHeuristic);
	spam::XMLData((FTDrivenHeuristic::Desc&)*val.physPlannerDesc->pPlannerDesc->pHeuristicDesc, xmlcontext->getContextFirst("rag_planner heuristic"), create);
}

//------------------------------------------------------------------------------

void spam::XMLData(RagGraphPlanner::Desc &val, golem::XMLContext* context, bool create) {
	golem::XMLData((golem::GraphPlanner::Desc&)val, context, create);
}

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

	trialData.approachAction.clear();
	read(trialData.approachAction, trialData.approachAction.begin(), grasp::RobotState(trialData.controller));
	trialData.manipAction.clear();
	read(trialData.manipAction, trialData.manipAction.begin(), grasp::RobotState(trialData.controller));

	trialData.approachWithdraw.clear();
	read(trialData.approachWithdraw, trialData.approachWithdraw.begin(), grasp::RobotState(trialData.controller));
	//trialData.action.clear();
	//read(trialData.action, trialData.action.begin(), trialData.controller.createState());

	trialData.density.clear();
	read(trialData.density, trialData.density.begin());

	trialData.hypotheses.clear();
	read(trialData.hypotheses, trialData.hypotheses.begin());
}

template <> void golem::Stream::write(const spam::TrialData& trialData) {
	*this << trialData.headerName << trialData.headerVersion;

	write(trialData.approachAction.begin(), trialData.approachAction.end());
	write(trialData.manipAction.begin(), trialData.manipAction.end());

	write(trialData.approachWithdraw.begin(), trialData.approachWithdraw.end());
	//write(trialData.action.begin(), trialData.action.end());
	write(trialData.density.begin(), trialData.density.end());
	write(trialData.hypotheses.begin(), trialData.hypotheses.end());
}

//------------------------------------------------------------------------------
