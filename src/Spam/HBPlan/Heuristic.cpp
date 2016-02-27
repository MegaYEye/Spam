/** @file Heuristic.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Spam/HBPlan/Heuristic.h>
#include <Golem/Device/MultiCtrl/MultiCtrl.h>
#include <Golem/Plan/Data.h>

//------------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;

//------------------------------------------------------------------------------

#ifdef _HEURISTIC_PERFMON
U32 FTDrivenHeuristic::perfCollisionWaypoint;
U32 FTDrivenHeuristic::perfCollisionPath;
U32 FTDrivenHeuristic::perfCollisionGroup;
U32 FTDrivenHeuristic::perfCollisionBounds;
U32 FTDrivenHeuristic::perfCollisionSegs;
U32 FTDrivenHeuristic::perfCollisionPointCloud;
U32 FTDrivenHeuristic::perfBoundDist;
U32 FTDrivenHeuristic::perfH;

void FTDrivenHeuristic::resetLog() {
	perfCollisionWaypoint = 0;
	perfCollisionPath = 0;
	perfCollisionGroup = 0;
	perfCollisionBounds = 0;
	perfCollisionSegs = 0;
	perfCollisionPointCloud = 0;
	perfBoundDist = 0;
	perfH = 0;
}

void FTDrivenHeuristic::writeLog(Context &context, const char *str) {
	context.debug(
		"%s: collision_{waypoint, path, group, bounds, point cloud, <segments>} = {%u, %u, %u, %u, %u, %f}, calls_{getBoundedDist(), h()} = {%u, %u}\n", str, perfCollisionWaypoint, perfCollisionPath, perfCollisionGroup, perfCollisionBounds, perfCollisionPointCloud, perfCollisionPath > 0 ? Real(perfCollisionSegs) / perfCollisionPath : REAL_ZERO, perfBoundDist, perfH
		);
}
#endif

//------------------------------------------------------------------------------

inline golem::Vec3 dot(const golem::Vec3 &v, const golem::Vec3 &diagonal) {
	golem::Vec3 result;
	for (size_t i = 0; i < 3; ++i)
		result[i] = v[i] * diagonal[i];
	return result;
}

void dotInverse(const std::vector<Real>& v0, const std::vector<Real>& v1, std::vector<Real> &result) {
	if (v0.size() != v1.size())
		throw MsgIncompatibleVectors(Message::LEVEL_CRIT, "spam::dot(std::vector<Real>, std::vector<Real>): Invalid vectors size.");
//	printf("spam::dotInverse()\n");
	result.reserve(v0.size());
	for (size_t i = 0; i < v0.size(); ++i) {
		const Real r = v0[i] * (REAL_ONE/v1[i]);
		result.push_back(r);
//		printf("result[%d] = %.5f * %.5f = %.5f\n", i, v0[i], (REAL_ONE/v1[i]), r);
	}
}

Real dot(const std::vector<Real>& v0, const std::vector<Real>& v1) {
	if (v0.size() != v1.size())
		throw MsgIncompatibleVectors(Message::LEVEL_CRIT, "spam::dot(std::vector<Real>, std::vector<Real>): Invalid vectors size.");
	
	Real result;
	result = REAL_ZERO;
	for (size_t i = 0; i < v0.size(); ++i)
		result += v0[i] * v1[i];
//	printf("dot(v0, v1) %.5f\n", result);
	return result;
}

//------------------------------------------------------------------------------

FTDrivenHeuristic::FTDrivenHeuristic(golem::Controller &controller) : Heuristic(controller), rand(context.getRandSeed()) {}

bool FTDrivenHeuristic::create(const Desc &desc) {
//	context.debug("FTDrivenHeuristic::create()...\n");
	Heuristic::create((Heuristic::Desc&)desc);

	// printing of debug
//	context.write("contact_fac=%f, max_points=%d, covariance=%f\n", desc.contactFac, desc.maxSurfacePoints, desc.covariance.p[0]);

	ftDrivenDesc = desc;
//	context.write("Heuristic params: dflt=%f limits=%f root=%f\n", desc.costDesc.distDfltFac, desc.costDesc.distLimitsFac, desc.costDesc.distRootFac);
	//ftDrivenDesc.ftModelDesc.dim = 0;
	//for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
	//	const ChainDesc* cdesc = getChainDesc()[i];
	//	if (cdesc->enabledObs)
	//		++ftDrivenDesc.ftModelDesc.dim;
	//}

	// controllers
	MultiCtrl *multiCtrl = dynamic_cast<MultiCtrl*>(&controller);
	if (multiCtrl == NULL)
		throw Message(Message::LEVEL_CRIT, "Unknown controller");
	// which has two other controllers: arm + hand
	if (multiCtrl->getControllers().size() < 2)
		throw Message(Message::LEVEL_CRIT, "Arm and hand controllers expected");
	arm = dynamic_cast<SingleCtrl*>(multiCtrl->getControllers()[0]);
	hand = dynamic_cast<SingleCtrl*>(multiCtrl->getControllers()[1]);
	if (arm == NULL || hand == NULL)
		throw Message(Message::LEVEL_CRIT, "Robot::create(): Robot requires SingleCtrl");	

	armInfo = arm->getStateInfo();
	handInfo = hand->getStateInfo();

	jointFac = desc.ftModelDesc.jointFac;

	manipulator.reset();

	enableUnc = false;

	pointCloudCollision = false;

	collision.reset();
	waypoint.setToDefault();
	waypoint.points = 100000;//ftDrivenDesc.ftModelDesc.points;

	hypothesisBoundsSeq.clear();

	testCollision = false;

	return true;
}

void FTDrivenHeuristic::setDesc(const Desc &desc) {
//	Heuristic::setDesc((Heuristic::Desc&)desc);
//	(Heuristic::Desc&)this->desc = this->Heuristic::desc;
//	this->bsDesc.beliefDesc = desc.beliefDesc;
	this->ftDrivenDesc = desc;
	this->ftDrivenDesc.ftModelDesc = desc.ftModelDesc;
}

void FTDrivenHeuristic::setDesc(const Heuristic::Desc &desc) {
	Heuristic::setDesc(desc);
}

//------------------------------------------------------------------------------

golem::Real FTDrivenHeuristic::cost(const golem::Waypoint &w, const golem::Waypoint &root, const golem::Waypoint &goal) const {
	golem::Real c = golem::REAL_ZERO;
	const bool enable = enableUnc && pBelief->getHypotheses().size() > 0;

	if (desc.costDesc.distRootFac > golem::REAL_ZERO)
		c += desc.costDesc.distRootFac*getDist(w, root);
	
//	if (desc.costDesc.distGoalFac > golem::REAL_ZERO) {
		const Real dist = (enable)?getMahalanobisDist(w, goal):getWorkspaceDist(w, goal);
		c += /*desc.costDesc.distGoalFac**/dist;
//		c += desc.costDesc.distGoalFac*getMahalanobisDist(w, goal); getWorkspaceDist
//		context.getMessageStream()->write(Message::LEVEL_DEBUG, "MyHeuristic::cost(w, root, goal) goal <%.2f, %.2f, %.2f>\n", goal.wpos[chainArm].p.x, goal.wpos[chainArm].p.y, goal.wpos[chainArm].p.z);
//	}

	if (desc.costDesc.distLimitsFac > golem::REAL_ZERO)
		c += desc.costDesc.distLimitsFac*getConfigspaceLimitsDist(w.cpos);
	if (desc.costDesc.distDfltFac > golem::REAL_ZERO)
		c += desc.costDesc.distDfltFac*getConfigspaceDist(w.cpos, dfltPos);

	return c;
}

Real FTDrivenHeuristic::cost(const golem::Waypoint &w0, const golem::Waypoint &w1) const {
//	SecTmReal init = context.getTimer().elapsed();
	//if (collides(w0, w1))
	//	return Node::COST_INF;
	Real c = REAL_ZERO, d;
	const bool enable = enableUnc && (pBelief.get() && pBelief->getHypotheses().size() > 0);

	const Chainspace::Index chainArm = stateInfo.getChains().begin();
//	if (desc.costDesc.distPathFac > REAL_ZERO) {
		d = Heuristic::getBoundedDist(w0, w1);
		if (d >= Node::COST_INF)
			return Node::COST_INF;

		//context.debug("FTHeuristic::cost(w0 <%.2f %.2f %.2f>, w1 <%.2f %.2f %.2f>, root <%.2f %.2f %.2f>, goal <%.2f %.2f %.2f>\n",
		//	w0.wpos[chainArm].p.x, w0.wpos[chainArm].p.y, w0.wpos[chainArm].p.z, w1.wpos[chainArm].p.x, w1.wpos[chainArm].p.y, w1.wpos[chainArm].p.z, 
		//	root.wpos[chainArm].p.x, root.wpos[chainArm].p.y, root.wpos[chainArm].p.z, goal.wpos[chainArm].p.x, root.wpos[chainArm].p.y, root.wpos[chainArm].p.z);
//		const Real bound = getBoundedDist(w1);
		const Real r = (enable && getBoundedDist(w1) < Node::COST_INF) ? expectedObservationCost(w0, w1) : 1;//(d < 5*bsDesc.ftModelDesc.tactileRange)?getObservationalCost(w0, w1):1;
//		if (r < 1) context.write("rewarded trajectory dist=%f reward %f (enalbe=%s)\n", bound < Node::COST_INF ? bound : -1, r, enable ? "ON" : "OFF");
		c += /*desc.costDesc.distPathFac**/d*r;
//	}
	// rewards more natural approaches (palm towards object)
//	c *= directionApproach(w1);
	
	if (desc.costDesc.distLimitsFac > REAL_ZERO)
		c += desc.costDesc.distLimitsFac*getConfigspaceLimitsDist(w1.cpos);
	if (desc.costDesc.distDfltFac > REAL_ZERO)
		c += desc.costDesc.distDfltFac*getConfigspaceDist(w1.cpos, dfltPos);

	//if (getWorkspaceDist(w1, goal) < ftDrivenDesc.ftModelDesc.distMax)
	//	c += getCollisionCost(w0, w1, samples.begin());

	//context.debug("  bounded distance %.4f\n", d);
	//context.debug("  Euclid. distance %.4f\n", getWorkspaceDist(w1, goal));
	//context.debug("  Mahala. distance %.4f\n", w);
	//context.debug("  total cost func. %.4f\n\n", c);

//	context.debug("FTDrivenHeuristic::cost(enable=%s): preforming time %.7f\n", enableUnc ? "ON" : "OFF", context.getTimer().elapsed() - init);
	return c;
}

/**
computes the bounded distance between the chains' poses (only for chains with enabled obs)
and the reference frame of the samples. If at least one of the distance is <= of the max distance
it returns the distance.. otherwise it returns INF.
*/
golem::Real FTDrivenHeuristic::getBoundedDist(const golem::Waypoint& w) const {
#ifdef _HEURISTIC_PERFMON
	++perfBoundDist;
#endif
	const Real ret = golem::Node::COST_INF;
	if (pBelief.get() && pBelief->getHypotheses().empty())
		return ret;

	Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
	for (Chainspace::Index i = stateInfo.getChains().end() - 1; i >= stateInfo.getChains().begin(); --i){
		const RBCoord c(w.wpos[i]);
		const Real dist = c.p.distance((*maxLhdPose)->toRBPoseSampleGF().p);
		if (dist < ftDrivenDesc.ftModelDesc.distMax)
				return dist;
	}

	//for (Belief::Hypothesis::Seq::const_iterator s = ++pBelief->getHypotheses().begin(); s != pBelief->getHypotheses().end(); ++s) {
	//	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
	//		const ChainDesc* cdesc = getChainDesc()[i];
	//		if (cdesc->enabledObs) {
	//			const RBCoord c(w.wpos[i]);
	//			const Real dist = c.p.distance((*s)->toRBPoseSampleGF().p);
	//			if (dist < this->ftDrivenDesc.ftModelDesc.distMax)
	//				return dist;
	//		}
	//	}
	//}
	return ret;
}

golem::Real FTDrivenHeuristic::getMahalanobisDist(const golem::Waypoint& w0, const golem::Waypoint& goal) const {
//	context.debug("FTHeuristic::getMahalanobisDist()\n");
	golem::Real dist = golem::REAL_ZERO;
		
	for (golem::Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		const ChainDesc* desc = getChainDesc()[i];

		const grasp::RBCoord a(w0.wpos[i]), b(goal.wpos[i]);
		// linear distance
		if (desc->enabledLin) {
	//const Real d0 = covarianceInv[0]*golem::Math::sqr(a[0] - b[0]) + covarianceInv[1]*golem::Math::sqr(a[1] - b[1]) + covarianceInv[2]*golem::Math::sqr(a[2] - b[2]);
	//const Real d1 = covarianceInv[3]*golem::Math::sqr(a[3] - b[3]) + covarianceInv[4]*golem::Math::sqr(a[4] - b[4]) + covarianceInv[5]*golem::Math::sqr(a[5] - b[5]) + covarianceInv[6]*golem::Math::sqr(a[6] - b[6]);
	//const Real d2 = covarianceInv[3]*golem::Math::sqr(a[3] + b[3]) + covarianceInv[4]*golem::Math::sqr(a[4] + b[4]) + covarianceInv[5]*golem::Math::sqr(a[5] + b[5]) + covarianceInv[6]*golem::Math::sqr(a[6] + b[6]);
	//return golem::REAL_HALF*(d0 + std::min(d1, d2));
			const golem::Vec3 p = w0.wpos[i].p - goal.wpos[i].p;
			const golem::Real d = dot(p, pBelief->getSampleProperties().covarianceInv.p).dot(p);
//			context.debug("  Vec p<%.4f,%.4f,%.4f>, p^TC^-1<%.4f,%.4f,%.4f>, d=%.4f\n",
//				p.x, p.y, p.z, dot(p, covarianceInv.p).x, dot(p, covarianceInv.p).y, dot(p, covarianceInv.p).z,
//				d);
			/*if (d < 0.001)
				context.getMessageStream()->write(Message::LEVEL_DEBUG, "MyHeuristic::getMahalanobisDist() w0 <%.2f, %.2f, %.2f> Md %.4f, d %.4f\n", 
					w0.wpos[i].p.x,  w0.wpos[i].p.y,  w0.wpos[i].p.z, d, Heuristic::getLinearDist(w0.wpos[i].p, goal.wpos[i].p));*/
			dist += golem::Math::sqrt(d);
		}
		// angular distance
		if (desc->enabledAng) {
			const golem::Real q = Heuristic::getAngularDist(w0.qrot[i], goal.qrot[i]);
			dist += (golem::REAL_ONE - desc->distNorm)*q;
//			context.debug("  Ang distance %.4f\n", q);
		}
	}
//	context.debug("  Mahalanobis distance %.4f\n", ftDrivenDesc.ftModelDesc.mahalanobisDistFac*dist);
	return ftDrivenDesc.ftModelDesc.mahalanobisDistFac*dist;
}

/*
 J(wi, wj)=SUM_k e^-PSI(wi, wj, k)
 where PSI(wi, wj, k)=||h(wj,p^k)-h(wj,p^1)||^2_Q
*/
Real FTDrivenHeuristic::expectedObservationCost(const golem::Waypoint &wi, const golem::Waypoint &wj) const {
//	context.write("FTDrivenHeuristic::getObservationalCost(const Waypoint &wi, const Waypoint &wj)\n");
	if (pBelief->getHypotheses().size() == 0)
		return REAL_ONE;

	Real cost = REAL_ZERO;
	/*Real boundDist = REAL_ZERO;
	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
		for (U32 k = 1; k <= desc.beliefDesc.nParticles; ++k)
			boundDist += getLinearDist(wj.wpos[i].p, particleSet[k].object.shapes[0]->pose.p); 
	}
	if (boundDist > desc.ftModelDesc.boundFTDist)
		return -1;*/

	std::stringstream str;
	std::vector<Real> hij;
//	for (HypSample::Map::const_iterator k = ++samples.begin(); k != samples.end(); ++k) {
//		Real value = Math::exp(-REAL_HALF*psi(wi, wj, k));
		// for debug purposes I compute psi function here.
		hij.clear();
		h(wi, wj, /*k,*/ hij);
		Real value(REAL_ZERO);
		for (std::vector<Real>::const_iterator i = hij.begin(); i != hij.end(); ++i)
			value += Math::sqr(*i);
		//str << "h(size=" << hij.size() << ") <";
		//for (std::vector<golem::Real>::const_iterator i = hij.begin(); i != hij.end(); ++i)
		//	str << *i << " ";
		//str << "> magnitude=" << Math::sqrt(value) << " e=" << Math::exp(-REAL_HALF*Math::sqrt(value)) << "\n";
		cost += Math::exp(-Math::sqrt(value)); // REAL_HALF*
//	}

//	str << "cost=" << cost/(samples.size() - 1) << "\n";
//	std::printf("%s\n", str.str().c_str());
	return cost/(pBelief->getHypotheses().size() - 1);
}

//Real FTDrivenHeuristic::psi(const Waypoint &wi, const Waypoint &wj, HypSample::Map::const_iterator p) const {
//	context.getMessageStream()->write(Message::LEVEL_DEBUG, "psi(wi, wj) = ||h(p^i) - h(p^0)||_gamma\n");
//	std::vector<Real> hij, gamma;
//	h(wi, wj, p, hij);
//	
//	Real magnitude(REAL_ZERO);
//	for (std::vector<Real>::const_iterator i = hij.begin(); i != hij.end(); ++i)
//		magnitude += Math::sqr(*i);
//
//	context.write("h <");
//	for (std::vector<golem::Real>::const_iterator i = hij.begin(); i != hij.end(); ++i)
//		context.write("%f ", *i);
//	context.write("> magnitude=%f \n", Math::sqrt(magnitude));
//	return Math::sqrt(magnitude);
//	//// generate diagonal matrix
//	//gamma.clear();
//	//gamma.reserve(hij.size());
//	//const Real noise = 2*covarianceDet;
//	//for (size_t i = 0; i < hij.size(); ++i)
//	//	gamma.push_back(noise);
//	//	
//	//// sqrt((h(wi,wj,p^k)-h(wi,wj,p^-1)) \ GAMMA * (h(wi,wj,p^k)-h(wi,wj,p^-1)))
//	//std::vector<Real> v;
//	//dotInverse(hij, gamma, v);
//	//Real d = dot(v, hij);
//	////std::printf("FTHeuristic::h(hij):");
//	////for (std::vector<golem::Real>::const_iterator i = hij.begin(); i != hij.end(); ++i)
//	////	std::printf(" %.10f", *i);
//	////std::printf("\n");
//	////std::printf("FTHeuristic::h(v):");
//	////for (std::vector<golem::Real>::const_iterator i = v.begin(); i != v.end(); ++i)
//	////	std::printf(" %.10f", *i);
//	////std::printf("\n");
//	////std::printf("FTHeuristic::psi():");
//	////for (std::vector<golem::Real>::const_iterator i = gamma.begin(); i != gamma.end(); ++i)
//	////	std::printf(" %.10f", *i);
//	////std::printf("\n");
//	////std::printf("||h(p^%d) - h(p^0)||_gamma = %.10f\n", p->first, d); 
//	//return REAL_HALF*d;
//}

void FTDrivenHeuristic::h(const golem::Waypoint &wi, const golem::Waypoint &wj, std::vector<golem::Real> &y) const {	
//	context.write("FTDrivenHeuristic::h(const Waypoint &wi, const Waypoint &wj, std::vector<golem::Real> &y)\n");
#ifdef _HEURISTIC_PERFMON
	++perfH;
#endif
	auto kernel = [&] (Real x, Real lambda) -> Real {
		return /*lambda**/golem::Math::exp(-lambda*x);
	};
	Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
	const Real norm = (REAL_ONE - /*pBelief->*/kernel(ftDrivenDesc.ftModelDesc.distMax, /*pBelief->*/ftDrivenDesc.ftModelDesc.lambda));//(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.lambda)):REAL_ONE;
	y.clear();
	const U32 meanIdx = 0;
	U32 steps = 1;

	// computes the estimate of contact for the starting PRM node
	//grasp::RealSeq estimates; estimates.assign(pBelief->getHypotheses().size(), REAL_ZERO); U32 hypIndex = 0;
	//for (auto p = pBelief->getHypotheses().cbegin(); p != pBelief->getHypotheses().cend(); ++p)
	//	estimates[hypIndex++] = estimate(p, wi);

	if (false) {
		const Real dist = getDist(wi, wj);	
		steps = (U32)Math::round(dist/(desc.collisionDesc.pathDistDelta));
	}
	y.reserve((pBelief->getHypotheses().size() - 1)*steps/**(armInfo.getChains().size() + handInfo.getJoints().size())*/);

	golem::Waypoint w;
	U32 i = (steps == 1) ? 0 : 1;
	for (; i < steps; ++i) {
			if (steps != 1) {
				for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
					w.cpos[j] = wj.cpos[j] - (wj.cpos[j] - wi.cpos[j])*Real(i)/Real(steps);
		
				// skip reference pose computation
				w.setup(controller, false, true);
			}
			else w = wj;

//		Mat34 closestShape_idx, closestShape_mean;
//		context.getMessageStream()->write(Message::LEVEL_DEBUG, "||h(wj, p^%d) - h(wj, p^0)||\n", idx);
		// evaluated only the end effector(s) of the arm (wrist)
#ifdef _PER_JOINT_
			//for (Chainspace::Index i = armInfo.getChains().begin(); i != armInfo.getChains().end(); ++i) {
			//	const ChainDesc* cdesc = getChainDesc()[i];
			//	if (cdesc->enabledObs) {
			//		// computes the workspace coordinate of the relative chain, if it should be in the obs
			//		const RBCoord c(w.wpos[i]);
			//		// computes the likelihood of contacting the mean pose
			//		const Real likelihood_p1 = norm*pBelief->density((*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));

			//		// computes the likelihood of contacting the other hypotheses
			//		for (Belief::Hypothesis::Seq::const_iterator p = ++pBelief->getHypotheses().begin(); p != pBelief->getHypotheses().end(); ++p) {
			//			const Real likelihood_pi = norm*pBelief->density((*p)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
			//			// the gain is saved in a vector
			//			y.push_back(likelihood_pi - likelihood_p1);
			//		}
			//	}
			//}


		CriticalSection cs;
		Chainspace::Index armIndex = armInfo.getChains().begin();
		ParallelsTask((golem::Parallels*)context.getParallels(), [&] (ParallelsTask*) {
			for (Chainspace::Index i = armInfo.getChains().begin();;) {
				{
					CriticalSectionWrapper csw(cs);
					if (armIndex == armInfo.getChains().end())
						break;
					i = armIndex++;
				}
				const ChainDesc* cdesc = getChainDesc()[i];
				if (cdesc->enabledObs) {
					// computes the workspace coordinate of the relative chain, if it should be in the obs
					const RBCoord c(w.wpos[i]);
					// computes the likelihood of contacting the mean pose
					const Real likelihood_p1 = norm*pBelief->density((*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));

					// computes the likelihood of contacting the other hypotheses
					for (Belief::Hypothesis::Seq::const_iterator p = ++pBelief->getHypotheses().begin(); p != pBelief->getHypotheses().end(); ++p) {
						const Real likelihood_pi = norm*pBelief->density((*p)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
						// the gain is saved in a vector
						CriticalSectionWrapper csw(cs);
						y.push_back(likelihood_pi - likelihood_p1);
					}
				}
			}
		});

			//for (Chainspace::Index i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
			//	const ChainDesc* cdesc = getChainDesc()[i];
			//	// if enable obs is ON than we compute observations (first for the finger [chain] and then for the single joint in the finger)
			//	if (cdesc->enabledObs) {
			//		// compute the observation for the finger tip
			//		const RBCoord c(w.wpos[i]);
			//		// computes the likelihood of contacting the mean pose
			//		const Real likelihood_p1 = norm*pBelief->density((*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
			//		//				std::printf("chain %d pose<%f %f %f> density_sample=%f density_hyp=%f\n", *i, c.p.x, c.p.y, c.p.z, likelihood_pi, likelihood_p1);
			//		// store the computed observations for the finger tip
			//		// computes the likelihood of contacting the other hypotheses
			//		for (Belief::Hypothesis::Seq::const_iterator p = ++pBelief->getHypotheses().begin(); p != pBelief->getHypotheses().end(); ++p) {
			//			const Real likelihood_pi = norm*pBelief->density((*p)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
			//			// the gain is saved in a vector
			//			y.push_back(likelihood_pi - likelihood_p1);
			//		}
			//	}
			//}


		Chainspace::Index handIndex = handInfo.getChains().begin();
		ParallelsTask((golem::Parallels*)context.getParallels(), [&] (ParallelsTask*) {
			// evaluated all the joints of the hand
			for (Chainspace::Index i = handInfo.getChains().begin();;) {
				{
					CriticalSectionWrapper csw(cs);
					if (handIndex == handInfo.getChains().end())
						break;
					i = handIndex++;
				}
				const ChainDesc* cdesc = getChainDesc()[i];
				// if enable obs is ON than we compute observations (first for the finger [chain] and then for the single joint in the finger)
				if (cdesc->enabledObs) {
					// compute the observation for the finger tip
					const RBCoord c(w.wpos[i]);
					// computes the likelihood of contacting the mean pose
					const Real likelihood_p1 = norm*pBelief->density((*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
	//				std::printf("chain %d pose<%f %f %f> density_sample=%f density_hyp=%f\n", *i, c.p.x, c.p.y, c.p.z, likelihood_pi, likelihood_p1);
					// store the computed observations for the finger tip
					// computes the likelihood of contacting the other hypotheses
					for (Belief::Hypothesis::Seq::const_iterator p = ++pBelief->getHypotheses().begin(); p != pBelief->getHypotheses().end(); ++p) {
						const Real likelihood_pi = norm*pBelief->density((*p)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
						// the gain is saved in a vector
						CriticalSectionWrapper csw(cs);
						y.push_back(likelihood_pi - likelihood_p1);
					}
				}
			}
		
}); // end parallels task
#endif
		//U32 index = 0;

		//const Parallels* parallels = context.getParallels();
		//if (parallels != NULL) {
		//	const Job* job = parallels->getCurrentJob();
		//	if (job != NULL) {
		//		index = job->getJobId();
		//	}
		//}
		//auto discountedEstimation = [&](const Hypothesis::Seq::const_iterator& ptr, const Waypoint& w, const U32 prev) -> Real {
		//	const Real next = estimate(ptr, w);
		//	const Real delta = Math::exp(next - prev);
		//	return next * (delta > 1 ? 1 : delta);
		//};
		//const Real estimate_p1 = estimate(pBelief->getHypotheses().cbegin(), w);
		const Real likelihood_p1 = estimate(pBelief->getHypotheses().cbegin(), w); //Math::exp(estimate_p1 - estimates[0]);//pBelief->getHypotheses().front()->evaluate(ftDrivenDesc.evaluationDesc, manipulator->getPose(w.cpos), testCollision);
		//hypIndex = 1;
		for (auto p = ++pBelief->getHypotheses().cbegin(); p != pBelief->getHypotheses().cend(); ++p) {
		//for (auto j = 1; j < pBelief->getHypotheses().size(); ++j) {
			//const Real likelihood_pi = pBelief->getHypotheses()[j]->evaluate(ftDrivenDesc.evaluationDesc, manipulator->getPose(w.cpos), testCollision);
			//			y.push_back(likelihood_pi - likelihood_p1);
			//y.push_back(Math::exp(estimates[hypIndex++] - estimate(p, w)) - likelihood_p1);
			y.push_back(estimate(p, w) - likelihood_p1);
		}

		//for (Belief::Hypothesis::Seq::const_iterator p = ++pBelief->getHypotheses().begin(); p != pBelief->getHypotheses().end(); ++p) {
		//	const Real likelihood_pi = collision->evaluate(waypoint, (*p)->getCloud(), golem::Rand(rand), manipulator->getPose(w.cpos), testCollision);
		//	y.push_back(likelihood_pi - likelihood_p1);
		//}
	}
}

Real FTDrivenHeuristic::evaluate(const Hypothesis::Seq::const_iterator &hypothesis, const golem::Waypoint &w) const {
	Real eval = REAL_ZERO;
	golem::Controller::State state = manipulator->getController().createState();
	manipulator->getController().lookupState(golem::SEC_TM_REAL_MAX, state);
	// position control
	state.cpos = w.cpos;
	grasp::Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
	if (intersect(manipulator->getBounds(config.config, config.frame.toMat34()), (*hypothesis)->bounds(), false))
		eval = (*hypothesis)->evaluate(ftDrivenDesc.evaluationDesc, config.config);
	else {
		const Real dist = config.frame.toMat34().p.distance((*hypothesis)->toRBPoseSampleGF().p);
		if (dist < ftDrivenDesc.ftModelDesc.distMax)
			eval = dist;
	}
	return eval;
}

//------------------------------------------------------------------------------

Real FTDrivenHeuristic::directionApproach(const golem::Waypoint &w) const {
	const Chainspace::Index armIndex = armInfo.getChains().begin();
	
	Vec3 v;
	Mat34 tcpFrameInv, hypothesis;
	hypothesis = (*pBelief->getHypotheses().begin())->toRBPoseSampleGF().toMat34();
	tcpFrameInv.setInverse(w.wpos[armIndex]);
	tcpFrameInv.multiply(v, hypothesis.p);
	v.normalise();
	return v.z > 0 ? REAL_ONE - ftDrivenDesc.directionFac : REAL_ONE;
}

//------------------------------------------------------------------------------

bool FTDrivenHeuristic::collides(const golem::Waypoint &w, ThreadData* data) const {
#ifdef _HEURISTIC_PERFMON
	++perfCollisionWaypoint;
#endif
	// check for collisions with the object to grasp. only the hand
	if (pointCloudCollision && !pBelief->getHypotheses().empty()) {
		Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
		grasp::Manipulator::Config config(w.cpos, manipulator->getBaseFrame(w.cpos));
		if (intersect(manipulator->getBounds(config.config, config.frame.toMat34()), (*maxLhdPose)->bounds(), false)) {
#ifdef _HEURISTIC_PERFMON
			++perfCollisionPointCloud;
#endif
			if ((*maxLhdPose)->checkNN(ftDrivenDesc.checkDesc, config))
				return true;
		}
	}

	return Heuristic::collides(w, data);
}

bool FTDrivenHeuristic::collides(const golem::Waypoint &w0, const golem::Waypoint &w1, ThreadData* data) const {
	const Real dist = getDist(w0, w1);
	const U32 size = (U32)Math::round(dist / desc.collisionDesc.pathDistDelta) + 1;
	Real p[2];
	golem::Waypoint w;

#ifdef _HEURISTIC_PERFMON
	++perfCollisionPath;
	perfCollisionSegs += size - 1;
#endif

	// test for collisions in the range (w0, w1) - excluding w0 and w1
	for (U32 i = 1; i < size; ++i) {
		p[0] = Real(i) / Real(size);
		p[1] = REAL_ONE - p[0];

		// lineary interpolate coordinates
		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
			w.cpos[j] = p[0] * w0.cpos[j] + p[1] * w1.cpos[j];

		// skip reference pose computation
		w.setup(controller, false, true);

		// test for collisions
		if (collides(w, data))
			return true;
	}

	return false;
}

//------------------------------------------------------------------------------

bool FTDrivenHeuristic::collides(const golem::Waypoint &w) const {
	if (!desc.collisionDesc.enabled && !pointCloudCollision)
		return false;

	// find data pointer for the current thread
	ThreadData* data = getThreadData();

	return collides(w, data);
}

bool FTDrivenHeuristic::collides(const golem::Waypoint &w0, const golem::Waypoint &w1) const {
	if (!desc.collisionDesc.enabled && !pointCloudCollision)
		return false;

	// find data pointer for the current thread
	ThreadData* data = getThreadData();

	return collides(w0, w1, data);
}


//------------------------------------------------------------------------------

golem::Real FTDrivenHeuristic::getCollisionCost(const golem::Waypoint &wi, const golem::Waypoint &wj, Hypothesis::Seq::const_iterator p) const {
	// Create the normal estimation class, and pass the input dataset to it
	//pcl::KdTree<pcl::PointXYZ>::PointCloudConstPtr cloud = p->second->pTree->getInputCloud();
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	//ne.setInputCloud(cloud);

	//// Create an empty kdtree representation, and pass it to the normal estimation object.
	//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	//ne.setSearchMethod(tree);

	//// Output datasets
	//pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);

	//// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch (0.01);

	//// Compute the features
	//ne.compute(*cloudNormals);		// compute the normal at the closest point of the surface
	auto kernel = [&](Real x, Real lambda) -> Real {
		return /*lambda**/golem::Math::exp(-lambda*x);
	};

	auto density = [&](const Real dist) -> Real {
		return (dist > ftDrivenDesc.ftModelDesc.distMax) ? REAL_ZERO : (dist < REAL_EPS) ? kernel(REAL_EPS, ftDrivenDesc.ftModelDesc.lambda) : kernel(dist, ftDrivenDesc.ftModelDesc.lambda); // esponential up to norm factor
	};

	Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
	const Real norm = (REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax/*pBelief->myDesc.sensory.sensoryRange*/));//const Real norm = //(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.lambda*ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
	Real threshold(0.01), cost(REAL_ZERO);
	for (Chainspace::Index i = handInfo.getChains().begin(); i < handInfo.getChains().end(); ++i) {
		const RBCoord c(wj.wpos[i]);

		const Real d = 0;// (*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces);
//		context.write("FTDrivenHeuristic::getCollisionCost(): distance=%4.5f, cost=%4.5f\n", d, d < threshold ? Node::COST_INF : REAL_ZERO);
		// penalise collisions with the shape of the mean pose
		if (d < threshold)
			return golem::Node::COST_INF;

		//Real distMin(REAL_MAX), dist;
		//size_t k = 0, kMin;
		//for (; k != cloud->size(); ++k) { 
		//	if (dist = c.p.distance(Vec3(cloud->points[k].x, cloud->points[k].y, cloud->points[k].z)) < distMin) {
		//		distMin = dist;
		//		kMin = k;
		//	}
		//}
		// compute the cost relative to the approaching direction
		//Vec3 approachDir = wi.wpos[i].p - wj.wpos[i].p;
		//Vec3 normal(cloudNormals->points[kMin].normal[0], cloudNormals->points[kMin].normal[1], cloudNormals->points[kMin].normal[2]);
		//cost += normal.dot(approachDir);
	}

	return cost;
}

Real FTDrivenHeuristic::testObservations(const grasp::RBCoord &pose, const bool normal) const {
//	const Real norm = (ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
	auto kernel = [&](Real x, Real lambda) -> Real {
		return /*lambda**/golem::Math::exp(-lambda*x);
	};

	auto density = [&](const Real dist) -> Real {
		return (dist > ftDrivenDesc.ftModelDesc.distMax) ? REAL_ZERO : (dist < REAL_EPS) ? kernel(REAL_EPS, ftDrivenDesc.ftModelDesc.lambda) : kernel(dist, ftDrivenDesc.ftModelDesc.lambda); // esponential up to norm factor
	};
	const Real norm = (REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax));//(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.lambda)):REAL_ONE;
	pcl::PointXYZ searchPoint;
	searchPoint.x = (float)pose.p.x;
	searchPoint.y = (float)pose.p.y;
	searchPoint.z = (float)pose.p.z;

	std::vector<int> indeces;
	std::vector<float> distances;
//	pcl::KdTree<pcl::PointXYZ>::PointCloudConstPtr cloud = (*pBelief->getHypotheses().begin())->pTree->getInputCloud();
	Real result = REAL_ZERO;
	Vec3 median;
	median.setZero();
	//if ((*pBelief->getHypotheses().begin())->pTree->nearestKSearch(searchPoint, ftDrivenDesc.ftModelDesc.k, indeces, distances) > 0) {
	//	for (size_t i = 0; i < indeces.size(); ++i) {
	//		//Point point;
	//		//point.frame.setId();
	//		//point.frame.p.set(cloud->points[indeces[i]].x, cloud->points[indeces[i]].y, cloud->points[indeces[i]].z);
	//		//result += pose.p.distance(point.frame.p);
	//		//median += Vec3(cloud->points[indeces[i]].x, cloud->points[indeces[i]].y, cloud->points[indeces[i]].z);
	//		result += pose.p.distance(Vec3(cloud->points[indeces[i]].x, cloud->points[indeces[i]].y, cloud->points[indeces[i]].z));
	//		median += Vec3(cloud->points[indeces[i]].x, cloud->points[indeces[i]].y, cloud->points[indeces[i]].z);
	//	}
	//	result /= indeces.size();
	//	median /= indeces.size();
	//}
	
	context.write("FTDrivenHeuristic:testObs(normal=%s):\n", normal ? "ON" : "OFF");
	if (normal) {
		Vec3 v;
		Mat34 jointFrame(Mat33(pose.q), pose.p);
		Mat34 jointFrameInv;
		jointFrameInv.setInverse(jointFrame);
		jointFrameInv.multiply(v, (*pBelief->getHypotheses().begin())->sample.p);
		v.normalise();
//		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
//		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
		if (v.z < 0 /*|| v.z < ftDrivenDesc.ftModelDesc.coneTheta1*/)
			result = ftDrivenDesc.ftModelDesc.distMax + 1;

		context.write("joint pose <%f %f %f> mug frame <%f %f %f> v <%f %f %f> distance=%f result=%f density=%f\n",
			pose.p.x, pose.p.y, pose.p.z, (*pBelief->getHypotheses().begin())->sample.p.x, (*pBelief->getHypotheses().begin())->sample.p.y, (*pBelief->getHypotheses().begin())->sample.p.z,
			v.x, v.y, v.z, pose.p.distance(median), result, norm*density(result));
		//else {
		//	Mat33 rot;
		//	Real roll, pitch, yaw;
		//	Quat quat(pose.q);
		//	quat.toMat33(rot);
		//	rot.toEuler(roll, pitch, yaw);
		//	std::printf("nearest(): pose [%.4f,%.4f,%.4f]<%.4f,%.4f,%.4f>; median <%.4f,%.4f,%.4f>, thx %.4f, thy %.4f\n",
		//		roll, pitch, yaw, pose.p.x, pose.p.y, pose.p.z, median.x, median.y, median.z, thx, thy);
		//}
	}
	else {
	context.write("joint pose <%f %f %f> mug frame <%f %f %f> distance=%f result=%f density=%f\n",
		pose.p.x, pose.p.y, pose.p.z, (*pBelief->getHypotheses().begin())->sample.p.x, (*pBelief->getHypotheses().begin())->sample.p.y, (*pBelief->getHypotheses().begin())->sample.p.z,
		pose.p.distance(median), result, norm*density(result));
	}
	return result;
}


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
	XMLData(val.evaluationDesc, context->getContextFirst("evaluation_model"), create);
	XMLData(val.checkDesc, context->getContextFirst("check_model"), create);
}
