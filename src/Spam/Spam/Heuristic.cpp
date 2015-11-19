/** @file Heuristic.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Spam/Spam/Heuristic.h>
#include <Golem/Device/MultiCtrl/MultiCtrl.h>
#include <Golem/Plan/Data.h>

//#ifdef WIN32
//	#pragma warning (push)
//	#pragma warning (disable : 4244)
//	#pragma warning (disable : 4996)
//#endif
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#ifdef WIN32
//	#pragma warning (pop)
//#endif

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

//bool FTDrivenHeuristic::HypSample::build() {
//	pTree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>);
//
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
//	cloud->width = points.size();
//	cloud->height = 1;
//	cloud->points.resize (cloud->width * cloud->height);
//	int j = 0;
//	for (Cloud::PointSeq::const_iterator point = points.begin(); point != points.end(); ++point) {
//		cloud->points[j].x = (float)point->x;
//		cloud->points[j].y = (float)point->y;
//		cloud->points[j++].z = (float)point->z;
//	}
//	pTree->setInputCloud(cloud);
//
//	return true;
//}
//
//bool FTDrivenHeuristic::HypSample::buildMesh() {
//	pTriangles.reset(new pcl::PolygonMesh);
//	// build point cloud form set of points
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
//	cloud->width = points.size();
//	cloud->height = 1;
//	cloud->points.resize(cloud->width * cloud->height);
//	int j = 0;
//	for (Cloud::PointSeq::const_iterator point = points.begin(); point != points.end(); ++point) {
//		cloud->points[j].x = (float)point->x;
//		cloud->points[j].y = (float)point->y;
//		cloud->points[j++].z = (float)point->z;
//	}
//
//	  // Normal estimation*
//	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//	tree->setInputCloud(cloud);
//	n.setInputCloud(cloud);
//	n.setSearchMethod(tree);
//	n.setKSearch(20);
//	n.compute(*normals);
//	//* normals should not contain the point normals + surface curvatures
//
//	// Concatenate the XYZ and normal fields*
//	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
//	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
//	//* cloud_with_normals = cloud + normals
//
//	// Create search tree*
//	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
//	tree2->setInputCloud(cloud_with_normals);
//
//	// Initialize objects
//	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
//	pcl::PolygonMesh triangles;
//
//	// Set the maximum distance between connected points (maximum edge length)
//	gp3.setSearchRadius(0.025);
//
//	// Set typical values for the parameters
//	gp3.setMu(2.5);
//	gp3.setMaximumNearestNeighbors(1000);
//	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//	gp3.setMinimumAngle(M_PI/18); // 10 degrees
//	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//	gp3.setNormalConsistency(false);
//
//	// Get result
//	gp3.setInputCloud(cloud_with_normals);
//	gp3.setSearchMethod(tree2);
//	gp3.reconstruct(*pTriangles.get());
//	std::printf("HypSample::buildMesh: Triangle vertices %d\n", pTriangles->polygons.size());
//	//for (std::vector<pcl::Vertices>::iterator i = pTriangles->polygons.begin(); i != pTriangles->polygons.end(); ++i)
//	//	std::printf("Vertex n.%d size %d\n", i, i->vertices.size());
//	
//	return true;
//}


//------------------------------------------------------------------------------
//
//inline golem::Real getLinearDist(const golem::Vec3& v0, const golem::Vec3& v1) {
//	return v0.distance(v1);
//}
//
//inline golem::Real getAngularDist(const golem::Quat& q0, const golem::Quat& q1) {
//	const golem::Real d = q0.dot(q1);
//	return golem::REAL_ONE - golem::Math::abs(d);
//}
//
//inline golem::Real distance(const golem::Mat34 &wpos, const GraspableObject::Desc &p) {
//	golem::Real min = golem::REAL_MAX;
//	for (golem::Bounds::Desc::Seq::const_iterator i = p.shapes.begin(); i != p.shapes.end(); ++i) {
//		const golem::Real d = getLinearDist(wpos.p, i->get()->pose.p);
//		if (d < min)
//			min = d;
//	}
//	return min;
//}
//
//inline golem::Mat34 getClosestShapePose(const golem::Mat34 &wpos, const GraspableObject::Desc &p) {
//	golem::Real min = Node::COST_INF;
//	golem::Mat34 minPose;
//	minPose.setId();
//	for (golem::Bounds::Desc::Seq::const_iterator i = p.shapes.begin(); i != p.shapes.end(); ++i) {
//		const golem::Real d = getLinearDist(wpos.p, i->get()->pose.p);
//		if (d < min) {
//			min = d;
//			minPose.p.set(i->get()->pose.p);
//			minPose.R.set(i->get()->pose.R);
//		}
//	}
////	printf("ClosestShapePose() <%.2f %.2f %.2f>\n", minPose.p.x, minPose.p.y, minPose.p.z);
//	return minPose;
//}

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

//inline void toMat34(const RBCoord &c, Mat34 &m) {
//	m.p.set(c.p);
//	m.R.fromQuat(c.q);
//}

//inline void generateGamma(const size_t size, const Real &noise, std::vector<Real> &gamma) {
//	gamma.clear();
//	gamma.reserve(size);
//	const Real two_noise = 2*noise;
//	for (size_t i = 0; i < size; ++i)
//		gamma.push_back(two_noise);
////	std::fill(gamma.begin(), gamma.end(), noiseSqr);
//}
//
//inline Real density(const Real &x, const Real &lambda, const Real bound) {
//	if (x > bound)
//		return REAL_ZERO;
//	
//	const Real eta = REAL_ONE/(REAL_ONE - Math::exp(-lambda*bound));
//	return eta*Math::exp(-lambda*x);
//}
//
//}; // namespace spam for static functions
//
//Real spam::density(const RBCoord &x, const RBCoord &mean, const RBCoord &covariance, const Real bound) {
//	Real determinant, eta, e;
//	const Real dist = getLinearDist(x.p, mean.p);
//	if (dist > bound) {
////		printf("density() 0 (||x-p||=%.4f>%.4f)\n", dist, bound);
//		return 0;
//	}
//	RBCoord p = x - mean;
//	p.q.setZero();
//	determinant = Math::sqrt(covariance.det());
//	eta = (REAL_ONE/(Math::pow<Real>(REAL_2_PI, grasp::RBCoord::N/2.0)*determinant));
//	e = (p/covariance).dot(p);
////	printf("det %.4f, eta %.4f, e %.4f distance lin %.6f ang %.6f\n", determinant, eta, e, getLinearDist(x.p, mean.p), getAngularDist(x.q, mean.q));
//	Real c = eta*Math::exp(-REAL_HALF*e);
////	printf("density() %.10f (||x-p||=%.4f)\n", c, dist);
//	return c;
//}

//------------------------------------------------------------------------------

FTDrivenHeuristic::FTDrivenHeuristic(golem::Controller &controller) : Heuristic(controller), rand(context.getRandSeed()) {}

bool FTDrivenHeuristic::create(const Desc &desc) {
//	context.debug("FTDrivenHeuristic::create()...\n");
	Heuristic::create((Heuristic::Desc&)desc);

	// printing of debug
//	context.write("contact_fac=%f, max_points=%d, covariance=%f\n", desc.contactFac, desc.maxSurfacePoints, desc.covariance.p[0]);

	ftDrivenDesc = desc;
//	context.write("Heuristic params: dflt=%f limits=%f root=%f\n", desc.costDesc.distDfltFac, desc.costDesc.distLimitsFac, desc.costDesc.distRootFac);
	ftDrivenDesc.ftModelDesc = desc.ftModelDesc;
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

//void FTDrivenHeuristic::setModel(Cloud::PointSeq::const_iterator begin, Cloud::PointSeq::const_iterator end, const Mat34 &transform) {
//	//modelPoints.clear();
//	//for (Cloud::PointSeq::const_iterator i = begin; i != end; ++i) {
//	//	Point point = *i;
//	//	point.frame.multiply(transform, point.frame);
//	//	modelPoints.push_back(point);
//	//}
//	modelPoints.clear();
//	modelPoints.insert(modelPoints.begin(), begin, end);
//	Cloud::transform(transform, modelPoints, modelPoints);
//}
//
//void FTDrivenHeuristic::setBeliefState(RBPose::Sample::Seq &samples, const golem::Mat34 &transform) {
//	U32 idx = 0;
//	this->samples.clear();
//	context.write("Heuristic:setBeliefState(model size = %d, max points = %d): samples: cont_fac = %f\n", modelPoints.size(), ftDrivenDesc.maxSurfacePoints, ftDrivenDesc.contactFac);
//	for (RBPose::Sample::Seq::const_iterator s = samples.begin(); s != samples.end(); ++s) {
//		const Mat34 sampleFrame(Mat34(s->q, s->p) * transform);
//		RBPose::Sample sample(RBCoord(sampleFrame), s->weight, s->cdf);
////		this->samples.push_back(RBPose::Sample(RBCoord(sampleFrame), s->weight, s->cdf));
//		Cloud::PointSeq sampleCloud;
//		//for (Point::Seq::const_iterator i = modelPoints.begin(); i != modelPoints.end(); ++i) {
//		//	Point p = *i;
//		//	p.frame.multiply(sampleFrame, p.frame);
//		//	sampleCloud.push_back(p);
//		//}
//
//		// limits the number of point inserted into the kd-tree to save performance while quering it
//		const size_t size = modelPoints.size() < ftDrivenDesc.maxSurfacePoints ? modelPoints.size() : ftDrivenDesc.maxSurfacePoints;
//		for (size_t i = 0; i < size; ++i) {
//			// CHECK: Points from modelPoints are modified multiple times (loop over samples, then nested loop)!
//
//			//Point& p = size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]; // this is reference!
//			//p.frame.multiply(sampleFrame, p.frame);
//			//sampleCloud.push_back(p);
//			Cloud::Point p = size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]; // make a copy here
//			Cloud::setPoint(sampleFrame * Cloud::getPoint(p), p);
//			sampleCloud.push_back(p);
//		}
//		this->samples.insert(HypSample::Map::value_type(idx, HypSample::Ptr(new HypSample(idx, sample, sampleCloud))));
//		context.write(" - sample n.%d <%.4f %.4f %.4f> <%.4f %.4f %.4f %.4f>\n", idx, sample.p.x, sample.p.y, sample.p.z, sample.q.w, sample.q.x, sample.q.y, sample.q.z); 
//		idx++;
////		samplePoints.insert(std::map<U32, Point::Seq>::value_type(idx++, sampleCloud));
//		//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//		//kdtree.setInputCloud(cloud);
//		//queryPoints.insert(std::map<RBPose::Sample::Seq::const_iterator, pcl::KdTreeFLANN<pcl::PointXYZ>>::value_type(s, kdtree));
//	}
//	
//	// mean and covariance
//	if (!sampleProperties.create<golem::Ref1, RBPose::Sample::Ref>(ftDrivenDesc.covariance, samples))
//		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
//		
//	context.write("spam::Belief::createQuery(TEST): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
//	
//	covarianceDet = REAL_ONE;
//	for (golem::U32 j = 0; j < RBCoord::N; ++j)
//			covarianceDet *= sampleProperties.covariance[j];	
//	
//	// compute the covariance matrix
//	//RBCoord mean, covariance, c;
//	//const Real norm = REAL_ONE/samples.size();
//
//	//mean.setZero();
//	//c.setZero();
//	//for (RBPose::Sample::Seq::const_iterator i = samples.begin(); i != samples.end(); ++i)
//	//	golem::kahanSum(&mean[0], &c[0], &(*i)[0], RBCoord::N);
//	//for (golem::U32 j = 0; j < RBCoord::N; ++j)
//	//	mean[j] *= norm;
//	//
//	//covariance.setZero();
//	//c.setZero();
//	//for (RBPose::Sample::Seq::const_iterator i = samples.begin(); i != samples.end(); ++i)
//	//	for (golem::U32 j = 0; j < RBCoord::N; ++j)
//	//		golem::kahanSum(covariance[j], c[j], golem::Math::sqr((*i)[j] - mean[j]));
//	//for (golem::U32 j = 0; j < RBCoord::N; ++j)
//	//	covariance[j] *= norm*ftDrivenDesc.covariance[j];
//	////covariance.q.setId();
//	////covariance.p.set(Vec3(.04, .025, .0001));
//
//	//covarianceDet = REAL_ONE;
//	//for (golem::U32 j = 0; j < RBCoord::N; ++j)
//	//		covarianceDet *= covariance[j];	
//
//	//if (covarianceDet < REAL_EPS)
//	//	covarianceDet = REAL_EPS;
//
//	//while (covarianceDet < ftDrivenDesc.covarianceDetMin)
//	//	covarianceDet *= ftDrivenDesc.covarianceDet;
//	//context.debug("FTDrivenHeuristic::setBeliefState: covariance hypothesis = {(%.f, %f, %f) (%f, %f, %f, %f) det=%.10f}\n", covariance[3], covariance[4], covariance[5], covariance[6], covariance[0], covariance[1], covariance[2], covarianceDet);
//
//	//for (golem::U32 j = 0; j < RBCoord::N; ++j) {
//	//	covarianceInv[j] = REAL_ONE/(covariance[j]);
//	//	covarianceSqrt[j] = Math::sqrt(covariance[j]);
//	//}
//}

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
	grasp::Manipulator::Config config(w.cpos);
	if (intersect(manipulator->getBounds(config.config, config.frame.toMat34()), (*hypothesis)->bounds(), false))
		eval = (*hypothesis)->evaluate(ftDrivenDesc.evaluationDesc, config.config);
	else {
		const Real dist = config.frame.toMat34().p.distance((*hypothesis)->toRBPoseSampleGF().p);
		if (dist < ftDrivenDesc.ftModelDesc.distMax)
			eval = dist;
	}
	return eval;
}


//void FTDrivenHeuristic::h(const golem::Waypoint &wi, const golem::Waypoint &wj, Belief::Hypothesis::Seq::const_iterator p, std::vector<golem::Real> &y) const {
//	Belief::Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
//	const Real norm = (ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - kernel(ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.lambda)):REAL_ONE;
//	y.clear();
//	const U32 meanIdx = 0;
//	U32 steps = 1;
//
//	if (false) {
//		const Real dist = getDist(wi, wj);	
//		steps = (U32)Math::round(dist/(desc.collisionDesc.pathDistDelta*4));
//	}
//	y.reserve(steps*(armInfo.getChains().size() + handInfo.getJoints().size()));
//
//	Waypoint w;
//	U32 i = (steps == 1) ? 0 : 1;
//	for (; i < steps; ++i) {
//		if (steps != 1) {
//			for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
//				w.cpos[j] = wj.cpos[j] - (wj.cpos[j] - wi.cpos[j])*Real(i)/Real(steps);
//		
//			// skip reference pose computation
//			w.setup(controller, false, true);
//		}
//		else w = wj;
//
////		Mat34 closestShape_idx, closestShape_mean;
////		context.getMessageStream()->write(Message::LEVEL_DEBUG, "||h(wj, p^%d) - h(wj, p^0)||\n", idx);
//		// evaluated only the end effector(s) of the arm (wrist)
//		for (Chainspace::Index i = armInfo.getChains().begin(); i < armInfo.getChains().end(); ++i) {
//			const ChainDesc* cdesc = getChainDesc()[i];
//			if (cdesc->enabledObs) {
//				const RBCoord c(w.wpos[i]);
//				const Real likelihood_pi = norm*density((*p)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
//				const Real likelihood_p1 = norm*density((*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
//				
//				y.push_back(likelihood_pi - likelihood_p1);
//			}
//		}
//		// evaluated all the joints of the hand
//		for (Chainspace::Index i = handInfo.getChains().begin(); i < handInfo.getChains().end(); ++i) {
//			const ChainDesc* cdesc = getChainDesc()[i];
//			// if enable obs is ON than we compute observations (first for the finger [chain] and then for the single joint in the finger)
//			if (cdesc->enabledObs) {
//				// compute the observation for the finger tip
//				const RBCoord c(w.wpos[i]);
//				const Real likelihood_pi = norm*density((*p)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
//				const Real likelihood_p1 = norm*density((*maxLhdPose)->dist2NearestKPoints(c, ftDrivenDesc.ftModelDesc.distMax, ftDrivenDesc.ftModelDesc.k, ftDrivenDesc.numIndeces));
////				std::printf("chain %d pose<%f %f %f> density_sample=%f density_hyp=%f\n", *i, c.p.x, c.p.y, c.p.z, likelihood_pi, likelihood_p1);
//				// store the computed observations for the finger tip
//				y.push_back(likelihood_pi - likelihood_p1);
//				// for each enabled joint we compute the observations
////				if (false) {
////				for (Configspace::Index j = handInfo.getJoints(i).begin(); j < handInfo.getJoints(i).end(); ++j) {
////					const JointDesc* jdesc = getJointDesc()[j];
//////					if (jdesc->enabled) {
////						const RBCoord c(w.wposex[j]);
////						const Real distToHyp = dist2NearestKPoints(c, maxLhdPose);
////						Real distToSample = dist2NearestKPoints(c, p);
////						// check if the fingertip is inside the object 
////						//if (j == handInfo.getJoints(i).end() - 1 && distance(c, p->second->sample) < ftDrivenDesc.ftModelDesc.distMax) 
////						//	distToSample = distToHyp;
////						const Real likelihood_pi = norm*density(distToSample);
////						const Real likelihood_p1 = norm*density(distToHyp);
////				//Real d_idx, d_mean;
////				//if (ftDrivenDesc.ftModelDesc.enabledLikelihood && !ftDrivenDesc.ftModelDesc.enabledExpLikelihood) {
////				//	d_idx = density(RBCoord(w.wpos[i]), RBCoord(closestShape_idx), covariance, ftDrivenDesc.ftModelDesc.tactileRange);
////				//	d_mean = density(RBCoord(w.wpos[i]), RBCoord(closestShape_mean), covariance, ftDrivenDesc.ftModelDesc.tactileRange);
////				//}
////				//else {
////				//	const Real distToIdx  = getLinearDist(w.wpos[i].p, closestShape_idx.p);
////				//	const Real distToMean = getLinearDist(w.wpos[i].p, closestShape_mean.p);
////				//	d_idx = (ftDrivenDesc.ftModelDesc.enabledLikelihood)?distToIdx:density(distToIdx, ftDrivenDesc.ftModelDesc.lambda, ftDrivenDesc.ftModelDesc.tactileRange);
////				//	d_mean = (ftDrivenDesc.ftModelDesc.enabledLikelihood)?distToMean:density(distToMean, ftDrivenDesc.ftModelDesc.lambda, ftDrivenDesc.ftModelDesc.tactileRange);
////				//}
////						y.push_back(likelihood_pi - likelihood_p1);
//////					}
////	//				context.getMessageStream()->write(Message::LEVEL_DEBUG, "  %.5f  -   %.5f  = %.5f\n", d_idx, d_mean, d_idx - d_mean);
////				}
////				}
//			}
//		}
//	}
//}
//	
//golem::Real FTDrivenHeuristic::h(const Waypoint &wij, U32 idx) const {
//	if (collides(wij)) 
//		return Node::COST_INF;
//
//	Real cost = REAL_ZERO;
//	Mat34 closestShape;
//	for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i) {
//		closestShape = getClosestShapePose(wij.wpos[i], particleSet[idx].object);
//		const Real d = getLinearDist(wij.wpos[i].p, closestShape.p);//density(RBCoord(wij.wpos[i]), RBCoord(closestShape), bsDesc.beliefDesc.covariance, bsDesc.ftModelDesc.tactileRange);
//		cost += d;
//	}
//	cost /= stateInfo.getChains().size();
////	printf("p.weigth %.6f, eta_hit %.4f, cost %.4f\n", particleSet[idx].weight, hitNormFacInv, cost);
//	return particleSet[idx].weight*/*hitNormFacInv**/cost;
//}

//------------------------------------------------------------------------------

//Real FTDrivenHeuristic::dist2NearestKPoints(const grasp::RBCoord &pose,  Belief::Hypothesis::Seq::const_iterator p, const bool normal) const {
//	if (pose.p.distance(p->second->sample.p) > (REAL_ONE + REAL_HALF)*ftDrivenDesc.ftModelDesc.distMax)
//		return ftDrivenDesc.ftModelDesc.distMax + 1;;
//
//	SecTmReal init = context.getTimer().elapsed();
//	if (!ftDrivenDesc.knearest)
//		return dist2NearestPoint(pose, p, normal);
//	if (ftDrivenDesc.trimesh)
//		return dist2NearestTriangle(pose, p, normal);
//
//	pcl::PointXYZ searchPoint;
//	searchPoint.x = (float)pose.p.x;
//	searchPoint.y = (float)pose.p.y;
//	searchPoint.z = (float)pose.p.z;
//
//	std::vector<int> indeces;
//	std::vector<float> distances;
//	pcl::KdTree<pcl::PointXYZ>::PointCloudConstPtr cloud = p->second->pTree->getInputCloud();
//	Real result = REAL_ZERO;
//	Vec3 median;
//	median.setZero();
//	if (p->second->pTree->nearestKSearch(searchPoint, ftDrivenDesc.ftModelDesc.k, indeces, distances) > 0) {
//		const size_t size = indeces.size() < ftDrivenDesc.numIndeces ? indeces.size() : ftDrivenDesc.numIndeces;
//		for (size_t i = 0; i < size; ++i) {
//			idxdiff_t idx = size < indeces.size() ? indeces[size_t(rand.next())%indeces.size()] : indeces[i];
//			//Point point;
//			//point.frame.setId();
//			//point.frame.p.set(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
//			//result += pose.p.distance(point.frame.p);
//			//median += Vec3(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
//			result += pose.p.distance(Vec3(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z));
//			median += Vec3(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
//		}
//		result /= size;
//		median /= size;
//	}
//	
//	if (normal) {
//		Vec3 v;
//		Mat34 jointFrame(Mat33(pose.q), pose.p);
//		Mat34 jointFrameInv;
//		jointFrameInv.setInverse(jointFrame);
//		jointFrameInv.multiply(v, median);
//		v.normalise();
////		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
////		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
//		if (v.z < 0 || v.z < ftDrivenDesc.ftModelDesc.coneTheta1)
//			result = ftDrivenDesc.ftModelDesc.distMax + 1;
//		//else {
//		//	Mat33 rot;
//		//	Real roll, pitch, yaw;
//		//	Quat quat(pose.q);
//		//	quat.toMat33(rot);
//		//	rot.toEuler(roll, pitch, yaw);
//		//	std::printf("nearest(): pose [%.4f,%.4f,%.4f]<%.4f,%.4f,%.4f>; median <%.4f,%.4f,%.4f>, thx %.4f, thy %.4f\n",
//		//		roll, pitch, yaw, pose.p.x, pose.p.y, pose.p.z, median.x, median.y, median.z, thx, thy);
//		//}
//	}
//	//if (result < ftDrivenDesc.ftModelDesc.distMax + 1) {
//	//	std::printf("NearestKNeigh (t %.7f) %.7f\n", context.getTimer().elapsed() - init, result);
//	//	dist2NearestPoint(pose, p, normal);
//	//}
//	return result;
//}
//
//Real FTDrivenHeuristic::dist2NearestTriangle(const grasp::RBCoord &pose,  Belief::Hypothesis::Seq::const_iterator p, const bool normal) const {
//	//Point::Seq samplePoints = p->second.points;
//	//
//	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
//	//cloud->width = samplePoints.size();
//	//cloud->height = 1;
//	//cloud->points.resize (cloud->width * cloud->height);
//	//int j = 0;
//	//for (Point::Seq::const_iterator point = samplePoints.begin(); point != samplePoints.end(); ++point/*, ++j*/) {
//	//	cloud->points[j].x = (float)point->frame.p.x;
//	//	cloud->points[j].y = (float)point->frame.p.y;
//	//	cloud->points[j++].z = (float)point->frame.p.z;
//	//}
//	//for (size_t i = 0; i < cloud->points.size (); ++i)
//	//	std::printf("<%.4f, %.4f, %.4f>\n", cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
//
//	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
//	//kdtree.setInputCloud(cloud);
//	SecTmReal init = context.getTimer().elapsed();
//
//	pcl::PointXYZ searchPoint;
//	searchPoint.x = (float)pose.p.x;
//	searchPoint.y = (float)pose.p.y;
//	searchPoint.z = (float)pose.p.z;
//	
//	Real result = REAL_MAX;
//	Vec3 median;
//	median.setZero();
//	const size_t size = (*p)->pTriangles->polygons.size() < ftDrivenDesc.ftModelDesc.points ? (*p)->pTriangles->polygons.size() : ftDrivenDesc.ftModelDesc.points;
//	for (size_t i = 0; i < size; ++i) {
//		const pcl::Vertices vertex = size < (*p)->pTriangles->polygons.size() ? (*p)->pTriangles->polygons[size_t(rand.next())%(*p)->pTriangles->polygons.size()] : (*p)->pTriangles->polygons[i];
//		for (std::vector<golem::U32>::const_iterator j = vertex.vertices.begin(); j != vertex.vertices.end(); ++j) {
//			//const Real dist = pose.p.distance(p->second->points[*j].frame.p);
//			//if (result > dist) {
//			//	result = dist;
//			//	median.set(p->second->points[*j].frame.p);
//			//}
//			const Real dist = pose.p.distance(Cloud::getPoint((*p)->points[*j]));
//			if (result > dist) {
//				result = dist;
//				median.set(Cloud::getPoint((*p)->points[*j]));
//			}
//		}
//	}
//	
//	if (normal) {
//		Vec3 v;
//		Mat34 jointFrame(Mat33(pose.q), pose.p);
//		Mat34 jointFrameInv;
//		jointFrameInv.setInverse(jointFrame);
//		jointFrameInv.multiply(v, median);
//		v.normalise();
////		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
////		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
//		if (v.z < 0 || v.z < ftDrivenDesc.ftModelDesc.coneTheta1)
//			result = ftDrivenDesc.ftModelDesc.distMax + 1;
//		//else {
//		//	Mat33 rot;
//		//	Real roll, pitch, yaw;
//		//	Quat quat(pose.q);
//		//	quat.toMat33(rot);
//		//	rot.toEuler(roll, pitch, yaw);
//		//	std::printf("nearest(): pose [%.4f,%.4f,%.4f]<%.4f,%.4f,%.4f>; median <%.4f,%.4f,%.4f>, thx %.4f, thy %.4f\n",
//		//		roll, pitch, yaw, pose.p.x, pose.p.y, pose.p.z, median.x, median.y, median.z, thx, thy);
//		//}
//	}
//	std::printf("dist2NearestTriangle: preforming time %.7f\n", context.getTimer().elapsed() - init);
//	return result;
//}
//
//
//Real FTDrivenHeuristic::dist2NearestPoint(const grasp::RBCoord &pose,  Belief::Hypothesis::Seq::const_iterator p, const bool normal) const {
//	SecTmReal init = context.getTimer().elapsed();
//	Real result = REAL_MAX;
//	Vec3 median;
//	median.setZero();
//	const size_t size = modelPoints.size() < ftDrivenDesc.ftModelDesc.points ? modelPoints.size() : ftDrivenDesc.ftModelDesc.points;
//	for (size_t i = 0; i < size; ++i) {
//		//const Point& point = size < p->second->points.size() ? p->second->points[size_t(rand.next())%p->second->points.size()] : p->second->points[i];
//		//const Real dist = pose.p.distance(point.frame.p);
//		//if (result > dist) {
//		//	result = dist;
//		//	median.set(point.frame.p);
//		//}
//		const Vec3 point = Cloud::getPoint(size < (*p)->points.size() ? (*p)->points[size_t(rand.next())%(*p)->points.size()] : (*p)->points[i]);
//		const Real dist = pose.p.distance(point);
//		if (result > dist) {
//			result = dist;
//			median.set(point);
//		}
//	}
//	
//	if (normal) {
//		Vec3 v;
//		Mat34 jointFrame(Mat33(pose.q), pose.p);
//		Mat34 jointFrameInv;
//		jointFrameInv.setInverse(jointFrame);
//		jointFrameInv.multiply(v, median);
//		v.normalise();
////		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
////		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
//		if (v.z < 0 || v.z < ftDrivenDesc.ftModelDesc.coneTheta1)
//			result = ftDrivenDesc.ftModelDesc.distMax + 1;
//		//else {
//		//	Mat33 rot;
//		//	Real roll, pitch, yaw;
//		//	Quat quat(pose.q);
//		//	quat.toMat33(rot);
//		//	rot.toEuler(roll, pitch, yaw);
//		//	std::printf("nearest(): pose [%.4f,%.4f,%.4f]<%.4f,%.4f,%.4f>; median <%.4f,%.4f,%.4f>, thx %.4f, thy %.4f\n",
//		//		roll, pitch, yaw, pose.p.x, pose.p.y, pose.p.z, median.x, median.y, median.z, thx, thy);
//		//}
//	}
//	std::printf("NearestPoint (t %.7f) %.7f\n", context.getTimer().elapsed() - init, result);
//	return result;
//}
//
//Real FTDrivenHeuristic::distance(const RBCoord &a, const RBCoord &b, bool enableAng) const {
//	Real dist = REAL_ZERO;
//	
//	dist += a.p.distance(b.p);
//
//	if (enableAng)
//		dist += golem::REAL_ONE - golem::Math::abs(a.q.dot(b.q));
//
//	return dist;
//}
//
//Real FTDrivenHeuristic::kernel(Real x, Real lambda) const {
//	return /*lambda**/golem::Math::exp(-lambda*x);
//}
//
//Real FTDrivenHeuristic::density(const RBCoord &x, const RBCoord &mean) const {
//	Real dist = x.p.distance(mean.p);
//	dist = (dist > ftDrivenDesc.ftModelDesc.distMax) ? ftDrivenDesc.ftModelDesc.distMax : dist;
//	if (ftDrivenDesc.ftModelDesc.enabledLikelihood)
//		return kernel(dist, ftDrivenDesc.ftModelDesc.lambda); // esponential up to norm factor
//
//	return dist; //linear distance
//}
//
//Real FTDrivenHeuristic::density(const Real dist) const {
//	if (ftDrivenDesc.ftModelDesc.enabledLikelihood) 
////		return kernel(ftDrivenDesc.ftModelDesc.lambda*dist);
//		return (dist > ftDrivenDesc.ftModelDesc.distMax) ? REAL_ZERO : (dist < REAL_EPS) ? kernel(REAL_EPS, ftDrivenDesc.ftModelDesc.lambda) : kernel(dist, ftDrivenDesc.ftModelDesc.lambda); // esponential up to norm factor
//
//	return dist; //linear distance
//}
//
//golem::Real FTDrivenHeuristic::evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const Real &force, const golem::Mat34 &trn, bool &intersect) const {
//	Real distMin = ftDrivenDesc.ftModelDesc.distMax + REAL_ONE;	
//	const Real norm = (ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
//	Mat34 actionFrame(sample.q, sample.p), queryFrame(Mat34(sample.q, sample.p) * trn), queryFrameInv, fromQueryToJoint;
//	intersect = false;
//
//	Vec3 boundFrame, surfaceFrame;
//	if (pose.p.distance(queryFrame.p) < distMin) {
//		const size_t size = modelPoints.size() < ftDrivenDesc.ftModelDesc.points ? modelPoints.size() : ftDrivenDesc.ftModelDesc.points;
//		for (size_t i = 0; i < size; ++i) {
//			// CHECK: Do you assume that a point has a full frame with **orientation**? Where does the orientation come from?
//
//			//const Point& point = size < modelPoints.size() ? modelPoints[size_t(rand.next())%
//			//	modelPoints.size()] : modelPoints[i];
//			//Mat34 pointFrame;
//			//pointFrame.multiply(actionFrame, point.frame);
//			const Vec3 point = Cloud::getPoint(size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]);
//			Mat34 pointFrame = actionFrame;
//			pointFrame.p += point; // only position is updated
//			const Real dist = [&] () -> Real {
//				Real min = REAL_MAX;
//				for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b) {
//					const Real d = (*b)->getSurfaceDistance(pointFrame.p);
//					if (d < min) {
//						min = d;
//						boundFrame = (*b)->getPose().p;
//						surfaceFrame = pointFrame.p;
//					}
//				}
//				return min;
//			} ();
//			if (dist <= REAL_ZERO) 
//				intersect = true;
//			if (dist > -REAL_EPS && dist < distMin)
//				distMin = dist;
//		}
//		
//	}
//	//context.write(" -> dist(to shape)=%5.7f, bound <%f %f %f>, point <%f %f %f>, intersection %s, norm=%5.7f, density=%5.7f, prob of touching=%5.7f\n", distMin, 
//	//	boundFrame.x, boundFrame.y, boundFrame.z, surfaceFrame.x, surfaceFrame.y, surfaceFrame.z,
//	//	intersect ? "Y" : "N", norm, density(distMin), norm*density(distMin));
//	//return norm*density(distMin); // if uncomment here there is no notion of direction of contacts
//
//	// compute the direction of contact (from torques)
//	Vec3 v;
//	Mat34 jointFrame(Mat33(pose.q), pose.p);
//	Mat34 jointFrameInv;
//	jointFrameInv.setInverse(jointFrame);
//	jointFrameInv.multiply(v, queryFrame.p);
//	v.normalise();
////		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
////		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
//	if ((v.z < 0 && force > 0) || (v.z > 0 && force < 0)) {
//		//context.write("evaluation pose <%f %f %f> sample <%f %f %f> v.z=%f force=%f\n", 
//		//	pose.p.x, pose.p.y, pose.p.z, queryFrame.p.x, queryFrame.p.y, queryFrame.p.z, v.z, force);
//		return REAL_ZERO;
//	}
//	// return the likelihood of observing such a contact for the current joint
//	return norm*density(distMin);
//
//	
////	queryFrameInv.setInverse(queryFrame);
////	fromQueryToJoint.multiply(queryFrameInv, Mat34(pose.q, pose.p));
////	//context.write("pose <%5.7f %5.7f %5.7f>\n sample <%5.7f %5.7f %5.7f>\n",
////	//	pose.p.x, pose.p.y, pose.p.z, sample.p.x, sample.p.y, sample.p.z);
////	//context.write("query-to-joint frame <%5.7f %5.7f %5.7f>\n",
////	//	fromQueryToJoint.p.x, fromQueryToJoint.p.y, fromQueryToJoint.p.z);
////
////	//context.write(" -> magnitude=%5.7f\n", fromQueryToJoint.p.magnitude());
////	//if (fromQueryToJoint.p.magnitude() > 2*ftDrivenDesc.ftModelDesc.distMax)
////	//	return golem::REAL_ZERO; //norm*density(distMin);
////
////	fromQueryToJoint.multiply(trn, fromQueryToJoint);
////	//context.write("new pose <%5.7f %5.7f %5.7f>,  model <%5.7f %5.7f %5.7f>, dist=%5.7f\n",
////	//	fromQueryToJoint.p.x, fromQueryToJoint.p.y, fromQueryToJoint.p.z, trn.p.x, trn.p.y, trn.p.z, trn.p.distance(fromQueryToJoint.p));
////	if (fromQueryToJoint.p.magnitude() < distMin) {
////		const size_t size = modelPoints.size() < ftDrivenDesc.ftModelDesc.points ? modelPoints.size() : ftDrivenDesc.ftModelDesc.points;
////		for (size_t i = 0; i < size; ++i) {
////			const Point& point = size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i];
////			const Real dist = fromQueryToJoint.p.distance(point.frame.p);
////			if (dist < distMin)
////				distMin = dist;
////		}
////	}
//////	context.write(" -> dist(to shape)=%5.7f, norm=%5.7f, density=%5.7f, prob of touching=%5.7f\n", distMin, norm, density(distMin), norm*density(distMin));
////	return /*norm**/density(distMin);
//}
//
//golem::Real FTDrivenHeuristic::evaluate(const grasp::Manipulator *manipulator, const golem::Waypoint &w, const grasp::RBPose::Sample &sample, const std::vector<FTGuard> &triggeredGuards, const grasp::RealSeq &force, const golem::Mat34 &trn) const {
//	Real weight = golem::REAL_ONE;
//	bool intersect = false; 
//	// retrieve wrist's workspace pose and fingers configuration
//	//golem::Mat34 poses[grasp::Manipulator::JOINTS];
//	//manipulator->getPoses(manipulator->getPose(w.cpos), poses);
//
//	grasp::RealSeq forces(force);
//	for (Chainspace::Index i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
//		// check if any of the joint in the current chain (finger) has been triggered
//		bool triggered = false;
////		context.write("Chain %d\n", i);
//		for (Configspace::Index j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) { 
//			const size_t idx = j - handInfo.getJoints().begin();
//			triggered = triggered || [&] () -> bool {
//				for (std::vector<FTGuard>::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
//					if (j == i->jointIdx) {
//						forces[idx] = i->type == FTGuard_ABS ? 0 : i->type == FTGuard_LESSTHAN ? -1 : 1;
//						return true;
//					}
//				return false;
//			} ();
//			//const golem::U32 joint = manipulator->getArmJoints() + idx;
//			//golem::Bounds::Seq bounds;
//			//manipulator->getJointBounds(joint, poses[joint], bounds);
//			golem::Bounds::Seq bounds;
//			manipulator->getJointBounds(golem::U32(j - manipulator->getController()->getStateInfo().getJoints().begin()), w.wposex[j], bounds);
//			//for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b)
//			//		context.write("finger bound frame <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
//			const Real eval = evaluate(bounds, grasp::RBCoord(w.wposex[j]), sample, forces[idx], trn, intersect);
//			weight *= (triggered ? ftDrivenDesc.contactFac*eval : intersect ? ftDrivenDesc.nonContactFac*(REAL_ONE - eval) : 1);
//			//context.write("evaluate %f prob %f partial weight %f, intersect %s, triggered %s\n", eval, 
//			//	/*jointFac[j - handInfo.getJoints().begin()]**/(triggered ? ftDrivenDesc.contactFac*eval : ftDrivenDesc.nonContactFac*(REAL_ONE - eval)), weight,
//			//	intersect ? "Y" : "N", triggered ? "Y" : "N");
//		}
////		const Real eval = evaluate(grasp::RBCoord(w.wpos[i]), sample, 0, trn);
////		weight *= triggered ? eval : REAL_ONE;
////		weight *= triggered ? ftDrivenDesc.contactFac*eval : ftDrivenDesc.nonContactFac*(REAL_ONE - eval);
////		context.write(" -> (triggered %d) weight=%f partial_weight=%f\n", triggered, eval, weight);
//
//	}
//
//	//for (Configspace::Index j = handInfo.getJoints().begin(); j != handInfo.getJoints().end(); ++j) {
//	//	context.write("Joint %d\n", j);
//	//	const size_t k = j - handInfo.getJoints().begin();
//	//	bool triggered = [&] () -> bool {
//	//		for (std::vector<Configspace::Index>::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
//	//			if (j == *i)
//	//				return true;
//	//		return false;
//	//	} ();
//	//	const Real eval = evaluate(grasp::RBCoord(w.wposex[j]), sample, force[k], trn);
//	//	const Real tmp = triggered ? eval : 1 - eval;
//	//	weight *= jointFac[j - handInfo.getJoints().begin()]*tmp;
//	//	context.write(" -> (triggered %d) weight=%f partial_weight=%f\n", triggered, jointFac[j - handInfo.getJoints().begin()]*tmp, weight);
//	//}
////	context.write("final weight=%f\n", weight);
//
//	return weight;
//}
//
//golem::Real FTDrivenHeuristic::evaluate(const grasp::Manipulator *manipulator, const golem::Waypoint &w, const grasp::RBPose::Sample &sample, const std::vector<golem::Configspace::Index> &triggeredGuards, const grasp::RealSeq &force, const golem::Mat34 &trn) const {
//	Real weight = golem::REAL_ONE;
//	bool intersect = false; 
//	// retrieve wrist's workspace pose and fingers configuration
//	//golem::Mat34 poses[grasp::Manipulator::JOINTS];
//	//manipulator->getPoses(manipulator->getPose(w.cpos), poses);
//
//	for (Chainspace::Index i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
//		// check if any of the joint in the current chain (finger) has been triggered
//		bool triggered = false;
////		context.write("Chain %d\n", i);
//		for (Configspace::Index j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) { 
//			const size_t idx = j - handInfo.getJoints().begin();
//			triggered = triggered || [&] () -> bool {
//				for (std::vector<Configspace::Index>::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
//					if (j == *i)
//						return true;
//				return false;
//			} ();
//			//const golem::U32 joint = manipulator->getArmJoints() + idx;
//			//golem::Bounds::Seq bounds;
//			//manipulator->getJointBounds(joint, poses[joint], bounds);
//			golem::Bounds::Seq bounds;
//			manipulator->getJointBounds(golem::U32(j - manipulator->getController()->getStateInfo().getJoints().begin()), w.wposex[j], bounds);
//			//for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b)
//			//		context.write("finger bound frame <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
//			const Real eval = evaluate(bounds, grasp::RBCoord(w.wposex[j]), sample, force[idx], trn, intersect);
//			weight *= (triggered ? ftDrivenDesc.contactFac*eval : intersect ? ftDrivenDesc.nonContactFac*(REAL_ONE - eval) : 1);
//			//context.write("evaluate %f prob %f partial weight %f, intersect %s, triggered %s\n", eval, 
//			//	/*jointFac[j - handInfo.getJoints().begin()]**/(triggered ? ftDrivenDesc.contactFac*eval : ftDrivenDesc.nonContactFac*(REAL_ONE - eval)), weight,
//			//	intersect ? "Y" : "N", triggered ? "Y" : "N");
//		}
////		const Real eval = evaluate(grasp::RBCoord(w.wpos[i]), sample, 0, trn);
////		weight *= triggered ? eval : REAL_ONE;
////		weight *= triggered ? ftDrivenDesc.contactFac*eval : ftDrivenDesc.nonContactFac*(REAL_ONE - eval);
////		context.write(" -> (triggered %d) weight=%f partial_weight=%f\n", triggered, eval, weight);
//
//	}
//
//	//for (Configspace::Index j = handInfo.getJoints().begin(); j != handInfo.getJoints().end(); ++j) {
//	//	context.write("Joint %d\n", j);
//	//	const size_t k = j - handInfo.getJoints().begin();
//	//	bool triggered = [&] () -> bool {
//	//		for (std::vector<Configspace::Index>::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
//	//			if (j == *i)
//	//				return true;
//	//		return false;
//	//	} ();
//	//	const Real eval = evaluate(grasp::RBCoord(w.wposex[j]), sample, force[k], trn);
//	//	const Real tmp = triggered ? eval : 1 - eval;
//	//	weight *= jointFac[j - handInfo.getJoints().begin()]*tmp;
//	//	context.write(" -> (triggered %d) weight=%f partial_weight=%f\n", triggered, jointFac[j - handInfo.getJoints().begin()]*tmp, weight);
//	//}
////	context.write("final weight=%f\n", weight);
//
//	return weight;
//}

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

//bool FTDrivenHeuristic::collides(const Waypoint &w) const {
//#ifdef _HEURISTIC_PERFMON
//	++waypointCollisionCounter;
//#endif
//
//	//if (!desc.collisionDesc.enabled)
//	//	return false;
//
//	// find data pointer for the current thread
//	ThreadData* data = getThreadData();
//
//	// all chains
//	//for (Chainspace::Index i = stateInfo.getChains().begin(); i < stateInfo.getChains().end(); ++i)
//	if (desc.collisionDesc.enabled) {
//		for (Chainspace::Index i = stateInfo.getChains().end() - 1; i >= stateInfo.getChains().begin(); --i)
//		{
//			// all joints in a chain
//			//for (Configspace::Index j = stateInfo.getJoints(i).begin(); j < stateInfo.getJoints(i).end(); ++j)
//			for (Configspace::Index j = stateInfo.getJoints(i).end() - 1; j >= stateInfo.getJoints(i).begin(); --j)
//			{
//				const JointDesc* jdesc = getJointDesc()[j];
//				Bounds::Seq& jointBounds = data->jointBounds[j];
//
//				if (jointBounds.empty() || !jdesc->collisionBounds)
//					continue;
//
//				// reset to the current joint pose
//				setPose(data->jointBoundsPoses[j], w.wposex[j], jointBounds);
//
//				// check for collision between the joint bounds and the estimated pose of the object to grasp
//				if (pBelief.get())
//					if (pBelief->intersect(jointBounds, w.wposex[j]))
//						return true;
//
//				// check for collisions between the current joint and the environment bounds, test bounds groups
//				if (!collisionBounds.empty() && intersect(jointBounds, collisionBounds, true)) {
//					//				context.debug("FTDrivenHeuristic::collides(): collisions between the current joint and the environment bounds. collision bounds %s empty\n", collisionBounds.empty()?"are":"are not");
//					return true;
//				}
//
//				// check for collisions between the current joint and the collision joints, do not test bounds groups
//				const U32Seq& collisionJoints = jdesc->collisionJoints;
//				for (U32Seq::const_iterator k = collisionJoints.begin(); k != collisionJoints.end(); ++k) {
//					const Configspace::Index collisionIndex(*k);
//
//					golem::Bounds::Seq &collisionJointBounds = data->jointBounds[collisionIndex];
//					if (collisionJointBounds.empty())
//						continue;
//
//					setPose(data->jointBoundsPoses[collisionIndex], w.wposex[collisionIndex], collisionJointBounds);
//					if (intersect(jointBounds, collisionJointBounds, false)) {
//						//					context.debug("FTDrivenHeuristic::collides(): collisions between the current joint and the collision joints\n");
//						return true;
//					}
//				}
//
//
//				//if (pointCloudCollision && pBelief.get() && i != stateInfo.getChains().begin())
//				//	if (pBelief->intersect(jointBounds, w.wposex[j]))
//				//		return true;
//
//				// TODO joint bounds - chain bounds collisions
//			}
//
//			// TODO bounds - chain bounds collisions
//		}
//	}
//	//// check for collisions with the object to grasp. only the hand
//	if (pointCloudCollision && pBelief.get()) {
//		if (manipulator.get()) {
//			if (Math::abs(collision->evaluate(waypoint, (*pBelief->getHypotheses().begin())->getCloud(), golem::Rand(rand), manipulator->getPose(w.cpos), testCollision)) > REAL_EPS) {
//				//Controller::State state = controller.createState();
//				//state.cpos = w.cpos;
//				//std::stringstream str;
//				//for (golem::Configspace::Index i = state.getInfo().getJoints().begin(); i < state.getInfo().getJoints().end(); ++i)
//				//	str << " c" << (*i - *state.getInfo().getJoints().begin() + 1) << "=\"" << state.cpos[i] << "\"";
//				//context.write("<pose dim=\"%d\"%s/>\n", state.getInfo().getJoints().size(), str.str().c_str());
//				return true;
//			}
//		}
//	}
////		SecTmReal init = context.getTimer().elapsed();
////		//CriticalSection cs;
////		//Chainspace::Index handIndex = handInfo.getChains().begin();
//////		grasp::Manipulator::Ptr manipulator(new grasp::Manipulator(&controller));
////		//bool collision = false;
//////		ParallelsTask((golem::Parallels*)context.getParallels(), [&] (ParallelsTask*) {
////			// evaluated all the joints of the hand
////		for (Chainspace::Index i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
////				//{
////				//	CriticalSectionWrapper csw(cs);
////				//	if (handIndex == handInfo.getChains().end())
////				//		break;
////				//	i = handIndex++;
////				//}
////				// bounds of the entire finger
//////		context.write("chain\n");
////				//golem::Bounds::Seq bounds;
////				for (Configspace::Index j = handInfo.getJoints(i).end()-1; j >= handInfo.getJoints(i).begin(); --j) { 
//////		context.write("joint\n");
////					//manipulator->getJointBounds(golem::U32(j - manipulator->getController()->getStateInfo().getJoints().begin()), w.wposex[j], bounds);
////					const size_t k = j - armInfo.getJoints().begin(); // from the first joint
////					Bounds::Seq boundsSeq;
////					golem::Bounds::Desc::SeqPtr boundsDescSeq = controller.getJoints()[armInfo.getJoints().begin() + k]->getBoundsDescSeq();
////					for (golem::Bounds::Desc::Seq::const_iterator b = boundsDescSeq->begin(); b != boundsDescSeq->end(); ++b) {
////						boundsSeq.push_back(b->get()->create());
////						boundsSeq.back()->multiplyPose(w.wposex[j], boundsSeq.back()->getPose());
////					}
////					if (pBelief->intersect(boundsSeq, w.wposex[j])) {
//////						context.debug("FTDrivenHeuristic::collides(): collision between chain %d, joint %d and point cloud. (time %.5f)\n", i - handInfo.getChains().begin(), k, context.getTimer().elapsed() - init);
////						return true;
////					}
////					//{
////					//	CriticalSectionWrapper csw(cs);
////					//	if (!collision)
////					//		collision = pBelief->intersect(bounds, w.wposex[j]);
////					//	else
////					//		break; // collision is true. no need to keep looping
////					//}
////				}
////			}
////		}); // end parallels task
////		context.write("done(0). (time %.5f)\n", context.getTimer().elapsed() - init);
////		return collision;
////	}
//
//	return false;
//
//	//// check if the waypoint collides with objects in the scene
//	//if (Heuristic::collides(w))
//	//	return true;
//
//	//// check if the trajectory collides with the point cloud of the ML sample
//	//if (pBelief->getHypotheses().empty()) 
//	//	return false;
//
//	//Belief::Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
//	//for (Chainspace::Index i = handInfo.getChains().begin(); i < handInfo.getChains().end(); ++i) {
//	//	const RBCoord c(w.wpos[i]);
//	//	if (c.p.distance((*maxLhdPose)->sample.p) < 0.045) //if (distance(maxLhdPose->second->sample, c) < 0.05)
//	//		return true;
//	//}
//
//	//return false;
//}
//
////bool FTDrivenHeuristic::collides2(const golem::Waypoint &w1, const golem::Waypoint &w2) const {
////	if (w1.index == Node::IDX_GOAL || w1.index == Node::IDX_GOAL)
////		return false;
////
////	const Real dist = getDist(w1, w2);
////	U32 steps = (U32)Math::round(dist / (desc.collisionDesc.pathDistDelta*4));
////	for (int i = 1; i < steps; ++i) {
////		Waypoint w;
////		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
////			w.cpos[j] = w2.cpos[j] - (w2.cpos[j] - w1.cpos[j])*Real(i) / Real(steps);
////
////		// skip reference pose computation
////		w.setup(controller, false, true);
////		if (collides(w))
////			return true;
////	}
////	return false;
////}
//
//bool FTDrivenHeuristic::collides(const Waypoint &w0, const Waypoint &w1) const {
//#ifdef _HEURISTIC_PERFMON
//	++pathCollisionCounter;
//#endif
//
//	//if (!desc.collisionDesc.enabled)
//	//	return false;
//
//	// check the end waypoint first
//	if (collides(w1))
//		return true;
//
//	Waypoint w;
//
//	const Real dist = getDist(w0, w1);
//	
//	const U32 steps = (U32)Math::round(dist / desc.collisionDesc.pathDistDelta);
//
//	// the first waypoint does not collide by assumption
//	// TODO binary collision detection
//	for (U32 i = 1; i < steps; ++i) {
//		// lineary interpolate coordinates
//		for (Configspace::Index j = stateInfo.getJoints().begin(); j < stateInfo.getJoints().end(); ++j)
//			w.cpos[j] = w1.cpos[j] - (w1.cpos[j] - w0.cpos[j])*Real(i) / Real(steps);
//
//		// skip reference pose computation
//		w.setup(controller, false, true);
//
//		if (collides(w))
//			return true;
//	}
//
//	return false;
//}

bool FTDrivenHeuristic::collides(const golem::Waypoint &w, ThreadData* data) const {
#ifdef _HEURISTIC_PERFMON
	++perfCollisionWaypoint;
#endif
	// check for collisions with the object to grasp. only the hand
	grasp::Manipulator::Config config(w.cpos);
	if (pointCloudCollision && !pBelief->getHypotheses().empty()) {
		Hypothesis::Seq::const_iterator maxLhdPose = pBelief->getHypotheses().begin();
		if (intersect(manipulator->getBounds(config.config, config.frame.toMat34()), (*maxLhdPose)->bounds(), false)) {
#ifdef _HEURISTIC_PERFMON
			++perfCollisionPointCloud;
#endif
			//if ((*maxLhdPose)->check(waypoint, manipulator->getPose(w.cpos)))
			if ((*maxLhdPose)->check(ftDrivenDesc.checkDesc, rand, config.config))
				return true;
		}
//		}
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
