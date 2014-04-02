/** @file Heuristic.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/Phys/Data.h>
#include <Spam/Spam/Belief.h>
#include <Grasp/Grasp/Grasp.h>

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

void spam::XMLData(Belief::Desc& val, golem::XMLContext* context, bool create) {
	context = context->getContextFirst("tactile_model");
	//golem::XMLData("kernels", val.tactile.kernels, context, create);
	//golem::XMLData("test", val.tactile.test, context, create);
	//grasp::XMLData(val.tactile.stddev, context->getContextFirst("test_stddev"), create);
	//golem::XMLData(&val.tactile.covariance[0], &val.tactile.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);
	golem::XMLData("num_hypotheses", val.numHypotheses, context, create);
	golem::XMLData("max_surface_points", val.maxSurfacePoints, context, create);
	golem::XMLData(&val.covariance[0], &val.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);
}

//------------------------------------------------------------------------------

Mat34 Belief::RigidBodyTransformation::transform(Mat34 &p) {
	Mat34 poseInverse;
	poseInverse.setInverse(pose);
	pose.multiply(poseInverse, p);
	return pose;
}

//------------------------------------------------------------------------------

Real Belief::Hypothesis::dist2NearestKPoints(const grasp::RBCoord &pose,  const golem::Real &maxDist, const size_t clusters, const size_t &nIndeces, const bool normal) const {
	if (pose.p.distance((sample.toMat34() * modelFrame).p) > (REAL_ONE + REAL_HALF)*maxDist)
		return maxDist + 1;;

	pcl::PointXYZ searchPoint;
	searchPoint.x = (float)pose.p.x;
	searchPoint.y = (float)pose.p.y;
	searchPoint.z = (float)pose.p.z;

	std::vector<int> indeces;
	std::vector<float> distances;
	pcl::KdTree<pcl::PointXYZ>::PointCloudConstPtr cloud = pTree->getInputCloud();
	Real result = REAL_ZERO;
	Vec3 median;
	median.setZero();
	golem::Rand rand;
	if (pTree->nearestKSearch(searchPoint, clusters, indeces, distances) > 0) {
		const size_t size = indeces.size() < nIndeces ? indeces.size() : nIndeces;
		for (size_t i = 0; i < size; ++i) {
			idxdiff_t idx = size < indeces.size() ? indeces[size_t(rand.next())%indeces.size()] : indeces[i];
			//Point point;
			//point.frame.setId();
			//point.frame.p.set(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
			//result += pose.p.distance(point.frame.p);
			//median += Vec3(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
			result += pose.p.distance(Vec3(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z));
			median += Vec3(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
		}
		result /= size;
		median /= size;
	}
	
	if (normal) {
		Vec3 v;
		Mat34 jointFrame(Mat33(pose.q), pose.p);
		Mat34 jointFrameInv;
		jointFrameInv.setInverse(jointFrame);
		jointFrameInv.multiply(v, median);
		v.normalise();
//		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
//		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
		if (v.z < 0 || v.z < golem::Real(.5))
			result = maxDist + 1;
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
	//if (result < ftDrivenDesc.ftModelDesc.distMax + 1) {
	//	std::printf("NearestKNeigh (t %.7f) %.7f\n", context.getTimer().elapsed() - init, result);
	//	dist2NearestPoint(pose, p, normal);
	//}
	return result;
}

bool Belief::Hypothesis::build() {
	pTree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ, flann::L2_Simple<float>>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
	cloud->width = points.size();
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);
	int j = 0;
	for (grasp::Cloud::PointSeq::const_iterator point = points.begin(); point != points.end(); ++point) {
		cloud->points[j].x = (float)point->x;
		cloud->points[j].y = (float)point->y;
		cloud->points[j++].z = (float)point->z;
	}
	pTree->setInputCloud(cloud);

	return true;
}

bool Belief::Hypothesis::buildMesh() {
	pTriangles.reset(new pcl::PolygonMesh);
	// build point cloud form set of points
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); 
	cloud->width = points.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	int j = 0;
	for (grasp::Cloud::PointSeq::const_iterator point = points.begin(); point != points.end(); ++point) {
		cloud->points[j].x = (float)point->x;
		cloud->points[j].y = (float)point->y;
		cloud->points[j++].z = (float)point->z;
	}

	  // Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(1000);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*pTriangles.get());
	std::printf("HypSample::buildMesh: Triangle vertices %d\n", pTriangles->polygons.size());
	//for (std::vector<pcl::Vertices>::iterator i = pTriangles->polygons.begin(); i != pTriangles->polygons.end(); ++i)
	//	std::printf("Vertex n.%d size %d\n", i, i->vertices.size());
	
	return true;
}

//------------------------------------------------------------------------------

Belief::Belief(golem::Context& context) : grasp::RBPose(context) {
	pRBPose.reset((grasp::RBPose*)this);
}

bool Belief::create(const Desc& desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "spam::RBPose::create(): Invalid description");
	
	grasp::RBPose::create((grasp::RBPose::Desc&)desc);
	
	hypotheses.clear();
	hypotheses.reserve(desc.numHypotheses);

	myDesc = desc;
//	mfsePoses.resize(desc.tactile.kernels);

	return true;
}

grasp::RBPose::Sample Belief::createHypotheses(const grasp::Cloud::PointSeq& model, const golem::Mat34 &transform) {
	U32 idx = 0;
	// copy model and model frame. Note: it is done only the very first time.
	if (modelPoints.empty()) {
		modelPoints.clear();
		modelPoints.reserve(model.size());
		for (grasp::Cloud::PointSeq::const_iterator i = model.begin(); i != model.end(); ++i) 
			modelPoints.push_back(*i);
		modelFrame = transform;
	}
	// reset container for hypotheses
	hypotheses.clear();
	hypotheses.reserve(myDesc.numHypotheses);

	const grasp::RBPose::Sample actionFrame = maximum();
//	context.write("Heuristic:setBeliefState(model size = %d, max points = %d): samples: cont_fac = %f\n", model.size(), ftDrivenDesc.maxSurfacePoints, ftDrivenDesc.contactFac);
	for (size_t i = 0; i < myDesc.numHypotheses; ++i) {
		// sample hypothesis. NOTE: The first element is the max scoring pose
		grasp::RBCoord sampleFrame = (i == 0) ? actionFrame : sample();
		// transform the sample in the reference coordinate (default: robot's coordinate frame)
//		sampleFrame.multiply(sampleFrame, grasp::RBCoord(transform));
		// container for the point cloud of this sample (default: the same points of the model)
		grasp::Cloud::PointSeq sampleCloud;
		
		// copy model point cloud
		// optimisation: limit the number of points to save performance in the kd-tree
		const size_t size = model.size() < myDesc.maxSurfacePoints ? model.size() : myDesc.maxSurfacePoints;
		for (size_t i = 0; i < size; ++i) {
			grasp::Cloud::Point p = size < model.size() ? model[size_t(rand.next())%model.size()] : model[i]; // make a copy here
			grasp::Cloud::setPoint(sampleFrame.toMat34() * transform * grasp::Cloud::getPoint(p), p);
			sampleCloud.push_back(p);
		}
//		hypotheses.insert(Hypothesis::Map::value_type(idx, Hypothesis::Ptr(new Hypothesis(idx, grasp::RBPose::Sample(sampleFrame), sampleCloud))));
		hypotheses.push_back(Hypothesis::Ptr(new Hypothesis(idx, transform, grasp::RBPose::Sample(sampleFrame), sampleCloud)));
		context.write(" - sample n.%d <%.4f %.4f %.4f> <%.4f %.4f %.4f %.4f>\n", idx, sampleFrame.p.x, sampleFrame.p.y, sampleFrame.p.z, sampleFrame.q.w, sampleFrame.q.x, sampleFrame.q.y, sampleFrame.q.z); 
		idx++;
	}
	
	// mean and covariance
	if (!sampleProperties.create<golem::Ref1, RBPose::Sample::Ref>(myDesc.covariance, getSamples()))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
		
	context.write("spam::Belief::createQuery(TEST): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
	
	// compute determinant for the sampled hypotheses
	covarianceDet = REAL_ONE;
	for (golem::U32 j = 0; j < grasp::RBCoord::N; ++j)
			covarianceDet *= sampleProperties.covariance[j];

	return actionFrame;
}

grasp::RBPose::Sample::Seq Belief::getSamples() {
	// return the rbpose::sample associated with each hypothesis.
	// NOTE: useful for creating mean and covariance in Belief::setHypotheses
	grasp::RBPose::Sample::Seq samples;
	for (Hypothesis::Seq::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h)
		samples.push_back((*h)->getSample());
	return samples;
}

//------------------------------------------------------------------------------

void Belief::createQuery(const grasp::Cloud::PointSeq& points) {
	// call the super class
	grasp::RBPose::createQuery(points);
	
	// set initial belief
	initPoses.clear();
	for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
		initPoses.push_back(*i);
	initProperties = sampleProperties;
	
	context.write("spam::Belief::createQuery(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
}

//void Belief::createUpdate(const Mat34 &trn) {
//	 TODO
//	if (false) {
//	size_t i = 0;
//	for (grasp::RBPose::Sample::Seq::iterator s = mfsePoses.begin(); s != mfsePoses.end(); ++s, ++i) {
//		Mat34 sample = s->toMat34();
//		sample.multiply(trn, sample);
//		Quat q(sample.R);
//		*s = Sample(grasp::RBCoord(sample.p, q), REAL_ONE, i*REAL_ONE);
//	}
//	}
//}

void Belief::createResample() {
	size_t N = poses.size(), index = rand.nextUniform<size_t>(0, N);
	Real beta = golem::REAL_ZERO;
	grasp::RBPose::Sample::Seq newPoses;
	newPoses.reserve(N);
	for (size_t i = 0; i < N; ++i) {
		beta += rand.nextUniform<golem::Real>()*2*maxWeight();
//		context.debug("spam::RBPose::createResampling(): beta=%4.6f\n", beta);
		while (beta > poses[index].weight) {
			beta -= poses[index].weight;
			index = (index + 1) % N;
//			context.debug("beta=%4.6f\n, index=%d, weight=%4.6f\n", beta, index, mfsePoses[index].weight);
		}
		newPoses.push_back(poses.at(index));
	}

	// compute mean and covariance
	if (!pose.create<golem::Ref1, RBPose::Sample::Ref>(myDesc.covariance, newPoses))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createResample(): Unable to create mean and covariance for the high dimensional representation");

	context.write("spam::Belief::createResample(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3], pose.covariance[4], pose.covariance[5], pose.covariance[6]);
	
	// add noise to the resampled elements and overwrite poses
	poses.clear();
	poses.reserve(N);

	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < N; ++i) {
		//mfsePoses.push_back(Sample(newPoses[i], REAL_ONE, i*REAL_ONE));
		//continue;
		grasp::RBCoord c;
		rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &(newPoses[i])[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian
		poses.push_back(Sample(c, REAL_ONE, i*REAL_ONE));
	}
	
	// generate new set of hypotheses
	createHypotheses(modelPoints, modelFrame);
}

//------------------------------------------------------------------------------

Real Belief::kernel(Real x, Real lambda) const {
	return /*lambda**/golem::Math::exp(-lambda*x);
}

Real Belief::density(const Real dist) const {
	return (dist > myDesc.sensory.sensoryRange) ? REAL_ZERO : (dist < REAL_EPS) ? kernel(REAL_EPS, myDesc.lambda) : kernel(dist, myDesc.lambda); // esponential up to norm factor
}

void Belief::createUpdate(const grasp::Manipulator *manipulator, const golem::Waypoint &w, const grasp::FTGuard::Seq &triggeredGuards, const grasp::RealSeq &force) {
	Real weight = golem::REAL_ONE;
	bool intersect = false; 
	// retrieve wrist's workspace pose and fingers configuration
	//golem::Mat34 poses[grasp::Manipulator::JOINTS];
	//manipulator->getPoses(manipulator->getPose(w.cpos), poses);
	grasp::RealSeq forces(force);
	for (Chainspace::Index i = manipulator->getController()->getStateInfo().getChains().begin(); i != manipulator->getController()->getStateInfo().getChains().end(); ++i) {
		// check if any of the joint in the current chain (finger) has been triggered
		bool triggered = false;
//		context.write("Chain %d\n", i);
		for (Configspace::Index j = manipulator->getController()->getStateInfo().getJoints(i).begin(); j != manipulator->getController()->getStateInfo().getJoints(i).end(); ++j) { 
			const size_t idx = j - manipulator->getController()->getStateInfo().getJoints().begin();
			triggered = triggered || [&] () -> bool {
				for (grasp::FTGuard::Seq::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
					if (j == i->jointIdx) {
						forces[idx] = i->type == grasp::FTGUARD_ABS ? 0 : i->type == grasp::FTGUARD_LESSTHAN ? -1 : 1;
						return true;
					}
				return false;
			} ();
			//const golem::U32 joint = manipulator->getArmJoints() + idx;
			//golem::Bounds::Seq bounds;
			//manipulator->getJointBounds(joint, poses[joint], bounds);
			golem::Bounds::Seq bounds;
			manipulator->getJointBounds(golem::U32(j - manipulator->getController()->getStateInfo().getJoints().begin()), w.wposex[j], bounds);
			//for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b)
			//		context.write("finger bound frame <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
			golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
			for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
				const Real eval = evaluate(bounds, grasp::RBCoord(w.wposex[j]), *sampledPose, forces[idx], intersect);
				sampledPose->weight *= (triggered ? myDesc.sensory.contactFac*eval : intersect ? myDesc.sensory.noContactFac*(REAL_ONE - eval) : 1);
				golem::kahanSum(norm, c, sampledPose->weight);
			}
		}
	}

	// normalise weights
	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO, cdf = golem::REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose)
		golem::kahanSum(norm, c, sampledPose->weight);
	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
		sampledPose->weight /= norm;
		cdf += sampledPose->weight;
		sampledPose->cdf = cdf;
	}
}

golem::Real Belief::evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const Real &force, bool &intersect) {
	Real distMin = myDesc.sensory.sensoryRange + REAL_ONE;	
	const Real norm = REAL_ONE - density(myDesc.sensory.sensoryRange); //(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
	Mat34 actionFrame(sample.q, sample.p), queryFrame(Mat34(sample.q, sample.p) * modelFrame), queryFrameInv, fromQueryToJoint;

	Vec3 boundFrame, surfaceFrame;
	if (pose.p.distance(queryFrame.p) < distMin) {
		const size_t size = modelPoints.size() < myDesc.maxSurfacePoints ? modelPoints.size() : myDesc.maxSurfacePoints;
		for (size_t i = 0; i < size; ++i) {
			// CHECK: Do you assume that a point has a full frame with **orientation**? Where does the orientation come from?

			//const Point& point = size < modelPoints.size() ? modelPoints[size_t(rand.next())%
			//	modelPoints.size()] : modelPoints[i];
			//Mat34 pointFrame;
			//pointFrame.multiply(actionFrame, point.frame);
			const Vec3 point = grasp::Cloud::getPoint(size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]);
			Mat34 pointFrame = actionFrame;
			pointFrame.p += point; // only position is updated
			const Real dist = [&] () -> Real {
				Real min = REAL_MAX;
				for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b) {
					const Real d = (*b)->getSurfaceDistance(pointFrame.p);
					if (d < min) {
						min = d;
						boundFrame = (*b)->getPose().p;
						surfaceFrame = pointFrame.p;
					}
				}
				return min;
			} ();
			if (dist <= REAL_ZERO) 
				intersect = true;
			if (dist > -REAL_EPS && dist < distMin)
				distMin = dist;
		}
		
	}
	//context.write(" -> dist(to shape)=%5.7f, bound <%f %f %f>, point <%f %f %f>, intersection %s, norm=%5.7f, density=%5.7f, prob of touching=%5.7f\n", distMin, 
	//	boundFrame.x, boundFrame.y, boundFrame.z, surfaceFrame.x, surfaceFrame.y, surfaceFrame.z,
	//	intersect ? "Y" : "N", norm, density(distMin), norm*density(distMin));
	//return norm*density(distMin); // if uncomment here there is no notion of direction of contacts

	// compute the direction of contact (from torques)
	Vec3 v;
	Mat34 jointFrame(Mat33(pose.q), pose.p);
	Mat34 jointFrameInv;
	jointFrameInv.setInverse(jointFrame);
	jointFrameInv.multiply(v, queryFrame.p);
	v.normalise();
//		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
//		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
	if ((v.z < 0 && force > 0) || (v.z > 0 && force < 0)) {
		//context.write("evaluation pose <%f %f %f> sample <%f %f %f> v.z=%f force=%f\n", 
		//	pose.p.x, pose.p.y, pose.p.z, queryFrame.p.x, queryFrame.p.y, queryFrame.p.z, v.z, force);
		return REAL_ZERO;
	}
	// return the likelihood of observing such a contact for the current joint
	return norm*density(distMin);

	
//	queryFrameInv.setInverse(queryFrame);
//	fromQueryToJoint.multiply(queryFrameInv, Mat34(pose.q, pose.p));
//	//context.write("pose <%5.7f %5.7f %5.7f>\n sample <%5.7f %5.7f %5.7f>\n",
//	//	pose.p.x, pose.p.y, pose.p.z, sample.p.x, sample.p.y, sample.p.z);
//	//context.write("query-to-joint frame <%5.7f %5.7f %5.7f>\n",
//	//	fromQueryToJoint.p.x, fromQueryToJoint.p.y, fromQueryToJoint.p.z);
//
//	//context.write(" -> magnitude=%5.7f\n", fromQueryToJoint.p.magnitude());
//	//if (fromQueryToJoint.p.magnitude() > 2*ftDrivenDesc.ftModelDesc.distMax)
//	//	return golem::REAL_ZERO; //norm*density(distMin);
//
//	fromQueryToJoint.multiply(trn, fromQueryToJoint);
//	//context.write("new pose <%5.7f %5.7f %5.7f>,  model <%5.7f %5.7f %5.7f>, dist=%5.7f\n",
//	//	fromQueryToJoint.p.x, fromQueryToJoint.p.y, fromQueryToJoint.p.z, trn.p.x, trn.p.y, trn.p.z, trn.p.distance(fromQueryToJoint.p));
//	if (fromQueryToJoint.p.magnitude() < distMin) {
//		const size_t size = modelPoints.size() < ftDrivenDesc.ftModelDesc.points ? modelPoints.size() : ftDrivenDesc.ftModelDesc.points;
//		for (size_t i = 0; i < size; ++i) {
//			const Point& point = size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i];
//			const Real dist = fromQueryToJoint.p.distance(point.frame.p);
//			if (dist < distMin)
//				distMin = dist;
//		}
//	}
////	context.write(" -> dist(to shape)=%5.7f, norm=%5.7f, density=%5.7f, prob of touching=%5.7f\n", distMin, norm, density(distMin), norm*density(distMin));
//	return /*norm**/density(distMin);
}
