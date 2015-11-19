/** @file Heuristic.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Tools/XMLData.h>
#include <Golem/Tools/Data.h>
#include <Golem/UI/Data.h>
#include <Spam/Spam/Belief.h>
#include <Grasp/Contact/Contact.h>

//#ifdef WIN32
//	#pragma warning (push)
//	#pragma warning (disable : 4291 4244 4996 4305)
//#endif
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/common/centroid.h>
//#include <pcl/common/eigen.h>
//#include <flann/flann.hpp>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>
//#ifdef WIN32
//	#pragma warning (pop)
//#endif

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::XMLData(Belief::Desc& val, golem::XMLContext* context, bool create) {
	golem::XMLData("kernels", val.kernels, context, create);
	golem::XMLData("neighbours", val.neighbours, context, create);
	golem::XMLData("distance_range", val.distanceRange, context, create);
	golem::XMLData("feature_norm_eps", val.featNormEps, context, create);

	golem::XMLData("cluster", val.cluster, context, create);

	grasp::XMLData(val.dist, context->getContextFirst("dist"), create);
	golem::XMLData("prod", val.distProd, context->getContextFirst("dist"), create);
	golem::XMLData("feature", val.distFeature, context->getContextFirst("dist"), create);

	grasp::XMLData(val.poseStdDev, context->getContextFirst("pose_stddev"), create);

	golem::XMLData(&val.covariance[0], &val.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);

	golem::XMLData("population_size", val.populationSize, context->getContextFirst("mean_shift"), create);
	golem::XMLData("generations_min", val.generationsMin, context->getContextFirst("mean_shift"), create);
	golem::XMLData("generations_max", val.generationsMax, context->getContextFirst("mean_shift"), create);
	golem::XMLData("distance_diff", val.distanceDiff, context->getContextFirst("mean_shift"), create);

	try {
		val.localEnabled = true;
		val.optimisationDesc.load(context->getContextFirst("optimisation"));
	}
	catch (const golem::Message&) {
		val.localEnabled = false;
	}
	golem::XMLData("distance_max", val.distanceMax, context, create);
	XMLData(*val.hypothesisDescPtr, context->getContextFirst("hypothesis"), create);
	try {
		XMLData((grasp::RBPose::Desc&)*val.rbPoseDescPtr, context->getContextFirst("pose_estimation"));
	}
	catch (const golem::MsgXMLParser&) {}

	//golem::XMLData("kernels", val.tactile.kernels, context, create);
	//golem::XMLData("test", val.tactile.test, context, create);
	//grasp::XMLData(val.tactile.stddev, context->getContextFirst("test_stddev"), create);
	//golem::XMLData(&val.tactile.covariance[0], &val.tactile.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);
	golem::XMLData("num_hypotheses", val.numHypotheses, context->getContextFirst("tactile_model"), create);
	golem::XMLData("max_surface_points", val.maxSurfacePoints, context->getContextFirst("tactile_model"), create);
}

template <> void golem::Stream::read(spam::Belief& belief) const {
	belief.getSamples().clear();
	read(belief.getSamples(), belief.getSamples().begin());
	belief.getHypotheses().clear();
	read(belief.getHypotheses(), belief.getHypotheses().begin());
}

template <> void golem::Stream::write(const spam::Belief& belief) {
	write(belief.getSamples().begin(), belief.getSamples().end());
	write(belief.getHypotheses().begin(), belief.getHypotheses().end());
}

//------------------------------------------------------------------------------

Belief::Ptr Belief::Desc::create(golem::Context &context) const {
	Belief::Ptr belief(new Belief(context));
	belief->create(*this);
	return belief;
}

//------------------------------------------------------------------------------

Belief::Belief(golem::Context& context) : context(context), rand(context.getRandSeed()){
//	pRBPose.reset((grasp::RBPose*)this);
}

Belief::~Belief() {
}


bool Belief::create(const Desc& desc) {
	desc.assertValid(grasp::Assert::Context("Belief::Desc."));
	
	this->desc = desc;

	rbPosePtr = desc.rbPoseDescPtr->create(context);

	appearance.setToDefault();

	poses.clear();
	poses.resize(this->desc.kernels);

	// reset container for the low-dim rep belief
	hypotheses.clear();
	hypotheses.reserve(this->desc.numHypotheses);

	this->desc.poseStdDev.ang = Math::sqrt(REAL_ONE / this->desc.poseStdDev.ang);	// stdDev ~ 1/cov
	poseCovInv.lin = REAL_ONE / (poseCov.lin = Math::sqr(this->desc.poseStdDev.lin));
	poseCovInv.ang = REAL_ONE / (poseCov.ang = Math::sqr(this->desc.poseStdDev.ang));

	manipulator.reset();

	normaliseFac = REAL_ZERO;

	realPose.setId();

	context.write("Belief::create(): kernels %d\n RBPose::create(): kernels %d, features %d\n", desc.kernels, desc.rbPoseDescPtr->kernels, desc.rbPoseDescPtr->features);
//	mfsePoses.resize(desc.tactile.kernels);

	return true;
}

//------------------------------------------------------------------------------

void Belief::drawSamples(const size_t numSamples, golem::DebugRenderer& renderer) const {
	for (size_t t = 0; t < numSamples; ++t) {
		grasp::RBCoord s = sample();

		if (appearance.showFrames)
			renderer.addAxes(s.toMat34() * modelFrame, appearance.frameSize);

		if (appearance.showPoints) {
			grasp::Cloud::PointSeq seq;
			for (grasp::Cloud::PointSeq::const_iterator i = modelPoints.begin(); i != modelPoints.end(); ++i) {
				grasp::Cloud::Point point = *i;
				grasp::Cloud::setColour(appearance.colour, point);
				seq.push_back(point);
			}
			grasp::Cloud::transform(s.toMat34(), seq, seq);
			for (grasp::Cloud::PointSeq::const_iterator i = seq.begin(); i != seq.end(); ++i)
				renderer.addPoint(grasp::Cloud::getPoint<Real>(*i));
		}
	}
}

void Belief::drawHypotheses(golem::DebugRenderer &renderer, const bool showOnlyMeanPose) const {
	context.write("Belief::drawHypotheses (%s)\n", showOnlyMeanPose ? "ON" : "OFF");
	//for (Hypothesis::Seq::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h) {
	//	(*h)->draw(renderer);
	//	if (showOnlyMeanPose)
	//		return;
	//}
}

golem::Bounds::Seq Belief::uncertaintyRegionBounds() {
	Bounds::Seq bounds;
	bounds.push_back(uncertaintyDesc.create());
	return bounds;
}

//------------------------------------------------------------------------------

void Belief::set(const grasp::RBPose::Sample::Seq &poseSeq, const grasp::RBPose::Sample::Seq &hypothesisSeq, const Mat34 &trn, const grasp::Cloud::PointSeq &points) {
	modelPoints.clear();
	modelPoints.reserve(points.size());
	for (grasp::Cloud::PointSeq::const_iterator i = points.begin(); i != points.end(); ++i)
		modelPoints.push_back(*i);
	context.write("Belief::set(): model points size %d\n", modelPoints.size());
	modelFrame = trn;

	try {
		setPoses(poseSeq);
		setHypotheses(hypothesisSeq);
	}
	catch (const Message &msg) {
		context.write("%s\n", msg.str().c_str());
	}
}

void Belief::setPoses(const grasp::RBPose::Sample::Seq &poseSeq) {
	if (poseSeq.empty())
		throw Message(golem::Message::LEVEL_ERROR, "Belief::setPoses(): Invalid samples.");

	// reset containers
	poses.clear();
	poses.reserve(poseSeq.size());
	initPoses.clear();
	initPoses.reserve(poseSeq.size());

	// copy items
	for (grasp::RBPose::Sample::Seq::const_iterator p = poseSeq.begin(); p != poseSeq.end(); ++p) {
		poses.push_back(*p);
		initPoses.push_back(*p);
	}

	// mean and covariance
	if (!pose.create<golem::Ref1, grasp::RBPose::Sample::Ref>(grasp::RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
	// copy initial distribution properties
	initProperties = pose;


	context.write("spam::Belief::setPoses(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", initProperties.covariance[0], initProperties.covariance[1], initProperties.covariance[2], initProperties.covariance[3], initProperties.covariance[4], initProperties.covariance[5], initProperties.covariance[6]);
}

/** Sets hypotheses */
void Belief::setHypotheses(const grasp::RBPose::Sample::Seq &hypothesisSeq) {
	if (hypothesisSeq.empty() || !modelFrame.isFinite() || modelPoints.empty())
		throw Message(golem::Message::LEVEL_ERROR, "Belief::setHypotheses(): Invalid samples (Hypotheses size %d, modelFrame is %s, modelPoints size %d)", hypothesisSeq.size(), modelFrame.isFinite()?"finite":"not finite", modelPoints.size());

	context.debug("----------------------------------------------------------\n");
	context.debug("Belif:setHypotheses()\n");

	// reset container for hypotheses
	hypotheses.clear();
	hypotheses.resize(hypothesisSeq.size());
	U32 idx = 0;
	Hypothesis::Seq::iterator i = hypotheses.begin();
	for (grasp::RBPose::Sample::Seq::const_iterator p = hypothesisSeq.begin(); p != hypothesisSeq.end(); ++p, ++i) {
		// container for the point cloud of this sample (default: the same points of the model)
//		grasp::RBPose::Sample sample(*p);
		grasp::Cloud::PointSeq sampleCloud;
		const size_t size = modelPoints.size() < desc.maxSurfacePoints ? modelPoints.size() : desc.maxSurfacePoints;
		for (size_t j = 0; j < size; ++j) {
			grasp::Cloud::Point point = size < modelPoints.size() ? modelPoints[size_t(rand.next()) % modelPoints.size()] : modelPoints[j]; // make a copy here
			grasp::Cloud::setPoint(p->toMat34() * grasp::Cloud::getPoint<Real>(point)/* + actionFrame.p*/, point);
			grasp::Cloud::setColour((p == hypothesisSeq.begin()) ? RGBA::GREEN : RGBA::BLUE, point);
			sampleCloud.push_back(point);
		}
//		hypotheses.push_back(Hypothesis::Ptr(new Hypothesis(idx, modelFrame, *p, sampleCloud)));
		(*i) = desc.hypothesisDescPtr->create(*manipulator.get());
		(*i)->create(idx, modelFrame, *p, rand, sampleCloud);
		(*i)->appearance.colour = (p == hypothesisSeq.begin()) ? RGBA::GREEN : RGBA::BLUE;
		(*i)->appearance.showPoints = true;
		context.debug("Hypothesis n.%d <%.4f %.4f %.4f> <%.4f %.4f %.4f %.4f>\n", idx, p->p.x, p->p.y, p->p.z, p->q.w, p->q.x, p->q.y, p->q.z);
		idx++;
	}

	// mean and covariance
	if (!sampleProperties.create<golem::Ref1, grasp::RBPose::Sample::Ref>(grasp::RBCoord::N, desc.covariance, getHypothesesToSample()))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");

	// compute a volumetric region of the uncertainty (min 10 cms in each direction)
	Vec3 cov(pose.covariance[0], pose.covariance[1], pose.covariance[2]*100);
	uncertaintyDesc.dimensions = hypotheses.front()->boundsDesc.dimensions + cov*1000;//Vec3(pose.covariance[0] * 5000 > d ? pose.covariance[0] * 5000 : d, pose.covariance[1] * 5000 > d ? pose.covariance[1] * 5000 : d, pose.covariance[2] * 5000 > d ? pose.covariance[2] * 5000 : d);
	context.write("Uncertainty dimensions = [%f, %f, %f]\n", uncertaintyDesc.dimensions.x, uncertaintyDesc.dimensions.y, uncertaintyDesc.dimensions.z);
	uncertaintyDesc.pose.p = hypotheses.front()->toRBPoseSampleGF().p;

	context.debug("Sub-sampled covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
}


grasp::RBPose::Sample Belief::createHypotheses(const grasp::Cloud::PointSeq& model, const golem::Mat34 &transform/*, const bool init*/) {
	U32 idx = 0;
	// copy model and model frame. Note: it is done only the very first time.
	if (modelPoints.empty()) {
		modelPoints.clear();
		modelPoints.resize(model.size());
		for (grasp::Cloud::PointSeq::const_iterator i = model.begin(); i != model.end(); ++i) 
			modelPoints.push_back(*i);
		modelFrame = transform;
	}
	// reset container for hypotheses
	hypotheses.clear();
	hypotheses.resize(desc.numHypotheses);

	//const grasp::RBPose::Sample maximumFrame = maximum();
	grasp::RBPose::Sample bestSample;
	grasp::RBPose::Sample::Seq clusters;
	meanShiftClustering(bestSample, clusters);
	
	//Vec3 offset(-.1, .07, .0);
	//grasp::RBPose::Sample actionFrame = maximum();
	//	context.write("Heuristic:setBeliefState(model size = %d, max points = %d): samples: cont_fac = %f\n", model.size(), ftDrivenDesc.maxSurfacePoints, ftDrivenDesc.contactFac);
	context.debug("----------------------------------------------------------\n");
	context.debug("Belief:createHypotheses()\n");
	U32 clusterId = 0;
	for (Hypothesis::Seq::iterator i = hypotheses.begin(); i < hypotheses.end(); ++i) {
		// sample hypothesis. NOTE: The first element is the max scoring pose
		//grasp::RBCoord actionFrame = (i == hypotheses.begin()) ? maximumFrame : sample();
		const grasp::RBCoord actionFrame = (i == hypotheses.begin()) ? bestSample : [&]() -> const grasp::RBCoord {
			if (clusterId < clusters.size()) return clusters[clusterId++];
			else return sample();
		}();

		//sampleFrame.p += offset;
		// transform the sample in the reference coordinate (default: robot's coordinate frame)
//		sampleFrame.multiply(sampleFrame, grasp::RBCoord(transform));
		// container for the point cloud of this sample (default: the same points of the model)
		grasp::Cloud::PointSeq sampleCloud;
		
		// copy model point cloud
		// optimisation: limit the number of points to save performance in the kd-tree
		const size_t size = model.size() < desc.maxSurfacePoints ? model.size() : desc.maxSurfacePoints;
		for (size_t j = 0; j < size; ++j) {
			grasp::Cloud::Point p = size < model.size() ? model[size_t(rand.next())%model.size()] : model[j]; // make a copy here
			grasp::Cloud::setPoint(actionFrame.toMat34() * grasp::Cloud::getPoint<Real>(p)/* + actionFrame.p*/, p);
			grasp::Cloud::setColour((i == hypotheses.begin()) ? RGBA::GREEN : RGBA::BLUE, p);
			sampleCloud.push_back(p);
		}
		//		hypotheses.insert(Hypothesis::Map::value_type(idx, Hypothesis::Ptr(new Hypothesis(idx, grasp::RBPose::Sample(sampleFrame), sampleCloud))));
		//hypotheses.push_back(Hypothesis::Ptr(new Hypothesis(idx, transform, grasp::RBPose::Sample(actionFrame), sampleCloud)));
		try {
			(*i) = desc.hypothesisDescPtr->create(*manipulator);
			(*i)->create(idx, modelFrame, grasp::RBPose::Sample(actionFrame), rand, sampleCloud);
			(*i)->appearance.colour = (i == hypotheses.begin()) ? RGBA::GREEN : RGBA::BLUE;
			(*i)->appearance.showPoints = true;
		}
		catch (const Message &msg) {
			context.write("%s\n", msg.what());
		}
		//Mat34 sampleFrame; 
		//sampleFrame.multiply(actionFrame.toMat34(), modelFrame);
		//grasp::RBCoord sf(sampleFrame);
		context.write("Hypothesis %d { %.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f }\n", idx, actionFrame.p.x, actionFrame.p.y, actionFrame.p.z, actionFrame.q.w, actionFrame.q.x, actionFrame.q.y, actionFrame.q.z);
		//context.write("HypothesisGF %d { %.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f }\n", idx, sf.p.x, sf.p.y, sf.p.z, sf.q.w, sf.q.x, sf.q.y, sf.q.z);
		idx++;
	}
	
	// mean and covariance
	if (!sampleProperties.create<golem::Ref1, grasp::RBPose::Sample::Ref>(grasp::RBCoord::N, desc.covariance, getHypothesesToSample()))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createHypotheses(): Unable to create mean and covariance for the high dimensional representation");
		
	context.write("Sub-sampled covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n", sampleProperties.covariance[0], sampleProperties.covariance[1], sampleProperties.covariance[2], sampleProperties.covariance[3], sampleProperties.covariance[4], sampleProperties.covariance[5], sampleProperties.covariance[6]);
	
	// compute determinant for the sampled hypotheses
	covarianceDet = REAL_ONE;
	for (golem::U32 j = 0; j < grasp::RBCoord::N; ++j)
			covarianceDet *= sampleProperties.covariance[j];

	// compute a volumetric region of the uncertainty (min 10 cms in each direction)
	Vec3 cov(pose.covariance[0], pose.covariance[1], pose.covariance[2] * 100);
	uncertaintyDesc.dimensions = hypotheses.front()->boundsDesc.dimensions + cov * 1000;//Vec3(pose.covariance[0] * 5000 > d ? pose.covariance[0] * 5000 : d, pose.covariance[1] * 5000 > d ? pose.covariance[1] * 5000 : d, pose.covariance[2] * 5000 > d ? pose.covariance[2] * 5000 : d);
	uncertaintyDesc.pose.p = hypotheses.front()->toRBPoseSampleGF().p;

	//// Reduce the number of poses to reduce the computational time of belief update
	//if (init) {
	//	grasp::RBPose::Sample::Seq tmp;
	//	for (size_t i = 0; i < myDesc.numPoses; ++i)
	//		tmp.push_back(sample());
	//	poses.clear();
	//	poses.reserve(myDesc.numPoses);
	//	for (size_t i = 0; i < myDesc.numPoses; ++i) {
	//		poses.push_back(grasp::RBPose::Sample(grasp::RBCoord(tmp[i].p, tmp[i].q), REAL_ONE, i*REAL_ONE));
	//	}
	//
	//	// set initial belief
	//	initPoses.clear();
	//	initPoses.reserve(myDesc.numPoses);
	//	for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
	//		initPoses.push_back(*i);
	//	}
	//	initProperties = sampleProperties;
	//}
	//actionFrame.p += offset;
	return bestSample; // maximumFrame;
}

grasp::RBPose::Sample::Seq Belief::getHypothesesToSample() const {
	// return the rbpose::sample associated with each hypothesis.
	// NOTE: useful for creating mean and covariance in Belief::setHypotheses
	grasp::RBPose::Sample::Seq samples;
	for (Hypothesis::Seq::const_iterator h = hypotheses.begin(); h != hypotheses.end(); ++h)
		samples.push_back((*h)->toRBPoseSample());
	return samples;
}

//------------------------------------------------------------------------------

void Belief::createQuery(const grasp::Cloud::PointSeq& points) {
	SecTmReal init = context.getTimer().elapsed();
	size_t id = 1;
	poses.clear();
	poses.reserve(desc.kernels);
	for (size_t i = 0; i < desc.kernels; ++i) {
		// reset RBPose::poses for the fitting dist
		//poses.clear();
		//poses.reserve(myDesc.kernels);
		context.write("Belief::Query: %d/%d\r", id++, desc.kernels);
		rbPosePtr->createQuery(points);
		grasp::RBPose::Sample s = rbPosePtr->maximum();
//		grasp::RBCoord c; c.multiply(s, modelFrame);
		context.debug("%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\t%.5f\n", s.p.x, s.p.y, s.p.z, s.q.w, s.q.x, s.q.y, s.q.z);
		poses.push_back(s);
		context.debug("----------------------------------------------------------\n");
	}
	const SecTmReal t_end = context.getTimer().elapsed() - init;
	
	// mean and covariance
	if (!pose.create<golem::Ref1, grasp::RBPose::Sample::Ref>(grasp::RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
	
	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < desc.kernels; ++i)
		rand.nextGaussianArray<golem::Real>(&(poses[i])[0], &(poses[i])[0] + grasp::RBCoord::N, &((poses[i]))[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian

	context.write("Belief::createQuery(): elapsed_time=%f\n", t_end);
	context.write("Belief::createQuery(): sampled covariance = {(%f, %f, %f), (%f, %f, %f, %f)}\n",
		pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3],
		pose.covariance[4], pose.covariance[5], pose.covariance[6]);
}

Real Belief::maxWeight(const bool normalised) const {
	Real max = golem::REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::const_iterator s = poses.begin(); s != poses.end(); ++s) {
		const Real w = normalised && normaliseFac > REAL_ZERO ? s->weight / normaliseFac : s->weight;
		if (w > max)
			max = w;
	}
	return max;
}

void Belief::meanShiftClustering(grasp::RBPose::Sample& solution, grasp::RBPose::Sample::Seq& clusters) {
	context.debug("RBPose::alignGlobal(): Global alignment...\n");
	struct Cluster {
		grasp::RBPose::Sample sample;
		Real eval;
		U32 votes;
	};
	typedef std::map<U32, Cluster> ClusterMap;
	
	ClusterMap clusterMap;
	U32 numClusters = 0, totalVotes = 0;
	clusters.clear();
	
	for (size_t i = 0; i < 10; ++i) {
		grasp::RBPose::Sample sol;
		U32 votes = 0;
		Real solutionEval = REAL_MIN;
		alignGlobal(sol, solutionEval, votes);
		totalVotes += votes;
		context.write("align global [eval votes] = [%f %u/%u]\n", solutionEval, votes, totalVotes);

		ClusterMap::iterator ptr = clusterMap.end();
		Real min = REAL_MAX;
		for (ClusterMap::iterator j = clusterMap.begin(); j != clusterMap.end(); ++j) {
			const Real d = distance(sol, j->second.sample);
			if (d < desc.distanceRange && d < min) {
				ptr = j;
				min = d;
			}
		}
		if (ptr != clusterMap.end()) {
			grasp::RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
			Real norm = REAL_ZERO, normBuf = REAL_ZERO;
			const Real w = solutionEval*kernel(min);
			golem::kahanSum(norm, normBuf, w);
			golem::kahanSum(norm, normBuf, ptr->second.eval);
			for (size_t l = 0; l < grasp::RBCoord::N; ++l)
				golem::kahanSum(mean[l], meanBuf[l], ptr->second.eval*ptr->second.sample[l]);
			for (size_t l = 0; l < grasp::RBCoord::N; ++l)
				golem::kahanSum(mean[l], meanBuf[l], w*sol[l]);
			
			if (norm <= REAL_ZERO)
				break;
			
			const Real normInv = REAL_ONE / norm;
			mean.p *= normInv;
			mean.q *= normInv;
			mean.q.normalise();
			ptr->second.sample = mean;
			ptr->second.eval = norm / 2;
			ptr->second.votes += votes;
		}
		else {
			Cluster cluster; cluster.sample = sol; cluster.eval = solutionEval; cluster.votes = votes;
			clusterMap.insert(clusterMap.end(), ClusterMap::value_type(numClusters++, cluster));
		}
	}

	if (clusterMap.empty())
		throw Message(Message::LEVEL_ERROR, "Belief::meanShiftClustering(): no clusters");

	ClusterMap::iterator best = clusterMap.end();
	Real bestEval = REAL_ZERO;
	U32 voteMax = 0;
	context.write("Sim obj [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n", realPose.p.x, realPose.p.y, realPose.p.z, realPose.q.w, realPose.q.x, realPose.q.y, realPose.q.z);

	for (ClusterMap::iterator j = clusterMap.begin(); j != clusterMap.end(); ++j) {
		const Real score = (j->second.votes / Real(totalVotes)) * j->second.eval;
		grasp::RBCoord s(j->second.sample * modelFrame);
		const grasp::RBDist d(realPose, grasp::RBCoord(j->second.sample * modelFrame));
		context.write("cluster [%u] eval=%.5f votes=%u score=%.5f error lin=%.5f ang=%.5f\n  mean [%.2f %.2f %.2f %.2f %.2f %.2f %.2f]\n", j->first, j->second.eval, j->second.votes, score, Math::sqrt(d.lin), d.ang,
			s.p.x, s.p.y, s.p.z, s.q.w, s.q.x, s.q.y, s.q.z);

		if (/*j->second.eval*/ score > bestEval) {
			best = j;
			bestEval = score;// j->second.eval;
			voteMax = j->second.votes;
		}
	}
	context.write("BEST: cluster [%u]\n", best->first);

	solution = best->second.sample;
	for (ClusterMap::iterator j = clusterMap.begin(); j != clusterMap.end(); ++j)
		if (j->first != best->first)
			clusters.push_back(j->second.sample);

//	return;
//
//	// mean shift parameters. TODO: add them to a descriptor file
//	Real maxLinDist = Real(0.1), maxAngDist = Real(0.5);
//	grasp::RBDist stopCriterion(0.05, 0.05);
//	const size_t attempts = 10;
//	// collection of distances for RBPoses
//	typedef std::vector<grasp::RBDist> RBDistSeq;
//
//	// N is the dimension of poses
//	U32 numClusters = 0, N = poses.size();
//	Real solutionEval = REAL_MIN;
//	// index of non-visited and visited poses
//	std::vector<bool> visited; visited.assign(N, false);
//	clusters.clear();
//	grasp::U32Seq clusterVotes; clusterVotes.clear();
//
//	for (size_t j = 0; j < desc.generationsMax; ++j) {
//		grasp::U32Seq members, votes; members.clear(); votes.assign(N, U32(0));
//		//const U32 dim = [&]() -> const U32 {
//		//	U32 r(0);
//		//	for (auto i = visited.begin(); i != visited.end(); ++i)
//		//		if (!*i) ++r;
//		//	return r;
//		//}();
//		const U32 seed = rand.nextUniform<U32>(0, N);
//		if (seed >= N)
//			throw Message(Message::LEVEL_ERROR, "Belief::meanShiftClustering(): Sampling error");
//
//		grasp::RBPose::Sample tmpMean = poses[seed];
//		context.write("Belief::MeanShift(): seed %u, pose [%.2f %.2f %.2f]\n", seed, poses[seed].p.x, poses[seed].p.y, poses[seed].p.z);
//
//		for (size_t n = 0; n < attempts; ++n) {
//			// compute distance from tmpMean to all the poses still active
//			RBDistSeq distances; distances.assign(N, grasp::RBDist());
//			grasp::RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
//			Real norm = REAL_ZERO, normBuf = REAL_ZERO;
//			U32 numVisited(0);
//			for (U32 i = 0; i < N; ++i) {
//				// skip visited poses
//				if (visited[i]) continue;
//				// compute distance [lin, ang]
//				distances[i].set(tmpMean, poses[i]);
////				context.write("Belief::MeanShift(): %i distance [%f %f]\n", i, distances[i].lin, distances[i].ang);
//				// add to new mean if the distance is less then treashold
//				if (distances[i].lin < maxLinDist && distances[i].ang < maxAngDist) {
//					visited[i] = true;
//					++numVisited;
//					for (size_t l = 0; l < grasp::RBCoord::N; ++l)
//						golem::kahanSum(mean[l], meanBuf[l], poses[i][l]);
//				}
//			}
//			if (numVisited == 0)
//				continue;
//			mean.p *= 1/numVisited;
//			mean.q *= 1/numVisited;
//			grasp::RBDist distance(tmpMean, mean);
//			// if mean doesn't move much stop this cluster 
//			if (distance.lin < stopCriterion.lin && distance.ang < stopCriterion.ang) {
//				grasp::RBDist min(REAL_MAX, REAL_MAX);
//				I32 mergeWith = -1, idx = 0;
//				for (grasp::RBPose::Sample::Seq::const_iterator clMean = clusters.begin(); clMean != clusters.end(); ++clMean, ++idx) {
//					grasp::RBDist dist(mean, *clMean);
//					if (dist.lin < min.lin && dist.ang < min.ang) {
//						mergeWith = idx;
//						min.set(dist.lin, dist.ang);
//					}
//				}
//				if (mergeWith > -1) {
//					grasp::RBCoord _mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), _meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
//					for (size_t l = 0; l < grasp::RBCoord::N; ++l)
//						golem::kahanSum(_mean[l], _meanBuf[l], clusters[mergeWith][l]);
//					_mean.p *= 0.5;
//					_mean.q *= 0.5;
//					clusters[mergeWith] = _mean;
//					clusterVotes[mergeWith] += numVisited;
//				}
//				else {
//					clusters.push_back(mean);
//					clusterVotes.push_back(numVisited);
//				}
//				break;
//			}
//		} // loop until converge
//	} // while nonVisited is empty
//	if (clusterVotes.size() == 0 && clusters.size() == 0)
//		throw Message(Message::LEVEL_ERROR, "Belief::meanShiftClustering(): no clusters");
//
//	U32 voteMax = 0, best = 0;
//	for (U32 i = 0; i < clusterVotes.size(); ++i) {
//		if (clusterVotes[i] > voteMax) {
//			voteMax = clusterVotes[i];
//			best = i;
//		}
//	}
//	solution = clusters[best];
//	// print debug message
//	context.debug("RBPose::alignGlobal(): Objective value: %f\n", solutionEval);
}

void Belief::alignGlobal(grasp::RBPose::Sample& solution, Real& solutionEval, U32& votes) {
	context.debug("RBPose::alignGlobal(): Global alignment...\n");

	size_t k = 0;
	solutionEval = REAL_MIN;
	U32 thisClusterVotes = 0;
	CriticalSection cs;
	grasp::ParallelsTask(context.getParallels(), [&](grasp::ParallelsTask*) {
		grasp::RBCoord test;
		Real testEval = REAL_MIN;
		for (;;) {
			{
				CriticalSectionWrapper csw(cs);
				if (solutionEval < testEval) {
					solutionEval = testEval;
					solution = test;
					votes = thisClusterVotes;
				}
				if (++k > desc.populationSize)
					break;
			}

			test = sample();
			thisClusterVotes = 1;
			for (size_t j = 0; j < desc.generationsMax; ++j) {
				// approximate affine combination of quaternions: renormalisation
				grasp::RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
				Real norm = REAL_ZERO, normBuf = REAL_ZERO;
				for (size_t i = 0; i < desc.neighbours; ++i) {
					const grasp::RBPose::Sample s = poses[rand.next() % poses.size()];
					const Real d = distance(test, s);
					if (d < desc.distanceRange) {
						++thisClusterVotes;
						const Real w = s.weight*kernel(d);
						golem::kahanSum(norm, normBuf, w);
						for (size_t l = 0; l < grasp::RBCoord::N; ++l)
							golem::kahanSum(mean[l], meanBuf[l], w*s[l]);
					}
				}
				if (norm <= REAL_ZERO)
					break;
				const Real normInv = REAL_ONE / norm;
				mean.p *= normInv;
				mean.q *= normInv;
				mean.q.normalise();
				const Real d = distance(test, mean);
				test = mean;
				if (j > desc.generationsMin && d < desc.distanceDiff)
					break;
			}

			testEval = density(test);
		}
	});

	// print debug message
	context.debug("RBPose::alignGlobal(): Objective value: %f\n", solutionEval);
}

grasp::RBPose::Sample Belief::maximum() {
	grasp::RBPose::Sample solution;
	grasp::RBPose::Sample::Seq clusters;
	U32 votes;
	Real solutionEval = REAL_ZERO;
	if (desc.cluster)
		meanShiftClustering(solution, clusters);
	else
		alignGlobal(solution, solutionEval, votes);
	return solution;
}


void Belief::createResample(/*const grasp::Manipulator::Config& robotPose*/) {
	// check for collisions to reject samples
	//Collision::FlannDesc waypointDesc;
	//Collision::Desc::Ptr cloudDesc;
	//cloudDesc.reset(new Collision::Desc());
	//Collision::Ptr cloud = cloudDesc->create(*manipulator);
	//waypointDesc.depthStdDev = 0.0005; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
	//waypointDesc.radius = -0.005;
	 
	size_t N = poses.size(), index = rand.nextUniform<size_t>(0, N);
	Real beta = golem::REAL_ZERO;
	grasp::RBPose::Sample::Seq newPoses;
	newPoses.reserve(N);
	for (size_t i = 0; i < N; ++i) {
		beta += rand.nextUniform<golem::Real>()*2*maxWeight();
//		context.write("spam::RBPose::createResampling(): beta=%4.6f\n", beta);
		while (beta > poses[index].weight) {
			beta -= poses[index].weight;
			index = (index + 1) % N;
//			context.write("beta=%4.6f\n, index=%d, weight=%4.6f\n", beta, index, poses[index].weight);
		}
		newPoses.push_back(poses.at(index));
	}
	
	// add noise to the resampled elements and overwrite poses
	poses.clear();
	poses.reserve(N);

	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < N; ++i) {
		//mfsePoses.push_back(Sample(newPoses[i], REAL_ONE, i*REAL_ONE));
		//continue;
		grasp::RBCoord c = newPoses[i];
		rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &(newPoses[i])[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian
		//grasp::Cloud::PointSeq points;
		//grasp::Cloud::transform(c.toMat34(), modelPoints, points);
		//cloud->create(rand, points);
		//if (cloud->check(waypointDesc, rand, robotPose)) continue;
		poses.push_back(grasp::RBPose::Sample(c, REAL_ONE, i*REAL_ONE));
		//++i;
	}
	normaliseFac = REAL_ZERO;
	
	// compute mean and covariance
	if (!pose.create<golem::Ref1, grasp::RBPose::Sample::Ref>(grasp::RBCoord::N, desc.covariance, poses))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createResample(): Unable to create mean and covariance for the high dimensional representation");

	context.write("spam::Belief::createResample(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", pose.covariance[0], pose.covariance[1], pose.covariance[2], pose.covariance[3], pose.covariance[4], pose.covariance[5], pose.covariance[6]);
}

//------------------------------------------------------------------------------

grasp::RBCoord Belief::sample() const {
	grasp::RBPose::Sample::Seq::const_iterator ptr = grasp::RBPose::Sample::sample<golem::Ref1, grasp::RBPose::Sample::Seq::const_iterator>(poses, rand);
	if (ptr == poses.end())
		throw Message(Message::LEVEL_ERROR, "RBPose::sample(): Sampling error");

	grasp::RBCoord c;
	//Vec3 v;
	//v.next(rand); // |v|==1
	//v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, desc.poseStdDev.lin)), v);
	//c.p.add(ptr->p, v);
	//Quat q;
	//q.next(rand, poseCovInv.ang);
	//c.q.multiply(ptr->q, q);
	this->rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &(*ptr)[0], &pose.covarianceSqrt[0]); // normalised multivariate Gaussian
	//grasp::RBDist noise(*ptr, c);
	//context.write("Sampling noise [lin ang] = [%f %f]\n", noise.lin, noise.ang);
	return c;
}

golem::Real Belief::distance(const grasp::RBCoord& a, const grasp::RBCoord& b) const {
	//return poseCovInv.dot(RBDist(a, b));

	const Real d0 = pose.covarianceInv[0] * golem::Math::sqr(a[0] - b[0]) + pose.covarianceInv[1] * golem::Math::sqr(a[1] - b[1]) + pose.covarianceInv[2] * golem::Math::sqr(a[2] - b[2]);
	const Real d1 = pose.covarianceInv[3] * golem::Math::sqr(a[3] - b[3]) + pose.covarianceInv[4] * golem::Math::sqr(a[4] - b[4]) + pose.covarianceInv[5] * golem::Math::sqr(a[5] - b[5]) + pose.covarianceInv[6] * golem::Math::sqr(a[6] - b[6]);
	const Real d2 = pose.covarianceInv[3] * golem::Math::sqr(a[3] + b[3]) + pose.covarianceInv[4] * golem::Math::sqr(a[4] + b[4]) + pose.covarianceInv[5] * golem::Math::sqr(a[5] + b[5]) + pose.covarianceInv[6] * golem::Math::sqr(a[6] + b[6]);
	return golem::REAL_HALF*(d0 + std::min(d1, d2));
}

golem::Real Belief::kernel(golem::Real distance) const {
	return golem::Math::exp(-distance); // exponential
}

golem::Real Belief::density(const grasp::RBCoord &c) const {
	Real sum = REAL_ZERO;
	Real buf = REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i) {
		const Real dist = distance(c, *i);
		if (dist < desc.distanceRange)
			golem::kahanSum(sum, buf, i->weight*kernel(dist));
	}

	return sum;// up to scaling factor
}

void Belief::createUpdate(const Collision::Ptr collision, const golem::Waypoint &w, FTGuard::Seq &triggeredGuards, const grasp::RBCoord &rbPose) {
//	context.debug("Belief::createUpdate(collision)...\n");
	Collision::FlannDesc waypointDesc;
	Collision::Desc::Ptr cloudDesc;
	cloudDesc.reset(new Collision::Desc());
	Collision::Ptr cloud = cloudDesc->create(*manipulator);
	waypointDesc.depthStdDev = 0.0035/*0.0005*/; waypointDesc.likelihood = 1000.0; waypointDesc.points = 10000; waypointDesc.neighbours = 100;
	waypointDesc.radius = REAL_ZERO;
	
	golem::Controller::State state = manipulator->getController().createState();

	manipulator->getController().lookupState(golem::SEC_TM_REAL_MAX, state);

	// position control
	state.cpos = w.cpos;

	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
		grasp::Cloud::PointSeq points;
		grasp::Cloud::transform(sampledPose->toMat34(), modelPoints, points);
		cloud->create(rand, points);
		sampledPose->weight = cloud->evaluate(waypointDesc, manipulator->getConfig(state), triggeredGuards, false);
		grasp::RBDist error;
		error.lin = rbPose.p.distance(sampledPose->p);
		error.ang = rbPose.q.distance(sampledPose->q);
		//context.write("sample.weight = %f, Error {lin, ang} = {%f, %f}\n", sampledPose->weight, error.lin, error.ang);
		//Mat34 p; p.multiply(sampledPose->toMat34(), modelFrame);
//		context.write("%.2f\t%.2f\t%.2f\t%.2f\n", p.p.x, p.p.y, p.p.z, sampledPose->weight);
	}

	// normalise weights
	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO, cdf = golem::REAL_ZERO;
	normaliseFac = REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose)
		golem::kahanSum(normaliseFac/*norm*/, c, sampledPose->weight/*sampledPose->weight > 0 ? sampledPose->weight : Math::log10(REAL_EPS)*/);
	c = golem::REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
//		sampledPose->weight /= norm;
//		context.write("normilised sample.weight = %f\n", sampledPose->weight);
		golem::kahanSum(cdf, c, sampledPose->weight);
		sampledPose->cdf = cdf;
	}
}


//void Belief::createUpdate(const grasp::Manipulator *manipulator, const golem::Controller::State::Info handInfo, const golem::Waypoint &w, const FTGuard::Seq &triggeredGuards, const grasp::RealSeq &force) {
//	context.debug("spam::Belief::createUpdate()...\n");
//	bool intersect = false; 
//	// retrieve wrist's workspace pose and fingers configuration
//	//golem::Mat34 poses[grasp::Manipulator::JOINTS];
//	//manipulator->getPoses(manipulator->getPose(w.cpos), poses);
////	grasp::RealSeq forces(force);
//	for (Chainspace::Index i = handInfo.getChains().begin(); i != handInfo.getChains().end(); ++i) {
//		// check if any of the joint in the current chain (finger) has been triggered
//		std::vector<bool> triggered;
//		triggered.reserve(handInfo.getJoints(i).size());
////		context.write("Chain %d\n", i);
//		// bounds of the entire finger
//		golem::Bounds::Seq bounds;
//		grasp::RealSeq forces;
//		forces.assign(handInfo.getJoints(i).size(), REAL_ZERO);
//		for (Configspace::Index j = handInfo.getJoints(i).begin(); j != handInfo.getJoints(i).end(); ++j) {
//			const size_t idx = j - handInfo.getJoints(i).begin();
//			triggered[idx] = [&] () -> bool {
//				for (FTGuard::Seq::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
//					if (j == i->jointIdx) {
//						forces[idx] = i->force; // i->type == FTGuard_ABS ? REAL_ZERO : i->type == FTGuard_LESSTHAN ? -REAL_ONE : REAL_ONE;
//						return true;
//					}
//				return false;
//			} ();
//			//const golem::U32 joint = manipulator->getArmJoints() + idx;
//			//golem::Bounds::Seq bounds;
//			//manipulator->getJointBounds(joint, poses[joint], bounds);
//			manipulator->getJointBounds(golem::U32(j - manipulator->getController().getStateInfo().getJoints().begin()), w.wposex[j], bounds);
//		}
//			//for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b)
//			//		context.write("finger bound frame <%f %f %f>\n", (*b)->getPose().p.x, (*b)->getPose().p.y, (*b)->getPose().p.z);
//		golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
////		context.debug("Finger %d\n", i);
//		for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
////			context.debug("<%f %f %f> <%f %f %f %f>\n", sampledPose->p.x, sampledPose->p.y, sampledPose->p.z, sampledPose->q.w, sampledPose->q.x, sampledPose->q.y, sampledPose->q.z);
//			const Real eval = evaluate(bounds, grasp::RBCoord(w.wpos[i]), *sampledPose, forces, triggered, intersect);
//			const bool fingerTriggered = triggered[0] || triggered[1] || triggered[2] || triggered[3]; /*[&] () -> bool {
//				for (std::vector<bool>::const_iterator i = triggered.begin(); i != triggered.end(); ++i)
//					if (*i) return true;
//				return false;
//			}();*/
////			sampledPose->weight += fingerTriggered ? myDesc.sensory.contactFac*eval : intersect ? myDesc.sensory.noContactFac*(REAL_ONE - eval) : REAL_ZERO;
////			context.debug("pose <%f %f %f>: eval=%f log=%f, trigguered %s intersect %s prob=%f\n", sampledPose->p.x, sampledPose->p.y, sampledPose->p.z, eval, -Math::log10(eval), fingerTriggered ? "Y" : "N", intersect ? "Y" : "N", fingerTriggered ? myDesc.sensory.contactFac*golem::Math::log10(eval) : intersect ? -REAL_MAX : REAL_ZERO);
////			if (fingerTriggered) context.write("Eval = %f, finger triggered = %s, log(eval) [contactFac, log10] = %f [%f, %f]\n", eval, fingerTriggered ? "YES" : "NO", Math::abs(1000.0*golem::Math::log10(eval)), 1000.0, golem::Math::log10(eval));
//			golem::kahanSum(sampledPose->weight, c, fingerTriggered ? 1000.0*golem::Math::log10(eval) : intersect ? -REAL_MAX : REAL_ZERO);
//			//golem::kahanSum(sampledPose->weight, c, fingerTriggered ? Math::abs(/*myDesc.sensory.contactFac*/1000.0*golem::Math::log10(eval)) : intersect ? -REAL_MAX : REAL_ZERO);
//			//const Real eval = evaluate(bounds, grasp::RBCoord(w.wposex[j]), *sampledPose, forces[idx], j == robot->getStateHandInfo().getJoints(i).begin(), intersect);
//			//sampledPose->weight *= (triggered ? myDesc.sensory.contactFac*eval : intersect ? myDesc.sensory.noContactFac*(REAL_ONE - eval) : REAL_ZERO);
//			//golem::kahanSum(norm, c, sampledPose->weight);
//		//}
//		}
////		context.debug("\n---------------------------------------------\n");
//	}
//
//	// normalise weights
//	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO, cdf = golem::REAL_ZERO;
//	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose)
//		golem::kahanSum(norm, c, sampledPose->weight > 0 ? sampledPose->weight : Math::log10(REAL_EPS));
//	//context.write("norm=%f\n", norm);
//	c = golem::REAL_ZERO;
//	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
//		sampledPose->weight /= norm;
////		context.write("sample.weight = %f\n", sampledPose->weight);
//		golem::kahanSum(cdf, c, sampledPose->weight);
//		sampledPose->cdf = cdf;
//	}
////	context.write("spam::Belief::createUpdate(): done!\n");
//
//}
//
//golem::Real Belief::evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const grasp::RealSeq &forces, std::vector<bool> &triggered, bool intersect) {
//	Real distMin = /*myDesc.sensory.sensoryRange + */REAL_ONE;	
//	const Real norm = REAL_ONE - density(myDesc.sensory.sensoryRange); //(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
//	Mat34 actionFrame(sample.q, sample.p), queryFrame(Mat34(sample.q, sample.p) * modelFrame), queryFrameInv, fromQueryToJoint;
//
//	Vec3 boundFrame, surfaceFrame;
//	if (pose.p.distance(queryFrame.p) < distMin) {
//		const size_t size = modelPoints.size() < myDesc.maxSurfacePoints ? modelPoints.size() : myDesc.maxSurfacePoints;
//		for (size_t i = 0; i < size; ++i) {
//			// CHECK: Do you assume that a point has a full frame with **orientation**? Where does the orientation come from?
//
//			//const Point& point = size < modelPoints.size() ? modelPoints[size_t(rand.next())%
//			//	modelPoints.size()] : modelPoints[i];
//			//Mat34 pointFrame;
//			//pointFrame.multiply(actionFrame, point.frame);
//			const Vec3 point = grasp::Cloud::getPoint<Real>(size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]);
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
//	//return norm*density(distMin); // if uncomment here there is no notion of direction of contacts
//
//	// compute the direction of contact (from torques)
//	Vec3 v;
//	Mat34 jointFrame(Mat33(pose.q), pose.p);
//	Mat34 jointFrameInv;
//	jointFrameInv.setInverse(jointFrame);
//	jointFrameInv.multiply(v, queryFrame.p);
//	v.normalise();
//	//context.write(" -> dist(to shape)=%5.7f, bound <%f %f %f>, point <%f %f %f>, v <%f %f %f>, intersection %s, triggered <%s %s %s %s>, forces <%f %f %f %f>, <norm=%5.7f, density=%5.7f, prob of touching=%5.7f\n", distMin, 
//	//	boundFrame.x, boundFrame.y, boundFrame.z, surfaceFrame.x, surfaceFrame.y, surfaceFrame.z, v.x, v.y, v.z,
//	//	intersect ? "Y" : "N", 
//	//	triggered[0] ? "Y" : "N", triggered[1] ? "Y" : "N", triggered[2] ? "Y" : "N", triggered[3] ? "Y" : "N",
//	//	forces[0], forces[1], forces[2], forces[3],
//	//	norm, density(distMin), norm*density(distMin));
//
////		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
////		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
//
//	// The first joint of the finger responds to side movements on the 'y' axis
//	if (triggered[0]) 
//		//if ((v.y < 0 && forces[0] < 0) || (v.y > 0 && forces[0] > 0)) return REAL_ZERO; // justin robot
//		if ((v.x < 0 && forces[0] < 0) || (v.x > 0 && forces[0] > 0)) return REAL_ZERO; // bham robot
//	// The other joint respond to movements on the 'z' axis
//	else 
//		for (size_t i = 1; i < triggered.size(); ++i) 
//			//if ( triggered[i] && ((v.z < 0 && forces[i] < 0) || (v.z > 0 && forces[i] > 0))) return REAL_ZERO; // justin robot
//			if ( triggered[i] && ((v.z < 0 && forces[i] < 0) || (v.z > 0 && forces[i] > 0))) return REAL_ZERO; // bham robot
//		
//		//context.write("evaluation pose <%f %f %f> sample <%f %f %f> v.z=%f force=%f\n", 
//		//	pose.p.x, pose.p.y, pose.p.z, queryFrame.p.x, queryFrame.p.y, queryFrame.p.z, v.z, force);	
//	
//	// return the likelihood of observing such a contact for the current joint
//	return 1000*norm*density(distMin);
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

//bool Belief::intersect(const golem::Bounds::Seq &bounds, const golem::Mat34 &pose) const {
//	if (hypotheses.empty())
//		return false;
//	grasp::RBPose::Sample frame = (*hypotheses.begin())->toRBPoseSample();
//	Mat34 actionFrame(frame.q, frame.p);
//	grasp::Cloud::PointSeq points;
//	std::vector<float> distances;
//
//	if ((*hypotheses.begin())->nearestKPoints(grasp::RBCoord(pose), points, distances) > 0) {
//		//const Vec3 point = grasp::Cloud::getPoint(points[0]);
//		for (size_t i = 0; i < points.size(); ++i) {
//			const Vec3 point = grasp::Cloud::getPoint(points[i]);
//			//Mat34 pointFrame = actionFrame;
//			//pointFrame.p += point; // only position is updated
//			//context.debug("Joint at pose <%f %f %f>, point at pose <%f %f %f>\n", pose.p.x, pose.p.y, pose.p.z, pointFrame.p.x, pointFrame.p.y, pointFrame.p.z);
//			for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b) {
//				if ((*b)->intersect(point)) { 
////					context.debug("Joint at pose <%f %f %f> intersects point at pose <%f %f %f>\n", pose.p.x, pose.p.y, pose.p.z, point.x, point.y, point.z);
//					return true;
//				}
//			}
//		}
//		//context.debug("returns false\n--------------------------------------\n");
//	}
//
//	// quick check of intersection
////	if (pose.p.distance(actionFrame.p) < myDesc.distanceRange) {
////		const size_t size = modelPoints.size() < myDesc.maxSurfacePoints ? modelPoints.size() : myDesc.maxSurfacePoints;
////		for (size_t i = 0; i < size; ++i) {
////			const Vec3 point = grasp::Cloud::getPoint(size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]);
////			Mat34 pointFrame = actionFrame;
////			pointFrame.p += point; // only position is updated
//////			std::cout << "dist ";
////			for (golem::Bounds::Seq::const_iterator b = bounds.begin(); b != bounds.end(); ++b) {
////				const Real d = (*b)->getSurfaceDistance(pointFrame.p);
//////				std::cout <<  d << " ";
////				if (d < REAL_ZERO) { 
////					context.debug("Joint at pose <%f %f %f> intersects estimated object at pose <%f %f %f>\n", pose.p.x, pose.p.y, pose.p.z, actionFrame.p.x, actionFrame.p.y, actionFrame.p.z);
////					return true;
////				}
////			}
//////			std::cout << "\n";
////		}
////	}
//	return false;
//}

//void Belief::createUpdate(const grasp::Manipulator *manipulator, const grasp::Robot *robot, const golem::Waypoint &w, const FTGuard::Seq &triggeredGuards, const grasp::RealSeq &force) {
//	context.write("spam::Belief::createUpdate()...\n");
//	Real weight = golem::REAL_ONE;
//	bool intersect = false; 
//	// retrieve wrist's workspace pose and fingers configuration
//	//golem::Mat34 poses[grasp::Manipulator::JOINTS];
//	//manipulator->getPoses(manipulator->getPose(w.cpos), poses);
//	grasp::RealSeq forces(force);
//	for (Chainspace::Index i = robot->getStateHandInfo().getChains().begin(); i != robot->getStateHandInfo().getChains().end(); ++i) {
//		// check if any of the joint in the current chain (finger) has been triggered
//		bool triggered = false;
////		context.write("Chain %d\n", i);
//		for (Configspace::Index j = robot->getStateHandInfo().getJoints(i).begin(); j != robot->getStateHandInfo().getJoints(i).end(); ++j) { 
//			const size_t idx = j - robot->getStateHandInfo().getJoints().begin();
//			triggered = triggered || [&] () -> bool {
//				for (FTGuard::Seq::const_iterator i = triggeredGuards.begin(); i < triggeredGuards.end(); ++i)
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
//			golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO;
//			context.write("Updating poses' weights for chain=%d joint=%d(%d) (triggered %s)\n", i, j - robot->getStateHandInfo().getJoints(i).begin(),j, triggered ? "Y" : "N");
//			for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
//				const Real eval = triggered ? evaluate(bounds, grasp::RBCoord(w.wposex[j]), *sampledPose, forces[idx], j == robot->getStateHandInfo().getJoints(i).begin(), intersect) : REAL_ONE;
//				sampledPose->weight *= 
//				//const Real eval = evaluate(bounds, grasp::RBCoord(w.wposex[j]), *sampledPose, forces[idx], j == robot->getStateHandInfo().getJoints(i).begin(), intersect);
//				//sampledPose->weight *= (triggered ? myDesc.sensory.contactFac*eval : intersect ? myDesc.sensory.noContactFac*(REAL_ONE - eval) : REAL_ZERO);
//				//golem::kahanSum(norm, c, sampledPose->weight);
//			}
//			context.write("\n---------------------------------------------\n");
//		}
//	}
//
//	// normalise weights
//	golem::Real norm = golem::REAL_ZERO, c = golem::REAL_ZERO, cdf = golem::REAL_ZERO;
//	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose)
//		golem::kahanSum(norm, c, sampledPose->weight);
//	context.write("norm=%f\n", norm);
//	for (grasp::RBPose::Sample::Seq::iterator sampledPose = poses.begin(); sampledPose != poses.end(); ++sampledPose) {
//		sampledPose->weight /= norm;
//		context.write("sample.weight = %f\n", sampledPose->weight);
//		cdf += sampledPose->weight;
//		sampledPose->cdf = cdf;
//	}
//	context.write("spam::Belief::createUpdate(): done!\n");
//
//}
//
//golem::Real Belief::evaluate(const golem::Bounds::Seq &bounds, const grasp::RBCoord &pose, const grasp::RBPose::Sample &sample, const Real &force, bool jointZero, bool intersect) {
//	Real distMin = myDesc.sensory.sensoryRange + REAL_ONE;	
//	const Real norm = REAL_ONE - density(myDesc.sensory.sensoryRange); //(ftDrivenDesc.ftModelDesc.enabledLikelihood)?/*REAL_ONE/*/(REAL_ONE - density(ftDrivenDesc.ftModelDesc.distMax)):REAL_ONE;
//	Mat34 actionFrame(sample.q, sample.p), queryFrame(Mat34(sample.q, sample.p) * modelFrame), queryFrameInv, fromQueryToJoint;
//
//	Vec3 boundFrame, surfaceFrame;
//	if (pose.p.distance(queryFrame.p) < distMin) {
//		const size_t size = modelPoints.size() < myDesc.maxSurfacePoints ? modelPoints.size() : myDesc.maxSurfacePoints;
//		for (size_t i = 0; i < size; ++i) {
//			// CHECK: Do you assume that a point has a full frame with **orientation**? Where does the orientation come from?
//
//			//const Point& point = size < modelPoints.size() ? modelPoints[size_t(rand.next())%
//			//	modelPoints.size()] : modelPoints[i];
//			//Mat34 pointFrame;
//			//pointFrame.multiply(actionFrame, point.frame);
//			const Vec3 point = grasp::Cloud::getPoint(size < modelPoints.size() ? modelPoints[size_t(rand.next())%modelPoints.size()] : modelPoints[i]);
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
//	//return norm*density(distMin); // if uncomment here there is no notion of direction of contacts
//
//	// compute the direction of contact (from torques)
//	Vec3 v;
//	Mat34 jointFrame(Mat33(pose.q), pose.p);
//	Mat34 jointFrameInv;
//	jointFrameInv.setInverse(jointFrame);
//	jointFrameInv.multiply(v, queryFrame.p);
//	v.normalise();
//	context.write(" -> dist(to shape)=%5.7f, bound <%f %f %f>, point <%f %f %f>, v <%f %f %f>, intersection %s, first_joint %s, norm=%5.7f, density=%5.7f, prob of touching=%5.7f\n", distMin, 
//		boundFrame.x, boundFrame.y, boundFrame.z, surfaceFrame.x, surfaceFrame.y, surfaceFrame.z, v.x, v.y, v.z,
//		intersect ? "Y" : "N", jointZero ? "Y" : "N", norm, density(distMin), norm*density(distMin));
//
////		const Real thx(Math::atan2(v.z, v.x)), thy(Math::atan2(v.z, v.y));
////		if (v.z < 0 || Math::abs(thx) > ftDrivenDesc.ftModelDesc.coneTheta1 || Math::abs(thy) > ftDrivenDesc.ftModelDesc.coneTheta2) 
//
//	// The first joint of the finger responds to side movements on the 'y' axis
//	if (jointZero) 
//		if ((v.y < 0 && force < 0) || (v.y > 0 && force > 0)) return REAL_ZERO;
//	// The other joint respond to movements on the 'z' axis
//	else 
//		if ((v.z < 0 && force < 0) || (v.z > 0 && force > 0)) return REAL_ZERO;
//		//context.write("evaluation pose <%f %f %f> sample <%f %f %f> v.z=%f force=%f\n", 
//		//	pose.p.x, pose.p.y, pose.p.z, queryFrame.p.x, queryFrame.p.y, queryFrame.p.z, v.z, force);	
//	
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
