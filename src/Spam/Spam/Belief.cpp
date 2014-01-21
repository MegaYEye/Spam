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

#ifdef WIN32
	#pragma warning (push)
	#pragma warning (disable : 4291 4244 4996 4305)
#endif
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <flann/flann.hpp>
#ifdef WIN32
	#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

void spam::XMLData(Belief::Desc& val, golem::XMLContext* context, bool create) {
	context = context->getContextFirst("tactile_model");
	golem::XMLData("kernels", val.tactile.kernels, context, create);
	golem::XMLData("test", val.tactile.test, context, create);
	grasp::XMLData(val.tactile.stddev, context->getContextFirst("test_stddev"), create);
	golem::XMLData(&val.tactile.covariance[0], &val.tactile.covariance[grasp::RBCoord::N], "c", context->getContextFirst("covariance"), create);
}

//------------------------------------------------------------------------------

Mat34 Belief::RigidBodyTransformation::transform(Mat34 &p) {
	Mat34 poseInverse;
	poseInverse.setInverse(pose);
	pose.multiply(poseInverse, p);
	return pose;
}

//------------------------------------------------------------------------------

Belief::Belief(golem::Context& context) : grasp::RBPose(context) {
	pRBPose.reset((grasp::RBPose*)this);
}

bool Belief::create(const Desc& desc) {
	if (!desc.isValid())
		throw Message(Message::LEVEL_CRIT, "spam::RBPose::create(): Invalid description");
	
	grasp::RBPose::create((grasp::RBPose::Desc&)desc);
	
	myDesc = desc;
	mfsePoses.resize(desc.tactile.kernels);

	return true;
}

//------------------------------------------------------------------------------

void Belief::createQuery(const grasp::Point::Seq& points, golem::U32 label) {
	mfsePoses.clear();

	if (myDesc.tactile.test) {
		/* To save time TEST generates only one query and then estimates the max score fitting as many time as KERNELS */
		grasp::RBPose::createQuery(points, label);
		const grasp::RBPose::Sample maximum = grasp::RBPose::maximum();
		for (size_t i = 0; i < myDesc.tactile.kernels; ++i)
			mfsePoses.push_back(grasp::RBPose::Sample(sample(maximum)));
		// mean and covariance
		if (!mfseProperties.create<golem::Ref1, RBPose::Sample::Ref>(myDesc.tactile.covariance, mfsePoses))
			throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
		
		context.write("spam::Belief::createQuery(TEST): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", mfseProperties.covariance[0], mfseProperties.covariance[1], mfseProperties.covariance[2], mfseProperties.covariance[3], mfseProperties.covariance[4], mfseProperties.covariance[5], mfseProperties.covariance[6]);
		return;
	}

	/* Right way of proceeding. For each kernel a new query is computed
	*/
	SecTmReal init = context.getTimer().elapsed();
	for (size_t i = 0; i < myDesc.tactile.kernels; ++i) {
			grasp::RBPose::createQuery(points, label);
			grasp::RBPose::Sample s = grasp::RBPose::maximum();
			mfsePoses.push_back(s);
			initPoses.push_back(s);
		}
	std::printf("Belief::createQuery(): computational time %.7f\n", context.getTimer().elapsed() - init);

	//SecTmReal init = context.getTimer().elapsed();
	//CriticalSection cs;
	//size_t index = 0;
	//grasp::ParallelsTask((golem::Parallels*)context.getParallels(), [&] (grasp::ParallelsTask*) {
	//	for (size_t i = 0;;) {
	//		{
	//			CriticalSectionWrapper csw(cs);
	//			if (index == myDesc.tactile.kernels)
	//				break;
	//			i = index++;
	//		}	
	//		pRBPose->createQuery(points, label);
	//		grasp::RBPose::Sample s = pRBPose->maximum();
	//		mfsePoses.push_back(s);
	//		initPoses.push_back(s);
	//	}
	//}); // end parallel task
	//std::printf("Belief::createQuery(): computational time %.7f\n", context.getTimer().elapsed() - init);
	
	// mean and covariance
	if (!mfseProperties.create<golem::Ref1, RBPose::Sample::Ref>(myDesc.tactile.covariance, mfsePoses))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createQuery(): Unable to create mean and covariance for the high dimensional representation");
	// copy initial distribution properties
	initProperties = mfseProperties;
	
	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < myDesc.tactile.kernels; ++i)
		rand.nextGaussianArray<golem::Real>(&(mfsePoses[i])[0], &(mfsePoses[i])[0] + grasp::RBCoord::N, &((mfsePoses[i]))[0], &mfseProperties.covarianceSqrt[0]); // normalised multivariate Gaussian

	context.write("spam::Belief::createQuery(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", mfseProperties.covariance[0], mfseProperties.covariance[1], mfseProperties.covariance[2], mfseProperties.covariance[3], mfseProperties.covariance[4], mfseProperties.covariance[5], mfseProperties.covariance[6]);
}

void Belief::createUpdate(const Mat34 &trn) {
	// TODO
	if (false) {
	size_t i = 0;
	for (grasp::RBPose::Sample::Seq::iterator s = mfsePoses.begin(); s != mfsePoses.end(); ++s, ++i) {
		Mat34 sample = s->toMat34();
		sample.multiply(trn, sample);
		Quat q(sample.R);
		*s = Sample(grasp::RBCoord(sample.p, q), REAL_ONE, i*REAL_ONE);
	}
	}
}

void Belief::createResample() {
	size_t N = mfsePoses.size(), index = rand.nextUniform<size_t>(0, N);
	Real beta = golem::REAL_ZERO;
	grasp::RBPose::Sample::Seq newPoses;
	newPoses.reserve(N);
	for (size_t i = 0; i < N; ++i) {
		beta += rand.nextUniform<golem::Real>()*2*maxWeight();
//		context.debug("spam::RBPose::createResampling(): beta=%4.6f\n", beta);
		while (beta > mfsePoses[index].weight) {
			beta -= mfsePoses[index].weight;
			index = (index + 1) % N;
//			context.debug("beta=%4.6f\n, index=%d, weight=%4.6f\n", beta, index, mfsePoses[index].weight);
		}
		newPoses.push_back(mfsePoses.at(index));
	}

	// compute mean and covariance
	if (!mfseProperties.create<golem::Ref1, RBPose::Sample::Ref>(myDesc.tactile.covariance, newPoses))
		throw Message(Message::LEVEL_ERROR, "spam::RBPose::createResample(): Unable to create mean and covariance for the high dimensional representation");

	context.write("spam::Belief::createResample(): covariance mfse = {(%f, %f, %f), (%f, %f, %f, %f)}\n", mfseProperties.covariance[0], mfseProperties.covariance[1], mfseProperties.covariance[2], mfseProperties.covariance[3], mfseProperties.covariance[4], mfseProperties.covariance[5], mfseProperties.covariance[6]);
	
	// add noise to the resampled elements and overwrite poses
	mfsePoses.clear();
	mfsePoses.reserve(N);

	// generate new (noisy) samples out of selected subset of poses 
	for (size_t i = 0; i < N; ++i) {
		//mfsePoses.push_back(Sample(newPoses[i], REAL_ONE, i*REAL_ONE));
		//continue;
		grasp::RBCoord c;
		rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &(newPoses[i])[0], &mfseProperties.covarianceSqrt[0]); // normalised multivariate Gaussian
		mfsePoses.push_back(Sample(c, REAL_ONE, i*REAL_ONE));
	}
}

Real Belief::maxWeight() const {
	Real max = golem::REAL_ZERO;
	for (Sample::Seq::const_iterator s = mfsePoses.begin(); s != mfsePoses.end(); ++s)
		if (s->weight > max)
			max = s->weight;
	return max;
}

//------------------------------------------------------------------------------

grasp::RBCoord Belief::sample() const {
	grasp::RBPose::Sample::Seq::const_iterator ptr = Sample::sample<golem::Ref1, grasp::RBPose::Sample::Seq::const_iterator>(mfsePoses, rand);
	if (ptr == mfsePoses.end())
		throw Message(Message::LEVEL_ERROR, "RBPose::sample(): Sampling error");
	
	grasp::RBCoord c;
	this->rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &(*ptr)[0], &mfseProperties.covarianceSqrt[0]); // normalised multivariate Gaussian
	
	return c;
}

grasp::RBCoord Belief::sample(const grasp::RBCoord& kernel) const {
	grasp::RBCoord c;
	//Vec3 v;
	//v.next(rand); // |v|==1
	//v.multiply(Math::abs(rand.nextGaussian<Real>(REAL_ZERO, myDesc.tactile.stddev.lin)), v);
	//c.p.add(kernel.p, v);
	//Quat q;
	//q.next(rand, myDesc.tactile.stddev.ang);
	//c.q.multiply(kernel.q, q);
	rand.nextGaussianArray<golem::Real>(&c[0], &c[0] + grasp::RBCoord::N, &kernel[0], &myDesc.tactile.covariance[0]); // normalised multivariate Gaussian

	return c;
}

grasp::RBPose::Sample Belief::sampleHypothesis() {
	grasp::RBPose::Sample::Seq::const_iterator ptr = Sample::sample<golem::Ref1, grasp::RBPose::Sample::Seq::const_iterator>(mfsePoses, rand);
	if (ptr == mfsePoses.end())
		throw Message(Message::LEVEL_ERROR, "RBPose::sample(): Sampling error");
	
	return *ptr;
}

golem::Real Belief::density(const grasp::RBCoord &c) const {
	Real sum = REAL_ZERO;
	Real buf = REAL_ZERO;
	for (grasp::RBPose::Sample::Seq::const_iterator i = mfsePoses.begin(); i != mfsePoses.end(); ++i) {
		const Real dist = distance(c, *i);
		if (dist < desc.distanceRange)
			golem::kahanSum(sum, buf, i->weight*kernel(dist));
	}

	return sum;// up to scaling factor
}

grasp::RBPose::Sample Belief::maximum() {
	context.write("RBPose::maximum(): Computing maximum likelihood frame...\n");
	
	size_t k = 0;
	RBPose::Sample solution;
	Real solutionEval = REAL_MIN;
	CriticalSection cs;
	grasp::ParallelsTask(context.getParallels(), [&] (grasp::ParallelsTask*) {
		grasp::RBCoord test;
		Real testEval = REAL_MIN;
		for (;;) {
			{
				CriticalSectionWrapper csw(cs);
				if (solutionEval < testEval) {
					solutionEval = testEval;
					solution = test;
				}
				if (++k > desc.populationSize)
					break;
			}

			test = sample();
			for (size_t j = 0; j < desc.generationsMax; ++j) {
				// approximate affine combination of quaternions: renormalisation
				grasp::RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO)), meanBuf(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
				Real norm = REAL_ZERO, normBuf = REAL_ZERO;
				for (grasp::RBPose::Sample::Seq::const_iterator i = mfsePoses.begin(); i != mfsePoses.end(); ++i) {
					const Real d = distance(test, *i);
					if (d < desc.distanceRange) {
						const Real w = i->weight*kernel(d);
						golem::kahanSum(norm, normBuf, w);
						for (size_t l = 0; l < grasp::RBCoord::N; ++l)
							golem::kahanSum(mean[l], meanBuf[l], w*(*i)[l]);
					}
				}
				if (norm <= REAL_ZERO)
					break;
				const Real normInv = REAL_ONE/norm;
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
	context.write("Objective value: %f\n", solutionEval);
//	context.write("Belief::maximum(): <%.4f %.4f %.4f %.4f %.4f %.4f %.4f>\n", solution.p.x, solution.p.y, solution.p.z, solution.q.w, solution.q.x, solution.q.y, solution.q.z);
	return solution;
}

