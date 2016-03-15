/** @file Player.cpp
*
* @author	Claudio Zito
*
* @version 1.0
*
*/

#include <Spam/Demo/TactileExploration/Player.h>
#include <Grasp/Data/Image/Image.h>
#include <Grasp/Data/PointsCurv/PointsCurv.h>

//------------------------------------------------------------------------------

using namespace golem;
using namespace grasp;
using namespace spam;

//------------------------------------------------------------------------------

grasp::data::Data::Ptr spam::RRTPlayer::Data::Desc::create(golem::Context &context) const {
	grasp::data::Data::Ptr data(new RRTPlayer::Data(context));
	static_cast<RRTPlayer::Data*>(data.get())->create(*this);
	return data;
}

spam::RRTPlayer::Data::Data(golem::Context &context) : R2GPlanner::Data(context), owner(nullptr) {
}

void spam::RRTPlayer::Data::create(const Desc& desc) {
	R2GPlanner::Data::create(desc);
}

void spam::RRTPlayer::Data::setOwner(grasp::Manager* owner) {
	R2GPlanner::Data::setOwner(owner);
	this->owner = grasp::is<RRTPlayer>(owner);
	if (!this->owner)
		throw Message(Message::LEVEL_CRIT, "RRTPlayer::Data::setOwner(): unknown data owner");
}

void spam::RRTPlayer::Data::createRender() {
	R2GPlanner::Data::createRender();
	{
		golem::CriticalSectionWrapper csw(owner->getCS());
		owner->cdebugRenderer.reset();
		//Vec3 frameSize(1, 1, 1);
		//for (auto i = owner->cloudPoints.begin(); i != owner->cloudPoints.end(); ++i) {
		//	Mat34 point; point.setId(); Cloud::setPoint<Real>(point.p, *i);
		//	point.p.x = i->x; point.p.y = i->y; point.p.z = i->z;
		//	owner->cdebugRenderer.addAxes(point, frameSize);
		//}
		//owner->pointAppearance.drawPoints(owner->cloudPoints, owner->cdebugRenderer);
	}
}

void spam::RRTPlayer::Data::load(const std::string& prefix, const golem::XMLContext* xmlcontext, const grasp::data::Handler::Map& handlerMap) {
	grasp::data::Data::load(prefix, xmlcontext, handlerMap);

}

void spam::RRTPlayer::Data::save(const std::string& prefix, golem::XMLContext* xmlcontext) const {
	grasp::data::Data::save(prefix, xmlcontext);

}

//------------------------------------------------------------------------------

void RRTPlayer::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	R2GPlanner::Desc::load(context, xmlcontext);

	try {
		xmlcontext = xmlcontext->getContextFirst("demo");
		XMLData(pRRTDesc->initialState, xmlcontext->getContextFirst("initial_state"), false);
		XMLData(pRRTDesc->goalState, xmlcontext->getContextFirst("goal_state"), false);
	}
	catch (const golem::MsgXMLParser& msg) { context.write("%s\n", msg.str().c_str()); }
}

//------------------------------------------------------------------------------

RRTPlayer::RRTPlayer(Scene &scene) : R2GPlanner(scene) {
}

RRTPlayer::~RRTPlayer() {
}

void RRTPlayer::create(const Desc& desc) {
	desc.assertValid(Assert::Context("RRTPlayer::Desc."));

	// create object
	R2GPlanner::create(desc); // throws

	// create planner
	pPlanner = desc.pRRTDesc->create(context);

	cloudPoints.clear();
	pointAppearance.setToDefault();

	menuCmdMap.insert(std::make_pair("Z", [=]() {
		context.write("RRTPlayer test\n");

		pPlanner->plan();

		context.write("RRTPlayer test: Done.\n");
	}));
	menuCmdMap.insert(std::make_pair("V", [=]() {
		context.write("RRT test\n");

		Tree3d *t = new Tree3d(golem::Vec3(.1, .1, .0));
		context.write("t: nodes=%d [%d]\n", t->getNodesSize(), t->getNodesSize());
		t->print();
		t->extend(*t->getNodes().begin(), golem::Vec3(.11, .2, .0));
		context.write("t: nodes=%d [%d]\n", t->getNodesSize(), t->getNodesSize());
		t->print();
	}));

	menuCmdMap.insert(std::make_pair("K", [=]() {
		context.write("Point cloud test\n");

		const std::string itemName = "jug-curv";
		grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(itemName);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", itemName.c_str());

		// retrive point cloud with curvature
		grasp::data::ItemPointsCurv *pointsCurv = is<grasp::data::ItemPointsCurv>(ptr->second.get());
		if (!pointsCurv)
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not support PointdCurv interface.");
		grasp::Cloud::PointCurvSeq curvPoints = *pointsCurv->cloud;

		// copy as a vector of points in 3D
		const bool draw = true;
		Vec3 modelFrame = Vec3::zero();
		const size_t Nsur = 300, Next = 2, Nint = 1, Ncurv = curvPoints.size();
		const size_t Ntot = Nsur + Next + Nint;
		Vec3Seq cloud, surCloud, points, pclNormals;
		cloud.resize(Ntot);
		surCloud.resize(Nsur);
		points.resize(Ncurv);
		pclNormals.resize(Ncurv);
		for (size_t i = 0; i < Ncurv; ++i) {
			points[i] = Cloud::getPoint<Real>(curvPoints[i]);
			pclNormals[i] = Cloud::getNormal<Real>(curvPoints[i]);
		}
		//		if (draw) renderCloud(points, RGBA::YELLOW);

		Vec3Seq extCloud;
		extCloud.resize(Next);
		RealSeq targets;
		targets.resize(Ntot);

		context.write("Load %d points from model point cloud (%d)\n", Nsur, Ncurv);
		for (size_t i = 0; i < Nsur; ++i) {
			const size_t index = size_t(rand.next() % Ncurv);
			surCloud[i] = cloud[i] = Cloud::getPoint<Real>(curvPoints[index]);
			targets[i] = REAL_ZERO;
			modelFrame += cloud[i];
		}
		// render model point cloud
		if (draw) renderCloud(surCloud, RGBA::YELLOW);
		modelFrame /= Nsur;
		if (draw) {
			Mat34 frame; frame.setId();
			frame.p = modelFrame;
			cdebugRenderer.addAxes(frame, Vec3(0.02, 0.02, 0.02));
		}
		// max radious
		Real length = REAL_ZERO;
		//		context.write("Model frame [%f %f %f] d=%f\n", modelFrame.x, modelFrame.y, modelFrame.z);
		//for (size_t i = 0; i < cloud.size(); ++i) {
		//	for (size_t j = i + 1; j < cloud.size(); ++j) {
		//		const Real d = cloud[i].distance(cloud[j]);
		//		//			context.write("point[%d] [%f %f %f] d=%f\n", i, cloud[i].x, cloud[i].y, cloud[i].z, d);
		//		if (length < d)
		//			length = d;
		//	}
		//}
		for (size_t i = 0; i < Nsur; ++i) {
			const Real d = modelFrame.distance(cloud[i]);
			if (length < d)
				length = d;
		}
		// zero-mean points
		for (size_t i = 0; i < Nsur; ++i)
			cloud[i] -= modelFrame;

		length = 1.2 * length;
		const Real lengthSqr = length * length;
		context.write("Create %d external points as a sphere of radius %f\n", Next, length);
		for (size_t i = 0; i < Next; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, length) - length) * Math::cos(colatitude);
			Vec3 point(Math::sin(theta)*sqrt(lengthSqr - z*z), Math::cos(theta)*Math::sqrt(lengthSqr - z*z), z);

			//const size_t index = size_t(rand.next() % Ncurv);
			//const Vec3 v = points[index] - modelFrame;
			//const Vec3 p = points[index] + v;
			cloud[Nsur + i] = point;
			extCloud[i] = point + modelFrame;
			targets[Nsur + i] = REAL_ONE;
		}
		if (draw) renderCloud(extCloud, RGBA::RED);

		cloud[Nsur + Next] = Vec3::zero();//modelFrame;
		targets[Nsur + Next] = -REAL_ONE;

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets));
#define	simplethinplate
//#define simple_cov_se
#ifdef simple_cov_se
		SimpleGaussianRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > Ntot ? covDesc.initialLSize : Ntot + 100;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.length = 1.0;
		covDesc.covTypeDesc.sigma = 1.0;
		covDesc.covTypeDesc.noise = 0.01;
		covDesc.optimise = true;
		SimpleGaussianRegressor::Ptr gp = covDesc.create(context);
		context.write("%s Regressor created. inputDim=%d, parmDim=%d, l=%f, s=%f\n", gp->getName().c_str(), covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.sigma);
#endif
#ifdef simplethinplate
		SimpleThinPlateRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > Ntot ? covDesc.initialLSize : Ntot + covDesc.initialLSize;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.noise = 0.0; //0.01;
		Real l = REAL_ZERO;
		for (size_t i = 0; i < cloud.size(); ++i) {
			for (size_t j = 0; j < i; ++j) {
				const Real d = cloud[i].distance(cloud[j]);
				if (l < d) l = d;
			}
		}
		covDesc.covTypeDesc.length = l + 0.01;
		covDesc.optimise = false;
		context.write("ThinPlate cov: inputDim=%d, parmDim=%d, R=%f noise=%f\n", covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.noise);
		SimpleThinPlateRegressor::Ptr gp = covDesc.create(context);
#endif
		gp->set(trainingData);

		/*****  Query the model with a point  *********************************/
		golem::Vec3 q(cloud[0]);
		const double qf = gp->f(q);
		const double qVar = gp->var(q);

		context.write("Test estimate first point in the model\ny = %f -> qf = %f qVar = %f\n\n", targets[0], qf, qVar);
		if (!isfinite(qf) || !isfinite(qVar))
			return;
		
		/*****  Evaluate points and normals  *********************************************/
		const size_t testSize = 50;
		context.write("Evaluate %d points and normals\n", testSize);
		std::vector<Real> fx, varx;
		Eigen::MatrixXd normals, tx, ty, V;
		Vec3Seq nn, xStar, normStar, xStarPoints;
		nn.resize(testSize);
		normStar.resize(testSize);
		RealSeq yStar;
		xStar.resize(testSize); yStar.assign(testSize, REAL_ZERO);
		xStarPoints.resize(testSize);
		for (size_t i = 0; i < testSize; ++i) {
			const size_t index = size_t(rand.next() % Ncurv);
			xStarPoints[i] = Cloud::getPoint<Real>(curvPoints[index]);
			xStar[i] = xStarPoints[i] - modelFrame;
			normStar[i] = Cloud::getNormal<Real>(curvPoints[index]);
			normStar[i].normalise();
			yStar[i] = REAL_ZERO;
		}
		Real evalError = REAL_ZERO, normError = REAL_ZERO, c = REAL_ZERO, c1 = REAL_ZERO;
		gp->evaluate(xStar, fx, varx, normals, tx, ty, V);
		for (size_t i = 0; i < testSize; ++i) {
			//points[i] = x_star[i];
			nn[i] = Vec3(normals(i, 0), normals(i, 1), normals(i, 2));
			golem::kahanSum(normError, c, (1 - ::abs(nn[i].dot(normStar[i]))) * 90);
			golem::kahanSum(evalError, c1, std::pow(fx[i] - yStar[i], 2));
			context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f normal=[%f %f %f] error=%f [deg]\n", i, yStar[i], fx[i], varx[i], nn[i].x, nn[i].y, nn[i].z, (1 - ::abs(nn[i].dot(normStar[i]))) * 90);
		}
		context.write("Evaluate(): error=%f avg=%f normError=%f avg=%f\n\n", evalError, evalError / testSize, normError, normError / testSize);
		// render normals
		if (draw) renderCromCloud(xStarPoints, varx);
		if (draw) renderNormals(xStarPoints, nn);
		if (draw) renderNormals(xStarPoints, normStar, RGBA::BLUE);
		return; 

		/*****  Re-construct the object  *********************************************/
		const Real deltaStep = length / 10;
		Vec3Seq xStar2; xStar2.resize(10000);
		size_t count = 0, jj = 0;
		for (Real i = -length; i < length; i = i + deltaStep) {
			for (Real j = -length; j < length; j = j + deltaStep) {
				for (Real z = -length; z < length; z = z + deltaStep) {
					if (count >= 10000)
						break;
					xStar2[count++] = Vec3(i, j, z);
				}
			}
		}
		gp->evaluate(xStar2, fx, varx, normals, tx, ty, V);
		count = 0; jj = 0;
		nn.clear(); nn.resize(xStar2.size());
		RealSeq varxx; varxx.resize(xStar2.size());
		Vec3Seq object; object.resize(xStar2.size());
		for (size_t i = 0; i < xStar2.size(); ++i) {
			if (::abs(fx[i]) < 0.02) {
				object[count] = modelFrame + xStar2[i];
				varxx[count] = varx[i];
				nn[count++] = Vec3(normals(i, 0), normals(i, 1), normals(i, 2));
			}
		}
		to<Data>(dataCurrentPtr)->createRender();
		renderCromCloud(object, varx);
		renderNormals(object, nn);
		//renderCloud(object, RGBA::BLACK);

		/*****  done  *********************************************/
		context.write("Done.\n");
	}));

	/****************************************************/
	/**													*/
	/**		TEST GUASSIAN PROCESS with DERIVATIVE		*/
	/**													*/
	/****************************************************/
	menuCmdMap.insert(std::make_pair("F", [=]() {
		context.write("Point cloud test\n");

		const std::string itemName = "jug-curv"; //"model-curv-6_views"; //
		grasp::data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.find(itemName);
		if (ptr == to<Data>(dataCurrentPtr)->itemMap.end())
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not find %s.", itemName.c_str());

		// retrive point cloud with curvature
		grasp::data::ItemPointsCurv *pointsCurv = is<grasp::data::ItemPointsCurv>(ptr->second.get());
		if (!pointsCurv)
			throw Message(Message::LEVEL_ERROR, "PosePlanner::estimatePose(): Does not support PointdCurv interface.");
		grasp::Cloud::PointCurvSeq curvPoints = *pointsCurv->cloud;

		// copy as a vector of points in 3D
		const bool draw = true;
		Vec3 modelFrame = Vec3::zero();
		const size_t Nsur = 200, Next = 0, Nint = 0, Ncurv =  curvPoints.size();
		const size_t Ntot = Nsur + Next + Nint;
		const Real extRadius = 1.2, intRho = 0.001;
		Vec3Seq cloud, surCloud, surNormals, points, pclNormals, nns;
		cloud.resize(Ntot);
		surNormals.resize(Nsur);
		nns.resize(Ntot);
		surCloud.resize(Nsur);
		points.resize(Ncurv);
		pclNormals.resize(Ncurv);
		for (size_t i = 0; i < Ncurv; ++i) {
			points[i] = Cloud::getPoint<Real>(curvPoints[i]);
			pclNormals[i] = Cloud::getNormal<Real>(curvPoints[i]);
		}
//		if (draw) renderCloud(points, RGBA::YELLOW);

		Vec3Seq extCloud, extNormals;
		extCloud.resize(Next);
		extNormals.resize(Next);
		RealSeq targets;
		targets.resize(Ntot);

		context.write("Load %d points from model point cloud (%d)\n", Nsur, Ncurv);
		for (size_t i = 0; i < Nsur; ++i) {
			const size_t index = size_t(rand.next() % Ncurv);
			surCloud[i] = cloud[i] = Cloud::getPoint<Real>(curvPoints[index]);
			nns[i] = surNormals[i] = pclNormals[index];
			nns[i].normalise(); surNormals[i].normalise();
			targets[i] = REAL_ZERO;
			modelFrame += cloud[i];
		}
		// render model point cloud
		if (draw) {
			renderCloud(surCloud, RGBA::YELLOW);
			renderNormals(surCloud, surNormals);
		}
		modelFrame /= Nsur;
		if (draw) {
			Mat34 frame; frame.setId();
			frame.p = modelFrame;
			cdebugRenderer.addAxes(frame, Vec3(0.02, 0.02, 0.02));
		}
		// max radious
		Real length = REAL_ZERO;
		//		context.write("Model frame [%f %f %f] d=%f\n", modelFrame.x, modelFrame.y, modelFrame.z);
		//for (size_t i = 0; i < cloud.size(); ++i) {
		//	for (size_t j = i + 1; j < cloud.size(); ++j) {
		//		const Real d = cloud[i].distance(cloud[j]);
		//		//			context.write("point[%d] [%f %f %f] d=%f\n", i, cloud[i].x, cloud[i].y, cloud[i].z, d);
		//		if (length < d)
		//			length = d;
		//	}
		//}
		for (size_t i = 0; i < Nsur; ++i) {
			const Real d = modelFrame.distance(cloud[i]);
			if (length < d)
				length = d;
		}
		// zero-mean points
		for (size_t i = 0; i < Nsur; ++i)
			cloud[i] -= modelFrame;

		length = extRadius * length;
		const Real lengthSqr = length * length;
		context.write("Create %d external points as a sphere of radius %f\n", Next, length);
		for (size_t i = 0; i < Next; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, length) - length) * Math::cos(colatitude);
			Vec3 point(Math::sin(theta)*sqrt(lengthSqr - z*z), Math::cos(theta)*Math::sqrt(lengthSqr - z*z), z);

			//const size_t index = size_t(rand.next() % Ncurv);
			//const Vec3 v = points[index] - modelFrame;
			//const Vec3 p = points[index] + v;
			cloud[Nsur + i] = nns[Nsur + i] = extNormals[i] = point;
			extNormals[i].normalise(); nns[Nsur + i].normalise();
			extCloud[i] = point + modelFrame;
			targets[Nsur + i] = REAL_ONE;
		}
		if (draw) {
			renderCloud(extCloud, RGBA::RED);
			renderNormals(extCloud, extNormals);
		}

		context.write("\nInternal point(s) %lu:\n", Nint);
		const Real intRhoSqr = intRho * intRho;
		for (size_t i = Nsur + Next; i < Nsur + Next + Nint; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, intRho) - intRho) * Math::cos(colatitude);
			golem::Vec3 point(Math::sin(theta)*sqrt(intRhoSqr - z*z), Math::cos(theta)*Math::sqrt(intRhoSqr - z*z), z);
			cloud[i] = nns[i] = point;
			nns[i].normalise();

			targets[i] = -REAL_ONE;
		}

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets, nns));
//#define cov_se
#define cov_se_sc
//#define cov_se_ard
//#define covf_thinplate
#ifdef cov_se
		GaussianRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > 4 * Ntot ? covDesc.initialLSize : 4 * Ntot + covDesc.initialLSize;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.length = 0.0; // 0.03;
		covDesc.covTypeDesc.sigma = 0.0; // 0.015;
		covDesc.covTypeDesc.noise = 0.005;
		covDesc.optimisationDescPtr->delta0 = 0.01;
		covDesc.optimisationDescPtr->deltaMax = 2;
		covDesc.optimisationDescPtr->deltaMin = 1e-6;
		covDesc.optimisationDescPtr->epsStop = 1e-6;
		covDesc.optimisationDescPtr->etaMinus = 0.5;
		covDesc.optimisationDescPtr->etaPlus = 1.2;
		covDesc.optimisationDescPtr->maxIter = 150;
		RealSeq limit; limit.assign(covDesc.covTypeDesc.inputDim + 1, 2.0);
		//limit[covDesc.covTypeDesc.inputDim] = 1.5;
		covDesc.optimisationDescPtr->limits = limit;
		covDesc.optimise = true;
		covDesc.debug = false;
		GaussianRegressor::Ptr gp = covDesc.create(context);
		context.write("%s Regressor created. inputDim=%d, parmDim=%d, l=%f, s=%f\n", gp->getName().c_str(), covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.sigma);
#endif
#ifdef cov_se_sc
		GaussianSESCRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > 4 * Ntot ? covDesc.initialLSize : 4 * Ntot + covDesc.initialLSize;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.length = 0.5; // 0.03;
		covDesc.covTypeDesc.sigma = 0.5; // 0.015;
		covDesc.covTypeDesc.noise = 0.005;
		covDesc.optimisationDescPtr->delta0 = 0.01;
		covDesc.optimisationDescPtr->deltaMax = 2;
		covDesc.optimisationDescPtr->deltaMin = 1e-6;
		covDesc.optimisationDescPtr->epsStop = 1e-6;
		covDesc.optimisationDescPtr->etaMinus = 0.5;
		covDesc.optimisationDescPtr->etaPlus = 1.2;
		covDesc.optimisationDescPtr->maxIter = 150;
		RealSeq limit; limit.assign(covDesc.covTypeDesc.inputDim + 1, 2.0);
		//limit[covDesc.covTypeDesc.inputDim] = 1.5;
		covDesc.optimisationDescPtr->limits = limit;
		covDesc.optimise = false;
		covDesc.debug = false;
		GaussianSESCRegressor::Ptr gp = covDesc.create(context);
		context.write("%s Regressor created. inputDim=%d, parmDim=%d, l=%f, s=%f\n", gp->getName().c_str(), covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.sigma);
#endif

#ifdef cov_se_ard
		GaussianARDRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > 4 * Ntot ? covDesc.initialLSize : 4 * Ntot + covDesc.initialLSize;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.length = 0.5; // 0.03;
		covDesc.covTypeDesc.sigma = 0.5; // 0.015;
		covDesc.covTypeDesc.noise = 0.005;
		covDesc.optimisationDescPtr->delta0 = 0.01;
		covDesc.optimisationDescPtr->deltaMax = 2;
		covDesc.optimisationDescPtr->deltaMin = 1e-6;
		covDesc.optimisationDescPtr->epsStop = 1e-6;
		covDesc.optimisationDescPtr->etaMinus = 0.5;
		covDesc.optimisationDescPtr->etaPlus = 1.2;
		covDesc.optimisationDescPtr->maxIter = 150;
		RealSeq limit; limit.assign(covDesc.covTypeDesc.inputDim + 1, 2.0);
		limit[covDesc.covTypeDesc.inputDim] = 1.5;
		covDesc.optimisationDescPtr->limits = limit;
		covDesc.optimise = true;
		covDesc.debug = false;
		GaussianARDRegressor::Ptr gp = covDesc.create(context);
		context.write("%s Regressor created. inputDim=%d, parmDim=%d, l=%f, s=%f\n", gp->getName().c_str(), covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.sigma);
#endif
#ifdef covf_thinplate
		ThinPlateRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > 4 * Ntot ? covDesc.initialLSize : 4 * Ntot + covDesc.initialLSize;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.noise = 0.01;
		Real l = REAL_ZERO;
		for (size_t i = 0; i < cloud.size(); ++i) {
			for (size_t j = i; j < cloud.size(); ++j) {
				const Real d = cloud[i].distance(cloud[j]);
				if (l < d) l = d;
			}
		}
		covDesc.covTypeDesc.length = l;
		covDesc.optimise = false;
		covDesc.debug = false;
		context.write("ThinPlate cov: inputDim=%d, parmDim=%d, R=%f noise=%f\n", covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.noise);
		ThinPlateRegressor::Ptr gp = covDesc.create(context);
#endif
		gp->set(trainingData);

		/*****  Query the model with a point  *********************************/
		const size_t index = size_t(rand.next() % Ncurv);
		golem::Vec3 cc = Cloud::getPoint<Real>(curvPoints[index]);
		golem::Vec3 q(cloud[0]);
		golem::Vec3 qn(nns[0]);
		const Eigen::Vector4d vv = gp->f(q);
		const double qf = vv(0);
		const golem::Vec3 qfn(vv(1), vv(2), vv(3));
		const double qVar = gp->var(q);
//		const double qVar = gp->var(q)(0);
		const double er = nns[0].distance(qfn);
		context.write("Test estimate first point in the model\ny = %f -> qf = %f qVar = %f error=%f\n\n", targets[0], qf, qVar, er);
		if (!::isfinite(qf) || !::isfinite(qVar))
			return;
		
		/*****  Evaluate points and normals  *********************************************/
		const size_t testSize = 50;
		context.write("Evaluate %d points and normals\n", testSize);
		RealSeq fx, varx;
		fx.resize(testSize); varx.resize(testSize);
		Vec3Seq nn, xStar, normStar, xStarPoints; 
		nn.resize(testSize);
		normStar.resize(testSize);
		RealSeq yStar;
		xStar.resize(testSize); yStar.assign(testSize, REAL_ZERO);
		xStarPoints.resize(testSize);
		for (size_t i = 0; i < testSize; ++i) {
			const size_t index = size_t(rand.next() % Ncurv);
			xStarPoints[i] = Cloud::getPoint<Real>(curvPoints[index]);
			xStar[i] = xStarPoints[i] - modelFrame;
			normStar[i] = Cloud::getNormal<Real>(curvPoints[index]);
			normStar[i].normalise();
			yStar[i] = REAL_ZERO;
		}
		Real evalError = REAL_ZERO, normError = REAL_ZERO, c = REAL_ZERO, c1 = REAL_ZERO;
		for (size_t i = 0; i < testSize; ++i) {
			//points[i] = x_star[i];
			const Eigen::Vector4d res = gp->f(q);
			fx[i] = res(0);
			nn[i] = Vec3(res(1), res(2), res(3));
			nn[i].normalise();
			const double er = (1 - abs(nn[i].dot(normStar[i]))) * 90;
			golem::kahanSum(normError, c, er);
			golem::kahanSum(evalError, c1, std::pow(fx[i] - yStar[i], 2));
			context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f normal=[%f %f %f] norm_error=%f (deg)\n", i, yStar[i], fx[i], varx[i], nn[i].x, nn[i].y, nn[i].z, er);
		}
		context.write("Evaluate(): error=%f avg=%f normError=%f avg=%f\n\n", evalError, evalError / testSize, normError, normError / testSize);
		// render normals
		if (draw) renderCromCloud(xStarPoints, varx);
		if (draw) renderNormals(xStarPoints, nn);
		if (draw) renderNormals(xStarPoints, normStar, RGBA::BLUE);

		/*****  done  *********************************************/
		context.write("\nDone.\n");
	}));

	menuCmdMap.insert(std::make_pair("U", [=]() {
		/*****  Global variables  ******************************************/
		size_t N_sur = 150, N_ext = 20, N_int = 1, N_tot = N_sur + N_ext + N_int;
		const Real surRho = 0.02, extRho = 0.02, intRho = 0.001;
		const Real surRhoSqr = Math::sqr(surRho), extRhoSqr = Math::sqr(extRho), intRhoSqr = Math::sqr(intRho);
		golem::Real noise = 0.0;// 0.001; //

		const bool prtInit = false;
		const bool prtPreds = true;
		const bool draw = true;

		/*****  Generate Input data  ******************************************/
		Vec3Seq cloud, surCloud, normals; cloud.resize(N_tot); surCloud.resize(N_sur), normals.resize(N_tot);
		RealSeq targets; targets.resize(N_tot);
		golem::Vec3 frameSize(.01, .01, .01);
		context.write("Generate input data %lu\n", N_sur + N_ext + N_int);
		if (prtInit)
			context.write("Cloud points %lu:\n", N_sur);
		size_t i = 0;
		for (; i < N_sur; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, surRho) - surRho) * Math::cos(colatitude);
			Vec3 point(Math::sin(theta)*sqrt(surRhoSqr - z*z), Math::cos(theta)*Math::sqrt(surRhoSqr - z*z), z);
			point += Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud[i] = normals[i] = point;
			normals[i].normalise();
			// draws
			surCloud[i] = point;

			double y = 0.0;// point.magnitudeSqr() - surRhoSqr;
			targets[i] = y;
			if (prtInit)
				context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
		}


		// reset render
		if (draw) to<Data>(dataCurrentPtr)->createRender();
		// render model point cloud
		if (draw) renderCloud(surCloud, RGBA::GREEN);
		if (prtInit)
			context.write("\nExternal points %lu:\n", N_ext);
		Vec3Seq extCloud; extCloud.resize(N_tot);
		for (; i < N_sur + N_ext; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, extRho) - extRho) * Math::cos(colatitude);
			golem::Vec3 point(Math::sin(theta)*sqrt(extRhoSqr - z*z), Math::cos(theta)*Math::sqrt(extRhoSqr - z*z), z);
			point += golem::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud[i] = normals[i] = point;
			normals[i].normalise();
			extCloud[i] = point;

			double y = 1.0;// point.magnitudeSqr() - surRhoSqr;
			targets[i] = y;
			if (prtInit)
				context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
		}
		// render model point cloud
		if (draw) renderCloud(extCloud, RGBA::BLACK);

		if (prtInit)
			context.write("\nInternal point(s) %lu:\n", N_int);
		for (; i < N_sur + N_ext + N_int; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, intRho) - intRho) * Math::cos(colatitude);
			golem::Vec3 point(Math::sin(theta)*sqrt(intRhoSqr - z*z), Math::cos(theta)*Math::sqrt(intRhoSqr - z*z), z);
			point += golem::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud[i] = normals[i] = point;
			normals[i].normalise();

			double y = -1.0;// point.magnitudeSqr() - surRhoSqr;
			targets[i] = y;
			if (prtInit)
				context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
		}

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets));
#define covu_thin_plate //gaussianReg
//#define covu_se
#ifdef covu_se
		SimpleGaussianRegressor::Desc covDesc;
		covDesc.initialLSize = covDesc.initialLSize > N_tot ? covDesc.initialLSize : N_tot + 100;
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.length = 0.03;
		covDesc.covTypeDesc.sigma = 0.015;
		covDesc.covTypeDesc.noise = noise;// 0.0001;
		covDesc.optimise = false;
		SimpleGaussianRegressor::Ptr gp = covDesc.create(context);
		context.write("%s Regressor created. inputDim=%d, parmDim=%d, l=%f, s=%f noise=%f\n", gp->getName().c_str(), covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.sigma, covDesc.covTypeDesc.noise);
#endif
#ifdef covu_thin_plate
		SimpleThinPlateRegressor::Desc covDesc;
		//			covDesc.initialLSize = trainingData->rows();
		covDesc.covTypeDesc.inputDim = trainingData->cols();
		covDesc.covTypeDesc.noise = 0.0;
		Real l = REAL_ZERO;
		for (size_t i = 0; i < cloud.size(); ++i) {
			for (size_t j = i; j < cloud.size(); ++j) {
				const Real d = cloud[i].distance(cloud[j]);
				if (l < d) l = d;
			}
		}
		covDesc.covTypeDesc.length = 1.0;
		covDesc.optimise = false;
		context.write("ThinPlate cov: inputDim=%d, parmDim=%d, R=%f noise=%f\n", covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
			covDesc.covTypeDesc.length, covDesc.covTypeDesc.noise);
		SimpleThinPlateRegressor::Ptr gp = covDesc.create(context);
		context.write("%s Regressor created\n", gp->getName().c_str());
#endif
		gp->set(trainingData);
		context.write("Training dataset X[%d %d] Y[%d]\n", trainingData->rows(), trainingData->cols(), trainingData->y().size());

		/*****  Query the model with a point  *********************************/
		golem::Vec3 q(cloud[0]);
		const double qf = gp->f(q);
		const double qVar = gp->var(q);
		q.normalise();
		const double er = normals[0].distance(q);

		if (prtPreds)
			context.write("Test estimate first point in the model\ny = %f -> qf = %f qVar = %f error=%f\n\n", targets[0], qf, qVar, er);

		/*****  Query the model with new points  *********************************/
		const size_t testSize = 50;
		context.write("Query model with new %d points\n", testSize);
		Vec3Seq x_star, n_star, nx_star; x_star.resize(testSize); n_star.resize(testSize); nx_star.resize(testSize);
		RealSeq vars;
		RealSeq y_star; y_star.resize(testSize);
		Real predError = REAL_ZERO;
		for (size_t i = 0; i < testSize; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, surRho) - surRho) * Math::cos(colatitude);
			Vec3 point(Math::sin(theta)*sqrt(surRhoSqr - z*z), Math::cos(theta)*Math::sqrt(surRhoSqr - z*z), z);
			point += Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);

			// save the point for later
			x_star[i] = n_star[i] = point;
			n_star[i].normalise();
			//			points[i] = point;
			// compute the y value
			double y = point.magnitudeSqr() - surRhoSqr;
			// save the y value for later
			y_star[i] = y;
		}
		if (prtPreds)
			context.write("Prediction(): error=%f avg=%f\n\n", predError, predError / testSize);
		// render estimated points
		if (draw) renderCromCloud(x_star, vars);

		/*****  Evaluate points and normals  *********************************************/
		context.write("Evaluate %d points and normals\n", testSize);
		std::vector<Real> fx, varx;
		Eigen::MatrixXd normals2, tx, ty, V;
		Vec3Seq nn; nn.resize(testSize);
//		points.clear(); points.resize(testSize);
		Real evalError = REAL_ZERO, evalNError = REAL_ZERO;
		gp->evaluate(x_star, fx, varx, normals2, tx, ty, V);
		for (size_t i = 0; i < testSize; ++i) {
			//points[i] = x_star[i];
			fx[i] = gp->f(x_star[i]);
			varx[i] = gp->var(x_star[i]);
			nn[i] = Vec3(normals2(i, 0), normals2(i, 1), normals2(i, 2));
			Vec3 np = x_star[i];
			np.normalise();
			evalError += std::pow(fx[i] - y_star[i], 2);
			evalNError += nn[i].distance(np);
			if (prtPreds)
				context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f normal=[%f %f %f] V=[%f %f %f]\n", i, y_star[i], fx[i], varx[i], normals2(i, 0), normals2(i, 1), normals2(i, 2), V(i, 0), V(i, 1), V(i, 2));
		}
		if (prtPreds)
			context.write("Evaluate(): error=%f avg=%f n_avg=%f\n\n", evalError, evalError / testSize, evalNError / testSize);
		// render normals
		if (draw) renderNormals(x_star, nn);
		return;

		/*****  Add point to the model  *********************************************/
		context.write("Add %d patterns to the model\n", testSize);
		gp->add_patterns(x_star, y_star);

		// test on the test points
		for (size_t i = 0; i < testSize; ++i) {
			const double q2f = 0; // gp->f(x_star[i]);
			const double q2Var = gp->var(x_star[i]);

			if (prtPreds)
				context.write("f(x_star) = %f -> f_star = %f v_star = %f\n", y_star[i], q2f, q2Var);

		}
	}));

	menuCmdMap.insert(std::make_pair("Y", [=]() {
		/*****  Global variables  ******************************************/
		size_t N_sur = 250, N_ext = 0, N_int = 0, N_tot = N_sur + N_ext + N_int;
		const Real surRho = 0.025, extRho = 0.05, intRho = 0.001;
		golem::Real noise = 0.001; //

		const bool prtInit = false;
		const bool prtPreds = true;
		const bool draw = true;

		/*****  Generate Input data  ******************************************/
		Vec3Seq cloud, surCloud, normals;
		RealSeq targets;
		golem::Vec3 frameSize(.01, .01, .01);
		context.write("Generate input data %lu\n", N_sur + N_ext + N_int);
		
		context.write("Cloud points %lu:\n", N_sur);
		createSphere(REAL_ZERO, surRho, noise, N_sur, cloud, targets, normals, prtInit);
		// reset render
		if (draw) to<Data>(dataCurrentPtr)->createRender();
		// render model point cloud
		if (draw) renderCloud(cloud, RGBA::GREEN);
		
		context.write("\nExternal points %lu:\n", N_ext);
		createSphere(REAL_ONE, extRho, noise, N_ext, cloud, targets, normals, prtInit);

		context.write("\nInternal point(s) %lu:\n", N_int);
		createSphere(-REAL_ONE, intRho, noise, N_int, cloud, targets, normals, prtInit);

		//cloud.push_back(Vec3(0.031193, 0.177345, 0.033029));
		//cloud.push_back(Vec3(-0.058635, 0.173399, 0.164797));
		//cloud.push_back(Vec3(-0.225947, - 0.020312, 0.122944));
		//cloud.push_back(Vec3(0.566214, 0.076836, - 0.137992));
		//cloud.push_back(Vec3(-0.075327, 0.007571, - 0.006788));

		//normals.push_back(Vec3(0.031193, 0.177345, 0.033029).normalise());
		//normals.push_back(Vec3(-0.058635, 0.173399, 0.164797).normalise());
		//normals.push_back(Vec3(-0.225947, -0.020312, 0.122944).normalise());
		//normals.push_back(Vec3(0.566214, 0.076836, -0.137992).normalise());
		//normals.push_back(Vec3(-0.075327, 0.007571, -0.006788).normalise());

		//targets.push_back(0);
		//targets.push_back(0);
		//targets.push_back(0);
		//targets.push_back(1.0);
		//targets.push_back(-1.0);

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets, normals));
//#define cov_thin_plate
#define gaussianReg
#ifdef gaussianReg
			GaussianRegressor::Desc covDesc;
			covDesc.initialLSize = covDesc.initialLSize > N_tot ? covDesc.initialLSize : N_tot + 100;
			covDesc.covTypeDesc.inputDim = trainingData->cols();
			covDesc.covTypeDesc.length = 1.0; // 0.03;
			covDesc.covTypeDesc.sigma = 1.0; // 0.015;
			covDesc.covTypeDesc.noise = noise;// 0.0001;
			covDesc.optimise = true;
			covDesc.debug = false;
			GaussianRegressor::Ptr gp = covDesc.create(context);
			context.write("%s Regressor created. inputDim=%d, parmDim=%d, l=%f, s=%f noise=%f\n", gp->getName().c_str(), covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
				covDesc.covTypeDesc.length, covDesc.covTypeDesc.sigma, covDesc.covTypeDesc.noise);
#endif
#ifdef cov_thin_plate
			ThinPlateRegressor::Desc covDesc;
//			covDesc.initialLSize = trainingData->rows();
			covDesc.covTypeDesc.inputDim = trainingData->cols();
			covDesc.covTypeDesc.noise = 0.0; // noise;
			Real l = REAL_ZERO;
			for (size_t i = 0; i < cloud.size(); ++i) {
				for (size_t j = i; j < cloud.size(); ++j) {
					const Real d = cloud[i].distance(cloud[j]);
					if (l < d) l = d;
				}
			}
			covDesc.covTypeDesc.length = l;
			covDesc.optimise = false;
			covDesc.debug = false;
			context.write("ThinPlate cov: inputDim=%d, parmDim=%d, R=%f noise=%f\n", covDesc.covTypeDesc.inputDim, covDesc.covTypeDesc.paramDim,
				covDesc.covTypeDesc.length, covDesc.covTypeDesc.noise);
			ThinPlateRegressor::Ptr gp = covDesc.create(context);
			context.write("%s Regressor created\n", gp->getName().c_str());
#endif
		gp->set(trainingData);
		context.write("Training dataset X[%d %d] Y[%d]\n", trainingData->rows(), trainingData->cols(), trainingData->y().size());

		/*****  Query the model with new points  *********************************/
		const size_t testSize = 250;
		context.write("Query model with new %d points\n", testSize);
		Vec3Seq x_star, n_star, nx_star; nx_star.resize(testSize);
		RealSeq vars;
		RealSeq y_star;
		createSphere(0.0, surRho, noise, testSize, x_star, y_star, n_star, prtInit);

		Real predError = REAL_ZERO;
		for (size_t i = 0; i < testSize; ++i) {
			// gp estimate of y value
			const Eigen::Vector4d vv = gp->f(x_star[i]);
			const double f_star = vv(0);
			nx_star[i] = golem::Vec3(vv(1), vv(2), vv(3));
			nx_star[i].normalise();
			const double v_star = gp->var(x_star[i]);
			//const double v_star = gp->var(x_star[i])(0);
			const double er = (1 - abs(n_star[i].dot(nx_star[i])))*90;
			vars.push_back(v_star);
			predError += std::pow(f_star - y_star[i], 2);
			if (prtPreds)
				context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f normal=[%f %f %f] err = %f [deg]\n", i, y_star[i], f_star, v_star, nx_star[i].x, nx_star[i].y, nx_star[i].z, er);
		}
		if (prtPreds)
			context.write("Prediction(): error=%f avg=%f\n\n", predError, predError / testSize);

		// render normals
		if (draw) renderNormals(x_star, nx_star);
		return;

		/*****  Add point to the model  *********************************************/
		context.write("Add %d patterns to the model\n", testSize);
		gp->add_patterns(x_star, y_star);

		// test on the test points
		for (size_t i = 0; i < testSize; ++i) {
			const double q2f = 0; // gp->f(x_star[i]);
			const double q2Var = gp->var(x_star[i]);

			if (prtPreds)
				context.write("f(x_star) = %f -> f_star = %f v_star = %f\n", y_star[i], q2f, q2Var);

		}
	}));

}

//------------------------------------------------------------------------------

void RRTPlayer::createSphere(const golem::Real surRho, const golem::Real rho, const golem::Real noise, const size_t size, spam::Vec3Seq& points, spam::RealSeq& targets, spam::Vec3Seq& normals, const bool verbose) const {
	const golem::Real rhoSqr = Math::sqr(rho), surRhoSqr = Math::sqr(surRho);
	for (size_t i = 0; i < size; ++i) {
		const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
		const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
		const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, rho) - rho) * Math::cos(colatitude);
		golem::Vec3 point(Math::sin(theta)*sqrt(rhoSqr - z*z), Math::cos(theta)*Math::sqrt(rhoSqr - z*z), z);
		point += golem::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
		points.push_back(point);
		normals.push_back(point);
		normals.back().normalise();
		const double y = surRho; // point.magnitudeSqr() - surRhoSqr;
		targets.push_back(y);
		if (verbose)
			context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
	}

}

//------------------------------------------------------------------------------

void RRTPlayer::render() const {
//	R2GPlanner::render();
	{
		golem::CriticalSectionWrapper cswRenderer(getCS());
		cdebugRenderer.render();
	}
}

void RRTPlayer::renderCloud(const spam::Vec3Seq& cloud, const RGBA colour) {
	{
		golem::CriticalSectionWrapper cswRenderer(getCS());
		for (auto i = cloud.begin(); i != cloud.end(); ++i)
			cdebugRenderer.addPoint(*i, colour);
	}
}

void RRTPlayer::renderCromCloud(const spam::Vec3Seq& cloud, const spam::RealSeq& vars, const golem::RGBA cInit, const golem::RGBA cEnd) {
	if (cloud.size() != vars.size()) {
		context.write("renderCromCloud(): Error: Size mismatch [cloud %d - vars %d]!\n", cloud.size(), vars.size());
		return;
	}
	cdebugRenderer.setPointSize(0.02);

	Real maximum = -numeric_const<Real>::MIN;
	for (auto i = vars.begin(); i != vars.end(); ++i) {
		if (*i > maximum)
			maximum = *i;
	}
//	context.write("Max %f\n", maximum);
	{
		golem::CriticalSectionWrapper cswRenderer(getCS());
		for (size_t i = 0; i < cloud.size(); ++i) {
			const Real red = (Math::abs(vars[i]) / maximum) * 255;
			const Real blue = 255 - red;
//			context.write("Var %f red %f blue %f\n", vars[i], red, blue);
			cdebugRenderer.addPoint(cloud[i], RGBA(U8(red), U8(0), U8(blue), U8(200)));
		}
	}

}

void RRTPlayer::renderNormals(const spam::Vec3Seq& cloud, const spam::Vec3Seq& normals, const golem::RGBA cNormals) {
	const size_t size = cloud.size() != normals.size() ? 0 : normals.size();
	if (size == 0) 
		context.write("renderNormals(): Error: Size mismatch [cloud %d - normals %d]!\n", cloud.size(), normals.size());

	const Real normalSize = Real(0.01);
	for (auto ni = 0; ni < size; ++ni) {
		Mat34 frame; frame.setId();
		frame.p = cloud[ni];
		Vec3 n;
		n.multiply(normalSize, normals[ni]);
		//frame.R.multiply(n, n);
		Real v1 = frame.R.m11 * n.v1 + frame.R.m12 * n.v2 + frame.R.m13 * n.v3;
		Real v2 = frame.R.m21 * n.v1 + frame.R.m22 * n.v2 + frame.R.m23 * n.v3;
		Real v3 = frame.R.m31 * n.v1 + frame.R.m32 * n.v2 + frame.R.m33 * n.v3;
		n.v1 = v1;
		n.v2 = v2;
		n.v3 = v3;
		n.add(frame.p, n);
		cdebugRenderer.addLine(frame.p, n, cNormals);
//		cdebugRenderer.addPoint(frame.p, RGBA::BLACK);
	}

}


//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return RRTPlayer::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
