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

	menuCmdMap.insert(std::make_pair("F", [=]() {
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
		const size_t Nsur = 3000, Next = 150, Nint = 1, Ncurv =  curvPoints.size();
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
		if (draw) renderCloud(points, RGBA::YELLOW);

		Vec3Seq extCloud;
		extCloud.assign(Next, REAL_ZERO);
		Vec targets;
		targets.resize(Ntot);

		context.write("Load %d points from model point cloud (%d)\n", Nsur, Ncurv);
		for (size_t i = 0; i < Nsur; ++i) {
			const size_t index = size_t(rand.next() % Ncurv);
			surCloud[i] = cloud[i] = Cloud::getPoint<Real>(curvPoints[index]);
			targets[i] = REAL_ZERO;
			modelFrame += cloud[i];
		}
		// render model point cloud
		if (draw) renderCloud(surCloud, RGBA::BLACK);

		modelFrame /= Nsur;
		context.write("Create %d external points\n", Next);
		for (size_t i = 0; i < Next; ++i) {
			const size_t index = size_t(rand.next() % Nsur);
			const Vec3 v = cloud[index] - modelFrame;
			const Vec3 p = cloud[index] + v;
			extCloud[i] = cloud[Nsur + i] = p;
			targets[i] = REAL_ONE;
		}
		if (draw) renderCloud(extCloud, RGBA::RED);

		cloud[Nsur + Next] = modelFrame;
		targets[Nsur + Next] = -REAL_ONE;

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets));
		GaussianARDRegressor::Desc guassianDesc;
		guassianDesc.initialLSize = guassianDesc.initialLSize > Ntot ? guassianDesc.initialLSize : Ntot + 100;
		guassianDesc.covTypeDesc.inputDim = trainingData->cols();
		guassianDesc.noise = 0.003;
		GaussianARDRegressor::Ptr gp = guassianDesc.create(context);
		context.write("Gaussian ARD Regressor created %s\n", gp->getName().c_str());
		gp->set(trainingData);

		/*****  Evaluate points and normals  *********************************************/
		const size_t testSize = 20;
		context.write("Evaluate %d points and normals\n", testSize);
		std::vector<Real> fx, varx;
		Eigen::MatrixXd normals, tx, ty;
		Vec3Seq nn, xStar, normStar; 
		nn.resize(testSize);
		normStar.resize(testSize);
		Vec yStar;
		xStar.resize(testSize); yStar.assign(testSize, REAL_ZERO);
		for (size_t i = 0; i < testSize; ++i) {
			const size_t index = size_t(rand.next() % Ncurv);
			xStar[i] = Cloud::getPoint<Real>(curvPoints[index]);
			normStar[i] = Cloud::getNormal<Real>(curvPoints[index]);
			normStar[i].normalise();
			yStar[i] = REAL_ZERO;
		}
		Real evalError = REAL_ZERO, normError = REAL_ZERO;
		gp->evaluate(xStar, fx, varx, normals, tx, ty);
		for (size_t i = 0; i < testSize; ++i) {
			//points[i] = x_star[i];
			nn[i] = Vec3(normals(i, 0), normals(i, 1), normals(i, 2));
			normError += nn[i].dot(normStar[i]);
			evalError += std::pow(fx[i] - yStar[i], 2);
			context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f normal=[%f %f %f]\n", i, yStar[i], fx, varx, normals(i, 0), normals(i, 1), normals(i, 2));
		}
		context.write("Evaluate(): error=%f avg=%f normError=%f avg=%f\n\n", evalError, evalError / testSize, normError, normError / testSize);
		// render normals
		if (draw) renderNormals(xStar, nn);
		if (draw) renderNormals(xStar, normStar, RGBA::BLUE);


	}));


	menuCmdMap.insert(std::make_pair("Y", [=]() {
		/*****  Global variables  ******************************************/
		size_t N_sur = 30, N_ext = 30, N_int = 1, N_tot = N_sur + N_ext + N_int;
		const Real surRho = 0.05, extRho = 0.4;
		const Real surRhoSqr = Math::sqr(surRho), extRhoSqr = Math::sqr(extRho);
		golem::Real noise = 0.001; //REAL_ZERO; //

		const bool prtInit = true;
		const bool prtPreds = true;
		const bool draw = true;

		/*****  Generate Input data  ******************************************/
		Vec3Seq cloud, surCloud; cloud.resize(N_tot); surCloud.resize(N_sur);
		Vec targets; targets.resize(N_tot);
		golem::Vec3 frameSize(.01, .01, .01);
		context.write("Generate input data %lu\n", N_sur + N_ext + N_int);
		if (prtInit)
			context.write("Cloud points %lu:\n", N_sur);
		for (size_t i = 0; i < N_sur; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, surRho) - surRho) * Math::cos(colatitude);
			Vec3 point(Math::sin(theta)*sqrt(surRhoSqr - z*z), Math::cos(theta)*Math::sqrt(surRhoSqr - z*z), z);
			point += Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud[i] = point;
			// draws
			surCloud[i] = point;

			double y = point.magnitudeSqr() - surRhoSqr;
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
		for (size_t i = 0; i < N_ext; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, extRho) - extRho) * Math::cos(colatitude);
			golem::Vec3 point(Math::sin(theta)*sqrt(extRhoSqr - z*z), Math::cos(theta)*Math::sqrt(extRhoSqr - z*z), z);
			point += golem::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud[i] = point;
			extCloud[i] = point;

			double y = point.magnitudeSqr() - surRhoSqr;
			if (prtInit)
				context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
			targets[y];
		}
		// render model point cloud
		if (draw) renderCloud(extCloud, RGBA::BLACK);

		if (prtInit)
			context.write("\nInternal point(s) %lu:\n", N_int);
		Vec3 zero; zero.setZero();
		cloud.push_back(zero);
		targets.push_back(zero.magnitudeSqr() - surRho);
		if (prtInit)
			context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n\n", 1, 0, zero.x, zero.y, zero.z, 1, targets.back()/*zero.magnitudeSqr() - surRho*/);

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets));
#define gaussianReg
#ifdef laplaceReg
			LaplaceRegressor::Desc laplaceDesc;
			laplaceDesc.covTypeDesc.inputDim = trainingData->rows();
			laplaceDesc.noise = noise;
			LaplaceRegressor::Ptr gp = laplaceDesc.create(context);
			context.write("Laplace Regressor created %s\n", gp->getName().c_str());
#endif
#ifdef gaussianReg
			GaussianRegressor::Desc guassianDesc;
			guassianDesc.covTypeDesc.inputDim = trainingData->rows();
			guassianDesc.noise = noise;
			GaussianRegressor::Ptr gp = guassianDesc.create(context);
			context.write("Gaussian Regressor created %s\n", gp->getName().c_str());
#endif
		gp->set(trainingData);
		//ThinPlateRegressor gp(&trainingData);

		// render model point cloud with normals
		//if (draw) {
		//	Vec3Seq nn; nn.resize(surCloud.size());
		//	Eigen::MatrixXd NN = gp->getNormals();
		//	for (auto i = 0; i < surCloud.size(); ++i) {
		//		nn[i] = Vec3(NN(i, 0), NN(i, 1), NN(i, 2));
		//		context.write("Normal[%d] = [%f %f %f]\n", i, nn[i].x, nn[i].y, nn[i].z);
		//	}
		//	renderNormals(surCloud, nn);
		//}

		/*****  Query the model with a point  *********************************/
		golem::Vec3 q(cloud[0]);
		const double qf = gp->f(q);
		const double qVar = gp->var(q);

		if (prtPreds)
			context.write("Test estimate first point in the model\ny = %f -> qf = %f qVar = %f\n\n", targets[0], qf, qVar);

		/*****  Query the model with new points  *********************************/
		const size_t testSize = 150;
		context.write("Query model with new %d points\n", testSize);
		const double range = 1.0;
		Vec3Seq x_star; x_star.resize(testSize);
//		Vec3Seq points; points.resize(testSize);
		Vec vars;
		Vec y_star; y_star.resize(testSize);
		Real predError = REAL_ZERO;
		for (size_t i = 0; i < testSize; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real colatitude = (REAL_PI / 2) - (2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI);
			const golem::Real z = (2 * rand.nextUniform<golem::Real>(0, surRho) - surRho) * Math::cos(colatitude);
			Vec3 point(Math::sin(theta)*sqrt(surRhoSqr - z*z), Math::cos(theta)*Math::sqrt(surRhoSqr - z*z), z);
			point += Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);

			// save the point for later
			x_star[i] = point;
//			points[i] = point;
			// compute the y value
			double y = point.magnitudeSqr() - surRhoSqr;
			// save the y value for later
			y_star[i] = y;

			// gp estimate of y value
			const double f_star = gp->f(point);
			const double v_star = gp->var(point);
			vars.push_back(v_star);
			predError += std::pow(f_star - y, 2);
			if (prtPreds)
				context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f\n", i, y, f_star, v_star);
		}
		if (prtPreds)
			context.write("Prediction(): error=%f avg=%f\n\n", predError, predError / testSize);
		// render estimated points
		if (draw) renderCromCloud(x_star, vars);

		/*****  Evaluate points and normals  *********************************************/
		context.write("Evaluate %d points and normals\n", testSize);
		std::vector<Real> fx, varx;
		Eigen::MatrixXd normals, tx, ty;
		Vec3Seq nn; nn.resize(testSize);
//		points.clear(); points.resize(testSize);
		Real evalError = REAL_ZERO;
		gp->evaluate(x_star, fx, varx, normals, tx, ty);
		for (size_t i = 0; i < testSize; ++i) {
			//points[i] = x_star[i];
			nn[i] = Vec3(normals(i, 0), normals(i, 1), normals(i, 2));
			evalError += std::pow(fx[i] - y_star[i], 2);
			if (prtPreds)
				context.write("Evaluate[%d]: f(x_star) = %f -> f_star = %f v_star = %f normal=[%f %f %f]\n", i, y_star[i], fx, varx, normals(i, 0), normals(i, 1), normals(i,2));
		}
		if (prtPreds)
			context.write("Evaluate(): error=%f avg=%f\n\n", evalError, evalError / testSize);
		// render normals
		if (draw) renderNormals(x_star, nn);
//		return;

		/*****  Add point to the model  *********************************************/
		context.write("Add %d patterns to the model\n", testSize);
		gp->add_patterns(x_star, y_star);

		// test on the test points
		for (size_t i = 0; i < testSize; ++i) {
			const double q2f = gp->f(x_star[i]);
			const double q2Var = gp->var(x_star[i]);

			if (prtPreds)
				context.write("f(x_star) = %f -> f_star = %f v_star = %f\n", y_star[i], q2f, q2Var);

		}
	}));

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

void RRTPlayer::renderCromCloud(const spam::Vec3Seq& cloud, const Vec& vars, const golem::RGBA cInit, const golem::RGBA cEnd) {
	if (cloud.size() != vars.size()) {
		context.write("Error: Size mismatch!\n");
		return;
	}

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
		context.write("Error: Size mismatch!\n");

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
		cdebugRenderer.addPoint(frame.p, RGBA::BLACK);
	}

}


//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return RRTPlayer::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
