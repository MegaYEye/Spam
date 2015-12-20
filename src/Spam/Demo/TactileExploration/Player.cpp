/** @file Player.cpp
*
* @author	Claudio Zito
*
* @version 1.0
*
*/

#include <Spam/Demo/TactileExploration/Player.h>

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
		owner->debugRenderer.reset();
		owner->pointAppearance.drawPoints(owner->cloudPoints, owner->debugRenderer);
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

	menuCmdMap.insert(std::make_pair("Y", [=]() {
		/*****  Global variables  ******************************************/
		size_t N_sur = 10, N_ext = 5, N_int = 1;
		golem::Real noise = 0.001;

		const bool prtInit = true;
		const bool prtPreds = true;

		/*****  Generate Input data  ******************************************/
		Vec3Seq cloud;
		Vec targets;
		golem::Vec3 frameSize(.01, .01, .01);
		context.write("Generate input data %lu\n", N_sur + N_ext + N_int);
		if (prtInit)
			context.write("Cloud points %lu:\n", N_sur);
		for (size_t i = 0; i < N_sur; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real z = 2 * rand.nextUniform<golem::Real>(0, 1) - 1;
			spam::Vec3 point(Math::sin(theta)*sqrt(1 - z*z), Math::cos(theta)*Math::sqrt(1 - z*z), z);
			point += spam::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud.push_back(point);
			// set point cloud
			Cloud::Point p = Cloud::Point(); 
			Cloud::setPoint<golem::Real>(point, p);
//			cloudPoints.push_back(p);

			double y = point.magnitudeSqr() - 1;
			if (prtInit) 
				context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
			targets.push_back(y);
		}


		// render point cloud
		to<Data>(dataCurrentPtr)->createRender();

		if (prtInit)
			context.write("\nExternal points %lu:\n", N_ext);
		for (size_t i = 0; i < N_ext; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real z = 2 * rand.nextUniform<golem::Real>(0, 1) - 1;
			golem::Vec3 point(Math::sin(theta)*sqrt(1.2 - z*z), Math::cos(theta)*Math::sqrt(1.2 - z*z), z);
			point += golem::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);
			cloud.push_back(point);

			double y = point.magnitudeSqr() - 1;
			if (prtInit) 
				context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n", i, theta, point.x, point.y, point.z, i, y);
			targets.push_back(y);
		}

		if (prtInit)
			context.write("\nInternal point(s) %lu:\n", N_int);
		spam::Vec3 zero; zero.setZero();
		cloud.push_back(zero);
		targets.push_back(zero.magnitudeSqr() - 1);
		if (prtInit)
			context.write("points[%d] th(%f) = [%f %f %f] targets[%d] = %f\n\n", 1, 0, zero.x, zero.y, zero.z, 1, zero.magnitudeSqr() - 1);

		/*****  Create the model  *********************************************/
		SampleSet::Ptr trainingData(new SampleSet(cloud, targets));
		LaplaceRegressor::Desc laplaceDesc;
		laplaceDesc.noise = noise;
		LaplaceRegressor::Ptr gp = laplaceDesc.create(context);
		context.write("Regressor created %s\n", gp->getName().c_str());
		gp->set(trainingData);
		//ThinPlateRegressor gp(&trainingData);


		/*****  Query the model with a point  *********************************/
		golem::Vec3 q(cloud[0]);
		const double qf = gp->f(q);
		const double qVar = gp->var(q);

		if (prtPreds)
			context.write("y = %f -> qf = %f qVar = %f\n\n", targets[0], qf, qVar);

		/*****  Query the model with new points  *********************************/
		const size_t testSize = 50;
		const double range = 1.0;
		Vec3Seq x_star; x_star.resize(testSize);
		Vec y_star; y_star.resize(testSize);
		for (size_t i = 0; i < testSize; ++i) {
			const golem::Real theta = 2 * REAL_PI*rand.nextUniform<golem::Real>(0, 1) - REAL_PI;
			const golem::Real z = 2 * rand.nextUniform<golem::Real>(0, 1) - 1;
			spam::Vec3 point(Math::sin(theta)*sqrt(1 - z*z), Math::cos(theta)*Math::sqrt(1 - z*z), z);
			point += spam::Vec3(2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise, 2 * noise * rand.nextUniform<golem::Real>(0, 1) - noise);

			// save the point for later
			x_star[i] = point;
			// compute the y value
			double y = point.magnitudeSqr() - 1;
			// save the y value for later
			y_star[i] = y;

			// gp estimate of y value
			const double f_star = gp->f(point);
			const double v_star = gp->var(point);
			if (prtPreds)
				context.write("f(x_star) = %f -> f_star = %f v_star = %f\n", y, f_star, v_star);
		}
		context.write("\n");

		/*****  Evaluate point and normal  *********************************************/
		Real fx, varx; 
		Eigen::Vector3d normal, tx, ty;
		gp->evaluate(x_star[0], fx, varx, normal, tx, ty);
		if (prtPreds)
			context.write("Evaluate[0] -> f=%f var=%f normal=[%f %f %f]\n", fx, varx, normal(0), normal(1), normal(2));

		/*****  Add point to the model  *********************************************/
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
	R2GPlanner::render();
	{
		golem::CriticalSectionWrapper cswRenderer(getCS());
		debugRenderer.render();
	}
}

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return RRTPlayer::Desc().main(argc, argv);
}

//------------------------------------------------------------------------------
