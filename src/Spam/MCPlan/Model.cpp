#include <spam/MCPlan/Model.h>

//-------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//-------------------------------------------------------------------------

Model::Model(const golem::Context& context) : rand(context.getRandSeed()), context(context) {
}

bool Model::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	name = desc.name;
	modelDeltaT = desc.modelDeltaT;
	modelDeltaS = desc.modelDeltaS;
	stateDim = desc.stateDim;

	pHeuristic = desc.pHeuristicDesc->create();

	defaultNextState = [&](const Vec3& x) -> Vec3 {
		Vec3 v;
		v.next(rand);
		v.normalise();
		return x + v;
	};
	nextState = desc.nextState;

	defaultNextInput = [&](const Vec3& node, const Vec3& state) -> Vec3 {
		return state - node;
	};
	nextInput = desc.nextInput;

	return true;
}

//-------------------------------------------------------------------------

void Model::render(const Node3d::SeqPtr& nodes) {
	// ToDo
}

void Model::render() {
	// ToDo
}

//-------------------------------------------------------------------------

Vec3 Model::getNextState(const Vec3& x) const {
	return nextState ? nextState(x) : defaultNextState(x);
}

Vec3 Model::selectInput(const Vec3& node, const Vec3& state) const {
	return nextInput ? nextInput(node, state) : defaultNextInput(node, state);
}


Vec3 Model::integrate(const Vec3& x, const Vec3& u, const double h) {	
	return rungeKuttaIntegration(x, u, h);
}

Vec3 Model::stateTransitionEquation(const Vec3 &x, const Vec3 &u) {
	Vec3 dx;
	dx[0] = x.x * u.x;
	dx[1] = x.y * u.y;
	dx[2] = x.z * u.z;
	return dx;
}

Vec3 Model::rungeKuttaIntegration(const Vec3 &x, const Vec3 &u, const double &h) {
	Vec3 k1, k2, k3, k4;
	int s, i, k;
	double c, deltat;
	Vec3 nx;

	s = (h > 0) ? 1 : -1;

	c = s*h/modelDeltaT;  // Number of iterations (as a double)
	k = (int) c;
	deltat = s * modelDeltaT;

	nx = x;
	for (i = 0; i < k; i++) {
		k1 = stateTransitionEquation(nx,u);
		k2 = stateTransitionEquation(nx + k1*(0.5*deltat),u);
		k3 = stateTransitionEquation(nx + k2*(0.5*deltat),u);
		k4 = stateTransitionEquation(nx + k3*deltat,u);
		nx += (k1 + k2*2.0 + k3*2.0 + k4) * (deltat / 6.0);
	}

	// Integrate the last step for the remaining time
	k1 = stateTransitionEquation(nx,u);
	k2 = stateTransitionEquation(nx + k1*(0.5*deltat),u);
	k3 = stateTransitionEquation(nx + k2*(0.5*deltat),u);
	k4 = stateTransitionEquation(nx + k3*deltat,u);
	nx += (k1 + k2*2.0 + k3*2.0 + k4) * (deltat / 6.0);

	return nx;
}

bool Model::satisfied(const Vec3& x) {
	return true;
}

