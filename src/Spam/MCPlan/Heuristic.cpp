#include <Spam/MCPlan/Heuristic.h>

//-------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//-------------------------------------------------------------------------

/** static function to compute the linear distance between to points. */
static Real EuclidianDistance(const Vec3 &x1, const Vec3 &x2){
	Real dx = x1.x - x2.x;
	Real dy = x1.y - x2.y;
	Real dz = x1.z - x2.z; 
	return ::sqrt(dx*dx + dy*dy + dz*dz);
};

/** static function to compute the linear distance between two points 
	in the plane defined by x,z-axis */
static Real PlanarEuclidianDistance(const Vec3 &x1, const Vec3 &x2){
	Real dx = x1.x - x2.x;
	Real dy = x1.y - x2.y;
	return ::sqrt(dx*dx + dy*dy);
};

///** static function to compute the angular distance between to points in the 3D quaternoin sphere */
//static Real AngularDistance(const Quat& q0, const Quat& q1) {
//	// acos(quat0 * quat1): range: <0, Pi>, -Pi - identity, Pi - the largest distance
//	Quat q1neg = q1;
//	q1neg.negate();
//	return (REAL_ONE/REAL_PI)*std::min(acos(q0.dot(q1)), acos(q0.dot(q1neg)));
//};

//-------------------------------------------------------------------------

spam::Heuristic::Heuristic() {
}

bool spam::Heuristic::create(const Desc &desc) {
	if (!desc.isValid())
		return false;

	linDistFac = desc.linDistFac;
	angDistFac = desc.angDistFac;
	return true;
}

//-------------------------------------------------------------------------

//Real spinta::Heuristic::metric(const Vec3 &p1, const Vec3 &p2) const {
//	return EuclidianDistance(p1, p2);
//}
//
//Real spinta::Heuristic::metric(const Quat &q1, const Quat &q2) const {
//	return AngularDistance(q1, q2);
//}
//
//Real spinta::Heuristic::metric(const Mat34 &x1, const Mat34 &x2) const {
//	const Real d = metric(x1.p, x2.p);
//	Quat q1(x1.R), q2(x2.R);
//	const Real a = metric(q1, q2);
//	return linDistFac*d + angDistFac*a;
//}

Real spam::Heuristic::cost(const Vec3& x1, const Vec3& x2) const {
	return linDistFac * x1.distance(x2);
	//const Real d = metric(x1.p, x2.p);
	//Quat q1(x1.R), q2(x2.R);
	//const Real a = metric(q1, q2);
	//return linDistFac*d + angDistFac*a;
}

Real spam::Heuristic::cost(const Vec3& x) const {
	return x.magnitude();
}


//-------------------------------------------------------------------------

//void spinta::XMLData(spinta::Heuristic::Desc &val,  Context *context, XMLContext *xmlcontext, bool create) {
//	ASSERT(xmlcontext)
//
//	XMLData("lin_dist_fac", val.linDistFac, xmlcontext, create);
//	XMLData("ang_dist_fac", val.angDistFac, xmlcontext, create);
//}