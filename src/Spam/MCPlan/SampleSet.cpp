/** @file SampleSet.cpp
 * 
 * 
 * @author	Claudio Zito
 *
 */
#include <Spam/MCPlan/SampleSet.h>
#include <Eigen/StdVector>

//------------------------------------------------------------------------------

using namespace golem;
using namespace spam;

//------------------------------------------------------------------------------

SampleSet::SampleSet() {
	n = 0;
	d = 3;
}

SampleSet::SampleSet(const Vec3Seq& inputs, const RealSeq& targets) {
	assert(inputs.size() == targets.size());

	n = inputs.size();
	d = 3;
	X.clear();
	Y.clear();
	X.insert(X.end(), inputs.begin(), inputs.end());
	Y.insert(Y.end(), targets.begin(), targets.end());
	N.clear();
}

SampleSet::SampleSet(const Vec3Seq& inputs, const RealSeq& targets, const Vec3Seq& normals) {
	assert(inputs.size() == targets.size());

	n = inputs.size();
	d = 3;
	X.clear();
	Y.clear();
	X.insert(X.end(), inputs.begin(), inputs.end());
//	Y.insert(Y.end(), targets.begin(), targets.end());
	
	Y.assign(4 * n, REAL_ZERO);
	for (size_t i = 0; i < targets.size(); ++i) {
		Y[i * 4] = targets[i];
		Y[i * 4 + 1] = normals[i].x;
		Y[i * 4 + 2] = normals[i].y;
		Y[i * 4 + 3] = normals[i].z;
	}
	//for (size_t i = 0; i < targets.size(); ++i)
	//	Y[i] = targets[i];
	//for (size_t i = 0; i < normals.size(); ++i) {
	//	Y[n + 3 * i] = normals[i].x;
	//	Y[n + 3 * i + 1] = normals[i].y;
	//	Y[n + 3 * i + 2] = normals[i].z;
	//}

	//printf("Trainind data:\n");
	//for (size_t i = 0; i < Y.size(); ++i)
	//	printf("Y[%d] =%f\n", i, Y[i]);
	//printf("\n");
}


SampleSet::~SampleSet() {
	clear();
}

//------------------------------------------------------------------------------

//void SampleSet::add(const double x[], double y) {
//	Eigen::VectorXd * v = new Eigen::VectorXd(input_dim);
//	for (size_t i=0; i<input_dim; ++i) (*v)(i) = x[i];
//	X.push_back(v);
//	Y.push_back(y);
//	assert(X.size()==Y.size());
//	n = X.size();
//}

//void SampleSet::add(const Eigen::VectorXd x, double y) {
//	Eigen::VectorXd * v = new Eigen::VectorXd(x);
//	X.push_back(v);
//	Y.push_back(y);
//	assert(X.size()==Y.size());
//	n = X.size();
//}

void SampleSet::add(const Vec3Seq& newInputs, const RealSeq& newTargets) {
	assert(newInputs.size() == newTargets.size());

	n += newInputs.size();
	X.insert(X.end(), newInputs.begin(), newInputs.end());
	Y.insert(Y.end(), newTargets.begin(), newTargets.end());
}

void SampleSet::add(const Vec3Seq& newInputs, const RealSeq& newTargets, const Vec3Seq& newNormals) {
	assert(newInputs.size() == newTargets.size());

	n += newInputs.size();
	X.insert(X.end(), newInputs.begin(), newInputs.end());
//	Y.insert(Y.end(), newTargets.begin(), newTargets.end());

	RealSeq tmp; tmp.assign(4 * newNormals.size(), REAL_ZERO);
	for (size_t i = 0; i < newTargets.size(); ++i)
		tmp[i] = newTargets[i];
	for (size_t i = 0; i < newNormals.size(); ++i) {
		tmp[3 * i] = newNormals[i].x;
		tmp[3 * i + 1] = newNormals[i].y;
		tmp[3 * i + 2] = newNormals[i].z;
	}
	Y.insert(Y.end(), tmp.begin(), tmp.end());
}

//------------------------------------------------------------------------------

const golem::Vec3& SampleSet::x(size_t k) const {
	return X[k];
}

double SampleSet::y(size_t k) const {
	return Y[k];
}

RealSeq SampleSet::y() const {
	return Y;
}

bool SampleSet::set_y(size_t i, double y) {
	if (i >= n) return false;
	Y[i] = y;
	return true;
}

//------------------------------------------------------------------------------

void SampleSet::clear() {
	n = 0;
	d = 0;
	X.clear();
	Y.clear();
}


//------------------------------------------------------------------------------
