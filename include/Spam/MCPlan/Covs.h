/** @file Covs.h
 * 
 * 
 * @author	Claudio Zito
 *
 * @copyright  Copyright (C) 2015 Claudio, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 * Refer to Gaussian process library for Machine Learning.
 *
 */
#ifndef ___GP_COV_FUNCTIONS_H
#define ___GP_COV_FUNCTIONS_H

//------------------------------------------------------------------------------

#include <cmath>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <string>

#if defined(__unix__)
#include <unistd.h>
#include <pwd.h>
#include <sys/time.h>
#include <sys/times.h>
#include <sys/types.h>
#include <Vec3.h>
#endif
//#define WIN32
#ifdef WIN32
//#include <time.h>
//#include <boost/shared_ptr.hpp>
#include <Golem/Math/Math.h>
#include <Golem/Math/Vec3.h>
#include <Golem/Tools/Context.h>
#include <Golem/Math/Sample.h>

//namespace spam {
//	typedef golem::Vec3 Vec3;
//	typedef std::vector<golem::Vec3> Vec3Seq;
//	typedef golem::U32 U32;
//	typedef golem::U64 U64;
//	typedef golem::I32 I32;
//	typedef golem::Real Real;
//	static const Real REAL_ZERO = golem::numeric_const<Real>::ZERO;
//	static const Real REAL_MAX = golem::numeric_const<Real>::MAX;
//	static const Real TWO_PI = golem::numeric_const<golem::Real>::TWO_PI;
//}
#endif


//------------------------------------------------------------------------------

namespace spam
{

//------------------------------------------------------------------------------

static Eigen::VectorXd convertToEigen(const RealSeq& v) {
	return Eigen::Map<Eigen::VectorXd>((double *)v.data(), v.size());
}
static Eigen::Vector3d convertToEigen(const golem::Vec3& v) {
	return Eigen::Map<Eigen::VectorXd>((double *)v.data(), 3);
}
static Eigen::VectorXd convertToEigenXd(const golem::Vec3& v) {
	return Eigen::Map<Eigen::VectorXd>((double *)v.data(), 3);
}

static Eigen::MatrixXd convertSeqToEigenXd(const Vec3Seq& seq) {
	Eigen::MatrixXd res = Eigen::MatrixXd::Zero(seq.size(), 3);
	for (size_t i = 0; i < seq.size(); ++i)
		res.row(i) = convertToEigenXd(seq[i]);
	return res;
}

static Eigen::VectorXd convertVecToEigenXd(const RealSeq& seq, const size_t d = 1) {
	Eigen::VectorXd res = Eigen::VectorXd::Zero(seq.size());
	for (size_t i = 0; i < seq.size(); ++i)
		res(i) = seq[i];
	return res;
}

//------------------------------------------------------------------------------

/** Object creating function from the description. */
#define CREATE_FROM_OBJECT_DESC_0(OBJECT, POINTER) virtual POINTER create() const {\
	OBJECT *pObject = new OBJECT();\
	POINTER pointer(pObject);\
	pObject->create(*this);\
	return pointer;\
}

/** Object creating function from the description. */
#define CREATE_FROM_OBJECT_DESC_1(OBJECT, POINTER, PARAMETER) virtual POINTER create(PARAMETER parameter) const {\
	OBJECT *pObject = new OBJECT(parameter);\
	POINTER pointer(pObject);\
	pObject->create(*this);\
	return pointer;\
}

/** Template bject creating function from the description. */
#define CREATE_FROM_OBJECT_TEMPLATE_DESC_1(OBJECT, TEMPLATE, POINTER, PARAMETER) virtual POINTER create(PARAMETER parameter) const {\
	OBJECT<TEMPLATE> *pObject = new OBJECT<TEMPLATE>(parameter);\
	POINTER pointer(pObject);\
	pObject->create(*this);\
	return pointer;\
}

//------------------------------------------------------------------------------

class BaseCovFunc
{
public:
	typedef boost::shared_ptr<BaseCovFunc> Ptr;
	
	/** Descriptor file */
	class Desc {
	public:
		typedef boost::shared_ptr<BaseCovFunc::Desc> Ptr;

		/** Input dimensionality. */
		size_t inputDim;
		/** Size of parameter vector. */
		size_t paramDim;

		/** Noise parameter */
		double noise;

		/** Default C'tor */
		Desc() {
			setToDefault();
		}
		
		/** Set values to default */
		void setToDefault() {
			inputDim = 0;
			paramDim = 1;
			noise = 0.0;
		}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(BaseCovFunc, BaseCovFunc::Ptr, const golem::Context&)
		
		/** Assert valid descriptor files */
		bool isValid(){ 
			if (inputDim < 0 || paramDim < 0)
				return false;
			return true; 
		}
		
	};
	
	/** Get name of the covariance functions */
	virtual std::string getName() const {
		return "BaseCovFunc";
	}

	/** Get the hyperparameters as a string */
	virtual std::string getHyper2str() const {
		std::stringstream ss;
		for (size_t i = 0; i < paramDim; ++i)
			ss << loghyper(i) << " ";
		return ss.str();
	}

	  
 //   /** Compute the kernel */
    virtual inline double get(const golem::Vec3& x1, const golem::Vec3& x2, const bool dirac = false) const { 
		return get(convertToEigenXd(x1), convertToEigenXd(x2), dirac);
	}
	/** Compute the kernel */
	virtual double get(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, const bool dirac = false) const {
		const double z = ((x1 - x2)).squaredNorm();
		const double noise = dirac ? sn2 : .0;

		return z + noise;
	}
	virtual inline double getDiff(const golem::Vec3& xi, const golem::Vec3& xj, const size_t dx, const bool dirac = false) const {
		return getDiff(convertToEigenXd(xi), convertToEigenXd(xj), dx, dirac);
	}

	virtual inline double getDiff(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx, const bool dirac = false) const {
		return xi.dot(xj);
	}

	virtual inline double getDiff2(const golem::Vec3& xi, const golem::Vec3& xj, const size_t dx1, const size_t dx2, const bool dirac = false) const {
		return getDiff2(convertToEigenXd(xi), convertToEigenXd(xj), dx1, dx2, dirac);
	}
	virtual inline double getDiff2(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx1, const size_t dx2, const bool dirac = false) const {
		const double k = get(xi, xj, dirac); //sqrt(DD);
		const double noise = dirac ? sn2 : .0;
		return noise * k;
	}

	//virtual inline double getDiff(const golem::Vec3& x1, const golem::Vec3& x2) const { 
	//	return x1.distanceSqr(x2)*2; 
	//}

    /** Access to loghyper_change */
    inline bool isLogHyper() const { return loghyper_changed; }
    inline void setLogHyper(const bool b) { loghyper_changed = b; }

	/** Return input dimension */
	inline size_t getInputDim() const { return inputDim; }
	/** Return parmeters dimension */
	inline size_t getParamDim() const { return paramDim; }
	/** Get log-hyperparameter of covariance function */
	Eigen::VectorXd getLogHyper() const { return loghyper; }
	
	virtual void grad(const golem::Vec3& x1, const golem::Vec3& x2, Eigen::VectorXd& g) const {
		grad(convertToEigenXd(x1), convertToEigenXd(x2), g);
	};

	/** Covariance gradient of two input vectors with respect to the hyperparameters.
	*  @param x1 first input vector
	*  @param x2 second input vector
	*  @param grad covariance gradient */
	virtual void grad(const Eigen::VectorXd &x1, const Eigen::VectorXd &x2, Eigen::VectorXd& g) const {};

	/** Update parameter vector.
	*  @param p new parameter vector */
	virtual void setLogHyper(const Eigen::VectorXd &p) {
		assert(p.size() == loghyper.size());
		loghyper = p;
		loghyper_changed = true;
	}

	/** Update parameter vector.
	*  @param p new parameter vector */
	virtual void setLogHyper(const double p[]) {
		Eigen::Map<const Eigen::VectorXd> p_vec_map(p, paramDim);
		setLogHyper(p_vec_map);
	}

	/** Draw random target values from this covariance function for input X. */
	Eigen::VectorXd drawRandomSample(Eigen::MatrixXd &X) {
		assert(X.cols() == int(inputDim));
		int n = X.rows();
		Eigen::MatrixXd K(n, n);
		Eigen::LLT<Eigen::MatrixXd> solver;
		Eigen::VectorXd y(n);
		// compute kernel matrix (lower triangle)
		//for (int i = 0; i < n; ++i) {
		//	for (int j = i; j < n; ++j) {
		//		K(j, i) = get((Eigen::VectorXd&)X.row(j), (Eigen::VectorXd&)X.row(i));
		//	}
		//	y(i) = randn();
		//}
		// perform cholesky factorization
		solver = K.llt();
		return solver.matrixL() * y;
	}

	inline Eigen::VectorXd sampleParams() const {
		Eigen::VectorXd sol; 
		sol.resize(paramDim);
		for (size_t i = 0; i < paramDim; ++i) {
			sol(i) = 2 * range(i) * rand.nextUniform<golem::Real>(0, 1) - range(i);
		}
		return sol;
//		return Eigen::VectorXd::Random(paramDim, 1);
	}

    virtual ~BaseCovFunc() {};

protected:
	/** Context */
	golem::Context context;
	/** Determine when to recompute the kernel */
	bool loghyper_changed;
	/** Input dimensionality. */
	size_t inputDim;
	/** Size of parameter vector. */
	size_t paramDim;

	/** Noise on the input points */
	double sn2; 
	golem::Rand rand;

	/** Parameter vector containing the log hyperparameters of the covariance function.
	*  The number of necessary parameters is given in param_dim. */
	Eigen::VectorXd loghyper;
	Eigen::VectorXd range;

	//static inline double drand48() {
	//	return (rand() / (RAND_MAX + 1.0));
	//};

	//static double randn() {
	//	double u1 = 1.0 - drand48(), u2 = 1.0 - drand48();
	//	return sqrt(-2 * log(u1))*cos(2 * M_PI*u2);
	//};

	/** Create from descriptor */
    virtual void create(const Desc& desc) {
        loghyper_changed = true;
		inputDim = desc.inputDim;
		paramDim = desc.paramDim;
		loghyper.resize(paramDim);
		loghyper.setZero();
		sn2 = desc.noise * desc.noise;
		range.setOnes(paramDim);
    }
        
	/** Default C'tor */
	BaseCovFunc(const golem::Context& context) : context(context), rand(context.getRandSeed()) {}
};

//------------------------------------------------------------------------------

}

#endif
