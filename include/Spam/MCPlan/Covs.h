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

namespace spam {
	typedef golem::Vec3 Vec3;
	typedef std::vector<golem::Vec3> Vec3Seq;
	typedef golem::U32 U32;
	typedef golem::U64 U64;
	typedef golem::I32 I32;
	typedef golem::Real Real;
	static const Real REAL_ZERO = golem::numeric_const<Real>::ZERO;
	static const Real REAL_MAX = golem::numeric_const<Real>::MAX;
	static const Real TWO_PI = golem::numeric_const<golem::Real>::TWO_PI;
}
#endif


//------------------------------------------------------------------------------

namespace spam
{

//------------------------------------------------------------------------------

static Eigen::VectorXd convertToEigen(const Vec& v) {
	return Eigen::Map<Eigen::VectorXd>((double *)v.data(), v.size());
}
static Eigen::Vector3d convertToEigen(const golem::Vec3& v) {
	return Eigen::Map<Eigen::VectorXd>((double *)v.data(), 3);
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
		/** Default C'tor */
		Desc() {
			setToDefault();
		}
		
		/** Set values to default */
		void setToDefault() {}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_0(BaseCovFunc, BaseCovFunc::Ptr)
		
		/** Assert valid descriptor files */
		bool isValid(){ return true; }
		
	};
	
	/** Get name of the covariance functions */
	virtual std::string getName() const {
		return "BaseCovFunc";
	}
	  
    /** Compute the kernel */
    virtual inline double get(const golem::Vec3& x1, const golem::Vec3& x2) const { return x1.distance(x2); }
	/** Compute the derivate */
	virtual inline double getDiff(const golem::Vec3& x1, const golem::Vec3& x2, const double noise = .0) const { return (x1.distanceSqr(x2) + noise)*2; }

    /** Access to loghyper_change */
    inline bool getLogHyper() const { return loghyper_changed; }
    inline void setLogHyper(const bool b) { loghyper_changed = b; }
        
    virtual ~BaseCovFunc() {};

protected:
	/** Determine when to recompute the kernel */
	bool loghyper_changed;
	
	/** Create from descriptor */
    virtual void create(const Desc& desc) {
        loghyper_changed = true;
    }
        
	/** Default C'tor */
    BaseCovFunc() {}
};

//------------------------------------------------------------------------------

}

#endif
