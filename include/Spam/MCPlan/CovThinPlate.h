/** @file CovThinPlate.h
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
#ifndef __GP__THINPLATE_H__
#define __GP__THINPLATE_H__

//------------------------------------------------------------------------------

#include <Spam/MCPlan/Covs.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

class ThinPlate : public BaseCovFunc 
{
public:
	/** Pointer to the Covariance function */
	typedef boost::shared_ptr<ThinPlate> Ptr;

	/** Descriptor file */
	class Desc : public BaseCovFunc::Desc {
	public:
		/** Pointer to description file */
		typedef boost::shared_ptr<ThinPlate::Desc> Ptr;
		
		/** Hyper-parameters */
        double length;
		
		/** Default C'tor */
		Desc() {
			setToDefault();
		}
		
		/** Set values to default */
		void setToDefault() {
			BaseCovFunc::Desc::setToDefault();
			inputDim = 3;
			paramDim = 2;
			length = 1.0;
		}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(ThinPlate, BaseCovFunc::Ptr, const golem::Context&)
		
		/** Assert valid descriptor files */
		bool isValid(){ 
			if (!std::isfinite(length))
				return false;
			return true;
		}
	}; 
	
	/** Get name of the covariance functions */
	virtual std::string getName() const {
		return "ThinPlate";
	}
	
	/** Compute the kernel */
	virtual double get(const Eigen::VectorXd &xi, const Eigen::VectorXd &xj, const bool dirac = false) const {
		const double r = (xi - xj).norm();
		const double noise = dirac ? sn2 : .0;
		return 2 * std::pow(r, 3.0) - threeLength * pow(r, 2.0) + length3/* + noise*/;
	}

	/** thin plate kernel derivative = 6(xi - xj)(r - R) */
	virtual inline double getDiff(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx, const bool dirac = false) const {
		const double r = (xi - xj).norm();
//		printf("r=%f R=%f dx=%d xi-xj=%f dk=%f\n", r, loghyper(0), dx, (xi(dx) - xj(dx)), 6 * (r - loghyper(0)) * ((xi(dx) - xj(dx))));
		return dx < 0 ? 6 * r * (r - loghyper(0)) : 6 * (r - loghyper(0)) * ((xi(dx) - xj(dx))); // 6 * r * r - 6 * loghyper(0) * r;
	}

	/** thin plate kernel derivative = 6[(xi - xj)(xi - xj) + r - R] */
	virtual inline double getDiff2(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx1, const size_t dx2, const bool dirac = false) const {
		const double r = (xi - xj).norm();
		const double noise = dirac ? sn2 : .0;
		return 6 * (r - loghyper(0)) * (((xi(dx1) - xj(dx1)) * (xi(dx2) - xj(dx2))));
		//return dirac || r < golem::REAL_EPS ? dx1 == dx2 ? loghyper(0)/*6 * (r - loghyper(0))*/ : golem::REAL_ZERO : 6 * (r - loghyper(0)) * (((xi(dx1) - xj(dx1)) * (xi(dx2) - xj(dx2))) / r/* + noise*/);
	}

	/** Update parameter vector.
	*  @param p new parameter vector */
	virtual void setLogHyper(const Eigen::VectorXd &p) {
		BaseCovFunc::setLogHyper(p);
		threeLength = 3 * loghyper(0);
		length3 = std::pow(loghyper(0), 3.0);
		sn2 = loghyper(1) * loghyper(1); //2 * loghyper(1) * loghyper(1);//
	}

    ~ThinPlate() {};

private:
    /** Hyper-parameters */
    double threeLength;
    double length3;
        
        /** Create from descriptor */
	void create(const Desc& desc) {
		BaseCovFunc::create(desc);
		loghyper(0) = range(0) = desc.length;
		loghyper(1) = range(1) = desc.noise;
		threeLength = 3 * loghyper(0);
		length3 = std::pow(loghyper(0), 3.0);
		sn2 = loghyper(1) * loghyper(1); //2 * loghyper(1) * loghyper(1);//
	}

	ThinPlate(const golem::Context& context) : BaseCovFunc(context) {}
};

//------------------------------------------------------------------------------

}

#endif
