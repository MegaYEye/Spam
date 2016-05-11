/** @file CovCovSE.h
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
#ifndef __GP__COVSE_H__
#define __GP__COVSE_H__

//------------------------------------------------------------------------------

#include <Spam/MCPlan/Covs.h>
#include <cmath>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

class CovSE : public BaseCovFunc {
public:
	typedef boost::shared_ptr<CovSE> Ptr;

	/** Descriptor file */
	class Desc : public BaseCovFunc::Desc {
	public:
		typedef boost::shared_ptr<CovSE::Desc> Ptr;
		double sigma;
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
			sigma = 1.0;
			length = 1.0;
		}
		
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(CovSE, BaseCovFunc::Ptr, const golem::Context&)
		
		/** Assert valid descriptor files */
		bool isValid(){ 
			if (!std::isfinite(sigma) || !std::isfinite(length))
				return false;
			return true;
		}
	};
	
	/** Get name of the covariance functions */
	virtual std::string getName() const {
		return "CovSE";
	}
        
	/** Compute the kernel */
	virtual double get(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const bool dirac = false) const {
		const double z = ((xi - xj) / ell).norm(); //((xi - xj) / ell).norm();
		const double noise = dirac ? sn2 : .0;
		return sf2*exp(-0.5*z) + noise;
	}

	/** SE derivate = -1*invLenght*[sigma_f^2*exp(sqrt((p_1-p_2)'*(p_1-p_2))/(-leng))] */
	virtual inline double getDiff(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const int dx, const bool dirac = false) const {
		//const double k = get(xi, xj, dirac); //sqrt(DD);
		const double z = ((xi - xj) / ell).norm();
		const double k = exp(-0.5*z);
		// if dx < 0 then I compute the sum of the partial derivative
//		return ((xi(dx) - xj(dx)) * k) / ell;
		return dx < 0 ? -sf2 * ell * k : -sf2 * ell * (xi(dx) - xj(dx)) * k;
		//-golem::REAL_ONE * ell * k
	}

	virtual inline double getDiff2(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx1, const size_t dx2, const bool dirac = false) const {
		//const double k = get(xi, xj, dirac); //sqrt(DD);
		const double z = ((xi - xj) / ell).norm();
		const double k = exp(-0.5*z);
		
		//const double noise = dirac ? sn2 : .0;
//		printf("sf2=%f ell=%f 1/ell=%f noise=%f xi-xj=%f xi-xj=%f, p1=%f p2=%f\n", sf2, ell, 1/ell, sn2, xi[dx1] - xj[dx1], xi[dx2] - xj[dx2], sf2 * ell * noise, sf2 * ell * (xi[dx1] - xj[dx1]) * (xi[dx2] - xj[dx2]) * k);
//		return ((ell * dirac - (xi[dx1] - xj[dx1]) * (xi[dx2] - xj[dx2])) * k) / ell;
		return sf2 * ell * (dirac - ell * (xi[dx1] - xj[dx1]) * (xi[dx2] - xj[dx2])) * k;
		//		return noise + (1/ell * k) + (((xj(dx1) - xi(dx1)) * (xj(dx2) - xi(dx2))) / (ell*ell)) * k;
	}

	/** Covariance gradient of two input vectors with respect to the hyperparameters.
	*  @param x1 first input vector
	*  @param x2 second input vector
	*  @param grad covariance gradient */
	virtual void grad(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, Eigen::VectorXd& g) const {
		const double z = ((x1 - x2) / ell).norm();
		const double k = sf2*exp(-0.5*z);
		g << k*z, 2 * k;// , 2 * loghyper(2);
	};

	virtual void gradDiff(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, const int dx, Eigen::VectorXd& g) const {
		const double z = ((x1 - x2) / ell).norm();
		const double k = exp(-0.5*z);
		g << -ell * (x1(dx) - x2(dx) * k), -2 * sf2 * (x1(dx) - x2(dx) * k);// , 2 * loghyper(2);
//		g << -sf2*(x1(dx) - x2(dx))*k - sf2*ell*k*z, 4 * k;// , 2 * loghyper(2);
	};

	virtual void gradDiff2(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, const size_t dx1, const size_t dx2, Eigen::VectorXd& g) const {
		const double z = ((x1 - x2) / ell).norm();
		const double k = exp(-0.5*z);
		const double p = dx1 == dx2 ? sf2 : 0.0;
		const double d = (x1[dx1] - x2[dx1]) * (x1[dx2] - x2[dx2]);
		g << (p - d) * k, ell * ((dx1 == dx2 - ell * d) * k);// , 2 * loghyper(2);
		//		g << p - d*k - k*z*ell*d, 4 * k;// , 2 * loghyper(2);
	};

	/** Update parameter vector.
	*  @param p new parameter vector */
	virtual void setLogHyper(const Eigen::VectorXd &p) {
		BaseCovFunc::setLogHyper(p);
		ell = exp(loghyper(0)); // loghyper(0) * loghyper(0);
		sf2 = exp(2 * loghyper(1)); // loghyper(1) * loghyper(1); //2 * loghyper(1) * loghyper(1);//
//		sn2 = loghyper(2) * loghyper(2); //2 * loghyper(1) * loghyper(1);//
	}

	~CovSE(){}

private:
	/** Hyper-parameters */
	double ell; // first element of loghyper
    double sf2; // second element of loghyper
       
    /** Create from descriptor */
	void create(const Desc& desc) {
		BaseCovFunc::create(desc);
		loghyper.resize(paramDim);
		range.resize(paramDim);
		loghyper(0) =  desc.length;
		loghyper(1) = desc.sigma;
//		loghyper(2) = range(2) = desc.noise;
		ell = exp(loghyper(0)); // loghyper(0) * loghyper(0);
		sf2 = exp(2 * loghyper(1)); // loghyper(1) * loghyper(1); //2 * loghyper(1) * loghyper(1);//
		sn2 = desc.noise * desc.noise;//loghyper(2) * loghyper(2); //2 * loghyper(1) * loghyper(1);//
		range(0) = range(1) = 2.0;
		//		range(2) = 0.003;
	}
        
	CovSE(const golem::Context& context) : BaseCovFunc(context) {}
};

//------------------------------------------------------------------------------

class CovSEArd : public BaseCovFunc {
public:
	typedef boost::shared_ptr<CovSEArd> Ptr;

	/** Descriptor file */
	class Desc : public BaseCovFunc::Desc {
	public:
		typedef boost::shared_ptr<CovSEArd::Desc> Ptr;
		double sigma;
		double length;

		/** Default C'tor */
		Desc() {
			setToDefault();
		}

		/** Set values to default */
		void setToDefault() {
			BaseCovFunc::Desc::setToDefault();
			inputDim = 0;
			paramDim = 1;
			sigma = 1.0;
			length = 1.0;
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(CovSEArd, BaseCovFunc::Ptr, const golem::Context&)

			/** Assert valid descriptor files */
			bool isValid(){
				if (!std::isfinite(sigma) || !std::isfinite(length))
					return false;
				return true;
			}
	};

	/** Get name of the covariance functions */
	virtual std::string getName() const {
		return "CovSEArd";
	}

	/** Compute the kernel */
	virtual double get(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, const bool dirac = false) const {
		const double z = (x1 - x2).cwiseQuotient(ell).norm();
		const double noise = dirac ? sn2 : .0;
		return sf2*exp(-0.5*z) + noise;
	}
	/** SE derivate = -1*invLenght*[sigma_f^2*exp(sqrt((p_1-p_2)'*(p_1-p_2))/(-leng))] */
	virtual inline double getDiff(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx, const bool dirac = false) const {
		const double k = get(xi, xj, dirac);
		return -sf2 * ell[dx] * (xi(dx) - xj(dx)) * k;
	}

	virtual inline double getDiff2(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx1, const size_t dx2, const bool dirac = false) const {
		const double k = get(xi, xj, dirac); //sqrt(DD);
		return sf2 * ell[dx1] * (dirac - ell[dx2] * (xi[dx1] - xj[dx1]) * (xi[dx2] - xj[dx2])) * k;
	}

	/** Covariance gradient of two input vectors with respect to the hyperparameters.
	*  @param x1 first input vector
	*  @param x2 second input vector
	*  @param grad covariance gradient */
	virtual void grad(const Eigen::VectorXd& x1, const Eigen::VectorXd& x2, Eigen::VectorXd& g) const {
		Eigen::VectorXd z = (x1 - x2).cwiseQuotient(ell).array().square();
		const double k = sf2*exp(-0.5*z.sum());
		g.head(inputDim) = z * k;
		g(inputDim) = 2.0 * k;		
	}

	/** Update parameter vector.
	*  @param p new parameter vector */
	virtual void setLogHyper(const Eigen::VectorXd &p) {
		BaseCovFunc::setLogHyper(p);
		for (size_t i = 0; i < inputDim; ++i) 
			ell(i) = exp(loghyper(i)); // loghyper(i) * loghyper(i);
		sf2 = exp(2 * loghyper(inputDim)); // loghyper(inputDim) * loghyper(inputDim);
//		sn2 = loghyper(inputDim + 1) * loghyper(inputDim + 1);
	}

	~CovSEArd(){}

private:
	/** Hyper-parameters */
	Eigen::VectorXd ell; // second element of loghyper
	double sf2; // first element of loghyper

	/** Create from descriptor */
	void create(const Desc& desc) {
		BaseCovFunc::create(desc);
		inputDim = desc.inputDim;
		paramDim = desc.inputDim + 1;
		loghyper.resize(paramDim);
		ell.resize(inputDim);
		for (size_t i = 0; i < inputDim; ++i) {
			loghyper(i) = desc.length;
			ell(i) = exp(loghyper(i));  // loghyper(i) * loghyper(i);
		}
		loghyper(inputDim) = desc.sigma;
		sf2 = exp(2 * loghyper(inputDim)); //loghyper(inputDim) * loghyper(inputDim);
		sn2 = desc.noise * desc.noise;//loghyper(2) * loghyper(2); //2 * loghyper(1) * loghyper(1);//
	}

	CovSEArd(const golem::Context& context) : BaseCovFunc(context) {}
};

//------------------------------------------------------------------------------

class CovSESC : public BaseCovFunc {
public:
	typedef boost::shared_ptr<CovSESC> Ptr;

	/** Spherical coordinate */
	class SphericalCoord {
	public:
		golem::Real radius;
		golem::Real theta;
		golem::Real phi;

		SphericalCoord() {
			radius = theta = phi = golem::REAL_ZERO;
		}
		SphericalCoord(const Eigen::VectorXd& point) {
			radius = point.norm();
			theta = golem::Math::atan2(point(1), point(0));
			phi = golem::Math::acos(point(3) / radius);
		}

		/** Converts to a Cartesian coordinate */
		golem::Vec3 toCartesian(const SphericalCoord& v) {
			golem::Vec3 point;
			point.x = golem::Math::cos(theta) * golem::Math::cos(phi) * radius;
			point.y = golem::Math::sin(theta) * golem::Math::cos(phi) * radius;
			point.z = golem::Math::sin(phi) * radius;
			return point;
		}

		/** Distance as length of arc */
		golem::Real distance(const SphericalCoord& v) {
			const golem::Real sinTh = golem::Math::sin(theta);
			const golem::Real sinTh2 = golem::Math::sin(v.theta);
			const golem::Real cosTh = golem::Math::cos(theta);
			const golem::Real cosTh2 = golem::Math::cos(v.theta);
			const golem::Real sinPhi = golem::Math::sin(phi);
			const golem::Real sinPhi2 = golem::Math::sin(v.phi);
			const golem::Real cosPhi = golem::Math::cos(phi - v.phi); //golem::Math::cos(phi);
			const golem::Real cosPhi2 = golem::Math::cos(v.phi);

			//return sqrt(radius * radius + v.radius * v.radius - 2 * radius * v.radius * (sinTh * sinTh2 * cosPhi * cosPhi2 + sinTh * sinTh2 * sinPhi * sinPhi2 + cosTh * cosTh2));
			return sqrt(radius * radius + v.radius * v.radius - 2 * radius * v.radius * (sinTh * sinTh2 * cosPhi + cosTh * cosTh2));
		}

		golem::Real diff(const int dx, const SphericalCoord& v) {
			golem::Real der = golem::REAL_ZERO;
			if (dx == 0) {
				der = radius - v.radius;
			}
			else if (dx == 1) {
				der = theta - v.theta;
			}
			else if (dx == 2) {
				der = phi - v.phi;
			}
			return der;

		}
		/** Partial derivative */
		golem::Real derive(const int dx, const SphericalCoord& v) {
			const golem::Real dr = golem::REAL_ONE; // radius - v.radius;
			const golem::Real dTh = golem::REAL_ONE; //theta - v.theta;
			const golem::Real dPhi = golem::REAL_ONE; //phi - v.phi;
			const golem::Real sinTh = golem::Math::sin(theta);
			const golem::Real cosTh = golem::Math::cos(theta);
			const golem::Real sinPhi = golem::Math::sin(phi);
			const golem::Real cosPhi = golem::Math::cos(phi);
			golem::Real der = golem::REAL_ZERO;
			if (dx == 0) {
				der = cosTh * sinPhi * dr - radius * sinTh * sinPhi * dTh + radius * cosTh * cosPhi * dPhi;
			}
			else if(dx == 1) {
				der = sinTh * sinPhi * dr + radius * cosTh * sinPhi * dTh + radius * sinTh * cosPhi * dPhi;
			}
			else if (dx == 2) {
				der = cosPhi * dr - radius * sinPhi * dPhi;
			}
			return der;
		}
		/** Partial derivative */
		golem::Real derive(const int dd, const SphericalCoord& v, const Eigen::VectorXd& xi, const Eigen::VectorXd& xj) {
			const golem::Real dx = golem::REAL_ONE; //xi(0) - xj(0);
			const golem::Real dy = golem::REAL_ONE; //xi(1) - xj(1);
			const golem::Real dz = golem::REAL_ONE; //xi(2) - xj(2);
			const golem::Real sinTh = golem::Math::sin(theta);
			const golem::Real cosTh = golem::Math::cos(theta);
			const golem::Real sinPhi = golem::Math::sin(phi);
			const golem::Real cosPhi = golem::Math::cos(phi);
			golem::Real der = golem::REAL_ZERO;
			if (dd == 0) {
				der = cosTh * sinPhi * dx - sinTh * sinPhi * dy + cosTh * dz;
			}
			else if (dd == 1) {
				der = - (sinTh / (radius * sinPhi)) * dx + (cosTh / (radius *sinPhi)) * dy;
			}
			else if (dd == 2) {
				der = ((cosTh * cosPhi) / radius) * dx - ((sinTh * cosPhi) / radius) * dy - (sinPhi / radius) * dz;
			}
			return der;
		}

	};
	/** Descriptor file */
	class Desc : public BaseCovFunc::Desc {
	public:
		typedef boost::shared_ptr<CovSESC::Desc> Ptr;
		double sigma;
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
			sigma = 1.0;
			length = 1.0;
		}

		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(CovSESC, BaseCovFunc::Ptr, const golem::Context&)

			/** Assert valid descriptor files */
			bool isValid(){
				if (!std::isfinite(sigma) || !std::isfinite(length))
					return false;
				return true;
			}
	};

	/** Get name of the covariance functions */
	virtual std::string getName() const {
		return "CovSE with Spherical Coordinates";
	}

	/** Compute the kernel */
	virtual double get(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const bool dirac = false) const {
		SphericalCoord spi(xi), spj(xj);
		const double z = abs(spi.distance(spj) / ell);
		const double noise = dirac ? sn2 : .0;
		return sf2*exp(-0.5*z) + noise;
	}

	/** SE derivate = -1*invLenght*[sigma_f^2*exp(sqrt((p_1-p_2)'*(p_1-p_2))/(-leng))] */
	virtual inline double getDiff(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const int dx, const bool dirac = false) const {
		const double k = get(xi, xj, dirac); //sqrt(DD);
		// if dx < 0 then I compute the sum of the partial derivative
		SphericalCoord spi(xi), spj(xj);
//		return dx < 0 ? -sf2 * ell * k : -sf2 * ell * spi.derive(dx, spj, xi, xj) * k;
//		return dx < 0 ? -sf2 * ell * k : -sf2 * ell * spi.derive(dx, spj) * k;
		return dx < 0 ? -sf2 * ell * k : -sf2 * ell * spi.diff(dx, spj) * k;
		//-golem::REAL_ONE * ell * k
	}

	virtual inline double getDiff2(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, const size_t dx1, const size_t dx2, const bool dirac = false) const {
		const double k = get(xi, xj, dirac); //sqrt(DD);
		//const double noise = dirac ? sn2 : .0;
		//		printf("sf2=%f ell=%f 1/ell=%f noise=%f xi-xj=%f xi-xj=%f, p1=%f p2=%f\n", sf2, ell, 1/ell, sn2, xi[dx1] - xj[dx1], xi[dx2] - xj[dx2], sf2 * ell * noise, sf2 * ell * (xi[dx1] - xj[dx1]) * (xi[dx2] - xj[dx2]) * k);
		SphericalCoord spi(xi), spj(xj);
//		context.write("dx1=%f dx2=%f\n", spi.derive(dx1, spj, xi, xj), spi.derive(dx2, spj, xi, xj));
//		return sf2 * ell * (dirac - ell * spi.derive(dx1, spj, xi, xj) * spi.derive(dx2, spj, xi, xj)) * k;
//		return sf2 * ell * (dirac - ell * spi.derive(dx1, spj) * spi.derive(dx2, spj)) * k;
		return sf2 * ell * (dirac - ell * spi.diff(dx1, spj) * spi.diff(dx2, spj)) * k;
		//		return noise + (1/ell * k) + (((xj(dx1) - xi(dx1)) * (xj(dx2) - xi(dx2))) / (ell*ell)) * k;
	}

	/** Covariance gradient of two input vectors with respect to the hyperparameters.
	*  @param x1 first input vector
	*  @param x2 second input vector
	*  @param grad covariance gradient */
	virtual void grad(const Eigen::VectorXd& xi, const Eigen::VectorXd& xj, Eigen::VectorXd& g) const {
		SphericalCoord spi(xi), spj(xj);
		const double z = abs(spi.distance(spj) / ell);
		const double k = sf2*exp(-0.5*z);
		g << k*z, 2 * k;// , 2 * loghyper(2);
	};

	/** Update parameter vector.
	*  @param p new parameter vector */
	virtual void setLogHyper(const Eigen::VectorXd &p) {
		BaseCovFunc::setLogHyper(p);
		ell = exp(loghyper(0)); // loghyper(0) * loghyper(0);
		sf2 = exp(2 * loghyper(1)); // loghyper(1) * loghyper(1); //2 * loghyper(1) * loghyper(1);//
		//		sn2 = loghyper(2) * loghyper(2); //2 * loghyper(1) * loghyper(1);//
	}

	~CovSESC(){}

private:
	/** Hyper-parameters */
	double ell; // first element of loghyper
	double sf2; // second element of loghyper

	/** Create from descriptor */
	void create(const Desc& desc) {
		BaseCovFunc::create(desc);
		loghyper.resize(paramDim);
		range.resize(paramDim);
		loghyper(0) = desc.length;
		loghyper(1) = desc.sigma;
		//		loghyper(2) = range(2) = desc.noise;
		ell = exp(loghyper(0)); // loghyper(0) * loghyper(0);
		sf2 = exp(2 * loghyper(1)); // loghyper(1) * loghyper(1); //2 * loghyper(1) * loghyper(1);//
		sn2 = desc.noise * desc.noise;//loghyper(2) * loghyper(2); //2 * loghyper(1) * loghyper(1);//
		range(0) = range(1) = 1.2;
		//		range(2) = 0.003;
	}

	CovSESC(const golem::Context& context) : BaseCovFunc(context) {}
};

//------------------------------------------------------------------------------

}

#endif
