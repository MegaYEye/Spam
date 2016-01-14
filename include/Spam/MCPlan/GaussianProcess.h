/** @file GaussianProcess.h
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
#ifndef __GP_GAUSSIANPROCESS_H__
#define __GP_GAUSSIANPROCESS_H__
#define _USE_MATH_DEFINES

//------------------------------------------------------------------------------

#include <cstdio>
#include <cmath>
#include <Eigen/Dense>
#include <Spam/MCPlan/SampleSet.h>
#include <Spam/MCPlan/CovLaplace.h>
#include <Spam/MCPlan/CovThinPlate.h>
#include <Spam/MCPlan/CovSE.h>
#include <omp.h>
#include <Grasp/Core/Defs.h>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

/** Utily constants */
static const golem::Real log2pi = std::log(golem::numeric_const<golem::Real>::TWO_PI);
template <class CovTypePtr, class CovTypeDesc> class GaussianProcess;

//------------------------------------------------------------------------------

/** Gradient-based optimizer (RProp). */
class Optimisation {
public:
	/** Pointer */
	typedef boost::shared_ptr<Optimisation> Ptr;

	/** Description file */
	class Desc {
	public:
		/** Pointer to the descriptor */
		typedef boost::shared_ptr<Desc> Ptr;

		golem::U32 populationSize;

		golem::Real delta0;
		golem::Real deltaMin;
		golem::Real deltaMax;
		golem::Real etaMinus;
		golem::Real etaPlus;
		golem::Real epsStop;
		size_t maxIter;

		Desc() {
			setToDefault();
		}
		void setToDefault() {
			populationSize = 10;

			delta0 = 0.01;
			deltaMin = 1e-6;
			deltaMax = 5;
			etaMinus = 0.05;
			etaPlus = 0.2;
			epsStop = 1e-4;

			maxIter = 100;
		}
		bool isValid() {
			return true;
		}
		/** Creates the object from the description. */
		Optimisation::Ptr create(const golem::Context& context) const {
			Optimisation::Ptr pointer(new Optimisation(context));
			pointer->create(*this);
			return pointer;
		}
	};

	template <typename CovTypePtr, typename CovTypeDesc> void find(GaussianProcess<CovTypePtr, CovTypeDesc>* gp, bool verbose = true) {
		clock_t t = clock();

		golem::CriticalSection cs;
		size_t k = 0;
		int paramDim = gp->cf->getParamDim();
		Real solutionEval = log(0);
		Eigen::VectorXd bestParams = gp->cf->getLogHyper();
		context.write("Optimisation::find(): iter=0 params size=%d\n", bestParams.size());
		for (size_t i = 0; i < bestParams.size(); ++i)
			context.write("params[%d] = %f\n", i, bestParams(i));

		grasp::ParallelsTask(context.getParallels(), [&](grasp::ParallelsTask*) {
			Real testEval = log(0);
			Eigen::VectorXd delta = Eigen::VectorXd::Ones(paramDim) * delta0;
			Eigen::VectorXd gradOld = Eigen::VectorXd::Zero(paramDim);
			Eigen::VectorXd testParams = bestParams;

			for (;;) {
				{
					CriticalSectionWrapper csw(cs);
					if (solutionEval < testEval) {
						context.debug("Solution %f params [%f %f %f]\n", testEval, testParams(0), testParams(1), 0/*, testParams(2)*/);
						solutionEval = testEval;
						bestParams = testParams;
						gp->cf->setLogHyper(bestParams);
					}
					if (++k > desc.populationSize)
						break;
				}

				Eigen::VectorXd params = gp->cf->sampleParams();
//				context.debug("Init params [%f %f]\n", params(0), params(1));
				for (size_t i = 0; i < maxIter; ++i) {
					Eigen::VectorXd grad = -gp->logLikelihoodGradient();
					gradOld = gradOld.cwiseProduct(grad);
					for (int j = 0; j < gradOld.size(); ++j) {
						if (gradOld(j) > 0) {
							delta(j) = std::min(delta(j)*etaPlus, deltaMax);
						}
						else if (gradOld(j) < 0) {
							delta(j) = std::max(delta(j)*etaMinus, deltaMin);
							grad(j) = 0;
						}
						params(j) += -sign(grad(j)) * delta(j);
					}
					gradOld = grad;
					if (gradOld.norm() < epsStop) break;
					double eval = gp->logLikelihood();
					if (testEval < eval) {
						testEval = eval;
						testParams = params;
					}
					CriticalSectionWrapper csw(cs);
					gp->cf->setLogHyper(params);
				}
			}
		});

		//gp->cf->setLogHyper(bestParams);
		context.write("Optimisation::find(): Elapsed time: %.4fs\n", (float)(clock() - t) / CLOCKS_PER_SEC);
		context.write("Optimisation::find(): best solutionEval=%f params size=%d\n", solutionEval, bestParams.size());
		for (size_t i = 0; i < bestParams.size(); ++i)
			context.write("params[%d] = %f\n", i, bestParams(i));

	}

	//template <typename CovTypePtr, typename CovTypeDesc> void find(GaussianProcess<CovTypePtr, CovTypeDesc>* gp, bool verbose = true) {
	//	clock_t t = clock();
	//	int paramDim = gp->cf->getParamDim();
	//	Eigen::VectorXd delta = Eigen::VectorXd::Ones(paramDim) * delta0;
	//	Eigen::VectorXd gradOld = Eigen::VectorXd::Zero(paramDim);
	//	Eigen::VectorXd params = gp->cf->getLogHyper();
	//	Eigen::VectorXd bestParams = params;
	//	double best = log(0);

	//	context.write("Optimisation::find(): iter=0 params size=%d\n", params.size());
	//	for (size_t i = 0; i < params.size(); ++i)
	//		context.write("params[%d] = %f\n", i, params(i));

	//	for (size_t i = 0; i < maxIter; ++i) {
	//		Eigen::VectorXd grad = -gp->logLikelihoodGradient();
	//		gradOld = gradOld.cwiseProduct(grad);
	//		for (int j = 0; j < gradOld.size(); ++j) {
	//			if (gradOld(j) > 0) {
	//				delta(j) = std::min(delta(j)*etaPlus, deltaMax);
	//			}
	//			else if (gradOld(j) < 0) {
	//				delta(j) = std::max(delta(j)*etaMinus, deltaMin);
	//				grad(j) = 0;
	//			}
	//			params(j) += -sign(grad(j)) * delta(j);
	//		}
	//		gradOld = grad;
	//		if (gradOld.norm() < epsStop) break;
	//		gp->cf->setLogHyper(params);
	//		double lik = gp->logLikelihood();
	//		if (verbose)
	//			context.write("Optimisation::find(): iter=%d lik=%f params[l,s]=[%f, %f]\n", i+1, lik, params(0), params(1));
	//		//std::cout << i << " " << -lik << std::endl;
	//		if (lik > best) {
	//			best = lik;
	//			bestParams = params;
	//		}
	//	}
	//	gp->cf->setLogHyper(bestParams);
	//	context.write("Optimisation::find(): Elapsed time: %.4fs\nbest lik=%f params size=%d\n", (float)(clock() - t) / CLOCKS_PER_SEC, best, bestParams.size());
	//	for (size_t i = 0; i < bestParams.size(); ++i)
	//		context.write("params[%d] = %f\n", i, bestParams(i));

	//}

	/** D'ctor */
//	~Optimisation() {} // don't need to do anything

protected:
	/** Context */
	golem::Context context;
	/** Description file */
	Desc desc;

	/** Optimisation parameters */
	golem::Real delta0;
	golem::Real deltaMin;
	golem::Real deltaMax;
	golem::Real etaMinus;
	golem::Real etaPlus;
	golem::Real epsStop;

	size_t maxIter;

	double sign(double x) {
		if (x > 0) return 1.0;
		if (x < 0) return -1.0;
		return 0.0;
	}

	/** Create from descriptor */
	void create(const Desc& desc) {
		this->desc = desc;

		delta0 = desc.delta0;
		deltaMin = desc.deltaMin;
		deltaMax = desc.deltaMax;
		etaMinus = desc.etaMinus;
		etaPlus = desc.etaPlus;
		epsStop = desc.epsStop;

		maxIter = desc.maxIter;
	};
	/** Default C'ctor */
	Optimisation(const golem::Context& context) : context(context) {}
};

//------------------------------------------------------------------------------

/*
    * Gaussian Process.
    *
    */
template <class CovTypePtr, class CovTypeDesc> class GaussianProcess {
public:
    /** Needed for good alignment for the Eigen's' data structures */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /** Pointer to the class */
    typedef boost::shared_ptr<GaussianProcess<CovTypePtr, CovTypeDesc> > Ptr;

	friend class Optimisation;

    /** Descriptor file */
    class Desc {
    public:
        /** Initial size of the kernel matrix */
		golem::U32 initialLSize;
        /** Covariance description file */
        CovTypeDesc covTypeDesc;

		/** Enable optimisation */
		bool optimise;
		/** Optimisation procedure descriptor file */
		Optimisation::Desc::Ptr optimisationDescPtr;

		/** Enable atlas */
		bool atlas;

        /** C'tor */
        Desc() {
            setToDefault();
        }

        /** Set to default */
        void setToDefault() {
            initialLSize = 1500;
            covTypeDesc.setToDefault();

			optimise = true;
			optimisationDescPtr.reset(new Optimisation::Desc);

			atlas = true;
        }

        /** Creates the object from the description. */
        Ptr create(const golem::Context& context) const {
			Ptr pointer(new GaussianProcess<CovTypePtr, CovTypeDesc>(context));
            pointer->create(*this);
            return pointer;
        }

        /** Assert valid descriptor files */
        bool isValid(){
            if (!std::isfinite(initialLSize))
                return false;
            if (noise < golem::REAL_ZERO)
                return false;
            return true;
        }
    };

    /** Predict f_* ~ GP(x_*) */
	void evaluate(const Vec3Seq& x, std::vector<golem::Real>& fx, std::vector<golem::Real>& vars) {
		const size_t size = x.size();
		fx.assign(size, REAL_ZERO);
		vars.assign(size, REAL_ZERO);
		for (size_t i = 0; i < size; ++i) {
			fx[i] = f(x[i]);
			vars[i] = var(x[i]);
		}
	}
    virtual golem::Real f(const golem::Vec3& xStar) {
        if (sampleset->empty()) return 0;

        //Eigen::Map<const Eigen::VectorXd> x_star(x.v, input_dim);
        compute(true);
        update_alpha();
        update_k_star(xStar);
        //std::cout << "size alpha=" << alpha->size() << " k_star=" << k_star->size() << std::endl;
        return k_star->dot(*alpha);
    }

    /** Predict variance v[f_*] ~ var(x_*)  */
	virtual golem::Real var(const golem::Vec3& xStar) {
        if (sampleset->empty()) return 0;

        //Eigen::Map<const Eigen::VectorXd> x_star(x.v, input_dim);
        compute(true);
        update_alpha();
        update_k_star(xStar); //update_k_star(x_star);
        size_t n = sampleset->rows();
        Eigen::VectorXd v = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(*k_star);
		return cf->get(xStar, xStar) - v.dot(v); //cf->get(x_star, x_star) - v.dot(v);
    }

	/** Predict f, var, N, Tx and Ty */
//	virtual void evaluate(const golem::Vec3& x, golem::Real& fx, golem::Real& varx, Eigen::Vector3d& normal, Eigen::Vector3d& tx, Eigen::Vector3d& ty) {
//		clock_t t = clock();
//
//		fx = f(x);
//		varx = var(x);
//
//		const size_t n = sampleset->rows(), nnew = sampleset->rows() + 1;
//		const size_t dim = sampleset->cols();
//
//		// differential of covariance with selected kernel
//		Eigen::MatrixXd Kppdiff, Kpp;
//		Kppdiff.resize(nnew, nnew);
//		Kpp.resize(nnew, nnew);
//		N.resize(nnew, dim);
//		Tx.resize(nnew, dim);
//		Ty.resize(nnew, dim);
//
//		for (size_t i = 0; i < nnew; ++i) {
//			for (size_t j = 0; j <= i; ++j) {
//				const double k_ij = cf->get(sampleset->x(i), sampleset->x(j));
//				// add noise on the diagonal of the kernel
//				Kpp(i, j) = i == j ? k_ij + delta_n : k_ij;
//				Kppdiff(i, j) = cf->getDiff(sampleset->x(i), sampleset->x(j), i == j ? delta_n : .0);
//				//			context.write("GP::compute(): Computing k(%lu, %lu)\r", i, j);
//			}
//		}
//		const Eigen::MatrixXd invKpp = Kpp.inverse();
//		InvKppY = invKpp * convertToEigen(sampleset->y());
//		//#pragma omp parallel for
//		for (size_t i = 0; i < nnew; ++i) {
//			for (size_t j = 0; j <= i; ++j)
//				N.row(i) += InvKppY(j)*Kppdiff(i, j)*(convertToEigen(sampleset->x(i) - sampleset->x(j)));
//
//			N.row(i).normalize();
////			context.write("N[%d] = [%f %f %f]\n", i, N(i, 0), N(i, 1), N(i, 2));
//			Eigen::Vector3d Ni = N.row(i);
//			Eigen::Vector3d Txi, Tyi;
//			computeTangentBasis(Ni, Txi, Tyi);
//			Tx.row(i) = Txi;
//			Ty.row(i) = Tyi;
//		}
//		normal = N.row(nnew - 1);
//		//context.write("GP::evaluate(): Elapsed time: %.4fs\n", (float)(clock() - t) / CLOCKS_PER_SEC);
//	}
//	virtual void evaluate(Eigen::MatrixXd& normal, Eigen::MatrixXd& tx, Eigen::MatrixXd& ty) {
//		clock_t t = clock();
//
//		const size_t n = sampleset->rows();
//		const size_t dim = sampleset->cols();
//
//		// differential of covariance with selected kernel
//		Eigen::MatrixXd Kppdiff;
//		Kppdiff.resize(n, n);
//		N.resize(n, dim);
//		Tx.resize(n, dim);
//		Ty.resize(n, dim);
//
//		for (size_t i = 0; i < n; ++i) {
//			for (size_t j = 0; j <= i; ++j) {
//				Kppdiff(i, j) = cf->getDiff(sampleset->x(i), sampleset->x(j)/*, i == j ? delta_n : .0*/);
////				context.write("KppDiff[%d, %d] = %f\n", i, j, Kppdiff(i, j));
//			}
////			context.write("InvKppY[%d] = %f\n", i, InvKppY(i));
//		}
//		Real error = REAL_ZERO;
//		//#pragma omp parallel for
//		for (size_t i = 0; i < n; ++i) {
//			for (size_t j = 0; j <= i; ++j)
//				N.row(i) += InvKppY(j)*Kppdiff(i, j)*(convertToEigen(sampleset->x(i) - sampleset->x(j)));
//				
//			N.row(i).normalize();
//			Vec3 nn(N(i, 0), N(i, 1), N(i, 2));
//			context.write("Error[%d] = %f \n", i, sampleset->x(i).distance(nn));
//			error += sampleset->x(i).distance(nn);
//			Eigen::Vector3d Ni = N.row(i);
//			Eigen::Vector3d Txi, Tyi;
//			computeTangentBasis(Ni, Txi, Tyi);
//			Tx.row(i) = Txi;
//			Ty.row(i) = Tyi;
//		}
//		normal = N;
//		tx = Tx;
//		ty = Ty;
//		context.write("GP::evaluate(): Elapsed time: %.4fs. Tot error %f\n", (float)(clock() - t) / CLOCKS_PER_SEC, error / n);
//	}
	virtual void evaluate(const Vec3Seq& x, std::vector<golem::Real>& fx, std::vector<golem::Real>& varx, Eigen::MatrixXd& normals, Eigen::MatrixXd& tx, Eigen::MatrixXd& ty) {
		clock_t t = clock();

		// compute f(x) and V(x)
		evaluate(x, fx, varx);

		const size_t np = sampleset->rows(), nq = x.size(), n = sampleset->rows() + x.size();
		const size_t dim = sampleset->cols();

		// differential of covariance with selected kernel
		Eigen::MatrixXd Kqp, Kqpdiff;
		// row = number of query points
		// col = number of points in the kernels + 1
		Kqp.resize(nq, np + 1);
		Kqpdiff.resizeLike(Kqp);
		normals.resize(nq, dim);
		tx.resize(nq, dim);
		ty.resize(nq, dim);

		// compute Kqp and KqpDiff
		for (size_t i = 0; i < nq; ++i) {
			for (size_t j = 0; j < np; ++j) {
				Kqp(i, j) = cf->get(x[i], sampleset->x(j));
				Kqpdiff(i, j) = cf->getDiff(x[i], sampleset->x(j));
			}
		}
		// compute error
		golem::Real error = REAL_ZERO;
		//#pragma omp parallel for
		for (size_t i = 0; i < nq; ++i) {
			for (size_t j = 0; j < np; ++j)
				normals.row(i) += InvKppY(j)*Kqpdiff(i, j)*(convertToEigen(x[i] - sampleset->x(j)));

			normals.row(i).normalize();
			golem::Vec3 ni(normals(i, 0), normals(i, 1), normals(i, 2)), p = x[i];
			p.normalise();
			error += p.distance(ni);
			//			context.write("N[%d] = [%f %f %f]\n", i, N(i, 0), N(i, 1), N(i, 2));
			Eigen::Vector3d Ni = normals.row(i);
			Eigen::Vector3d Txi, Tyi;
			computeTangentBasis(Ni, Txi, Tyi);
			tx.row(i) = Txi;
			ty.row(i) = Tyi;
		}
		context.write("GP::evaluate(): Error=%d\nElapsed time: %.4fs\n", error / nq, (float)(clock() - t) / CLOCKS_PER_SEC);
	}

    /** Set training data */
    void set(SampleSet::Ptr trainingData) {
        sampleset = trainingData;
		// param optimisation
		if (desc.optimise)
			optimisationPtr->find<CovTypePtr, CovTypeDesc>(this);
    }

    /** Get name of the covariance function */
    std::string getName() const {
        return cf->getName();
    }

    /** Add input-output pairs to sample set. */
    void add_patterns(const Vec3Seq& newInputs, const Vec& newTargets) {
        assert(newInputs.size() == newTargets.size());

        // the size of the inputs before adding new samples
        const size_t n = sampleset->rows();
        sampleset->add(newInputs, newTargets);

        // create kernel matrix if sampleset is empty
        if (n == 0) {
            cf->setLogHyper(true);
            compute();
        }
        else {
            clock_t t = clock();
            const size_t nnew = sampleset->rows();
            // resize L if necessary
            if (sampleset->rows() > static_cast<std::size_t>(L->rows())) {
                L->conservativeResize(nnew + initialLSize, nnew + initialLSize);
            }
//#pragma omp parallel for
            for (size_t j = n; j < nnew; ++j) {
                Eigen::VectorXd k(j);
                for (size_t i = 0; i<j; ++i) {
                    k(i) = cf->get(sampleset->x(i), sampleset->x(j));
                    //context.write("GP::add_patters(): Computing k(%lu, %lu)\r", i, j);
                }
                const double kappa = cf->get(sampleset->x(j), sampleset->x(j));
                L->topLeftCorner(j, j).triangularView<Eigen::Lower>().solveInPlace(k);
                L->block(j,0,1,j) = k.transpose();
                (*L)(j,j) = sqrt(kappa - k.dot(k));
            }
            context.write("GP::add_patterns(): Elapsed time: %.4fs\n", (float)(clock() - t)/CLOCKS_PER_SEC);
        }
        alpha_needs_update = true;
    }

    /** Compute loglikelihood */
    golem::Real logLikelihood() {
        compute();
        update_alpha();
        int n = sampleset->rows();
		const std::vector<golem::Real>& targets = sampleset->y();
        Eigen::Map<const Eigen::VectorXd> y(&targets[0], sampleset->rows());
		golem::Real det = (golem::Real)(2 * L->diagonal().head(n).array().log().sum());
		return (golem::Real)(- 0.5*y.dot(*alpha) - 0.5*det - 0.5*n*log2pi);
    }

	Eigen::VectorXd logLikelihoodGradient()
	{
		compute(false);
		update_alpha();
		size_t n = sampleset->rows();
		Eigen::VectorXd grad = Eigen::VectorXd::Zero(cf->getParamDim());
		Eigen::VectorXd g(grad.size());
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(n, n);

		// compute kernel matrix inverse
		L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solveInPlace(W);
		L->topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(W);

		W = (*alpha) * alpha->transpose() - W;

		for (size_t i = 0; i < n; ++i) {
			for (size_t j = 0; j <= i; ++j) {
				cf->grad(sampleset->x(i), sampleset->x(j), g);
				if (i == j) grad += W(i, j) * g * 0.5;
				else grad += W(i, j) * g;
			}
		}

		return grad;
	}

	inline Eigen::MatrixXd getNormals() const { return N; }

    /** D'tor */
//    virtual ~GaussianProcess() {};

protected:
	/** Context */
	golem::Context context;
	/** Desriptor file */
	Desc desc;

	/** Optimisation procedure */
	Optimisation::Ptr optimisationPtr;

    /** pointer to the covariance function type */
    CovTypePtr cf;
    /** The training sample set. */
    boost::shared_ptr<SampleSet> sampleset;
    /** Alpha is cached for performance. */
    boost::shared_ptr<Eigen::VectorXd> alpha;
    /** Last test kernel vector. */
    boost::shared_ptr<Eigen::VectorXd> k_star;
	/** Linear solver used to invert the covariance matrix. */
    // Eigen::LLT<Eigen::MatrixXd> solver;
	/** Lower triangle matrix of kernel (points) */
    boost::shared_ptr<Eigen::MatrixXd> L;
	/** Input vector dimensionality. */
    size_t input_dim;
    /** Enable/disable to update the alpha vector */
	bool alpha_needs_update;
    //initial L size
	golem::U32 initialLSize;

	/** (inward) normal sequence */
	Eigen::MatrixXd N; 
	/** Tangent basis in x direction */
	Eigen::MatrixXd Tx;
	/** Tangen basis in y direction */
	Eigen::MatrixXd Ty;
	/** Weights */
	Eigen::VectorXd InvKppY;
	/** Derivative of the kernel */
	Eigen::MatrixXd Kppdiff;

    /** Compute k_* = K(x_*, x) */
	void update_k_star(const golem::Vec3& x_star) {
        k_star->resize(sampleset->rows());
		for (size_t i = 0; i < sampleset->rows(); ++i) {
            (*k_star)(i) = cf->get(x_star, sampleset->x(i));
		}
    }

    /** Update alpha vector (mean) */
    void update_alpha() {
        // can previously computed values be used?
        if (!alpha_needs_update) return;
        alpha_needs_update = false;
        alpha->resize(sampleset->rows());
		// Map target values to VectorXd
        const std::vector<double>& targets = sampleset->y();
        Eigen::Map<const Eigen::VectorXd> y(&targets[0], sampleset->rows());
		size_t n = sampleset->rows();
        *alpha = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(y);
		L->topLeftCorner(n, n).triangularView<Eigen::Lower>().adjoint().solveInPlace(*alpha);
	}

    /** Compute covariance matrix and perform cholesky decomposition. */
    virtual void compute(const bool verbose = false) {
        // can previously computed values be used?
        if (!cf->isLogHyper()) return;
        clock_t t = clock();
        cf->setLogHyper(false);
		// input size
        const size_t n = sampleset->rows();
		const size_t dim = sampleset->cols();
		// resize L if necessary
        if (n > L->rows()) L->resize(n + initialLSize, n + initialLSize);
        // compute kernel matrix (lower triangle)
        size_t counter = 0;
		Kppdiff.resize(n, n);
        //#pragma omp parallel for
#pragma omp parallel for collapse(2)
        for(size_t i = 0; i < n; ++i) {
            for(size_t j = 0; j <= i; ++j) {
                // compute kernel and add noise on the diagonal of the kernel
				(*L)(i, j) = cf->get(sampleset->x(i), sampleset->x(j), i == j);
				Kppdiff(i, j) = cf->getDiff(sampleset->x(i), sampleset->x(j));
				//context.write("GP::compute(): Computing k(%lu, %lu) = %f\r", i, j, (*L)(i, j));
			}
		}		
		InvKppY = L->topLeftCorner(n, n).inverse() * convertToEigen(sampleset->y());
		// perform cholesky factorization
        //solver.compute(K.selfadjointView<Eigen::Lower>());
        L->topLeftCorner(n, n) = L->topLeftCorner(n, n).selfadjointView<Eigen::Lower>().llt().matrixL();
		alpha_needs_update = true;
		if (verbose) context.write("GP::Compute(): Elapsed time: %.4fs\n", (float)(clock() - t) / CLOCKS_PER_SEC);
    }

	/** Compute tangent basis. N must be normalised */
	void computeTangentBasis(const Eigen::Vector3d &N, Eigen::Vector3d &Tx, Eigen::Vector3d &Ty)
	{
		Eigen::Matrix3d TProj = Eigen::Matrix3d::Identity() - N*N.transpose();
		Eigen::JacobiSVD<Eigen::Matrix3d> svd(TProj, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Tx = svd.matrixU().col(0);
		Ty = svd.matrixU().col(1);
	}

    /** Create from description file */
    void create(const Desc& desc) {
		this->desc = desc;

		optimisationPtr = desc.optimisationDescPtr->create(context);

        cf = desc.covTypeDesc.create();
        //sampleset = desc.trainingData;
        input_dim = 3;
        initialLSize = desc.initialLSize;
        alpha.reset(new Eigen::VectorXd);
        k_star.reset(new Eigen::VectorXd);
        L.reset(new Eigen::MatrixXd);
        L->resize(initialLSize, initialLSize);
        alpha_needs_update = true;
	}

    /** Default C'tor */
	GaussianProcess(const golem::Context& context) : context(context) {}
};

//------------------------------------------------------------------------------

/** List of legal types */
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::Laplace::Desc> LaplaceRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::CovSE::Desc> GaussianRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::CovSEArd::Desc> GaussianARDRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::ThinPlate::Desc> ThinPlateRegressor;
//typedef GaussianProcess<gp::Laplace::Ptr, gp::Laplace::Desc> LaplaceRegressor;
//typedef GaussianProcess<gp::ThinPlate> ThinPlateRegressor;
//class LaplaceRegressor : public GaussianProcess<gp::Laplace> {};

//------------------------------------------------------------------------------

}
#endif /* __GP_GAUSSIANPROCESS_H__ */
