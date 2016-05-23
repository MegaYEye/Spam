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
#include <Grasp/Core/RB.h>

//------------------------------------------------------------------------------
#if EIGEN_FAST_MATH > 0
#undef EIGEN_FAST_MATH
#define EIGEN_FAST_MATH 0
#endif

namespace spam {

//------------------------------------------------------------------------------

/** Utily constants */
static const golem::Real log2pi = std::log(golem::numeric_const<golem::Real>::TWO_PI);
template <class CovTypePtr, class CovTypeDesc> class GaussianProcess;
template <class CovTypePtr, class CovTypeDesc> class SimpleGP;

//------------------------------------------------------------------------------

static golem::Real determinant(const Eigen::MatrixXd& M, golem::Real& maxEVR, golem::Real& maxEVI, golem::Real& minEVR, golem::Real& minEVI) {
	Eigen::EigenSolver<Eigen::MatrixXd> es(M);
	Eigen::VectorXcd ei = es.eigenvalues();
	golem::Real det = golem::REAL_ONE, maxR = -golem::REAL_MAX, maxI = -golem::REAL_MAX, minR = golem::REAL_MAX, minI = golem::REAL_MAX;
	for (size_t i = 0; i < ei.size(); ++i) {
		const golem::Real mr = ei(i).real();
		const golem::Real mi = ei(i).imag();
		if (maxR < mr) maxR = mr;
		if (minR > mr) minR = mr;
		if (maxI < mi) maxI = mi;
		if (minI > mi) minI = mi;
		det *= ei(i).real();
	}
	maxEVR = maxR; minEVR = minR; maxEVI = maxI; minEVI = minI;
	return (golem::Real)det;
}
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

		golem::U32 minPopulation;
		golem::U32 populationSize;

		golem::Real delta0;
		golem::Real deltaMin;
		golem::Real deltaMax;
		golem::Real etaMinus;
		golem::Real etaPlus;
		golem::Real epsStop;
		size_t maxIter;
		RealSeq limits;

		Desc() {
			setToDefault();
		}
		void setToDefault() {
			minPopulation = 50;
			populationSize = 250;

			delta0 = 0.1;
			deltaMin = 1e-6;
			deltaMax = 50;
			etaMinus = 0.5;
			etaPlus = 1.2;
			epsStop = 0.0;// 1e-6;

			maxIter = 150;
			limits.assign(2, 10.0);
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
		size_t k = 0, population = 0;
		U32 jumps = 0;

		int paramDim = gp->cf->getParamDim();
		Eigen::VectorXd delta = Eigen::VectorXd::Ones(paramDim) * delta0;
		Eigen::VectorXd bestParams = gp->cf->getLogHyper(); //params;
		Real solutionEval = log(0);
		//grasp::ParallelsTask(context.getParallels(), [&](grasp::ParallelsTask*) {
		for (;jumps < desc.minPopulation;) {
			const U32 jobId = ++jumps;// context.getParallels()->getCurrentJob()->getJobId();
			bool limit = limits.size() >= paramDim;
			Eigen::VectorXd gradOld = Eigen::VectorXd::Zero(paramDim);
			Eigen::VectorXd params = bestParams = gp->cf->sampleParams();
			gp->cf->setLogHyper(bestParams);
			double eval = gp->logLikelihood();
			for (size_t i = 0; i < maxIter; ++i) {
				context.debug("Thread[%d] population=%d params=[%f %f] eval=%f best=%d\n", jobId, k, params(0), params(1), eval, solutionEval < eval);
				{
					CriticalSectionWrapper csw(cs);
					if (solutionEval < eval) {
						solutionEval = eval;
						bestParams = params;
						gp->cf->setLogHyper(bestParams);
					}
					++k;
					//if (++k > desc.populationSize)
					//	break;
				}
				Eigen::VectorXd grad = -gp->logLikelihoodGradient();
				//context.debug("thread[%d] grad [%f %f] norm %f\n", jobId, grad(0), grad(1), grad.norm());
				//context.debug("thread[%d] gradOld [%f %f] norm %f\n", jobId, gradOld(0), gradOld(1), gradOld.norm());
				gradOld = gradOld.cwiseProduct(grad);
				for (int j = 0; j < gradOld.size(); ++j) {
					bool update = true;
					if (gradOld(j) > 0) {
						delta(j) = std::min(delta(j)*etaPlus, deltaMax);
					}
					else if (gradOld(j) < 0) {
						delta(j) = std::max(delta(j)*etaMinus, deltaMin);
						if (grad(j) <= gradOld(j)) update = false;
						grad(j) = 0;
					}
					params(j) += update ? -sign(grad(j)) * delta(j) : golem::REAL_ZERO;
					// bound the parameters
					if (abs(params(j)) > limits[j])
						params(j) = sign(params(j)) * limits[j];
				}
				gradOld = grad;
				if (gradOld.norm() <= epsStop) break;
				gp->cf->setLogHyper(params);
				eval = gp->logLikelihood();
				//context.debug("Thread[%d][%d] params=[%s] lik=%f grad_norm=%f [%f]\n", jobId, i, gp->cf->getHyper2str().c_str(), eval, gradOld.norm(), epsStop);
				//if (solutionEval < eval) {
				//	solutionEval = eval;
				//	bestParams = params;
				//}
			}
		}
	//	});
		gp->cf->setLogHyper(bestParams);

		context.write("Optimisation::find(): Elapsed time: %.4fs\n", (float)(clock() - t) / CLOCKS_PER_SEC);
		context.write("Optimisation::find(): best solutionEval=%f params size=%d\n", solutionEval, bestParams.size());
		for (size_t i = 0; i < bestParams.size(); ++i)
			context.write("params[%d] = %f\n", i, bestParams(i));

	}

	template <typename CovTypePtr, typename CovTypeDesc> void find(SimpleGP<CovTypePtr, CovTypeDesc>* gp, bool verbose = true) {
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
						context.debug("Solution %f\n", testEval);
						for (size_t i = 0; i < testParams.size(); ++i)
							context.debug("params[%d] = %f\n", i, testParams(i));
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
	RealSeq limits;

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

		limits = desc.limits;

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

		/** Enable derivative */
		bool derivative;

		/** Enable debug */
		bool debug;
		/** Enable debug */
		bool verbose;


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
			derivative = true;
			debug = verbose = false;
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
	void evaluate(const Vec3Seq& x, RealSeq& fx, RealSeq& vars) {
		const size_t size = x.size();
		fx.assign(size, REAL_ZERO);
		vars.assign(size, REAL_ZERO);
		for (size_t i = 0; i < size; ++i) {
			fx[i] = f(x[i])(0);
			vars[i] = var(x[i]);
		}
	}
    virtual Eigen::Vector4d f(const golem::Vec3& xStar) {
        if (sampleset->empty()) 
			throw Message(Message::LEVEL_ERROR, "No training data available.");

        //Eigen::Map<const Eigen::VectorXd> x_star(x.v, input_dim);
        compute(true);
        update_alpha();
        update_k_star(xStar);
		Eigen::Vector4d sol = (*k_star) * (*alpha);
		return sol;
    }

    /** Predict variance v[f_*] ~ var(x_*)  */
	virtual golem::Real var(const golem::Vec3& xStar) {
        if (sampleset->empty()) return 0;

        //Eigen::Map<const Eigen::VectorXd> x_star(x.v, input_dim);
        compute(true);
        update_alpha();
        update_k_star(xStar); //update_k_star(x_star);

		// computes variance only on the estimated shape
        size_t n = sampleset->rows();
		Eigen::VectorXd ks = k_star->block(0, 0, 1, n);
        Eigen::VectorXd v = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(ks);
		return cf->get(xStar, xStar) - v.dot(v); //cf->get(x_star, x_star) - v.dot(v);
    }
	/** Predict f_* ~ GP(x_*) for derived values too */
	virtual Eigen::MatrixXd var2(const golem::Vec3& xStar) {
		if (sampleset->empty()) return Eigen::MatrixXd();

		//Eigen::Map<const Eigen::VectorXd> x_star(x.v, input_dim);
		compute(true);
		update_alpha();
		update_k_star(xStar); //update_k_star(x_star);

		size_t n = 4 * sampleset->rows();
		Eigen::VectorXd vf = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star->row(0));
		Eigen::VectorXd vd0 = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star->row(1));
		Eigen::VectorXd vd1 = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star->row(2));
		Eigen::VectorXd vd2 = L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solve(k_star->row(3));
		//context.write("v rows %d cols %d\n", v.rows(), v.cols());
		Eigen::MatrixXd kss; kss.resize(4, 4);

		size_t i = 0;
		for (size_t j = 0; j < 1; ++j) {
			kss(i, j) = cf->get(xStar, xStar);
			if (desc.debug) context.write("K**[%lu,%lu]=K**(x*,x*) = %f\n", i, j, kss(i, j));
		}
		for (size_t j = 0; j < 1; ++j) {
			for (size_t d = 0; d < 3; ++d) {
				kss(i, 1 + 3 * j + d) = cf->getDiff(xStar, xStar, d);
				if (desc.debug) context.write("K**[%lu,%lu]=d%luK**(x*,x*) = %f\n", i, 1 + 3 * j + d, d, kss(i, 1 + 3 * j + d));
			}
		}

		// 2nd row in K*
		// -> [dx(x*x1), dx(x*x2), dx(x*x3), ..., dx(x*xn), ...
		//    ... dxdx(x*x1), dxdy(x*x1), dxdz(x*x1), ..., dxdx(x*xn), dxdy(x*xn), dxdz(x*xn)] [1x4n]
		// the loop goes for 3rd and 4th row as well -> [3x4n]
		for (i = 1; i < 4; ++i) {
			for (size_t j = 0; j < 1; ++j) {
				kss(i, j) = cf->getDiff(xStar, xStar, i - 1);
				if (desc.debug) context.write("K**[%lu,%lu]=d%luK**(x*,x*) = %f\n", i, j, i - 1, kss(i, j));
			}
			for (size_t j = 0; j < 1; ++j) {
				for (size_t d = 0; d < 3; ++d) {
					kss(i, 1 + j * 3 + d) = cf->getDiff2(xStar, xStar, i - 1, d);
					if (desc.debug) context.write("K**[%lu,%lu]=d%lud%luK**(x*,x*) = %f\n", i, 1 + j * 3 + d, i - 1, d, kss(i, 1 + j * 3 + d));
				}
			}

		}
			
		kss(0, 0) = kss(0, 0) - vf.dot(vf);
		kss(0, 1) = kss(0, 1) - vd0.dot(vd0);
		kss(1, 0) = kss(0, 1);
		kss(0, 2) = kss(0, 2) - vd1.dot(vd1);
		kss(2, 0) = kss(0, 2);
		kss(0, 3) = kss(0, 3) - vd2.dot(vd2);
		kss(3, 0) = kss(0, 3);

		//context.write("Var[K**]:\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n[%f %f %f %f]\n", kss(0, 0), kss(0, 1), kss(0, 2), kss(0, 3),
		//	kss(1, 0), kss(1, 1), kss(1, 2), kss(1, 3),
		//	kss(2, 0), kss(2, 1), kss(2, 2), kss(2, 3),
		//	kss(3, 0), kss(3, 1), kss(3, 2), kss(3, 3));

		return kss;
	}


	/** Predict f, var, N, Tx and Ty */
	virtual void evaluate(const Vec3Seq& x, RealSeq& fx, RealSeq& varx, Eigen::MatrixXd& normals, Eigen::MatrixXd& tx, Eigen::MatrixXd& ty) {
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
				Kqpdiff(i, j) = cf->getDiff(x[i], sampleset->x(j), -1);
			}
		}
		// compute error
		golem::Real error = REAL_ZERO;
		//#pragma omp parallel for
		for (size_t i = 0; i < nq; ++i) {
			for (size_t j = 0; j < np; ++j)
				normals.row(i) += (*alpha)(j)*Kqpdiff(i, j)*(convertToEigen(x[i] - sampleset->x(j)));

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
	void add_patterns(const Vec3Seq& newInputs, const RealSeq& newTargets) {
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
		const bool onedimension = false;
		const int n = onedimension ? sampleset->rows() : 4 * sampleset->rows();
		Eigen::VectorXd a(n);
		a = alpha->block(0, 0, n, 1); 
		const std::vector<golem::Real>& targets = sampleset->y();
		Eigen::Map<const Eigen::VectorXd> y(&targets[0], n/*targets.size()*/);
		golem::Real det = (golem::Real)(2 * L->diagonal().head(n).array().log().sum());
		return (golem::Real)(- 0.5*y.dot(a/**alpha*/) - 0.5*det - 0.5*n*log2pi);
    }

	Eigen::VectorXd logLikelihoodGradient()
	{
		const bool onedimension = false;
		compute(false);
		update_alpha();
		size_t l = sampleset->rows();
		size_t n = onedimension ? l : 4 * l;
		Eigen::VectorXd grad = Eigen::VectorXd::Zero(cf->getParamDim());
		Eigen::VectorXd g(grad.size());
		Eigen::MatrixXd W = Eigen::MatrixXd::Identity(n, n);

		// compute kernel matrix inverse
		L->topLeftCorner(n, n).triangularView<Eigen::Lower>().solveInPlace(W);
		L->topLeftCorner(n, n).triangularView<Eigen::Lower>().transpose().solveInPlace(W);

		Eigen::VectorXd a(n);
		a = alpha->block(0, 0, n, 1);
		//W = (*alpha) * alpha->transpose() - W;
		W = a * a.transpose() - W;
		//context.debug("l=%d n=%d W=[%d %d]\n", l, n, W.rows(), W.cols());

		for (size_t i = 0; i < l; ++i) {
			for (size_t j = 0; j <= i; ++j) {
				cf->grad(sampleset->x(i), sampleset->x(j), g);
				//context.debug("x_%d x_%d g=[%f %f]\n", i, j, g(0), g(1));
				if (i == j) grad += W(i, j) * g * 0.5;
				else grad += W(i, j) * g;
			}
		}
		if (!onedimension) {
			for (size_t i = 0; i < l; ++i) {
				for (size_t d = 0; d < 3; ++d) {
					for (size_t j = 0; j < l; ++j) {
						cf->gradDiff(sampleset->x(i), sampleset->x(j), d, g);
						//context.debug("d1(%d) x_%d x_%d g=[%f %f]\n", d, i, j, g(0), g(1));
						if (i == j) grad += W(l + i * 3 + d, j) * g * 0.5;
						else grad += W(l + i * 3 + d, j) * g;
					}
					size_t j = 0;
					for (size_t z = 0; z < i * 3 + d + 1; ++z) {
						cf->gradDiff2(sampleset->x(i), sampleset->x(j), d, z % 3, g);
						//context.debug("d1(%d) d2(%d) x_%d x_%d g=[%f %f]\n", d, z % 3, i, j, g(0), g(1));
						if (i == j) grad += W(l + i * 3 + d, l + z) * g * 0.5;
						else grad += W(l + i * 3 + d, l + z) * g;
						if (z % 3 == 2) ++j;
					}
				}
			}
		}

		return grad;
	}

	//grasp::RBDist crossValidation(const Vec3Seq& x, const RealSeq& fx, const Vec3Seq& normals) {
	//	if (x.size() != fx.size || x.size() != normals.size())
	//		throw Message("Cross validation invalid input length.")
	//	const size_t size = x.size();
	//	grasp::RBDist error;
	//	for (size_t i = 0; i < size; i++) {
	//		const eigen::VectorXd estimate = f(x[i]);
	//		error.lin = sqr(fx[i] - estimate(0));
	//		const Vec3 n(estimate(1), estimate(2), estimate(3));
	//		error.ang = sqr((1 - n.dot(normals[i]) * 90 * golem::REAL_PI/180);
	//	}
	//	error.lin /= size;
	//	error.ang /= size;
	//	return error;
	//}

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
	boost::shared_ptr<Eigen::MatrixXd> k_star;
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

    /** Compute k_* = K(x_*, x) */
	void update_k_star(const golem::Vec3& x_star) {
		const size_t n = sampleset->rows();
		const size_t ndt = 4 * n;
        k_star->resize(4, ndt);
		// first row in K*
		// -> [x*x1, ... , x*xn, dx(x*x1), dy(x*x1), dz(x*x1), ..., dx(x*xn), dy(x*xn), dz(x*xn)] [1x4n]
		size_t i = 0;
		for (size_t j = 0; j < n; ++j) {
			(*k_star)(i, j) = cf->get(x_star, sampleset->x(j));
			if (desc.debug) context.write("K*[%lu,%lu]=K*(x*,%lu) = %f\n", i, j, j, (*k_star)(i, j));
		}
		for (size_t j = 0; j < n; ++j) {
			for (size_t d = 0; d < 3; ++d) {
				(*k_star)(i, n + 3 * j + d) = cf->getDiff(x_star, sampleset->x(j), d);
				if (desc.debug) context.write("K*[%lu,%lu]=d%luK*(x*,%lu) = %f\n", i, n + 3 * j + d, d, j, (*k_star)(i, n + 3 * j + d));
			}
		}

		// 2nd row in K*
		// -> [dx(x*x1), dx(x*x2), dx(x*x3), ..., dx(x*xn), ...
		//    ... dxdx(x*x1), dxdy(x*x1), dxdz(x*x1), ..., dxdx(x*xn), dxdy(x*xn), dxdz(x*xn)] [1x4n]
		// the loop goes for 3rd and 4th row as well -> [3x4n]
		for (i = 1; i < 4; ++i) {
			for (size_t j = 0; j < n; ++j) {
				(*k_star)(i, j) = cf->getDiff(x_star, sampleset->x(j), i - 1);
				if (desc.debug) context.write("K*[%lu,%lu]=d%luK*(x*,%lu) = %f\n", i, j, i - 1, j, (*k_star)(i, j));
			}
			for (size_t j = 0; j < n; ++j) {
				for (size_t d = 0; d < 3; ++d) {
					(*k_star)(i, n + j * 3 + d) = cf->getDiff2(x_star, sampleset->x(j), i - 1, d);
					if (desc.debug) context.write("K*[%lu,%lu]=d%lud%luK*(x*,%lu) = %f\n", i, n + j * 3 + d, i - 1, d, j, (*k_star)(i, n + j * 3 + d));
				}
			}

		}
		if (desc.debug) {
			context.write("K* [%d %d]\n", k_star->rows(), k_star->cols());
			for (size_t i = 0; i < k_star->rows(); ++i) {
				for (size_t j = 0; j < k_star->cols(); ++j)
					context.write("%.5f ", (*k_star)(i, j));
				context.write("\n");
			}
		}
		//for (size_t i = 0; i < 4; ++i) {
		//	(*k_star)(i, j) = cf->get(x_star, sampleset->x(i));
		//}
    }

    /** Update alpha vector (mean) */
    void update_alpha() {
        // can previously computed values be used?
        if (!alpha_needs_update) return;
        alpha_needs_update = false;
		const size_t ndt = 4 * sampleset->rows();
        alpha->resize(ndt);
		// Map target values to VectorXd
        //const std::vector<double>& targets = sampleset->y();
		const RealSeq& targets = sampleset->y();
		if (targets.size() != ndt)
			throw Message(Message::LEVEL_ERROR, "Target and alpha vector have a size dismatch.");

		Eigen::Map<const Eigen::VectorXd> y(&targets[0], ndt);
		*alpha = L->topLeftCorner(ndt, ndt).triangularView<Eigen::Lower>().solve(y);
		if (desc.debug) {
			context.write("alpha [%d %d]\n", alpha->rows(), alpha->cols());
			for (size_t i = 0; i < alpha->size(); ++i)
				context.write("alpha[%d] = %f\n", i, (*alpha)(i));
		}
		L->topLeftCorner(ndt, ndt).triangularView<Eigen::Lower>().adjoint().solveInPlace(*alpha);
		if (desc.debug) {
			context.write("\nalpha^-1 [%d %d]\n", alpha->rows(), alpha->cols());
			for (size_t i = 0; i < alpha->size(); ++i)
				context.write("alpha^-1[%d] = %f\n", i, (*alpha)(i));
		}
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
        if (4 * n > L->rows()) L->resize(4 * n + initialLSize, 4 * n + initialLSize);

		// compute kernel matrix (lower triangle)
//#pragma omp parallel for collapse(2)
        for(size_t i = 0; i < n; ++i) {
            for(size_t j = 0; j <= i; ++j) {
                // compute kernel and add noise on the diagonal of the kernel
				(*L)(i, j) = cf->get(sampleset->x(i), sampleset->x(j), i == j);
				(*L)(i, j) += i == j ? 0.01 : 0.0;
				if (desc.debug) context.write("Computing L(%lu, %lu) = %f\n", i, j, (*L)(i, j));
			}
		}	
		Eigen::MatrixXd k = L->topLeftCorner(n, n).selfadjointView<Eigen::Lower>();
		golem::Real maxR, minR, maxI, minI;
//		golem::Real d1 = determinant(k, maxR, minR, maxI, minI);
//		context.write("det[K(X,X)] = %f eigenvalues max = (%f, %f) min = (%f, %f)\n", d1, maxR, maxI, minR, minI);
		//context.write("K(X,X): [%d %d]\n", n, n);
		//for (size_t i = 0; i < n; ++i) {
		//	for (size_t j = 0; j < n; ++j) {
		//		context.write("%.5f ", k(i, j));
		//	}
		//	context.write("\n");
		//}
		// derivatives
		for (size_t i = 0; i < n; ++i) {
			for (size_t d = 0; d < 3; ++d) {
				// first derivatives
				for (size_t j = 0; j < n; ++j) {
					(*L)(n + i * 3 + d, j) = cf->getDiff(sampleset->x(i), sampleset->x(j), d, i == j);
					if (desc.debug) context.write("Computing L(%lu, %lu)=d%luk(%lu, %lu) = %f\n", n + i * 3 + d, j, d, i, j, (*L)(n + i * 3 + d, j));
				}
				// second derivatives
				//for (size_t j = 0; j < i * 3 + d + 1; ++j) {
				//	(*L)(n + i * 3 + d, n + j) = cf->getDiff2(sampleset->x(i), sampleset->x(j), d, j % 3, i == j);
				//	printf("Computing L(%lu, %lu)=d%lud%luk(%lu, %lu) = %f\n", n + i * 3 + d, n + j, d, j % 3, i, j, (*L)(n + i * 3 + d, n + j));
				//}
				size_t j = 0;
				for (size_t z = 0; z < i * 3 + d + 1; ++z) {
					(*L)(n + i * 3 + d, n + z) = cf->getDiff2(sampleset->x(i), sampleset->x(j), d, z % 3, i == j);
					(*L)(n + i * 3 + d, n + z) += i == j && d == z % 3 ? 0.01 : 0.0;
					if (desc.debug) context.write("Computing L(%lu, %lu)=d%lud%luk(%lu, %lu) = %f\n", n + i * 3 + d, n + z, d, z % 3, i, j, (*L)(n + i * 3 + d, n + z));
					if (z % 3 == 2) ++j;
				}
			}
		}
//		context.write("det[K*(x,X)] = %f\n", L->block(n, 0, 3 * n, n).determinant());
//		Eigen::MatrixXd kx1 = L->block(n, 0, n, n);
//		golem::Real d2 = determinant(kx1, maxR, minR, maxI, minI);
//		context.write("det[dxK(x,x)] = %f eigenvalues max = (%f, %f) min = (%f, %f)\n", d2, maxR, maxI, minR, minI);
//		context.write("dxK(X,X): [%d %d]\n", n, n);
		//for (size_t i = 0; i < n; ++i) {
		//	for (size_t j = 0; j < n; ++j) {
		//		context.write("%.5f ", kx1(i, j));
		//	}
		//	context.write("\n");
		//}
//		Eigen::MatrixXd kx2 = L->block(2 * n, 0, n, n);
//		golem::Real d3 = determinant(kx2, maxR, minR, maxI, minI);
//		context.write("det[dyK(x,x)] = %f eigenvalues max = (%f, %f) min = (%f, %f)\n", d3, maxR, maxI, minR, minI);
		//context.write("dyK(X,X): [%d %d]\n", n, n);
		//for (size_t i = 0; i < n; ++i) {
		//	for (size_t j = 0; j < n; ++j) {
		//		context.write("%.5f ", kx2(i, j));
		//	}
		//	context.write("\n");
		//}
//		Eigen::MatrixXd kx3 = L->block(3 * n, 0, n, n);
//		golem::Real d4 = determinant(kx3, maxR, minR, maxI, minI);
//		context.write("det[dzK(x,x)] = %f eigenvalues max = (%f, %f) min = (%f, %f)\n", d4, maxR, maxI, minR, minI);
		//context.write("dzK(X,X): [%d %d]\n", n, n);
		//for (size_t i = 0; i < n; ++i) {
		//	for (size_t j = 0; j < n; ++j) {
		//		context.write("%.5f ", kx3(i, j));
		//	}
		//	context.write("\n");
		//}
//		Eigen::MatrixXd kxx = L->block(n, n, 3 * n, 3 * n).selfadjointView<Eigen::Lower>();
//		golem::Real d5 = determinant(kxx, maxR, minR, maxI, minI);
//		context.write("det[K(x,x)] = %f eigenvalues max = (%f, %f) min = (%f, %f)\n", d5, maxR, maxI, minR, minI);
		//context.write("L: [%d %d]\n", 4*n, 4*n);
		//for (size_t i = 0; i < 4*n; ++i) {
		//	for (size_t j = 0; j < 4*n; ++j) {
		//		context.write("%.5f ", (*L)(i, j));
		//	}
		//	context.write("\n");
		//}
		//Eigen::MatrixXd tmp = L->topLeftCorner(4 * n, 4 * n).selfadjointView<Eigen::Lower>();
		//golem::Real kmean = tmp.mean();
		//context.write("K mean %f\n", kmean);
		//context.write("K: [%d %d]\n", 4 * n, 4 * n);
		//for (size_t i = 0; i < 4 * n; ++i) {
		//	for (size_t j = 0; j < 4 * n; ++j) {
		//		context.write("%.5f ", tmp(i, j));
		//	}
		//	context.write("\n");
		//}
		//for (size_t i = 0; i < tmp.rows(); ++i) {
		//	const golem::Real rowMean = tmp.row(i).mean();
		//	for (size_t j = 0; j <= i; ++j)
		//		tmp(i, j) -= rowMean;
		//}
		//for (size_t j = 0; j < tmp.cols(); ++j) {
		//	const golem::Real colMean = tmp.col(j).mean();
		//	for (size_t i = j; i < tmp.rows(); ++i)
		//		tmp(i, j) -= colMean;
		//}
		//for (size_t i = 0; i < tmp.rows(); ++i) {
		//	for (size_t j = 0; j <= i; ++j)  
		//		tmp(i, j) = -tmp(i,j)*(*L)(i,j)*(*L)(i,j)/2;
		//}
		//tmp = tmp.selfadjointView<Eigen::Lower>();
		//context.write("K: [%d %d]\n", 4 * n, 4 * n);
		//for (size_t i = 0; i < 4 * n; ++i) {
		//	for (size_t j = 0; j < 4 * n; ++j) {
		//		context.write("%.5f ", tmp(i, j));
		//	}
		//	context.write("\n");
		//}
		//context.write("det[tmp] = %f row sum= %f  col sum = %f\n", determinant(context, tmp), tmp.row(0).sum(), tmp.col(0).sum());
		//for (size_t i = 0; i < 4 * n; ++i) {
		//	for (size_t j = 0; j <= i; ++j)
		//		(*L)(i, j) = tmp(i, j);
		//}
		//context.write("L: [%d %d]\n", 4 * n, 4 * n);
		//for (size_t i = 0; i < 4 * n; ++i) {
		//	for (size_t j = 0; j < 4 * n; ++j) {
		//		context.write("%.5f ", (*L)(i, j));
		//	}
		//	context.write("\n");
		//}
		//context.write("d2xK(X,X): [%d %d]\n", 3 * n, 3 * n);
		//for (size_t i = 0; i < 3*n; ++i) {
		//	for (size_t j = 0; j < 3*n; ++j) {
		//		context.write("%.5f ", kxx(i, j));
		//	}
		//	context.write("\n");
		//}
		const size_t ndt = 4 * n;
		// save temporarely the original covariance matrix
		Eigen::MatrixXd K = L->topLeftCorner(ndt, ndt).selfadjointView<Eigen::Lower>();
		// perform cholesky factorization
		L->topLeftCorner(ndt, ndt) = L->topLeftCorner(ndt, ndt).selfadjointView<Eigen::Lower>().ldlt().matrixL();
		// compute K = LL^T to check if the K matrix is defined positive
		//Eigen::MatrixXd LLT = L->topLeftCorner(ndt, ndt) * L->topLeftCorner(ndt, ndt).transpose();
		//golem::Real Ldet = pow(L->diagonal().head(ndt).array().prod(), 2.0);
		//golem::Real Kdet = pow(K.diagonal().head(ndt).array().prod(), 2.0);
		//golem::Real LLTdet = LLT.determinant();
//		context.write("det[L] = %f det[K] = %f det[LL^T] = %f\n", Ldet, Kdet, LLTdet);

//		if (abs(Ldet) < golem::REAL_EPS || !isfinite(Ldet) || isnan(Ldet) || isinf(Ldet))
//			context.write("ERROR: not invertable matrix L. det[L] = %f\n", Ldet);
//		if (!LLT.isApprox(K.topLeftCorner(ndt, ndt), 1e-3)) {
//			Eigen::EigenSolver<Eigen::MatrixXd> es(K);
//			bool isSemidefined = true;
//			Eigen::VectorXcd eigenvalues = es.eigenvalues();
//			for (size_t i = 0; i < eigenvalues.size(); ++i) {
//				if (eigenvalues(i).real() <= -golem::REAL_EPS) {
//					isSemidefined = false;
////					context.write("K Eigenvalue[%d] = (%f, %f)\n", i, eigenvalues(i).real(), eigenvalues(i).imag());
//					//break;
//				}
//			}
//			if (isSemidefined)
//				context.write("MATRIX K IS DEFINED SEMI-POSITIVE!\n");
//			else
//				context.write("ERROR MATRIX K IS NOT DEFINED SEMI-POSITIVE!\n");
//		}
//		else
//			context.write("MATRIX K IS DEFINED POSITIVE!\n");

		//if (desc.debug)  {
		//	context.write("K: [%d %d]\n", ndt, ndt);
		//	for (size_t i = 0; i < ndt; ++i) {
		//		for (size_t j = 0; j < ndt; ++j) {
		//			context.write("%.5f ", K(i, j));
		//		}
		//		context.write("\n");
		//	}

		//	context.write("LL^T: [%d %d]\n", LLT.rows(), LLT.cols());
		//	for (size_t i = 0; i < LLT.rows(); ++i) {
		//		for (size_t j = 0; j < ndt; ++j) {
		//			context.write("%.5f ", LLT(i, j));
		//		}
		//		context.write("\n");
		//	}
		//}

		if (desc.verbose) {
			context.write("L: [%d %d]\n", ndt, ndt);
			for (size_t i = 0; i < ndt; ++i) {
				for (size_t j = 0; j <= i; ++j) {
					context.write("%.5f ", (*L)(i, j));
				}
				context.write("\n");
			}
		}
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

        cf = desc.covTypeDesc.create(context);
        //sampleset = desc.trainingData;
        input_dim = 3;
        initialLSize = desc.initialLSize;
        alpha.reset(new Eigen::VectorXd);
		k_star.reset(new Eigen::MatrixXd);
        L.reset(new Eigen::MatrixXd);
        L->resize(initialLSize, initialLSize);
		alpha_needs_update = true;
	}

    /** Default C'tor */
	GaussianProcess(const golem::Context& context) : context(context) {}
};

//------------------------------------------------------------------------------

template <class CovTypePtr, class CovTypeDesc> class SimpleGP {
public:
	/** Needed for good alignment for the Eigen's' data structures */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/** Pointer to the class */
	typedef boost::shared_ptr<SimpleGP<CovTypePtr, CovTypeDesc> > Ptr;

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

		/** Enable derivative */
		bool derivative;

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

			derivative = true;
		}

		/** Creates the object from the description. */
		Ptr create(const golem::Context& context) const {
			Ptr pointer(new SimpleGP<CovTypePtr, CovTypeDesc>(context));
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
	void evaluate(const Vec3Seq& x, RealSeq& fx, RealSeq& vars) {
		const size_t size = x.size();
		fx.assign(size, REAL_ZERO);
		vars.assign(size, REAL_ZERO);
		for (size_t i = 0; i < size; ++i) {
			fx[i] = f(x[i]);
			vars[i] = var(x[i]);
		}
	}

	virtual golem::Real f(const golem::Vec3& xStar) {
		if (sampleset->empty())
			throw Message(Message::LEVEL_ERROR, "No training data available.");

		//Eigen::Map<const Eigen::VectorXd> x_star(x.v, input_dim);
		compute(true);
		update_alpha();
		update_k_star(xStar);
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
	virtual void evaluate(const Vec3Seq& x, RealSeq& fx, RealSeq& varx, Eigen::MatrixXd& normals, Eigen::MatrixXd& tx, Eigen::MatrixXd& ty, Eigen::MatrixXd& V) {
		clock_t t = clock();

		// compute f(x) and V(x)
		evaluate(x, fx, varx);

		const size_t np = sampleset->rows(), nq = x.size();
		const size_t dim = sampleset->cols();

		printf("np=%d nq=%d dim=%d\n", np, nq, dim);

		// differential of covariance with selected kernel
		Eigen::MatrixXd Kqp, Kqpdiffx, Kqpdiffy, Kqpdiffz;;
		// row = number of query points
		// col = number of points in the kernels + 1
//		Kqp.resize(nq, np + 1);
		Kqpdiffx.resize(nq, np);
		Kqpdiffy.resize(nq, np);
		Kqpdiffz.resize(nq, np);
		// normals variance
		V.resize(nq, 3);
		normals.resize(nq, dim);
		tx.resize(nq, dim);
		ty.resize(nq, dim);

		// compute Kqp and KqpDiff
		for (size_t i = 0; i < nq; ++i) {
			for (size_t j = 0; j < np; ++j) {
//				Kqp(i, j) = cf->get(x[i], sampleset->x(j));
				//Kqpdiffx(i, j) = cf->getDiff(x[i], sampleset->x(j), 0);
				//Kqpdiffy(i, j) = cf->getDiff(x[i], sampleset->x(j), 1);
				//Kqpdiffz(i, j) = cf->getDiff(x[i], sampleset->x(j), 2);
				Kqpdiffx(i, j) = cf->getDiff(sampleset->x(j), x[i], 0);
				Kqpdiffy(i, j) = cf->getDiff(sampleset->x(j), x[i], 1);
				Kqpdiffz(i, j) = cf->getDiff(sampleset->x(j), x[i], 2);
			}
		}

		// compute error
		golem::Real error = REAL_ZERO;
		//#pragma omp parallel for
		for (size_t i = 0; i < nq; ++i) {
			normals(i, 0) = Kqpdiffx.row(i).dot(*alpha);
			normals(i, 1) = Kqpdiffy.row(i).dot(*alpha);
			normals(i, 2) = Kqpdiffz.row(i).dot(*alpha);
			Eigen::VectorXd vx = L->topLeftCorner(np, np).triangularView<Eigen::Lower>().solve(Kqpdiffx.row(i));
			V(i, 0) = cf->getDiff2(x[i], x[i], 0, 0) - vx.dot(vx);
			Eigen::VectorXd vy = L->topLeftCorner(np, np).triangularView<Eigen::Lower>().solve(Kqpdiffy.row(i));
			V(i, 1) = cf->getDiff2(x[i], x[i], 1, 1) - vy.dot(vy);
			Eigen::VectorXd vz = L->topLeftCorner(np, np).triangularView<Eigen::Lower>().solve(Kqpdiffz.row(i));
			V(i, 2) = cf->getDiff2(x[i], x[i], 2, 2) - vz.dot(vz);
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
	void add_patterns(const Vec3Seq& newInputs, const RealSeq& newTargets) {
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
				L->block(j, 0, 1, j) = k.transpose();
				(*L)(j, j) = sqrt(kappa - k.dot(k));
			}
			context.write("GP::add_patterns(): Elapsed time: %.4fs\n", (float)(clock() - t) / CLOCKS_PER_SEC);
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
		return (golem::Real)(-0.5*y.dot(*alpha) - 0.5*det - 0.5*n*log2pi);
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
	/** Weights */
	Eigen::VectorXd InvKppY;

	/** Input vector dimensionality. */
	size_t input_dim;
	/** Enable/disable to update the alpha vector */
	bool alpha_needs_update;
	//initial L size
	golem::U32 initialLSize;

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
		//context.write("alpha [%d %d]\n", alpha->rows(), alpha->cols());
		//for (size_t i = 0; i < alpha->size(); ++i)
		//	context.write("alpha[%d] = %f\n", i, (*alpha)(i));
		L->topLeftCorner(n, n).triangularView<Eigen::Lower>().adjoint().solveInPlace(*alpha);
		for (size_t i = 0; i < alpha->size(); ++i)
			if (!isfinite((*alpha)(i)) || isnan((*alpha)(i)) || isinf((*alpha)(i)))
				context.write("GP::update_alpha(): alpha[%d] = %f is not finite.\n", i, (*alpha)(i));

		//context.write("\nalpha^-1 [%d %d]\n", alpha->rows(), alpha->cols());
		//for (size_t i = 0; i < alpha->size(); ++i)
		//	context.write("alpha^-1[%d] = %f\n", i, (*alpha)(i));
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
		//#pragma omp parallel for
		for (size_t i = 0; i < n; ++i) {
			for (size_t j = 0; j <= i; ++j) {
				// add noise on the diagonal of the kernel
				(*L)(i, j) = cf->get(sampleset->x(i), sampleset->x(j), i == j);
				if (!isfinite((*L)(i, j)) || isnan((*L)(i, j)) || isinf((*L)(i, j)))
					context.write("GP::compute(): L[%d, %d] = %f is not finite.\n", i, j, (*L)(i, j));
				//printf("GP::compute(): Computing k(%lu, %lu)\r", i, j);
			}
		}
		// perform cholesky factorization
		Eigen::MatrixXd K = L->topLeftCorner(n, n).selfadjointView<Eigen::Lower>();
		L->topLeftCorner(n, n) = L->topLeftCorner(n, n).selfadjointView<Eigen::Lower>().ldlt().matrixL();
		Eigen::MatrixXd LLT = L->topLeftCorner(n, n) * L->topLeftCorner(n, n).transpose();
		golem::Real maxR, minR, maxI, minI;
		golem::Real Ldet = determinant(L->topLeftCorner(n, n), maxR, minR, maxI, minI);
		golem::Real L2det = pow(L->diagonal().head(n).array().prod(), 2.0);
		golem::Real Kdet = determinant(K, maxR, minR, maxI, minI);
		golem::Real LLTdet = determinant(LLT, maxR, minR, maxI, minI);
		context.write("det[L] = %f (%f) det[K] = %f det[LL^T] = %f\n", Ldet, L2det, Kdet, LLTdet);
		if (abs(Ldet) < golem::REAL_EPS || !isfinite(Ldet) || isnan(Ldet) || isinf(Ldet))
			context.write("ERROR: Cholesky has produced a L matrix with det[L] = %f\n", Ldet);
		if (!LLT.isApprox(K.topLeftCorner(n, n), 1e-3)) {
			Eigen::EigenSolver<Eigen::MatrixXd> es(K);
			bool isSemidefined = true;
			Eigen::VectorXcd eigenvalues = es.eigenvalues();
			for (size_t i = 0; i < eigenvalues.size(); ++i) {
				if (eigenvalues(i).real() <= golem::REAL_ZERO) {
					isSemidefined = false;
					context.write("K Eigenvalue[%d] = (%f, %f)\n", i, eigenvalues(i).real(), eigenvalues(i).imag());
					//break;
				}
			}
			if (isSemidefined)
				context.write("MATRIX K IS DEFINED SEMI-POSITIVE!\n");
			else
				context.write("ERROR MATRIX K IS NOT DEFINED SEMI-POSITIVE!\n");
		}
		else
			context.write("MATRIX K IS DEFINED POSITIVE!\n");

		//context.write("K: [%d %d]\n", n, n);
		//for (size_t i = 0; i < n; ++i) {
		//	for (size_t j = 0; j < n; ++j) {
		//		context.write("%.5f ", K(i, j));
		//	}
		//	context.write("\n");
		//}

		//context.write("L: [%d %d]\n", n, n);
		//for (size_t i = 0; i < n; ++i) {
		//	for (size_t j = 0; j <= i; ++j) {
		//		context.write("%.5f ", (*L)(i, j));
		//	}
		//	context.write("\n");
		//}

		alpha_needs_update = true;
		if (verbose) printf("GP::Compute(): Elapsed time: %.4fs\n", (float)(clock() - t) / CLOCKS_PER_SEC);
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

		cf = desc.covTypeDesc.create(context);
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
	SimpleGP(const golem::Context& context) : context(context) {}
};

//------------------------------------------------------------------------------

/** List of legal types */
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::Laplace::Desc> LaplaceRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::CovSE::Desc> GaussianRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::CovSEArd::Desc> GaussianARDRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::CovSESC::Desc> GaussianSESCRegressor;
typedef GaussianProcess<spam::BaseCovFunc::Ptr, spam::ThinPlate::Desc> ThinPlateRegressor;

typedef SimpleGP<spam::BaseCovFunc::Ptr, spam::CovSE::Desc> SimpleGaussianRegressor;
typedef SimpleGP<spam::BaseCovFunc::Ptr, spam::ThinPlate::Desc> SimpleThinPlateRegressor;

//typedef GaussianProcess<gp::Laplace::Ptr, gp::Laplace::Desc> LaplaceRegressor;
//typedef GaussianProcess<gp::ThinPlate> ThinPlateRegressor;
//class LaplaceRegressor : public GaussianProcess<gp::Laplace> {};

//------------------------------------------------------------------------------

}
#endif /* __GP_GAUSSIANPROCESS_H__ */
