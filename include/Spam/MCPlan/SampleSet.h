/** @file SampleSet.h
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
#ifndef __SAMPLESET_H__
#define __SAMPLESET_H__

//------------------------------------------------------------------------------

#include <Eigen/Dense>
#include <Golem/Math/Vec3.h>
#include <Golem/Tools/Context.h>
#include <boost/shared_ptr.hpp>

//------------------------------------------------------------------------------

namespace spam {

//------------------------------------------------------------------------------

//typedef golem::Vec3 Vec3;
typedef std::vector<golem::Vec3> Vec3Seq;
typedef std::vector<golem::Real> RealSeq;

//------------------------------------------------------------------------------

class SampleSet {
public:
	typedef boost::shared_ptr<SampleSet> Ptr;
	/** Default C'tor. Does nothing.
	*/
	SampleSet();
	/** C'tor.
	*   @param x input
	*/
	SampleSet(const Vec3Seq& inputs, const RealSeq& targets);
	SampleSet(const Vec3Seq& inputs, const RealSeq& targets, const Vec3Seq& normals);

	/** Destructor. */
	virtual ~SampleSet();

	/** Add input-output patterns to sample set.
	* @param x input array
	* @param y target values */
	void add(const Vec3Seq& newInputs, const RealSeq& newTargets);
	void add(const Vec3Seq& newInputs, const RealSeq& newTargets, const Vec3Seq& Normals);

	/** Get input vector at index k. */
	//const Eigen::VectorXd & x(size_t k);
	const golem::Vec3& x(size_t k) const;

	/** Get target value at index k. */
	double y(size_t k) const;
	/** Set target value at index i. */
	bool set_y(size_t i, double y);

	/** Get reference to vector of target values. */
	RealSeq y() const;

	/** Get number of samples. */
	inline size_t rows() const { return n; };
	/** Get dimensionality of input space. */
	inline size_t cols() const { return d; };

	/** Clear sample set. */
	void clear();

	/** Check if sample set is empty. */
	inline bool empty() const { return n == 0; };

	RealSeq getNormals() const { return N; }

private:
	/** Container holding input vectors. */
	Vec3Seq X;
	/** Container holding target values. */
	RealSeq Y;
	/** Container holding target normal vectors. */
	RealSeq N;

	/** Number of samples. */
	size_t n;
	/** Dimensionality of input space. */
	size_t d;
};
//------------------------------------------------------------------------------

}
#endif /* __SAMPLESET_H__ */
