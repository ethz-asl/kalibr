#ifndef INCLUDE_ASLAM_BACKEND_THREADEDVECTORPROCESSOR_HPP_
#define INCLUDE_ASLAM_BACKEND_THREADEDVECTORPROCESSOR_HPP_

#include <vector>

#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace aslam {
namespace backend {
namespace util {

/**
 * The job will be run nThreads times in parallel. The index range (0 .. rangeLength - 1) will be partitioned into nThreads many subranges the job instances should work on.
 * It throws the exception thrown in the first job throwing an exception unless non is thrown.
 *
 * @param job
 *  The job will be run nThreads times in parallel.
 *  The first argument will be the job index {0 .. nThreads - 1}).
 *  The second (=:a) and third (=:b) argument specify which subrange of (0..rangeLength-1) the job should work on as (a..b-1).
 * @param rangeLength specifies the length of the range (0 .. rangeLength - 1), which will be processed by the job function after dividing it in subranges.
 * @param number of threads to create
 */

void runThreadedJob(boost::function<void(size_t, size_t, size_t)> job, size_t rangeLength, size_t nThreads);

/**
 * The job will be run nThreads times in parallel. The index range (0 .. rangeLength - 1) will be partitioned into nThreads many subranges the job instances should work on.
 * The i-th job thread will be given a output reference taken from out[i].
 * It throws the exception thrown in the first job throwing an exception unless non is thrown.
 *
 * @param function
 *  The function will be run nThreads times in parallel.
 *  For the first three arguments it gets see runThreadedJob.
 *  The fourth will be a reference to out[i] for the i-th thread.
 * @param rangeLength
 * custom length of a range, NOT related to \p out. This range
 * is related to external containers the \p function works upon.
 * @param out the vector of output variables.
 */

template <typename Output>
void runThreadedFunction(boost::function<void(size_t, size_t, size_t, Output&)> function, size_t rangeLength, std::vector<Output>& out){
  runThreadedJob(boost::bind(function, _1, _2, _3, boost::bind(static_cast<Output & (std::vector<Output>::*)(size_t) >(&std::vector<Output>::at), &out, _1)), rangeLength, out.size());
}

}
}
}

#endif /* INCLUDE_ASLAM_BACKEND_THREADEDVECTORPROCESSOR_HPP_ */
