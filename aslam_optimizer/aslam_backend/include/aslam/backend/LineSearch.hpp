#ifndef ASLAM_BACKEND_LINESEARCH_HPP
#define ASLAM_BACKEND_LINESEARCH_HPP

/*
Most of the following is a c++ translations of code from https://github.com/scipy/scipy/blob/master/scipy/optimize/linesearch.py,
the function dcstep is based on https://github.com/scipy/scipy/blob/master/scipy/optimize/minpack2/dcstep.f,
the class Dcsrch is based on https://github.com/scipy/scipy/blob/master/scipy/optimize/minpack2/dcsrch.f .

For those parts the following license applies:

SciPy project (http://www.scipy.org/):

Copyright (c) 2001, 2002 Enthought, Inc.
All rights reserved.

Copyright (c) 2003-2016 SciPy Developers.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

  a. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
  b. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
  c. Neither the name of Enthought nor the names of the SciPy Developers
     may be used to endorse or promote products derived from this software
     without specific prior written permission.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <boost/function.hpp>

#include <aslam/Exceptions.hpp>
#include <aslam/backend/util/CostFunctionInterface.hpp>

namespace sm {
  class PropertyTree;
}

namespace aslam {
  namespace backend {

    /**
     * Computes a safeguarded step for a search procedure and updates an interval that contains a step that satisfies a sufficient decrease and a curvature condition.
     * This method is adopted from https://github.com/funsim/moola/blob/master/moola/linesearch/dcstep.py
     * @param stx     On entry stx is the best step obtained so far and is an endpoint of the interval that contains the minimizer.
     *                On exit stx is the updated best step.
     * @param fx      On entry fx is the function at stx. On exit fx is the function at stx.
     * @param dx      On entry dx is the derivative of the function at stx. The derivative must be negative in the direction of the step, that is, dx and stp - stx must have opposite signs.
     *                On exit dx is the derivative of the function at stx.
     * @param sty     On entry sty is the second endpoint of the interval that contains the minimizer. On exit sty is the updated endpoint of the interval that contains the minimizer.
     * @param fy      On entry fy is the function at sty. On exit fy is the function at sty.
     * @param dy      On entry dy is the derivative of the function at sty. On exit dy is the derivative of the function at the exit sty.
     * @param stp     On entry stp is the current step. If brackt is set to true: on input stp must be between stx and sty. On exit stp is a new trial step.
     * @param fp      On entry fp is the function at stp.
     * @param dp      dp is the the derivative of the function at stp.
     * @param stpmin  A lower bound for the step.
     * @param stpmax  An upper bound for the step.
     */
    void dcstep(double& stx, double& fx, double& dx, double& sty, double& fy, double& dy, double& stp,
                const double fp, const double dp, bool& brackt, const double stpmin, const double stpmax);

    /**
     * Finds the minimizer for a cubic polynomial that goes through the points (a,fa), (b,fb), and (c,fc) with derivative at a of fpa.
     * f(x) = A *(x-a)^3 + B*(x-a)^2 + C*(x-a) + D
     * @return Abscissa of minimizer. If no minimizer can be found return NaN.
     */
    double cubicMin(double a, double fa, double fpa, double b, double fb, double c, double fc);

    /**
     * Finds the minimizer for a quadratic polynomial that goes through the points (a,fa), (b,fb) with derivative at a of fpa.
     * f(x) = B*(x-a)^2 + C*(x-a) + D
     * @return Abscissa of minimizer. If no minimizer can be found return NaN.
     */
    double quadMin(double a, double fa, double fpa, double b, double fb);


    /**
     * @class Dcsrch
     * Computes a safeguarded step for a search procedure and updates an interval
     * that contains a step that satisfies a sufficient decrease and a curvature
     * condition.
     */
    class Dcsrch {

     public:

      enum Status { RUNNING, CONVERGED, WARNING };

      /**
       * Constructor
       * @param stepLengthInit
       * @param error Error at stepLengthInit
       * @param errorDerivative Error derivative at stepLengthInit
       * @param minStepLength Minimum allowed step length
       * @param maxStepLength Maximum allowed step length
       * @param ftol A nonnegative value for the sufficient decrease condition
       * @param xtol A nonnegative relative tolerance for an acceptable step.
       * @param gtol A nonnegative value for the curvature condition
       */
      Dcsrch(double stepLengthInit, double error, double errorDerivative, double minStepLength, double maxStepLength, double ftol, double xtol, double gtol);

      /**
       * Finds a step that satisfies a sufficient decrease condition and a curvature condition.
       * https://github.com/funsim/moola/blob/master/moola/linesearch/dcsrch.py
       * @param stepLength     On entry stepLength is the current estimate of a satisfactory step. On initial entry, a positive initial estimate must be provided.
       *                On exit stp is the current estimate of a satisfactory step if task = 'FG'. If task = 'CONV' then stp satisfies the sufficient decrease and curvature condition.
       * @param f       On initial entry f is the value of the function at 0. On subsequent entries f is the value of the function at stp.
       * @param g       On initial entry g is the derivative of the function at 0. On subsequent entries g is the derivative of the function at stp.
       */
      double updateStepLength(double error, double errorDerivative);

      /// \brief Getter for the status of the search
      Status status() const { return _status; }

     private:

      bool _brackt = false; /// \brief Specifies if a minimizer has been bracketed.
      Status _status = RUNNING; /// \brief The status of the search
      std::size_t _stage = 0; /// \brief  The stage of the search

      /**
       *  \brief A nonnegative relative tolerance for an acceptable step. The
       *  algorithm exits with a warning if the relative difference between sty
       *  and stx is less than xtol.
       */
      const double _xtol;

      const double _ftol; /// \brief A nonnegative tolerance for the sufficient decrease condition.
      const double _gtol;/// \brief A nonnegative tolerance for the curvature condition.
      const double _minStepLength;/// \brief A nonnegative lower bound for the step length.
      const double _maxStepLength;/// \brief A nonnegative upper bound for the step length.
      const double _xtrapl = 1.1;
      const double _xtrapu = 4.0;

      /**
       * \brief The current estimate of a satisfactory step. If _status is
       * CONVERGED, then _stepLength satisfies the sufficient decrease and
       * curvature condition.
       */
      double _stepLength;

      const double _finit;  /// \brief Initial function (error) value
      double _fx;           /// \brief Function (error) value at point x
      double _fy;           /// \brief Function (error) value at point y
      const double _ginit;  /// \brief Initial function (error) derivative
      const double _gtest;
      const double _ginitTimesNegGtol; /// \brief Helper variable to save computation time
      double _gx;           /// \brief Function (error) derivative at point x
      double _gy;           /// \brief Function (error) derivative at point y
      double _stx = 0.0;    /// \brief The best step obtained so far. stx is an endpoint of the interval that contains the minimizer.
      double _sty = 0.0;    /// \brief The second endpoint of the interval that contains the minimizer.
      double _stmin = 0.0;  /// \brief Interval point x
      double _stmax;        /// \brief Interval point y
      double _width;        /// \brief Width of the interval
      double _width1;       /// \brief Width of the interval
    };


    struct LineSearchOptions {
      LineSearchOptions();
      LineSearchOptions(const sm::PropertyTree& config);
      void check() const;

      double c1WolfeCondition = 1e-4; /// \brief A nonnegative tolerance for the sufficient decrease condition
      double c2WolfeCondition = 0.9;  /// \brief A nonnegative tolerance for the curvature condition
      double maxStepLength = 50;      /// \brief A nonnegative upper bound for the step.
      double minStepLength = 1e-8;    /// \brief A nonnegative lower bound for the step.
      double xtol = 1e-14;            /// \brief A nonnegative relative tolerance for an acceptable step.
      double initialStepLength = 1.0; /// \brief A nonnegative initial step length.
      std::size_t nMaxIterWolfe1 = 30; /// \brief Maximum number of iterations for method wolfe1
      std::size_t nMaxIterWolfe2 = 10; /// \brief Maximum number of iterations for method wolfe2
      std::size_t nMaxIterZoom = 10;   /// \brief Maximum number of iterations for the internal zoom method

      template<class Archive>
      inline void serialize(Archive & ar, const unsigned int version);
    };

    std::ostream& operator<<(std::ostream& out, const aslam::backend::LineSearchOptions& options);

    /**
     * \class LineSearch
     *
     * Various line-search methods
     */
    class LineSearch {

    public:// public methods

      typedef boost::shared_ptr<LineSearch> Ptr;
      typedef boost::shared_ptr<const LineSearch> ConstPtr;

      /// \brief Constructor with default options
      LineSearch(const boost::shared_ptr<CostFunctionInterface>& cf);
      /// \brief Constructor with custom options
      LineSearch(const boost::shared_ptr<CostFunctionInterface>& cf, const LineSearchOptions& options);
      /// \brief Constructor from property tree
      LineSearch(const boost::shared_ptr<CostFunctionInterface>& cf, const sm::PropertyTree& config);
      /// \brief Destructor
      ~LineSearch();

      /// \brief Mutable getter for the options.
      LineSearchOptions& options() { return _options; }

      /// \brief Const getter for the options.
      const LineSearchOptions& options() const { return _options; }

      /**
       * @brief Initialize the class. Mandatory to call before using any of the line-search methods.
       * Do not call initialize() in case you perform subsequent line-searches always starting at the endpoint of the previous search but
       * rather use setSearchDirection() in case the search direction has to be modified.
       * @param searchDirection Direction of the search. Often you want to use -gradient. You may pass it later via setSearchDirection().
       * @param error Optionally specify the error at the current point. If not supplied, initialize() will recompute it.
       * @param gradient Optionally specify the gradient at the current point. If not supplied, initialize() will recompute it.
       */
      void initialize(boost::optional<const RowVectorType&> searchDirection = boost::optional<const RowVectorType&>(),
                      boost::optional<double> error = boost::optional<double>(),
                      boost::optional<const RowVectorType&> gradient = boost::optional<const RowVectorType&>());

      /**
       * Modifies the current search direction
       * @param searchDirection New search direction
       */
      void setSearchDirection(const RowVectorType& searchDirection);

      /**
       * Set a callback when the error is evaluated
       */
      inline void setEvaluateErrorCallback(const boost::function<void(void)>& cb);

      /**
       * Set a callback when the gradient is evaluated
       */
      inline void setEvaluateGradientCallback(const boost::function<void(void)>& cb);

      /**
       * Search for a step length that satisfies strong Wolfe conditions
       * @return Successful or not
       */
      bool lineSearchWolfe1();

      /**
       * Search for a step length that satisfies strong Wolfe conditions.
       * Uses the line search algorithm to enforce strong Wolfe conditions. See Wright and Nocedal, 'Numerical Optimization', 1999, pg. 59-60.
       * Adapted from scipy https://github.com/scipy/scipy/blob/master/scipy/optimize/linesearch.py#L296
       * @return Successful or not
       */
      bool lineSearchWolfe2();

      /**
       * Search for a step length that satisfies strong Wolfe conditions
       * Same as lineSearchWolfe1, but fall back to lineSearchWolfe2 if suitable step length is not found.
       * @return Successful or not
       */
      bool lineSearchWolfe12();

      /**
       * Updates the state with a step of length s and the related error and gradient information at the new location
       * @param s step length
       * @param gradient Whether or not to update gradient related information.
       */
      void applyStateUpdate(const double s);

      /**
       * Updates the error-related information of the class
       */
      void updateError();

      /**
       * Updates the gradient at the current state
       */
      void updateGradient();

      /**
       * Updates the gradient-related information of the class
       */
      void updateErrorDerivative();

      /**
       * Computes the derivative of the error in the search direction
       * @return derivative of the error in the search direction
       */
      double computeErrorDerivative() const;

      /**
       * Return the error at the current point, checking whether the information is up to date
       * @return error at the current point
       */
      inline double getError() const;

      /**
       * Return the derivative of the error in the search direction at the current point, checking whether the information is up to date
       * @return derivative of the error in the search direction at the current point
       */
      inline double getErrorDerivative() const;

      /**
       * Return the gradient at the current point
       * @return gradient at the current point
       */
      inline const RowVectorType& getGradient() const;

      /**
       * Getter for the current step length
       */
      inline double getCurrentStepLength() const;

    private: // private methods

      /**
       * Part of the optimization algorithm in scalarSearchWolfe2.
       */
      bool zoom(double minStepLength, double maxStepLength, double error_lo, double error_hi, double derror_lo, double error0, double derror0);

    private: // private members

      /// \brief Cost function
      boost::shared_ptr<CostFunctionInterface> _costFunction;

      /// \brief current search direction
      RowVectorType _searchDirection;

      /// \brief gradient at current point
      RowVectorType _gradient;

      /// \brief current step length
      double _stepLength = 0.0;

      /// \brief Error at the current point xk
      double _error = std::numeric_limits<double>::signaling_NaN();

      /// \brief Error at the previous step xk-1, i.e. beginning of last line search
      double _errorOld = std::numeric_limits<double>::signaling_NaN();

      /// \brief Value of the gradient of the error in the search direction at the current point
      double _derror = std::numeric_limits<double>::signaling_NaN();

      /// \brief Whether an update of the error-related information is neccesary
      bool _errorOutdated = true;

      /// \brief Whether an update of the gradient-related information is neccesary
      bool _derrorOutdated = true;

      /// \brief Callback  that is called when the objective function is evaluated
      boost::function<void(void)> _evalErrorCallback;

      /// \brief Callback  that is called when the gradient is evaluated
      boost::function<void(void)> _evalGradCallback;

      /// \brief the current set of options
      LineSearchOptions _options;

    };






    // Inlined methods

    inline void LineSearch::setEvaluateErrorCallback(const boost::function<void(void)>& cb) {
      _evalErrorCallback = cb;
    }

    inline void LineSearch::setEvaluateGradientCallback(const boost::function<void(void)>& cb) {
      _evalGradCallback = cb;
    }


    inline double LineSearch::getError() const {
      SM_ASSERT_FALSE(Exception, _errorOutdated, "Missing call to updateError()");
      return _error;
    }
    inline double LineSearch::getErrorDerivative() const {
      SM_ASSERT_FALSE(Exception, _derrorOutdated, "Missing call to updateErrorDerivative()");
      return _derror;
    }
    inline const RowVectorType& LineSearch::getGradient() const {
      return _gradient;
    }
    inline double LineSearch::getCurrentStepLength() const {
      return _stepLength;
    }


  } // namespace backend
} // namespace aslam

#include "implementation/LineSearchImpl.hpp"

#endif /* ASLAM_BACKEND_LINESEARCH_HPP */
