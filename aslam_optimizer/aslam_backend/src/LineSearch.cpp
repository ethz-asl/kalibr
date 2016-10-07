#include <cmath>
#include <aslam/backend/util/utils.hpp>
#include <aslam/backend/LineSearch.hpp>
#include <Eigen/Dense>
#include <sm/eigen/assert_macros.hpp>
#include <sm/PropertyTree.hpp>
#include <sm/logging.hpp>

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

using namespace std;

namespace aslam {
namespace backend {

using std::isnan;

void dcstep(double& stx, double& fx, double& dx, double& sty, double& fy, double& dy, double& stp,
            const double fp, const double dp, bool& brackt, const double stpmin, const double stpmax) {

  double sgnd = dp*(utils::sign(dx));
  double stpf;

  // First case: A higher function value. The minimum is bracketed.
  // If the cubic step is closer to stx than the quadratic step, the
  // cubic step is taken, otherwise the average of the cubic and
  // quadratic steps is taken.
  if (fp > fx) {

    const double theta = 3.0*(fx-fp)/(stp-stx) + dx + dp;
    const double s = max(abs(theta), max(abs(dx), abs(dp)));
    const double srec = 1./s;
    double gamma = s*sqrt(utils::sqr(theta*srec)-(dx*srec)*(dp*srec));
    if (stp < stx)
        gamma = -gamma;
    const double p = (gamma-dx) + theta;
    const double q = ((gamma-dx)+gamma) + dp;
    const double r = p/q;
    const double stpc = stx + r*(stp-stx);
    const double stpq = stx + ((dx/((fx-fp)/(stp-stx)+dx))/2.0)*(stp-stx);
    if (abs(stpc-stx) < abs(stpq-stx))
      stpf = stpc;
    else
      stpf = stpc + (stpq-stpc)/2.0;

    brackt = true;

  // Second case: A lower function value and derivatives of opposite
  // sign. The minimum is bracketed. If the cubic step is farther from
  // stp than the secant step, the cubic step is taken, otherwise the
  // secant step is taken.
  } else if (sgnd < 0.0) {

    const double theta = 3.0*(fx-fp)/(stp-stx) + dx + dp;
    const double s = max(abs(theta), max(abs(dx), abs(dp)));
    const double srec = 1./s;
    double gamma = s*sqrt(utils::sqr(theta*srec)-(dx*srec)*(dp*srec));
    if (stp > stx)
      gamma = -gamma;
    const double p = (gamma-dp) + theta;
    const double q = ((gamma-dp)+gamma) + dx;
    const double r = p/q;
    const double stpc = stp + r*(stx-stp);
    const double stpq = stp + (dp/(dp-dx))*(stx-stp);
    if (abs(stpc-stp) > abs(stpq-stp))
      stpf = stpc;
    else
      stpf = stpq;
    brackt = true;

  // Third case: A lower function value, derivatives of the same sign,
  // and the magnitude of the derivative decreases.
  } else if (abs(dp) < abs(dx)) {

      // The cubic step is computed only if the cubic tends to infinity
      // in the direction of the step or if the minimum of the cubic
      // is beyond stp. Otherwise the cubic step is defined to be the
      // secant step.
      const double theta = 3.0*(fx-fp)/(stp-stx) + dx + dp;
      const double s = max(abs(theta), max(abs(dx), abs(dp)));

      // The case gamma = 0 only arises if the cubic does not tend
      // to infinity in the direction of the step.
      const double srec = 1./s;
      double gamma = s*sqrt(max(0.0, utils::sqr(theta*srec)-(dx*srec)*(dp*srec)));
      if (stp > stx)
        gamma = -gamma;
      const double p = (gamma-dp) + theta;
      const double q = (gamma+(dx-dp)) + gamma;
      const double r = p/q;
      double stpc;
      if (r < 0.0 and gamma != 0.0)
        stpc = stp + r*(stx-stp);
      else if (stp > stx)
        stpc = stpmax;
      else
        stpc = stpmin;

      const double stpq = stp + (dp/(dp-dx))*(stx-stp);

      if (brackt) {

        // A minimizer has been bracketed. If the cubic step is
        // closer to stp than the secant step, the cubic step is
        // taken, otherwise the secant step is taken.

        if (abs(stpc-stp) < abs(stpq-stp))
          stpf = stpc;
        else
          stpf = stpq;

        if (stp > stx)
          stpf = min(stp+0.66*(sty-stp),stpf);
        else
          stpf = max(stp+0.66*(sty-stp),stpf);

      } else {

        // A minimizer has not been bracketed. If the cubic step is
        // farther from stp than the secant step, the cubic step is
        // taken, otherwise the secant step is taken.

        if (abs(stpc-stp) > abs(stpq-stp))
          stpf = stpc;
        else
          stpf = stpq;

        stpf = max( min(stpmax,stpf), stpmin);
      }

  // Fourth case: A lower function value, derivatives of the same sign,
  // and the magnitude of the derivative does not decrease. If the
  // minimum is not bracketed, the step is either stpmin or stpmax,
  // otherwise the cubic step is taken.
  } else {

    if (brackt) {
      const double theta = 3.0*(fp-fy)/(sty-stp) + dy + dp;
      const double s = max(abs(theta), max(abs(dy), abs(dp)));
      const double srec = 1./s;
      double gamma = s*sqrt(utils::sqr(theta*srec)-(dy*srec)*(dp*srec));
      if (stp > sty)
          gamma = -gamma;
      const double p = (gamma-dp) + theta;
      const double q = ((gamma-dp)+gamma) + dy;
      const double r = p/q;
      const double stpc = stp + r*(sty-stp);
      stpf = stpc;

    } else if (stp > stx) {
      stpf = stpmax;
    } else {
      stpf = stpmin;
    }
  }

  // Update the interval which contains a minimizer.
  if (fp > fx) {
    sty = stp;
    fy = fp;
    dy = dp;
  } else {
    if (sgnd < 0) {
      sty = stx;
      fy = fx;
      dy = dx;
    }

    stx = stp;
    fx = fp;
    dx = dp;
  }

  // Compute the new step.
  stp = stpf;
}


double cubicMin(double a, double fa, double fpa, double b, double fb, double c, double fc) {
  const double db = b - a;
  const double dc = c - a;
  const double denom = utils::sqr(db * dc) * (db - dc);
  Eigen::Matrix2d d1;
  d1(0,0) = utils::sqr(dc);
  d1(0,1) = -utils::sqr(db);
  d1(1,0) = -d1(0,0)*dc;
  d1(1,1) = -d1(0,1)*db;
  Eigen::Vector2d AB = d1*(Eigen::Vector2d(fb - fa - fpa * db, fc - fa - fpa * dc))/denom;
  const double radical = utils::sqr((double)AB[1]) - 3.0 * AB[0] * fpa;
  return a + (-AB[1] + sqrt(radical)) / (3.0 * AB[0]);
}

double quadMin(double a, double fa, double fpa, double b, double fb) {
  const double db = b - a;
  const double B = (fb - fa - fpa * db) / utils::sqr(db);
  return a - 0.5 * fpa / B;
}


Dcsrch::Dcsrch(double stepLengthInit, double error, double errorDerivative, double minStepLength, double maxStepLength, double ftol, double xtol, double gtol) :
    _xtol(xtol),
    _ftol (ftol),
    _gtol(gtol),
    _minStepLength(minStepLength),
    _maxStepLength(maxStepLength),
    _stepLength(stepLengthInit),
    _finit(error),
    _fx(error),
    _fy(error),
    _ginit(errorDerivative),
    _gtest(_ftol*_ginit),
    _ginitTimesNegGtol(-_gtol*_ginit),
    _gx(errorDerivative),
    _gy(errorDerivative),
    _stmax(stepLengthInit + _xtrapu*stepLengthInit),
    _width(maxStepLength - minStepLength),
    _width1(_width*2.0)
{

  SM_ASSERT_GT_DBG(Exception, _xtol, 0.0, "");
  SM_ASSERT_GT_DBG(Exception, _ftol, 0.0, "");
  SM_ASSERT_GT_DBG(Exception, _gtol, 0.0, "");
  SM_ASSERT_GT_DBG(Exception, _minStepLength, 0.0, "");
  SM_ASSERT_GT_DBG(Exception, _maxStepLength, 0.0, "");

  SM_ASSERT_LT(Exception, errorDerivative, 0.0, "");

  SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "Dcsrch: initial interval: [" << _stx << ", " << _sty << "], step length: " << _stepLength);

}

double Dcsrch::updateStepLength(double error, double errorDerivative) {

  if (_stage == 0) {
    _stage++;
    return _stepLength;
  }

  // If psi(stepLength) <= 0 and f'(stepLength) >= 0 for some step, then the
  // algorithm enters the second stage.
  const double ftest = _finit + _stepLength*_gtest;
  if (_stage == 1 && error <= ftest && errorDerivative >= 0.0)
    _stage = 2;

  SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: dcsrch -- stage: " << _stage << ", ftest: " << ftest <<
                      ", minimum step length: " << _stmin << ", maximum step length: " << _stmax);

  // Test for warnings.
  if (_brackt && (_stepLength <= _stmin || _stepLength >= _stmax)) {
    SM_WARN_STREAM(setprecision(20) << "LineSearch: dcsrch -- Rounding errors prevent progress: step length " << _stepLength <<
            " outside interval (" << _stmin << ", " << _stmax << ")");
    _status = WARNING;
  }
  if (_brackt && _stmax-_stmin <= _xtol*_stmax) {
    SM_WARN_STREAM(setprecision(20) << "LineSearch: dcsrch -- xtol test satisfied: " << (_stmax-_stmin) << " <= " << _xtol*_stmax);
    _status = WARNING;
  }
  if (_stepLength == _maxStepLength && error <= ftest && errorDerivative <= _gtest) {
    SM_WARN("LineSearch: dcsrch -- step length reached maximum step length");
    _status = WARNING;
  }
  if (_stepLength == _minStepLength && (error > ftest || errorDerivative >= _gtest)) {
    SM_WARN("LineSearch: dcsrch -- step length reached minimum step length");
    _status = WARNING;
  }

  // Test for convergence.
  if (error <= ftest) {

    SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: dcsrch -- sufficient decrease condition satisfied: " << error << " <= " << ftest);

    if (abs(errorDerivative) <= _ginitTimesNegGtol)
      _status = CONVERGED;
    else
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: dcsrch -- curvature condition not satisfied: " << abs(errorDerivative) << " <= " << _ginitTimesNegGtol);

  } else {
    SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: dcsrch -- sufficient decrease condition not satisfied: " << error << " <= " << ftest);
  }

  // Test for termination.
  if (_status == WARNING || _status == CONVERGED)
    return _stepLength;

  // A modified function is used to predict the step during the
  // first stage if a lower function value has been obtained but
  // the decrease is not sufficient.
  if (_stage == 1 && error <= _fx && error > ftest) {

    // Define the modified function and derivative values.
    const double fm = error - _stepLength*_gtest;
    double fxm = _fx - _stx*_gtest;
    double fym = _fy - _sty*_gtest;
    double gm = errorDerivative - _gtest;
    double gxm = _gx - _gtest;
    double gym = _gy - _gtest;

    // Call dcstep to update stx, sty, and to compute the new step.
    dcstep(_stx, fxm, gxm, _sty, fym, gym, _stepLength, fm, gm, _brackt, _stmin, _stmax);

    // Reset the function and derivative values for error.
    _fx = fxm + _stx*_gtest;
    _fy = fym + _sty*_gtest;
    _gx = gxm + _gtest;
    _gy = gym + _gtest;

  } else {
    // Call dcstep to update stx, sty, and to compute the new step.
    dcstep(_stx, _fx, _gx, _sty, _fy, _gy, _stepLength, error, errorDerivative, _brackt, _stmin, _stmax);
  }

  // Decide if a bisection step is needed.
  if (_brackt) {
    const double dstxy = _sty - _stx;
    const double absdstxy = abs(dstxy);
    if (absdstxy >= 0.66 * _width1)
      _stepLength = _stx + 0.5*dstxy;
    _width1 = _width;
    _width = absdstxy;
  }

  // Set the minimum and maximum steps allowed for stepLength.
  if (_brackt) {
    _stmin = min(_stx,_sty);
    _stmax = max(_stx,_sty);
  } else {
    const double dstpx = _stepLength - _stx;
    _stmin = _stepLength + _xtrapl*dstpx;
    _stmax = _stepLength + _xtrapu*dstpx;
  }

  // Force the step to be within the bounds.
  _stepLength = min( max(_stepLength, _minStepLength), _maxStepLength);

  // If further progress is not possible, let stepLength be the best
  // point obtained during the search.
  if ((_brackt && (_stepLength <= _minStepLength || _stepLength >= _maxStepLength)) ||
      (_brackt && _maxStepLength-_minStepLength <= _xtol*_maxStepLength))
    _stepLength = _stx;

  SM_ALL_STREAM_NAMED("optimization", "Dcsrch: new interval: [" << _stx << ", " << _sty << "], step length = " << _stepLength);

  return _stepLength;
}



LineSearchOptions::LineSearchOptions() {
  check();
}

LineSearchOptions::LineSearchOptions(const sm::PropertyTree& config)
{
  c1WolfeCondition = config.getDouble("c1WolfeCondition", c1WolfeCondition);
  c2WolfeCondition = config.getDouble("c2WolfeCondition", c2WolfeCondition);
  maxStepLength = config.getDouble("maxStepLength", maxStepLength);
  minStepLength = config.getDouble("minStepLength", minStepLength);
  xtol = config.getDouble("xtol", xtol);
  initialStepLength = config.getDouble("initialStepLength", initialStepLength);
  nMaxIterWolfe1 = config.getInt("nMaxIterWolfe1", nMaxIterWolfe1);
  nMaxIterWolfe2 = config.getInt("nMaxIterWolfe2", nMaxIterWolfe2);
  nMaxIterZoom = config.getInt("nMaxIterZoom", nMaxIterZoom);
  check();
}

void LineSearchOptions::check() const {
  SM_ASSERT_GE(Exception, c1WolfeCondition, 0.0, "");
  SM_ASSERT_GE(Exception, c2WolfeCondition, c1WolfeCondition, "");
  SM_ASSERT_GE(Exception, maxStepLength, 0.0, "");
  SM_ASSERT_GE(Exception, minStepLength, 0.0, "");
  SM_ASSERT_GE(Exception, xtol, 0.0, "");
  SM_ASSERT_GT(Exception, initialStepLength, 0.0, "");
  SM_ASSERT_GT(Exception, nMaxIterWolfe1, 0, "");
  SM_ASSERT_GT(Exception, nMaxIterWolfe2, 0, "");
  SM_ASSERT_GT(Exception, nMaxIterZoom, 0, "");
}

ostream& operator<<(ostream& out, const aslam::backend::LineSearchOptions& options)
{
  out << "LineSearchOptions:\n";
  out << "\tc1WolfeCondition: " << options.c1WolfeCondition << endl;
  out << "\tc1WolfeCondition: " << options.c1WolfeCondition << endl;
  out << "\tc2WolfeCondition: " << options.c2WolfeCondition << endl;
  out << "\tmaxStepLength: " << options.maxStepLength << endl;
  out << "\tminStepLength: " << options.minStepLength << endl;
  out << "\txtol: " << options.xtol << endl;
  out << "\tinitialStepLength: " << options.initialStepLength << endl;
  out << "\tnMaxIterWolfe1: " << options.nMaxIterWolfe1 << endl;
  out << "\tnMaxIterWolfe2: " << options.nMaxIterWolfe2 << endl;
  out << "\tnMaxIterZoom: " << options.nMaxIterZoom;
  return out;
}


LineSearch::LineSearch(const boost::shared_ptr<CostFunctionInterface>& cf, const LineSearchOptions& options) :
    _costFunction(cf),
    _options(options)
{
  SM_ASSERT_TRUE(Exception, cf != nullptr, "");
  _options.check();
}

LineSearch::LineSearch(const boost::shared_ptr<CostFunctionInterface>& cf) :
    LineSearch::LineSearch(cf, LineSearchOptions())
{
}

LineSearch::LineSearch(const boost::shared_ptr<CostFunctionInterface>& cf, const sm::PropertyTree& config) :
    LineSearch::LineSearch(cf, LineSearchOptions(config))
{
}

LineSearch::~LineSearch()
{

}


void LineSearch::initialize(boost::optional<const RowVectorType&> searchDirection /*= boost::optional<const RowVectorType&>()*/,
                            boost::optional<double> error /*= boost::optional<double>()*/,
                            boost::optional<const RowVectorType&> gradient /*= boost::optional<const RowVectorType&>()*/)
{
  _stepLength = 0.0;
  _errorOutdated = _derrorOutdated = true;
  _errorOld = std::numeric_limits<double>::signaling_NaN();

  if (error)
    _error = error.get();
  else
    this->updateError();
  _errorOutdated = false;

  if (gradient)
    _gradient = gradient.get();
  else
    this->updateGradient();

  if (searchDirection)
    this->setSearchDirection(searchDirection.get());
  else
    _searchDirection = RowVectorType::Zero(0);

}


void LineSearch::setSearchDirection(const RowVectorType& searchDirection) {
  using namespace Eigen;
  _stepLength = 0.0; // if the search direction changed, we must avoid skipping updates with same step lengths
  _searchDirection = searchDirection;
  _derror = computeErrorDerivative();
  _derrorOutdated = false;
  SM_VERBOSE_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: set search direction to " << _searchDirection.format(IOFormat(15, DontAlignCols, ", ", ", ", "", "", "[", "]")));
  SM_VERBOSE_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: computed error derivative " << _derror);
  SM_ASSERT_LE(Exception, _derror, 0.0, "Wrong search direction supplied! In case approximate Hessian information is used, "
      "this could mean your Hessian estimate became negative");
}


void LineSearch::applyStateUpdate(const double s) {

  using namespace Eigen;
  static IOFormat fmt(15, 0, ", ", ", ", "", "", "[", "]");

  double ds = s - _stepLength;
  _stepLength = s;

  if (ds != 0.0) { // save computation time
    Eigen::RowVectorXd p = sm::logging::getLevel() <= sm::logging::Level::Verbose ?
        utils::getFlattenedDesignVariableParameters(_costFunction->getDesignVariables()).transpose() :  Eigen::RowVectorXd();
    utils::applyStateUpdate(_costFunction->getDesignVariables(), ds*_searchDirection);
    _errorOutdated = _derrorOutdated = true;
    SM_VERBOSE_STREAM_NAMED("optimization", "LineSearch: update step length " << s - ds << " -> " << _stepLength << " (ds: " << ds<< ")");
    SM_VERBOSE_STREAM_NAMED("optimization", "LineSearch: update state" << std::endl <<
                            "Old  : " << p.format(fmt) << std::endl <<
                            "New  : " << utils::getFlattenedDesignVariableParameters(_costFunction->getDesignVariables()).transpose().format(fmt) << std::endl <<
                            "Delta: " << (utils::getFlattenedDesignVariableParameters(_costFunction->getDesignVariables()).transpose() - p).format(fmt));
  } else {
    SM_ALL_NAMED("optimization", "LineSearch: skipping unnecessary update of information");
  }
}


void LineSearch::updateError() {
  if (_errorOutdated) {
    const double errorOld = _error;
    _error = _costFunction->evaluateError();
    if (_evalErrorCallback) _evalErrorCallback();
    SM_VERBOSE_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: update error " << errorOld << " -> " << _error << " (" << _error - errorOld << ")");
  }
  _errorOutdated = false;
}

void LineSearch::updateGradient() {
  _costFunction->computeGradient(_gradient);
  if (_evalGradCallback) _evalGradCallback();
}

void LineSearch::updateErrorDerivative() {
  if (_derrorOutdated) {
    const double dErrorOld = _derror;
    this->updateGradient();
    _derror = computeErrorDerivative();
    SM_VERBOSE_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: update error derivative "<< dErrorOld << " -> " << _derror << " (" << _derror - dErrorOld << ")");
  }
  _derrorOutdated = false;
}

bool LineSearch::zoom(double minStepSize, double maxStepSize, double error_lo, double error_hi, double derror_lo, double error0, double derror0) {

  size_t i = 0;
  const double delta1 = 0.2;  // cubic interpolant check
  const double delta2 = 0.1;  // quadratic interpolant check
  double error_rec = error0;
  double stepSize_rec = 0.0;

  while (true) {
    // Interpolate to find a trial step length between a_lo and a_hi.
    // Use cubic interpolation in the first step.
    // If the result is within delta * dalpha or outside of the bounded interval defined by a_lo or a_hi use quadratic interpolation.
    // If the result is still too close, then use bisection

    const double dStepLength = maxStepSize - minStepSize;
    double a, b;
    if (dStepLength < 0.0) {
      a = maxStepSize;
      b = minStepSize;
    } else {
      a = minStepSize;
      b = maxStepSize;
    }

    // Try cubic interpolation
    double cubicchk;
    double stepSize_j = numeric_limits<double>::signaling_NaN();
    if (i > 0) {
      cubicchk = delta1 * dStepLength;
      stepSize_j = cubicMin(minStepSize, error_lo, derror_lo, maxStepSize, error_hi, stepSize_rec, error_rec);
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- cubic interpolation through points [" << minStepSize <<
                          ", " << maxStepSize << ", " << stepSize_rec << "] with errors " << "[" << error_lo << ", " << error_hi << ", " << error_rec <<
                          "] and derivative at lower interval point " << derror_lo << " returned step length " << stepSize_j);
    }

    // Try quadratic interpolation
    if (i == 0 || isnan(stepSize_j) || stepSize_j > b - cubicchk || stepSize_j < a + cubicchk) {
      const double quadchk = delta2 * dStepLength;
      stepSize_j = quadMin(minStepSize, error_lo, derror_lo, maxStepSize, error_hi);
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- quadratic interpolation through points [" << minStepSize <<
                          ", " << maxStepSize << "] with errors " << "[" << error_lo << ", " << error_hi << "] and derivative at lower interval point " <<
                          derror_lo << " returned step length " << stepSize_j);
      if (isnan(stepSize_j) || stepSize_j > b - quadchk || stepSize_j < a + quadchk)
        stepSize_j = minStepSize + 0.5*dStepLength;
    }

    // Move state to stepSize_j, do not compute gradient information at new point yet since
    // we have to check first whether the error decreased. If not, there's no need to compute the gradient.
    this->applyStateUpdate(stepSize_j);
    this->updateError();

    // Check new value of stepSize_j
    const double error_j = getError();

    // Check Wolfe condition 1 (Armijo rule)
    if ((error_j > error0 + _options.c1WolfeCondition*stepSize_j*derror_lo) or (error_j >= error_lo)) {
      // If condition is not satisfied, set endpoint of interval to new point stepSize_j
      error_rec = error_hi;
      stepSize_rec = maxStepSize;
      maxStepSize = stepSize_j;
      error_hi = error_j;
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- sufficient decrease condition not satisfied: " << error_j << " <= " << error0 + _options.c1WolfeCondition*stepSize_j*derror_lo);
    } else {
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- sufficient decrease condition satisfied: " << error_j << " <= " << error0 + _options.c1WolfeCondition*stepSize_j*derror_lo);
      // If Armijo rule is satisfied, also check curvature condition.
      // Therefore we have to update the gradient based information now.
      this->updateErrorDerivative();
      const double derror_j = getErrorDerivative();
      if (abs(derror_j) <= -_options.c2WolfeCondition*derror0)  { // If curvature condition is satisfied, we found a suitable point
        SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- curvature condition satisfied: " << abs(derror_j) << " <= " << -_options.c2WolfeCondition*derror0);
        break;
      }

      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- curvature condition not satisfied: " << abs(derror_j) << " <= " << -_options.c2WolfeCondition*derror0);

      if (derror_j*(maxStepSize - minStepSize) >= 0) {
        error_rec = error_hi;
        stepSize_rec = maxStepSize;
        maxStepSize = minStepSize;
        error_hi = error_lo;
      } else {
        error_rec = error_lo;
        stepSize_rec = minStepSize;
      }
      minStepSize = stepSize_j;
      error_lo = error_j;
      derror_lo = derror_j;
    }

    i++;
    if (i == _options.nMaxIterZoom) {
      SM_ERROR("LineSearch: zoom -- Failed to find a conforming step size");
      return false;
    }

    SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: zoom -- update interval: [" << minStepSize << ", " << maxStepSize << "]");
  }

  return true;
}

double LineSearch::computeErrorDerivative() const {
  return _gradient*_searchDirection.transpose();
}


bool LineSearch::lineSearchWolfe1() {

  // Check that the error and gradient information is up to date and not NaN
  SM_ASSERT_FALSE(Exception, isnan(getError()), "");
  SM_ASSERT_FALSE(Exception, isnan(getErrorDerivative()), "");

  double stepLength = _options.initialStepLength;
  if (!isnan(_errorOld) && _derror != 0.0) {
    stepLength = min(_options.maxStepLength, 1.01*2.0*(_error - _errorOld)/_derror);
    if (stepLength < 0.0) stepLength = _options.initialStepLength;
  }

  _errorOld = _error;

  SM_ASSERT_GE(Exception, stepLength, _options.minStepLength, "");
  SM_ASSERT_LE(Exception, stepLength, _options.maxStepLength, "");

  SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe1 -- starting line search at error value " <<
                      _error << " and derivative " << _derror);

  if (_derror == 0.0) {
    SM_FINE_STREAM_NAMED("optimization", "LineSearch: Error derivative is zero, seems like the system is at its optimum.");
    return true;
  }

  bool success = false;
  bool terminate = false;
  Dcsrch dcsrch(stepLength, getError(), getErrorDerivative(), _options.minStepLength,
                _options.maxStepLength, _options.c1WolfeCondition, _options.xtol, _options.c2WolfeCondition);

  size_t cnt = 0;
  while(!terminate && cnt < _options.nMaxIterWolfe1) {

    SM_ALL_STREAM_NAMED("optimization", "LineSearch: wolfe1 -- iteration " << cnt);

    const double stp = dcsrch.updateStepLength(getError(), getErrorDerivative());

    switch(dcsrch.status()) {
      case Dcsrch::RUNNING:
        stepLength = stp;
        this->applyStateUpdate(stp);
        this->updateError();
        this->updateErrorDerivative();
        break;
      case Dcsrch::CONVERGED:
        SM_FINE_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe1 -- converged, final step length " << stp <<
                              ", final error " << getError() << ", final error derivative " << getErrorDerivative());
        success = terminate = true;
        break;
      case Dcsrch::WARNING:
        terminate = true;
        break;
    }

    cnt++;
  }

  if (cnt == _options.nMaxIterWolfe1) { // maxiter reached, the line search did not converge
    SM_ERROR_STREAM("LineSearch: wolfe1 -- no solution found in " << _options.nMaxIterWolfe1 << " iterations");
    return false;
  }

  if (!success) {
    SM_ERROR("LineSearch: wolfe1 -- dcsrch exited with a warning");
    return false;
  }

  return true;

}

bool LineSearch::lineSearchWolfe2() {

  // Check that the error and gradient information is up to date and not NaN
  SM_ASSERT_FALSE(Exception, isnan(getError()), "");
  SM_ASSERT_FALSE(Exception, isnan(getErrorDerivative()), "");

  double minStepLength = 0.0;
  double maxStepLength = _options.initialStepLength;
  if (!isnan(_errorOld) && _derror != 0) {
    maxStepLength = min(_options.initialStepLength, 1.01*2.0*(_error - _errorOld)/_derror);
    if (maxStepLength < 0.0) maxStepLength = _options.initialStepLength;
  }

  _errorOld = _error;

  SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- starting line search at error value " <<
                      _error << " and derivative " << _derror);

  if (_derror == 0.0) {
    SM_FINE_STREAM_NAMED("optimization", "LineSearch: Error derivative is zero, seems like the system is at its optimum.");
    return true;
  }

  if (maxStepLength == 0.0) {
    SM_WARN("LineSearch: wolfe2 -- Maximum step length is zero. This shouldn't happen. "
        "Perhaps the increment has slipped below machine precision?");
    return false;
  }

  const double error0 = _error;
  const double derror0 = _derror;
  double errorStepMin = error0;
  double derrorStepMin = derror0;

  this->applyStateUpdate(maxStepLength); // Move to position x + maxStepLength*searchDirection
  this->updateError();
  double errorStepMax = getError();
//  double derrorStepMax; // evaluated below

  bool success = false;
  for (size_t i=0; i<_options.nMaxIterWolfe2; ++i) {

    SM_ALL_STREAM_NAMED("optimization", "LineSearch: wolfe2 -- iteration " << i);

    if (maxStepLength == 0.0)
      break;

    // Check Wolfe condition 1 (Armijo rule)
    if ((errorStepMax > error0 + _options.c1WolfeCondition * maxStepLength * derror0) || ((errorStepMax >= errorStepMin) && (i > 0))) {
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- sufficient decrease condition not satisfied: " <<
                          errorStepMax << " <= " <<  error0 + _options.c1WolfeCondition * maxStepLength * derror0);
      // zoom will move the state to a good position in the interval [minStepLength, maxStepLength]
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- calling zoom with interval [ " << minStepLength << ", " << maxStepLength << "] with errors " <<
                          "[ " << errorStepMin << ", " << errorStepMax << "] and derivative at lower interval point " << derrorStepMin);
      success = this->zoom(minStepLength, maxStepLength, errorStepMin, errorStepMax, derrorStepMin, error0, derror0);
      break;
    }

    this->updateErrorDerivative();
    const double derrorStepMax = getErrorDerivative();

    // Check curvature condition
    if ((abs(derrorStepMax) <= -_options.c2WolfeCondition*derror0)) {
      success = true;
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- curvature condition satisfied: " << abs(derrorStepMax) << " <= " << -_options.c2WolfeCondition*derror0);
      break;
    } else {
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- curvature condition not satisfied: " << abs(derrorStepMax) << " <= " << -_options.c2WolfeCondition*derror0);
    }

    if ((derrorStepMax >= 0.0)) {
      SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- calling zoom with interval [ " << maxStepLength << ", " << minStepLength << "] with errors " <<
                          "[ " << errorStepMax << ", " << errorStepMin << "] and derivative at lower interval point " << derrorStepMax);
      success = zoom(maxStepLength, minStepLength, errorStepMax, errorStepMin, derrorStepMax, error0, derror0);
      break;
    }

    double maxStepLengthNew = 2.0 * maxStepLength; // increase by factor of two on each iteration
    minStepLength = maxStepLength;
    maxStepLength = maxStepLengthNew;
    errorStepMin = errorStepMax;

    this->applyStateUpdate(maxStepLength);
    this->updateError();
    errorStepMax = getError();
    derrorStepMin = derrorStepMax;

    SM_ALL_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- update interval: [" << minStepLength << ", " << maxStepLength << "]");

  }

  if (success)
    SM_FINE_STREAM_NAMED("optimization", setprecision(20) << "LineSearch: wolfe2 -- converged, final step length " << getCurrentStepLength() <<
                         ", final error " << getError());
  else
    SM_ERROR_STREAM("LineSearch: wolfe2 -- no solution found in " << _options.nMaxIterWolfe2 << " iterations");

  return success;

}

bool LineSearch::lineSearchWolfe12() {

  const double errorOld0 = _errorOld; // _errorOld gets modified by lineSearchWolfe1
  const double error0 = _error;
  const double derror0 = _derror;

  utils::DesignVariableState dvstate(_costFunction->getDesignVariables());

  if (!lineSearchWolfe1()) {
    SM_FINE_STREAM_NAMED("optimization", "LineSearch: method wolfe1 failed, trying method wolfe2");

    // restore error values to the ones before calling lineSearchWolfe1().
    // These are the values that correspond to step length zero.
    _errorOld = errorOld0;
    _error = error0;
    _derror = derror0;
    _stepLength = 0.0;
    dvstate.restore();

    return lineSearchWolfe2();
  }

  return true;
}

} // namespace backend
} // namespace aslam
