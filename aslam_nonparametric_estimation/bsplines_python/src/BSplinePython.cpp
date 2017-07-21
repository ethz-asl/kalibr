#include <numpy_eigen/boost_python_headers.hpp>

#include <bsplines/BSpline.hpp>

using namespace bsplines;
using namespace boost::python;

  // Function wrappers turn std::pairs into tuples.
boost::python::tuple timeInterval1(const bsplines::BSpline * bs)
{
  std::pair<double,double> ti = bs->timeInterval();
  return boost::python::make_tuple(ti.first,ti.second);
}

boost::python::tuple timeInterval2(const bsplines::BSpline * bs, int i)
{
  std::pair<double,double> ti = bs->timeInterval(i);
  return boost::python::make_tuple(ti.first,ti.second);
}

template <class T> class BiFunction {
private:
	T biVector_;
public:
	BiFunction(const T & biVector):
		biVector_(biVector)
	{

	}
	double getBi(int i){
		// Note this is normally a VectorXd
		// So dynamic rows and 1 column
		return biVector_.coeff(i,0);
	}
};

boost::python::object getBiFunction(const bsplines::BSpline * bs, double t)
{
	typedef Eigen::CwiseNullaryOp <BiVector, Eigen::VectorXd> T;
	return boost::python::make_function(boost::bind(&BiFunction<T>::getBi, BiFunction<T>(bs->getBiVector(t)), _1)
	 	 , boost::python::default_call_policies(), boost::mpl::vector2<double,int>()
	);
}
boost::python::object getCumulativeBiFunction(const bsplines::BSpline * bs, double t)
{
	typedef Eigen::CwiseNullaryOp <BiVector, Eigen::VectorXd> T;
	return boost::python::make_function(boost::bind(&BiFunction<T>::getBi, BiFunction<T>(bs->getCumulativeBiVector(t)), _1)
	 	 , boost::python::default_call_policies(), boost::mpl::vector2<double,int>()
	);
}

void import_bspline_python()
{
  // Function pointers are necessary for boost::python to distinguish between
  // overloaded functions.
  int (BSpline::*numValidTimeSegments1)(int) const = &BSpline::numValidTimeSegments;
  int (BSpline::*numValidTimeSegments2)() const = &BSpline::numValidTimeSegments;



  class_<BSpline>("BSpline",init<int>())
    .def("splineOrder",&BSpline::splineOrder, "The order of the spline")
    .def("polynomialDegree", &BSpline::polynomialDegree, "The degree of the polynomial spline")
    .def("minimumKnotsRequired", &BSpline::minimumKnotsRequired, "The minimum number of knots required based on the spline order")
    .def("numCoefficientsRequired", &BSpline::numCoefficientsRequired, "The number of coefficients required for a specified number of valid time segments")
    .def("numKnotsRequired", &BSpline::numKnotsRequired, "The number of knots required for a target number of valid time segments")
    .def("numValidTimeSegments", numValidTimeSegments1, "The number of valid time segments for a given number of knots")
    .def("numValidTimeSegments", numValidTimeSegments2, "The number of valid time segments for the current knot sequence")
    .def("setKnotVectorAndCoefficients", &BSpline::setKnotVectorAndCoefficients, "Sets the knot vector and spline coefficients")
    .def("knots", &BSpline::knotVector, "returns the current knot sequence")
    .def("coefficients", &BSpline::coefficients, "returns the current coefficient matrix", return_value_policy<copy_const_reference>())
    .def("t_min", &BSpline::t_min, "The minimum time that the spline is well-defined on")
    .def("t_max", &BSpline::t_max, "The maximum time that the spline is well-defined on")
    .def("eval", &BSpline::eval, "Evaluate the spline curve at a point in time")
    .def("evalD", &BSpline::evalD, "Evaluate a spline curve derivative at a point in time")
    .def("Phi", &BSpline::Phi, "Evaluate the local basis matrix at a point in time")
    .def("localBasisMatrix", &BSpline::localBasisMatrix, "Evaluate the local basis matrix at a point in time")
    .def("localCoefficientMatrix", &BSpline::localCoefficientMatrix, "Get the matrix of locally-active coefficients for a specified time in matrix form")
    .def("localCoefficientVector", &BSpline::localCoefficientVector, "Get the stacked vector of locally-active coefficients for a specified time.")
    .def("segmentCoefficientVector", &BSpline::localCoefficientVector, "Get the stacked vector of locally-active coefficients for a specified segment.")
    .def("initSpline", &BSpline::initSpline, "Initialize the spline to interpolate a set of points")
    .def("initSpline3", &BSpline::initSpline3, "Initialize the spline to interpolate a set of points")
    .def("initSplineSparse", &BSpline::initSplineSparse, "Initialize the spline to interpolate a set of points (Sparse Solution)")
    .def("initSplineSparseKnots", &BSpline::initSplineSparseKnots, "Initialize the spline to interpolate a set of points with a given knot sequence. (Sparse Solution)")
    .def("basisMatrix", &BSpline::basisMatrix, "Get the basis matrix active on the ith time segment.", return_value_policy<copy_const_reference>())
    .def("timeInterval", &timeInterval1, "Returns a tuple with the time interval that the spline is well-defined on.")
    .def("timeInterval", &timeInterval2, "Returns a tuple with the time interval of the ith segment.")
    .def("addCurveSegment", &BSpline::addCurveSegment, "Adds a curve segment on the right that interpolates the given point at the given time.")
    .def("removeCurveSegment", &BSpline::removeCurveSegment, "removes a curve segment on the left")
    .def("setLocalCoefficientVector", &BSpline::setLocalCoefficientVector, "Sets the local coefficient vector for a specified time")
    .def("localVvCoefficientVectorIndices", &BSpline::localVvCoefficientVectorIndices, "")
    .def("localCoefficientVectorIndices", &BSpline::localCoefficientVectorIndices, "For the elements of a local coefficient vector, this gets the indices into the full coefficient vector")
    .def("segmentVvCoefficientVectorIndices", &BSpline::segmentVvCoefficientVectorIndices, "")
    .def("segmentCoefficientVectorIndices", &BSpline::segmentCoefficientVectorIndices, "For the elements of a segment coefficient vector, this gets the indices into the full coefficient vector")
    .def("setCoefficientVector", &BSpline::setCoefficientVector, "Sets the full stacked coefficient vector of the spline")
    .def("setCoefficientMatrix", &BSpline::setCoefficientMatrix, "Sets the full coefficient matrix of the spline")
    .def("addCurveSegment2", &BSpline::addCurveSegment2, "")
    .def("initSpline2", &BSpline::initSpline2, "")
    .def("evalI", &BSpline::evalI, "")
    .def("evalIntegral", &BSpline::evalIntegral, "")
    .def("Vi",&BSpline::Vi,"")
    .def("Mi", &BSpline::Mi, "")
    .def("Bij", &BSpline::Bij, "")
    .def("U", &BSpline::U, "U(time, derivativeOrder)")
    .def("u", &BSpline::u, "")
    .def("Di", &BSpline::Di, "")
    .def("Dii", &BSpline::Dii, "")
    .def("getLocalBi", &BSpline::getLocalBiVector, "getLocalBi(time)")
    .def("getLocalCumulativeBi", &BSpline::getLocalCumulativeBiVector, "getLocalCumulativeBi(time)")
    .def("getBiFunction", &getBiFunction, "getBiFunction(time)")
    .def("getCumulativeBiFunction", &getCumulativeBiFunction, "getBiFunction(time)")
    .def("segmentIndex", &BSpline::segmentIndex, "")
    .def("segmentQuadraticIntegral", &BSpline::segmentQuadraticIntegral, "")
    .def("segmentIntegral", &BSpline::segmentIntegral, "")
    .def("segmentQuadraticIntegralDiag", &BSpline::segmentQuadraticIntegralDiag, "")
    .def("curveQuadraticIntegral", &BSpline::curveQuadraticIntegral, "")
    .def("curveQuadraticIntegralDiag", &BSpline::curveQuadraticIntegralDiag, "")
    .def("curveQuadraticIntegralSparse", &BSpline::curveQuadraticIntegralSparse, "")
    .def("curveQuadraticIntegralDiagSparse", &BSpline::curveQuadraticIntegralDiagSparse, "")
    .def("coefficientVectorLength", &BSpline::coefficientVectorLength, "")
    .def("initConstantSpline", &BSpline::initConstantSpline, "initConstantSpline(double t_min, double t_max, int numSegments, const Eigen::VectorXd & constant")
    .def("numVvCoefficients", &BSpline::numVvCoefficients, "numVvCoefficients()");

  //.def("", &BSpline::, "")


}
