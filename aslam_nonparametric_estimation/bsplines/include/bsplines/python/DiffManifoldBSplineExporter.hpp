/*
 * DiffManifoldBSplineImporter.hpp
 *
 *  Created on: Dec 4, 2013
 *      Author: hannes
 */

#ifndef DIFFMANIFOLDBSPLINEEXPORTER_HPP_
#define DIFFMANIFOLDBSPLINEEXPORTER_HPP_
#include <functional>
#include <bsplines/DiffManifoldBSpline.hpp>
#include <bsplines/EuclideanBSpline.hpp>
#include <bsplines/UnitQuaternionBSpline.hpp>
#include <bsplines/BSplineFitter.hpp>

#include <numpy_eigen/boost_python_headers.hpp>

namespace bspline_exporter {

using namespace bsplines;
using namespace boost::python;

template <typename TSpline, typename Evaluator_ = typename TSpline::template Evaluator<Eigen::Dynamic>>
class BSplineExporter {
 public:
	typedef Evaluator_ Evaluator;
	typedef BSplineFitter<TSpline> Fitter;

	static typename TSpline::point_t eval(const TSpline * bsp, typename TSpline::time_t t){
		SM_ASSERT_GE(typename TSpline::Exception, t, bsp->getMinTime(), "The time is out of range.");
		SM_ASSERT_LE(typename TSpline::Exception, t, bsp->getMaxTime(), "The time is out of range.");

		return bsp->template getEvaluatorAt<0>(t).eval();
	}

	static typename TSpline::point_t evalD(const TSpline * bsp, typename TSpline::time_t t, int derivativeOrder){
		SM_ASSERT_GE(typename TSpline::Exception, t, bsp->getMinTime(), "The time is out of range.");
		SM_ASSERT_LE(typename TSpline::Exception, t, bsp->getMaxTime(), "The time is out of range.");

		switch(derivativeOrder){
			case 0:
				return bsp->template getEvaluatorAt<0>(t).evalD(derivativeOrder);
			case 1:
				return bsp->template getEvaluatorAt<1>(t).evalD(derivativeOrder);
			case 2:
				return bsp->template getEvaluatorAt<2>(t).evalD(derivativeOrder);
			default:
				return bsp->template getEvaluatorAt<Eigen::Dynamic>(t).evalD(derivativeOrder);
		}
	}

	static Eigen::MatrixXd evalJacobianD(const TSpline * bsp, typename TSpline::time_t t, int derivativeOrder){
		SM_ASSERT_GE(typename TSpline::Exception, t, bsp->getMinTime(), "The time is out of range.");
		SM_ASSERT_LE(typename TSpline::Exception, t, bsp->getMaxTime(), "The time is out of range.");

		typename TSpline::full_jacobian_t ret;
		switch(derivativeOrder){
			case 0:
				bsp->template getEvaluatorAt<0>(t).evalJacobian(derivativeOrder, ret);
				break;
			case 1:
				bsp->template getEvaluatorAt<1>(t).evalJacobian(derivativeOrder, ret);
				break;
			case 2:
				bsp->template getEvaluatorAt<2>(t).evalJacobian(derivativeOrder, ret);
				break;
			default:
				bsp->template getEvaluatorAt<Eigen::Dynamic>(t).evalJacobian(derivativeOrder, ret);
		}
		return ret;
	}

	static typename Eigen::MatrixXd evalJacobian(const TSpline * bsp, typename TSpline::time_t t){
		return evalJacobianD(bsp, t, 0);
	}

	static typename Eigen::MatrixXd evalEvaluatorJacobian(const typename TSpline::template Evaluator<Eigen::Dynamic> * eval, int derivativeOrder){
		typename TSpline::full_jacobian_t ret;
		eval->evalJacobian(derivativeOrder, ret);
		return ret;
	}

	static double getLastRelevantKnot(const TSpline * bsp, double t){
		return bsp->getSegmentIterator(t).getTime();
	}

	static double getFirstRelevantKnot(const TSpline * bsp, double t){
		return bsp->getFirstRelevantSegmentByLast(bsp->getSegmentIterator(t)).getTime();
	}

	// Function wrappers turn std::pairs into tuples.
	static boost::python::tuple timeInterval(const TSpline * bsp)
	{
		std::pair<double,double> ti = bsp->getTimeInterval();
		return boost::python::make_tuple(ti.first,ti.second);
	}

	static void initUniformSplineFromMatrix(TSpline * bsp, const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, int numSegments, double lambda)
	{
		const int numPoints = interpolationPoints.cols();
		const int numTimes= times.size();

		SM_ASSERT_GE(typename TSpline::Exception, numTimes, numPoints, "The there must be as much times as points.");
		std::vector<typename TSpline::time_t> timesVector(numTimes);
		for(int i = 0, end = numTimes; i != end; i++){
			timesVector[i] = times[i];
		}
		std::vector<typename TSpline::point_t> interpolationPointsVector(numPoints);
		for(int i = 0; i != numPoints; i++){
			interpolationPointsVector[i] = interpolationPoints.col(i);
		}

		Fitter::initUniformSpline(*bsp, timesVector, interpolationPointsVector, numSegments, lambda);
	}

	static DeltaUniformKnotGenerator<typename TSpline::TimePolicy> initUniformSplineFromMatrixWithKnotDelta(TSpline * bsp, const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, typename TSpline::duration_t delta, double lambda)
	{
		const int numPoints = interpolationPoints.cols();
		const int numTimes= times.size();

		SM_ASSERT_GE(typename TSpline::Exception, numTimes, numPoints, "The there must be as much times as points.");
		std::vector<typename TSpline::time_t> timesVector(numTimes);
		for(int i = 0, end = numTimes; i != end; i++){
			timesVector[i] = times[i];
		}
		std::vector<typename TSpline::point_t> interpolationPointsVector(numPoints);
		for(int i = 0; i != numPoints; i++){
			interpolationPointsVector[i] = interpolationPoints.col(i);
		}

		return Fitter::initUniformSplineWithKnotDelta(*bsp, timesVector, interpolationPointsVector, delta, lambda);
	}


	static void fitSplineFromMatrix(TSpline * bsp, const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, double lambda, int fixNFirstRelevantPoints, const Eigen::VectorXd & weights)
	{
    const int dim = (int)bsp->getDimension();
		const int numPoints = interpolationPoints.cols();
		const int numTimes= times.size();
		const int numWeights = weights.size();
    
    SM_ASSERT_EQ(typename TSpline::Exception, interpolationPoints.rows(), dim, "The layout of the interpolation points is (dim x nPoints).");
		SM_ASSERT_GE(typename TSpline::Exception, numTimes, numPoints, "There must be as much times as points. The layout of the interpolation points is (dim x nPoints).");

		if(numWeights){
			SM_ASSERT_GE(typename TSpline::Exception, numWeights, numTimes, "There must be no or as much weights as points!");
		}
		std::vector<typename TSpline::time_t> timesVector(numTimes);
		for(int i = 0, end = numTimes; i != end; i++){
			timesVector[i] = times[i];
		}
		std::vector<typename TSpline::point_t> pointsVector(numPoints);
		for(int i = 0; i != numPoints; i++){
			pointsVector[i] = interpolationPoints.col(i);
		}

		Fitter::fitSpline(*bsp, timesVector, pointsVector, lambda, fixNFirstRelevantPoints, numWeights? [&weights](int i){ return weights[i]; } : std::function<double(int)>());
	}

	static void extendAndFitSplineFromMatrix(TSpline * bsp, bsplines::DeltaUniformKnotGenerator<typename TSpline::TimePolicy> & knotGenerator, const Eigen::VectorXd & times, const Eigen::MatrixXd & interpolationPoints, double lambda, unsigned char honorCurrentValueCoefficient)
	{
    const int dim = (int)bsp->getDimension();
		const int numPoints = interpolationPoints.cols();
		const int numTimes= times.size();

    SM_ASSERT_EQ(typename TSpline::Exception, interpolationPoints.rows(), dim, "The layout of the interpolation points is (dim x nPoints).");
		SM_ASSERT_GE(typename TSpline::Exception, numTimes, numPoints, "There must be as much times as points. The layout of the interpolation points is (dim x nPoints).");

		std::vector<typename TSpline::time_t> timesVector(numTimes);
		for(int i = 0, end = numTimes; i != end; i++){
			timesVector[i] = times[i];
		}
		std::vector<typename TSpline::point_t> pointsVector(numPoints);
		for(int i = 0; i != numPoints; i++){
			pointsVector[i] = interpolationPoints.col(i);
		}

		Fitter::extendAndFitSpline(*bsp, knotGenerator, timesVector, pointsVector, lambda, honorCurrentValueCoefficient);
	}


	static Eigen::VectorXd getKnotsVector(const TSpline * bsp)
	{
		Eigen::VectorXd times(bsp->getNumKnots());

		int c = 0;
		for(typename TSpline::SegmentConstIterator it = bsp->getAbsoluteBegin(), end = bsp->getAbsoluteEnd(); it != end; it++){
			times[c++] = it.getKnot();
		}
		return times;
	}

	static Eigen::MatrixXd getControlVertices(const TSpline * bsp)
	{
		Eigen::MatrixXd vertices(bsp->getNumControlVertices(), (int)bsp->getDimension());

		typename TSpline::SegmentConstIterator it = bsp->getAbsoluteBegin();
		for(int c = 0, end = vertices.rows(); c != end; c++, it++){
			vertices.row(c) = it->getControlVertex();
		}
		return vertices;
	}

	inline static BSplineExporter exportEuclideanSpline(const std::string name, bool exportEvaluator = true){
		BSplineExporter exporter = BSplineExporter(name.c_str(), init<int, int>((name + "(splineOrder, dimension)").c_str()));
		exporter.exportClass()
			.def("evalI", &TSpline::evalIntegral, "")
			.def("evalIntegral", &TSpline::evalIntegral, "");
		if(exportEvaluator) exporter.exportEvaluator();
		return exporter;
	}

	inline static BSplineExporter exportUnitQuaternionSpline(const std::string name, bool exportEvaluator = true){
		BSplineExporter exporter = BSplineExporter(name.c_str(), init<int>((name + "(splineOrder)").c_str()));
		exporter.exportClass();
		if(exportEvaluator){
			exporter.exportEvaluator()
			.def("evalAngularVelocity", &Evaluator::evalAngularVelocity)
			.def("evalAngularAcceleration", &Evaluator::evalAngularAcceleration)
			;
		}
		return exporter;
	}

	class_<Evaluator>& getEvaluatorClass()
	{
		return evaluatorClass;
	}

	class_<TSpline>& getSplineClass()
	{
		return splineClass;
	}
 private:
	template <class DerivedT>
	BSplineExporter(const char * name, init_base<DerivedT> const& i) :
		splineClass(name, i),
		evaluatorClass("Evaluator", no_init)
	{
	}

	class_<Evaluator> & exportEvaluator(){
		evaluatorClass
		.def("eval", &Evaluator::eval, "Evaluate the spline curve at the evaluators time")
		.def("evalD", &Evaluator::evalD, "Evaluate a spline curve derivative at the evaluators time")
		.def("evalJacobian", &evalEvaluatorJacobian, "Matrix evalJacobian(int derivativeOrder)")
		.def("getKnot", &Evaluator::getKnot, "double getKnot()")
		.def("getTime", &Evaluator::getTime, "double getTime()")
		;
		return evaluatorClass;
	}

	class_<TSpline> & exportClass(){
		splineClass
		.def("init", &TSpline::init)
		.def("splineOrder", &TSpline::getSplineOrder, "int splineOrder() - deprecated : use getSplineOrder() instead")
		.def("getSplineOrder", &TSpline::getSplineOrder, "int getSplineOrder()")
		.def("getPointSize", &TSpline::getPointSize, "int getPointSize()")
		//		.def("polynomialDegree", &TSpline::polynomialDegree, "The degree of the polynomial spline")
		.def("minimumKnotsRequired", &TSpline::getMinimumKnotsRequired, "The minimum number of knots required")
		.def("numKnotsRequired", &TSpline::getNumKnotsRequired, "The number of knots required for a specified number of valid time segments")
		.def("getKnotsRequired", &TSpline::getNumKnotsRequired, "The number of knots required for a specified number of valid time segments")
		.def("numCoefficientsRequired", &TSpline::getNumControlVerticesRequired, "The number of control vertices required for a specified number of valid time segments")
		.def("getNumControlVerticesRequired", &TSpline::getNumControlVerticesRequired, "The number of control vertices required for a specified number of valid time segments")
		.def("numKnots", &TSpline::getNumKnots, "The number of knots in the spline")
		.def("getNumKnots", &TSpline::getNumKnots, "The number of knots in the spline")
		//		.def("numValidTimeSegments", numValidTimeSegments1, "The number of valid time segments for a given number of knots")
		//		.def("numValidTimeSegments", numValidTimeSegments2, "The number of valid time segments for the current knot sequence")
		.def("addKnots", &TSpline::addKnots, "Adds the knots to the spline")
		.def("addKnotsAndControlVertices", &TSpline::addKnotsAndControlVertices, "Adds the knots and control vertices to the spline")
		.def("initWithKnots", &TSpline::initWithKnots, "Sets the spline's knots initializes it")
		.def("initWithKnotsAndControlVertices", &TSpline::initWithKnotsAndControlVertices, "Sets the spline's knots and control vertices and initializes it")
		.def("setControlVertices", static_cast<void (TSpline::*)(const Eigen::MatrixXd &)>(&TSpline::setControlVertices), "Sets the spline's control vertices")
		.def("getKnotsVector", getKnotsVector, "returns the current knot sequence")
		.def("knots", getKnotsVector, "returns the current knot sequence")
		.def("coefficients", getControlVertices, "get the current control vertices as rows of a matrix")
		.def("getControlVertices", getControlVertices, "get the current control vertices as rows of a matrix")
		.def("getMinTime", &TSpline::getMinTime, "The minimum time that the spline is well-defined on")
		.def("getMaxTime", &TSpline::getMaxTime, "The maximum time that the spline is well-defined on")
		.def("eval", eval, "Evaluate the spline curve at a point in time")
		.def("evalD", evalD, "Evaluate a spline curve derivative at a point in time")
		.def("evalJacobian", evalJacobian, "Matrix evalJacobian(double t) :  Evaluate the spline curve evaluation's jacobian with respect to changes in the control vertices at time t.")
		.def("evalJacobianD", evalJacobianD, "Matrix evalJacobianD(double t, derivativeOrder) :  Evaluate the spline curve derivative's (or evaluation's, for derivativeOrder = 0) jacobian with respect to changes in the control vertices at time t.")
		.def("getEvaluatorAt", &TSpline::template getEvaluatorAt<Eigen::Dynamic> , "Get a evaluator at a point in time")
		//		.def("Phi", &TSpline::Phi, "Evaluate the local basis matrix at a point in time")
		//		.def("localBasisMatrix", &TSpline::localBasisMatrix, "Evaluate the local basis matrix at a point in time")
		//		.def("localCoefficientMatrix", &TSpline::localCoefficientMatrix, "Get the matrix of locally-active coefficients for a specified time in matrix form")
		//		.def("localCoefficientVector", &TSpline::localCoefficientVector, "Get the stacked vector of locally-active coefficients for a specified time.")
		.def("initUniformSpline", &initUniformSplineFromMatrix, "Initialize the spline to smooth a set of points in time")
		.def("initUniformSplineWithKnotDelta", &initUniformSplineFromMatrixWithKnotDelta, "Initialize the spline to smooth a set of points in time")
		.def("fitSpline", &fitSplineFromMatrix, "fitSplineFromMatrix(np.array times, np.array interpolationPoints, double lambda, int fixNFirstRelevantPoints, np.array weights): Fit the initialized spline to smooth a set of points in time")
		//		.def("basisMatrix", &TSpline::basisMatrix, "Get the basis matrix active on the ith time segment.", return_value_policy<copy_const_reference>())
		.def("timeInterval", &timeInterval, "Returns a tuple with the time interval that the spline is well-defined on.")
		//		.def("timeInterval", &timeInterval2, "Returns a tuple with the time interval of the ith segment.")
		.def("getTimeInterval", &timeInterval, "Returns a tuple with the time interval that the spline is well-defined on.")
		//		.def("getTimeInterval", &timeInterval2, "Returns a tuple with the time interval of the ith segment.")
		.def("appendSegmentsUniformly", &TSpline::appendSegmentsUniformly, "Adds segments assuming uniform knot spacing.")
		.def("extendAndFitSpline", &extendAndFitSplineFromMatrix, "Extend and fit the initialized spline to smooth a set of points in time.")
		.def("getFirstRelevantKnot", &getFirstRelevantKnot, "double getFirstRelevantKnot(double forT)")
		.def("getLastRelevantKnot", &getLastRelevantKnot, "double getLastRelevantKnot(double forT)")
		//		.def("removeCurveSegment", &TSpline::removeCurveSegment, "removes a curve segment on the left")
		//		.def("setLocalCoefficientVector", &TSpline::setLocalCoefficientVector, "Sets the local coefficient vector for a specified time")
		//		.def("localVvCoefficientVectorIndices", &TSpline::localVvCoefficientVectorIndices, "")
		//		.def("localCoefficientVectorIndices", &TSpline::localCoefficientVectorIndices, "For the elements of a local coefficient vector, this gets the indices into the full coefficient vector")
		//		.def("setCoefficientVector", &TSpline::setCoefficientVector, "Sets the full stacked coefficient vector of the spline")
		//		.def("setCoefficientMatrix", &TSpline::setCoefficientMatrix, "Sets the full coefficient matrix of the spline")
		//		.def("addCurveSegment2", &TSpline::addCurveSegment2, "")
		//		.def("initSpline2", &TSpline::initSpline2, "")
		//		.def("Vi",&TSpline::Vi,"")
		//		.def("Mi", &TSpline::Mi, "")
		//		.def("Bij", &TSpline::Bij, "")
		//		.def("U", &TSpline::U, "U(time, derivativeOrder)")
		//		.def("u", &TSpline::u, "")
		//		.def("Di", &TSpline::Di, "")
		//		.def("Dii", &TSpline::Dii, "")
		//		.def("getLocalBi", &TSpline::getLocalBiVector, "getLocalBi(time)")
		//		.def("getLocalCumulativeBi", &TSpline::getLocalCumulativeBiVector, "getLocalCumulativeBi(time)")
		//		.def("getBiFunction", &getBiFunction, "getBiFunction(time)")
		//		.def("getCumulativeBiFunction", &getCumulativeBiFunction, "getBiFunction(time)")
		//		.def("segmentIndex", &TSpline::segmentIndex, "")
		//		.def("segmentQuadraticIntegral", &TSpline::segmentQuadraticIntegral, "")
		//		.def("segmentQuadraticIntegralDiag", &TSpline::segmentQuadraticIntegralDiag, "")
		//		.def("curveQuadraticIntegral", &TSpline::curveQuadraticIntegral, "")
		//		.def("curveQuadraticIntegralDiag", &TSpline::curveQuadraticIntegralDiag, "")
		//		.def("curveQuadraticIntegralSparse", &TSpline::curveQuadraticIntegralSparse, "")
		//		.def("curveQuadraticIntegralDiagSparse", &TSpline::curveQuadraticIntegralDiagSparse, "")
		//		.def("coefficientVectorLength", &TSpline::coefficientVectorLength, "")
		.def("initConstantUniformSpline", &TSpline::initConstantUniformSpline, "void initConstantUniformSpline(double t_min, double t_max, int numSegments, const Eigen::VectorXd & constant")
		.def("initConstantSpline",        &TSpline::initConstantUniformSpline, "void initConstantSpline(double t_min, double t_max, int numSegments, const Eigen::VectorXd & constant")
		.def("initConstantUniformSplineWithKnotDelta", &TSpline::initConstantUniformSplineWithKnotDelta, "DeltaUniformKnotGenerator initConstantUniformSplineWithKnotDelta(double t_min, double t_max, double knotDelta, const Eigen::VectorXd & constant")
		.def("getNumControlVertices", &TSpline::getNumControlVertices, "")
		.def("numVvCoefficients", &TSpline::getNumControlVertices, "")
		;

		return splineClass;
	}

	class_<TSpline> splineClass;
	class_<Evaluator> evaluatorClass;
};

struct DynamicOrTemplateInt_to_python_int
{
	static PyObject* convert(eigenTools::DynamicOrTemplateInt<Eigen::Dynamic> const& s)
	{
		return boost::python::incref(
				boost::python::object(s.getValue()).ptr());
	}
};

}

#endif /* DIFFMANIFOLDBSPLINEEXPORTER_HPP_ */
