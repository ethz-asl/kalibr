/*
 * NumericIntegratorTests.cpp
 *
 *  Created on: Oct 7, 2012
 *      Author: hannes
 */

#include "gtest/gtest.h"
#include "boost/typeof/typeof.hpp"
#include "bsplines/NumericIntegrator.hpp"
#include <sm/assert_macros.hpp>
#include <cmath>

using namespace numeric_integrator;

const double bounds[] = {-10, -2, -0.5, 0, 0.5, 2, 10};
const int numberOfBounds = sizeof(bounds) / sizeof(double);


double fabs(Eigen::Vector3d a){
	return a.norm();
}


template <typename TValue>
struct ZeroGetter{
	static TValue getZero(){
		return static_cast<TValue>(0);
	}
};


template <>
struct ZeroGetter<Eigen::Vector3d>{
	inline static Eigen::Vector3d getZero(){
		return Eigen::Vector3d::Zero();
	}
};

template <typename TValue, typename TAlgorithm, int INumberOfPoints, typename TFunctor>
void checkIntegral(TFunctor & f){
	for(int i = 0; i < numberOfBounds; i ++){
		for(int j = 0; j < numberOfBounds; j ++){
			double lBound = bounds[i], uBound = bounds[j];
			TValue val = integrateFunctor<TAlgorithm>(lBound, uBound, f, INumberOfPoints, ZeroGetter<TValue>::getZero());
			TValue analyticalVal = f.calcIntegral(lBound, uBound);
			SM_ASSERT_NEAR(std::runtime_error, val, analyticalVal, fmax(fabs(analyticalVal) * 1e-3, 1e-6), f.getName());
		}
	}
}

template <typename TFunctor>
void checkIntegral(TFunctor & f){
	checkIntegral<typename TFunctor::ValueT, algorithms::TrapezoidalRule, 200>(f);
	checkIntegral<typename TFunctor::ValueT, algorithms::SimpsonRule, 50>(f);
//	checkIntegral<DOUBLE_EXPONENTIAL3>(f);
//	checkIntegral<DOUBLE_EXPONENTIAL5>(f);
}


namespace test_functions {
	struct XSquared {
		typedef double ValueT;
		inline double operator () (double x) const {
			return x * x;
		}

		inline double calcIntegral(double a, double b){
			return (b * b * b - a * a * a) / 3.;
		}
		inline const char * getName() { return "XSquared"; }
	} xSquared;

	struct XCubix{
		typedef double ValueT;
		inline double operator () (double x) const {
			return x * x * x;
		}

		inline double calcIntegral(double a, double b){
			return (b * b * b * b - a * a * a * a) / 4.;
		}
		inline const char * getName() { return "XCubix"; }
	} xCubic;

	struct SinX{
		typedef double ValueT;
		inline double operator () (double x)const {
			return sin(x);
		}

		inline double calcIntegral(double a, double b){
			return cos(a) - cos(b);
		}
		inline const char * getName() { return "Sin"; }
	} sinX;

	struct ExpX{
		typedef double ValueT;
		inline double operator () (double x) const {
			return exp(x);
		}

		inline double calcIntegral(double a, double b){
			return exp(b) - exp(a);
		}
		inline const char * getName() { return "Exp"; }
	} expX;

	struct SinhX{
		typedef double ValueT;
		inline double operator () (double x) const {
			return sinh(x);
		}
		inline double calcIntegral(double a, double b){
			return cosh(b) - cosh(a);
		}
		inline const char * getName() { return "Sinh"; }
	} sinhX;

	struct XSquaredInR3{
		typedef Eigen::Vector3d ValueT;
		inline ValueT operator () (double x) const {
			double v = x * x;
			return ValueT(v, 2 * v, 3 * v);
		}

		inline ValueT calcIntegral(double a, double b){
			double I = (b * b * b - a * a * a) / 3.;
			return ValueT(I, 2 * I, 3 * I);
		}
		inline const char * getName() { return "XSquaredInR3"; }
	} xSquaredInR3;
}

using namespace test_functions;

TEST(NumericIntegratorTestSuite, compareWithAnalyticalIntegration)
{
	checkIntegral(xSquared);
	checkIntegral(xCubic);
	checkIntegral(sinX);
	checkIntegral(expX);
	checkIntegral(sinhX);
}

TEST(NumericIntegratorTestSuite, eigenVectorValueIntegration)
{
	checkIntegral(xSquaredInR3);
}

TEST(NumericIntegratorTestSuite, testFunctionInterface)
{
	struct Integrand {
		typedef double ValueT;
		inline static ValueT integrand(const double & x){
			return x * x;
		}
	};

	integrateFunction(0., 1., Integrand::integrand, 100, 0.);
}
