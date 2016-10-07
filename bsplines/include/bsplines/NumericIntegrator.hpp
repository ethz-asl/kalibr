/*
 * NumericIntegrator.hpp
 *
 *  Created on: Oct 7, 2012
 *      Author: hannes
 */

#ifndef NUMERICINTEGRATOR_HPP_
#define NUMERICINTEGRATOR_HPP_

#include <cmath>

namespace numeric_integrator {
	template <typename ValueFactor, typename IntegrationScalar>
	class Integrator {
	 public:
		virtual ~Integrator() {}
		virtual bool isAtEnd() const = 0;
		virtual void next() = 0;
		virtual int getNIntegrationPoints() const = 0;
		virtual IntegrationScalar getIntegrationScalar() const = 0;
		virtual ValueFactor getValueFactor() const = 0;
		virtual ValueFactor getCommonFactor() const = 0;
	};

	template <typename ValueFactor, typename IntegrationScalar>
	class AbstractIntegrator : public Integrator<ValueFactor, IntegrationScalar> {
	 public:
		AbstractIntegrator(IntegrationScalar a, IntegrationScalar b, int nIntegrationPoints) : a(a), b(b), maxIndex(nIntegrationPoints - 1) {}
		virtual ~AbstractIntegrator() {}
		int getNIntegrationPoints() const { return maxIndex + 1; }
	 protected:
		IntegrationScalar a, b;
		int maxIndex;
	};

	template <typename DERIVED, template<typename, typename> class Integrator>
	class IntegrationAlgorithm {
	 private:
		const DERIVED & derived() const {
			return static_cast<const DERIVED&>(*this);
		}
	 public:
		template <typename ValueFactor, typename IntegrationScalar>
		Integrator<ValueFactor, IntegrationScalar> getIntegrator(IntegrationScalar a, IntegrationScalar b, int nIntegrationPoints) const
		{
			return Integrator<ValueFactor, IntegrationScalar>(a, b, nIntegrationPoints);
		}
	};

	namespace internal {
		template <typename ValueFactor, typename IntegrationScalar>
		class FixStepSizeIntegrator : public AbstractIntegrator<ValueFactor, IntegrationScalar> {
		 public:
			FixStepSizeIntegrator(IntegrationScalar a, IntegrationScalar b, int nIntegrationPoints): AbstractIntegrator<ValueFactor, IntegrationScalar>(a, b, nIntegrationPoints), stepSize((b-a) / this->maxIndex), iIntegrationPoint(0) {}
			inline bool isAtEnd() const { return iIntegrationPoint > this->maxIndex; }
			inline void next() { ++iIntegrationPoint;}
			inline IntegrationScalar getIntegrationScalar() const { return (iIntegrationPoint == this->maxIndex) ? this->b : this->a + stepSize * iIntegrationPoint; }
			inline bool atBounds() const { return this->iIntegrationPoint == 0 || this->iIntegrationPoint == this->maxIndex; }
		 protected:
			IntegrationScalar stepSize;
			int iIntegrationPoint;
		};

		template <typename ValueFactor, typename IntegrationScalar>
		class SimpsonRuleIntegrator : public FixStepSizeIntegrator<ValueFactor, IntegrationScalar> {
		 public:
			SimpsonRuleIntegrator(IntegrationScalar a, IntegrationScalar b, int nIntegrationPoints): FixStepSizeIntegrator<ValueFactor, IntegrationScalar>(a, b, (nIntegrationPoints + 1 - nIntegrationPoints % 2)) {}
			inline ValueFactor getValueFactor() const { return this->atBounds() ? 0.5 : (this->iIntegrationPoint % 2 == 1 ? 2 : 1); }
			inline ValueFactor getCommonFactor() const { return (this->stepSize * 2. / 3.); }
		};


		template <typename ValueFactor, typename IntegrationScalar>
		class TrapezoidalRuleIntegrator : public FixStepSizeIntegrator<ValueFactor, IntegrationScalar> {
		 public:
			TrapezoidalRuleIntegrator(IntegrationScalar a, IntegrationScalar b, int nIntegrationPoints): FixStepSizeIntegrator<ValueFactor, IntegrationScalar>(a, b, nIntegrationPoints) {}
			inline ValueFactor getValueFactor() const { return this->atBounds() ? 0.5 : 1.0; }
			inline ValueFactor getCommonFactor() const { return this->stepSize; }
		};
	}

	namespace algorithms {
		class SimpsonRule : public IntegrationAlgorithm<SimpsonRule, internal::SimpsonRuleIntegrator> {
		};
		class TrapezoidalRule : public IntegrationAlgorithm<TrapezoidalRule, internal::TrapezoidalRuleIntegrator> {
		};

		typedef SimpsonRule Default;
	}
	typedef algorithms::Default DefaultAlgorithm;

	template <typename TValue, typename TArgScalar>
	struct Integrand {
		typedef TValue ValueT;
		typedef TArgScalar ArgScalarT;

		typedef ValueT (&IntegrandFunctionT)(TArgScalar & t) ;


		Integrand(IntegrandFunctionT integrand) : _integrand(integrand){}

		inline TValue operator () (const TArgScalar arg) const {
			return _integrand(arg);
		}

	private:
		IntegrandFunctionT _integrand;
	};

	template <typename TValue, typename TArgScalar>
	inline Integrand<TValue, TArgScalar> createIntegrand(TValue (&integrand)(TArgScalar & t)){
		return Integrand<TValue, TArgScalar>(integrand);
	}

//	enum Algorithm {
//		TRAPEZOIDAL,
//		SIMPSON,
//		TANH1,
//		TANH3,
//		TANH5,
//		TANH_SINH,
//		DEFAULT = SIMPSON
//	};

	namespace internal {
		template <typename TValue, int IExponent, typename TArgScalar, typename TFunctor>
		inline TValue tanhFunctor(const TFunctor & f, TArgScalar pos, TArgScalar mean, TArgScalar diffHalf){
			const double posPowExp = IExponent == 1 ? pos : pow(pos, IExponent);
			double ch = cosh(posPowExp);
			if(IExponent == 1)
				return f((TArgScalar)(diffHalf * tanh(posPowExp) + mean)) / ch * ch;
			else
				return f((TArgScalar)(diffHalf * tanh(posPowExp) + mean)) * IExponent * posPowExp / (pos * ch * ch);
		}
	}

	//TODO allow integration according to arbitrary TimePolicy
	template <typename Algorithm, typename TValue, typename TArgScalar, typename TFunctor>
	inline TValue integrateFunctor(TArgScalar a, TArgScalar b, const TFunctor & f, int numberOfPoints, TValue zero = TValue(0)){
		if(a == b) return zero;

		auto integrator = Algorithm().template getIntegrator<double>(a, b, numberOfPoints);

		SM_ASSERT_TRUE_DBG(std::runtime_error, numberOfPoints>2 && !integrator.isAtEnd(), "too few integration points given : " << numberOfPoints);

		TValue sum = f(integrator.getIntegrationScalar()) * integrator.getValueFactor();
		integrator.next();

		for(; !integrator.isAtEnd(); integrator.next()){
			double valueFactor = integrator.getValueFactor();
			if(valueFactor == 1)
				sum += f(integrator.getIntegrationScalar());
			else
				sum += f(integrator.getIntegrationScalar()) * valueFactor;
		}
		return sum * integrator.getCommonFactor();

// TODO implement TANH* stuff
//		TArgScalar diff = b-a, stepSize = diff / (numberOfPoints - 1);
//		if(diff == 0) return zero;
//
//		switch(EAlgorithm){
//		case TANH1:
//		case TANH3:
//		case TANH5:
//			throw std::runtime_error("TANHX integration scheme not supported yet");
//			{
//				const double diffHalf = diff /2.;
//				const double mean = 0.5 * (a+b);
//				const int exponent = 2 * (EAlgorithm - TANH1) + 1;
//				TValue sum = internal::tanhFunctor<TValue, exponent>(f, (TArgScalar)0, mean, diffHalf);
//
//				for(int i = numberOfPoints / 2, end = numberOfPoints / 2; i <= end; i++){
//					sum += internal::tanhFunctor<TValue, exponent>(f, (TArgScalar) i, mean, diffHalf);
//				}
//				return sum / diffHalf;
//			}
//		case TANH_SINH:
//			throw std::runtime_error("TANH_SINH integration scheme not supported yet");
//			break;
//		}
	}
	template <typename TValue, typename TArgScalar, typename TFunctor>
	inline TValue integrateFunctor(TArgScalar a, TArgScalar b, const TFunctor & f, int numberOfPoints, TValue zero = TValue(0)){
		return integrateFunctor<algorithms::Default> (a, b, f, numberOfPoints, zero);
	}

	template <typename Algorithm, typename TValue, typename TArgScalar>
	inline TValue integrateFunction(TArgScalar a, TArgScalar b, TValue (& integrand)(const TArgScalar & t), int numberOfPoints, TValue zero = TValue(0)){
		return integrateFunctor<Algorithm>(a, b, createIntegrand(integrand), numberOfPoints, zero);
	}

	template <typename TValue, typename TArgScalar>
	inline TValue integrateFunction(TArgScalar a, TArgScalar b, TValue (& integrand)(const TArgScalar & t), int numberOfPoints, TValue zero = TValue(0)){
		return integrateFunction<algorithms::Default>(a, b, integrand, numberOfPoints, zero);
	}
}

#endif /* NUMERICINTEGRATOR_HPP_ */
