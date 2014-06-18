#ifndef SM_NUMERICAL_DIFF_HPP
#define SM_NUMERICAL_DIFF_HPP

#include <Eigen/Core>
#include <sm/assert_macros.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

namespace sm { namespace eigen {

    template<typename RESULT_VEC_T, typename INPUT_VEC_T, typename JACOBIAN_T = Eigen::MatrixXd>
    struct NumericalDiffFunctor
    {
      typedef RESULT_VEC_T value_t;
      typedef typename value_t::Scalar scalar_t;
      typedef INPUT_VEC_T input_t;
      typedef JACOBIAN_T jacobian_t;
      

      NumericalDiffFunctor( boost::function< value_t(input_t) > f) : _f(f){}
      
      value_t operator()(const input_t & x) { return _f(x); }

      input_t update(const input_t & x, int c, scalar_t delta) { input_t xnew = x; xnew[c] += delta; return xnew; }
      boost::function<value_t(input_t)> _f;
    };

    // A simple implementation of central differences to estimate a Jacobian matrix
    template<typename FUNCTOR_T>
    struct NumericalDiff
    {
      typedef FUNCTOR_T functor_t;
      typedef typename functor_t::input_t input_t;
      typedef typename functor_t::value_t value_t;
      typedef typename functor_t::scalar_t scalar_t;
      typedef typename functor_t::jacobian_t jacobian_t;

      NumericalDiff(functor_t f, scalar_t eps = sqrt(std::numeric_limits<scalar_t>::epsilon())) : functor(f), eps(eps) {}
      
      jacobian_t estimateJacobian(input_t const & x0)
      {
        // evaluate the function at the operating point:
        value_t fx0 = functor(x0);
        size_t N = x0.size();
        size_t M = fx0.size();

        //std::cout << "Size: " << M << ", " << N << std::endl;
        jacobian_t J;
        J.resize(M, N);

        SM_ASSERT_EQ(std::runtime_error,x0.size(),J.cols(),"Unexpected number of columns for input size");
        SM_ASSERT_EQ(std::runtime_error,fx0.size(),J.rows(),"Unexpected number of columns for output size");

        for(unsigned c = 0; c < N; c++) {
          // Calculate a central difference.
          // This step size was stolen from cminpack: temp = eps * fabs(x[j]);
          scalar_t rcEps = std::max(static_cast<scalar_t>(fabs(x0(c))) * eps,eps);

          value_t fxp = functor(functor.update(x0,c,rcEps));
          value_t fxm = functor(functor.update(x0,c,-rcEps));
          J.block(0, c, M, 1) = (fxp - fxm).template cast<typename jacobian_t::Scalar>()/(typename jacobian_t::Scalar)(rcEps*(scalar_t)2.0);
        }
        return J;
      }

      functor_t functor;
      scalar_t eps;
    };


    template < typename ValueType_, typename InputType_>
    Eigen::MatrixXd numericalDiff(std::function<ValueType_ (const InputType_ &) > function, InputType_ const & input, double eps = sqrt(std::numeric_limits<typename NumericalDiffFunctor<ValueType_, InputType_>::scalar_t>::epsilon())){
      typedef NumericalDiffFunctor<ValueType_, InputType_> Functor;

      NumericalDiff<Functor> numDiff(Functor(function), eps);
      return numDiff.estimateJacobian(input);
    }

  }}


#endif /* SM_NUMERICAL_DIFF_HPP */



