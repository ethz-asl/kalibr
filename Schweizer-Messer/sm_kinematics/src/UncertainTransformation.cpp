#include <sm/kinematics/UncertainTransformation.hpp>
#include <sm/kinematics/transformations.hpp>
#include <sm/serialization_macros.hpp>

namespace sm {
  namespace kinematics {

    UncertainTransformation::UncertainTransformation() :
      Transformation()
    {
      _U.setZero();
    }

    UncertainTransformation::UncertainTransformation(const Eigen::Matrix4d & T, const covariance_t & U) :
      Transformation(T), _U(U)
    {

    }


    UncertainTransformation::UncertainTransformation(const Eigen::Matrix4d & T, double diagonalTranslationVariance, double diagonalRotationVariance) :
    Transformation(T)
    {
      _U.setZero();
      _U(0,0) = diagonalTranslationVariance;
      _U(1,1) = diagonalTranslationVariance;
      _U(2,2) = diagonalTranslationVariance;
      _U(3,3) = diagonalRotationVariance;
      _U(4,4) = diagonalRotationVariance;
      _U(5,5) = diagonalRotationVariance;
    }


    UncertainTransformation::UncertainTransformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a, const covariance_t & U) :
      Transformation(q_a_b, t_a_b_a), _U(U)
    {

    }


    UncertainTransformation::UncertainTransformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a, double diagonalTranslationVariance, double diagonalRotationVariance) :
      Transformation(q_a_b, t_a_b_a)
    {

      _U.setZero();
      _U(0,0) = diagonalTranslationVariance;
      _U(1,1) = diagonalTranslationVariance;
      _U(2,2) = diagonalTranslationVariance;
      _U(3,3) = diagonalRotationVariance;
      _U(4,4) = diagonalRotationVariance;
      _U(5,5) = diagonalRotationVariance;

    }

    UncertainTransformation::UncertainTransformation(const Transformation & T, const covariance_t & U)
      : Transformation(T), _U(U)
    {

    }
    
    UncertainTransformation::UncertainTransformation(const Transformation & T, double diagonalTranslationVariance, double diagonalRotationVariance)
      : Transformation(T)
    {
      _U.setZero();
      _U(0,0) = diagonalTranslationVariance;
      _U(1,1) = diagonalTranslationVariance;
      _U(2,2) = diagonalTranslationVariance;
      _U(3,3) = diagonalRotationVariance;
      _U(4,4) = diagonalRotationVariance;
      _U(5,5) = diagonalRotationVariance;
    }


    /// \brief Initialize with zero uncertainty
    UncertainTransformation::UncertainTransformation(const Transformation & T) :
      Transformation(T)
    {
      _U.setZero();
    }

    /// \brief Initialize with zero uncertainty
    UncertainTransformation::UncertainTransformation(const Eigen::Matrix4d & T) :
      Transformation(T)
    {
      _U.setZero();
    }

    /// \brief Initialize with zero uncertainty
    UncertainTransformation::UncertainTransformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a) :
      Transformation(q_a_b, t_a_b_a)
    {
      _U.setZero();
    }


    UncertainTransformation::~UncertainTransformation()
    {

    }


    // transformation_t T_ac = T_ab_ * T_bc.T();
    // covariance_t T_ab_boxtimes = boxTimes(T_ab_.matrix());
    
    // covariance_t U_ac = U_ab_ + T_ab_boxtimes * T_bc.U() * T_ab_boxtimes.transpose();
    
    // return Transformation(T_ac, U_ac);
 
    const UncertainTransformation::covariance_t & UncertainTransformation::U() const
    {
      return _U;
    }

    UncertainTransformation::covariance_t UncertainTransformation::UOplus() const
    {
      Eigen::Matrix<double,6,6> S;
      S.setIdentity();
      S.topRightCorner<3,3>() = -sm::kinematics::crossMx(_t_a_b_a);
      return S.inverse().eval()*_U*S.transpose().inverse().eval();
    }
     
    UncertainTransformation UncertainTransformation::operator*(const UncertainTransformation & UT_b_c) const
    {
      const UncertainTransformation & UT_a_b = *this;

      const Transformation & T_b_c = UT_b_c;
      Transformation T_a_c = Transformation::operator*(T_b_c);
      covariance_t T_a_b_boxtimes = boxTimes(UT_a_b.T());
      
      covariance_t U_a_c = UT_a_b.U() + T_a_b_boxtimes * UT_b_c.U() * T_a_b_boxtimes.transpose();

      return UncertainTransformation(T_a_c, U_a_c);
    }

      
    UncertainTransformation UncertainTransformation::inverse() const
    {
      const Transformation & T_a_b = *this;
      // Invert the transformation.
      Transformation T_b_a = T_a_b.inverse();
      
      // Invert the uncertainty.
      covariance_t T_b_a_boxtimes = boxTimes(T_b_a.T());
      covariance_t U_b_a = T_b_a_boxtimes * _U * T_b_a_boxtimes.transpose();
      
      return UncertainTransformation(T_b_a,U_b_a);

    }


    // Set this to a random transformation.
     void UncertainTransformation::setRandom()
    {
      Transformation::setRandom();
      _U.setRandom();
      _U = _U * _U.transpose() * covariance_t::Identity();
    }


    void UncertainTransformation::setRandom( double translationMaxMeters, double rotationMaxRadians )
    {
      Transformation::setRandom(translationMaxMeters, rotationMaxRadians);
      _U.setRandom();
      _U = _U * _U.transpose() * covariance_t::Identity();
    }

    bool UncertainTransformation::isBinaryEqual(const UncertainTransformation & rhs) const
    {
      return Transformation::isBinaryEqual(rhs) && SM_CHECKMEMBERSSAME(rhs, _U);
    }

    /// \brief This sets the uncertainty directly.
    void UncertainTransformation::setU(const covariance_t & U)
    {
      _U = U;
    }

    /// \brief This sets the uncertainty based on the "oplus" function for updating the transformation matrix.
    /// This is a different uncertainty than that carried around by the class, so one should be careful. 
    void UncertainTransformation::setUOplus(const covariance_t & oplusU)
    {
      // For reference, see section 2.4 "Using Quaternions with Transformation Matrices"
      covariance_t S;
      S.setIdentity();
      S.topRightCorner<3,3>() = -crossMx(t());
      
      _U = S * oplusU * S.transpose();
    }


    UncertainHomogeneousPoint UncertainTransformation::operator*(const UncertainHomogeneousPoint & p_1) const
    {
      const Transformation & T_0_1 = *this;
      Eigen::Vector4d p_0 = T_0_1 * p_1.toHomogeneous();

      Eigen::Matrix4d T01 = T_0_1.T();
      Eigen::Matrix<double,4,6> Jt = boxMinus(p_0);
      
      UncertainHomogeneousPoint::covariance_t U = Jt * _U * Jt.transpose() + T01 * p_1.U4() * T01.transpose();
      
      return UncertainHomogeneousPoint(p_0,U);
    }

    UncertainTransformation UncertainTransformation::operator*(const Transformation & T_b_c) const
    {
      const Transformation & T_a_b = *this;

      Transformation T_a_c = T_a_b * T_b_c;
            
      return UncertainTransformation(T_a_c, U());

    }

    UncertainHomogeneousPoint UncertainTransformation::operator*(const HomogeneousPoint & p_1) const
    {
      const Transformation & T_0_1 = *this;
      Eigen::Vector4d p_0 = T_0_1 * p_1.toHomogeneous();

      Eigen::Matrix<double,4,6> Jt = boxMinus(p_0);
      
      UncertainHomogeneousPoint::covariance_t U = Jt * _U * Jt.transpose();
      
      return UncertainHomogeneousPoint(p_0,U);
    }

	Transformation UncertainTransformation::toTransformation() const
	{
	  return *this;
	}


  } // namespace kinematics
} // namespace sm
