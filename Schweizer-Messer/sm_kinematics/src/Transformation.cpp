#include <sm/kinematics/Transformation.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/random.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <sm/kinematics/UncertainTransformation.hpp>
#include <sm/kinematics/transformations.hpp>
#include <sm/serialization_macros.hpp>


namespace sm {
  namespace kinematics {
    
      double * Transformation::qptr() { return &_q_a_b[0]; }
      double * Transformation::tptr() { return &_t_a_b_a[0]; }
    
    Transformation::Transformation() :
      _q_a_b(quatIdentity()), _t_a_b_a(0.0, 0.0, 0.0)
    {
      
    }

    
    Transformation::Transformation(Eigen::Matrix4d const & T_a_b) :
      _q_a_b( r2quat(T_a_b.topLeftCorner<3,3>()) ),
      _t_a_b_a( T_a_b.topRightCorner<3,1>() )
    {

    }

    Transformation::Transformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a) :
      _q_a_b(q_a_b), _t_a_b_a(t_a_b_a)
    {
        _q_a_b.normalize();
    }

    Transformation::~Transformation(){}

    /// @return the rotation matrix
    Eigen::Matrix3d Transformation::C() const
    {
      return quat2r(_q_a_b);
    }

    /// @return the translation vector
    const Eigen::Vector3d & Transformation::t() const
    {
      return _t_a_b_a;
    }


    const Eigen::Vector4d & Transformation::q() const
    {
      return _q_a_b;
    }
    
      void Transformation::set( const Eigen::Matrix4d & T_a_b )
      {
          _q_a_b = r2quat(T_a_b.topLeftCorner<3,3>());
          _t_a_b_a = T_a_b.topRightCorner<3,1>();
      }

    Eigen::Matrix4d Transformation::T() const
    {
      Eigen::Matrix4d T_a_b;
      // \todo...make this do less copying.
      T_a_b.topLeftCorner<3,3>() = quat2r(_q_a_b);
      T_a_b.topRightCorner<3,1>() = _t_a_b_a;
      T_a_b.bottomLeftCorner<1,3>().setZero();
      T_a_b(3,3) = 1.0;
      return T_a_b;
    }

    Eigen::Matrix<double, 3, 4> Transformation::T3x4() const
    {
      Eigen::Matrix<double, 3, 4> T3x4;
      // \todo...make this do less copying.
      T3x4.topLeftCorner<3,3>() = quat2r(_q_a_b);
      T3x4.topRightCorner<3,1>() = _t_a_b_a;
      return T3x4;
    }

    Transformation Transformation::inverse() const
    {      
      // \todo Make this do less copying.
      return Transformation(quatInv(_q_a_b), quatRotate(quatInv(_q_a_b), -_t_a_b_a));
    }

    void Transformation::checkTransformationIsValid( void ) const
    {
      // \todo.
    }


    Transformation Transformation::operator*(const Transformation & rhs) const
    {
      return Transformation(qplus(_q_a_b, rhs._q_a_b), quatRotate(_q_a_b, rhs._t_a_b_a) + _t_a_b_a);
    }

    Eigen::Vector3d Transformation::operator*(const Eigen::Vector3d & rhs) const
    {
      return quatRotate(_q_a_b, rhs) + _t_a_b_a;
    }

    Eigen::Vector4d Transformation::operator*(const Eigen::Vector4d & rhs) const
    {
      Eigen::Vector4d rval;
      rval.head<3>() = quatRotate(_q_a_b, rhs.head<3>()) + rhs[3] * _t_a_b_a;
      rval[3] = rhs[3];
      
      return rval;
      
    }


    HomogeneousPoint Transformation::operator*(const HomogeneousPoint & rhs) const
    {
      Eigen::Vector4d rval = rhs.toHomogeneous();
      rval.head<3>() = (quatRotate(_q_a_b, rhs.toHomogeneous().head<3>()) + rval[3] * _t_a_b_a).eval();
      return HomogeneousPoint(rval);
      
    }


    void Transformation::setRandom()
    {
      _q_a_b = quatRandom();
      _t_a_b_a = (Eigen::Vector3d::Random().array() - 0.5) * 100.0;
    }

    bool Transformation::isBinaryEqual(const Transformation & rhs) const
    {
      return SM_CHECKMEMBERSSAME(rhs, _q_a_b) && SM_CHECKMEMBERSSAME(rhs, _t_a_b_a);
    }

    /// \brief The update step for this transformation from a minimal update.
    void Transformation::oplus(const Eigen::Matrix<double,6,1> & dt)
    {
      _q_a_b = updateQuat( _q_a_b, dt.tail<3>() );
      _t_a_b_a += dt.head<3>();
    }

    Eigen::Matrix<double,6,6> Transformation::S() const
    {
      Eigen::Matrix<double,6,6> S;
      S.setIdentity();
      S.topRightCorner<3,3>() = -crossMx(_t_a_b_a);
      return S;
    }

    void Transformation::setIdentity()
    {
      _q_a_b = quatIdentity();
      _t_a_b_a.setZero();
    }

    /// \brief Set this to a random transformation.
    void Transformation::setRandom( double translationMaxMeters, double rotationMaxRadians)
    {
      // Create a random unit-length axis.
      Eigen::Vector3d axis = Eigen::Vector3d::Random().array() - 0.5;
      
      // Create a random rotation angle in radians.
      double angle = sm::random::randLU(0.0, rotationMaxRadians);

      // Now a random axis/angle.cp 
      axis.array() *= angle/axis.norm(); 

      
      Eigen::Vector3d t;
      t.setRandom();
      t.array() -= 0.5;
      t.array() *= sm::random::randLU(0.0, translationMaxMeters)/t.norm();

      _q_a_b = axisAngle2quat(axis);
      _t_a_b_a = t;

    }


    UncertainTransformation Transformation::operator*(const UncertainTransformation & UT_b_c) const
    {
      const Transformation & T_a_b = *this;

      const Transformation & T_b_c = UT_b_c;
      Transformation T_a_c = T_a_b * T_b_c;
      UncertainTransformation::covariance_t T_a_b_boxtimes = boxTimes(T_a_b.T());
      
      UncertainTransformation::covariance_t U_a_c = T_a_b_boxtimes * UT_b_c.U() * T_a_b_boxtimes.transpose();

      return UncertainTransformation(T_a_c, U_a_c);
      
    }

    UncertainHomogeneousPoint Transformation::operator*(const UncertainHomogeneousPoint & p_1) const
    {
      const Transformation & T_0_1 = *this;
      Eigen::Vector4d p_0 = T_0_1 * p_1.toHomogeneous();

      Eigen::Matrix4d T01 = T_0_1.T();
      
      UncertainHomogeneousPoint::covariance_t U = T01 * p_1.U4() * T01.transpose();
      
      return UncertainHomogeneousPoint(p_0,U);
    }

      /// \brief rotate a point (do not translate)
      Eigen::Vector3d Transformation::rotate(const Eigen::Vector3d & p) const
      {
          return quatRotate(_q_a_b, p);
      }

      /// \brief rotate a point (do not translate)
      Eigen::Vector4d Transformation::rotate(const Eigen::Vector4d & p) const
      {
          Eigen::Vector4d rval = p;
          rval.head<3>() = quatRotate(_q_a_b, rval.head<3>());
          return rval;
      }

      UncertainVector3 Transformation::rotate(const UncertainVector3 & p) const
      {
          
          Eigen::Vector3d mean = rotate(p.mean());
          Eigen::Matrix3d R = C();
          Eigen::Matrix3d P = R * p.covariance() * R.transpose();
          
          return UncertainVector3(mean, P);
      }


      // Interpolate the transformation at time si between T0 (at time s0) and T1 (at time s1)
      Transformation interpolateTransformations(const Transformation & T0, double s0,
                                                const Transformation & T1, double s1,
                                                double si)
      {
          if( s0 > s1 )
          {
              return interpolateTransformations( T1, s1, T0, s0, si );
          }
          
          // The time span of interpolation. Make sure this is greater than zero.
          double ds = std::max(1e-14, s1 - s0);
          // The place of si on this timespan 0.0 --> s0, 1.0 -> s1
          double di = (si - s0) / ds;
          return slerpTransformations(T0, T1, di);
          
      }

      /// brief linear interpolate between T0 and T1 as si moves from 0.0 to 1.0
      Transformation slerpTransformations(const Transformation & T0, 
                                          const Transformation & T1, 
                                          double si)
      {
          // lazy. interpolate separately.
          return Transformation( qslerp( T0.q(), T1.q(), si), lerp( T0.t(), T1.t(), si) );
      }



  } // namespace kinematics
} // namespace sm
