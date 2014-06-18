#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/serialization_macros.hpp>

namespace sm {
  namespace kinematics {
    
    /// \brief set the point to the origin with zero uncertainty.
    UncertainHomogeneousPoint::UncertainHomogeneousPoint() :
      HomogeneousPoint()
    {
      _U.setZero();
    }

    /// \brief initialize the point with zero uncertainty
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const Eigen::Vector3d & p) :
      HomogeneousPoint(p)
    {
      _U.setZero();
    }

    /// \brief initialize the point with zero uncertainty
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const Eigen::Vector4d & p) :
      HomogeneousPoint(p)
    {
      _U.setZero();
    }

    /// \brief initialize the point with zero uncertainty
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const HomogeneousPoint & p) :
      HomogeneousPoint(p)
    {
      _U.setZero();
    }


    /// \brief Initialize the point from an uncertain Euclidean point.
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const Eigen::Vector3d & p, const Eigen::Matrix3d & U3) :
      HomogeneousPoint(p)
    {
      _U.setZero();
      _U.topLeftCorner<3,3>() = U3;
    }


    /// \brief Initialize the point from an uncertain homogeneous point.
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const Eigen::Vector4d & p, const Eigen::Matrix4d & U4)  :
      HomogeneousPoint(p)
    {
      _U = U4;
    }
    /// \brief Initialize the point from an uncertain homogeneous point.
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const HomogeneousPoint & p, const Eigen::Matrix4d & U4) :
      HomogeneousPoint(p)
    {
      _U = U4;
    }

    /// \brief Initialize the point from an uncertain homogeneous point. The uncertainty
    /// here is 3x3 and related to the oplus operation. This is the uncertainty coming
    /// out of Gauss-Newton
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const Eigen::Vector4d & p, const Eigen::Matrix3d & Uav) :
      HomogeneousPoint(p)
    {
      setUOplus(Uav);
    }


    /// \brief Initialize the point from an uncertain homogeneous point. The uncertainty
    /// here is 3x3 and related to the oplus operation. This is the uncertainty coming
    /// out of Gauss-Newton
    UncertainHomogeneousPoint::UncertainHomogeneousPoint(const HomogeneousPoint & p, const Eigen::Matrix3d & Uav) :
      HomogeneousPoint(p)
    {
      setUOplus(Uav);
    }


     UncertainHomogeneousPoint::~UncertainHomogeneousPoint()
    {

    }


    /// Set a random point and uncertainty
    void UncertainHomogeneousPoint::setRandom()
    {
      HomogeneousPoint::setRandom();
      Eigen::Matrix4d U;
      U.setRandom();
      _U = U.transpose() * U + Eigen::Matrix4d::Identity();
    }

            
    /// \brief Return the 4x4 uncertainty of the homogeneous point.
    const Eigen::Matrix4d & UncertainHomogeneousPoint::U4() const
    {
      return _U;
    }

    /// \brief Return the uncertainty of the 3x1 Euclidean point.
    Eigen::Matrix3d UncertainHomogeneousPoint::U3() const
    {
      Eigen::Matrix<double,3,4> J = fromHomogeneousJacobian(_ph);
      
      return J * _U * J.transpose();
    }

    /// \breif Return the uncertainty of the homogeneous point in "angular velocity" form.
    Eigen::Matrix3d UncertainHomogeneousPoint::U_av_form() const
    {
      SM_ASSERT_NEAR(HomogeneousPoint::Exception, _ph.norm(), 1.0, 1e-6, "The \"oplus\" form of covariance is only valid for points on the unit sphere in R^4"); 
      //std::cout << "uav: " << _ph.transpose() << std::endl;
      Eigen::Matrix<double,3,4> S = quatS(_ph);
      return S * _U * S.transpose();
    }

      
    /// \brief directly set the 4x4 uncertainty.
    void UncertainHomogeneousPoint::setU(const Eigen::Matrix4d & U)
    {
      _U = U;
    }


    /// \brief set the 3x3 uncertainty related to the oplus operation.
    /// this is the uncertainty that comes out of Gauss-Newton.
    void UncertainHomogeneousPoint::setUOplus(const Eigen::Matrix3d & U)
    {
      SM_ASSERT_NEAR(HomogeneousPoint::Exception, _ph.norm(), 1.0, 1e-6, "The \"oplus\" form of covariance is only valid for points on the unit sphere in R^4"); 
      Eigen::Matrix<double,4,3> invS = quatInvS(_ph);
      _U = invS * U * invS.transpose();
    }

    Eigen::Matrix<double,4,3> UncertainHomogeneousPoint::V()
    {
      Eigen::Matrix<double,4,3> V;
      V.setZero();
      V(0,0) = V(1,1) = V(2,2) = 0.5;
      return V;
    }

    /// \brief Normalize the point so that it is unit length
    void UncertainHomogeneousPoint::normalize()
    {
      Eigen::Matrix4d J;
      _ph = sm::kinematics::normalizeAndJacobian<4>(_ph,J);
      
      _U = (J * _U * J.transpose()).eval();
    }


    /// \brief Add two homogeneous points. 
    /// The result of this operation is the same as adding
    /// the equivelent vectors in R^3
    UncertainHomogeneousPoint UncertainHomogeneousPoint::operator+(const HomogeneousPoint & rhs) const
    {
      Eigen::Vector4d rval;
      const Eigen::Vector4d & pr = rhs.toHomogeneous();
      rval.head<3>() = _ph.head<3>() * pr[3] + pr.head<3>() * _ph[3];
      rval[3] = _ph[3] * pr[3];
      
      return UncertainHomogeneousPoint(rval, U4());

    }
    
      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
      UncertainHomogeneousPoint UncertainHomogeneousPoint::operator-(const HomogeneousPoint & rhs) const
      {
	Eigen::Vector4d rval;
	const Eigen::Vector4d & pr = rhs.toHomogeneous();
	rval.head<3>() = _ph.head<3>() * pr[3] - pr.head<3>() * _ph[3];
	rval[3] = _ph[3] * pr[3];
	
	return UncertainHomogeneousPoint(rval, U4());

      }

      /// \brief Add two homogeneous points. 
      /// The result of this operation is the same as adding
      /// the equivelent vectors in R^3
      UncertainHomogeneousPoint UncertainHomogeneousPoint::operator+(const UncertainHomogeneousPoint & rhs) const
      {
	Eigen::Vector4d rval;
	rval.head<3>() = _ph.head<3>() * rhs._ph[3] + rhs._ph.head<3>() * _ph[3];
	rval[3] = _ph[3] * rhs._ph[3];

	Eigen::Matrix4d PL = toHomogeneousPlus(_ph);
	Eigen::Matrix4d PR = toHomogeneousPlus(rhs._ph);
	
	return UncertainHomogeneousPoint(rval, (Eigen::Matrix4d)(PR * U4() * PR.transpose() + PL * rhs.U4() * PL.transpose()));
      }
      
      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
      UncertainHomogeneousPoint UncertainHomogeneousPoint::operator-(const UncertainHomogeneousPoint & rhs) const
      {
	Eigen::Vector4d rval;
	rval.head<3>() = _ph.head<3>() * rhs._ph[3] - rhs._ph.head<3>() * _ph[3];
	rval[3] = _ph[3] * rhs._ph[3];

	Eigen::Matrix4d PL = toHomogeneousPlus(_ph);
	Eigen::Matrix4d PR = toHomogeneousPlus(rhs._ph);
	
	return UncertainHomogeneousPoint(rval, (Eigen::Matrix4d)(PR * U4() * PR.transpose() + PL * rhs.U4() * PL.transpose()));	
	
      }


	bool UncertainHomogeneousPoint::isBinaryEqual(const UncertainHomogeneousPoint & rhs) const
	{
	  return HomogeneousPoint::isBinaryEqual(rhs) && SM_CHECKMEMBERSSAME(rhs, _U);
	}
  } // namespace kinematics
} // namespace sm
