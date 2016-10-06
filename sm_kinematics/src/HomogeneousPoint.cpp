#include <sm/kinematics/HomogeneousPoint.hpp>
#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/UncertainHomogeneousPoint.hpp>
#include <sm/serialization_macros.hpp>

namespace sm {
  namespace kinematics {
    
    /// \brief Initialize the point to zero
    HomogeneousPoint::HomogeneousPoint() :
      _ph( 0.0, 0.0, 0.0, 1.0 )
    {
      
    }

    /// \brief Initialize the point from a Euclidean point
    HomogeneousPoint::HomogeneousPoint(const Eigen::Vector3d & p) :
      _ph( p(0), p(1), p(2), 1.0 )
    {

    }

    /// \brief Initialize the point from a Homogeneous point
    HomogeneousPoint::HomogeneousPoint(const Eigen::Vector4d & p) :
      _ph(p)
    {
      SM_ASSERT_GT_DBG(Exception, _ph.norm(), 0.0, "The point " << p.transpose() << " is not a valid member of the set of homogeneous points");
    }

      
    HomogeneousPoint::~HomogeneousPoint()
    {

    }

    /// \brief Add two homogeneous points. 
    /// The result of this operation is the same as adding
    /// the equivelent vectors in R^3
    HomogeneousPoint HomogeneousPoint::operator+(const HomogeneousPoint & rhs) const
    {
      Eigen::Vector4d rval;
      
      rval.head<3>() = _ph.head<3>() * rhs._ph[3] + rhs._ph.head<3>() * _ph[3];
      rval[3] = _ph[3] * rhs._ph[3];
      return  rval;
    }

    UncertainHomogeneousPoint HomogeneousPoint::operator+(const UncertainHomogeneousPoint & rhs) const
    {
      Eigen::Vector4d rval;
      rval.head<3>() = _ph.head<3>() * rhs._ph[3] + rhs._ph.head<3>() * _ph[3];
      rval[3] = _ph[3] * rhs._ph[3];
      
      return UncertainHomogeneousPoint(rval, rhs.U4());

    }
      

      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
    UncertainHomogeneousPoint HomogeneousPoint::operator-(const UncertainHomogeneousPoint & rhs) const
    {
      Eigen::Vector4d rval;
      rval.head<3>() = _ph.head<3>() * rhs._ph[3] - rhs._ph.head<3>() * _ph[3];
      rval[3] = _ph[3] * rhs._ph[3];
      
      return UncertainHomogeneousPoint(rval, rhs.U4());
    }


    HomogeneousPoint & HomogeneousPoint::operator=(const Eigen::Vector4d & rhs)
    {
      SM_ASSERT_GT_DBG(Exception, rhs.norm(), 0.0, "The point " << rhs.transpose() << " is not a valid member of the set of homogeneous points");
      _ph = rhs;
      
      return *this;
    }


    HomogeneousPoint & HomogeneousPoint::operator=(const Eigen::Vector3d & rhs)
    {
      _ph.head<3>() = rhs;
      _ph[3] = 1.0;
      return *this;
    }

    /// \brief Subtract one homogeneous point from another. 
    /// The result of this operation is the same as subtracting
    /// the equivelent vectors in R^3
    HomogeneousPoint HomogeneousPoint::operator-(const HomogeneousPoint & rhs) const
    {
      Eigen::Vector4d rval;
      
      rval.head<3>() = _ph.head<3>() * rhs._ph[3] - rhs._ph.head<3>() * _ph[3];
      rval[3] = _ph[3] * rhs._ph[3];
      return  rval;      
    }

      
    /// \brief Implements the "oplus" operator for updating the point from a minimal perturbation
    ///
    /// This oplus operator maps to the \rho(dp)^+ p from quaternion algebra.
    void HomogeneousPoint::oplus(const Eigen::Vector3d & dp)
    {
      _ph = updateQuat(_ph,dp);
    }
    
    /// \brief converts the homogenous point to a pointing vector by setting the homogeneous coordinate to zero
    void HomogeneousPoint::convertToVector()
    {
      _ph[3] = 0.0;
    }

	/// \brief converts a pointing vector to a normalized homogeneous point with unit length
	void HomogeneousPoint::convertToPoint()
    {
	  SM_ASSERT_TRUE_DBG(Exception, isVector(), "The homogeneous point is not a vector.");
	  normalize();
      _ph[3] = 1.0;
    }
      
    /// \brief scale the point by a constant factor
    void HomogeneousPoint::scale(double scaleFactor)
    {
      _ph[0] *= scaleFactor;
      _ph[1] *= scaleFactor;
      _ph[2] *= scaleFactor;
    }

    /// \brief get the Euclidean representation
    Eigen::Vector3d HomogeneousPoint::toEuclidean() const
    {
      SM_ASSERT_GT_DBG(Exception, fabs(_ph[3]), 0.0, "The point " << _ph.transpose() << " can not be represented in Euclidean space. It is infinitely far away from the origin");

      double recip_p3 = 1.0/_ph[3];
      return Eigen::Vector3d(_ph[0] * recip_p3, _ph[1] * recip_p3, _ph[2] * recip_p3);
    }

    /// \brief get the Euclidean representation and the jacobian of the oplus operator with respect to this point. The Jacobian should be 3x3.
    Eigen::Vector3d HomogeneousPoint::toEuclideanAndJacobian(euclidean_jacobian_t & J) const
    {
      double recip_p3 = 1.0/_ph[3];
      Eigen::Vector3d v3 = Eigen::Vector3d(_ph[0] * recip_p3, _ph[1] * recip_p3, _ph[2] * recip_p3);
      
      homogeneous_jacobian_t Jh;
      toHomogeneousAndJacobian(Jh);

      Eigen::Matrix<double,3,4> Je;
      Je.topRightCorner<3,1>() = -v3 * recip_p3;
      Je.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity() * recip_p3;
      
      J = Je * Jh;
      return v3;
    }
    
    /// \brief get the homogeneous representation
    const Eigen::Vector4d & HomogeneousPoint::toHomogeneous() const
    {
      return _ph;
    }

    /// \brief get the homogeneous representation and the Jacobian of the oplus operator with respect to this point. The Jacobian should be 4x3.
    const Eigen::Vector4d & HomogeneousPoint::toHomogeneousAndJacobian(homogeneous_jacobian_t & J) const
    {

      // 1 [  p3, -p2,  p1]
      // - [  p2,  p3, -p0]
      // 2 [ -p1,  p0,  p3]
      //   [ -p0, -p1, -p2]
    
      J(0,0) =  _ph[3];
      J(0,1) = -_ph[2];
      J(0,2) =  _ph[1];
      J(1,0) =  _ph[2];
      J(1,1) =  _ph[3];
      J(1,2) = -_ph[0];
      J(2,0) = -_ph[1];
      J(2,1) =  _ph[0];
      J(2,2) =  _ph[3];
      J(3,0) = -_ph[0];
      J(3,1) = -_ph[1];
      J(3,2) = -_ph[2];

      J *= 0.5;

      return _ph;
    }


    void HomogeneousPoint::setRandom()
    {
      _ph.setRandom();
      normalize();
    }

    /// \brief Normalize the point so that it is unit length
    void HomogeneousPoint::normalize()
    {
      _ph = _ph/_ph.norm();
    }

    void HomogeneousPoint::setZero()
    {
      _ph.head<3>().setZero();
      _ph[3] = 1.0;
    }

	bool HomogeneousPoint::atInfinity() const
	{
		return (_ph[3] == 0.0);
	}

	bool HomogeneousPoint::isVector() const
	{
		return atInfinity();
	}

	bool HomogeneousPoint::isBinaryEqual(const HomogeneousPoint & rhs) const
	{
	  return SM_CHECKMEMBERSSAME(rhs, _ph);
	}

  } // namespace kinematics
} // namespace sm
