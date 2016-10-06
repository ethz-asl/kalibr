#ifndef SM_HOMOGENEOUS_POINT_HPP
#define SM_HOMOGENEOUS_POINT_HPP

#include <Eigen/Core>
#include <sm/assert_macros.hpp>
#include <sm/eigen/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>


namespace sm {
  namespace kinematics {
    class UncertainHomogeneousPoint;

    class HomogeneousPoint
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
      
      typedef Eigen::Matrix3d euclidean_jacobian_t;
      typedef Eigen::Matrix<double,4,3> homogeneous_jacobian_t;

      /// \brief Initialize the point to zero
      HomogeneousPoint();
      /// \brief Initialize the point from a Euclidean point
      HomogeneousPoint(const Eigen::Vector3d & p);
      /// \brief Initialize the point from a Homogeneous point
      HomogeneousPoint(const Eigen::Vector4d & p);
      
      virtual ~HomogeneousPoint();
      
      /// \brief get the Euclidean representation
      Eigen::Vector3d toEuclidean() const;

      /// \brief get the Euclidean representation and the jacobian of the oplus operator with respect to this point. The Jacobian should be 3x3.
      Eigen::Vector3d toEuclideanAndJacobian(euclidean_jacobian_t & J) const;

      /// \brief get the homogeneous representation
      const Eigen::Vector4d & toHomogeneous() const;

      /// \brief get the homogeneous representation and the Jacobian of the oplus operator with respect to this point. The Jacobian should be 3x3.
      const Eigen::Vector4d & toHomogeneousAndJacobian(homogeneous_jacobian_t & J) const;

      /// \brief Add two homogeneous points. 
      /// The result of this operation is the same as adding
      /// the equivelent vectors in R^3
      HomogeneousPoint operator+(const HomogeneousPoint & rhs) const;
      
      /// \brief Add two homogeneous points. 
      /// The result of this operation is the same as adding
      /// the equivelent vectors in R^3
      /// This version adds uncertainty.
      virtual UncertainHomogeneousPoint operator+(const UncertainHomogeneousPoint & rhs) const;
      

      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
      HomogeneousPoint operator-(const HomogeneousPoint & rhs) const;

      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
      virtual UncertainHomogeneousPoint operator-(const UncertainHomogeneousPoint & rhs) const;


      /// \brief Set this point from a 4x1 column
      HomogeneousPoint & operator=(const Eigen::Vector4d & rhs);
      
      /// \brief Set this point from a 3x1 column
      HomogeneousPoint & operator=(const Eigen::Vector3d & rhs);

      template<typename DERIVED>
      void set(const Eigen::MatrixBase<DERIVED> & p);

      /// \brief Implements the "oplus" operator for updating the point from a minimal perturbation
      ///
      /// This oplus operator maps to the \rho(dp)^+ p from quaternion algebra.
      void oplus(const Eigen::Vector3d & dp);
      
      /// \brief converts the homogenous point to a pointing vector by setting the homogeneous coordinate to zero
      void convertToVector();

	  /// \brief converts a pointing vector to a normalized homogeneous point with unit length
	  void convertToPoint();
      
      /// \brief scale the point by a constant factor
      void scale(double scaleFactor);

      /// \brief set to a random point.
      void setRandom();

      /// \brief set to the Euclidean point equaling zero
      void setZero();

	  /// \brief checks if homogeneous point is at infinity (if fourth coordinate is zero)
      bool atInfinity() const;

	  /// \brief checks if homogeneous point is a vector, i.e. it has infinite length (if fourth coordinate is zero)
	  bool isVector() const;

        double * pptr(){ return &_ph[0]; }

      /// \brief Normalize the point so that it is unit length
      virtual void normalize();
	  
	  /// \brief boost::serialization support
        enum {CLASS_SERIALIZATION_VERSION = 0};
        BOOST_SERIALIZATION_SPLIT_MEMBER()

	  template<class Archive>
        void save(Archive & ar, const unsigned int /* version */) const
	  {
          ar << ::boost::serialization::make_nvp("_ph",_ph);
	  }

	  template<class Archive>
	  void load(Archive & ar, const unsigned int version)
	  {
          SM_ASSERT_LE(std::runtime_error, version, (unsigned int)CLASS_SERIALIZATION_VERSION, "Unsupported serialization version");
          ar >> ::boost::serialization::make_nvp("_ph",_ph);
      }

	  bool isBinaryEqual(const HomogeneousPoint & rhs) const;
   protected:
      Eigen::Vector4d _ph;
    };

    
    template<typename DERIVED>
    void HomogeneousPoint::set(const Eigen::MatrixBase<DERIVED> & p)
    {
      SM_ASSERT_EQ_DBG(Exception, p.cols(), 1, "Trying to set a homogeneous point from a matrix");
      
      if(p.rows() == 3)
	{
	  _ph[0] = p[0];
	  _ph[1] = p[1];
	  _ph[2] = p[2];
	  _ph[3] = 1.0;
	}
      else if(p.rows() == 4)
	{
	  _ph[0] = p[0];
	  _ph[1] = p[1];
	  _ph[2] = p[2];
	  _ph[3] = p[3];
	}
      else
	{
	  SM_THROW(Exception, "Trying to initialize a homogeneous point with a " << p.rows() << "x" << p.cols() << " matrix. Only 3x1 and 4x1 are supported");
	}

    }


  } // namespace kinematics
} // namespace sm

BOOST_CLASS_VERSION(sm::kinematics::HomogeneousPoint, sm::kinematics::HomogeneousPoint::CLASS_SERIALIZATION_VERSION);


#endif /* SM_HOMOGENEOUS_POINT_HPP */
