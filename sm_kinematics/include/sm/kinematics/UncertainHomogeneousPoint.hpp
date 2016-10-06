#ifndef SM_UNCERTAIN_HOMOGENEOUS_POINT_HPP
#define SM_UNCERTAIN_HOMOGENEOUS_POINT_HPP
#include "HomogeneousPoint.hpp"

namespace sm {
  namespace kinematics {
    
    class UncertainHomogeneousPoint : public HomogeneousPoint
    {
    public:
      typedef Eigen::Matrix4d covariance_t;

      /// \brief set the point to the origin with zero uncertainty.
      UncertainHomogeneousPoint();
      /// \brief initialize the point with zero uncertainty
      UncertainHomogeneousPoint(const Eigen::Vector3d & p);
      /// \brief initialize the point with zero uncertainty
      UncertainHomogeneousPoint(const Eigen::Vector4d & p);
      /// \brief initialize the point with zero uncertainty
      UncertainHomogeneousPoint(const HomogeneousPoint & p);

      /// \brief Initialize the point from an uncertain Euclidean point.
      UncertainHomogeneousPoint(const Eigen::Vector3d & p, const Eigen::Matrix3d & U3);

      /// \brief Initialize the point from an uncertain homogeneous point.
      UncertainHomogeneousPoint(const Eigen::Vector4d & p, const Eigen::Matrix4d & U4);
      /// \brief Initialize the point from an uncertain homogeneous point.
      UncertainHomogeneousPoint(const HomogeneousPoint & p, const Eigen::Matrix4d & U4);

      /// \brief Initialize the point from an uncertain homogeneous point. The uncertainty
      /// here is 3x3 and related to the oplus operation. This is the uncertainty coming
      /// out of Gauss-Newton
      UncertainHomogeneousPoint(const Eigen::Vector4d & p, const Eigen::Matrix3d & Uav);

      /// \brief Initialize the point from an uncertain homogeneous point. The uncertainty
      /// here is 3x3 and related to the oplus operation. This is the uncertainty coming
      /// out of Gauss-Newton
      UncertainHomogeneousPoint(const HomogeneousPoint & p, const Eigen::Matrix3d & Uav);

      virtual ~UncertainHomogeneousPoint();

      /// Set a random point and uncertainty
      void setRandom();
      
      /// \brief Add two homogeneous points. 
      /// The result of this operation is the same as adding
      /// the equivelent vectors in R^3
      UncertainHomogeneousPoint operator+(const HomogeneousPoint & rhs) const;
      
      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
      UncertainHomogeneousPoint operator-(const HomogeneousPoint & rhs) const;

      /// \brief Add two homogeneous points. 
      /// The result of this operation is the same as adding
      /// the equivelent vectors in R^3
      virtual UncertainHomogeneousPoint operator+(const UncertainHomogeneousPoint & rhs) const;
      
      /// \brief Subtract one homogeneous point from another. 
      /// The result of this operation is the same as subtracting
      /// the equivelent vectors in R^3
      virtual UncertainHomogeneousPoint operator-(const UncertainHomogeneousPoint & rhs) const;

      
      /// \brief Return the 4x4 uncertainty of the homogeneous point.
      const Eigen::Matrix4d & U4() const;
      /// \brief Return the uncertainty of the 3x1 Euclidean point.
      Eigen::Matrix3d U3() const;
      /// \breif Return the uncertainty of the homogeneous point in "angular velocity" form.
      Eigen::Matrix3d U_av_form() const;
      
      /// \brief directly set the 4x4 uncertainty.
      void setU(const Eigen::Matrix4d & U);

      /// \brief set the 3x3 uncertainty related to the oplus operation.
      /// this is the uncertainty that comes out of Gauss-Newton.
      void setUOplus(const Eigen::Matrix3d & U);


      /// \brief Normalize the homogeneous point so that p.transpose() * p = 1.0
      ///        This function will convert the uncertainty accordingly.
      virtual void normalize();


        enum {CLASS_SERIALIZATION_VERSION = 0};
        BOOST_SERIALIZATION_SPLIT_MEMBER()
        
	  template<class Archive>
        void save(Archive & ar, const unsigned int /* version */) const
	  {
          ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(HomogeneousPoint);
          ar << BOOST_SERIALIZATION_NVP(_U);
	  }

	  template<class Archive>
	  void load(Archive & ar, const unsigned int version)
	  {
          SM_ASSERT_LE(std::runtime_error, version, (unsigned int)CLASS_SERIALIZATION_VERSION, "Unsupported serialization version");
          ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(HomogeneousPoint);
          ar >> BOOST_SERIALIZATION_NVP(_U);
	  }


	  bool isBinaryEqual(const UncertainHomogeneousPoint & rhs) const;
    private:
      /// \brief the V matrix from the note.
      Eigen::Matrix<double,4,3> V();
      Eigen::Matrix4d _U;
    };
    
  } // namespace kinematics
} // namespace sm

BOOST_CLASS_VERSION(sm::kinematics::UncertainHomogeneousPoint, sm::kinematics::UncertainHomogeneousPoint::CLASS_SERIALIZATION_VERSION);



#endif /* SM_UNCERTAIN_HOMOGENEOUS_POINT_HPP */
