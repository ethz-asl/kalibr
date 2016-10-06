#ifndef SM_UNCERTAIN_TRANSFORMATION_HPP
#define SM_UNCERTAIN_TRANSFORMATION_HPP

#include "Transformation.hpp"
#include <boost/serialization/base_object.hpp>
#include "UncertainHomogeneousPoint.hpp"

namespace sm {
  namespace kinematics {
    
    class UncertainTransformation : public Transformation
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef Eigen::Matrix<double,6,6> covariance_t;
      /// 
      /// Default constructor. The transformation and uncertainty will
      /// both be set to identity.
      ///
      UncertainTransformation();

      UncertainTransformation(const Eigen::Matrix4d & T, const covariance_t & U);

      UncertainTransformation(const Eigen::Matrix4d & T, double diagonalTranslationVariance, double diagonalRotationVariance);

      UncertainTransformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a, const covariance_t & U);

      UncertainTransformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a, double diagonalTranslationVariance, double diagonalRotationVariance);

      UncertainTransformation(const Transformation & T, const covariance_t & U);

      UncertainTransformation(const Transformation & T, double diagonalTranslationVariance, double diagonalRotationVariance);

      /// \brief Initialize with zero uncertainty
      UncertainTransformation(const Transformation & T);
      /// \brief Initialize with zero uncertainty
      UncertainTransformation(const Eigen::Matrix4d & T);
      /// \brief Initialize with zero uncertainty
      UncertainTransformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a);


      virtual ~UncertainTransformation();
      
      virtual UncertainTransformation operator*(const UncertainTransformation & rhs) const;
      UncertainTransformation operator*(const Transformation & rhs) const;
      UncertainHomogeneousPoint operator*(const HomogeneousPoint & rhs) const;

      Eigen::Vector3d operator*(const Eigen::Vector3d & rhs) const{ return Transformation::operator*(rhs); }
      Eigen::Vector4d operator*(const Eigen::Vector4d & rhs) const{ return Transformation::operator*(rhs); }

      virtual UncertainHomogeneousPoint operator*(const UncertainHomogeneousPoint & rhs) const;

      Transformation toTransformation() const;
      UncertainTransformation inverse() const;

      const covariance_t & U() const;

      /// \brief hhis gets the uncertainty based on the "oplus" function for updating the transformation matrix.
      covariance_t UOplus() const;

      /// \brief This sets the uncertainty directly.
      void setU(const covariance_t & U);

      /// \brief This sets the uncertainty based on the "oplus" function for updating the transformation matrix.
      /// This is a different uncertainty than that carried around by the class, so one should be careful. 
      void setUOplus(const covariance_t & oplusU);

      // Set this to a random transformation.
      virtual void setRandom();

      virtual void setRandom( double translationMaxMeters, double rotationMaxRadians );

      bool isBinaryEqual(const UncertainTransformation & rhs) const;

        enum {CLASS_SERIALIZATION_VERSION = 0};
        BOOST_SERIALIZATION_SPLIT_MEMBER()
      template<class Archive>
      void save(Archive & ar, const unsigned int version) const;
        template<class Archive>
      void load(Archive & ar, const unsigned int version);

	  
    private:
      covariance_t _U;
    };



    template<class Archive>
    void UncertainTransformation::save(Archive & ar, const unsigned int /* version */) const
    {

	  using ::boost::serialization::make_nvp;
      ar << BOOST_SERIALIZATION_BASE_OBJECT_NVP(Transformation);
      ar << make_nvp("_U", _U);//BOOST_SERIALIZATION_NVP(_U);
    }

    template<class Archive>
    void UncertainTransformation::load(Archive & ar, const unsigned int version)
    {
        SM_ASSERT_LE(std::runtime_error, version, (unsigned int)CLASS_SERIALIZATION_VERSION, "Unsupported serialization version");

	  using ::boost::serialization::make_nvp;
      ar >> BOOST_SERIALIZATION_BASE_OBJECT_NVP(Transformation);
      ar >> make_nvp("_U", _U);//BOOST_SERIALIZATION_NVP(_U);
    }

    
  } // namespace kinematics
} // namespace sm

BOOST_CLASS_VERSION(sm::kinematics::UncertainTransformation, sm::kinematics::UncertainTransformation::CLASS_SERIALIZATION_VERSION);      

#endif /* SM_UNCERTAIN_TRANSFORMATION_HPP */
