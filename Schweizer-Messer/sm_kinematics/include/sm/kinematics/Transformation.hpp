
#ifndef SM_TRANSFORMATION_HPP
#define SM_TRANSFORMATION_HPP

#include <sm/kinematics/quaternion_algebra.hpp>
#include <boost/serialization/nvp.hpp>
#include <sm/eigen/serialization.hpp>
#include "HomogeneousPoint.hpp"
#include <boost/serialization/split_member.hpp>
#include <boost/serialization/version.hpp>
#include <sm/kinematics/UncertainVector.hpp>

namespace sm {
  namespace kinematics {
    
    class UncertainTransformation;
    class UncertainHomogeneousPoint;

      ///
    /// @class Transformation
    /// @brief a class that represents a transformation.
    /// @todo describe how these transformations work
    ///
    class Transformation 
    {
    public:
      // See: http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW      

      /// 
      /// Default constructor. The transformation will
      /// be set to identity.
      ///
      Transformation();

      /// 
      /// Constructor. The transformation will be set to T_ab
      ///
      /// @param T_ab the initializing transformation.
      ///
      Transformation(const Eigen::Matrix4d & T_a_b);

      /// 
      /// Constructor. 
      ///
      /// @param q_a_b The quaternion that transforms vectors from b to a
      /// @param t_a_b_a the vector from the origin of frame a, to the origin of frame b, expresessed in frame a.
      ///
      Transformation(const Eigen::Vector4d & q_a_b, const Eigen::Vector3d & t_a_b_a);

      virtual ~Transformation();

      /// 
      ///
      /// @return The underlying transformation
      ///
      Eigen::Matrix4d T() const;

      /// @return the rotation matrix
      Eigen::Matrix3d C() const;

      /// @return the translation vector
      const Eigen::Vector3d & t() const;

      const Eigen::Vector4d & q() const;

      Eigen::Matrix<double, 3,4> T3x4() const;

      /** 
       * Return a copy of the transformation inverted.
       * 
       * @return The inverted transformation
       */
      Transformation inverse() const;

      /// \brief Set this to a random transformation.
      virtual void setRandom();
      
        virtual void set( const Eigen::Matrix4d & T_a_b );

      /// \brief Set this to a random transformation with bounded rotation and translation.
      virtual void setRandom( double translationMaxMeters, double rotationMaxRadians);
      /// \brief Set this transformation to identity
      void setIdentity();
      
      Transformation operator*(const Transformation & rhs) const;
      Eigen::Vector3d operator*(const Eigen::Vector3d & rhs) const;
      Eigen::Vector4d operator*(const Eigen::Vector4d & rhs) const;
      HomogeneousPoint operator*(const HomogeneousPoint & rhs) const;

      virtual UncertainTransformation operator*(const UncertainTransformation & rhs) const;
      virtual UncertainHomogeneousPoint operator*(const UncertainHomogeneousPoint & rhs) const;

      void checkTransformationIsValid( void ) const;

        /// \brief rotate a point (do not translate)
        Eigen::Vector3d rotate(const Eigen::Vector3d & p) const;

        /// \brief rotate a point (do not translate)
        Eigen::Vector4d rotate(const Eigen::Vector4d & p) const;

        /// \brief rotate a point (do not translate)
        UncertainVector3 rotate(const UncertainVector3 & p) const;

        
        double * qptr();
        double * tptr();

      
        enum {CLASS_SERIALIZATION_VERSION = 0};


        BOOST_SERIALIZATION_SPLIT_MEMBER()
      /// 
      /// Serialize the Transformation to a boost::serialization archive.
      ///
      /// @param ar The archive
      /// @param version The archive file version number.
      ///
      template<class Archive>
      void save(Archive & ar, const unsigned int version) const;

      template<class Archive>
      void load(Archive & ar, const unsigned int version);

      
      bool isBinaryEqual(const Transformation & rhs) const;

      /// \brief The update step for this transformation from a minimal update.
      void oplus(const Eigen::Matrix<double,6,1> & dt);        

      /// \brief Return the S matrix that puts the oplus operation in the form
      ///        of a small transformation.
      Eigen::Matrix<double,6,6> S() const;


      
    protected:
        
      /// The quaternion that will become a rotation matrix C_a_b that 
      /// transforms vectors from b to a.
      Eigen::Vector4d _q_a_b;

      /// The vector from the origin of a to the origin of b, expressed in a
      Eigen::Vector3d _t_a_b_a;

    };

    
    template<class Archive>
    void Transformation::save(Archive & ar, const unsigned int /* version */) const
    {
      ar & BOOST_SERIALIZATION_NVP(_q_a_b);
      ar & BOOST_SERIALIZATION_NVP(_t_a_b_a);
    }

    template<class Archive>
    void Transformation::load(Archive & ar, const unsigned int version)
    {
        SM_ASSERT_LE(std::runtime_error, version, (unsigned int)CLASS_SERIALIZATION_VERSION, "Unsupported serialization version");
        ar >> BOOST_SERIALIZATION_NVP(_q_a_b);
        ar >> BOOST_SERIALIZATION_NVP(_t_a_b_a);
    }



      // Interpolate the transformation at time si between T0 (at time s0) and T1 (at time s1)
      Transformation interpolateTransformations(const Transformation & T0, double s0,
                                                const Transformation & T1, double s1,
                                                double si);

      /// brief linear interpolate between T0 and T1 as si moves from 0.0 to 1.0
      Transformation slerpTransformations(const Transformation & T0, 
                                                const Transformation & T1, 
                                                double si);



  } // namespace kinematics
  
} // namespace sm

BOOST_CLASS_VERSION(sm::kinematics::Transformation, sm::kinematics::Transformation::CLASS_SERIALIZATION_VERSION)


#endif /* SM_TRANSFORMATION_HPP */
