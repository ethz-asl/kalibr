#ifndef SM_UNCERTAIN_VECTOR_HPP
#define SM_UNCERTAIN_VECTOR_HPP

#include <sm/kinematics/homogeneous_coordinates.hpp>
#include <sm/boost/serialization.hpp>
#include <sm/random.hpp>


namespace sm {
    namespace kinematics {
        
        template<int D>
        class UncertainVector
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /// \brief The number of dimensions
            enum { Dimension = D };

            /// \brief The underlying vector type
            typedef Eigen::Matrix<double, Dimension, 1> value_t;

            /// \brief The underlying covariance type
            typedef Eigen::Matrix<double, Dimension, Dimension> covariance_t;

            /// \brief A linear operator type
            typedef Eigen::Matrix<double, Dimension, Dimension> linear_operator_t;
            
            /// \brief a default constructor
            UncertainVector();
            
            /// \brief a constructor for vectors without uncertainty
            UncertainVector(const value_t & v);

            /// \brief a constructor for an uncertain vector
            UncertainVector(const value_t & v, const covariance_t & E);
            
            /// \brief the destructor
            virtual ~UncertainVector();
            
            /// \brief get the vector
            const value_t & v() const;
            
            /// \brief get the value
            const value_t & mean() const;

            /// \brief set the vector
            void setMean(const value_t & v);
                        
            /// \brief get the covariance
            const covariance_t & E() const;

            /// \brief get the covariance
            const covariance_t & covariance() const;

            /// \brief set the covariance
            void setCovariance(const covariance_t & E);

            UncertainVector<D> operator+(const UncertainVector<D> & rhs) const;
            UncertainVector<D> operator+(const value_t & rhs) const;

            UncertainVector<D> operator-(const UncertainVector<D> & rhs) const;
            UncertainVector<D> operator-(const value_t & rhs) const;

            UncertainVector<D> operator*(const UncertainVector<1> & rhs) const;
            UncertainVector<D> operator*(double s) const;

            UncertainVector<D> normalized();

            void normalize();

            UncertainVector<1> dot(const UncertainVector<D> & rhs) const;
            UncertainVector<1> dot(const value_t & rhs) const;

            void setRandom();
            void setZero();
            bool isBinaryEqual(const UncertainVector<D> & rhs) const;
            
            enum {CLASS_SERIALIZATION_VERSION = 0};
            BOOST_SERIALIZATION_SPLIT_MEMBER()

            template<class Archive>
            void load(Archive & ar, const unsigned int version)
                {
                    SM_ASSERT_LE(std::runtime_error, version, (unsigned int)CLASS_SERIALIZATION_VERSION, "Unsupported serialization version");
             
                    ar >> BOOST_SERIALIZATION_NVP(_v);
                    ar >> BOOST_SERIALIZATION_NVP(_E);
                }

            template<class Archive>
            void save(Archive & ar, const unsigned int /* version */) const
                {
                    ar << BOOST_SERIALIZATION_NVP(_v);
                    ar << BOOST_SERIALIZATION_NVP(_E);
                }

        private:
            
            /// \brief the vector
            value_t _v;
            
            /// \brief the covariance
            covariance_t _E;

        };

        template<>
        class UncertainVector<1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            /// \brief The number of dimensions
            enum { Dimension = 1 };

            /// \brief The underlying vector type
            typedef double value_t;

            /// \brief The underlying covariance type
            typedef double covariance_t;
            
            /// \brief a default constructor
            UncertainVector();
            
            /// \brief a constructor for vectors without uncertainty
            UncertainVector(const value_t & v);

            /// \brief a constructor for an uncertain vector
            UncertainVector(const value_t & v, const covariance_t & E);
            
            /// \brief the destructor
            virtual ~UncertainVector();
            
            /// \brief get the vector
            const value_t & v() const;
            
            /// \brief get the vector
            const value_t & mean() const;

            /// \brief set the vector
            void setMean(const value_t & v);
                        
            /// \brief get the covariance
            const covariance_t & E() const;

            /// \brief get the covariance
            const covariance_t & covariance() const;

            /// \brief set the covariance
            void setCovariance(const covariance_t & E);

            UncertainVector<1> dot(const UncertainVector<1> & rhs) const;
            UncertainVector<1> dot(const value_t & rhs) const;

            UncertainVector<1> operator*(const UncertainVector<1> & rhs) const;
            UncertainVector<1> operator*(value_t rhs) const;

            UncertainVector<1> operator+(const UncertainVector<1> & rhs) const;
            UncertainVector<1> operator+(value_t rhs) const;

            UncertainVector<1> operator-(const UncertainVector<1> & rhs) const;
            UncertainVector<1> operator-(value_t rhs) const;

            UncertainVector<1> operator/(const UncertainVector<1> & rhs) const;
            UncertainVector<1> operator/(value_t rhs) const;

            template<int DD>
            UncertainVector<DD> operator*(const UncertainVector<DD> & rhs) const;

            template<int DD>
            UncertainVector<DD> operator*(const Eigen::Matrix<double,DD,1> & rhs) const;

            void setRandom();
            void setZero();
            bool isBinaryEqual(const UncertainVector<1> & rhs) const;

            enum {CLASS_SERIALIZATION_VERSION = 0};
            BOOST_SERIALIZATION_SPLIT_MEMBER()

            template<class Archive>
            void load(Archive & ar, const unsigned int version)
                {
                    SM_ASSERT_LE(std::runtime_error, version, (unsigned int)CLASS_SERIALIZATION_VERSION, "Unsupported serialization version");
             
                    ar >> BOOST_SERIALIZATION_NVP(_v);
                    ar >> BOOST_SERIALIZATION_NVP(_sigma2);
                }

            template<class Archive>
            void save(Archive & ar, const unsigned int /* version */) const
                {
                    ar << BOOST_SERIALIZATION_NVP(_v);
                    ar << BOOST_SERIALIZATION_NVP(_sigma2);
                }
            
        private:
            
            /// \brief the vector
            value_t _v;
            
            /// \brief the covariance
            covariance_t _sigma2;

        };


    } // namespace kinematics
} // namespace sm

#include "implementation/UncertainVector.hpp"

template<int D>
sm::kinematics::UncertainVector<D> operator*(const typename sm::kinematics::UncertainVector<D>::linear_operator_t & M, const sm::kinematics::UncertainVector<D> & v);

namespace sm {
    namespace kinematics {
        typedef UncertainVector<1> UncertainScalar;
        typedef UncertainVector<2> UncertainVector2;
        typedef UncertainVector<3> UncertainVector3;
        typedef UncertainVector<4> UncertainVector4;

    } // namespace kinematics
} // namespace sm

SM_BOOST_CLASS_VERSION_I1(sm::kinematics::UncertainVector);

#endif /* SM_UNCERTAIN_VECTOR_HPP */
