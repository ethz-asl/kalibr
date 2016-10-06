#include <sm/serialization_macros.hpp>

template<int D>
sm::kinematics::UncertainVector<D> operator*(const typename sm::kinematics::UncertainVector<D>::linear_operator_t & M, const sm::kinematics::UncertainVector<D> & v)
{
    return sm::kinematics::UncertainVector<D>( M * v.v(), M * v.E() * M.transpose() );
}


namespace sm {
     namespace kinematics {
        
         /// \brief a default constructor
         template<int D>
         UncertainVector<D>::UncertainVector() : _v(value_t::Zero()), _E(covariance_t::Zero())
         {
             
         }

        
         /// \brief a constructor for vectors without covariance
         template<int D>
         UncertainVector<D>::UncertainVector(const value_t & v) : _v(v), _E(covariance_t::Zero())
         {
             
         }


         /// \brief a constructor for an uncertain vector
         template<int D>
         UncertainVector<D>::UncertainVector(const value_t & v, const covariance_t & E) : _v(v)
         {
             setCovariance(E);
         }

            
         /// \brief the destructor
         template<int D>
         UncertainVector<D>::~UncertainVector()
         {
             
         }

            
         /// \brief get the vector
         template<int D>
         const typename UncertainVector<D>::value_t & UncertainVector<D>::v() const
         {
             return _v;
         }

            
         /// \brief get the vector
         template<int D>
         const typename UncertainVector<D>::value_t & UncertainVector<D>::mean() const
         {
             return _v;
         }


         /// \brief set the vector
         template<int D>
         void UncertainVector<D>::setMean(const value_t & v)
         {
             _v = v;
         }

         template<int D>
         void UncertainVector<D>::setZero()
         {
             _v.setZero();
             _E.setZero();
         }

                        
         /// \brief get the covariance
         template<int D>
         const typename UncertainVector<D>::covariance_t & UncertainVector<D>::E() const
         {
             return covariance();
         }

         /// \brief get the covariance
         template<int D>
         const typename UncertainVector<D>::covariance_t & UncertainVector<D>::covariance() const
         {
             return _E;
         }



         /// \brief set the covariance
         template<int D>
         void UncertainVector<D>::setCovariance(const covariance_t & E)
         {
             _E = E;
         }



         template<int D>
         UncertainVector<D> UncertainVector<D>::operator+(const UncertainVector<D> & rhs) const
         {
             return UncertainVector<D>(_v + rhs._v, _E + rhs._E);
         }


         template<int D>
         UncertainVector<D> UncertainVector<D>::operator+(const value_t & rhs) const
         {
             return UncertainVector<D>(_v + rhs, _E);

         }


         template<int D>
         UncertainVector<D> UncertainVector<D>::operator-(const UncertainVector<D> & rhs) const
         {
             return UncertainVector<D>(_v - rhs._v, _E + rhs._E);
         }


         template<int D>
         UncertainVector<D> UncertainVector<D>::operator-(const value_t & rhs) const
         {
             return UncertainVector<D>(_v - rhs, _E);
         }


         template<int D>
         UncertainVector<D> UncertainVector<D>::operator*(const UncertainVector<1> & rhs) const
         {
             return UncertainVector<D>( _v * rhs.v(), _v * _v.transpose() * rhs.E() + _E * rhs.v() * rhs.v());
         }
         
         template<int D>
         UncertainVector<D> UncertainVector<D>::operator*(double s) const
         {
             return UncertainVector<D>(_v * s, _E * s * s);
         }
         
         template<int D>
         UncertainVector<D> UncertainVector<D>::normalized()
         {
             covariance_t nJ;
             value_t v = normalizeAndJacobian<D>( _v, nJ);
             
             return UncertainVector<D>(v, nJ * _E * nJ.transpose());
         }


         template<int D>
         void UncertainVector<D>::normalize()
         {
             covariance_t nJ;
             _v = normalizeAndJacobian<D>( _v, nJ);
             _E = nJ * _E * nJ.transpose();
   
         }

         template<int D>
         UncertainVector<1> UncertainVector<D>::dot(const UncertainVector<D> & rhs) const
         {
             return UncertainVector<1>( _v.dot(rhs._v), _v.dot( rhs._E * _v ) + rhs._v.dot( _E * rhs._v ) );
         }

         template<int D>
         UncertainVector<1> UncertainVector<D>::dot(const value_t & rhs) const
         {
             return UncertainVector<1>( _v.dot(rhs),  rhs.dot( _E * rhs ) );
         }

         template<int D>
         void UncertainVector<D>::setRandom()
         {
             _v.setRandom();
             _E.setRandom();
             _E = (_E * _E.transpose() + covariance_t::Identity()).eval();
         }

         template<int D>
         bool UncertainVector<D>::isBinaryEqual(const UncertainVector<D> & rhs) const
         {
             return SM_CHECKMEMBERSSAME(rhs, _v) && SM_CHECKMEMBERSSAME(rhs, _E);
         }



         inline void UncertainVector<1>::setRandom()
         {
             _v = sm::random::rand();
             _sigma2 = sm::random::rand();
             _sigma2 = _sigma2 * _sigma2;
         }


         inline bool UncertainVector<1>::isBinaryEqual(const UncertainVector<1> & rhs) const
         {
             return SM_CHECKMEMBERSSAME(rhs, _v) && SM_CHECKMEMBERSSAME(rhs, _sigma2);
         }





         inline UncertainVector<1>::UncertainVector() : _v(0.), _sigma2(0.)
         {

         }

        
         /// \brief a constructor for vectors without uncertainty

         
         inline UncertainVector<1>::UncertainVector(const value_t & v) : _v(v), _sigma2(0.)
         {

         }


         /// \brief a constructor for an uncertain vector
         
         inline UncertainVector<1>::UncertainVector(const value_t & v, const covariance_t & E): _v(v), _sigma2(E)
         {
             
         }

            
         /// \brief the destructor
         
         inline UncertainVector<1>::~UncertainVector()
         {

         }

            
         /// \brief get the vector
         
         inline const UncertainVector<1>::value_t & UncertainVector<1>::v() const
         {
             return _v;
         }

            
         /// \brief get the vector
         
         inline const UncertainVector<1>::value_t & UncertainVector<1>::mean() const
         {
             return _v;
         }


         /// \brief set the vector
         
         inline void UncertainVector<1>::setMean(const value_t & v)
         {
             _v = v;
         }


         inline void UncertainVector<1>::setZero()
         {
             _v = 0.0; _sigma2 = 0.0;
         }
                        
         /// \brief get the covariance
         
         inline const UncertainVector<1>::covariance_t & UncertainVector<1>::E() const
         {
             return _sigma2;
         }


         /// \brief get the covariance
         
         inline const UncertainVector<1>::covariance_t & UncertainVector<1>::covariance() const
         {
             return _sigma2;
         }


         /// \brief set the covariance
         
         inline void UncertainVector<1>::setCovariance(const covariance_t & E) 
         {
             _sigma2 = E;
         }


         
         inline UncertainVector<1> UncertainVector<1>::dot(const UncertainVector<1> & rhs) const
         {
             return UncertainVector<1>(_v * rhs._v, rhs._v * _sigma2 * rhs._v + _v * rhs._sigma2 * _v);
         }


         
         inline UncertainVector<1> UncertainVector<1>::dot(const value_t & rhs) const
         {
             return UncertainVector<1>(_v * rhs, rhs * _sigma2 * rhs);

         }


         
         inline UncertainVector<1> UncertainVector<1>::operator*(const UncertainVector<1> & rhs) const
         {
             return this->dot(rhs);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator*(value_t rhs) const
         {
             return this->dot(rhs);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator+(const UncertainVector<1> & rhs) const
         {
             return UncertainVector<1>(_v + rhs._v, _sigma2 + rhs._sigma2);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator+(value_t rhs) const
         {
             return UncertainVector<1>(_v + rhs, _sigma2);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator-(const UncertainVector<1> & rhs) const
         {
             return UncertainVector<1>(_v - rhs._v, _sigma2 + rhs._sigma2);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator-(value_t rhs) const
         {
             return UncertainVector<1>(_v - rhs, _sigma2);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator/(const UncertainVector<1> & rhs) const
         {
             double dfda = 1/rhs._v;
             double dfdb = -_v/(rhs._v * rhs._v);

             return UncertainVector<1>(_v/rhs._v, dfda * _sigma2 * dfda + dfdb * rhs._sigma2 * dfdb);
         }


         
         inline UncertainVector<1> UncertainVector<1>::operator/(value_t rhs) const
         {
             double dfda = 1/rhs;

             return UncertainVector<1>(_v/rhs, dfda * _sigma2 * dfda);

         }


         template<int DD>
         inline UncertainVector<DD> UncertainVector<1>::operator*(const UncertainVector<DD> & rhs) const
         {
             return rhs * (*this);
         }
         
         template<int DD>
         inline UncertainVector<DD> UncertainVector<1>::operator*(const Eigen::Matrix<double,DD,1> & rhs) const
         {
             return UncertainVector<DD>(rhs * _v, rhs * rhs.transpose() * _sigma2);
         }
         




     } // namespace kinematics
} // namespace sm
