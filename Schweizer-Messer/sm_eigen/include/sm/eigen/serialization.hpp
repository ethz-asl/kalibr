///
/// @file   EigenSerialization.hpp
/// @author Paul Furgale <paul.furgale@utoronto.ca>
/// @date   Tue Oct 26 12:57:31 2010
/// 
/// @brief  Functions to serialize dense Eigen matrices.
/// 
/// 
///


#ifndef SM_EIGEN_SERIALIZATION_HPP
#define SM_EIGEN_SERIALIZATION_HPP

#include <Eigen/Core>
#include <sm/assert_macros.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/nvp.hpp>
#include <boost/serialization/array.hpp>

namespace sm { namespace eigen {
    // An exception for errors during serialization/deserialization
    SM_DEFINE_EXCEPTION( SerializationException, std::runtime_error );
  }} // namespace sm::eigen

namespace boost { namespace serialization {


    /// 
    /// boost::serialization helper function for the Eigen3 Matrix object.
    /// The matrix is a base class for dense matrices.
    /// http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html
    ///
    /// This function should not be called directly but will be called
    /// when serializing a matrix.
    ///
    ///
    /// @param ar  The boost::serialization archive
    /// @param M   The matrix to serialize
    /// @param file_version The file version to be serialized.
    ///  
    template<class Archive, class Scalar, int A, int B, int C, int D, int E>
    inline void save( Archive & ar, 
                      const Eigen::Matrix<Scalar,A,B,C,D,E> & M, 
                      const unsigned int /* file_version */)
    {
      typedef typename Eigen::Matrix<Scalar,A,B,C,D,E>::Index index_t;
      index_t rows = M.rows();
      index_t cols = M.cols();
      ar << BOOST_SERIALIZATION_NVP(rows);
      ar << BOOST_SERIALIZATION_NVP(cols);
      
      if(rows > 0 && cols > 0)
      {
          ar << make_nvp("data", make_array(&M(0,0),M.rows()*M.cols()));
      }
    }


    /// 
    /// Loads an Eigen matrix object from a boost::serialization archive.
    /// This function has several specializations to handle Matrices with
    /// fixed sizes and those with dynamic sizes. This function is called
    /// by the boost serialization code and should not be called directly.
    ///
    /// This specialization handles fixed sized matrices
    ///
    /// @param ar The boost::serialization archive
    /// @param M  The matrix to be loaded
    /// @param file_version The archive version number.
    ///
    template<class Archive, class Scalar, int A, int B, int C, int D, int E>
    inline void load( Archive & ar, 
                      Eigen::Matrix<Scalar,A,B,C,D,E> & M,
                      const unsigned int /* file_version */)
    {
      typedef typename Eigen::Matrix<Scalar,A,B,C,D,E>::Index index_t;
      index_t rows, cols;
      ar >> BOOST_SERIALIZATION_NVP(rows);
      ar >> BOOST_SERIALIZATION_NVP(cols);
      SM_ASSERT_EQ(sm::eigen::SerializationException,rows,A,"Unexpected number of rows found for fixed-sized type");
      SM_ASSERT_EQ(sm::eigen::SerializationException,cols,B,"Unexpected number of columns found for fixed-sized type");

      if(rows > 0 && cols > 0)
	{
	  ar >> make_nvp("data", make_array(&M(0,0),M.rows()*M.cols()));
	}
    }


    /// 
    /// Loads an Eigen matrix object from a boost::serialization archive.
    /// This function has several specializations to handle Matrices with
    /// fixed sizes and those with dynamic sizes. This function is called
    /// by the boost serialization code and should not be called directly.
    ///
    /// This specialization handles dynamically-sized sized column vectors
    ///
    /// @param ar The boost::serialization archive
    /// @param M  The matrix to be loaded
    /// @param file_version The archive version number.
    ///
    template<class Archive, class Scalar, int B, int C, int D, int E>
    inline void load( Archive & ar, 
                      Eigen::Matrix<Scalar,Eigen::Dynamic,B,C,D,E> & M,
                      const unsigned int /* file_version */)
    {
        typedef typename Eigen::Matrix<Scalar,Eigen::Dynamic,B,C,D,E>::Index index_t;
      index_t rows, cols;
      ar >> BOOST_SERIALIZATION_NVP(rows);
      ar >> BOOST_SERIALIZATION_NVP(cols);
      SM_ASSERT_EQ(sm::eigen::SerializationException,cols,B,"Unexpected number of columns found for fixed-sized type");
      
      M.resize(rows,Eigen::NoChange);
      if(rows > 0 && cols > 0)
	{
	  ar >> make_nvp("data",make_array(&M(0,0),M.rows()*M.cols()));
	}
    }

    /// 
    /// Loads an Eigen matrix object from a boost::serialization archive.
    /// This function has several specializations to handle Matrices with
    /// fixed sizes and those with dynamic sizes. This function is called
    /// by the boost serialization code and should not be called directly.
    ///
    /// This specialization handles dynamically sized row vectors
    ///
    /// @param ar The boost::serialization archive
    /// @param M  The matrix to be loaded
    /// @param file_version The archive version number.
    ///
    template<class Archive, class Scalar, int A, int C, int D, int E>
    inline void load( Archive & ar, 
                      Eigen::Matrix<Scalar,A,Eigen::Dynamic,C,D,E> & M,
                      const unsigned int /* file_version */)
    {
      typedef typename Eigen::Matrix<Scalar,A,Eigen::Dynamic,C,D,E>::Index index_t;
      index_t rows, cols;
      ar >> BOOST_SERIALIZATION_NVP(rows);
      ar >> BOOST_SERIALIZATION_NVP(cols);
      SM_ASSERT_EQ(sm::eigen::SerializationException,rows,A,"Unexpected number of rows found for fixed-sized type");
    
      M.resize(Eigen::NoChange,cols);
      ar >> make_nvp("data",make_array(&M(0,0),M.rows()*M.cols()));

    }

    /// 
    /// Loads an Eigen matrix object from a boost::serialization archive.
    /// This function has several specializations to handle Matrices with
    /// fixed sizes and those with dynamic sizes. This function is called
    /// by the boost serialization code and should not be called directly.
    ///
    /// This specialization handles dynamically sized matrices
    ///
    /// @param ar The boost::serialization archive
    /// @param M  The matrix to be loaded
    /// @param file_version The archive version number.
    ///
    template<class Archive, class Scalar, int C, int D, int E>
    inline void load( Archive & ar, 
                      Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,C,D,E> & M,
                      const unsigned int /* file_version */)
    {
      typedef typename Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic,C,D,E>::Index index_t;
      index_t rows, cols;
      ar >> BOOST_SERIALIZATION_NVP(rows);
      ar >> BOOST_SERIALIZATION_NVP(cols);    
      
      M.resize(rows,cols);
      if(rows > 0 && cols > 0)
	{
	  ar >> make_nvp("data", make_array(&M(0,0),M.rows()*M.cols()));
	}
    }


    /// 
    /// The function that causes boost::serialization to look for
    /// seperate save() and load() functions when serializing
    /// and Eigen matrix.
    ///
    /// @param ar The boost::serialization archive
    /// @param M  The matrix to be serialized
    /// @param file_version The archive version number.
    ///
    template<class Archive, class Scalar, int A, int B, int C, int D, int E>
    inline void serialize(
        Archive & ar,
        Eigen::Matrix<Scalar,A,B,C,D,E> & M,
        const unsigned int file_version
                          ){
      boost::serialization::split_free(ar, M, file_version);
    }


    /// 
    /// A function that serialized an Eigen Transform by causing the underlying
    /// Eigen matrix to be serialized.
    ///
    /// @param ar The boost::serialization archive
    /// @param M  The transform to be serialized
    /// @param file_version The archive version number. 
    ///
    template<class Archive, class Scalar, int Dim, int Mode>
    inline void serialize( Archive & ar, 
                           Eigen::Transform<Scalar, Dim, Mode> & M, 
                           const unsigned int /* file_version */ )
     {
       
       ar & boost::serialization::make_nvp("matrix",M.matrix());
     }


}}

#endif // SM_EIGEN_SERIALIZATION_HPP
