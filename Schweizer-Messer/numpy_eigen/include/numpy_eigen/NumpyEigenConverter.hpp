/**
 * @file   NumpyEigenConverter.hpp
 * @author Paul Furgale <paul.furgale@utoronto.ca>
 * @date   Fri Feb  4 11:17:25 2011
 * 
 * @brief  Classes to support conversion from numpy arrays in Python
 *         to Eigen3 matrices in c++
 * 
 * 
 */

#ifndef NUMPY_EIGEN_CONVERTER_HPP
#define NUMPY_EIGEN_CONVERTER_HPP

#include <numpy_eigen/boost_python_headers.hpp>
//#include <iostream>


#define PY_ARRAY_UNIQUE_SYMBOL NP_Eigen_AS
#include <numpy/arrayobject.h> 

#include "type_traits.hpp"
#include <boost/lexical_cast.hpp>
#include "copy_routines.hpp"



/**
 * @class NumpyEigenConverter
 * @tparam the Eigen3 matrix type this class is specialized for
 * 
 * adapted from http://misspent.wordpress.com/2009/09/27/how-to-write-boost-python-converters/
 * General help available http://docs.scipy.org/doc/numpy/reference/c-api.array.html
 *
 * To use: 
 * 
 * #include <NumpyEigenConverter.hpp>
 * 
 * 
 * BOOST_PYTHON_MODULE(libmy_module_python)
 * {
 *   // The converters will cause a segfault unless import_array() is called before the first one
 *   import_array();
 *   NumpyEigenConverter<Eigen::Matrix< double, 1, 1 > >::register_converter();
 *   NumpyEigenConverter<Eigen::Matrix< double, 2, 1 > >::register_converter();
 * }
 * 
 */
template<typename EIGEN_MATRIX_T>
struct NumpyEigenConverter
{

  typedef EIGEN_MATRIX_T matrix_t;
  typedef typename matrix_t::Scalar scalar_t;

  enum {
    RowsAtCompileTime = matrix_t::RowsAtCompileTime,
    ColsAtCompileTime = matrix_t::ColsAtCompileTime,
    MaxRowsAtCompileTime = matrix_t::MaxRowsAtCompileTime,
    MaxColsAtCompileTime = matrix_t::MaxColsAtCompileTime,
    NpyType = TypeToNumPy<scalar_t>::NpyType,
    //Flags = ei_compute_matrix_flags<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>::ret,
    //CoeffReadCost = NumTraits<Scalar>::ReadCost,
    Options = matrix_t::Options
    //InnerStrideAtCompileTime = 1,
    //OuterStrideAtCompileTime = (Options&RowMajor) ? ColsAtCompileTime : RowsAtCompileTime
  };

  static std::string castSizeOption(int option)
  {
    if(option == Eigen::Dynamic)
      return "Dynamic";
    else
      return boost::lexical_cast<std::string>(option);
  }

  static std::string toString()
  {
    return std::string() + "Eigen::Matrix<" + TypeToNumPy<scalar_t>::typeString() + ", " +
      castSizeOption(RowsAtCompileTime) + ", " +
      castSizeOption(ColsAtCompileTime) + ", " +
      boost::lexical_cast<std::string>((int)Options) + ", " +
      castSizeOption(MaxRowsAtCompileTime) + ", " +
      castSizeOption(MaxColsAtCompileTime) + ">";
  }

  // The "Convert from C to Python" API
  static PyObject * convert(const matrix_t & M)
  {
    PyObject * P = NULL;
    if(RowsAtCompileTime == 1 || ColsAtCompileTime == 1)
      {
	// Create a 1D array
	npy_intp dimensions[1];
	dimensions[0] = M.size();
	P = PyArray_SimpleNew(1, dimensions, TypeToNumPy<scalar_t>::NpyType);	    	
	numpyTypeDemuxer< CopyEigenToNumpyVector<const matrix_t> >(&M,P);	
      }
    else
      {
	// create a 2D array.
	npy_intp dimensions[2];
	dimensions[0] = M.rows();
	dimensions[1] = M.cols();
	P = PyArray_SimpleNew(2, dimensions, TypeToNumPy<scalar_t>::NpyType);
	numpyTypeDemuxer< CopyEigenToNumpyMatrix<const matrix_t> >(&M,P);	
      }
    
    // incrementing the reference seems to cause a memory leak.
    // boost::python::incref(P);
    // This agrees with the sample code found here:
    // http://mail.python.org/pipermail/cplusplus-sig/2008-October/013825.html
    return P;
  }

  static bool isDimensionValid(int requestedSize, int sizeAtCompileTime, int maxSizeAtCompileTime)
  {
    bool valid = true;
    if(sizeAtCompileTime == Eigen::Dynamic)
      {
	// Check for dynamic fixed size
	// http://eigen.tuxfamily.org/dox-devel/TutorialMatrixClass.html#TutorialMatrixOptTemplParams
	if(!(maxSizeAtCompileTime == Eigen::Dynamic || requestedSize <= maxSizeAtCompileTime))
	  {
	    valid = false;
	  }
      }
    else if(sizeAtCompileTime != requestedSize)
      {
	valid = false;
      }
    return valid;
  }
      
  static void checkMatrixSizes(PyObject * obj_ptr)
  {
    int rows = PyArray_DIM(obj_ptr, 0);
    int cols = PyArray_DIM(obj_ptr, 1);

    bool rowsValid = isDimensionValid(rows, RowsAtCompileTime, MaxRowsAtCompileTime);
    bool colsValid = isDimensionValid(cols, ColsAtCompileTime, MaxColsAtCompileTime);
    if(!rowsValid || !colsValid)
    {
	THROW_TYPE_ERROR("Can not convert " << npyArrayTypeString(obj_ptr) << " to " << toString() 
			 << ". Mismatched sizes.");
      }
  }

  static void checkRowVectorSizes(PyObject * obj_ptr, int cols)
  {
    if(!isDimensionValid(cols, ColsAtCompileTime, MaxColsAtCompileTime))
      {
	THROW_TYPE_ERROR("Can not convert " << npyArrayTypeString(obj_ptr) << " to " << toString() 
			 << ". Mismatched sizes.");
      }
  }

  static void checkColumnVectorSizes(PyObject * obj_ptr, int rows)
  {
    // Check if the type can accomidate one column.
    if(ColsAtCompileTime == Eigen::Dynamic || ColsAtCompileTime == 1)
      {
	if(!isDimensionValid(rows, RowsAtCompileTime, MaxRowsAtCompileTime))
	  {
	    THROW_TYPE_ERROR("Can not convert " << npyArrayTypeString(obj_ptr) << " to " << toString() 
			     << ". Mismatched sizes.");
	  }
      }
    else
      {
	THROW_TYPE_ERROR("Can not convert " << npyArrayTypeString(obj_ptr) << " to " << toString() 
			 << ". Mismatched sizes.");
      }

  }

  static void checkVectorSizes(PyObject * obj_ptr)
  {
	int size = PyArray_DIM(obj_ptr, 0);

    // If the number of rows is fixed at 1, assume that is the sense of the vector.
    // Otherwise, assume it is a column.
    if(RowsAtCompileTime == 1)
      {
	checkRowVectorSizes(obj_ptr, size);
      }
    else
      {
	checkColumnVectorSizes(obj_ptr, size);
      }
  }

    
  static void* convertible(PyObject *obj_ptr)
  {
    // Check for a null pointer.
    if(!obj_ptr)
      {
        //THROW_TYPE_ERROR("PyObject pointer was null");
        return 0;
      }

    // Make sure this is a numpy array.
    if (!PyArray_Check(obj_ptr))
      {
        //THROW_TYPE_ERROR("Conversion is only defined for numpy array and matrix types");
        return 0;
      }

    // Check the type of the array.
    int npyType = PyArray_ObjectType(obj_ptr, 0);
    
    if(!TypeToNumPy<scalar_t>::canConvert(npyType))
      {
        //THROW_TYPE_ERROR("Can not convert " << npyArrayTypeString(obj_ptr) << " to " << toString() 
        //                 << ". Mismatched types.");
        return 0;
      }

    

    // Check the array dimensions.
    int nd = PyArray_NDIM(obj_ptr);
    
    if(nd != 1 && nd != 2)
      {
	THROW_TYPE_ERROR("Conversion is only valid for arrays with 1 or 2 dimensions. Argument has " << nd << " dimensions");
      }

    if(nd == 1)
      {
	checkVectorSizes(obj_ptr);
      }
    else 
      {
	// Two-dimensional matrix type.
	checkMatrixSizes(obj_ptr);
      }


    return obj_ptr;
  }
  

  static void construct(PyObject *obj_ptr, boost::python::converter::rvalue_from_python_stage1_data *data)
  {
    boost::python::converter::rvalue_from_python_storage<matrix_t> * matData = reinterpret_cast<boost::python::converter::rvalue_from_python_storage<matrix_t> * >(data);
    void* storage = matData->storage.bytes;
    
    // Make sure storage is 16byte aligned. With help from code from Memory.h
    void * aligned = reinterpret_cast<void*>((reinterpret_cast<size_t>(storage) & ~(size_t(15))) + 16);
    
    matrix_t * Mp = new (aligned) matrix_t();
    // Stash the memory chunk pointer for later use by boost.python
    // This signals boost::python that the new value must be deleted eventually
    data->convertible = storage;

    
    // std::cout << "Creating aligned pointer " << aligned << " from storage " << storage << std::endl;
    // std::cout << "matrix size: " << sizeof(matrix_t) << std::endl;
    // std::cout << "referent size: " << boost::python::detail::referent_size< matrix_t & >::value << std::endl;
    // std::cout << "sizeof(storage): " << sizeof(matData->storage) << std::endl;
    // std::cout << "sizeof(bytes): " << sizeof(matData->storage.bytes) << std::endl;
    
    

    matrix_t & M = *Mp;

    int nd = PyArray_NDIM(obj_ptr);
    if(nd == 1)
      {
	int size = PyArray_DIM(obj_ptr, 0);
	// This is a vector type
	if(RowsAtCompileTime == 1)
	  {
	    // Row Vector
	    M.resize(1,size);
	  }
	else
	  {
	    // Column Vector
	    M.resize(size,1);
	  }
	numpyTypeDemuxer< CopyNumpyToEigenVector<matrix_t> >(&M,obj_ptr);	
      }
    else
      {
	int rows = PyArray_DIM(obj_ptr, 0);
	int cols = PyArray_DIM(obj_ptr, 1);
	
	M.resize(rows,cols);
	numpyTypeDemuxer< CopyNumpyToEigenMatrix<matrix_t> >(&M,obj_ptr);	
      }

    


  }


  // The registration function.
  static void register_converter()
  {
    boost::python::to_python_converter<matrix_t,NumpyEigenConverter>();
    boost::python::converter::registry::push_back(
						  &NumpyEigenConverter::convertible,
						  &NumpyEigenConverter::construct,
						  boost::python::type_id<matrix_t>());

  }
  
};




#endif /* NUMPY_EIGEN_CONVERTER_HPP */
