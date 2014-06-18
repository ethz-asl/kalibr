#ifndef NUMPY_EIGEN_COPY_ROUTINES_HPP
#define NUMPY_EIGEN_COPY_ROUTINES_HPP


template<typename EIGEN_T>
struct CopyNumpyToEigenMatrix
{
  typedef EIGEN_T matrix_t;
  typedef typename matrix_t::Scalar scalar_t;
  
  template<typename T>
  void exec(EIGEN_T * M_, PyObject * P_)
  {
    // Assumes M is already initialized.
    for(int r = 0; r < M_->rows(); r++)
      {
	for(int c = 0; c < M_->cols(); c++)
	  {
	    T * p = static_cast<T*>(PyArray_GETPTR2(P_, r, c));
	    (*M_)(r,c) = static_cast<scalar_t>(*p);
	  }
      }
  }

};

template<typename EIGEN_T>
struct CopyEigenToNumpyMatrix
{
  typedef EIGEN_T matrix_t;
  typedef typename matrix_t::Scalar scalar_t;
  
  template<typename T>
  void exec(EIGEN_T * M_, PyObject * P_)
  {
    // Assumes M is already initialized.
    for(int r = 0; r < M_->rows(); r++)
      {
	for(int c = 0; c < M_->cols(); c++)
	  {
	    T * p = static_cast<T*>(PyArray_GETPTR2(P_, r, c));
	    *p = static_cast<T>((*M_)(r,c));
	  }
      }
  }

};

template<typename EIGEN_T>
struct CopyEigenToNumpyVector
{
  typedef EIGEN_T matrix_t;
  typedef typename matrix_t::Scalar scalar_t;
  
  template<typename T>
  void exec(EIGEN_T * M_, PyObject * P_)
  {
    // Assumes M is already initialized.
    for(int i = 0; i < M_->size(); i++)
      {
	T * p = static_cast<T*>(PyArray_GETPTR1(P_, i));
	*p = static_cast<T>((*M_)(i));
      }
  }

};


template<typename EIGEN_T>
struct CopyNumpyToEigenVector
{
  typedef EIGEN_T matrix_t;
  typedef typename matrix_t::Scalar scalar_t;
  
  template<typename T>
  void exec(EIGEN_T * M_, PyObject * P_)
  {
    // Assumes M is already initialized.
    for(int i = 0; i < M_->size(); i++)
      {
	T * p = static_cast<T*>(PyArray_GETPTR1(P_, i));
	(*M_)(i) = static_cast<scalar_t>(*p);
      }
  }

};




// Crazy syntax in this function was found here:
// http://stackoverflow.com/questions/1840253/c-template-member-function-of-template-class-called-from-template-function/1840318#1840318
template< typename FUNCTOR_T>
inline void numpyTypeDemuxer(typename FUNCTOR_T::matrix_t * M, PyObject * P)
{
  FUNCTOR_T f;
  int npyType = PyArray_ObjectType(P, 0);
  switch(npyType)
    {
    case NPY_BOOL:
      f.template exec<bool>(M,P);
      break;
    case NPY_BYTE:
      f.template exec<char>(M,P);
      break;
    case NPY_UBYTE:
      f.template exec<unsigned char>(M,P);
      break;
    case NPY_SHORT:
      f.template exec<short>(M,P);
      break;
    case NPY_USHORT:
      f.template exec<unsigned short>(M,P);
      break;
    case NPY_INT:
      f.template exec<int>(M,P);
      break;
    case NPY_UINT:
      f.template exec<unsigned int>(M,P);
      break;
    case NPY_LONG:
      f.template exec<long>(M,P);
      break;
    case NPY_ULONG:
      f.template exec<unsigned long>(M,P);
      break;
    case NPY_LONGLONG:
      f.template exec<long long>(M,P);
      break;
    case NPY_ULONGLONG:
      f.template exec<unsigned long long>(M,P);
      break;
    case NPY_FLOAT:
      f.template exec<float>(M,P);
      break;
    case NPY_DOUBLE:
      f.template exec<double>(M,P);
      break;
    case NPY_LONGDOUBLE:
      f.template exec<long double>(M,P);
      break;
     default:
       THROW_TYPE_ERROR("Unsupported type: " << npyTypeToString(npyType));
   }
}


#endif /* NUMPY_EIGEN_COPY_ROUTINES_HPP */
