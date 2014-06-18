#ifndef NUMPY_EIGEN_TYPE_TRAITS_HPP
#define NUMPY_EIGEN_TYPE_TRAITS_HPP

#define THROW_TYPE_ERROR(msg)						\
  {									\
    std::stringstream type_error_ss;					\
    type_error_ss << msg;						\
    PyErr_SetString(PyExc_TypeError, type_error_ss.str().c_str());	\
    throw boost::python::error_already_set();				\
  }


////////////////////////////////////////////////
// TypeToNumPy
// Defines helper functions based on the Eigen3 matrix type that
// decide what conversions can happen Eigen3 --> NumPy
// Also, converts a type to a NumPy enum.
template<typename Scalar> struct TypeToNumPy;

template<> struct TypeToNumPy<int>
{
  enum { NpyType = NPY_INT };
  static const char * npyString() { return "NPY_INT"; }
  static const char * typeString() { return "int"; }
  static bool canConvert(int type)
  {
    return type == NPY_INT || type == NPY_LONG;
  }
};

template<> struct TypeToNumPy<boost::int64_t>
{
  enum { NpyType = NPY_LONG };
  static const char * npyString() { return "NPY_LONG"; }
  static const char * typeString() { return "long"; }
  static bool canConvert(int type)
  {
    return type == NPY_INT || type == NPY_LONG;
  }
};

template<> struct TypeToNumPy<unsigned char>
{
  enum { NpyType = NPY_UBYTE };
  static const char * npyString() { return "NPY_UBYTE"; }
  static const char * typeString() { return "unsigned char"; }
  static bool canConvert(int type)
  {
    return type == NPY_UBYTE || type == NPY_BYTE || type == NPY_CHAR;
  }
};

template<> struct TypeToNumPy<char>
{
  enum { NpyType = NPY_BYTE };
  static const char * npyString() { return "NPY_BYTE"; }
  static const char * typeString() { return "char"; }
  static bool canConvert(int type)
  {
    return type == NPY_UBYTE || type == NPY_BYTE || type == NPY_CHAR;
  }
};


template<> struct TypeToNumPy<float>
{
  enum { NpyType = NPY_FLOAT };
  static const char * npyString() { return "NPY_FLOAT"; }
  static const char * typeString() { return "float"; }
  static bool canConvert(int type)
  {
    return type == NPY_INT || type == NPY_FLOAT || type == NPY_LONG;
  }
};

template<> struct TypeToNumPy<double>
{
  enum { NpyType = NPY_DOUBLE };
  static const char * npyString() { return "NPY_DOUBLE"; }
  static const char * typeString() { return "double"; }
  static bool canConvert(int type)
  {
    return type == NPY_INT || type == NPY_FLOAT || type == NPY_DOUBLE || type == NPY_LONG;
  }
};



inline const char * npyTypeToString(int npyType)
{
  switch(npyType)
    {
    case NPY_BOOL:
      return "NPY_BOOL";
    case NPY_BYTE:
      return "NPY_BYTE"; 
    case NPY_UBYTE:
      return "NPY_UBYTE";
    case NPY_SHORT:
      return "NPY_SHORT"; 
    case NPY_USHORT:
      return "NPY_USHORT";
    case NPY_INT:
      return "NPY_INT"; 
    case NPY_UINT:
      return "NPY_UINT";
    case NPY_LONG:
      return "NPY_LONG"; 
    case NPY_ULONG:
      return "NPY_ULONG";
    case NPY_LONGLONG:
      return "NPY_LONGLONG"; 
    case NPY_ULONGLONG:
      return "NPY_ULONGLONG";
    case NPY_FLOAT:
      return "NPY_FLOAT"; 
    case NPY_DOUBLE:
      return "NPY_DOUBLE"; 
    case NPY_LONGDOUBLE:
      return "NPY_LONGDOUBLE";
    case NPY_CFLOAT:
      return "NPY_CFLOAT"; 
    case NPY_CDOUBLE:
      return "NPY_CDOUBLE"; 
    case NPY_CLONGDOUBLE:
      return "NPY_CLONGDOUBLE";
    case NPY_OBJECT:
      return "NPY_OBJECT";
    case NPY_STRING:
      return "NPY_STRING"; 
    case NPY_UNICODE:
      return "NPY_UNICODE";
    case NPY_VOID:
      return "NPY_VOID";
    case NPY_NTYPES:
      return "NPY_NTYPES";
    case NPY_NOTYPE:
      return "NPY_NOTYPE";
    case NPY_CHAR:
      return "NPY_CHAR";
    default:
      return "Unknown type";
    }
}

inline std::string npyArrayTypeString(PyObject * obj_ptr)
{
  std::stringstream ss;
  int nd = PyArray_NDIM(obj_ptr);
  ss << "numpy.array<" << npyTypeToString(PyArray_ObjectType(obj_ptr, 0)) << ">[";
  if(nd > 0)
    {
      ss << PyArray_DIM(obj_ptr, 0);
      for(int i = 1; i < nd; i++)
	{
	  ss << ", " << PyArray_DIM(obj_ptr, i);
	}
    }
  ss << "]";
  return ss.str();
}


#endif /* NUMPY_EIGEN_TYPE_TRAITS_HPP */
