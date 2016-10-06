#ifndef SM_PYTHON_STL_CONVERTERS_HPP
#define SM_PYTHON_STL_CONVERTERS_HPP

#include<boost/python.hpp>

namespace sm {
    namespace python {
        
        /// \brief Convert an STL list to a boost::python list 
        template<typename LIST_ITERATOR_T>
        inline void stlToList(const LIST_ITERATOR_T & begin, const LIST_ITERATOR_T & end, boost::python::list & outlist)
        {
            LIST_ITERATOR_T it = begin;
            
            for( ; it != end; ++it)
            {
                outlist.append(*it);
            }

            
        } 

        template<typename MAP_ITERATOR_T>
        inline void stlToDict(const MAP_ITERATOR_T & begin, const MAP_ITERATOR_T & end, boost::python::dict & out)
        {
            MAP_ITERATOR_T it = begin;
            for( ; it != end; ++it)
            {
                out[it->first] = it->second;
            }
        }

    } // namespace python
} // namespace sm

#endif // SM_PYTHON_STL_CONVERTERS_HPP
