#ifndef SM_EIGEN_TRAITS
#define SM_EIGEN_TRAITS

#include <Eigen/StdVector>
#include <map>

namespace sm {
    namespace eigen {
        
        template<typename T>
        struct Traits
        {
            typedef std::vector< T, Eigen::aligned_allocator<T> > vector_t;
        };

        template<typename KEY_T, typename VALUE_T>
        struct MapTraits
        {

            typedef std::map<KEY_T, VALUE_T, std::less<KEY_T>,
                             Eigen::aligned_allocator<std::pair<const KEY_T, VALUE_T> > > map_t;

        };

    } // namespace eigen
} // namespace sm


#endif /* SM_EIGEN_TRAITS */
