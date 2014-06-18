#ifndef SM_EIGEN_PROPERTY_TREE_HPP
#define SM_EIGEN_PROPERTY_TREE_HPP

#include <sm/PropertyTree.hpp>
#include <Eigen/Core>

namespace sm {
    namespace eigen {
        
        inline Eigen::Vector3d vector3FromPropertyTree(const sm::PropertyTree & config)
        {
            Eigen::Vector3d rval;
            rval[0] = config.getDouble("x");
            rval[1] = config.getDouble("y");
            rval[2] = config.getDouble("z");

            return rval;
        }


        inline Eigen::Vector4d quaternionFromPropertyTree(const sm::PropertyTree & config)
        {
            Eigen::Vector4d rval;
            rval[0] = config.getDouble("x");
            rval[1] = config.getDouble("y");
            rval[2] = config.getDouble("z");
            rval[3] = config.getDouble("w");
            return rval;        
        }
        

    } // namespace eigen
} // namespace sm


#endif /* SM_EIGEN_PROPERTY_TREE_HPP */
