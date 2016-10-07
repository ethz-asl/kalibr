#ifndef ASLAM_BACKEND_MAP_TRANSFORMATION_HPP
#define ASLAM_BACKEND_MAP_TRANSFORMATION_HPP

#include <sm/kinematics/Transformation.hpp>
#include <aslam/backend/MappedRotationQuaternion.hpp>
#include <aslam/backend/MappedEuclideanPoint.hpp>


namespace aslam {
    namespace backend {

        TransformationExpression transformationToExpression( sm::kinematics::Transformation & T,
                                                             boost::shared_ptr<MappedRotationQuaternion> & outQ,
                                                             boost::shared_ptr<MappedEuclideanPoint> & outT);

        
        /// \brief Convert a Tranformation to an optimizable transformation expression
        ///
        /// The created design variables will be stored in the expression. You can
        /// get them with getDesignVariables()
        TransformationExpression transformationToExpression( sm::kinematics::Transformation & T );
    
    } // namespace backend
} // namespace aslam




#endif /* ASLAM_BACKEND_MAP_TRANSFORMATION_HPP */
