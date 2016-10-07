#include <aslam/backend/MapTransformation.hpp>

namespace aslam {

    namespace backend {

        TransformationExpression transformationToExpression( sm::kinematics::Transformation & T,
                                                             boost::shared_ptr<MappedRotationQuaternion> & outQ,
                                                             boost::shared_ptr<MappedEuclideanPoint> & outT) {
        
            outQ.reset( new MappedRotationQuaternion(T.qptr()) );
            outT.reset( new MappedEuclideanPoint(T.tptr()) );

            return TransformationExpression( RotationExpression(outQ), EuclideanExpression(outT) );

        }

    
        TransformationExpression transformationToExpression( sm::kinematics::Transformation & T ) {
        
            boost::shared_ptr< MappedRotationQuaternion > q;
            boost::shared_ptr< MappedEuclideanPoint > t;

            return transformationToExpression( T, q, t );

        }


    } // namespace backend    
} // namespace aslam
