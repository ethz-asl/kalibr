#ifndef KALIBR_IMU_CAM_LASER_ERROR_HPP
#define KALIBR_IMU_CAM_LASER_ERROR_HPP

#include<aslam/backend/ErrorTerm.hpp>
#include<aslam/backend/EuclideanExpression.hpp>
#include<aslam/backend/RotationExpression.hpp>
#include<aslam/backend/ScalarExpression.hpp>
#include<aslam/backend/TransformationExpression.hpp>

namespace laser_errorterms {

    class ScalarError : public aslam::backend::ErrorTermFs<1>
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
        ScalarError(const double & measurement, const Eigen::Matrix<double,1,1> & invR, 
            const aslam::backend::ScalarExpression & predictedMeasurement);
        ~ScalarError();


    protected:
        /// \brief evaluate the error term and return the weighted squared error e^T invR e
        virtual double evaluateErrorImplementation();

        /// \brief evaluate the jacobian
        virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians);
    private:
        Eigen::Matrix<double,1,1> _evaluatedErrorTerm;
        double _measurement;

        aslam::backend::ScalarExpression _predictedMeasurement;
    };


    class LaserError : public aslam::backend::ErrorTermFs<1>
    {
    public: 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
        LaserError(const double & measurement, const Eigen::Matrix<double,1,1> & invR, const Eigen::Vector3d & p, const aslam::backend::TransformationExpression & rt, const aslam::backend::EuclideanExpression & normal, const aslam::backend::ScalarExpression & offset, const aslam::backend::ScalarExpression & bias);
        ~LaserError();


    protected:
        /// \brief evaluate the error term and return the weighted squared error e^T invR e
        virtual double evaluateErrorImplementation();

        /// \brief evaluate the jacobian
        virtual void evaluateJacobiansImplementation(aslam::backend::JacobianContainer & _jacobians);
    private:
        Eigen::Matrix<double,1,1> _evaluatedErrorTerm;
        double _measurement;
        double _angle;
        const Eigen::Vector3d _p;

        aslam::backend::TransformationExpression _rt;
        aslam::backend::EuclideanExpression _normal;
        aslam::backend::ScalarExpression _offset;
        aslam::backend::ScalarExpression _bias;
        aslam::backend::ScalarExpression _predictedMeasurement;
    };
}


#endif /* KALIBR_IMU_CAM_LASER_ERROR_HPP */
