#include <bsplines/BSplinePose.hpp>
#include <sm/assert_macros.hpp>
// boost::tie
#include <boost/tuple/tuple.hpp>
#include <sm/kinematics/transformations.hpp>

namespace bsplines {
  using namespace sm::kinematics;

    BSplinePose::BSplinePose(int splineOrder, const RotationalKinematics::Ptr & rotationalKinematics) 
      : BSpline(splineOrder), rotation_(rotationalKinematics)
    {
      
    }

    BSplinePose::~BSplinePose()
    {

    }
      
    Eigen::Matrix4d BSplinePose::transformation(double tk) const
    {
      return curveValueToTransformation(eval(tk));
    }

    Eigen::Matrix4d BSplinePose::transformationAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      Eigen::MatrixXd JS;
      Eigen::VectorXd p;
      p = evalDAndJacobian(tk,0,&JS, coefficientIndices);
      
      Eigen::MatrixXd JT;
      Eigen::Matrix4d T = curveValueToTransformationAndJacobian( p, &JT );      
      
      if(J)
	{
	  *J = JT * JS;
	}

      return T;
    }

    Eigen::Matrix3d BSplinePose::orientationAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      Eigen::Matrix3d C;
      Eigen::MatrixXd JS;
      Eigen::VectorXd p;
      p = evalDAndJacobian(tk,0,&JS, coefficientIndices);

      Eigen::Matrix3d S;
      C = rotation_->parametersToRotationMatrix(p.tail<3>(), &S);

      Eigen::MatrixXd JO = Eigen::MatrixXd::Zero(3,6);
      JO.block(0,3,3,3) = S;
      if(J)
	{
	  *J = JO * JS;
	}

      return C;

    }

    Eigen::Matrix3d BSplinePose::inverseOrientationAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      Eigen::Matrix3d C;
      Eigen::MatrixXd JS;
      Eigen::VectorXd p;
      p = evalDAndJacobian(tk,0,&JS, coefficientIndices);

      Eigen::Matrix3d S;
      C = rotation_->parametersToRotationMatrix(p.tail<3>(), &S).transpose();

      Eigen::MatrixXd JO = Eigen::MatrixXd::Zero(3,6);
      JO.block(0,3,3,3) = S;
      if(J)
	{
	  *J = -C * JO * JS;
	}

      return C;

    }


    Eigen::Matrix4d BSplinePose::inverseTransformationAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      //std::cout << __FUNCTION__ << "()\n";
      //ASRL_THROW(std::runtime_error,"Not Implemented");
      Eigen::MatrixXd JS;
      Eigen::VectorXd p;
      p = evalDAndJacobian(tk,0,&JS, coefficientIndices);
      
      Eigen::MatrixXd JT;
      Eigen::Matrix4d T = curveValueToTransformationAndJacobian( p, &JT );   
      // Invert the transformation.
      T.topLeftCorner<3,3>().transposeInPlace();
      T.topRightCorner<3,1>() = (-T.topLeftCorner<3,3>() * T.topRightCorner<3,1>()).eval();      

      if(J)
	{
	  // The "box times" is the linearized transformation way of inverting the jacobian.
	  *J = -sm::kinematics::boxTimes(T) * JT * JS;
	}

      if(coefficientIndices)
	{
	  *coefficientIndices = localCoefficientVectorIndices(tk);
	}
      
      return T;
    }
    
    Eigen::Matrix4d BSplinePose::inverseTransformation(double tk) const
    {
      Eigen::Matrix4d T = curveValueToTransformation(eval(tk));
      T.topLeftCorner<3,3>().transposeInPlace();
      T.topRightCorner<3,1>() = (-T.topLeftCorner<3,3>() * T.topRightCorner<3,1>()).eval();
      return T;
    }



    Eigen::Vector4d BSplinePose::transformVectorAndJacobian(double tk, const Eigen::Vector4d & v_tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      Eigen::MatrixXd JT;
      Eigen::Matrix4d T_n_vk = transformationAndJacobian(tk, &JT, coefficientIndices);
      Eigen::Vector4d v_n = T_n_vk * v_tk;
      
      if(J)
	{
	  *J = sm::kinematics::boxMinus(v_n) * JT;
	}

      return v_n;
    }

      
    Eigen::Vector3d BSplinePose::position(double tk) const
    {
      return eval(tk).head<3>();
    }



    Eigen::Matrix3d BSplinePose::orientation(double tk) const
    {
      return rotation_->parametersToRotationMatrix(eval(tk).tail<3>());
    }

    Eigen::Matrix3d BSplinePose::inverseOrientation(double tk) const
    {
      return rotation_->parametersToRotationMatrix(eval(tk).tail<3>()).transpose();
    }



    Eigen::Vector3d BSplinePose::linearVelocity(double tk) const
    {
      return evalD(tk,1).head<3>();
    }

    Eigen::Vector3d BSplinePose::linearVelocityBodyFrame(double tk) const
    {
      Eigen::VectorXd r = evalD(tk, 0);
      Eigen::Matrix3d C_wb = rotation_->parametersToRotationMatrix(r.tail<3>());
      return C_wb.transpose() * evalD(tk, 1).head<3>();
    }

    Eigen::Vector3d BSplinePose::linearAcceleration(double tk) const
    {
      return evalD(tk,2).head<3>();
    }

    Eigen::Vector3d BSplinePose::linearAccelerationBodyFrame(double tk) const
    {
      Eigen::VectorXd r = evalD(tk, 0);
      Eigen::Matrix3d C_wb = rotation_->parametersToRotationMatrix(r.tail<3>());
      return C_wb.transpose() * evalD(tk, 2).head<3>();
    }

    Eigen::Vector3d BSplinePose::linearAccelerationAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      
      Eigen::Vector3d a = evalDAndJacobian(tk,2,J,coefficientIndices).head<3>();
      if(J)
	{
	  J->conservativeResize(3,J->cols());
	}
      return a;
    }
    

    // \omega_w_{b,w} (angular velocity of the body frame as seen from the world frame, expressed in the world frame)
    Eigen::Vector3d BSplinePose::angularVelocity(double tk) const
    {
      Eigen::Vector3d omega;
      Eigen::VectorXd r = evalD(tk,0);
      Eigen::VectorXd v = evalD(tk,1);

      // \omega = S(\bar \theta) \dot \theta
      omega = -rotation_->parametersToSMatrix(r.tail<3>()) * v.tail<3>();
      return omega;
    }

    // \omega_b_{w,b} (angular velocity of the world frame as seen from the body frame, expressed in the body frame)
    Eigen::Vector3d BSplinePose::angularVelocityBodyFrame(double tk) const
    {
      Eigen::Vector3d omega;
      Eigen::VectorXd r = evalD(tk,0);
      Eigen::VectorXd v = evalD(tk,1);
      Eigen::Matrix3d S;
      Eigen::Matrix3d C_w_b = rotation_->parametersToRotationMatrix(r.tail<3>(), &S);

      // \omega = S(\bar \theta) \dot \theta
      omega = -C_w_b.transpose() * S * v.tail<3>();
      return omega;      

    }

    // \omega_b_{w,b} (angular velocity of the world frame as seen from the body frame, expressed in the body frame)
    Eigen::Vector3d BSplinePose::angularVelocityBodyFrameAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
      Eigen::Vector3d omega;
      Eigen::Vector3d p;
      Eigen::Vector3d pdot;
      Eigen::MatrixXd Jp;
      Eigen::MatrixXd Jpdot;
      p = evalDAndJacobian(tk,0,&Jp,NULL).tail<3>();
      pdot = evalDAndJacobian(tk,1,&Jpdot,coefficientIndices).tail<3>();

      Eigen::MatrixXd Jr;
      Eigen::Matrix3d C_w_b = inverseOrientationAndJacobian(tk,&Jr,NULL);
      
      // Rearrange the spline jacobian matrices. Now Jpdot is the 
      // jacobian of p wrt the spline coefficients stacked on top
      // of the jacobian of pdot wrt the spline coefficients.
      Jpdot.block(0,0,3,Jpdot.cols()) = Jp.block(3,0,3,Jp.cols());
      
      //std::cout << "Jpdot\n" << Jpdot << std::endl;

      Eigen::Matrix<double,3,6> Jo;
      omega = -C_w_b * rotation_->angularVelocityAndJacobian(p,pdot,&Jo);
      Jo = (-C_w_b * Jo).eval();
      //std::cout << "Jo:\n" << Jo << std::endl;
      if(J)
	{
	  *J = Jo * Jpdot + sm::kinematics::crossMx(omega) * Jr;
	}

      return omega;

    }


    // \omega_w_{b,w} (angular velocity of the body frame as seen from the world frame, expressed in the world frame)
    Eigen::Vector3d BSplinePose::angularVelocityAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {

      Eigen::Vector3d omega;
      Eigen::Vector3d p;
      Eigen::Vector3d pdot;
      Eigen::MatrixXd Jp;
      Eigen::MatrixXd Jpdot;
      p = evalDAndJacobian(tk,0,&Jp,NULL).tail<3>();
      pdot = evalDAndJacobian(tk,1,&Jpdot,coefficientIndices).tail<3>();
      
      // Rearrange the spline jacobian matrices. Now Jpdot is the 
      // jacobian of p wrt the spline coefficients stacked on top
      // of the jacobian of pdot wrt the spline coefficients.
      Jpdot.block(0,0,3,Jpdot.cols()) = Jp.block(3,0,3,Jp.cols());
      
      //std::cout << "Jpdot\n" << Jpdot << std::endl;

      Eigen::Matrix<double,3,6> Jo;
      omega = rotation_->angularVelocityAndJacobian(p,pdot,&Jo);
      
      //std::cout << "Jo:\n" << Jo << std::endl;
      if(J)
	{
	  *J = Jo * Jpdot;
	}

      return omega;
    }

    // \omega_dot_b_{w,b} (angular acceleration of the world frame as seen from the body frame, expressed in the body frame)
    Eigen::Vector3d BSplinePose::angularAccelerationBodyFrame(double tk) const
    {
    	Eigen::Vector3d omega;
    	Eigen::VectorXd r = evalD(tk,0);
    	Eigen::VectorXd v = evalD(tk,2);
    	Eigen::Matrix3d S;
    	Eigen::Matrix3d C_w_b = rotation_->parametersToRotationMatrix(r.tail<3>(), &S);

    	// \omega = S(\bar \theta) \dot \theta
    	omega = -C_w_b.transpose() * S * v.tail<3>();
    	return omega;

    }

    // \omega_dot_b_{w,b} (angular acceleration of the world frame as seen from the body frame, expressed in the body frame)
    Eigen::Vector3d BSplinePose::angularAccelerationBodyFrameAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {
    	Eigen::Vector3d omega;
    	Eigen::Vector3d p;
    	Eigen::Vector3d pdot;
    	Eigen::MatrixXd Jp;
    	Eigen::MatrixXd Jpdot;
    	p = evalDAndJacobian(tk,0,&Jp,NULL).tail<3>();
    	pdot = evalDAndJacobian(tk,2,&Jpdot,coefficientIndices).tail<3>();

    	Eigen::MatrixXd Jr;
    	Eigen::Matrix3d C_w_b = inverseOrientationAndJacobian(tk,&Jr,NULL);

    	// Rearrange the spline jacobian matrices. Now Jpdot is the
    	// jacobian of p wrt the spline coefficients stacked on top
    	// of the jacobian of pdot wrt the spline coefficients.
    	Jpdot.block(0,0,3,Jpdot.cols()) = Jp.block(3,0,3,Jp.cols());

    	Eigen::Matrix<double,3,6> Jo;
    	omega = -C_w_b * rotation_->angularVelocityAndJacobian(p,pdot,&Jo);
    	Jo = (-C_w_b * Jo).eval();
    	if(J)
    	{
    		*J = Jo * Jpdot + sm::kinematics::crossMx(omega) * Jr;
    	}

    	return omega;

    }


    // \omega_dot_w_{b,w} (angular acceleration of the body frame as seen from the world frame, expressed in the world frame)
    Eigen::Vector3d BSplinePose::angularAccelerationAndJacobian(double tk, Eigen::MatrixXd * J, Eigen::VectorXi * coefficientIndices) const
    {

    	Eigen::Vector3d omega;
    	Eigen::Vector3d p;
    	Eigen::Vector3d pdot;
    	Eigen::MatrixXd Jp;
    	Eigen::MatrixXd Jpdot;
    	p = evalDAndJacobian(tk,0,&Jp,NULL).tail<3>();
    	pdot = evalDAndJacobian(tk,2,&Jpdot,coefficientIndices).tail<3>();

    	// Rearrange the spline jacobian matrices. Now Jpdot is the
    	// jacobian of p wrt the spline coefficients stacked on top
    	// of the jacobian of pdot wrt the spline coefficients.
    	Jpdot.block(0,0,3,Jpdot.cols()) = Jp.block(3,0,3,Jp.cols());

    	Eigen::Matrix<double,3,6> Jo;
    	omega = rotation_->angularVelocityAndJacobian(p,pdot,&Jo);
    	if(J)
    	{
    		*J = Jo * Jpdot;
    	}

    	return omega;
    }

    void BSplinePose::initPoseSpline(double t0, double t1, const Eigen::Matrix4d & T_n_t0, const Eigen::Matrix4d & T_n_t1)
    {
      Eigen::VectorXd v0 = transformationToCurveValue(T_n_t0);
      Eigen::VectorXd v1 = transformationToCurveValue(T_n_t1);
      
      initSpline(t0,t1,v0,v1);
    }
    
    void BSplinePose::addPoseSegment(double tk, const Eigen::Matrix4d & T_n_tk)
    {
      Eigen::VectorXd vk = transformationToCurveValue(T_n_tk);
      
      addCurveSegment(tk, vk);
    }

    void BSplinePose::addPoseSegment2(double tk, const Eigen::Matrix4d & T_n_tk, double lambda)
    {
      Eigen::VectorXd vk = transformationToCurveValue(T_n_tk);
      
      addCurveSegment2(tk, vk, lambda);
    }


    Eigen::Matrix4d BSplinePose::curveValueToTransformation( const Eigen::VectorXd & c ) const
    {
      SM_ASSERT_EQ_DBG(Exception, c.size(), 6, "The curve value is an unexpected size!");
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      T.topLeftCorner<3,3>() = rotation_->parametersToRotationMatrix(c.tail<3>());
      T.topRightCorner<3,1>() = c.head<3>();
      
      return T;
    }

    Eigen::Matrix4d BSplinePose::curveValueToTransformationAndJacobian( const Eigen::VectorXd & p, Eigen::MatrixXd * J ) const
    {
      SM_ASSERT_EQ_DBG(Exception, p.size(), 6, "The curve value is an unexpected size!");
      Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
      Eigen::Matrix3d S;
      T.topLeftCorner<3,3>() = rotation_->parametersToRotationMatrix(p.tail<3>(), &S);
      T.topRightCorner<3,1>() = p.head<3>();

      if(J)
	{
	  *J = Eigen::MatrixXd::Identity(6,6);
	  J->topRightCorner<3,3>() = -crossMx(p.head<3>()) * S;
	  J->bottomRightCorner<3,3>() = S;
          
	}
      
      return T;

    }

    Eigen::VectorXd BSplinePose::transformationToCurveValue( const Eigen::Matrix4d & T ) const
    {
      Eigen::VectorXd c(6);
      c.head<3>() = T.topRightCorner<3,1>();
      c.tail<3>() = rotation_->rotationMatrixToParameters(T.topLeftCorner<3,3>());
      
      return c;
    }

    void BSplinePose::initPoseSpline2(const Eigen::VectorXd & times, const Eigen::Matrix<double,6,Eigen::Dynamic> & poses, int numSegments, double lambda)
    {
      initSpline2(times, poses, numSegments, lambda);
    }

    void BSplinePose::initPoseSpline3(const Eigen::VectorXd & times, const Eigen::Matrix<double,6,Eigen::Dynamic> & poses, int numSegments, double lambda)
    {
      initSpline3(times, poses, numSegments, lambda);
    }

    void BSplinePose::initPoseSplineSparse(const Eigen::VectorXd & times, const Eigen::Matrix<double,6,Eigen::Dynamic> & poses, int numSegments, double lambda)
    {
      initSplineSparse(times, poses, numSegments, lambda);
    }
    
    void BSplinePose::initPoseSplineSparseKnots(const Eigen::VectorXd &times, const Eigen::MatrixXd &interpolationPoints, const Eigen::VectorXd knots, double lambda)
    {
    	initSplineSparseKnots(times, interpolationPoints, knots, lambda);
    }
    
    RotationalKinematics::Ptr BSplinePose::rotation() const
    {
      return rotation_;
    }
  } // namespace bsplines
