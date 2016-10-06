#include <sm/kinematics/EulerRodriguez.hpp>

namespace sm { namespace kinematics {


    EulerRodriguez::~EulerRodriguez()
    {

    }

    Eigen::Matrix3d EulerRodriguez::parametersToRotationMatrix(const Eigen::Vector3d & parameters, Eigen::Matrix3d * S) const
    {
      Eigen::Matrix3d C;
      
      Eigen::Matrix3d crossp = crossMx(parameters);
      
      C = Eigen::Matrix3d::Identity() + (2.0/(1.0 + parameters.squaredNorm()))*(crossp * crossp - crossp);

      if(S)
	{
	  *S = parametersToSMatrix(parameters);
	}

      return C;
    }

    Eigen::Vector3d EulerRodriguez::rotationMatrixToParameters(const Eigen::Matrix3d & C) const
    {
      Eigen::Vector3d p;
      double a = acos( (C(0,0) + C(1,1) + C(2,2) - double(1)) * double(0.5));

      if(fabs(a) < 1e-14){
	return Eigen::Vector3d::Zero();
      }
      
      p[0] = (C(2,1) - C(1,2));
      p[1] = (C(0,2) - C(2,0));
      p[2] = (C(1,0) - C(0,1));
      
      double n2 = p.norm();
      if(n2 < 1e-14)
	{
	  return Eigen::Vector3d::Zero();
	}
            
      double scale = -tan(0.5*a)/n2;
      p = scale * p;


      return p;
    }

    Eigen::Matrix3d EulerRodriguez::parametersToSMatrix(const Eigen::Vector3d & parameters) const
    {
      Eigen::Matrix3d S;
      Eigen::Matrix3d crossp = crossMx(parameters);
      S = (2.0/(1.0 + parameters.squaredNorm()))*(Eigen::Matrix3d::Identity() - crossp);
      
      return S;
    }

    Eigen::Vector3d EulerRodriguez::angularVelocityAndJacobian(const Eigen::Vector3d & p, const Eigen::Vector3d & pdot, Eigen::Matrix<double,3,6> * Jacobian) const
    {
      Eigen::Vector3d omega;
      Eigen::Matrix3d S;
      Eigen::Matrix3d crossp = crossMx(p);
      double recip_denom = 1.0/(1.0 + p.squaredNorm());
      double f = 2 * recip_denom; 
      S = f * (Eigen::Matrix3d::Identity() - crossp);
      omega = S*pdot;


      
      if(Jacobian)
	{
	  Eigen::Vector3d a = -4.0 * recip_denom * recip_denom * p;
	  *Jacobian = Eigen::Matrix<double,3,6>::Zero();
	  Jacobian->block(0,0,3,3) = pdot * a.transpose() - crossp * pdot * a.transpose() + f * crossMx(pdot);
	  Jacobian->block(0,3,3,3) = S;
	}

      return omega;
 
    }





  }} // sm::kinematics
