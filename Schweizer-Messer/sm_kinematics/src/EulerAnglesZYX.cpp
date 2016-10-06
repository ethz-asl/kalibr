#include <sm/kinematics/EulerAnglesZYX.hpp>

namespace sm { namespace kinematics {


    EulerAnglesZYX::~EulerAnglesZYX()
    {

    }


    Eigen::Matrix3d EulerAnglesZYX::parametersToRotationMatrix(const Eigen::Vector3d & parameters, Eigen::Matrix3d * S) const
    {
      Eigen::Matrix3d C;
      double cx = cos(-parameters[2]);
      double sx = sin(-parameters[2]);
      double cy = cos(-parameters[1]);
      double sy = sin(-parameters[1]);
      double cz = cos(-parameters[0]);
      double sz = sin(-parameters[0]);
      //[cos(z)*cos(y), -sin(z)*cos(x)+cos(z)*sin(y)*sin(x),  sin(z)*sin(x)+cos(z)*sin(y)*cos(x)]
      //[sin(z)*cos(y),  cos(z)*cos(x)+sin(z)*sin(y)*sin(x), -cos(z)*sin(x)+sin(z)*sin(y)*cos(x)]
      //[      -sin(y),                       cos(y)*sin(x),                       cos(y)*cos(x)]
      C << 
	cz*cy,  -sz*cx+cz*sy*sx,   sz*sx+cz*sy*cx,
	sz*cy,   cz*cx+sz*sy*sx,  -cz*sx+sz*sy*cx,
        -sy,         cy*sx,            cy*cx;

      if(S)
	{
	  *S << buildSMatrix(sz, cz, sy, cy);
	}
      
      return C;
    }

    Eigen::Vector3d EulerAnglesZYX::rotationMatrixToParameters(const Eigen::Matrix3d & rotationMatrix) const
    {
      double phi = asin(rotationMatrix(2,0));
      double theta = atan2(rotationMatrix(2,1),rotationMatrix(2,2));
      double psi = atan2(rotationMatrix(1,0), rotationMatrix(0,0));
    
      Eigen::Vector3d ret;
      ret[2] = -theta;
      ret[1] = phi;
      ret[0] = -psi;
      
      return ret;
    }
    
    Eigen::Matrix3d EulerAnglesZYX::parametersToSMatrix(const Eigen::Vector3d & parameters) const
    {
      double cy = cos(-parameters[1]);
      double sy = sin(-parameters[1]);
      double cz = cos(-parameters[0]);
      double sz = sin(-parameters[0]);      
      return buildSMatrix(sz, cz, sy, cy);
    }
    
    Eigen::Matrix3d EulerAnglesZYX::buildSMatrix(double sz, double cz, double sy, double cy) const
    {
      Eigen::Matrix3d S;
      S << 
	0.0, -sz, cy*cz,
	0.0,  cz, cy*sz,
	1.0, 0.0,  -sy;
      
      return S;
    }

    Eigen::Vector3d EulerAnglesZYX::angularVelocityAndJacobian(const Eigen::Vector3d & p, const Eigen::Vector3d & pdot, Eigen::Matrix<double,3,6> * Jacobian) const
    {
      //double cx = cos(-p[2]);
      //double sx = sin(-p[2]);
      double cy = cos(-p[1]);
      double sy = sin(-p[1]);
      double cz = cos(-p[0]);
      double sz = sin(-p[0]);

      Eigen::Matrix3d S = buildSMatrix(sz, cz, sy, cy);

      Eigen::Vector3d omega = S * pdot;
      
      if(Jacobian)
	{
	  *Jacobian = Eigen::Matrix<double,3,6>::Zero();
	  (*Jacobian)(0,0) = cz*pdot[1] + cy*sz*pdot[2];
	  (*Jacobian)(0,1) = cz*sy*pdot[2];
	  (*Jacobian)(1,0) = sz*pdot[1] - cz*cy*pdot[2];
	  (*Jacobian)(1,1) = sz*sy*pdot[2];
	  (*Jacobian)(2,1) = cy*pdot[2];
	  Jacobian->block(0,3,3,3) = S;
	}

      return omega;
    }


  }} // sm::kinematics
