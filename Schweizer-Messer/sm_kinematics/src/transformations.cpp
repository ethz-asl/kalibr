#include <sm/kinematics/transformations.hpp>
#include <Eigen/Geometry>

namespace sm { namespace kinematics {

    Eigen::Matrix4d rt2Transform(Eigen::Matrix3d const & C_ba, Eigen::Vector3d const & rho_b_ab)
    {
      Eigen::Matrix4d T_ba;
      
      T_ba(0,0) = C_ba(0,0);   T_ba(0,1) = C_ba(0,1); T_ba(0,2) = C_ba(0,2); T_ba(0,3) = rho_b_ab[0];
      T_ba(1,0) = C_ba(1,0);   T_ba(1,1) = C_ba(1,1); T_ba(1,2) = C_ba(1,2); T_ba(1,3) = rho_b_ab[1];
      T_ba(2,0) = C_ba(2,0);   T_ba(2,1) = C_ba(2,1); T_ba(2,2) = C_ba(2,2); T_ba(2,3) = rho_b_ab[2];
      T_ba(3,0) =     0.0  ;   T_ba(3,1) =     0.0  ; T_ba(3,2) =     0.0  ; T_ba(3,3) =      1      ;


      return T_ba;
    }

    Eigen::Matrix3d transform2C(Eigen::Matrix4d const & T_ba)
    {
      return T_ba.topLeftCorner<3,3>();
    }

    Eigen::Vector3d transform2rho(Eigen::Matrix4d const & T_ba)
    {
      return T_ba.topRightCorner<3,1>();
    }

    Eigen::Vector4d transform2rhoHomogeneous(Eigen::Matrix4d const & T_ba)
    {
      return T_ba.topRightCorner<4,1>();
    }

    Eigen::Matrix4d boxPlus(const Eigen::Matrix<double,6,1> & dt)
    {
      Eigen::Matrix4d Bp;
      Bp <<   0.0,  -dt[5],   dt[4],  -dt[0],
	    dt[5],     0.0,  -dt[3],  -dt[1],
	   -dt[4],   dt[3],     0.0,  -dt[2], 
    	      0.0,     0.0,     0.0,     0.0;
      return Bp;
    }

    
    Eigen::Matrix<double,4,6> boxMinus(const Eigen::Vector4d & p)
    {
      Eigen::Matrix<double,4,6> Bm;
      Bm <<    p[3],     0.0,     0.0,     0.0,     -p[2],     p[1],
	        0.0,    p[3],     0.0,    p[2],       0.0,    -p[0], 
	        0.0,     0.0,    p[3],   -p[1],      p[0],      0.0,
	        0.0,     0.0,     0.0,     0.0,       0.0,      0.0;
      return Bm;
    }

    Eigen::Matrix4d inverseTransform(const Eigen::Matrix4d T)
    {
      Eigen::Matrix4d invT;
      invT.topLeftCorner<3,3>().noalias() = T.topLeftCorner<3,3>().transpose();
      invT.topRightCorner<3,1>().noalias() = -invT.topLeftCorner<3,3>() * T.topRightCorner<3,1>();
      invT(3,0) = 0.0;
      invT(3,1) = 0.0;
      invT(3,2) = 0.0;
      invT(3,3) = 1.0;

      return invT;
    }

    Eigen::Matrix<double,6,1> fromTEuler(const Eigen::Matrix4d & T_ab)
    {
      Eigen::Matrix<double,6,1> t;
      t.head<3>()    = T_ab.topRightCorner<3,1>();
      t.tail<3>()   = -R2rph(T_ab.topLeftCorner<3,3>());
      return t;
    }

    Eigen::Matrix4d toTEuler(const Eigen::Matrix<double,6,1> & dt)
    {
      Eigen::Matrix4d T;
      T.topLeftCorner<3,3>() = rph2R(-dt.tail<3>());
      T.topRightCorner<3,1>() = dt.head<3>();
      T(3,0) = 0.0;
      T(3,1) = 0.0;
      T(3,2) = 0.0;
      T(3,3) = 1.0;
      return T;
    }

    Eigen::Matrix4d toTEuler(double x, double y, double z, double r, double p, double h)
    {
      Eigen::Matrix4d T;
      T.topLeftCorner<3,3>() = rph2R(-r,-p,-h);
      T(0,3) = x;
      T(1,3) = y;
      T(2,3) = z;
      T(3,0) = 0.0;
      T(3,1) = 0.0;
      T(3,2) = 0.0;
      T(3,3) = 1.0;
      return T;

    }

    void transformationAndJacobian(Eigen::Matrix4d const & T_a_b, 
				   Eigen::Vector4d const & v_b, 
				   Eigen::Vector4d & out_v_a, 
				   Eigen::Matrix<double,4,6> & out_B)
    {
      out_v_a = T_a_b * v_b;
      
      out_B = boxMinus(out_v_a);
      
    }

    void inverseTransformationAndJacobian(Eigen::Matrix4d const & T_ba, 
					  Eigen::Vector4d const & v_b, 
					  Eigen::Vector4d & out_v_a, 
					  Eigen::Matrix<double,4,6> & out_B)
    {
      Eigen::Matrix4d T_ab = inverseTransform(T_ba);
      out_v_a = T_ab * v_b;
      
      out_B.topLeftCorner<3,3>().noalias() = -v_b(3)*T_ab.topLeftCorner<3,3>();
      out_B.topRightCorner<3,3>().noalias() = -T_ab.topLeftCorner<3,3>() * crossMx(v_b(0),v_b(1),v_b(2));
      out_B(3,0) = 0.0;
      out_B(3,1) = 0.0;
      out_B(3,2) = 0.0;
      out_B(3,3) = 0.0;
      out_B(3,4) = 0.0;
      out_B(3,5) = 0.0;
    }

    Eigen::Matrix<double,6,6> boxTimes(Eigen::Matrix4d const & T_ba)
    {
      typedef Eigen::Matrix<double,6,6> out_t;
      out_t Tbp = out_t::Zero();

      Tbp.topLeftCorner<3,3>() = T_ba.topLeftCorner<3,3>();
      Tbp.bottomRightCorner<3,3>() = T_ba.topLeftCorner<3,3>();
      Tbp.topRightCorner<3,3>() = -crossMx(T_ba(0,3),T_ba(1,3),T_ba(2,3)) * T_ba.topLeftCorner<3,3>();

      return Tbp;
    }


  }} // namespace asrl::math
