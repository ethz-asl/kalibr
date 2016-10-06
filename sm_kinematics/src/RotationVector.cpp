#include <sm/kinematics/RotationVector.hpp>

namespace sm { namespace kinematics {

        RotationVector::~RotationVector()
        {

        }
    
        Eigen::Matrix3d RotationVector::parametersToRotationMatrix(const Eigen::Vector3d & parameters, Eigen::Matrix3d * S) const
        {
            Eigen::Matrix3d C;
      
            double angle = parameters.norm();
      
            if(angle < 1e-14)
            {
                // The angle goes to zero.
                C = Eigen::Matrix3d::Identity();
                if(S)
                {
                    *S = Eigen::Matrix3d::Identity();
                }
            }
            else
            {
                Eigen::Vector3d axis;
                double recip_angle = 1.0/angle;
                axis = parameters * recip_angle;

                double ax = axis[0];
                double ay = axis[1];
                double az = axis[2];
                double sa = sin(angle);
                double ca = cos(angle);
                double ax2 = ax*ax;
                double ay2 = ay*ay;
                double az2 = az*az;

                C <<
                    ax2+ca*(1.0-ax2),     ax*ay-ca*ax*ay+sa*az, ax*az-ca*ax*az-sa*ay,
                    ax*ay-ca*ax*ay-sa*az, ay2+ca*(1.0-ay2),     ay*az-ca*ay*az+sa*ax,
                    ax*az-ca*ax*az+sa*ay, ay*az-ca*ay*az-sa*ax, az2+ca*(1.0-az2);


                if(S)
                {
                    *S = parametersToSMatrix(parameters);
                }
            }

      

            return C;
        }

        Eigen::Vector3d RotationVector::rotationMatrixToParameters(const Eigen::Matrix3d & C) const
        {
            Eigen::Vector3d p;
            // Sometimes, because of roundoff error, the value of tr ends up outside
            // the valid range of arccos. Truncate to the valid range.
            double tr = std::max(-1.0, std::min( (C(0,0) + C(1,1) + C(2,2) - 1.0) * 0.5, 1.0));
            double a = acos( tr ) ;

            if(fabs(a) < 1e-14){
                return Eigen::Vector3d::Zero();
            }
      
            p[0] = (C(2,1) - C(1,2));
            p[1] = (C(0,2) - C(2,0));
            p[2] = (C(1,0) - C(0,1));
            double n2 = p.norm();
            if(fabs(n2) < 1e-14)
                return Eigen::Vector3d::Zero();
      
            double scale = -a/n2;
            p = scale * p;

            return p;
        }

        Eigen::Matrix3d RotationVector::parametersToSMatrix(const Eigen::Vector3d & parameters) const
        {
            Eigen::Matrix3d S;
            double angle = parameters.norm();
      
            if(angle < 1e-14)
            {
                S = Eigen::Matrix3d::Identity();
            }
            else
            {
                double recip_angle = 1.0/angle;
                Eigen::Vector3d axis = parameters * recip_angle;
                double st2 = sin(angle * 0.5);
                double st  = sin(angle);

                double c1 = -2.0 * st2 * st2 * recip_angle;
                double c2 = (angle - st) * recip_angle;
                Eigen::Matrix3d crossA = crossMx(axis);
      
                S = Eigen::Matrix3d::Identity() + (c1 * crossA) + (c2 * crossA * crossA);
            }

            return S;
        }


        Eigen::Vector3d RotationVector::angularVelocityAndJacobian(const Eigen::Vector3d & p, const Eigen::Vector3d & pdot, Eigen::Matrix<double,3,6> * Jacobian) const
        {
            Eigen::Vector3d omega;
            Eigen::Matrix3d S = parametersToSMatrix(p);

            omega = S*pdot;

      
            if(Jacobian)
            {
                *Jacobian = Eigen::Matrix<double,3,6>::Zero();

                //Jacobian->block(0,0,3,3) = Eigen::Matrix3d::Zero();
                Jacobian->block(0,3,3,3) = S;
    
                // LAZY
                // \todo...redo when there is time and not so lazy.
                Eigen::Matrix<double,3,6> & J = *Jacobian;
                double t1 = p[0];
                double t2 = p[1];
                double t3 = p[2];
                double dt1 = pdot[0];
                double dt2 = pdot[1];
                double dt3 = pdot[2];
                double t5 = t1*t1;
                double t6 = t2*t2;
                double t7 = t3*t3;
                double t8 = t5+t6+t7;
                double t9 = pow(t8,3.0/2.0);
                double t10 = sqrt(t8);
                double t11 = sin(t10);
                double t12 = cos(t10);
                double t13 = t12*(1.0/2.0);
                double t14 = t13-1.0/2.0;
                double t15 = 1.0/pow(t8,5.0/2.0);
                double t16 = dt3*t14*t9*2.0;
                double t17 = dt3*t1*t11*t2*t3*3.0;
                double t18 = dt3*t3*t9;
                double t19 = dt1*t1*t10*t6*2.0;
                double t20 = dt2*t11*t2*t5*3.0;
                double t21 = dt1*t1*t10*t12*t6;
                double t22 = dt2*t14*t9*2.0;
                double t23 = dt2*t1*t10*t2*t3*2.0;
                double t24 = dt2*t1*t10*t12*t2*t3;
                double t25 = dt1*t14*t9*2.0;
                double t26 = dt1*t1*t11*t2*t3*3.0;
                double t27 = dt1*t1*t9;
                double t28 = dt2*t2*t9;
                double t29 = dt1*t1*t10*t7*2.0;
                double t30 = dt2*t10*t2*t7*2.0;
                double t31 = dt3*t11*t3*t5*3.0;
                double t32 = dt3*t11*t3*t6*3.0;
                double t33 = dt2*t1*t11*t3*t8;
                double t34 = dt2*t1*t10*t14*t3*4.0;
                double t35 = dt1*t1*t10*t12*t7;
                double t36 = dt2*t10*t12*t2*t7;
                double t0  = t15*(t18+t19+t20+t21+t28+t29+t31+t33+t34+t35-dt1*t1*t11*t6*3.0-dt2*t10*t2*t5*2.0-dt1*t1*t11*t7*3.0-dt3*t10*t3*t5*2.0-dt2*t11*t2*t8-dt3*t11*t3*t8-dt3*t1*t10*t14*t2*4.0-dt2*t10*t12*t2*t5-dt3*t10*t12*t3*t5-dt3*t1*t11*t2*t8);


                J(0,0) = t0;
                J(0,1) = t15*(t16+t17+dt2*t1*t9-dt1*t2*t9*2.0-dt2*t1*t10*t6*2.0+dt2*t1*t11*t6*3.0-dt3*t10*t14*t6*4.0+dt1*t10*t2*t6*2.0-dt1*t11*t2*t6*3.0+dt1*t10*t2*t7*2.0-dt1*t11*t2*t7*3.0-dt2*t1*t11*t8+dt1*t11*t2*t8*2.0-dt3*t11*t6*t8-dt3*t1*t10*t2*t3*2.0+dt2*t10*t14*t2*t3*4.0-dt2*t1*t10*t12*t6+dt1*t10*t12*t2*t6+dt1*t10*t12*t2*t7+dt2*t11*t2*t3*t8-dt3*t1*t10*t12*t2*t3);
                J(0,2) = -t15*(t22+t23+t24-dt3*t1*t9+dt1*t3*t9*2.0+dt3*t1*t10*t7*2.0-dt3*t1*t11*t7*3.0-dt2*t10*t14*t7*4.0-dt1*t10*t3*t6*2.0+dt1*t11*t3*t6*3.0+dt3*t1*t11*t8-dt1*t10*t3*t7*2.0+dt1*t11*t3*t7*3.0-dt1*t11*t3*t8*2.0-dt2*t11*t7*t8-dt2*t1*t11*t2*t3*3.0+dt3*t10*t14*t2*t3*4.0+dt3*t1*t10*t12*t7-dt1*t10*t12*t3*t6-dt1*t10*t12*t3*t7+dt3*t11*t2*t3*t8);
                J(1,0) = -t15*(t16-t17+dt2*t1*t9*2.0-dt1*t2*t9-dt2*t1*t10*t5*2.0+dt2*t1*t11*t5*3.0-dt3*t10*t14*t5*4.0+dt1*t10*t2*t5*2.0-dt1*t11*t2*t5*3.0-dt2*t1*t10*t7*2.0+dt2*t1*t11*t7*3.0-dt2*t1*t11*t8*2.0+dt1*t11*t2*t8-dt3*t11*t5*t8+dt1*t1*t10*t14*t3*4.0+dt3*t1*t10*t2*t3*2.0-dt2*t1*t10*t12*t5+dt1*t10*t12*t2*t5-dt2*t1*t10*t12*t7+dt1*t1*t11*t3*t8+dt3*t1*t10*t12*t2*t3);
                J(1,1) = t15*(t18-t19-t20-t21+t27+t30+t32+t36+dt1*t1*t11*t6*3.0+dt2*t10*t2*t5*2.0-dt1*t1*t11*t8-dt2*t11*t2*t7*3.0-dt3*t10*t3*t6*2.0-dt3*t11*t3*t8+dt3*t1*t10*t14*t2*4.0-dt1*t10*t14*t2*t3*4.0+dt2*t10*t12*t2*t5-dt3*t10*t12*t3*t6+dt3*t1*t11*t2*t8-dt1*t11*t2*t3*t8);
                J(1,2) = t15*(t25+t26+dt3*t2*t9-dt2*t3*t9*2.0+dt2*t10*t3*t5*2.0-dt2*t11*t3*t5*3.0-dt1*t10*t14*t7*4.0-dt3*t10*t2*t7*2.0+dt3*t11*t2*t7*3.0+dt2*t10*t3*t7*2.0-dt2*t11*t3*t7*3.0-dt3*t11*t2*t8+dt2*t11*t3*t8*2.0-dt1*t11*t7*t8+dt3*t1*t10*t14*t3*4.0-dt1*t1*t10*t2*t3*2.0+dt2*t10*t12*t3*t5-dt3*t10*t12*t2*t7+dt2*t10*t12*t3*t7+dt3*t1*t11*t3*t8-dt1*t1*t10*t12*t2*t3);
                J(2,0) = t15*(t22-t23-t24-dt3*t1*t9*2.0+dt1*t3*t9+dt3*t1*t10*t5*2.0-dt3*t1*t11*t5*3.0-dt2*t10*t14*t5*4.0+dt3*t1*t10*t6*2.0-dt3*t1*t11*t6*3.0-dt1*t10*t3*t5*2.0+dt1*t11*t3*t5*3.0+dt3*t1*t11*t8*2.0-dt1*t11*t3*t8-dt2*t11*t5*t8+dt1*t1*t10*t14*t2*4.0+dt2*t1*t11*t2*t3*3.0+dt3*t1*t10*t12*t5+dt3*t1*t10*t12*t6-dt1*t10*t12*t3*t5+dt1*t1*t11*t2*t8);
                J(2,1) = -t15*(t25-t26+dt3*t2*t9*2.0-dt2*t3*t9-dt3*t10*t2*t5*2.0+dt3*t11*t2*t5*3.0-dt1*t10*t14*t6*4.0-dt3*t10*t2*t6*2.0+dt3*t11*t2*t6*3.0+dt2*t10*t3*t6*2.0-dt2*t11*t3*t6*3.0-dt3*t11*t2*t8*2.0+dt2*t11*t3*t8-dt1*t11*t6*t8+dt2*t1*t10*t14*t2*4.0+dt1*t1*t10*t2*t3*2.0-dt3*t10*t12*t2*t5-dt3*t10*t12*t2*t6+dt2*t10*t12*t3*t6+dt2*t1*t11*t2*t8+dt1*t1*t10*t12*t2*t3);
                J(2,2) = t15*(t27+t28-t29-t30-t31-t32-t33-t34-t35-t36+dt1*t1*t11*t7*3.0+dt3*t10*t3*t5*2.0-dt1*t1*t11*t8+dt2*t11*t2*t7*3.0+dt3*t10*t3*t6*2.0-dt2*t11*t2*t8+dt1*t10*t14*t2*t3*4.0+dt3*t10*t12*t3*t5+dt3*t10*t12*t3*t6+dt1*t11*t2*t3*t8);


            }

            return omega;
 
        }


        Eigen::Matrix3d RotationVector::parametersToInverseSMatrix(const Eigen::Vector3d & parameters) const
        {
            double phi = std::sqrt(parameters.transpose()*parameters);
            Eigen::Matrix3d invS;
            if(phi == 0)
            {
                invS = Eigen::Matrix3d::Identity();
            }
            else
            {
                double cot = - std::sin(phi)/(std::cos(phi)-1);
                double a1 = 1/(phi*phi) * (1- 0.5*phi*cot);
                invS = Eigen::Matrix3d::Identity() + 0.5*crossMx(parameters) + a1*crossMx(parameters)*crossMx(parameters);
            }
            return invS;
        }


    }} // sm::kinematics
