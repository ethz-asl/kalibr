#include <sm/kinematics/quaternion_algebra.hpp>
#include <sm/kinematics/rotations.hpp>
#include <sm/assert_macros.hpp>
#include <Eigen/Geometry>
#include <limits>
#include <cmath>

namespace sm { namespace kinematics {
        template <typename Scalar_ = double>
        inline bool isLessThenEpsilons4thRoot(Scalar_ x){
          static const Scalar_ epsilon4thRoot = pow(std::numeric_limits<Scalar_>::epsilon(), 1.0/4.0);
          return x < epsilon4thRoot;
        }

        // quaternion rotation.
        Eigen::Vector4d r2quat(Eigen::Matrix3d const & R){
      
            const double & c1 = R(0,0);
            const double & c2 = R(1,0);
            const double & c3 = R(2,0);
            const double & c4 = R(0,1);
            const double & c5 = R(1,1);
            const double & c6 = R(2,1);
            const double & c7 = R(0,2);
            const double & c8 = R(1,2);
            const double & c9 = R(2,2);

            Eigen::Vector4d dc(fabs(1.0+c1-c5-c9),
                               fabs(1.0-c1+c5-c9),
                               fabs(1.0-c1-c5+c9),
                               fabs(1.0+c1+c5+c9));

            unsigned maxq = 0;
            double maxqval = dc(0);

            for(unsigned i = 1; i < 4; i++) {
                if(dc(i) > maxqval) {
                    maxq = i;
                    maxqval = dc(i);
                }
            }

            double c;
            Eigen::Vector4d q;
            if(maxq == 0){
                q(0)=0.5*sqrt(dc(0));
                c = 0.25/q(0);
                q(1)=c*(c4+c2);
                q(2)=c*(c7+c3);
                q(3)=c*(c8-c6);
            } else if(maxq == 1){
                q(1)=0.5*sqrt(dc(1));
                c = 0.25/q(1);
                q(0)=c*(c4+c2);
                q(2)=c*(c6+c8);
                q(3)=c*(c3-c7);
            } else if(maxq == 2){
                q(2)=0.5*sqrt(dc(2));
                c = 0.25/q(2);
                q(0)=c*(c3+c7);
                q(1)=c*(c6+c8);
                q(3)=c*(c4-c2);
            } else {
                q(3)=0.5*sqrt(dc(3));
                c = 0.25/q(3);
                q(0)=c*(c8-c6);
                q(1)=c*(c3-c7);
                q(2)=c*(c4-c2);
            }

            if(q(3) < 0)
                q = -q;

            return q;
        }

        Eigen::Matrix3d quat2r(Eigen::Vector4d const & q){
    
            SM_ASSERT_NEAR_DBG(std::runtime_error,q.norm(),1.f,1e-4, "The quaternion must be a unit vector to represent a rotation");
            //double n = q(3);
            //Eigen::Vector3d e = makeV3(q(0),q(1),q(2));
            //R = (n^2 - e'*e) * eye(3) + 2 * e * e' + 2 * n * crossMx(e);
            //Eigen::Matrix3d R = (n*n - e.dot(e))*Eigen::Matrix3d::Identity() + 2 * e * e.transpose() + 2 * n * crossMx(e);
    
            Eigen::Matrix3d R;

            // [ q0^2 - q1^2 - q2^2 + q3^2,           2*q0*q1 + 2*q2*q3,           2*q0*q2 - 2*q1*q3]
            // [         2*q0*q1 - 2*q2*q3, - q0^2 + q1^2 - q2^2 + q3^2,           2*q0*q3 + 2*q1*q2]
            // [         2*q0*q2 + 2*q1*q3,           2*q1*q2 - 2*q0*q3, - q0^2 - q1^2 + q2^2 + q3^2]
            R(0,0) = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
            R(0,1) = q[0]*q[1]*2.0+q[2]*q[3]*2.0;
            R(0,2) = q[0]*q[2]*2.0-q[1]*q[3]*2.0;
            R(1,0) = q[0]*q[1]*2.0-q[2]*q[3]*2.0;
            R(1,1) = -q[0]*q[0]+q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
            R(1,2) = q[0]*q[3]*2.0+q[1]*q[2]*2.0;
            R(2,0) = q[0]*q[2]*2.0+q[1]*q[3]*2.0;
            R(2,1) = q[0]*q[3]*(-2.0)+q[1]*q[2]*2.0;
            R(2,2) = -q[0]*q[0]-q[1]*q[1]+q[2]*q[2]+q[3]*q[3];

            return R;
        }


        Eigen::Matrix4d quatPlus(Eigen::Vector4d const & q)
        {
            // [  q3,  q2, -q1, q0]
            // [ -q2,  q3,  q0, q1]
            // [  q1, -q0,  q3, q2]
            // [ -q0, -q1, -q2, q3]
            Eigen::Matrix4d Q;
            Q(0,0) =  q[3]; Q(0,1) =  q[2]; Q(0,2) = -q[1]; Q(0,3) =  q[0];
            Q(1,0) = -q[2]; Q(1,1) =  q[3]; Q(1,2) =  q[0]; Q(1,3) =  q[1];
            Q(2,0) =  q[1]; Q(2,1) = -q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
            Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];

            return Q;
        }

        Eigen::Matrix4d quatOPlus(Eigen::Vector4d const & q)
        {
            // [  q3, -q2,  q1, q0]
            // [  q2,  q3, -q0, q1]
            // [ -q1,  q0,  q3, q2]
            // [ -q0, -q1, -q2, q3]

            Eigen::Matrix4d Q;
            Q(0,0) =  q[3]; Q(0,1) = -q[2]; Q(0,2) =  q[1]; Q(0,3) =  q[0];
            Q(1,0) =  q[2]; Q(1,1) =  q[3]; Q(1,2) = -q[0]; Q(1,3) =  q[1];
            Q(2,0) = -q[1]; Q(2,1) =  q[0]; Q(2,2) =  q[3]; Q(2,3) =  q[2];
            Q(3,0) = -q[0]; Q(3,1) = -q[1]; Q(3,2) = -q[2]; Q(3,3) =  q[3];

            return Q;

        }

        Eigen::Vector4d qplus(Eigen::Vector4d const & q, Eigen::Vector4d const & p)
        {
            Eigen::Vector4d qplus_p;
            // p0*q3 + p1*q2 - p2*q1 + p3*q0
            qplus_p[0] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
            // p2*q0 - p0*q2 + p1*q3 + p3*q1
            qplus_p[1] = p[2]*q[0] - p[0]*q[2] + p[1]*q[3] + p[3]*q[1];
            // p0*q1 - p1*q0 + p2*q3 + p3*q2
            qplus_p[2] = p[0]*q[1] - p[1]*q[0] + p[2]*q[3] + p[3]*q[2];
            // p3*q3 - p1*q1 - p2*q2 - p0*q0
            qplus_p[3] = p[3]*q[3] - p[1]*q[1] - p[2]*q[2] - p[0]*q[0];

            return qplus_p;
        }

        Eigen::Vector4d qoplus(Eigen::Vector4d const & q, Eigen::Vector4d const & p)
        {
            Eigen::Vector4d qoplus_p;
            // p0*q3 - p1*q2 + p2*q1 + p3*q0
            qoplus_p[0] = p[0]*q[3] - p[1]*q[2] + p[2]*q[1] + p[3]*q[0];
            // p0*q2 - p2*q0 + p1*q3 + p3*q1
            qoplus_p[1] = p[0]*q[2] - p[2]*q[0] + p[1]*q[3] + p[3]*q[1];
            // p1*q0 - p0*q1 + p2*q3 + p3*q2
            qoplus_p[2] = p[1]*q[0] - p[0]*q[1] + p[2]*q[3] + p[3]*q[2];
            // p3*q3 - p1*q1 - p2*q2 - p0*q0
            qoplus_p[3] = p[3]*q[3] - p[1]*q[1] - p[2]*q[2] - p[0]*q[0];

            return qoplus_p;
        }

        Eigen::Vector4d quatInv(Eigen::Vector4d const & q)
        {
            Eigen::Vector4d qret = q;
            invertQuat(qret);
            return qret;
        }

        void invertQuat(Eigen::Vector4d & q)
        {
            q.head<3>() = -q.head<3>();
        }


        Eigen::Vector3d qeps(Eigen::Vector4d const & q)
        {
            return q.head<3>();
        }

        Eigen::Vector3f qeps(Eigen::Vector4f const & q)
        {
            return q.head<3>();
        }

        double qeta(Eigen::Vector4d const & q)
        {
            return q[3];
        }

        float qeta(Eigen::Vector4f const & q)
        {
            return q[3];
        }


        Eigen::Vector4d axisAngle2quat(Eigen::Vector3d const & a)
        {
            // Method of implementing this function that is accurate to numerical precision from
            // Grassia, F. S. (1998). Practical parameterization of rotations using the exponential map. journal of graphics, gpu, and game tools, 3(3):29–48.
      
            double theta = a.norm();

            // na is 1/theta sin(theta/2)
            double na;
            if(isLessThenEpsilons4thRoot(theta))
            {
                static const double one_over_48 = 1.0/48.0;
                na = 0.5 + (theta * theta) * one_over_48;
            }
            else
            {
                na = sin(theta*0.5) / theta; 
            }
            Eigen::Vector3d axis = a*na;
            double ct = cos(theta*0.5);
            return Eigen::Vector4d(axis[0],axis[1],axis[2],ct);
        }

        /**
         * calculate arcsin(x)/x
         * @param x
         * @return
         */
        template <typename Scalar_>
        inline Scalar_ arcSinXOverX(Scalar_ x) {
          if(isLessThenEpsilons4thRoot(fabs(x))){
            return Scalar_(1.0) + x * x * Scalar_(1/6);
          }
          return asin(x) / x;
        }

        template <typename Scalar_>
        Eigen::Matrix<Scalar_, 3, 1> quat2AxisAngle(Eigen::Matrix<Scalar_, 4, 1> const & q)
        {
          SM_ASSERT_LT_DBG(std::runtime_error, fabs(q.norm() - 1), 8 * std::numeric_limits<Scalar_>::epsilon(), "This function is inteded for unit quternions only.");
          const Eigen::Matrix<Scalar_, 3, 1> a = qeps(q);
          const Scalar_ na = a.norm(), eta = qeta(q);
          Scalar_ scale;
          if(fabs(eta) < na){ // use eta because it is more precise than na to calculate the scale. No singularities here.
            scale = acos(eta) / na;
          } else {
            /*
             * In this case more precision is in na than in eta so lets use na only to calculate the scale:
             *
             * assume first eta > 0 and 1 > na > 0.
             *               u = asin (na) / na  (this implies u in [1, pi/2], because na i in [0, 1]
             *    sin (u * na) = na
             *  sin^2 (u * na) = na^2
             *  cos^2 (u * na) = 1 - na^2
             *                              (1 = ||q|| = eta^2 + na^2)
             *    cos^2 (u * na) = eta^2
             *                              (eta > 0,  u * na = asin(na) in [0, pi/2] => cos(u * na) >= 0 )
             *      cos (u * na) = eta
             *                              (u * na in [ 0, pi/2] )
             *                 u = acos (eta) / na
             *
             * So the for eta > 0 it is acos(eta) / na == asin(na) / na.
             * From some geometric considerations (mirror the setting at the hyper plane q==0) it follows for eta < 0 that (pi - asin(na)) / na = acos(eta) / na.
             */
            if(eta > 0){
              // For asin(na)/ na the singularity na == 0 can be removed. We can ask (e.g. Wolfram alpha) for its series expansion at na = 0. And that is done in the following function.
              scale = arcSinXOverX(na);
            }else{
              // (pi - asin(na))/ na has a pole at na == 0. So we cannot remove this singularity.
              // It is just the cut locus of the unit quaternion manifold at identity and thus the axis angle description becomes necessarily unstable there.
              scale = (M_PI - asin(na)) / na;
            }
          }
          return a * (Scalar_(2) * scale);
        }
        template Eigen::Matrix<double, 3, 1> quat2AxisAngle(Eigen::Matrix<double, 4, 1> const & q);
        template Eigen::Matrix<float, 3, 1> quat2AxisAngle(Eigen::Matrix<float, 4, 1> const & q);

        Eigen::Matrix<double,4,3> quatJacobian(Eigen::Vector4d const & p)
        {
            Eigen::Matrix<double,4,3> J;
            // [  p3, -p2,  p1]
            // [  p2,  p3, -p0]
            // [ -p1,  p0,  p3]
            // [ -p0, -p1, -p2]
    
            J(0,0) =  p[3];
            J(0,1) = -p[2];
            J(0,2) =  p[1];
            J(1,0) =  p[2];
            J(1,1) =  p[3];
            J(1,2) = -p[0];
            J(2,0) = -p[1];
            J(2,1) =  p[0];
            J(2,2) =  p[3];
            J(3,0) = -p[0];
            J(3,1) = -p[1];
            J(3,2) = -p[2];

            return J*0.5;
        }

        Eigen::Vector4d updateQuat(Eigen::Vector4d const & q, Eigen::Vector3d const & dq)
        {
            // the following code is an optimized version of:
            // Eigen::Vector4d dq4 = axisAngle2quat(dq);
            // Eigen::Vector4d retq = quatPlus(dq4)*q;
            // return retq;

            Eigen::Vector4d dq3 = axisAngle2quat(dq);
            double ca = dq3[3];
            Eigen::Vector4d retq;
            retq[0] = q[0]*ca+dq3[0]*q[3]-dq3[1]*q[2]+dq3[2]*q[1];
            retq[1] = q[1]*ca+dq3[0]*q[2]+dq3[1]*q[3]-dq3[2]*q[0];
            retq[2] = q[2]*ca-dq3[0]*q[1]+dq3[1]*q[0]+dq3[2]*q[3];
            retq[3] = q[3]*ca-dq3[0]*q[0]-dq3[1]*q[1]-dq3[2]*q[2];

            return retq;
        }

        Eigen::Vector3d quatRotate(Eigen::Vector4d const & q_a_b, Eigen::Vector3d const & v_b)
        {
            return v_b + 2.0 * q_a_b.head<3>().cross(q_a_b.head<3>().cross(v_b) - q_a_b[3] * v_b);
        }

        Eigen::Vector4d quatRandom()
        {
            Eigen::Vector4d q_a_b;
            q_a_b.setRandom();
            q_a_b.array() -= 0.5;
            q_a_b /= q_a_b.norm();
            return q_a_b;
        }

        Eigen::Vector4d quatIdentity()
        {
            return Eigen::Vector4d(0,0,0,1);
        }

        Eigen::Matrix<double,3,4> quatS(Eigen::Vector4d q)
        {
            //   [  q3,  q2, -q1, -q0]
            // 2 [ -q2,  q3,  q0, -q1]
            //   [  q1, -q0,  q3, -q2]
            q *= 2.0;
      
            Eigen::Matrix<double,3,4> S;
            S <<
                q[3],  q[2], -q[1], -q[0],
                -q[2],  q[3],  q[0], -q[1],
                q[1], -q[0],  q[3], -q[2];

            return S;
        }

        Eigen::Matrix<double,4,3> quatInvS(Eigen::Vector4d q)
        {
            q *= 0.5;

            // 1 [  q3, -q2,  q1]
            // - [  q2,  q3, -q0]
            // 2 [ -q1,  q0,  q3]
            //   [ -q0, -q1, -q2]


            Eigen::Matrix<double,4,3> invS;
            invS <<
                q[3], -q[2],  q[1],
                q[2],  q[3], -q[0],
                -q[1],  q[0],  q[3],
                -q[0], -q[1], -q[2];
            return invS;
        }

        Eigen::Vector4d qslerp(const Eigen::Vector4d & q0, const Eigen::Vector4d & q1, double t)
        {
          if(t <= 0.0)
            {
                return q0;
            }
            else if(t >= 1.0)
            {
                return q1;
            }
            else
            {
              if( (q0-q1).squaredNorm() > (q0 + q1).squaredNorm() ) {
                // The quaternions are far away from eachother on the sphere.
                // Flip one around so that this works out.
                return qplus(q0, qexp(t * qlog( qplus(quatInv(q0),-q1))));
              } else {
                return qplus(q0, qexp(t * qlog( qplus(quatInv(q0),q1))));
              }
                
            }
        }
        

        /// \brief do linear interpolation between p0 and p1 for times t = [0.0,1.0]
        Eigen::VectorXd lerp(const Eigen::VectorXd & p0, const Eigen::VectorXd & p1, double t)
        {
            SM_ASSERT_EQ(std::runtime_error, p0.size(), p1.size(), "The vectors must be the same size");
            if(t <= 0.0)
            {
                return p0;
            }
            else if(t >= 1.0)
            {
                return p1;
            }
            else
            {
                return (1-t) * p0  +  t * p1;
            }
        }

        Eigen::Matrix<double,3,4> quatLogJacobian(const Eigen::Vector4d& p)
        {
          //      [qx]
          //      [qy]                -2*x
          // p =  [qz],    g(x) = ----------------
          //      [qw]               sqrt(1-qw²)
          //
          //
          //      [2*acos(qw)      0            0            g(qx)]
          // J =  [0            2*acos(qw)      0            g(qy)]
          //      [0            0            2*acos(qw)      g(qz)]


          Eigen::Matrix<double, 3,4> J;
          J.setZero();

          double n = qeps(p).norm();


          double de = n*n*n;//pow(n, 3);
          double u12 = p(1)*p(1) + p(2)*p(2);//pow(p(1), 2) + pow(p(2), 2);
          double u02 = p(0)*p(0) + p(2)*p(2);//pow(p(0), 2) + pow(p(2), 2);
          double u01 = p(0)*p(0) + p(1)*p(1);//pow(p(0), 2) + pow(p(1), 2);
          double a = acos(p(3));
          double uw = sqrt(-(p(3)*p(3) - 1)*n*n);

          J(0,0) = 2*a*u12 / de;
          J(0,1) = -2*a*p(1)*p(0) / de;
          J(0,2) = -2*a*p(2)*p(0) / de;
          J(0,3) = -2*p(0) / uw;
          J(1,0) = -2*a*p(0)*p(1) / de;
          J(1,1) = 2*a*u02 / de;
          J(1,2) = -2*a*p(1)*p(2) / de;
          J(1,3) = -2*p(1) / uw;
          J(2,0) = -2*a*p(0)*p(2) / de;
          J(2,1) = -2*a*p(1)*p(2) / de;
          J(2,2) = 2*a*u01 / de;
          J(2,3) = -2*p(2) / uw;

          return J;
        }

        template <typename Scalar_>
        inline const typename Eigen::Matrix<Scalar_, 4, 3> & quatV(){
          static const Eigen::Matrix<Scalar_, 4, 3> V = 0.5 * Eigen::Matrix<Scalar_, 4, 3>::Identity();
          return V;
        }
        template const Eigen::Matrix<double, 4, 3> & quatV();
        template const Eigen::Matrix<float, 4, 3> & quatV();


        template <typename Scalar_>
        Eigen::Matrix<Scalar_, 3, 3> expDiffMat(const Eigen::Matrix<Scalar_, 3, 1> & vec){
          Scalar_ phi = vec.norm();

          if(phi == 0){
            return Eigen::Matrix<Scalar_, 3, 3>::Identity();
          }

          Eigen::Matrix<Scalar_, 3, 3> vecCross = crossMx(vec);

          Scalar_ phiAbs = fabs(phi);
          Scalar_ phiSquare = phi * phi;

          Scalar_ a;
          Scalar_ b;
          if(!isLessThenEpsilons4thRoot(phiAbs)){
            Scalar_ siPhiHalf = sin(phi / 2);
            a = (2 * siPhiHalf * siPhiHalf / phiSquare);
            b = ((1 - sin(phi) / phi)/phiSquare);
          }
          else{
            a = (1.0/2) * (1 - (1.0 / (24 / 2)) * phiSquare);
            b = (1.0/6) * (1 - (1.0 / (120 / 6)) * phiSquare);
          }

          return Eigen::Matrix<Scalar_, 3, 3>::Identity() - a * vecCross + b * vecCross * vecCross;
        }
        template Eigen::Matrix<double, 3, 3> expDiffMat<double>(const Eigen::Matrix<double, 3, 1>  &);
        template Eigen::Matrix<float, 3, 3> expDiffMat<float>(const Eigen::Matrix<float, 3, 1>  &);

        template <typename Scalar_>
        Eigen::Matrix<Scalar_ , 4, 3> quatExpJacobian(const Eigen::Matrix<Scalar_ , 3, 1>& vec){
          return quatOPlus(axisAngle2quat(vec.template cast<double>())).template cast<Scalar_>() * quatV<Scalar_>() * expDiffMat(vec);
        }
        template Eigen::Matrix<double, 4,3> quatExpJacobian(const Eigen::Matrix<double, 3, 1>& vec);
        template Eigen::Matrix<float, 4,3> quatExpJacobian(const Eigen::Matrix<float, 3, 1>& vec);

        template <typename Scalar_>
        Eigen::Matrix<Scalar_, 3, 3> logDiffMat(const Eigen::Matrix<Scalar_, 3, 1>  & vec){
          Scalar_ phi = vec.norm();
          if(phi == 0){
            return Eigen::Matrix<Scalar_, 3, 3>::Identity();
          }

          Scalar_ phiAbs = fabs(phi);
          Eigen::Matrix<Scalar_, 3, 3> vecCross = crossMx(vec);

          Scalar_ a;
          if(!isLessThenEpsilons4thRoot(phiAbs)){
            Scalar_ phiHalf = 0.5 * phi;
            a = ((1 - phiHalf / tan(phiHalf))/phi/phi);
          }
          else{
            a = 1.0 / 12 * (1 + 1.0 / 60 * phi * phi);
          }
          return Eigen::Matrix<Scalar_, 3, 3>::Identity() + 0.5 * vecCross + a * vecCross * vecCross;
        }
        template Eigen::Matrix<double, 3, 3> logDiffMat<double>(const Eigen::Matrix<double, 3, 1>  &);
        template Eigen::Matrix<float, 3, 3> logDiffMat<float>(const Eigen::Matrix<float, 3, 1>  &);

        template <typename Scalar_>
        Eigen::Matrix<Scalar_ , 3, 4> quatLogJacobian2(const Eigen::Matrix<Scalar_ , 4, 1>& p){
          return logDiffMat(quat2AxisAngle<>(p)) * (quatV<Scalar_>().transpose() * Scalar_(4.0)) * quatOPlus(quatInv(p.template cast<double>())).template cast<Scalar_>();
        }
        
        template Eigen::Matrix<double, 3, 4> quatLogJacobian2(const Eigen::Matrix<double, 4, 1>&);
        template Eigen::Matrix<float, 3, 4> quatLogJacobian2(const Eigen::Matrix<float, 4, 1>& );
    }} // namespace sm::kinematics

