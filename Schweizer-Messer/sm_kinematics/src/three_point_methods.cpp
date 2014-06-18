#include <sm/kinematics/three_point_methods.hpp>
#include <Eigen/SVD>
#include <Eigen/LU>
#include <sm/assert_macros.hpp>
#include <Eigen/Dense>
#include <sm/kinematics/rotations.hpp>

namespace sm { namespace kinematics {

        // Original code from the ROS vslam package pe3d.cpp
        // uses the SVD procedure for aligning point clouds
        //   SEE: Arun, Huang, Blostein: Least-Squares Fitting of Two 3D Point Sets
        Eigen::Matrix4d threePointSvd(Eigen::MatrixXd const & p0, Eigen::MatrixXd const & p1)
        {
            using namespace Eigen;

            SM_ASSERT_EQ_DBG(std::runtime_error, p0.rows(), 3, "p0 must be a 3xK matrix");
            SM_ASSERT_EQ_DBG(std::runtime_error, p1.rows(), 3, "p1 must be a 3xK matrix");
            SM_ASSERT_EQ_DBG(std::runtime_error, p0.cols(), p1.cols(), "p0 and p1 must have the same number of columns");

            Vector3d c0 = p0.rowwise().mean();
            Vector3d c1 = p1.rowwise().mean();

            Matrix3d H(Matrix3d::Zero());
            // subtract out
            // p0a -= c0;
            // p0b -= c0;
            // p0c -= c0;
            // p1a -= c1;
            // p1b -= c1;
            // p1c -= c1;

            // Matrix3d H = p1a*p0a.transpose() + p1b*p0b.transpose() +
            // 	p1c*p0c.transpose();
            for(int i = 0; i < p0.cols(); ++i)
            {
                H += (p0.col(i) - c0) * (p1.col(i) - c1).transpose();
            }
      

            // do the SVD thang
            JacobiSVD<Matrix3d> svd(H,ComputeFullU | ComputeFullV);
            Matrix3d V = svd.matrixV();
            Matrix3d R = V * svd.matrixU().transpose();          
            double det = R.determinant();
        
            if (det < 0.0)
            {
                V.col(2) = V.col(2) * -1.0;
                R = V * svd.matrixU().transpose();
            }
            Vector3d tr = c0-R.transpose()*c1;    // translation 
      
            // transformation matrix, 3x4
            Matrix4d tfm(Matrix4d::Identity());
            //        tfm.block<3,3>(0,0) = R.transpose();
            //        tfm.col(3) = -R.transpose()*tr;
            tfm.topLeftCorner<3,3>() = R.transpose();
            tfm.topRightCorner<3,1>() = tr;
      
            return tfm;
      
        }

        Eigen::Matrix3d qMethod(Eigen::MatrixXd const & p0, Eigen::MatrixXd const & p1, const Eigen::VectorXd & w)
        {
            SM_ASSERT_EQ_DBG(std::runtime_error, p0.rows(), 3, "p0 must be a 3xK matrix");
            SM_ASSERT_EQ_DBG(std::runtime_error, p1.rows(), 3, "p1 must be a 3xK matrix");
            SM_ASSERT_EQ_DBG(std::runtime_error, p0.cols(), p1.cols(), "p0 and p1 must have the same number of columns");
            SM_ASSERT_EQ_DBG(std::runtime_error, w.size(), p0.cols(),  "w must have the same number of columns as p0");

            Eigen::MatrixXd W = p0;
            Eigen::MatrixXd V = p1;
      
            for(int i = 0; i < p0.cols(); i++)
            {
                double wi = sqrt(w[i]);
                SM_ASSERT_NEAR_DBG(std::runtime_error, p0.col(i).norm(),1.0,1e-4,"Column " << i << " of p0 was not a unit vector");
                SM_ASSERT_NEAR_DBG(std::runtime_error, p1.col(i).norm(),1.0,1e-4,"Column " << i << " of p1 was not a unit vector");
	  
                W.col(i) = wi * W.col(i);
                V.col(i) = wi * V.col(i);
            }

            Eigen::MatrixXd B = W * V.transpose();
            Eigen::MatrixXd Q = B + B.transpose();

            Eigen::Vector3d Z(B(1,2) - B(2,1), 
                              B(2,0) - B(0,2),
                              B(0,1) - B(1,0));
            double sigma = B(0,0) + B(1,1) + B(2,2);

            Eigen::Matrix4d K;
            K.topLeftCorner<3,3>() = Q - sigma * Eigen::Matrix3d::Identity();
            K.topRightCorner<3,1>() = Z;
            K.bottomLeftCorner<1,3>() = Z.transpose();
            K(3,3) = sigma;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> eigensolver(K);

            Eigen::Vector4d eigenvalues = eigensolver.eigenvalues();
            // Find the maximum eigenvalue

            double maxVal = eigenvalues(0);
            int maxValIdx = 0;
            for(int i = 1; i < 4; i++)
            {
                if(eigenvalues(i) > maxVal)
                {
                    maxVal = eigenvalues(i);
                    maxValIdx = i;
                }
            }

            // The corresponding eigenvector is the quaternion q_01
            Eigen::Vector4d q_01 = eigensolver.eigenvectors().col(maxValIdx);
            q_01 /= q_01.norm();
      
            Eigen::Vector3d qv = q_01.head<3>();
            double qs = q_01(3);

            Eigen::Matrix3d C_01 = (qs*qs - qv.dot(qv))*Eigen::Matrix3d::Identity() + 2.0 * qv * qv.transpose() - 2.0 * qs * sm::kinematics::crossMx(qv);

            return C_01;
        }

        Eigen::Matrix3d qMethod(Eigen::MatrixXd const & p0, Eigen::MatrixXd const & p1)
        {
            return qMethod(p0,p1,Eigen::VectorXd::Ones(p1.cols()));
        }


    }} // namespace sm::kinematics
