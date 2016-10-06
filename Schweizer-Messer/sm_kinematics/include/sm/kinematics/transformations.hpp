#ifndef ASRL_TRANSFORMATIONS_HPP
#define ASRL_TRANSFORMATIONS_HPP

/**
 * @file   transformations.hpp
 * @author Paul Furgale <paul.furgale@utoronto.ca>
 * @date   Tue Dec  7 10:47:44 2010
 * 
 * @brief  Functions for dealing with homogeneous transformation matrices.
 * 
 * 
 */




#include <sm/assert_macros.hpp>
#include "rotations.hpp"

namespace sm { namespace kinematics {
	
    // Homogeneous transformation matrices:


    /** 
     * Creates a transformation matrix \f$ \mathbf T_{ba} = \begin{bmatrix} \mathbf C_{ba} & \mathbf p_b^{ab} \\ \mathbf 0^T & 1 \end{bmatrix}\f$ from the constituent parts
     * 
     * @param C_ba     The rotation matrix \f$ \mathbf C_{ba} \f$
     * @param rho_b_ab The translation column \f$ \mathbf p_b^{ab} \f$
     * 
     * @return the transformation matrix \f$ \mathbf T_{ba} \f$
     */
    Eigen::Matrix4d rt2Transform(Eigen::Matrix3d const & C_ba, Eigen::Vector3d const & rho_a_ba);

    /** 
     * Extracts the rotation matrix \f$\mathbf C_{ba} \f$ from the transformation matrix \f$ \mathbf T_{ba} \f$
     * 
     * @param T_ba the transformation matrix \f$ \mathbf T_{ba} \f$
     * 
     * @return the rotation matrix \f$\mathbf C_{ba} \f$
     */
    Eigen::Matrix3d transform2C(Eigen::Matrix4d const & T_ba);

    /** 
     * Extracts the translation \f$\mathbf p_b^{ab} \f$ from the transformation matrix \f$ \mathbf T_{ba} \f$
     * 
     * @param T_ba the transformation matrix \f$ \mathbf T_{ba} \f$
     * 
     * @return the translation \f$\mathbf p_b^{ab} \f$
     */
    Eigen::Vector3d transform2rho(Eigen::Matrix4d const & T_ba);

    /** 
     * Extracts the translation \f$\mathbf p_b^{ab} \f$ from the transformation matrix \f$ \mathbf T_{ba} \f$
     * 
     * @param T_ba the transformation matrix \f$ \mathbf T_{ba} \f$
     * 
     * @return the translation \f$\mathbf p_b^{ab} \f$
     */

    Eigen::Vector4d transform2rhoHomogeneous(Eigen::Matrix4d const & T_ba);

    /** 
     * Creates the box plus matrix derived in the documentation.
     * 
     * @param dt a \f$6 \times 1\f$ perturbation \f$\delta \mathbf t \f$
     * 
     * @return The perturbation box plus matrix
     */
    Eigen::Matrix4d boxPlus(const Eigen::Matrix<double,6,1> & dt); 
    
    /** 
     * Creates the box minus matrix derived in the documentation.
     * 
     * @param p a \f$4 \times 1\f$ homogeneous point
     * 
     * @return The box minus matrix based on this point.
     */
    Eigen::Matrix<double,4,6> boxMinus(const Eigen::Vector4d & p); 

    /** 
     * Computes the inverse of a \f$4 \times 4\f$ transformation matrix
     * 
     * @param T The input transformation matrix 
     * 
     * @return The inverse of the transformation matrix.
     */
    Eigen::Matrix4d inverseTransform(const Eigen::Matrix4d T);
    
    /** 
     * Decompose a transformation matrix, \f$\mathbf T_{ab} \f$ into a 6x1 set of parameters
     * where the top 3x1 are associated with the translation \f$ \mathbf p_a^{ba} \f$ and
     * the bottom 3x1 are Euler angles of the rotation \f$ \mathbf C_{ab} \f$
     *
     * \see r2rph(), toTEuler()
     *
     * @param T_ab a 4x4 transformation matrix
     * 
     * @return The 6x1 column of parameters.
     */
    Eigen::Matrix<double,6,1> fromTEuler(const Eigen::Matrix4d & T_ab);

    /** 
     * Take a 6x1 column of transformation parameters and create a 4x4 transformation matrix
     * 
     * The top 3x1 are associated with the translation \f$ \mathbf p_a^{ba} \f$ and
     * the bottom 3x1 are Euler angles of the rotation \f$ \mathbf C_{ab} \f$
     *
     * @param dt a 6x1 column of transformation parameters
     * 
     * @return a 4x4 transformation matrix.
     */
    Eigen::Matrix4d toTEuler(const Eigen::Matrix<double,6,1> & dt);

    /** 
     * Take a 6x1 column of transformation parameters and create a 4x4 transformation matrix
     * 
     * The top 3x1 are associated with the translation \f$ \mathbf p_a^{ba} \f$ and
     * the bottom 3x1 are Euler angles of the rotation \f$ \mathbf C_{ab} \f$
     *
     * @param x The x coordinate of the translation
     * @param y The y coordinate of the translation
     * @param z The z coordinate of the translation
     * @param r The orientation roll
     * @param p The orientation pitch
     * @param h The orientation heading
     * 
     * @return a 4x4 transformation matrix.
     */
    Eigen::Matrix4d toTEuler(double x, double y, double z, double r, double p, double h);


    /** 
     * Transform a homogeneous point, \f$ \mathbf v_b,\f$, by the transformation matrix
     * \f$ \mathbf T_{a,b} \f$ and return the transformed point \f$\mathbf v_a \f$ and
     * the jacobian of the transformation with respect to local perturbations of the transformation
     * matrix.
     * 
     * @param T_a_b    The transformation matrix
     * @param v_b      The point being transformed by the matrix.
     * @param out_v_a  Output: The transformed point
     * @param out_B    Output: The jacobian of this transformation with respect to local perturbations of the transformation matrix.
     */
    void transformationAndJacobian(Eigen::Matrix4d const & T_a_b, 
				   Eigen::Vector4d const & v_b, 
				   Eigen::Vector4d & out_v_a, 
				   Eigen::Matrix<double,4,6> & out_B);

    /** 
     * Transform a homogeneous point, \f$ \mathbf v_b,\f$, by the transformation matrix
     * \f$ \mathbf T_{b,a} \f$ and return the transformed point \f$\mathbf v_a \f$ and
     * the jacobian of the transformation with respect to local perturbations of the transformation
     * matrix.
     * 
     * @param T_ba    The transformation matrix
     * @param v_b      The point being transformed by the inverse of the transformation matrix.
     * @param out_v_a  Output: The transformed point
     * @param out_B    Output: The jacobian of this transformation with respect to local perturbations of the transformation matrix.
     */
    void inverseTransformationAndJacobian(Eigen::Matrix4d const & T_ba, 
				   Eigen::Vector4d const & v_b, 
				   Eigen::Vector4d & out_v_a, 
				   Eigen::Matrix<double,4,6> & out_B);

    
    /** 
     * Compute the \f$ \mathbf T_{ba}^\boxtimes \f$ derived in the documentation.
     * 
     * @param T_ba The input transformation matrix
     * 
     * @return \f$ \mathbf T_{ba}^\boxtimes \f$
     */
    Eigen::Matrix<double,6,6> boxTimes(Eigen::Matrix4d const & T_ba);



    // inline void transformationBMatrix(VEC_T const & v, MX_T & out_B)


    // template<typename VEC1_T, typename VEC2_T, typename MX_T>
    // inline void transformationAndJacobian(Eigen::Matrix4d const & T_a_b, VEC1_T const & v_b, VEC2_T & out_v_a, MX_T & out_B)
    // {
    //   ASRL_ASSERT_EQ_DBG(v_b.size(),4,"The input vector must be of length 4");
    //   out_v_a = T_a_b * v_b;
    //   out_B = transformationBMatrix(out_v_a);
    // }

    // template<typename VEC_T, typename MX_T>
    // inline void transformationBMatrix(VEC_T const & v, MX_T & out_B)
    // {
    //   out_B = matrix_type::Zero(4,6);
    //   out_B(0,0) =  v[3];
    //   out_B(1,1) =  v[3];
    //   out_B(2,2) =  v[3];
    //   out_B(0,4) = -v[2];
    //   out_B(0,5) =  v[1];
    //   out_B(1,5) = -v[0];
    //   out_B(1,3) =  v[2];
    //   out_B(2,3) = -v[1];
    //   out_B(2,4) =  v[0];
    // }

    // template<typename VEC_T>
    // inline matrix_type transformationBMatrix(VEC_T const & v)
    // {
    //   matrix_type out_B;
    //   transformationBMatrix(v,out_B);
    //   return out_B;
    // }

    // inline Eigen::Matrix4d toTEuler(vector_type const & dt)
    // {
    //   Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    //   value_type cx = cos(-dt[3]);
    //   value_type sx = sin(-dt[3]);
    //   value_type cy = cos(-dt[4]);
    //   value_type sy = sin(-dt[4]);
    //   value_type cz = cos(-dt[5]);
    //   value_type sz = sin(-dt[5]);
    //   T(0,0) = cz*cy; T(0,1) = -sz*cx+cz*sy*sx; T(0,2) =   sz*sx+cz*sy*cx;
    //   T(1,0) = sz*cy; T(1,1) =  cz*cx+sz*sy*sx; T(1,2) =  -cz*sx+sz*sy*cx;
    //   T(2,0) = -sy;   T(2,1) =           cy*sx; T(2,2) =            cy*cx;

    //   T.col(3).start<3>() = dt.start<3>();
    //   T(3,3) = 1;
    //   return T;
    // }

    // inline vector_type fromTEuler(const Eigen::Matrix4d & T)
    // {
    //   vector_type dt;

    //   dt[0] = T(0,3);
    //   dt[1] = T(1,3);
    //   dt[2] = T(2,3);
    //   dt[4] = asin(T(2,0));
    //   dt[5] = atan2(-T(1,0),T(0,0));
    //   dt[3] = atan2(-T(2,1),T(2,2));

    //   return dt;
    // }

  }}// end namespace asrl::math


#endif /* ASRL_TRANSFORMATIONS_HPP */
