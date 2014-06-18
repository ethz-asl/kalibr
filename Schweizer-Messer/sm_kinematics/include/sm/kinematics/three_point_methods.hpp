#ifndef SM_THREE_POINT_METHODS
#define SM_THREE_POINT_METHODS

#include <Eigen/Core>

namespace sm { namespace kinematics {

    /** 
     * Use a three-point SVD method to compute the transformation between two sets of points.
     * 
     * @param p0 a 3xK matrix where each column is a point expressed in frame zero
     * @param p1 a 3xK matrix where each column is a point expressed in frame one
     * @return the 4x4 transformation matrix that is the best fit transformation \f$p_0 = \mathbf T_{0,1} p_1\f$ between the two point sets.
     */
    Eigen::Matrix4d threePointSvd(Eigen::MatrixXd const & p0, Eigen::MatrixXd const & p1);

    /** 
     * Compute the rotation between two sets of unit vectors using Davenport's q-method
     * P. Davenport, “A vector approach to the algebra of rotations with applications,” 
     * NASA Technical Note TN D-4696, Aug. 1968.
     * 
     * @param p0 a 3xK matrix where each column is a unit vector expressed in frame zero
     * @param p1 a 3xK matrix where each column is a unit vector expressed in frame one
     * @param w  a Kx1 vector where each entry is a scalar weight for the corresponding pair of unit vectors
     * 
     * @return the 3x3 transformation matrix that is the best fit rotation \f$p_0 = \mathbf C_{0,1} p_1\f$ between the two point sets.
     */
    Eigen::Matrix3d qMethod(Eigen::MatrixXd const & p0, Eigen::MatrixXd const & p1, const Eigen::VectorXd & w);

    /** 
     * Compute the rotation between two sets of unit vectors using Davenport's q-method
     * P. Davenport, “A vector approach to the algebra of rotations with applications,” 
     * NASA Technical Note TN D-4696, Aug. 1968.
     * 
     * In this overloaded version, all points are weighted equally
     *
     * @param p0 a 3xK matrix where each column is a unit vector expressed in frame zero
     * @param p1 a 3xK matrix where each column is a unit vector expressed in frame one
     * 
     * @return the 3x3 transformation matrix that is the best fit rotation \f$p_0 = \mathbf C_{0,1} p_1\f$ between the two point sets.
     */
    Eigen::Matrix3d qMethod(Eigen::MatrixXd const & p0, Eigen::MatrixXd const & p1);
    

  }} // namespace sm::kinematics

#endif /* SM_THREE_POINT_METHODS */
