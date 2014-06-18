/*!
 * \file ErrorTermEuclidean.hpp
 *
 * \author diemf
 *
 * A Pose ErrorTerm
 */
#ifndef ASLAM_BACKEND_ERROR_EUCLIDEAN_HPP
#define ASLAM_BACKEND_ERROR_EUCLIDEAN_HPP

#include <aslam/backend/ErrorTerm.hpp>
#include <aslam/backend/EuclideanExpression.hpp>
#include <Eigen/Core>

namespace aslam {
  namespace backend {

    /*!
    * \class ErrorTermEuclidean
    *
    * \brief An ErrorTerm implementation for the deviation of a translation pose wrt. a prior
    */

    class ErrorTermEuclidean : public aslam::backend::ErrorTermFs<3>
    {
    public:
      // This is important. The superclass holds some fixed-sized Eigen types
      // For more information, see:
      // http://eigen.tuxfamily.org/dox-devel/TopicStructHavingEigenMembers.html
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ErrorTermEuclidean(const aslam::backend::EuclideanExpression& t, const Eigen::Vector3d& prior, const Eigen::Matrix<double,3,3>& N, int debug=0);
      ErrorTermEuclidean(const aslam::backend::EuclideanExpression& t, const Eigen::Vector3d& prior, double weight, int debug=0);

      virtual ~ErrorTermEuclidean();

    protected:
      /// This is the interface required by ErrorTermFs<>

      /// \brief evaluate the error term and return the weighted squared error e^T invR e
      virtual double evaluateErrorImplementation();

      /// \brief evaluate the jacobian
      virtual void evaluateJacobiansImplementation(JacobianContainer & J);

    private:
      aslam::backend::EuclideanExpression _t;
      Eigen::Vector3d _prior;
      int _debug;
    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_ERROR_EUCLIDEAN_HPP */
