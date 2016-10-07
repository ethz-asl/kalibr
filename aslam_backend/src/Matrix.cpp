#include <aslam/backend/Matrix.hpp>

namespace aslam {
  namespace backend {

    Matrix::Matrix()
    {
    }

    Matrix::~Matrix()
    {
    }

    Eigen::MatrixXd Matrix::toDense() const
    {
      Eigen::MatrixXd M;
      toDenseInto(M);
      return M;
    }

    void Matrix::toDenseInto(Eigen::MatrixXd& outM) const
    {
      outM.resize(rows(), cols());
      for (size_t c = 0; c < cols(); ++c) {
        for (size_t r = 0; r < rows(); ++r) {
          outM(r, c) = (*this)(r, c);
        }
      }
    }

    void Matrix::write(std::ostream& stream) const {
      for (size_t r = 0; r < rows(); ++r) {
        for (size_t c = 0; c < cols(); ++c) {
          stream.width(10);
          stream.setf(std::ios::fixed, std::ios::floatfield); // floatfield set to fixed
          stream.precision(5);
          stream << (*this)(r, c) << " ";
        }
        stream << std::endl;
      }
    }

  std::ostream& operator<<(std::ostream& os, const aslam::backend::Matrix& ccm)
  {
    ccm.write(os);
    return os;
  }

  } // namespace backend
} // namespace aslam


