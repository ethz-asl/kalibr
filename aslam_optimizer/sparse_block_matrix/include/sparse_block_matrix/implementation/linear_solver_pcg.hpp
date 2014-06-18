// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

// helpers for doing fixed or variable size operations on the matrices

namespace sparse_block_matrix {
  template<typename MatrixType>
  inline void pcg_axy(const MatrixType& A, const Eigen::VectorXd& x, int xoff, Eigen::VectorXd& y, int yoff)
  {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) = A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
  }

  template<>
  inline void pcg_axy(const Eigen::MatrixXd& A, const Eigen::VectorXd& x, int xoff, Eigen::VectorXd& y, int yoff)
  {
    y.segment(yoff, A.rows()) = A * x.segment(xoff, A.cols());
  }

  template<typename MatrixType>
  inline void pcg_axpy(const MatrixType& A, const Eigen::VectorXd& x, int xoff, Eigen::VectorXd& y, int yoff)
  {
    y.segment<MatrixType::RowsAtCompileTime>(yoff) += A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
  }

  template<>
  inline void pcg_axpy(const Eigen::MatrixXd& A, const Eigen::VectorXd& x, int xoff, Eigen::VectorXd& y, int yoff)
  {
    y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
  }

  template<typename MatrixType>
  inline void pcg_atxpy(const MatrixType& A, const Eigen::VectorXd& x, int xoff, Eigen::VectorXd& y, int yoff)
  {
    y.segment<MatrixType::ColsAtCompileTime>(yoff) += A.transpose() * x.segment<MatrixType::RowsAtCompileTime>(xoff);
  }

  template<>
  inline void pcg_atxpy(const Eigen::MatrixXd& A, const Eigen::VectorXd& x, int xoff, Eigen::VectorXd& y, int yoff)
  {
    y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
  }
}
// helpers end

template <typename MatrixType>
bool LinearSolverPCG<MatrixType>::solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b)
{
  const bool indexRequired = _indices.size() == 0;
  _diag.clear();
  _J.clear();

  // put the block matrix once in a linear structure, makes mult faster
  int colIdx = 0;
  for (size_t i = 0; i < A.blockCols().size(); ++i){
    const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col = A.blockCols()[i];
    if (col.size() > 0) {
      typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;
      for (it = col.begin(); it != col.end(); ++it) {
        if (it->first == (int)i) { // only the upper triangular block is needed
          _diag.push_back(it->second);
          _J.push_back(it->second->inverse());
          break;
        }
        if (indexRequired) {
          _indices.push_back(std::make_pair(it->first > 0 ? A.rowBlockIndices()[it->first-1] : 0, colIdx));
          _sparseMat.push_back(it->second);
        }

      }
    }
    colIdx = A.colBlockIndices()[i];
  }

  int n = A.rows();
  Eigen::Map<Eigen::VectorXd> xvec(x, A.cols());
  const Eigen::Map<Eigen::VectorXd> bvec(b, n);
  xvec.setZero();

  Eigen::VectorXd r, d, q, s;
  d.setZero(n);
  q.setZero(n);
  s.setZero(n);

  r = bvec;
  multDiag(A.colBlockIndices(), _J, r, d);
  double dn = r.dot(d);
  double d0 = _tolerance * dn;

  if (_absoluteTolerance) {
    if (_residual > 0.0 && _residual > d0)
      d0 = _residual;
  }

  int maxIter = _maxIter < 0 ? A.rows() : _maxIter;

  int iteration;
  for (iteration = 0; iteration < maxIter; ++iteration) {
    if (_verbose)
      std::cerr << "residual[" << iteration << "]: " << dn << std::endl;
    if (dn <= d0)
      break;	// done
    mult(A.colBlockIndices(), d, q);
    double a = dn / d.dot(q);
    xvec += a*d;
    // TODO: reset residual here every 50 iterations
    r -= a*q;
    multDiag(A.colBlockIndices(), _J, r, s);
    double dold = dn;
    dn = r.dot(s);
    double ba = dn / dold;
    d = s + ba*d;
  }
  //std::cerr << "residual[" << iteration << "]: " << dn << std::endl;
  _residual = 0.5 * dn;
  
  return true;
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::multDiag(const std::vector<int>& colBlockIndices, MatrixVector& A, const Eigen::VectorXd& src, Eigen::VectorXd& dest)
{
  int row = 0;
  for (size_t i = 0; i < A.size(); ++i) {
    pcg_axy(A[i], src, row, dest, row);
    row = colBlockIndices[i];
  }
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::multDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A, const Eigen::VectorXd& src, Eigen::VectorXd& dest)
{
  int row = 0;
  for (size_t i = 0; i < A.size(); ++i) {
    pcg_axy(*A[i], src, row, dest, row);
    row = colBlockIndices[i];
  }
}

template <typename MatrixType>
void LinearSolverPCG<MatrixType>::mult(const std::vector<int>& colBlockIndices, const Eigen::VectorXd& src, Eigen::VectorXd& dest)
{
  // first multiply with the diagonal
  multDiag(colBlockIndices, _diag, src, dest);

  // now multiply with the upper triangular block
  for (size_t i = 0; i < _sparseMat.size(); ++i) {
    const int& srcOffset = _indices[i].second;
    const int& destOffsetT = srcOffset;
    const int& destOffset = _indices[i].first;
    const int& srcOffsetT = destOffset;

    const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a = _sparseMat[i];
    // destVec += *a * srcVec (according to the sub-vector parts)
    pcg_axpy(*a, src, srcOffset, dest, destOffset);
    // destVec += *a.transpose() * srcVec (according to the sub-vector parts)
    pcg_atxpy(*a, src, srcOffsetT, dest, destOffsetT);
  }
}
