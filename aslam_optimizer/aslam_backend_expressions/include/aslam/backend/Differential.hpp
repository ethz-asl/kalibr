#ifndef ASLAM_BACKEND_DIFFERENTIAL_HPP
#define ASLAM_BACKEND_DIFFERENTIAL_HPP

#include "BasisMatrixFunctor.hpp"

namespace aslam {
namespace backend {
template<typename TDomain, typename TScalar>
class Differential {
 public:
  typedef Eigen::Matrix<TScalar, Eigen::Dynamic, Eigen::Dynamic> dyn_matrix_t;
  typedef dyn_matrix_t matrix_t;
  typedef Eigen::Matrix<TScalar, Eigen::Dynamic, 1> dyn_vector_t;
  typedef dyn_vector_t result_vector_t;
  typedef Differential<dyn_vector_t, TScalar> composed_with_matrix_t;
  typedef TDomain domain_t;
  typedef TScalar scalar_t;

  inline domain_t getDomainBasisVectorByIndex(Eigen::DenseIndex index) const {
    static_assert(domain_t::RowsAtCompileTime != Eigen::Dynamic && domain_t::ColsAtCompileTime != Eigen::Dynamic, "dynamic dimension is not supported yet");
    return domain_t::NullaryExpr(domain_t::RowsAtCompileTime, domain_t::ColsAtCompileTime, internal::BasisMatrixNullaryFunctor(index, index % domain_t::RowsAtCompileTime, index / domain_t::RowsAtCompileTime));
  }

  typedef Differential<TDomain, TScalar> self_t;

  virtual ~Differential() {
  }

  virtual void applyBasisVectorInto(int index, result_vector_t & result) const = 0;
  virtual void applyInto(const domain_t & tangent_vector, result_vector_t & result) const = 0;
  virtual void convertIntoMatrix(int rows, int cols, dyn_matrix_t & result) const = 0;

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv) const = 0;
  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv, const dyn_matrix_t & jacobian) const = 0;
};

template<typename TDomain, typename TScalar>
class NullDifferential : public Differential<TDomain, TScalar> {
 public:
  typedef Differential<TDomain, TScalar> base_t;
  typedef NullDifferential<TDomain, TScalar> self_t;
  virtual ~NullDifferential() {
  }

  virtual void applyBasisVectorInto(int /* index */, typename base_t::result_vector_t & result) const {
    result.setZero();
  }

  virtual void applyInto(const typename base_t::domain_t & /* tangent_vector */, typename base_t::result_vector_t & result) const {
    result.setZero();
  }

  virtual void addToJacobianContainer(JacobianContainer & /* jc */, const DesignVariable * /* dv */) const {
  }

  virtual void addToJacobianContainer(JacobianContainer & /* jc */, const DesignVariable * /* dv */, const typename base_t::dyn_matrix_t & /* jacobian */) const {
  }

  virtual void convertIntoMatrix(int rows, int cols, typename base_t::dyn_matrix_t & result) const {
    result.setZero(rows, cols);
  }
};

template<typename TDomain, typename TScalar>
class IdentityDifferential : public Differential<TDomain, TScalar> {
 public:
  typedef Differential<TDomain, TScalar> base_t;
  typedef IdentityDifferential<TDomain, TScalar> self_t;
  virtual ~IdentityDifferential() {
  }

  virtual void applyBasisVectorInto(int index, typename base_t::result_vector_t & result) const {
    result.setZero();
    result[index] = 1;
  }

  virtual void applyInto(const typename base_t::domain_t & tangent_vector, typename base_t::result_vector_t & result) const {
    result = tangent_vector;
  }

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv) const {
    jc.add(const_cast<DesignVariable *>(dv), base_t::matrix_t::Identity(jc.rows(), jc.rows()));
  }

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv, const typename base_t::dyn_matrix_t & jacobian) const {
    jc.add(const_cast<DesignVariable *>(dv), jacobian);
  }

  virtual void convertIntoMatrix(int rows, int cols, typename base_t::dyn_matrix_t & result) const {
    result.setIdentity(rows, cols);
  }
};

template<typename TNextDomain, typename TScalar, typename TMatrix, int IDomainRows = Eigen::Dynamic>
class ComposedMatrixDifferential;

namespace internal {
template<typename TDiff, int ICols = TDiff::domain_t::ColsAtCompileTime>
struct DifferentialCalculator {
  typedef typename TDiff::matrix_t matrix_t;
  typedef NullDifferential<typename TDiff::dyn_matrix_t, typename TDiff::scalar_t> compose_result_t;

  inline static void addToJacobianByApplication(const TDiff & /* diff */, JacobianContainer& /* jc */, const DesignVariable* /* dv */) {
    throw std::runtime_error("this number of columns is not supported here!");
  }

  inline static matrix_t calcJacobianByApplication(int /* rows */, int /* cols */, const TDiff& /* diff */) {
    throw std::runtime_error("this number of columns is not supported here!");
  }

  inline static compose_result_t compose(const TDiff & /* diff */, const matrix_t & /* jacobian */) {
    throw std::runtime_error("this number of columns is not supported here!");
  }
};

template<typename TDiff>
struct DifferentialCalculator<TDiff, 1> {
  typedef typename TDiff::scalar_t scalar_t;
  typedef typename TDiff::matrix_t matrix_t;
  typedef typename TDiff::dyn_vector_t dyn_vector_t;
  typedef typename TDiff::domain_t domain_t;
  typedef ComposedMatrixDifferential<domain_t, scalar_t, const matrix_t &> compose_result_t;

  inline static void addToJacobianByApplication(const TDiff & diff, JacobianContainer& jc, const DesignVariable* dv) {
    jc.add(const_cast<DesignVariable *>(dv), calcJacobianByApplication(jc.rows(), dv->minimalDimensions(), diff));
  }

  inline static compose_result_t compose(const TDiff & diff, const matrix_t & jacobian) {
    return compose_result_t(jacobian, diff);
  }

  inline static matrix_t calcJacobianByApplication(int rows, int cols, const TDiff& diff) {
    matrix_t result(rows, cols);  //TODO check
    dyn_vector_t tmpVec(rows);  //TODO check
    for (int i = 0; i < cols; i++) {
      diff.applyBasisVectorInto(i, tmpVec);
      result.col(i) = tmpVec;
    }
    return result;
  }
};
template<typename TDiff>
struct DifferentialCalculator<TDiff, Eigen::Dynamic> {
  typedef typename TDiff::scalar_t scalar_t;
  typedef typename TDiff::matrix_t matrix_t;
  typedef typename TDiff::domain_t domain_t;

  inline static void addToJacobianByApplication(const TDiff & diff, JacobianContainer& jc, const DesignVariable* dv) {
    //TODO runtime check domain columns (there has to be one)
    DifferentialCalculator<TDiff, 1>::addToJacobianByApplication(jc, dv, diff);
  }
  inline static matrix_t calcJacobianByApplication(int rows, int cols, const TDiff& diff) {
    //TODO runtime check domain columns (there has to be one)
    return DifferentialCalculator<TDiff, 1>::calcJacobianByApplication(rows, cols, diff);
  }

  inline static typename DifferentialCalculator<TDiff, 1>::compose_result_t compose(const TDiff & diff, const matrix_t & jacobian) {
    //TODO runtime check domain columns (there has to be one)
    return DifferentialCalculator<TDiff, 1>::compose(diff, jacobian);
  }
};

}  // namespace internal

template<typename TDomain, typename TNextDomain, typename TScalar, typename DERIVED>
class ComposedDifferential : public Differential<TDomain, TScalar> {
 public:
  typedef Differential<TDomain, TScalar> base_t;
  typedef Differential<TNextDomain, TScalar> next_differential_t;
  typedef ComposedDifferential<TDomain, TNextDomain, TScalar, DERIVED> self_t;
  typedef TNextDomain next_domain_t;
  typedef typename base_t::matrix_t matrix_t;
  typedef typename base_t::dyn_matrix_t dyn_matrix_t;
  typedef typename base_t::dyn_vector_t dyn_vector_t;
  typedef typename base_t::domain_t domain_t;
  typedef typename base_t::composed_with_matrix_t composed_with_matrix_t;
  typedef typename base_t::result_vector_t result_vector_t;

  ComposedDifferential(const next_differential_t & next_differential)
      : _next_differential(next_differential) {
  }

  virtual ~ComposedDifferential() {
  }

  inline DERIVED & getDerived() {
    return static_cast<DERIVED&>(*this);
  }

  inline const DERIVED & getDerived() const {
    return static_cast<const DERIVED&>(*this);
  }

  inline typename internal::DifferentialCalculator<base_t>::compose_result_t compose(const matrix_t & jacobian) const {
    return internal::DifferentialCalculator<DERIVED>::compose(getDerived(), jacobian);
  }

  inline next_domain_t apply(const domain_t & tangent_vector) const {
    throw std::runtime_error("this method has to be statically overridden!");
  }

  inline next_domain_t applyBasisVector(int index) const {
    return getDerived().apply(getDerived().getDomainBasisVectorByIndex(index));
  }

  virtual void applyBasisVectorInto(int index, result_vector_t & result) const {
    _next_differential.applyInto(getDerived().applyBasisVector(index), result);
  }

  virtual void applyInto(const domain_t & tangent_vector, result_vector_t & result) const {
    _next_differential.applyInto(getDerived().apply(tangent_vector), result);
  }

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv) const {
    internal::DifferentialCalculator<DERIVED>::addToJacobianByApplication(getDerived(), jc, dv);
  }

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv, const matrix_t & jacobian) const {
    getDerived().compose(jacobian).addToJacobianContainer(jc, dv);
  }

  virtual void convertIntoMatrix(int rows, int cols, dyn_matrix_t & result) const {
    result = internal::DifferentialCalculator<DERIVED>::calcJacobianByApplication(rows, cols, getDerived());
  }

 protected:
  const next_differential_t & _next_differential;
};

template<typename TScalar, typename TMatrix, int ICols = TMatrix::ColsAtCompileTime>
class MatrixDifferential : public Differential<Eigen::Matrix<TScalar, ICols, 1>, TScalar> {
 protected:
  TMatrix _mat;
 public:
  typedef Differential<Eigen::Matrix<TScalar, ICols, 1>, TScalar> base_t;
  typedef MatrixDifferential<TScalar, TMatrix> self_t;

  inline MatrixDifferential(TMatrix mat)
      : _mat(mat) {
  }

  virtual ~MatrixDifferential() {
  }

  virtual void applyBasisVectorInto(int index, typename base_t::result_vector_t & result) const {
    result = _mat.col(index);
  }

  virtual void applyInto(const typename base_t::domain_t & tangent_vector, typename base_t::result_vector_t & result) const {
    result = _mat * tangent_vector;
  }

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv) const {
    jc.add(const_cast<DesignVariable *>(dv), _mat.template cast<double>());
  }

  virtual void addToJacobianContainer(JacobianContainer & jc, const DesignVariable * dv, const typename base_t::dyn_matrix_t & jacobian) const {
    jc.add(const_cast<DesignVariable *>(dv), (_mat * jacobian).template cast<double>());
  }

  virtual void convertIntoMatrix(int /* rows */, int /* cols */, typename base_t::dyn_matrix_t & result) const {
    //TODO assertions
    result = _mat;
  }
};

/**
 * \brief represents a composition of a TNextDomain-differential after a TMatrix differential.
 */
template<typename TNextDomain, typename TScalar, typename TMatrix, int IDomainRows>
class ComposedMatrixDifferential : public ComposedDifferential<Eigen::Matrix<TScalar, IDomainRows, 1>, TNextDomain, TScalar, ComposedMatrixDifferential<TNextDomain, TScalar, TMatrix, IDomainRows> > {
 protected:
  TMatrix _mat;
 public:
  typedef ComposedMatrixDifferential<TNextDomain, TScalar, TMatrix, IDomainRows> self_t;
  typedef ComposedDifferential<Eigen::Matrix<TScalar, IDomainRows, 1>, TNextDomain, TScalar, self_t> base_t;
  typedef typename base_t::matrix_t matrix_t;
  typedef typename base_t::dyn_matrix_t dyn_matrix_t;
  typedef typename base_t::dyn_vector_t dyn_vector_t;
  typedef ComposedMatrixDifferential<TNextDomain, TScalar, Eigen::Matrix<TScalar, Eigen::internal::remove_all<TMatrix>::type::RowsAtCompileTime, matrix_t::ColsAtCompileTime> > compose_result_t;

  inline ComposedMatrixDifferential(TMatrix mat, const typename base_t::next_differential_t & next_differential)
      : base_t(next_differential),
        _mat(mat) {
  }

  virtual ~ComposedMatrixDifferential() {
  }

  inline compose_result_t compose(const matrix_t & jacobian) const {
    return compose_result_t(_mat * jacobian, this->_next_differential);
  }

  inline typename base_t::next_domain_t apply(const typename base_t::domain_t & tangent_vector) const {
    return this->_mat * tangent_vector;
  }

  inline typename base_t::next_domain_t applyBasisVector(int index) const {
    return _mat.col(index);
  }

  virtual void convertIntoMatrix(int /* rows */, int /* cols */, dyn_matrix_t & result) const {
    //TODO assertions
    result = _mat;
  }
};

}  // namespace backend
}  // namespace aslam

#endif /* ASLAM_BACKEND_DIFFERENTIAL_HPP */
