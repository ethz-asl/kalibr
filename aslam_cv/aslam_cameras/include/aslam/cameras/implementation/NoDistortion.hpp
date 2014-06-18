namespace aslam {
namespace cameras {

template<typename DERIVED_Y>
void NoDistortion::distort(const Eigen::MatrixBase<DERIVED_Y> & /* y */) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
}

template<typename DERIVED_Y, typename DERIVED_JY>
void NoDistortion::distort(
    const Eigen::MatrixBase<DERIVED_Y> & /* y */,
    const Eigen::MatrixBase<DERIVED_JY> & outJyConst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);
  Eigen::MatrixBase<DERIVED_JY> & outJy = const_cast<Eigen::MatrixBase<
      DERIVED_JY> &>(outJyConst);
  outJy.derived().resize(2, 2);
  outJy.setIdentity();
}

template<typename DERIVED>
void NoDistortion::undistort(const Eigen::MatrixBase<DERIVED> & /* y */) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);
}

template<typename DERIVED, typename DERIVED_JY>
void NoDistortion::undistort(
    const Eigen::MatrixBase<DERIVED> & /* y */,
    const Eigen::MatrixBase<DERIVED_JY> & outJyConst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED>, 2);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_JY>, 2, 2);
  Eigen::MatrixBase<DERIVED_JY> & outJy = const_cast<Eigen::MatrixBase<
      DERIVED_JY> &>(outJyConst);
  outJy.derived().resize(2, 2);
  outJy.setIdentity();
}

template<typename DERIVED_Y, typename DERIVED_JD>
void NoDistortion::distortParameterJacobian(
    const Eigen::MatrixBase<DERIVED_Y> & /* imageY */,
    const Eigen::MatrixBase<DERIVED_JD> & outJdConst) const {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE_OR_DYNAMIC(
      Eigen::MatrixBase<DERIVED_Y>, 2);
  // EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE_OR_DYNAMIC(Eigen::MatrixBase<DERIVED_JD>, 2, 2);
  Eigen::MatrixBase<DERIVED_JD> & outJd = const_cast<Eigen::MatrixBase<
      DERIVED_JD> &>(outJdConst);
  outJd.derived().resize(2, 0);
  //outJd.setIdentity();
}

}  // namespace cameras
}  // namespace aslam
