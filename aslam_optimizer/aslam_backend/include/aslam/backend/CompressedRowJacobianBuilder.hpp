#ifndef ASLAM_BACKEND_COMPRESSED_ROW_JACOBIAN_BUILDER_HPP
#define ASLAM_BACKEND_COMPRESSED_ROW_JACOBIAN_BUILDER_HPP

#include "JacobianBuilder.hpp"

namespace aslam {
  namespace backend {

    class CompressedRowJacobianBuilder : public JacobianBuilder {
    public:
      CompressedRowJacobianBuilder();
      virtual ~CompressedRowJacobianBuilder();
      void initMatrixStructure(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors);
      void buildJacobian(const std::vector<ErrorTerm*>& errors);
      cholmod_sparse getJacobian();


    };

  } // namespace backend
} // namespace aslam


#endif /* ASLAM_BACKEND_COMPRESSED_ROW_JACOBIAN_BUILDER_HPP */
