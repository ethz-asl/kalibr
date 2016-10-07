#ifndef ASLAM_BACKEND_QR_SOLUTION_HPP
#define ASLAM_BACKEND_QR_SOLUTION_HPP


class QrSolution {
public:
  QrSolution();
  virtual ~QrSolution();

  void initMatrixStructure(const std::vector<DesignVariable*>& dvs, const std::vector<ErrorTerm*>& errors) = 0;
  void buildJacobianMatrices(const std::vector<ErrorTerm*>& errors) = 0;
};


#endif /* ASLAM_BACKEND_QR_SOLUTION_HPP */
