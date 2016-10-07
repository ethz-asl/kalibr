/*
 * Profiling.cpp
 *
 *  Created on: 21.03.2016
 *      Author: Ulrich Schwesinger
 */

// standard includes
#include <vector>
#include <string>

// boost includes
#include <boost/program_options.hpp>

// Schweizer Messer includes
#include <sm/logging.hpp>
#include <sm/timing/Timer.hpp>

// aslam backend includes
#include <aslam/backend/JacobianContainerSparse.hpp>
#include <aslam/backend/JacobianContainerDense.hpp>
#include <aslam/backend/Scalar.hpp>
#include <aslam/backend/GenericMatrixExpression.hpp>
#include <aslam/backend/DesignVariableGenericVector.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/DesignVariableVector.hpp>
#include <aslam/backend/VectorExpressionToGenericMatrixTraits.hpp>
#include <aslam/backend/CacheExpression.hpp>


using namespace std;
using namespace aslam::backend;

template <typename Expression>
void evaluateJacobian(const Expression& expr, JacobianContainer& jc) {
  expr.evaluateJacobians(jc);
}

int main(int argc, char** argv)
{
  try
  {
    string verbosity = "Info";
    vector<string> enableNamedStreams;
    bool disableDefaultStream = false;
    size_t nIterations = 100000;
    size_t updateDvEach = 1;
    bool useSparseJacobianContainer = false;
    bool useCaching = false, noUpdateDv = false;
    bool noDense = false, noSparse = false, noScalar = false,
         noMatrix = false, noError = false, noJacobian = false,
         noCached = false, noNonCached = false;

    namespace po = boost::program_options;
    po::options_description desc("local_planner options");
    desc.add_options()
      ("help", "Produce help message")
      ("verbosity,v", po::value(&verbosity)->default_value(verbosity), "Verbosity string")
      ("disable-default-stream", po::bool_switch(&disableDefaultStream), "Disable default logging stream")
      ("enable-named-streams", po::value< vector<string> >(&enableNamedStreams)->multitoken(), "Enable these named logging streams")
      ("num-iterations", po::value(&nIterations)->default_value(nIterations), "Number of iterations")
      ("use-sparse-jacobian-container", po::bool_switch(&useSparseJacobianContainer), "Use dense/sparse Jacobian container")
      ("use-caching", po::bool_switch(&useCaching), "Use caching expressions")
      ("update-dv-each", po::value(&updateDvEach), "Call update on the design variables each n-th time")
      ("no-dense", po::bool_switch(&noDense), "Don't profile dense Jacobian containers")
      ("no-sparse", po::bool_switch(&noSparse), "Don't profile sparse Jacobian containers")
      ("no-scalar", po::bool_switch(&noScalar), "Don't profile scalar expressions")
      ("no-matrix", po::bool_switch(&noMatrix), "Don't profile matrix expressions")
      ("no-error", po::bool_switch(&noError), "Don't profile error evaluation")
      ("no-jacobian", po::bool_switch(&noJacobian), "Don't profile Jacobian evaluation")
      ("no-cached", po::bool_switch(&noCached), "Don't profile cached expressions")
      ("no-noncached", po::bool_switch(&noNonCached), "Don't profile non-cached expressions")
      ("no-update-dv", po::bool_switch(&noUpdateDv), "Don't update the design variables after each call")
    ;
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    if (vm.count("help")) {
      cout << desc << endl;
      return EXIT_SUCCESS;
    }
    po::notify(vm);
    sm::logging::setLevel(sm::logging::levels::fromString(verbosity));
    for (auto& stream : enableNamedStreams)
      sm::logging::enableNamedStream(stream);
    if (disableDefaultStream)
      sm::logging::disableNamedStream("sm");

    // ********************** //
    //    ScalarExpression    //
    // ********************** //

    {
      Scalar dv(1.0);
      dv.setBlockIndex(0);
      dv.setColumnBase(0);
      dv.setActive(true);
      ScalarExpression expr = dv.toExpression();
      ScalarExpression expr2 = log(expr*expr);
      ScalarExpression cexpr2 = toCacheExpression(expr2);

      Eigen::MatrixXd J = Eigen::MatrixXd::Zero(ScalarExpression::Dimension, dv.minimalDimensions());
      JacobianContainerDense<Eigen::MatrixXd&, ScalarExpression::Dimension> jcDense(J);
      JacobianContainerSparse<ScalarExpression::Dimension> jcSparse(ScalarExpression::Dimension);
      const double dx = 1.0;

      // Test error evaluation non-cached
      if (!noError && !noScalar && !noNonCached) {
        sm::timing::Timer timer("ScalarExpression -- NoCache: Error", false);
        for (size_t i=0; i<nIterations; ++i) {
          expr2.evaluate();
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(&dx, 1);
        }
      }

      // Test error evaluation cached
      if (!noError && !noScalar && !noCached) {
        sm::timing::Timer timer("ScalarExpression -- Cached: Error", false);
        for (size_t i=0; i<nIterations; ++i) {
          cexpr2.evaluate();
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(&dx, 1);
        }
      }

      // Test Jacobian evaluation non-cached, sparse container
      if (!noJacobian && !noSparse && !noScalar && !noNonCached) {
        sm::timing::Timer timer("ScalarExpression -- NoCache/Sparse: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(expr2, jcSparse);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(&dx, 1);
        }
      }

      // Test Jacobian evaluation cached, sparse container
      if (!noJacobian && !noSparse && !noScalar&& !noCached) {
        sm::timing::Timer timer("ScalarExpression -- Cached/Sparse: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(cexpr2, jcSparse);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(&dx, 1);
        }
      }

      // Test Jacobian evaluation non-cached, dense container
      if (!noJacobian && !noDense && !noScalar && !noNonCached) {
        sm::timing::Timer timer("ScalarExpression -- NoCache/Dense: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(expr2, jcDense);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(&dx, 1);
        }
      }

      // Test Jacobian evaluation cached, dense container
      if (!noJacobian && !noDense && !noScalar&& !noCached) {
        sm::timing::Timer timer("ScalarExpression -- Cached/Dense: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(cexpr2, jcDense);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(&dx, 1);
        }
      }
    } // ScalarExpression

    // ***************************** //
    //    GenericMatrixExpression    //
    // ***************************** //
    {
      const int VEC_ROWS = 2;
      typedef GenericMatrixExpression<VEC_ROWS, 1, double> GME;
      typedef DesignVariableGenericVector<VEC_ROWS> DGvec;
      GME::matrix_t vec = GME::matrix_t::Random();
      DGvec dv(vec);
      dv.setActive(true);
      dv.setBlockIndex(0);
      dv.setColumnBase(0);
      GME matExp(&dv);
      const auto cMatExp = toCacheExpression(matExp);
      const auto matExp2 = matExp.transpose()*matExp;
      const auto cMatExp2 = cMatExp.transpose()*cMatExp;

      Eigen::MatrixXd J = Eigen::MatrixXd::Zero(1, dv.minimalDimensions());
      JacobianContainerDense<Eigen::MatrixXd&, 1> jcDense(J);
      JacobianContainerSparse<1> jcSparse(1);
      const GME::matrix_t dx = GME::matrix_t::Ones(dv.minimalDimensions(), 1);

      // Test error evaluation non-cached
      if (!noError && !noMatrix && !noNonCached) {
        sm::timing::Timer timer("GenericMatrixExpression -- No cache: Error", false);
        for (size_t i=0; i<nIterations; ++i) {
          matExp2.evaluate();
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(dx.data(), dx.size());
        }
      }

      // Test error evaluation cached
      if (!noError && !noMatrix&& !noCached) {
        sm::timing::Timer timer("GenericMatrixExpression -- Cached: Error", false);
        for (size_t i=0; i<nIterations; ++i) {
          cMatExp2.evaluate();
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(dx.data(), dx.size());
        }
      }


      // Test Jacobian evaluation non-cached, sparse container
      if (!noJacobian && !noSparse && !noMatrix && !noNonCached) {
        sm::timing::Timer timer("GenericMatrixExpression -- NoCache/Sparse: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(matExp2, jcSparse);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(dx.data(), dx.size());
        }
      }

      // Test Jacobian evaluation cached, sparse container
      if (!noJacobian && !noSparse && !noMatrix&& !noCached) {
        sm::timing::Timer timer("GenericMatrixExpression -- Cached/Sparse: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(cMatExp2, jcSparse);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(dx.data(), dx.size());
        }
      }

      // Test Jacobian evaluation non-cached, dense container
      if (!noJacobian && !noDense && !noMatrix && !noNonCached) {
        sm::timing::Timer timer("GenericMatrixExpression -- NoCache/Dense: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(matExp2, jcDense);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(dx.data(), dx.size());
        }
      }

      // Test Jacobian evaluation cached, dense container
      if (!noJacobian && !noDense && !noMatrix&& !noCached) {
        sm::timing::Timer timer("GenericMatrixExpression -- Cached/Dense: Jacobian", false);
        for (size_t i=0; i<nIterations; ++i) {
          evaluateJacobian(cMatExp2, jcDense);
          if (!noUpdateDv && i % updateDvEach == 0) dv.update(dx.data(), dx.size());
        }
      }
    } // GenericMatrixExpression

    sm::timing::Timing::print(cout, sm::timing::SortType::SORT_BY_TOTAL);

  }
  catch (exception& e)
  {
    SM_FATAL_STREAM(e.what());
    return EXIT_FAILURE;
  }

}

