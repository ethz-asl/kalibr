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

#include "linear_solver_csparse.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/solver_factory.h"
#include "g2o/core/graph_optimizer_sparse.h"

#include "g2o/stuff/macros.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_CSPARSE(s, p, l, blockorder) \
  if (1) { \
    std::cerr << "# Using CSparse poseDim " << p << " landMarkDim " << l << " blockordering " << blockorder << std::endl; \
    LinearSolverCSparse< DIM_TO_SOLVER(p, l)::PoseMatrixType >* linearSolver = new LinearSolverCSparse<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
    linearSolver->setBlockOrdering(blockorder); \
    s = new DIM_TO_SOLVER(p, l)(opt, linearSolver); \
  } else (void)0

namespace g2o {

  /**
   * helper function for allocating
   */
  static Solver* createSolver(SparseOptimizer* opt, const std::string& solverName)
  {
    g2o::Solver* s = 0;

    if (solverName == "var") {
      ALLOC_CSPARSE(s, -1, -1, false);
    }
    else if (solverName == "fix3_2") {
      ALLOC_CSPARSE(s, 3, 2, true);
    }
    else if (solverName == "fix6_3") {
      ALLOC_CSPARSE(s, 6, 3, true);
    }
    else if (solverName == "fix7_3") {
      ALLOC_CSPARSE(s, 7, 3, true);
    }
    else if (solverName == "fix3_2_scalar") {
      ALLOC_CSPARSE(s, 3, 2, false);
    }
    else if (solverName == "fix6_3_scalar") {
      ALLOC_CSPARSE(s, 6, 3, false);
    }
    else if (solverName == "fix7_3_scalar") {
      ALLOC_CSPARSE(s, 7, 3, false);
    }

    return s;
  }

  class CSparseSolverCreator : public AbstractSolverCreator
  {
    public:
      CSparseSolverCreator(const SolverProperty& p) : AbstractSolverCreator(p) {}
      virtual Solver* construct(SparseOptimizer* optimizer)
      {
        return createSolver(optimizer, property().name);
      }
  };

  void G2O_ATTRIBUTE_CONSTRUCTOR init_solver_csparse()
  {
    SolverFactory* factory = SolverFactory::instance();
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("var", "Cholesky solver using CSparse (variable blocksize)", "CSparse", false, -1, -1)));
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("fix3_2", "Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 3, 2)));
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("fix6_3", "Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 6, 3)));
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("fix7_3", "Cholesky solver using CSparse (fixed blocksize)", "CSparse", true, 7, 3)));
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("fix3_2_scalar", "Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 3, 2)));
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("fix6_3_scalar", "Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 6, 3)));
    factory->registerSolver(new CSparseSolverCreator(SolverProperty("fix7_3_scalar", "Cholesky solver using CSparse (fixed blocksize, scalar ordering)", "CSparse", true, 7, 3)));
  }

} // end namespace
