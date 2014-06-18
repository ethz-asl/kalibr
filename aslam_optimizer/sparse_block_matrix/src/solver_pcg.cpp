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

#include "linear_solver_pcg.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/solver_factory.h"

#define DIM_TO_SOLVER(p, l) BlockSolver< BlockSolverTraits<p, l> >

#define ALLOC_PCG(s, p, l) \
  if (1) { \
      std::cerr << "# Using PCG poseDim " << p << " landMarkDim " << l << std::endl; \
      DIM_TO_SOLVER(p, l)::LinearSolverType* linearSolver = new LinearSolverPCG<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
      s = new DIM_TO_SOLVER(p, l)(opt, linearSolver); \
  } else (void)0

namespace g2o {

  static Solver* createSolver(SparseOptimizer* opt, const std::string& solverName)
  {
    g2o::Solver* s = 0;

    if (solverName == "pcg") {
      ALLOC_PCG(s, -1, -1);
    }
    else if (solverName == "pcg3_2") {
      ALLOC_PCG(s, 3, 2);
    }
    else if (solverName == "pcg6_3") {
      ALLOC_PCG(s, 6, 3);
    }
    else if (solverName == "pcg7_3") {
      ALLOC_PCG(s, 7, 3);
    }

    return s;
  }

  class PCGSolverCreator : public AbstractSolverCreator
  {
    public:
      PCGSolverCreator(const SolverProperty& p) : AbstractSolverCreator(p) {}
      virtual Solver* construct(SparseOptimizer* optimizer)
      {
        return createSolver(optimizer, property().name);
      }
  };

  void __attribute__ ((constructor)) init_solver_csparse()
  {
    SolverFactory* factory = SolverFactory::instance();
    factory->registerSolver(new PCGSolverCreator(SolverProperty("pcg", "PCG solver using block-Jacobi pre-conditioner (variable blocksize)", "PCG", false, -1, -1)));
    factory->registerSolver(new PCGSolverCreator(SolverProperty("pcg3_2", "PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 3, 2)));
    factory->registerSolver(new PCGSolverCreator(SolverProperty("pcg6_3", "PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 6, 3)));
    factory->registerSolver(new PCGSolverCreator(SolverProperty("pcg7_3", "PCG solver using block-Jacobi pre-conditioner (fixed blocksize)", "PCG", true, 7, 3)));
  }

}
