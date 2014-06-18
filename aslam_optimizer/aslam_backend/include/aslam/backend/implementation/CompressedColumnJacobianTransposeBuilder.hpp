#include <aslam/backend/CompressedColumnJacobianTransposeBuilder.hpp>
#include <boost/thread.hpp>

namespace aslam {
  namespace backend {

    template<typename I>
    CompressedColumnJacobianTransposeBuilder<I>::CompressedColumnJacobianTransposeBuilder() : _isInitialized(false)
    {
    }

    template<typename I>
    CompressedColumnJacobianTransposeBuilder<I>::~CompressedColumnJacobianTransposeBuilder()
    {
    }


    template<typename I>
    void CompressedColumnJacobianTransposeBuilder<I>::initMatrixStructure(const std::vector<DesignVariable*> & dvs, const std::vector<ErrorTerm*> & errors)
    {
      _jacobianPointers.clear();
      _jacobianPointers.resize(errors.size());
      _J_transpose.clear();
      _J.reset();
      size_t nnz = 0;
      size_t num_cols = 0;
      std::vector<ErrorTerm*>::const_iterator eit = errors.begin();
      for (; eit != errors.end(); ++eit) {
        size_t D = (*eit)->dimension();
        num_cols += D;
        const std::vector<DesignVariable*> & dxx = (*eit)->designVariables();
        //std::cout << "Error term with dimension " << D << " and " << dxx.size() << " dvs " << std::endl;
        std::vector<DesignVariable*>::const_iterator dit = dxx.begin();
        for (; dit != dxx.end(); ++dit) {
          nnz += D * (*dit)->minimalDimensions();
        }
      }
      // Add room for a diagonal
      nnz += dvs.back()->columnBase() + dvs.back()->minimalDimensions();
      // std::cout << "Matrix storage: " << ((double)nnz * 64.0 * 1e-9) << " GB\n";
      // Initialize the matrix to be the right size.
      _J_transpose.init(dvs.back()->columnBase() + dvs.back()->minimalDimensions(), 0, nnz, num_cols);
      std::vector<ErrorTerm*>::const_iterator it = errors.begin();
      int i = 0;
      size_t eRow = 0;
      for (; it != errors.end(); ++it) {
        _jacobianPointers[i++].set(_J_transpose.appendErrorJacobiansSymbolic(*(*it)), *it, eRow);
        //std::cout << "Error " << i << "/" << errors.size() << ", Jacobian has " << ((double)_J_transpose.values().size() * (double)64 * (1e-9))  << " GB of data\n";
        eRow += (*it)->dimension();
      }
      //_e.resize(eRow);
      _isInitialized = true;
    }



    template<typename I>
    template<typename MEMBER_FUNCTION_PTR>
    void CompressedColumnJacobianTransposeBuilder<I>::setupThreadedJob(MEMBER_FUNCTION_PTR ptr, size_t nThreads, bool useMEstimator)
    {
      if (nThreads <= 1) {
        (this->*ptr)(0, 0, _jacobianPointers.size(), useMEstimator);
      } else {
        nThreads = std::min(nThreads, _jacobianPointers.size());
        // Give some error terms to each thread.
        std::vector<int> indices(nThreads + 1, 0);
        int nJPerThread = std::max(1, (int)(_jacobianPointers.size() / nThreads));
        for (unsigned i = 0; i < nThreads; ++i)
          indices[i + 1] = indices[i] + nJPerThread;
        // deal with the remainder.
        indices.back() = _jacobianPointers.size();
        // Build a thread pool and evaluate the jacobians.
        boost::thread_group threads;
        for (unsigned i = 0; i < nThreads; ++i) {
          threads.create_thread(boost::bind(ptr, this, i, indices[i], indices[i + 1], useMEstimator));
        }
        threads.join_all();
      }
    }


    /// \brief build the large, sparse internal Jacobian matrix from the error terms.
    template<typename I>
    void CompressedColumnJacobianTransposeBuilder<I>::buildSystem(size_t nThreads, bool useMEstimator)
    {
      _isJacobianBuiltFromJacobianTranspose = false;
      setupThreadedJob(&CompressedColumnJacobianTransposeBuilder::evaluateJacobians, nThreads, useMEstimator);
    }


    /// \brief a function to be run by a single thread.
    template<typename I>
    void CompressedColumnJacobianTransposeBuilder<I>::evaluateJacobians(int /* threadId */, int startIdx, int endIdx, bool useMEstimator)
    {
      Eigen::VectorXd ee;
      for (int i = startIdx; i < endIdx; ++i) {
        JacobianContainer jc(_jacobianPointers[i].errorTerm->dimension());
        _jacobianPointers[i].errorTerm->getWeightedJacobians(jc, useMEstimator);
        _J_transpose.writeJacobians(jc, _jacobianPointers[i].jcp);
      }
    }


    // /// \brief Get a view of the Jacobian as a cholmod sparse matrix.
    // cholmod_sparse CompressedColumnJacobianTransposeBuilder::getJacobianView()
    // {
    //     // \todo
    //     if(_isJacobianBuiltFromJacobianTranspose)
    //     {
    //         // Return the view.

    //     }
    //     else
    //     {
    //         if(_J.get() != NULL)
    //         {
    //             // We already have the right structure...copy over the transpose.
    //         }
    //         else
    //         {
    //             // We have to do the transpose for the first time.
    //         }
    //     }

    //     return cholmod_sparse();
    // }


    /// \brief Get a view of the transpose of the Jacobian as a cholmod sparse matrix.
    template<typename I>
    cholmod_sparse CompressedColumnJacobianTransposeBuilder<I>::getJacobianTransposeView()
    {
      return cholmod_sparse();
    }

    /// \brief Get a const version of the compressed column matrix.
    template<typename I>
    CompressedColumnMatrix<I> & CompressedColumnJacobianTransposeBuilder<I>::J_transpose()
    {
      return _J_transpose;
    }

    /// \brief Get a const version of the compressed column matrix.
    template<typename I>
    const CompressedColumnMatrix<I> & CompressedColumnJacobianTransposeBuilder<I>::J_transpose() const
    {
      return _J_transpose;
    }


    // const Eigen::VectorXd & CompressedColumnJacobianTransposeBuilder::e() const
    // {
    //     return _e;
    // }

  } // namespace backend
} // namespace aslam
