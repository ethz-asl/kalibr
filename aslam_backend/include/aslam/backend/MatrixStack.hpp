/*
 * MatrixStack.hpp
 *
 *  Created on: 21.03.2016
 *      Author: Ulrich Schwesinger
 */

#ifndef INCLUDE_ASLAM_BACKEND_MATRIXSTACK_HPP_
#define INCLUDE_ASLAM_BACKEND_MATRIXSTACK_HPP_

// standard includes
#include <cstdint>
#include <vector>
#include <utility> // std::move

// Eigen includes
#include <Eigen/Dense>

// Schweizer-Messer
#include <sm/logging.hpp>

// self includes
#include <aslam/Exceptions.hpp>
#include <aslam/backend/util/CommonDefinitions.hpp>

namespace aslam {
namespace backend {

  /**
   * \class MatrixStack
   * \brief Stores a chain rule of matrices in a stack-like data structure
   */
  class MatrixStack
  {
   public:
    typedef double Scalar;
    template <int Rows, int Cols>
    using Map = Eigen::Map<Eigen::Matrix<Scalar, Rows, Cols>, Eigen::Aligned>;
    template <int Rows, int Cols>
    using ConstMap = Eigen::Map<const Eigen::Matrix<Scalar, Rows, Cols>, Eigen::Aligned>;

    /**
     * \class Header
     * \brief Metadata for stack elements
     */
    struct Header
    {
      Header(const int cols_, const std::size_t dataIndex_)
          : cols(cols_), dataIndex(dataIndex_)
      {

      }
      uint16_t cols; /// \brief Number of columns of the data
      uint32_t dataIndex;  /// \brief Index to the beginning of the data of this element
    };

    /**
     * \class PopGuard
     * \brief Ensures that \p pop() is called on the stack when the guard goes
     *        out of scope
     */
    struct PopGuard
    {
      PopGuard(MatrixStack& stack)
          : _stack(&stack)
      {

      }

      PopGuard(const PopGuard&) = delete;
      void operator=(const PopGuard&) = delete;

      PopGuard(PopGuard&& pg)
      {
        *this = std::move(pg);
      }
      void operator=(PopGuard&& pg)
      {
        this->_stack = pg._stack;
        pg._stack = nullptr;
      }

      void pop()
      {
        if (_stack != nullptr) {
          _stack->pop();
          _stack = nullptr;
        }
      }
      ~PopGuard() { this->pop(); }

      MatrixStack& stack()
      {
        SM_ASSERT_NOTNULL_DBG(Exception, _stack, "");
        return *_stack;
      }

     private:
      MatrixStack* _stack;
    };

   public:

    /// \brief Constructs a stack
    MatrixStack(const uint16_t numRows, const std::size_t maxNumMatrices, const std::size_t estimatedNumElementsPerMatrix)
        : _data(maxNumMatrices*estimatedNumElementsPerMatrix), _numRows(numRows)
    {
      _headers.reserve(maxNumMatrices);
    }

    /// \brief Is the stack empty?
    bool empty() const { return _headers.empty(); }

    /// \brief Number of matrices stored
    std::size_t numMatrices() const { return _headers.size(); }

    /// \brief Number of matrix elements stored
    std::size_t numElements() const { return _dataSize; }

    /// \brief Push a matrix \p mat to the top of the stack
    template <typename DERIVED>
    EIGEN_ALWAYS_INLINE void push(const Eigen::MatrixBase<DERIVED>& mat)
    {
      if (LIKELY(!this->empty()))
      {
        SM_ASSERT_EQ_DBG(Exception, this->numTopCols(), mat.rows(), "Incompatible matrix sizes");
        this->allocate(mat.cols()); // We allocate space for 1 more matrix. Stack wasn't empty before, so now we have at least 2.
        this->top().noalias() = this->matrix(this->numMatrices()-2)*mat;
      }
      else
      {
        SM_ASSERT_EQ_DBG(Exception, this->numRows(), mat.rows(), "Incompatible matrix sizes");
        this->allocate(mat.cols());
        this->top() = mat;
      }
    }

    /// \brief Push a scalar factor \p scalar to the top of the stack
    EIGEN_ALWAYS_INLINE void push(const double scalar)
    {
      if (LIKELY(!this->empty()))
      {
        this->allocate(this->numTopCols()); // We allocate space for 1 more matrix. Stack wasn't empty before, so now we have at least 2.
        this->top() = this->matrix(this->numMatrices()-2)*scalar;
      }
      else
      {
        this->allocate(this->numRows());
        this->top() = Eigen::MatrixXd::Identity(this->numRows(), this->numRows())*scalar;
      }
    }

    /// \brief Push a matrix \p mat to the top of the stack and return a pop guard to ensure the matrix is popped again
    template <typename DERIVED>
    EIGEN_ALWAYS_INLINE PopGuard pushWithGuard(const Eigen::MatrixBase<DERIVED>& mat)
    {
      this->push(mat);
      return PopGuard(*this);
    }

    /// \brief Push a scalar \p scalar to the top of the stack and return a pop guard to ensure the matrix is popped again
    EIGEN_ALWAYS_INLINE PopGuard pushWithGuard(const double scalar)
    {
      this->push(scalar);
      return PopGuard(*this);
    }

    /// \brief Pop the top element from the stack
    void pop()
    {
      SM_ASSERT_FALSE(Exception, this->empty(), "pop() on an empty stack is forbidden!");
      _headers.pop_back();
      _dataSize = this->numMatrices() == 0 ? 0 : _headers.back().dataIndex + this->numRows()*_headers.back().cols; // Be careful with manual memory alignment in allocate()
    }

    /// \brief Const getter for the top matrix in the stack
    template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
    EIGEN_ALWAYS_INLINE ConstMap<Rows, Cols> top() const
    {
      SM_ASSERT_FALSE(Exception, this->empty(), "");
      return ConstMap<Rows, Cols>( &(_data[_headers.back().dataIndex]), this->numRows(), _headers.back().cols );
    }

    /// \brief Mutable getter for the top matrix in the stack
    template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
    EIGEN_ALWAYS_INLINE Map<Rows, Cols> top()
    {
      SM_ASSERT_FALSE(Exception, this->empty(), "");
      return Map<Rows, Cols>( &(_data[_headers.back().dataIndex]), this->numRows(), _headers.back().cols );
    }

   protected:

    /// \brief Allocates memory and metadata for an element of size \p _numRows x \p cols.
    void allocate(const int cols)
    {
      static constexpr int align = 16/sizeof(Scalar);
      std::size_t start = this->numElements();
      if (start % align != 0) // manual memory alignment
        start += align - start % align;

      const std::size_t newSize = start + this->numRows()*cols;
      if (UNLIKELY(newSize > _data.size())) {
        SM_WARN_STREAM_NAMED("optimization", "Matrix stack has to reallocate memory (" <<
            newSize << " > " <<  _data.size() << "). Consider constructing the matrix stack "
            "with more memory reserves for faster execution.");
        _data.resize(std::max(2*_data.size(), newSize));
      }

      _dataSize = newSize;
      _headers.emplace_back(cols, start);

      SM_ASSERT_TRUE_DBG(Exception, (uintptr_t)(&(_data[_headers.back().dataIndex])) % 16 == 0, "Memory is not properly aligned");
    }

    /// \brief Const getter for the \p i-th matrix in the stack
    template<int Rows = Eigen::Dynamic, int Cols = Eigen::Dynamic>
    EIGEN_ALWAYS_INLINE ConstMap<Rows, Cols> matrix(const std::size_t i) const
    {
      SM_ASSERT_LT(IndexOutOfBoundsException, i, this->numMatrices(), "");
      return ConstMap<Rows, Cols>( &(_data[_headers[i].dataIndex]), this->numRows(), _headers[i].cols );
    }

   protected:

    /// \brief Number of columns of the matrix on top of the stack
    int numTopCols() const
    {
      return _headers.back().cols;
    }

   private:
    /// \brief Number of rows determined by the first matrix that was pushed
    int numRows() const
    {
      return _numRows;
    }

   private:
    typedef Eigen::aligned_allocator< std::vector<Scalar> > AlignedAllocator;
    std::vector<Scalar, AlignedAllocator> _data; /// \brief The data of the matrices
    std::vector<Header> _headers; /// \brief Metadata for the matrix entries

    uint16_t _numRows; /// \brief Number of rows
    std::size_t _dataSize = 0; /// \brief Number of elements in \p _data. Note
  };

} /* namespace aslam */
} /* namespace backend */

#endif /* INCLUDE_ASLAM_BACKEND_MATRIXSTACK_HPP_ */
