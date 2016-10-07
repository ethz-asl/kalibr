// It is extremely important to use this header
// if you are using the numpy_eigen interface
#include <numpy_eigen/boost_python_headers.hpp>
#include <sparse_block_matrix/sparse_block_matrix.h>
#include <Eigen/Dense>

using namespace boost::python;
using namespace sparse_block_matrix;

template<typename SBM>
void clearSBM(SBM * sbm) { sbm->clear(); }

template<typename SBM>
typename SBM::SparseMatrixBlock blockSBM(const SBM * sbm,int r, int c) { 
  if(r < 0 || r >= sbm->bRows())
    throw std::runtime_error("Row index out of bounds");
  if(c < 0 || c >= sbm->bCols())
    throw std::runtime_error("Column index out of bounds");

  const typename SBM::SparseMatrixBlock * mat = sbm->block(r,c); 
  if(mat)
    {
      return *mat;
    }
  else 
    {
      typename SBM::SparseMatrixBlock rval;
      rval.resize(sbm->rowsOfBlock(r), sbm->colsOfBlock(c));
      rval.setZero();
      return rval;
    } 
}


template<typename SBM>
void setBlockSBM(SBM * sbm, int r, int c, const typename SBM::SparseMatrixBlock & mat) { 
  if(r < 0 || r >= sbm->bRows())
    throw std::runtime_error("Row index out of bounds");
  if(c < 0 || c >= sbm->bCols())
    throw std::runtime_error("Column index out of bounds");

  typename SBM::SparseMatrixBlock * blk = sbm->block(r,c,true);
  if(!blk)
    throw std::runtime_error("Unexpected error: block is null");

  *blk = mat; 

}

template<typename SBM>
Eigen::MatrixXd toDenseSymmetric(SBM * sbm)
{
  //return sbm->toDense().selfadjointView<Eigen::Upper>();
  return sbm->toDense(). template selfadjointView<Eigen::Upper>();
}

template<typename SBM>
double at(SBM * sbm, int r, int c)
{
    return (*sbm)(r,c);
}


void exportSBM()
{
  typedef SparseBlockMatrix<Eigen::MatrixXd> mat_t;

  boost::python::class_< mat_t > ("SparseBlockMatrixXd", init<>())
    .def(init<const Eigen::VectorXi &, const Eigen::VectorXi &, bool>( ("SparseBlockMatrixXd(rbi, cbi, hasStorage)\n\nrbi:  a vector of ints containing the row layout of the blocks. The component i of the array should contain the index of the first row of the block i+1\ncbi:  a vector of ints containing the column layout of the blocks. The component i of the array should contain the index of the first column of the block i+1")) )
    .def("rows",&mat_t::rows)
    .def("cols",&mat_t::cols)
    .def("bRows",&mat_t::bRows)
    .def("bCols",&mat_t::bCols)
    .def("clear",&clearSBM<mat_t>)
    .def("clear",&mat_t::clear)
    .def("isBlockSet",&mat_t::isBlockSet)
    .def("getBlock",&blockSBM<mat_t>)
    .def("setBlock",&setBlockSBM<mat_t>)
    .def("rowsOfBlock",&mat_t::rowsOfBlock)
    .def("colsOfBlock",&mat_t::colsOfBlock)
    .def("rowBaseOfBlock",&mat_t::rowBaseOfBlock)
    .def("colBaseOfBlock",&mat_t::colBaseOfBlock)
    .def("nonZeros",&mat_t::nonZeros)
    .def("nonZeroBlocks",&mat_t::nonZeroBlocks)
    .def("clone", &mat_t::clone, return_value_policy<manage_new_object>())
    .def("toDense", &mat_t::toDense)
    .def("toDenseSymmetric", &toDenseSymmetric<mat_t>)
      .def("at", &at<mat_t>)
    // Still more to do here, but this is the easy stuff that will allow us to look
    // at one of these things in Python.
    ;

}

