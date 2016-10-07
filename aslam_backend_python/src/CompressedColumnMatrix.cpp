#include <numpy_eigen/boost_python_headers.hpp>
#include <aslam/backend/CompressedColumnMatrix.hpp>
#include <boost/cstdint.hpp>

template<typename INDEX_T>
Eigen::MatrixXd toDense(aslam::backend::CompressedColumnMatrix<INDEX_T> * mat)
{
    Eigen::MatrixXd out;
    mat->toDenseInto(out);
    return out;
}

template<typename INDEX_T>
void exportCompressedColumnMatrixIt(const std::string & name)
{
    using namespace boost::python;
    using namespace aslam::backend;

    typedef CompressedColumnMatrix<INDEX_T> value_t;

    class_< value_t,
            boost::shared_ptr< value_t > >
        (("CompressedColumnMatrix" + name).c_str(), init<>())
        .def("rows", &value_t::rows)
        .def("cols", &value_t::cols)
        .def("value", &value_t::value)
        .def("nnz", &value_t::nnz)
        .def("toDense", &toDense<INDEX_T>)
        ;

}


void exportCompressedColumnMatrix()
{
    exportCompressedColumnMatrixIt<boost::int32_t>("Int");
    exportCompressedColumnMatrixIt<boost::int64_t>("Long");
}
