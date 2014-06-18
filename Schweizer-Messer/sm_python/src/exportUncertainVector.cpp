#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/kinematics/UncertainVector.hpp>







template<int D>
void exportUV()
{
    using namespace boost::python;
    using namespace sm::kinematics;
    std::stringstream str;
    typedef typename UncertainVector<D>::value_t value_t;
    typedef typename UncertainVector<D>::covariance_t covariance_t;
    UncertainVector<1> (UncertainVector<D>::*dot1)(const UncertainVector<D> & ) const = &UncertainVector<D>::dot;
    UncertainVector<1> (UncertainVector<D>::*dot2)(const value_t & ) const = &UncertainVector<D>::dot;

    str << "UncertainVector" << D;
    class_< UncertainVector<D>, boost::shared_ptr<UncertainVector<D> > >(str.str().c_str(), init<>())
        //         UncertainVector(const value_t & v);
        .def(init<value_t>())
        //         UncertainVector(const value_t & v, const covariance_t & E);
        .def(init<value_t,covariance_t>())
        //         /// \brief get the vector
        //         const value_t & v() const;
        .def("v",&UncertainVector<D>::v, return_value_policy<copy_const_reference>())
        //         /// \brief get the value
        //         const value_t & mean() const;
        .def("mean",&UncertainVector<D>::mean, return_value_policy<copy_const_reference>())
        //         /// \brief set the vector
        //         void setMean(const value_t & v);
        .def("setMean",&UncertainVector<D>::setMean)
        //         /// \brief get the covariance
        //         const covariance_t & E() const;
        .def("E",&UncertainVector<D>::E, return_value_policy<copy_const_reference>())
        //         /// \brief get the covariance
        //         const covariance_t & covariance() const;
        .def("covariance",&UncertainVector<D>::covariance, return_value_policy<copy_const_reference>())
        //         /// \brief set the covariance
        //         void setCovariance(const covariance_t & E) const;
        .def("setCovariance",&UncertainVector<D>::setCovariance)
        //         UncertainVector<D> operator+(const UncertainVector<D> & rhs);
        .def(self + UncertainVector<D>())
        .def(self - UncertainVector<D>())
        .def(self + value_t())
        .def(self - value_t())

        //         void normalize();
        .def("normalize",&UncertainVector<D>::normalize)

        //UncertainVector<D> operator*(const UncertainVector<1> & rhs) const;
        .def(self * UncertainVector<1>())
        // UncertainVector<D> operator*(double s) const;
        .def(self * double(1.0))
    
        // UncertainVector<D> normalized();
        .def("normalized",&UncertainVector<D>::normalized)
                
        .def("dot",dot1)
        .def("dotV",dot2)
        ;
}


void exportUS()
{
    using namespace boost::python;
    using namespace sm::kinematics;
    std::stringstream str;
    typedef UncertainVector<1>::value_t value_t;
    typedef UncertainVector<1>::covariance_t covariance_t;
    UncertainVector<1> (UncertainVector<1>::*dot1)(const UncertainVector<1> & ) const = &UncertainVector<1>::dot;
    UncertainVector<1> (UncertainVector<1>::*dot2)(const value_t & ) const = &UncertainVector<1>::dot;

    class_< UncertainVector<1>, boost::shared_ptr<UncertainVector<1> > >("UncertainScalar", init<>())
        //         UncertainVector(const value_t & v);
        .def(init<value_t>())
        //         UncertainVector(const value_t & v, const covariance_t & E);
        .def(init<value_t,covariance_t>())
        //         /// \brief get the vector
        //         const value_t & v() const;
        .def("v",&UncertainVector<1>::v, return_value_policy<copy_const_reference>())
        //         /// \brief get the value
        //         const value_t & mean() const;
        .def("mean",&UncertainVector<1>::mean, return_value_policy<copy_const_reference>())
        //         /// \brief set the vector
        //         void setMean(const value_t & v);
        .def("setMean",&UncertainVector<1>::setMean)
        //         /// \brief get the covariance
        //         const covariance_t & E() const;
        .def("E",&UncertainVector<1>::E, return_value_policy<copy_const_reference>())
        //         /// \brief get the covariance
        //         const covariance_t & covariance() const;
        .def("covariance",&UncertainVector<1>::covariance, return_value_policy<copy_const_reference>())
        //         /// \brief set the covariance
        //         void setCovariance(const covariance_t & E) const;
        .def("setCovariance",&UncertainVector<1>::setCovariance)
        //         UncertainVector<1> operator+(const UncertainVector<1> & rhs);
        .def(self + UncertainVector<1>())
        .def(self - UncertainVector<1>())
        .def(self * UncertainVector<1>())
        .def(self / UncertainVector<1>())

        .def(self + value_t())
        .def(self - value_t())

        .def(self * value_t())
        .def(self / value_t())
                
        .def("dot",dot1)
        .def("dotV",dot2)

        ;
}


void exportUncertainVector()
{
    using namespace boost::python;
    using namespace sm::kinematics;

    exportUS();
    exportUV<2>();
    exportUV<3>();
    exportUV<4>();
    exportUV<5>();
    exportUV<6>();
}
