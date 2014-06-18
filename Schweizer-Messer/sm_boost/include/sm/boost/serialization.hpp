#ifndef BOOST_SERIALIZATION_HPP
#define BOOST_SERIALIZATION_HPP

#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <sm/assert_macros.hpp>
// BOOST_SERIALIZATION_NVP
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/serialization/version.hpp>

// Save and load using boost serailization
namespace sm { namespace boost_serialization {

        template<typename T>
        void save(T & object, ::boost::filesystem::path const & filename)
        {
            std::ofstream ofs(filename.string().c_str(), std::ios::binary);
            SM_ASSERT_TRUE(std::runtime_error,ofs.good(), "Unable to open file " << filename << " for writing");
            ::boost::archive::binary_oarchive oa(ofs);
    
            oa << object;
        }

        template<typename T>
        void load(T & object, ::boost::filesystem::path const & filename)
        {
            std::ifstream ifs(filename.string().c_str(), std::ios::binary);
            SM_ASSERT_TRUE(std::runtime_error,ifs.good(), "Unable to open file " << filename << " for reading");
            ::boost::archive::binary_iarchive ia(ifs);
    
            ia >> object;

        }



        template<typename T>
        void save_txt(T & object, ::boost::filesystem::path const & filename)
        {
            std::ofstream ofs(filename.string().c_str(), std::ios::binary);
            SM_ASSERT_TRUE(std::runtime_error,ofs.good(), "Unable to open file " << filename << " for writing");
            ::boost::archive::text_oarchive oa(ofs);
    
            oa << object;
        }

        template<typename T>
        void load_txt(T & object, ::boost::filesystem::path const & filename)
        {
            std::ifstream ifs(filename.string().c_str(), std::ios::binary);
            SM_ASSERT_TRUE(std::runtime_error,ifs.good(), "Unable to open file " << filename << " for reading");
            ::boost::archive::text_iarchive ia(ifs);
    
            ia >> object;

        }



        template<typename T>
        void save_xml(T & object, std::string topLevelXmlTag, ::boost::filesystem::path const & filename)
        {
            std::ofstream ofs(filename.string().c_str());
            SM_ASSERT_TRUE(std::runtime_error,ofs.good(), "Unable to open file " << filename << " for writing");
            ::boost::archive::xml_oarchive oa(ofs);
            if(topLevelXmlTag.size() == 0)
            {
                topLevelXmlTag = "object";
            }

            oa << ::boost::serialization::make_nvp(topLevelXmlTag.c_str(), object);

        }

        template<typename T>
        void load_xml(T & object, std::string topLevelXmlTag, ::boost::filesystem::path const & filename)
        {
            std::ifstream ifs(filename.string().c_str());
            SM_ASSERT_TRUE(std::runtime_error,ifs.good(), "Unable to open file " << filename << " for reading");

            ::boost::archive::xml_iarchive ia(ifs);

            if(topLevelXmlTag.size() == 0)
            {
                topLevelXmlTag = "object";
            }
   
            ia >> ::boost::serialization::make_nvp(topLevelXmlTag.c_str(), object);

        }

        template<typename T>
        void save_xml(T & object, ::boost::filesystem::path const & filename)
        {
            save_xml(object, "object", filename);
        }

        template<typename T>
        void load_xml(T & object, ::boost::filesystem::path const & filename)
        {
            load_xml(object, "object", filename);
        }

    
    }} // namespace sm::boost_serialization


#define SM_BOOST_CLASS_VERSION(CLASS_T)                                 \
    BOOST_CLASS_VERSION(CLASS_T,CLASS_T::CLASS_SERIALIZATION_VERSION)

#define SM_BOOST_CLASS_VERSION_T1(CLASS_T)                              \
    namespace boost {                                                   \
        namespace serialization {                                       \
                                                                        \
            template<typename T1>                                       \
            struct version< CLASS_T<T1> >                               \
            {                                                           \
                enum { NNN = CLASS_T<T1>::CLASS_SERIALIZATION_VERSION }; \
                typedef mpl::int_<  NNN  > type;                        \
                typedef mpl::integral_c_tag tag;                        \
                BOOST_STATIC_CONSTANT(int, value = version::type::value); \
                BOOST_MPL_ASSERT((                                      \
                                     boost::mpl::less<                  \
                                                                        boost::mpl::int_<NNN>, \
                                                                        boost::mpl::int_<256> \
                                                                        > \
                    ));                                                 \
            };                                                          \
        }                                                               \
    }


#define SM_BOOST_CLASS_VERSION_T1T2(CLASS_T)                            \
    namespace boost {                                                   \
        namespace serialization {                                       \
                                                                        \
            template<typename T1, typename T2>                          \
            struct version< CLASS_T<T1,T2> >                            \
            {                                                           \
                enum { NNN = CLASS_T<T1,T2>::CLASS_SERIALIZATION_VERSION }; \
                typedef mpl::int_<  NNN  > type;                        \
                typedef mpl::integral_c_tag tag;                        \
                BOOST_STATIC_CONSTANT(int, value = version::type::value); \
                BOOST_MPL_ASSERT((                                      \
                                     boost::mpl::less<                  \
                                                                        boost::mpl::int_<NNN>, \
                                                                        boost::mpl::int_<256> \
                                                                        > \
                    ));                                                 \
            };                                                          \
        }                                                               \
    }


#define SM_BOOST_CLASS_VERSION_T1T2T3(CLASS_T)                          \
    namespace boost {                                                   \
        namespace serialization {                                       \
                                                                        \
            template<typename T1, typename T2, typename T3>             \
            struct version< CLASS_T<T1,T2,T3> >                         \
            {                                                           \
                enum { NNN = CLASS_T<T1,T2,T3>::CLASS_SERIALIZATION_VERSION }; \
                typedef mpl::int_<  NNN  > type;                        \
                typedef mpl::integral_c_tag tag;                        \
                BOOST_STATIC_CONSTANT(int, value = version::type::value); \
                BOOST_MPL_ASSERT((                                      \
                                     boost::mpl::less<                  \
                                                                        boost::mpl::int_<NNN>, \
                                                                        boost::mpl::int_<256> \
                                                                        > \
                    ));                                                 \
            };                                                          \
        }                                                               \
    }

#define SM_BOOST_CLASS_VERSION_T1T2T3T4(CLASS_T)                        \
    namespace boost {                                                   \
        namespace serialization {                                       \
                                                                        \
            template<typename T1, typename T2, typename T3, typename T4> \
            struct version< CLASS_T<T1,T2,T3,T4> >                      \
            {                                                           \
                enum { NNN = CLASS_T<T1,T2,T3,T4>::CLASS_SERIALIZATION_VERSION }; \
                typedef mpl::int_<  NNN  > type;                        \
                typedef mpl::integral_c_tag tag;                        \
                BOOST_STATIC_CONSTANT(int, value = version::type::value); \
                BOOST_MPL_ASSERT((                                      \
                                     boost::mpl::less<                  \
                                                                        boost::mpl::int_<NNN>, \
                                                                        boost::mpl::int_<256> \
                                                                        > \
                    ));                                                 \
            };                                                          \
        }                                                               \
    }



#define SM_BOOST_CLASS_VERSION_I1(CLASS_T)                              \
    namespace boost {                                                   \
        namespace serialization {                                       \
                                                                        \
            template<int I1>                                            \
            struct version< CLASS_T<I1> >                               \
            {                                                           \
                enum { NNN = CLASS_T<I1>::CLASS_SERIALIZATION_VERSION }; \
                typedef mpl::int_<  NNN  > type;                        \
                typedef mpl::integral_c_tag tag;                        \
                BOOST_STATIC_CONSTANT(int, value = version::type::value); \
                BOOST_MPL_ASSERT((                                      \
                                     boost::mpl::less<                  \
                                                                        boost::mpl::int_<NNN>, \
                                                                        boost::mpl::int_<256> \
                                                                        > \
                    ));                                                 \
            };                                                          \
        }                                                               \
    }


#define SM_BOOST_CLASS_VERSION_I1T2(CLASS_T)                            \
    namespace boost {                                                   \
        namespace serialization {                                       \
                                                                        \
            template<int I1, typename T2>                               \
            struct version< CLASS_T<I1,T2> >                            \
            {                                                           \
                enum { NNN = CLASS_T<I1,T2>::CLASS_SERIALIZATION_VERSION }; \
                typedef mpl::int_<  NNN  > type;                        \
                typedef mpl::integral_c_tag tag;                        \
                BOOST_STATIC_CONSTANT(int, value = version::type::value); \
                BOOST_MPL_ASSERT((                                      \
                                     boost::mpl::less<                  \
                                                                        boost::mpl::int_<NNN>, \
                                                                        boost::mpl::int_<256> \
                                                                        > \
                    ));                                                 \
            };                                                          \
        }                                                               \
    }






#endif /* ASRL_SERIALIZATION_HPP */
