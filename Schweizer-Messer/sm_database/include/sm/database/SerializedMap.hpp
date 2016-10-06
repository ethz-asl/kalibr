#ifndef SM_SERIALIZED_MAP_HPP
#define SM_SERIALIZED_MAP_HPP

// Because this class is templated, there
// is no shielding anyone from the full set 
// of header files.
#include <boost/filesystem.hpp>
#include <sqlite3.h>
#include <sm/boost/serialization.hpp>
#include <sm/assert_macros.hpp>
#include <sstream>
#include <boost/iostreams/device/array.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <boost/portable_binary_oarchive.hpp>
#include <boost/portable_binary_iarchive.hpp>
#include <sm/database/Database.hpp>

namespace sm {
    namespace database {


        struct PortableBinaryArchive
        {
            typedef ::boost::archive::portable_binary_oarchive oarchive_t;
            typedef ::boost::archive::portable_binary_iarchive iarchive_t;
        };


        struct BinaryArchive
        {
            typedef ::boost::archive::binary_oarchive oarchive_t;
            typedef ::boost::archive::binary_iarchive iarchive_t;
        };

        struct TextArchive
        {
            typedef ::boost::archive::text_oarchive oarchive_t;
            typedef ::boost::archive::text_iarchive iarchive_t;
        };

    
        template<typename T, typename ARCHIVE_T>
        class SerializedMap 
        {
        public:
            typedef T value_t;
            typedef ARCHIVE_T archive_t;
            typedef typename archive_t::iarchive_t iarchive_t;
            typedef typename archive_t::oarchive_t oarchive_t;
      
            SerializedMap(const ::boost::filesystem::path & dbFileName, 
                          const std::string & tableName);

            SerializedMap(boost::shared_ptr<Database> db,
                          const std::string & tableName);

            virtual ~SerializedMap();
      
            ::boost::shared_ptr<T> get(::boost::uint64_t id);      

            void get(::boost::uint64_t id, value_t & outValue);
            void set(::boost::uint64_t id, const ::boost::shared_ptr<T> & value);
            void set(::boost::uint64_t id, const T & value);
      
            const std::string & tableName(){ return _tableName; }
        private:
            void setUpTable();
            void validateTableName();

            ::boost::filesystem::path _dbName;
            std::string _tableName;
            boost::shared_ptr<Database> _db;
            sqlite3_stmt * _iStmt, * _sStmt;
      
        };


    } // namespace database
  
} // namespace sm

#include "implementation/SerializedMap.hpp"

#endif /* SM_SERIALIZED_MAP_HPP */
