#ifndef SM_DATABASE_HPP
#define SM_DATABASE_HPP

#include <boost/filesystem.hpp>
#include <sqlite3.h>
#include <sm/assert_macros.hpp>


namespace sm {
    namespace database {
        SM_DEFINE_EXCEPTION(Exception,std::runtime_error);
        SM_DEFINE_EXCEPTION(SqlException,Exception);
        SM_DEFINE_EXCEPTION(UnknownIdException, Exception);
        SM_DEFINE_EXCEPTION(UnableToOpenDatabaseException, Exception);
        SM_DEFINE_EXCEPTION(InvalidTableNameException, Exception);
        SM_DEFINE_EXCEPTION(InvalidDbNameException, Exception);
        SM_DEFINE_EXCEPTION(NullValueException, Exception);


        class Database
        {
        public:
            Database(const ::boost::filesystem::path & dbFileName);
            virtual ~Database();

            ::boost::filesystem::path dbName() const;
            sqlite3 * db();
            const sqlite3 * db() const;

            ::boost::filesystem::path _dbName;
            sqlite3 * _db;

        };
    } // namespace database
} // namespace sm


#endif /* SM_DATABASE_HPP */
