#include <sm/database/Database.hpp>


namespace sm {
    namespace database {
        Database::Database(const ::boost::filesystem::path & dbFileName)
        {
            SM_ASSERT_GT(InvalidDbNameException, dbFileName.string().size(), 0, "The database filename must be greater than zero");
            // Open a connection to the database.
            // http://www.sqlite.org/c3ref/open.html
            int flags = SQLITE_OPEN_READWRITE | // The connection is read/write
                SQLITE_OPEN_CREATE;               // The database will be created if it doesn't exist. 
            
            int result = sqlite3_open_v2( dbFileName.string().c_str(), &_db, flags, NULL);
            SM_ASSERT_TRUE(UnableToOpenDatabaseException, _db != NULL, "Unable to open sqlite3 database " << _dbName.string() << ", Error: " <<  sqlite3_errmsg(_db)); 
            SM_ASSERT_EQ(UnableToOpenDatabaseException, result, SQLITE_OK, "Unable to open sqlite3 database " << _dbName.string() << ", Error: " <<  sqlite3_errmsg(_db));

        }


        Database::~Database()
        {
            sqlite3_close(_db);
        }

        ::boost::filesystem::path Database::dbName() const { return _dbName; }
        sqlite3 * Database::db() { return _db; }
        const sqlite3 * Database::db() const { return _db; }


    } // namespace database
} // namespace sm
