
namespace sm {
  namespace database {
    
    template<typename T, typename A>
    SerializedMap<T,A>::SerializedMap(const ::boost::filesystem::path & dbFileName, const std::string & tableName) : _tableName(tableName), _iStmt(NULL), _sStmt(NULL)
    {
        _db.reset(new Database(dbFileName));

        setUpTable();


    }

    template<typename T, typename A>
    SerializedMap<T,A>::SerializedMap(boost::shared_ptr<Database> db, const std::string & tableName) : _tableName(tableName), _db(db), _iStmt(NULL), _sStmt(NULL)
    {
        SM_ASSERT_TRUE(UnableToOpenDatabaseException, _db.get() != NULL, "The database is null");

        setUpTable();


    }

      
      template<typename T, typename A>
      void SerializedMap<T,A>::setUpTable()
      {
          SM_ASSERT_TRUE(UnableToOpenDatabaseException, _db.get() != NULL, "The database is null");

          validateTableName();
          int result;

      // Make sure the table exists.
      // This table will have a 64 bit integer key and a binary blob of data.
      std::string sql = "create table if not exists " + _tableName + "(id INTEGER primary_key unique, data BLOB);";
      sqlite3_stmt * stmt;
      // http://www.sqlite.org/c3ref/prepare.html
      result = sqlite3_prepare_v2(_db->db(), sql.c_str(), -1, &stmt, NULL);
      SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to prepare statement: " << sqlite3_errmsg(_db->db()));
      result = sqlite3_step(stmt);
      SM_ASSERT_EQ(SqlException, result, SQLITE_DONE, "Unable to execute statement: " << sqlite3_errmsg(_db->db()));
      // Finalize is like a delete statement. Every prepared statement must be finalized.
      sqlite3_finalize(stmt);

      // Pre-prepare the insert statement.
      std::string insert = "INSERT or REPLACE into " + _tableName + " VALUES(?,?);";
      result = sqlite3_prepare(_db->db(), insert.c_str(), -1, &_iStmt, 0);
      SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to prepare statement: " << sqlite3_errmsg(_db->db()));
    
      // Pre-prepare the select statement.
      std::string select = "SELECT data FROM " + _tableName + " WHERE id = ?";
      result = sqlite3_prepare(_db->db(), select.c_str(), -1, &_sStmt, 0);
      SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to prepare statement: " << sqlite3_errmsg(_db->db()));

      // \todo: http://web.utk.edu/~jplyon/sqlite/SQLite_optimization_FAQ.html
      // \todo: Set up an LRU cache.

    }


    template<typename T, typename A>
    SerializedMap<T,A>::~SerializedMap()
    {
      // Finalize and close
      sqlite3_finalize(_iStmt);
      sqlite3_finalize(_sStmt);
    }

    
    template<typename T, typename A>
    ::boost::shared_ptr<T> SerializedMap<T,A>::get( ::boost::uint64_t id)
    { 
      ::boost::shared_ptr<T> value(new T);
      
      get(id, *value);
      return value;
    }

    template<typename T, typename A>
    void SerializedMap<T,A>::get( ::boost::uint64_t id, T & outValue)
    {
      int result;
      try {
	
	// Bind the id to the select statement.
	result = sqlite3_bind_int64(_sStmt, 1, id);
	SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to bind id " << id << " to SELECT statement: " << sqlite3_errmsg(_db->db()));
	// Execute the bound select statement to retrieve the row.
	result = sqlite3_step(_sStmt);
	// If the select was successful, this will return SQLITE_ROW. Otherwise, the frame is not in the database.
	SM_ASSERT_EQ(SqlException, result, SQLITE_ROW, "Unable to execute SELECT statement: " << sqlite3_errmsg(_db->db()));
	
	SM_ASSERT_GT(SqlException, sqlite3_column_bytes(_sStmt, 0), 0, "The SELECT statement returned zero bytes.");
	//std::cout << "Loading " << sqlite3_column_bytes(_sStmt, 0) << " bytes\n";
	// Deserialize the blob
	// This allows us to create a stream from the blob without incurring a copy
	typedef ::boost::iostreams::basic_array_source<char> Device;
	::boost::iostreams::stream<Device> buffer(reinterpret_cast<const char *>(sqlite3_column_blob(_sStmt, 0)),sqlite3_column_bytes(_sStmt, 0));
	
	iarchive_t ia(buffer);
	
	ia >> outValue;
      
      }
      catch(const SqlException & e)
	{
	  sqlite3_reset(_sStmt);
	  throw;
	}
    
    // After executing, we have to reset the statement
    // http://www.sqlite.org/c3ref/step.html
    result = sqlite3_reset(_sStmt);
    SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to reset the SELECT  statement: " << sqlite3_errmsg(_db->db()));
    
  }


  template<typename T, typename A>
  void SerializedMap<T,A>::set(::boost::uint64_t id, const ::boost::shared_ptr<T> & value)
  {
    SM_ASSERT_TRUE(NullValueException, value, "It is illegal to call this function with a null value");
    set(id,*value);
  }

  template<typename T, typename A>
  void SerializedMap<T,A>::set(::boost::uint64_t id, const T & value)
  {  
    
    int result;
    try {
      // Step 1 is to serialize the object. There is no way to know how big the object will be
      // so unfortunately, this will incur an extra copy.
      
      // The binary flag is important here.
      std::ostringstream oss(std::ios_base::binary);
      oarchive_t oa(oss);
      oa << value;
	
      //std::cout << "Saving " << oss.str().size() << " bytes\n";
      // Step 2 is to bind the frameId and data to the insert statement.
      // http://sqlite.org/c3ref/bind_blob.html
      result = sqlite3_bind_int64(_iStmt, 1, id);
      SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to bind id " << id << " to INSERT statement: " << sqlite3_errmsg(_db->db()));
      result = static_cast<int>(sqlite3_bind_blob(_iStmt, 2, oss.str().c_str(), oss.str().size(), SQLITE_TRANSIENT));
      SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to bind blob of size " << oss.str().size() << " to INSERT statement: " << sqlite3_errmsg(_db->db()));
      // Finally, we execute the bound insert statement.
      result = sqlite3_step(_iStmt);
      SM_ASSERT_EQ(SqlException, result, SQLITE_DONE, "Unable to execute INSERT statement for id " << id << " and blob of size " << oss.str().size() << ": " << sqlite3_errmsg(_db->db()));
	
    }
    catch(const SqlException & e)
      {
	sqlite3_reset(_iStmt);
	throw;
      }

    // After executing, we have to reset the statement
    // http://www.sqlite.org/c3ref/step.html
    result = sqlite3_reset(_iStmt);
    SM_ASSERT_EQ(SqlException, result, SQLITE_OK, "Unable to reset the INSERT  statement: " << sqlite3_errmsg(_db->db()));
  }

  template<typename T, typename A>
  void SerializedMap<T,A>::validateTableName()
  {
    SM_ASSERT_GE(InvalidTableNameException,_tableName.size(),1,"Table name \"" << _tableName << "\" is invalid.");
    SM_ASSERT_TRUE(InvalidTableNameException,isalpha(_tableName[0]), "Table name \"" << _tableName << "\" is invalid.");
    for(size_t i = 1; i < _tableName.size(); i++)
      {
	SM_ASSERT_TRUE(InvalidTableNameException,isalnum(_tableName[i]), "Table name \"" << _tableName << "\" is invalid. Character " << i << " is not alphanumeric.");
      }
  }

} // namespace database
} // namespace sm
