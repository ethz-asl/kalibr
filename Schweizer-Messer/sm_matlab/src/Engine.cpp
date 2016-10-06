#include <sm/matlab/Engine.hpp>
#include <cctype>
#include <stdio.h>

namespace sm {
  namespace matlab {
    
  
    Engine::Engine()
    {
		_engine = NULL;
        // make sure the output buffer is NULL terminated
        _outputBuffer[OUTPUT_BUFFER_SIZE] = '\0';
    }

	Engine::Engine(bool startMatlabAtInitialization)
    {
		// make sure the output buffer is NULL terminated
        _outputBuffer[OUTPUT_BUFFER_SIZE] = '\0';

		if (startMatlabAtInitialization)
		{
			_engine = engOpen(NULL);

			// tell Matlab where to store the output
			engOutputBuffer(_engine, _outputBuffer, OUTPUT_BUFFER_SIZE);
		}
    }

#ifdef UNIX
    Engine::Engine(std::string hostename)
    {
        // it would be possible to run Matlab on a different Machine under Linux
        // due to time constraints and lack of need, this is not implemnted yet
        SM_THROW(std::runtime_error, "Not implemented!");
    }
#endif

    Engine::~Engine()
    {
		if (_engine!=NULL)
		{
			int success = engClose(_engine);
			SM_ASSERT_EQ_DBG(EngineCloseException, success, 0, "Closing Matlab was not possible. Maybe already closed.");
		}
    }

	bool Engine::initialize()
	{
		if (isInitialized())
			return true;

		_engine = engOpen(NULL);

		// tell Matlab where to store the output
		engOutputBuffer(_engine, _outputBuffer, OUTPUT_BUFFER_SIZE);

		return (_engine!=NULL);
	}

	bool Engine::stop()
	{
		bool success = true;
		if (_engine != NULL)
		{
			success = (engClose(_engine) == 0);
			_engine = NULL;
		}
		return success;
	}

	bool Engine::isInitialized()
	{
		return (_engine != NULL);
	}

	bool Engine::good()
	{
		if (!isInitialized())
			return false;

		bool success = false;
		try {
			success = executeCommand("disp('sm::Matlab Engine-Test')") == ">> sm::Matlab Engine-Test";
		}
		catch (EngineExecutionException& e)
		{
			return false;
		}
		return success;
	}

    std::string Engine::executeCommand(const std::string& command)
    {
		assertIsInitialized();

        // execute the command
        int success = engEvalString(_engine, command.c_str());
        // check for failures
        SM_ASSERT_EQ(EngineExecutionException, success, 0, "Failed to execute command. Maybe Matlab is already closed. Note: This assert is NOT thrown due to invalid Matlab syntax");
        // return the result
        std::string output(_outputBuffer);
		// remove line break
		if(output.size()>0)
			output.resize(output.size() - 1);
		
		return output;
    }
    
    std::string Engine::changeWorkingDirectory(const std::string& directory)
    {
        return executeCommand("cd ('" + directory + "');");
    };
    
    void Engine::openCommandLine()
    {
        // Display info
        std::cout<<"Enter a MATLAB command to evaluate."<<std::endl;
	    std::cout<<"Type 'quit' or 'exit' to leave command line mode"<<std::endl;

        // allow the user to enter multiple (an infinite number) commands
        while (true)
        {
            // Prepare input line
            std::cout<<">> ";
            
            // Read from console
            if (!fgets(_inputBuffer, INPUT_BUFFER_SIZE, stdin))
                break;
            
            // Convert to string
            std::string command(_inputBuffer);
			// remove line break
			command.resize(command.size() - 1);
			
            // Check if user wants to exit
            // this intentionally blocks the MATLAB commands "exit" and "quit"
            if (command == "quit" || command == "exit")
                return;
                
            // execute and display command
            std::cout<<executeCommand(command);
        }
    }

	// TESTERS
	// *******

	  bool Engine::exists(const std::string& name)
	  {
		assertValidVariableName(name);
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());
		// Check if variable exists
		return (mxArray!=NULL);
	  }

	  bool Engine::isScalar(const std::string& name)
	  {
		assertIsInitialized();
		assertValidVariableName(name);
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());
		// Check if variable exists
		SM_ASSERT_NE(NoSuchVariableException, mxArray, NULL, "Variable does not exist");
		
		return mxGetNumberOfElements(mxArray) == 1;
	  }

	  bool Engine::isEmpty(const std::string& name)
	  {
		assertIsInitialized();
		assertValidVariableName(name);
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());
		// Check if variable exists
		SM_ASSERT_NE(NoSuchVariableException, mxArray, NULL, "Variable does not exist");
		
		return mxIsEmpty(mxArray) == 1;
	  }

	  bool Engine::isCharOrString(const std::string& name)
	  {
		assertIsInitialized();
		assertValidVariableName(name);
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());
		// Check if variable exists
		SM_ASSERT_NE(NoSuchVariableException, mxArray, NULL, "Variable does not exist");
		
		return mxIsChar(mxArray) == 1;
	  }

	  bool Engine::getDimensions(const std::string& name, size_t& rows, size_t& cols)
	  {
		assertIsInitialized();
		assertValidVariableName(name);
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());
		// Check if variable exists
		if(!mxArray)
			return false;

		rows = mxGetM(mxArray);
		cols = mxGetN(mxArray);

		return true;
	  }

	
	// SETTERS
	// *******
	
    void Engine::put(const std::string& name, const std::string& value)
    {
		assertIsInitialized();
        assertValidVariableName(name);
    
        // Matlab stores all variables as doubles so we can just type cast here
        mxArray* mxArray = mxCreateString(value.c_str());
              
        // send data and verify
        int success = engPutVariable(_engine, name.c_str(), mxArray);
        SM_ASSERT_EQ(EnginePutException, success, 0, "Failed to send matrix to Matlab");
        
        mxDestroyArray(mxArray);
    }
    
	
	void Engine::put(const std::string& name, bool value)
    {
		assertIsInitialized();
        assertValidVariableName(name);
    
        // Matlab stores all variables as doubles so we can just type cast here
        mxArray* mxArray = mxCreateLogicalScalar(value);
              
        // send data and verify
        int success = engPutVariable(_engine, name.c_str(), mxArray);
        SM_ASSERT_EQ(EnginePutException, success, 0, "Failed to send matrix to Matlab");
        
        mxDestroyArray(mxArray);
    }
	

	// GETTERS
	// *******
	
    bool Engine::get(const std::string& name, double& rValue)
    {
		assertIsInitialized();
        assertValidVariableName(name);
    
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());

		// Check if variable exists
		if (!mxArray)
			return false;
		
		// Check type
		SM_ASSERT_EQ(InvalidTypeException, mxIsNumeric(mxArray), 1, "Variable is not numeric (normally scalars are stored as doubles in Matlab)");
		SM_ASSERT_EQ(EmptyVariableException, mxIsEmpty(mxArray), 0, "Variable is empty!");
		SM_ASSERT_EQ(InvalidTypeException, mxGetNumberOfElements(mxArray), 1, "Variable is not a scalar (has more than 1 element)");
		
		// read data
		rValue = mxGetScalar(mxArray);
          
		// delete array
        mxDestroyArray(mxArray);

		return true;
    }
	
	bool Engine::get(const std::string& name, std::string& rValue)
    {
		assertIsInitialized();
        assertValidVariableName(name);
    
        // Get variable from matlab
		mxArray* mxArray = engGetVariable(_engine, name.c_str());

		// Check if variable exists
		if (!mxArray)
			return false;
		
		// Check type
		SM_ASSERT_EQ(EmptyVariableException, mxIsEmpty(mxArray), 0, "Variable is empty!");
		SM_ASSERT_EQ(InvalidTypeException, mxIsChar(mxArray), 1, "Variable is not a character/string");
		
		// read data
		// lenght + 1 because of 0 terminated string
		size_t lenght = mxGetNumberOfElements(mxArray)+1;
		char* buffer = new char[lenght];
		mxGetString(mxArray, buffer, lenght);
		rValue = buffer;
          
		// delete array
        mxDestroyArray(mxArray);

		return true;
    }
	
	bool Engine::get(const std::string& name, bool& rValue)
    {
		assertIsInitialized();
        assertValidVariableName(name);
    
        // Get variable from matlab
        mxArray* mxArray = engGetVariable(_engine, name.c_str());

		// Check if variable exists
		if (!mxArray)
			return false;
		
		// Check type
		SM_ASSERT_EQ(InvalidTypeException, mxIsLogical(mxArray), 1, "Variable is not of boolean type");
		SM_ASSERT_EQ(EmptyVariableException, mxIsEmpty(mxArray), 0, "Variable is empty!");
		SM_ASSERT_EQ(InvalidTypeException, mxGetNumberOfElements(mxArray), 1, "Variable is not a scalar (has more than 1 element)");
		
		// read data
		rValue = mxIsLogicalScalarTrue(mxArray);
          
		// delete array
        mxDestroyArray(mxArray);

		return true;
    }

	bool Engine::getEigen(const std::string& name, Eigen::MatrixXd& rValue)
    {
		assertIsInitialized();
		assertValidVariableName(name);

		// Get variable from matlab
        mxArray* mxArray = engGetVariable(_engine, name.c_str());

		// Check if variable exists
		if (!mxArray)
			return false;

		SM_ASSERT_EQ(InvalidTypeException, mxIsNumeric(mxArray), 1, "Variable is not numeric");
		SM_ASSERT_EQ(EmptyVariableException, mxIsEmpty(mxArray), 0, "Variable is empty!");
		SM_ASSERT_EQ(InvalidSizeException, mxGetNumberOfDimensions(mxArray), 2, "Variable is 2-dimensional");

		size_t rows = mxGetM(mxArray);
		size_t cols = mxGetN(mxArray);

		rValue.resize(rows, cols);

		void* mxArrayData = mxGetData(mxArray);

		SM_ASSERT_EQ(InvalidTypeException, sizeof(rValue(0,0)), mxGetElementSize(mxArray), "Datatype does not match");
		std::memcpy(rValue.data(), mxArrayData, mxGetElementSize(mxArray)*mxGetNumberOfElements(mxArray));

		// delete array
        mxDestroyArray(mxArray);

		return true;
	}
	
	void Engine::assertValidVariableName(const std::string& name) const
	{
		// Maybe better use regular expressions here?
        SM_ASSERT_EQ(InvalidVariableNameException, name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ01234567890_"), std::string::npos, "Illegal character in Variable name");
        SM_ASSERT_FALSE(InvalidVariableNameException, std::isdigit(name[0]), "First character of variable name must not be a number");
	}
	
	void Engine::assertIsInitialized() const
	{
		SM_ASSERT_NE(EngineNotInitializedException, _engine, NULL, "Matlab Engine is not initialized");
	}

    
  } // namespace matlab
} // namespace sm
