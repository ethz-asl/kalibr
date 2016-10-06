#ifndef SM_MATLAB_ENGINE_HPP
#define SM_MATLAB_ENGINE_HPP

#include <string>
#include <iostream>

#include <Eigen/Core>
#include <engine.h>

#include <sm/assert_macros.hpp>

namespace sm {
  namespace matlab {
  
	// Settings
    enum SETTINGS {
        OUTPUT_BUFFER_SIZE = 256,
        INPUT_BUFFER_SIZE = 256
    };

	// Exceptions
	SM_DEFINE_EXCEPTION(Exception, std::runtime_error);
	SM_DEFINE_EXCEPTION(MatlabException, Exception);
	
	SM_DEFINE_EXCEPTION(MatlabVariableException, MatlabException);
	SM_DEFINE_EXCEPTION(InvalidVariableNameException, MatlabVariableException);
	SM_DEFINE_EXCEPTION(InvalidTypeException, MatlabVariableException);
	SM_DEFINE_EXCEPTION(InvalidSizeException, MatlabVariableException);
	SM_DEFINE_EXCEPTION(NoSuchVariableException, MatlabVariableException);
	SM_DEFINE_EXCEPTION(EmptyVariableException, MatlabVariableException);

	SM_DEFINE_EXCEPTION(EngineException, MatlabException);
	SM_DEFINE_EXCEPTION(EngineCloseException, EngineException);
	SM_DEFINE_EXCEPTION(EngineNotInitializedException, EngineException);
	SM_DEFINE_EXCEPTION(EnginePutException, EngineException);
	SM_DEFINE_EXCEPTION(EngineGetException, EngineException);
	SM_DEFINE_EXCEPTION(EngineExecutionException, EngineException);
    
    ///
    /// @class Engine
    /// @brief a class that wraps the matlab C engine.
    /// @todo describe how this wrapper work
    ///
    class Engine
    {
    public:
      /// 
      /// Default constructor.
      ///
      Engine();

	  /// 
      /// Default constructor.
      ///
	  Engine(bool startMatlabAtInitialization);

	  ///
	  /// Initialization
	  ///
	  bool initialize();
	  bool stop();

	  ///
	  /// Status check
	  ///
	  bool isInitialized();
	  bool good();

#ifdef UNIX
      /// 
      /// Constructor. Opens the Matlab instance on a different host (UNIX only!)
      ///
      /// @param hostename the name of the host on which the client is opened
      ///
      Engine(std::string hostename);
#endif

	  ~Engine();
      
#ifdef WIN32

      /// 
      /// The Matlab instance is set to visible/invisible (Windows only!)
      ///
      /// @param visible defines if Matlab is visible
      /// @return true if successful
      ///
      bool setVisibility(bool visible) { return engSetVisible(_engine, static_cast<int>(visible)) == 0; }
      
      /// 
      /// The Matlab instance is set to visible/invisible (Windows only!)
      ///
      /// @return true if successful
      ///
      bool isVisible() 
      { 
        bool visible;
        int success = engSetVisible(_engine, visible);
        SM_ASSERT_EQ(EngineException, success, 0, "Determining the visibility was not possible. Matlab error");
        return visible;
      }
#endif

      /// 
      /// Executes a command in Matlab
      ///
      /// @param command The command to be executed (in Matlab syntax)
      /// @return the Matlab output of the command
      ///
      std::string executeCommand(const std::string& command);
      
      /// 
      /// Changes the working directory using the Matlab command "cd"
      ///
      /// @param directory The working directory
      /// @return the Matlab output of the command
      ///
      std::string changeWorkingDirectory(const std::string& directory);
      
      /// 
      /// Provides a command line where matlab commands can be directly typed and executed
      ///
      void openCommandLine();


	  // TESTERS

	  bool exists(const std::string& name);
	  bool isScalar(const std::string& name);
	  bool isEmpty(const std::string& name);
	  bool isCharOrString(const std::string& name);
	  bool getDimensions(const std::string& name, size_t& rows, size_t& cols);

      
      // SETTERS   
	  
      // Matlab stores all variables as doubles so we just template this functions for most Eigen types
      template <typename Derived>
      void putEigen(const std::string& name, const Eigen::MatrixBase<Derived>& matrixBase);

	  // Matlab stores all variables as doubles so we just template this functions for most scalar types
	  template <typename Scalar>
      void put(const std::string& name, const Scalar& value);
      
      // Special handler for strings
      void put(const std::string& name, const std::string& value);
	  
	  // Special handler for booleans
	  void put(const std::string& name, bool value);
    	  

	  // GETTERS
	  // Note: Result is returned in the argument
	  // Return specifies if operation is successful. "False" often means variable is not defined

	  // Special handlers for matrices
	  // Matlab returnes all variables as doubles so we just have on function here
	  // Note: In theory we could also get all integer types according to their types directly accessing the data from the mxArray
	  // However, performance improvement will be negligible in most cases.
      bool getEigen(const std::string& name, Eigen::MatrixXd& rValue);
	  
	  // Matlab returnes all variables as doubles so we just have on function here
	  // Note: In theory we could also get all integer types according to their types directly accessing the data from the mxArray
	  // However, performance improvement will be negligible in most cases.
      bool get(const std::string& name, double& rValue);
	  
	  // Special handler for strings
	  bool get(const std::string& name, std::string& rValue);
	  
	  // Special handler for booleans
	  bool get(const std::string& name, bool& rValue);
	 
   
    private:
      void assertValidVariableName(const std::string& name) const;
	  void assertIsInitialized() const;

      /// The handle for the matlab engine
      ::Engine *_engine;
      
      /// The output buffer in which returns of a command are stored
      char _outputBuffer[OUTPUT_BUFFER_SIZE+1];
      char _inputBuffer[OUTPUT_BUFFER_SIZE+1];
    };


    template <typename Derived>
    void Engine::putEigen(const std::string& name, const Eigen::MatrixBase<Derived>& matrixBase)
    {
		assertIsInitialized();
        assertValidVariableName(name);

		// convert to double
		Eigen::MatrixXd matrix = matrixBase.template cast<double>();
    
        mxArray* mxArray = mxCreateDoubleMatrix(matrix.rows(), matrix.cols(), mxREAL);
        
        SM_ASSERT_EQ(InvalidSizeException, mxGetElementSize(mxArray), sizeof(double), "Data types do not match");
        
        // typecast to double
        Eigen::MatrixXd matrixD = matrix.cast<double>();
    
        // get pointer to mxArray and copy over data
        double* mxArrayData = mxGetPr(mxArray);
        memcpy(mxArrayData, matrixD.data(), matrixD.size()*sizeof(double));
        
        // send data and verify
        int success = engPutVariable(_engine, name.c_str(), mxArray);
        SM_ASSERT_EQ(EnginePutException, success, 0, "Failed to send matrix to Matlab");
        
        mxDestroyArray(mxArray);
    }
	
	template <typename Scalar>
    void Engine::put(const std::string& name, const Scalar& value)
    {
		assertIsInitialized();
        assertValidVariableName(name);
    
        // Matlab stores all variables as doubles so we can just type cast here
        mxArray* mxArray = mxCreateDoubleScalar(static_cast<double>(value));
              
        // send data and verify
        int success = engPutVariable(_engine, name.c_str(), mxArray);
        SM_ASSERT_EQ(EnginePutException, success, 0, "Failed to send matrix to Matlab");
        
        mxDestroyArray(mxArray);
    }
    

  } // namespace matlab
  
} // namespace sm


#endif /* SM_MATLAB_ENGINE_HPP */
