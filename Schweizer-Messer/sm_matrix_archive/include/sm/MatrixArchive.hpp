#ifndef SM_AMA_MATRIX_IO_HPP
#define SM_AMA_MATRIX_IO_HPP

#include <string>
#include <map>
#include <set>
#include <boost/static_assert.hpp>
#include <boost/filesystem.hpp>
#include <sm/assert_macros.hpp>
#include <Eigen/Core>


// This code only works if a double is 8 bytes.
BOOST_STATIC_ASSERT(sizeof(double) == 8); 
BOOST_STATIC_ASSERT(sizeof(std::string::value_type) == 1); 


namespace sm { 

    SM_DEFINE_EXCEPTION(MatrixArchiveException,std::runtime_error);
    
    class MatrixArchive{
    public:
      typedef std::map< std::string, Eigen::MatrixXd > matrix_map_t;
      typedef std::map< std::string, std::string > string_map_t;

      MatrixArchive();
      ~MatrixArchive();
    
      // clears the matrix and strings archive.
      void clear();
      // clears a specific value from the archive.
      void clear(std::string const & entryName);
      // gets the number of matrices or strings in the archive.
      size_t size() const;
      // gets the number of matrices in the archive.
      size_t sizeMatrices() const;
      // gets the number of strings in the archive.
      size_t sizeStrings() const;

      matrix_map_t::const_iterator begin() const;
      matrix_map_t::const_iterator end() const;
      matrix_map_t::const_iterator find(std::string const & name) const;
    
      // Loads matrices from a file into the archive.
      void load(const std::string & amaFilePath);
      void load(boost::filesystem::path const & amaFilePath);
      void load(boost::filesystem::path const & amaFilePath, std::set<std::string> const & validNames);

      // Saves matrices from a file into the archive.
      void save(const std::string & amaFilePath) const;
      void save(boost::filesystem::path const & amaFilePath) const;
      void save(boost::filesystem::path const & amaFilePath, std::set<std::string> const & validNames) const;
      void save(std::ostream & fout, std::set<std::string> const & validNames) const;

      // Appends matrices to a file.
      void append(std::string const & amaFilePath) const;
      void append(boost::filesystem::path const & amaFilePath) const;
      void append(boost::filesystem::path const & amaFilePath, std::set<std::string> const & validNames) const;

      template<typename Derived>
      void setMatrix(std::string const & matrixName, Eigen::MatrixBase<Derived> const & matrix);
      template<typename Derived>
      void setVector(std::string const & matrixName, Eigen::MatrixBase<Derived> const & matrix);

      void setMatrixXd(std::string const & matrixName, Eigen::MatrixXd const & matrix);
      void setVectorXd(std::string const & matrixName, Eigen::VectorXd const & matrix);
      void setScalar(std::string const & scalarName, double scalar);

      void setString(std::string const & stringName, const std::string & value);

      void getMatrix(std::string const & matrixName, Eigen::MatrixXd & outMatrix) const;
      const Eigen::MatrixXd & getMatrix(std::string const & matrixName) const;
      Eigen::MatrixXd & getMatrix(std::string const & matrixName);
      Eigen::MatrixXd & createMatrix(std::string const & matrixName, int rows, int cols, bool overwriteExisting = false);
      void getVector(std::string const & vectorName, Eigen::VectorXd & outVector) const;
      void getScalar(std::string const & scalarName, double & outScalar) const;
      double getScalar(std::string const & scalarName) const;

      void getString(std::string const & stringName, std::string & stringValue) const;
      const std::string & getString(std::string const & stringName) const;
      std::string & getString(std::string const & stringName);

      const string_map_t & getStrings() const;

      bool isSystemLittleEndian() const;

      size_t maxNameSize();
    private:
      static const size_t s_fixedNameSize;
      static const char s_magicCharStartAMatrixBlock;
      static const char s_magicCharStartAStringBlock;
      static const char s_magicCharEnd;


      void writeMatrixBlock(std::ostream & fout, std::string const & name, Eigen::MatrixXd const & matrix) const;
      void writeMatrixBlockSwapBytes(std::ostream & fout, std::string const & name, Eigen::MatrixXd const & matrix) const;
      void writeStringBlock(std::ostream & fout, std::string const & name, std::string const & stringValue) const;

      void readMatrix(std::istream & fin, Eigen::MatrixXd & matrix) const;
      void readMatrixSwapBytes(std::istream & fin, std::string & name, Eigen::MatrixXd & matrix) const;
      void readString(std::istream & fin, std::string & stringValue) const;

      enum BlockType {
        MATRIX,
        STRING
      };
      BlockType readBlock(std::istream & fin, std::string & name, Eigen::MatrixXd & matrix, std::string & stringValue) const;

      void validateName(std::string const & name, sm::source_file_pos const & sfp) const;
      void writeName(std::ostream & fout, std::string const & name) const;

      void saveMatrices(std::ostream & fout, std::set<std::string> const & validNames) const;
      void saveStrings(std::ostream & fout, std::set<std::string> const & validNames) const;
    
      matrix_map_t m_values;
      string_map_t m_strings;

    }; // end class MatrixArchive

    template<typename Derived>
    void MatrixArchive::setMatrix(std::string const & matrixName, Eigen::MatrixBase<Derived> const & matrix)
    {
      if(m_strings.count(matrixName) > 0){
        m_strings.erase(matrixName);
      }
      validateName(matrixName, SM_SOURCE_FILE_POS);
      m_values[matrixName] = matrix;
    }
  
    template<typename Derived>
    void MatrixArchive::setVector(std::string const & vectorName, Eigen::MatrixBase<Derived> const & vector)
    {
      SM_ASSERT_EQ(MatrixArchiveException, vector.cols(),1, "The input must be a column vector");
      validateName(vectorName, SM_SOURCE_FILE_POS);
      m_values[vectorName] = vector;
    }

  } // end namespace sm


#endif

