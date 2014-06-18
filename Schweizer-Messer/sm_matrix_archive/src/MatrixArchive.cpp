#include <fstream>
#include <cstdio>
#include <boost/algorithm/string/trim.hpp>
#include <sm/MatrixArchive.hpp>

namespace sm 
{

  const size_t MatrixArchive::s_fixedNameSize = 32;
  const char MatrixArchive::s_magicCharStartAMatrixBlock = 'A';
  const char MatrixArchive::s_magicCharStartAStringBlock = 'S';
  const char MatrixArchive::s_magicCharEnd = 'B';

  MatrixArchive::MatrixArchive()
  {
    // 0
  }

  MatrixArchive::~MatrixArchive()
  {
    // 0
  }
    
  // clears the matrix archive.
  void MatrixArchive::clear()
  {
    m_values.clear();
    m_strings.clear();
  }

  // clears a specific value from the archive.
  void MatrixArchive::clear(std::string const & entryName)
  {
    m_values.erase(entryName);
    m_values.erase(entryName);
  }

  void MatrixArchive::setScalar(std::string const & scalarName, double scalar)
  {
    validateName(scalarName,SM_SOURCE_FILE_POS);
    Eigen::MatrixXd & M = m_values[scalarName];
    M.resize(1,1);
    M(0,0) = scalar;
  }
  
  void MatrixArchive::setString(std::string const & stringName, std::string const & value)
  {
    validateName(stringName, SM_SOURCE_FILE_POS);
    if(m_values.count(stringName) > 0){
      m_values.erase(stringName);
    }
    m_strings[stringName] = value;
  }

  const MatrixArchive::string_map_t & MatrixArchive::getStrings() const
  {
    return m_strings;
  }


  void MatrixArchive::getMatrix(std::string const & matrixName, Eigen::MatrixXd & outMatrix) const
  {
    outMatrix = getMatrix(matrixName);
  }

  Eigen::MatrixXd & MatrixArchive::createMatrix(std::string const & matrixName, int rows, int cols, bool overwriteExisting)
  {
    if(!overwriteExisting){
      matrix_map_t::const_iterator it = m_values.find(matrixName);
      if(it != m_values.end())
      {
        SM_THROW(MatrixArchiveException,"There is already a matrix of name \"" << matrixName << "\" in the archive");
      }
    }

    Eigen::MatrixXd & val = m_values[matrixName];
    m_values[matrixName].resize(rows, cols);
    return val;
  }

  const Eigen::MatrixXd & MatrixArchive::getMatrix(std::string const & matrixName) const
  {
    matrix_map_t::const_iterator it = m_values.find(matrixName);
    if(it == m_values.end())
    {
      SM_THROW(MatrixArchiveException,"There is no matrix named \"" << matrixName << "\" in the archive");
    }

    return it->second;
  }
  Eigen::MatrixXd & MatrixArchive::getMatrix(std::string const & matrixName)
  {
    return const_cast<Eigen::MatrixXd &>(const_cast<const MatrixArchive &>(*this).getMatrix(matrixName));
  }

  void MatrixArchive::getVector(std::string const & vectorName, Eigen::VectorXd & outVector) const
  {
    matrix_map_t::const_iterator it = m_values.find(vectorName);
    if(it == m_values.end())
    {
      SM_THROW(MatrixArchiveException, "There is no vector named \"" << vectorName << "\" in the archive");
    }
    Eigen::MatrixXd const & M = it->second;
    SM_ASSERT_EQ(MatrixArchiveException, M.cols(), 1, "The stored value is not a vector");

    outVector = M.col(0);
  }

  void MatrixArchive::getScalar(std::string const & scalarName, double & outScalar) const
  {
    outScalar = getScalar(scalarName);
  }
  double MatrixArchive::getScalar(std::string const & scalarName) const
  {
    matrix_map_t::const_iterator it = m_values.find(scalarName);
    if(it == m_values.end())
    {
      SM_THROW(MatrixArchiveException, "There is no scalar named \"" << scalarName << "\" in the archive");
    }

    Eigen::MatrixXd const & M = it->second;
    SM_ASSERT_EQ(MatrixArchiveException, M.rows(), 1, "The stored value is not a scalar");
    SM_ASSERT_EQ(MatrixArchiveException, M.cols(), 1, "The stored value is not a scalar");

    return M(0,0);
  }
  
  void MatrixArchive::getString(std::string const & stringName, std::string & stringValue) const {
    stringValue = getString(stringName);
  }

  const std::string & MatrixArchive::getString(std::string const & stringName) const {
    return const_cast<MatrixArchive*>(this)->getString(stringName);
  }

  std::string & MatrixArchive::getString(std::string const & stringName) {
    string_map_t::iterator it = m_strings.find(stringName);
    if(it == m_strings.end())
    {
      SM_THROW(MatrixArchiveException, "There is no string named \"" << stringName << "\" in the archive");
    }
    return it->second;
  }

  void MatrixArchive::validateName(std::string const & name, sm::source_file_pos const & /* sfp */) const
  {
    if(name.size() > s_fixedNameSize || name.size() == 0)
    {
      SM_THROW(MatrixArchiveException, "The name \"" << name << "\" is an incorrect size. Names length must be between 1 and " << s_fixedNameSize);
    }

    SM_ASSERT_TRUE(MatrixArchiveException, isalpha(name[0]), "The name \"" << name << "\" is invalid. The first character of the name must be a letter");

    for(unsigned i = 1; i < name.size(); i++)
    {
      SM_ASSERT_TRUE(MatrixArchiveException, isalnum(name[i]) || name[i] == '_', "The name \"" << name << "\" is invalid. The characters of the name must be alphanumeric or an underscore. (failed at character " << i << ")");
    }
  }


  bool MatrixArchive::isSystemLittleEndian() const
  {
    short int word = 0x0001;
    char *byte = (char *) &word;
    return byte[0] != 0;
  }

  size_t MatrixArchive::size() const
  {
    return sizeStrings() + sizeMatrices();
  }
  size_t MatrixArchive::sizeStrings() const
  {
    return m_strings.size();
  }
  size_t MatrixArchive::sizeMatrices() const
  {
    return m_strings.size();
  }

  MatrixArchive::matrix_map_t::const_iterator MatrixArchive::begin() const
  {
    return m_values.begin();
  }

  MatrixArchive::matrix_map_t::const_iterator MatrixArchive::end() const
  {
    return m_values.end();
  }

  MatrixArchive::matrix_map_t::const_iterator MatrixArchive::find(std::string const & name) const
  {
    return m_values.find(name);
  }

  void MatrixArchive::writeName(std::ostream & fout, std::string const & name) const
  {
    // fixed size character name
    validateName(name, SM_SOURCE_FILE_POS);
    fout.fill(' ');
    fout.width(s_fixedNameSize);
    fout << name;
  }

  void MatrixArchive::writeMatrixBlock(std::ostream & fout, std::string const & name, Eigen::MatrixXd const & matrix) const
  {
    // start character
    fout.write(&s_magicCharStartAMatrixBlock,1);

    writeName(fout, name);

    // 4 byte rows
    boost::uint32_t rows = matrix.rows();
    fout.write(reinterpret_cast<const char *>(&rows), 4);

    // 4 byte columns
    boost::uint32_t cols = matrix.cols();
    fout.write(reinterpret_cast<const char *>(&cols), 4);

    // data
    boost::uint32_t dataSize = rows * cols * sizeof(double);
    fout.write(reinterpret_cast<const char *>(matrix.data()),dataSize);

    // std::cout << "Writing matrix " << name << ", size: " << rows << "x" << cols << ", dataSize: " << dataSize << std::endl;
    // end character
    fout.write(&s_magicCharEnd,1);
  }


  void MatrixArchive::writeMatrixBlockSwapBytes(std::ostream & /* fout */, std::string const & /* name */, Eigen::MatrixXd const & /* matrix */) const
  {
    SM_THROW(MatrixArchiveException, "Not Implemented");
  }
  
  void MatrixArchive::readMatrix(std::istream & fin, Eigen::MatrixXd & matrix) const
  {
    // 4 byte rows
    boost::uint32_t rows;
    fin.read(reinterpret_cast<char *>(&rows),4);
    // 4 byte columns
    boost::uint32_t cols;
    fin.read(reinterpret_cast<char *>(&cols),4);
    
    // data

    matrix.resize(rows,cols);
    std::streamsize dataSize = rows * cols * sizeof(double);
    //std::cout << "Reading matrix \"" << name << "\", size: " << rows << "x" << cols << ", dataSize: " << dataSize << std::endl; 
    fin.read(reinterpret_cast<char *>(matrix.data()),dataSize);
  }

  void MatrixArchive::writeStringBlock(std::ostream & fout, std::string const & name, std::string const & stringValue) const
  {
    // start character
    fout.write(&s_magicCharStartAStringBlock,1);

    writeName(fout, name);

    // 4 byte rows
    boost::uint32_t stringSize = stringValue.size();
    fout.write(reinterpret_cast<const char *>(&stringSize), 4);

    fout.write(&stringValue[0], stringSize);

    // end character
    fout.write(&s_magicCharEnd,1);
  }

  void MatrixArchive::readString(std::istream & fin, std::string & stringValue) const
  {
    // 4 byte rows
    boost::uint32_t stringSize;
    fin.read(reinterpret_cast<char *>(&stringSize),4);

    // data
    stringValue.resize(stringSize);
    fin.read(&stringValue[0], stringSize);
  }

  MatrixArchive::BlockType MatrixArchive::readBlock(std::istream & fin, std::string & name, Eigen::MatrixXd & matrix, std::string & stringValue) const
  {
    char start, end;
    // start character
    fin.read(&start,1);
    BlockType blockType;
    if(start == s_magicCharStartAStringBlock){
      blockType = STRING;
    }
    else{
      SM_ASSERT_EQ(MatrixArchiveException, start, s_magicCharStartAMatrixBlock, "The block didn't start with the expected character");
      blockType = MATRIX;
    }

    // 64 character name
    char nameBuffer[s_fixedNameSize + 1];
    nameBuffer[s_fixedNameSize] = '\0';
    fin.read(nameBuffer,(std::streamsize)s_fixedNameSize);
    name = nameBuffer;
    boost::trim(name);

    switch(blockType){
      case MATRIX:
        readMatrix(fin, matrix);
        break;
      case STRING:
        readString(fin, stringValue);
        break;
    }

    // end character
    fin.read(&end,1);
    SM_ASSERT_EQ(MatrixArchiveException, end, s_magicCharEnd, "The matrix block didn't end with the expected character");
    return blockType;
  }

  void MatrixArchive::readMatrixSwapBytes(std::istream & /* fin */, std::string & /* name */, Eigen::MatrixXd & /* matrix */) const
  {
    SM_THROW(MatrixArchiveException, "Not Implemented");
  }

  // Loads matrices from a file into the archive.
  void MatrixArchive::load(std::string const & amaFilePath)
  {
    std::set<std::string> names;
    load(amaFilePath,names);
  }
  void MatrixArchive::load(boost::filesystem::path const & amaFilePath)
  {
    std::set<std::string> names;
    load(amaFilePath,names);
  }
  
  // Saves matrices from a file into the archive.
  void MatrixArchive::save(std::string const & amaFilePath) const
  {
    std::set<std::string> names;
    save(amaFilePath, names);
  }


  void MatrixArchive::save(boost::filesystem::path const & amaFilePath) const
  {
    std::set<std::string> names;
    save(amaFilePath, names);
  }
  

  // Appends matrices to a file.
  void MatrixArchive::append(std::string const & amaFilePath) const
  {
    std::set<std::string> names;
    append(amaFilePath,names);
  }

  void MatrixArchive::append(boost::filesystem::path const & amaFilePath) const
  {
    std::set<std::string> names;
    append(amaFilePath,names);
  }

  void MatrixArchive::save(boost::filesystem::path const & amaFilePath, std::set<std::string> const & validNames) const
  {
    std::ofstream fout(amaFilePath.string().c_str(), std::ios::binary);
    SM_ASSERT_TRUE(MatrixArchiveException, fout.good(), "Unable to open file " << amaFilePath.string() << " for writing");

    save(fout, validNames);

    fout.close();
  }

  void MatrixArchive::save(std::ostream & fout, std::set<std::string> const & validNames) const
  {
    saveMatrices(fout, validNames);
    saveStrings(fout, validNames);
  }

  void MatrixArchive::saveMatrices(std::ostream & fout, std::set<std::string> const & validNames) const
  {
    matrix_map_t::const_iterator it = m_values.begin();
    for( ; it != m_values.end(); it++)
    {
      if(validNames.empty() || validNames.count(it->first) > 0)
      {
        if(isSystemLittleEndian())
        {
          writeMatrixBlock(fout, it->first, it->second);
        }
        else
        {
          writeMatrixBlockSwapBytes(fout, it->first, it->second);
        }
        SM_ASSERT_TRUE(MatrixArchiveException, fout.good(), "Error while writing matrix " << it->first << " to file.");
      }
    }
  }
  void MatrixArchive::saveStrings(std::ostream & fout, std::set<std::string> const & validNames) const
  {
    string_map_t::const_iterator it = m_strings.begin();
    for( ; it != m_strings.end(); it++)
    {
      if(validNames.empty() || validNames.count(it->first) > 0)
      {
        writeStringBlock(fout, it->first, it->second);

        SM_ASSERT_TRUE(MatrixArchiveException, fout.good(), "Error while writing string " << it->first << " to file.");
      }
    }
  }

  void MatrixArchive::load(boost::filesystem::path const & amaFilePath, std::set<std::string> const & validNames)
  {
    std::ifstream fin(amaFilePath.string().c_str(), std::ios::binary);
    SM_ASSERT_TRUE(MatrixArchiveException, fin.good(), "Unable to open file " << amaFilePath << " for reading");

    std::string name, valueString;
    Eigen::MatrixXd matrix;
    fin.peek();
    while(!fin.eof())
    {
      BlockType blockType = readBlock(fin, name, matrix, valueString);

      if(validNames.empty() || validNames.count(name) > 0)
      {
        validateName(name,SM_SOURCE_FILE_POS);
        switch(blockType){
          case MATRIX:
            m_values[name] = matrix;
            break;
          case STRING:
            m_strings[name] = valueString;
            break;
        }
      }
      fin.peek();
    }

  }

  void MatrixArchive::append(boost::filesystem::path const & amaFilePath, std::set<std::string> const & /* validNames */) const
  {
    std::ofstream fout(amaFilePath.string().c_str(), std::ios::binary | std::ios::app);
    SM_ASSERT_TRUE(MatrixArchiveException, fout.good(), "Unable to open file " << amaFilePath.string() << " for writing");
  }

  size_t MatrixArchive::maxNameSize()
  {
    return s_fixedNameSize;
  }

  void MatrixArchive::setMatrixXd(std::string const & matrixName, Eigen::MatrixXd const & matrix)
  {
    setMatrix(matrixName, matrix);
  }
  void MatrixArchive::setVectorXd(std::string const & matrixName, Eigen::VectorXd const & matrix)
  {
    setVector(matrixName, matrix);
  }
} // namespace sm

