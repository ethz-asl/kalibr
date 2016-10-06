/*
 * TestCpp.cpp
 *
 *  Created on: May 17, 2013
 *      Author: hannes
 */
#include <gtest/gtest.h>
#include <unistd.h>

#include <sm/MatrixArchive.hpp>

TEST(MatrixArchive, testMatrixLoadAndSaveWorkTogether) {
  try {
    SCOPED_TRACE("");
    Eigen::MatrixXd testMat = Eigen::MatrixXd::Random(1, 1);
    std::string testString = "testString";
    sm::MatrixArchive archive;
    std::string tempfile("/tmp/testMatrixArchive.ama");

    archive.setMatrix("m", testMat);
    archive.setString("s", testString);

    archive.save(tempfile);

    archive.clear();
    ASSERT_EQ(0, archive.size());

    archive.load(tempfile);

    ASSERT_EQ(testString, archive.getString("s"));
    auto & resultMat = archive.getMatrix("m");

    ASSERT_EQ(testMat.rows(), resultMat.rows());
    ASSERT_EQ(testMat.cols(), resultMat.cols());
    ASSERT_EQ(testMat(0, 0), resultMat(0,0));

    unlink(tempfile.c_str());
  } catch (const std::exception & e) {
    FAIL()<< e.what();
  }
}
