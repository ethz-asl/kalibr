/*
 * Timing.cpp
 *
 *  Created on: 18.12.2015
 *      Author: Ulrich Schwesinger
 */

#include <numpy_eigen/boost_python_headers.hpp>
#include <sm/timing/Timer.hpp>

void printTiming()
{
    sm::timing::Timing::print(std::cout);
}

void printTimingSorted(const sm::timing::SortType sortType)
{
    sm::timing::Timing::print(std::cout, sortType);
}

void exportTiming()
{
  using namespace boost::python;
  using namespace sm::timing;

  enum_<SortType>("TimingSortType")
      .value("SORT_BY_TOTAL", SortType::SORT_BY_TOTAL)
      .value("SORT_BY_MEAN", SortType::SORT_BY_MEAN)
      .value("SORT_BY_STD", SortType::SORT_BY_STD)
      .value("SORT_BY_MIN", SortType::SORT_BY_MIN)
      .value("SORT_BY_MAX", SortType::SORT_BY_MAX)
      .value("SORT_BY_NUM_SAMPLES", SortType::SORT_BY_NUM_SAMPLES)
  ;

  def("printTiming", &printTiming);
  def("printTimingSorted", &printTimingSorted);
}
