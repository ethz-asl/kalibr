#ifndef PROGRESS_INFO_HPP_
#define PROGRESS_INFO_HPP_

#include <cstddef>
#include <stdexcept>

#include <sm/assert_macros.hpp>

namespace sm {

	void showProgress(double progress);

    template<typename T1, typename T2> 
    void showProgress(T1 done, T2 all)
    {
        SM_ASSERT_GT(std::runtime_error, all, 0, "#DIV0");
        showProgress(static_cast<double>(done) / static_cast<double>(all));
    }

} /* namespace sm */

#endif /* PROGRESS_INFO_HPP_ */
