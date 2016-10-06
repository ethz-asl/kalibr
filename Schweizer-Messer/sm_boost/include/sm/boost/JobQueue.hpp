#ifndef SM_JOB_QUEUE_HPP
#define SM_JOB_QUEUE_HPP

//  Copyright (c) 2007 Braddock Gaskill Distributed under the Boost
//  Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//  exception_ptr.hpp/cpp copyright Peter Dimov 
//  Adapted by Paul Furgale (2010/2011/2012)

#include <deque>

#include <boost/thread/future.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>

namespace sm {

    /**
     * \class JobQueue
     * 
     * A thread pool to execute work in a multithreaded way. The queue
     * isn't highly optimized so the units of work should be big.
     * 
     */
    class JobQueue {
    public:

        JobQueue();

        ~JobQueue();

        /// \brief start the queue processing with n threads.
        void start(int nThreads);

        /// \brief stop the queue processing (but don't block and wait)
        ///        Currently processing items will still finish but
        ///        items not started will remin unprocessed
        void stop();

        /// \brief stop the queue processing and block until currently processing items are complete
        ///        Currently processing items will still finish but
        ///        items not started will remin unprocessed
        void join();

        /// \brief is the queue empty?
        bool empty();

        /// \brief block untill all the items in the queue have been processed.
        ///        Be careful! If the queue has not been started with some threads, this will never exit.
        void waitForEmptyQueue();



        /// \brief schedule a future for processing.
        template <class T>
        void scheduleFuture(boost::function<T (void)> const& fn, boost::unique_future<T> & outFuture); 

        /// \brief submit a unit of work to be processed.
        void scheduleWork(boost::function<void(void)> const & fn);
    protected:
        void exec_loop();

        volatile bool killWorkers_;
        bool started_;
        int threadsInPool_;
        int activeThreads_;
        std::deque<boost::function<void ()> > q_;
        boost::mutex mutex_;
        boost::condition condition_; // signal we wait for when queue is empty
        boost::condition workCondition_; // signal we wait for when waiting for the queue to complete
        boost::shared_ptr<boost::thread_group> work_;
    };

} // end namespace sm

#include "implementation/JobQueue.hpp"

#endif /* SM_JOB_QUEUE_HPP */
