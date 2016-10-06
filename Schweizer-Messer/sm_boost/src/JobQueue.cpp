#include <sm/boost/JobQueue.hpp>

namespace sm {
    
    JobQueue::JobQueue() : killWorkers_(false), started_(false), threadsInPool_(0), activeThreads_(0) {}
    JobQueue::~JobQueue() { // we must kill thread before we're dead
        boost::mutex::scoped_lock lck(mutex_);
        killWorkers_ = true; // flag to tell thread to die
        condition_.notify_all();
        while (threadsInPool_) // all threads must exit
            condition_.wait(lck);
    }

    void JobQueue::scheduleWork(boost::function<void(void)> const & fn)
    {
        boost::mutex::scoped_lock lck(mutex_);
        q_.push_back(fn); //queue the job
        condition_.notify_all(); // wake worker thread(s)
    }

    void JobQueue::start(int nThreads) {
        if(!work_)
        {
            work_.reset(new boost::thread_group);
            for(int i = 0; i < nThreads; ++i)
            {
                work_->create_thread(boost::bind(&JobQueue::exec_loop,this));
            }
        }
    }

    void JobQueue::stop() {
        killWorkers_ = true;
    }

    void JobQueue::join()
    {
        stop();
        condition_.notify_all();
        work_->join_all();
    }

    bool JobQueue::empty()
    {
        boost::mutex::scoped_lock lck(mutex_);
        return q_.empty();
    }

    void JobQueue::waitForEmptyQueue()
    {
        boost::mutex::scoped_lock lck(mutex_);
        while( activeThreads_ || !q_.empty() )
        {
            workCondition_.wait(lck);
        }
    }

    void JobQueue::exec_loop() {
        boost::mutex::scoped_lock lck(mutex_);
        ++threadsInPool_;
        while (true) {
            while ((q_.empty()) && !killWorkers_)
                condition_.wait(lck); // wait for a job to be added to queue
            if (killWorkers_) {
                --threadsInPool_;
                condition_.notify_all();
                return;
            }
            boost::function<void ()> f = q_.front();
            q_.pop_front();
            ++activeThreads_;
            // unlock the queue while we exec the job
            lck.unlock(); 
            // call the work function
            f(); 
            lck.lock();
            --activeThreads_;
            workCondition_.notify_all();
        }
    }


} // namespace sm
