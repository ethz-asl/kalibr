

namespace sm {
    
    namespace detail {
        template<typename T>
        struct taskWrapper
        {
            boost::shared_ptr<boost::packaged_task<T> > _task;
            
            taskWrapper( boost::function<T(void)> const & fn ) :
                _task( new boost::packaged_task<T>(fn) ) {}
            
            ~taskWrapper() {
            }
            
            void operator()(){  (*_task)(); }
        };


    } // namespace detail

    template <class T>
    void JobQueue::scheduleFuture(boost::function<T (void)> const& fn, boost::unique_future<T> & outFuture) 
    {

        detail::taskWrapper<T> task(fn);
        outFuture = task._task->get_future();
        scheduleWork(task);
    }


} // namespace sm
