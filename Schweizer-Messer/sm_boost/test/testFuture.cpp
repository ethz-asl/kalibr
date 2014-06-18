#include <gtest/gtest.h>

#include <sm/boost/JobQueue.hpp>

void setTrue(bool * val)
{
   boost::this_thread::sleep(boost::posix_time::seconds(1));
   *val = true;
}

int work(int n)
{
    boost::this_thread::sleep(boost::posix_time::seconds(1));
    int sum = 0;
    for(int i = 0; i < n; ++i)
    {
            sum += i;
    }
    return sum;
}

void testQueue(int nThreads)
{

    sm::JobQueue queue;
    // Start the queue with n threads.
    queue.start(nThreads);


    boost::unique_future<int> job1;    
    queue.scheduleFuture<int>(boost::bind(&work,2000), job1);

    boost::unique_future<int> job2;    
    queue.scheduleFuture<int>(boost::bind(&work,2000), job2);

    boost::unique_future<int> job3;    
    queue.scheduleFuture<int>(boost::bind(&work,2000), job3);

    boost::unique_future<int> job4;    
    queue.scheduleFuture<int>(boost::bind(&work,2000), job4);

    bool complete = false;
    queue.scheduleWork( boost::bind(&setTrue, &complete) );
    ASSERT_FALSE(complete);

    std::cout << "Job 1: " << job1.get() << std::endl;
    
    queue.waitForEmptyQueue();
    ASSERT_TRUE(complete);
    ASSERT_TRUE(queue.empty());
    
    // boost::unique_future<int> job2 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job3 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job4 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job5 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job6 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job7 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job8 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    // boost::unique_future<int> job9 = queue.scheduleBoost<int>(boost::bind(&work,200000));
    
    std::cout << "Job 2: " << job2.get() << std::endl;
    std::cout << "Job 3: " << job3.get() << std::endl;
    std::cout << "Job 4: " << job4.get() << std::endl;
    //std::cout << "Job 2: " << job2.get() << std::endl;
    //std::cout << "Job 9: " << job9.get() << std::endl;

    queue.join();
}


TEST(FuturesTestSuite, testJobQueue1)
{
    testQueue(1);
}

TEST(FuturesTestSuite, testJobQueue2)
{
    testQueue(2);
}

TEST(FuturesTestSuite, testJobQueue8)
{
    testQueue(8);
}

