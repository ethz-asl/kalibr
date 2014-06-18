
#include <gtest/gtest.h>
#include <sm/logging.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

void print(int a, const char* fmt, ... ) SMCONSOLE_PRINTF_ATTRIBUTE(2, 3);

TEST(LoggingTestSuite, testBasic)
{
    try {

        sm::logging::setLevel(sm::logging::Level::All);

        int x = 1;
        SM_ALL_STREAM("Hey there: " << x );
        SM_FINEST_STREAM("Hey there: " << x );
        SM_VERBOSE_STREAM("Hey there: " << x );
        SM_FINER_STREAM("Hey there: " << x );
        SM_TRACE_STREAM("Hey there: " << x );
        SM_FINE_STREAM("Hey there: " << x );
        SM_DEBUG_STREAM("Hey there: " << x );
        SM_INFO_STREAM("Hey there: " << x );
        SM_WARN_STREAM("Hey there: " << x );
        SM_ERROR_STREAM("Hey there: " << x );
        SM_FATAL_STREAM("Hey there: " << x );
        SM_INFO_STREAM_NAMED("test", "Hey there: " << x);
        sm::logging::enableNamedStream("test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);
        SM_WARN_NAMED("test", "Hey there: %d",x);
        
        sm::logging::disableNamedStream("test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);

        SM_INFO("This with printf: %d, %f, %s", 1, 1.0, "one");

        for(int i = 0; i < 100; ++i)
        {
            SM_INFO_STREAM_THROTTLE(1.0,"test throttle: " << (++x));
            SM_INFO_THROTTLE(1.0,"test throttle: %d",(++x));
            boost::this_thread::sleep( boost::posix_time::milliseconds(100) );
            
        }
    }
    catch( const std::exception & e )
    {
        FAIL() << e.what();
    }

}
