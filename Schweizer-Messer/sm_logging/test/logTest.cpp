
#include <gtest/gtest.h>
#include <sm/logging.hpp>
#include <sm/logging/StdOutLogger.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

/// \brief Helper logger to catch console output
class TestLogger : public sm::logging::StdOutLogger {
 public:
  TestLogger() : sm::logging::StdOutLogger() {
    formatter.doColor_ = false;
  }
  virtual ~TestLogger() { }
  std::string string() { std::string str = _str; _str.clear(); return str; }
 protected:
  virtual void logImplementation(const sm::logging::LoggingEvent & event) override {
    std::ostringstream os;
    os << std::setprecision(6);
    formatter.print(event, os);
    _str = os.str();
  }
  virtual Time currentTimeImplementation() const {
    return Time();
  }
 private:
  std::string _str;
};


TEST(LoggingTestSuite, testBasic) {
    try {

        sm::logging::setLevel(sm::logging::Level::All);
        boost::shared_ptr<TestLogger> logger(new TestLogger());
        sm::logging::setLogger(logger);

        int x = 1;
        std::ostringstream os;
        os << " [0.000000]: Hey there: " << x << std::endl;
        const std::string expected = os.str();

        SM_ALL_STREAM("Hey there: " << x );
        EXPECT_EQ("[  ALL]" + expected, logger->string());
        SM_FINEST_STREAM("Hey there: " << x );
        EXPECT_EQ("[FINES]" + expected, logger->string());
        SM_VERBOSE_STREAM("Hey there: " << x );
        EXPECT_EQ("[VERBO]" + expected, logger->string());
        SM_FINER_STREAM("Hey there: " << x );
        EXPECT_EQ("[FINER]" + expected, logger->string());
        SM_TRACE_STREAM("Hey there: " << x );
        EXPECT_EQ("[TRACE]" + expected, logger->string());
        SM_FINE_STREAM("Hey there: " << x );
        EXPECT_EQ("[ FINE]" + expected, logger->string());
        SM_DEBUG_STREAM("Hey there: " << x );
        EXPECT_EQ("[DEBUG]" + expected, logger->string());
        SM_INFO_STREAM("Hey there: " << x );
        EXPECT_EQ("[ INFO]" + expected, logger->string());
        SM_WARN_STREAM("Hey there: " << x );
        EXPECT_EQ("[ WARN]" + expected, logger->string());
        SM_ERROR_STREAM("Hey there: " << x );
        EXPECT_EQ("[ERROR]" + expected, logger->string());
        SM_FATAL_STREAM("Hey there: " << x );
        EXPECT_EQ("[FATAL]" + expected, logger->string());

        // test named streams
        sm::logging::disableNamedStream("sm");
        SM_INFO_STREAM("Hey there: " << x);
        EXPECT_EQ("", logger->string()); // all non-named streams disabled
        sm::logging::enableNamedStream("sm");
        SM_INFO_STREAM("Hey there: " << x);
        EXPECT_EQ("[ INFO]" + expected, logger->string()); // all non-named streams enabled again

        SM_INFO_STREAM_NAMED("test", "Hey there: " << x);
        EXPECT_EQ("", logger->string()); // not enabled yet
        sm::logging::enableNamedStream("test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);
        EXPECT_EQ("[ WARN]" + expected, logger->string());
        SM_WARN_NAMED("test", "Hey there: %d",x);
        EXPECT_EQ("[ WARN]" + expected, logger->string());
        sm::logging::disableNamedStream("test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);
        EXPECT_EQ("", logger->string());

        // test deprecation solution
        sm::logging::enableNamedStream("sm.test");
        SM_WARN_STREAM_NAMED("test", "Hey there: " << x);
        EXPECT_EQ("[ WARN]" + expected, logger->string());

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

TEST(LoggingTestSuite, testLevel) {
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("All");
    EXPECT_EQ(sm::logging::Level::All, level);
    level = sm::logging::levels::fromString("all");
    EXPECT_EQ(sm::logging::Level::All, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Finest");
    EXPECT_EQ(sm::logging::Level::Finest, level);
    level = sm::logging::levels::fromString("finest");
    EXPECT_EQ(sm::logging::Level::Finest, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Verbose");
    EXPECT_EQ(sm::logging::Level::Verbose, level);
    level = sm::logging::levels::fromString("verbose");
    EXPECT_EQ(sm::logging::Level::Verbose, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Finer");
    EXPECT_EQ(sm::logging::Level::Finer, level);
    level = sm::logging::levels::fromString("finer");
    EXPECT_EQ(sm::logging::Level::Finer, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Trace");
    EXPECT_EQ(sm::logging::Level::Trace, level);
    level = sm::logging::levels::fromString("trace");
    EXPECT_EQ(sm::logging::Level::Trace, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Fine");
    EXPECT_EQ(sm::logging::Level::Fine, level);
    level = sm::logging::levels::fromString("fine");
    EXPECT_EQ(sm::logging::Level::Fine, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Debug");
    EXPECT_EQ(sm::logging::Level::Debug, level);
    level = sm::logging::levels::fromString("debug");
    EXPECT_EQ(sm::logging::Level::Debug, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Info");
    EXPECT_EQ(sm::logging::Level::Info, level);
    level = sm::logging::levels::fromString("info");
    EXPECT_EQ(sm::logging::Level::Info, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Warn");
    EXPECT_EQ(sm::logging::Level::Warn, level);
    level = sm::logging::levels::fromString("warn");
    EXPECT_EQ(sm::logging::Level::Warn, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Error");
    EXPECT_EQ(sm::logging::Level::Error, level);
    level = sm::logging::levels::fromString("error");
    EXPECT_EQ(sm::logging::Level::Error, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("Fatal");
    EXPECT_EQ(sm::logging::Level::Fatal, level);
    level = sm::logging::levels::fromString("fatal");
    EXPECT_EQ(sm::logging::Level::Fatal, level);
  }
  {
    sm::logging::Level level;
    level = sm::logging::levels::fromString("nonsense");
    EXPECT_EQ(sm::logging::Level::Info, level);
  }
}
