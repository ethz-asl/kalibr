#include "sm/logging.hpp"

#include <string>
#include <locale> // for case-insensitive string comparison
#include <algorithm>

/**
 * \brief Returns a Level enum from string
 * \param level level as string
 * \return logging level enum
 */
sm::logging::Level sm::logging::levels::fromString(std::string level) {

  std::transform(level.begin(), level.end(), level.begin(), ::tolower);

  if (level == "all") {
    return Level::All;
  } else if (level == "finest") {
    return Level::Finest;
  } else if (level == "verbose") {
    return Level::Verbose;
  } else if (level == "finer") {
    return Level::Finer;
  } else if (level == "trace") {
    return Level::Trace;
  } else if (level == "fine") {
    return Level::Fine;
  } else if (level == "debug") {
    return Level::Debug;
  } else if (level == "info") {
    return Level::Info;
  } else if (level == "warn") {
    return Level::Warn;
  } else if (level == "error") {
    return Level::Error;
  } else if (level == "fatal") {
    return Level::Fatal;
  } else {
    SM_ERROR_STREAM("Invalid logging level " << level << ", setting to Level::Info");
    return Level::Info;
  }

}
