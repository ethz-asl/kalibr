import numpy_eigen
import inspect

from libsm_python import *
from plotCoordinateFrame import plotCoordinateFrame
from Progress import Progress
from Progress import Progress2
from saveFigTight import saveFigTight


def logInfo(message):
    if getLoggingLevel() > LoggingLevel.Info:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(LoggingLevel.Info, info.filename, info.lineno, info.function, message)

def logDebug(message):
    if getLoggingLevel() > LoggingLevel.Debug:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(LoggingLevel.Debug, info.filename, info.lineno, info.function, message)

def logWarn(message):
    if getLoggingLevel() > LoggingLevel.Warn:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(LoggingLevel.Warn, info.filename, info.lineno, info.function, message)

def logFatal(message):
    if getLoggingLevel() > LoggingLevel.Fatal:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(LoggingLevel.Fatal, info.filename, info.lineno, info.function, message)

def logError(message):
    if getLoggingLevel() > LoggingLevel.Error:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(LoggingLevel.Error, info.filename, info.lineno, info.function, message)

def log(level,message):
    if getLoggingLevel() > level:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(level, info.filename, info.lineno, info.function, message)


def logInfoNamed(name,message):
    if getLoggingLevel() > LoggingLevel.Info:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,LoggingLevel.Info, info.filename, info.lineno, info.function, message)

def logDebugNamed(name,message):
    if getLoggingLevel() > LoggingLevel.Debug:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,LoggingLevel.Debug, info.filename, info.lineno, info.function, message)

def logWarnNamed(name,message):
    if getLoggingLevel() > LoggingLevel.Warn:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,LoggingLevel.Warn, info.filename, info.lineno, info.function, message)

def logFatalNamed(name,message):
    if getLoggingLevel() > LoggingLevel.Fatal:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,LoggingLevel.Fatal, info.filename, info.lineno, info.function, message)

def logErrorNamed(name,message):
    if getLoggingLevel() > LoggingLevel.Error:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,LoggingLevel.Error, info.filename, info.lineno, info.function, message)

def logNamed(name,level,message):
    if getLoggingLevel() > level:
        return
    callerframerecord = inspect.stack()[1]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,level, info.filename, info.lineno, info.function, message)
