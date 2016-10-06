import numpy_eigen
import inspect

from libsm_python import *
from plotCoordinateFrame import *
from Progress import Progress, Progress2
from saveFigTight import saveFigTight


def log(level, message, stackdepth=1):
    if getLoggingLevel() > level:
        return
    callerframerecord = inspect.stack()[stackdepth]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLog(level, info.filename, info.lineno, info.function, message)
    
def logNamed(name, level, message, stackdepth=1):
    if getLoggingLevel() > level:
        return
    callerframerecord = inspect.stack()[stackdepth]
    frame = callerframerecord[0]
    info = inspect.getframeinfo(frame)
    rawLogNamed(name,level, info.filename, info.lineno, info.function, message)

def logFinest(message):
    log(LoggingLevel.Finest, message, stackdepth=2)
    
def logVerbose(message):
    log(LoggingLevel.FineVerbose, message, stackdepth=2)
    
def logFiner(message):
    log(LoggingLevel.Finer, message, stackdepth=2)
    
def logTrace(message):
    log(LoggingLevel.Trace, message, stackdepth=2)
    
def logFine(message):
    log(LoggingLevel.Fine, message, stackdepth=2)
    
def logInfo(message):
    log(LoggingLevel.Info, message, stackdepth=2)

def logDebug(message):
    log(LoggingLevel.Debug, message, stackdepth=2)

def logWarn(message):
    log(LoggingLevel.Warn, message, stackdepth=2)

def logFatal(message):
    log(LoggingLevel.Fatal, message, stackdepth=2)

def logError(message):
    log(LoggingLevel.Error, message, stackdepth=2)


def logFinestNamed(name, message):
    logNamed(name, LoggingLevel.Finest, message, stackdepth=2)
    
def logVerboseNamed(name, message):
    logNamed(name, LoggingLevel.Verbose, message, stackdepth=2)
    
def logFinerNamed(name, message):
    logNamed(name, LoggingLevel.Finer, message, stackdepth=2)
    
def logTraceNamed(name, message):
    logNamed(name, LoggingLevel.Trace, message, stackdepth=2)
    
def logFineNamed(name, message):
    logNamed(name, LoggingLevel.Fine, message, stackdepth=2)

def logInfoNamed(name,message):
    logNamed(name, LoggingLevel.Info, message, stackdepth=2)

def logDebugNamed(name,message):
    logNamed(name, LoggingLevel.Debug, message, stackdepth=2)

def logWarnNamed(name,message):
    logNamed(name, LoggingLevel.Warn, message, stackdepth=2)

def logFatalNamed(name,message):
    logNamed(name, LoggingLevel.Fatal, message, stackdepth=2)

def logErrorNamed(name,message):
    logNamed(name, LoggingLevel.Error, message, stackdepth=2)
