import time
import datetime
import sys

class Progress(object):
    def __init__(self, numIterations):
        self.started = False
        self.elapsed = 0
        self.startTime = 0
        self.numIterations = numIterations
        self.iteration = 0

    def sample(self):
        if self.started:
            self.iteration = self.iteration + 1
            self.elapsed = time.time() - self.startTime
            timePerRun = self.elapsed / self.iteration
            totalTime = self.numIterations * timePerRun
        
            print "Progress %d / %d" % (self.iteration, self.numIterations)
            print "Time %s / %s  (%s * %d) " % (datetime.timedelta(seconds=self.elapsed), datetime.timedelta(seconds=totalTime), datetime.timedelta(seconds=timePerRun), self.numIterations)

        else:
            self.startTime = time.time()
            self.iteration = 0
            self.started = True


class Progress2(object):
    def __init__(self, numIterations):
        """
        Progress tracker that calculates and prints the time until a process is finished.
        
        example usage:
            import sm
            import time
            
            numIter = 10
            progress = sm.Progress2(numIter)
            for iter in range(0, numIter):
                progress.sample()
                time.sleep(1)
        """
        self.started = False
        self.elapsed = 0
        self.startTime = 0
        self.numIterations = numIterations
        self.iteration = 0

    def sample(self, steps=1):
        """
        Call this function at each iteration. It prints the remaining steps and time.
        """
        if self.started:
            self.iteration = self.iteration + steps
            self.elapsed = time.time() - self.startTime
            timePerRun = self.elapsed / self.iteration
            totalTime = self.numIterations * timePerRun
            
            m, s = divmod(totalTime-self.elapsed, 60)
            h, m = divmod(m, 60)
            t_remaining_str = ""
            if h > 0: t_remaining_str = "%d h " % h
            if m > 0: t_remaining_str = t_remaining_str + "%dm " % m
            if s > 0: t_remaining_str = t_remaining_str + "%ds" % s
            print "\r  Progress {0} / {1} \t Time remaining: {2}                 ".format(self.iteration, self.numIterations, t_remaining_str),
            sys.stdout.flush()
        else:
            self.startTime = time.time()
            self.iteration = 0
            self.started = True
        
    def reset(self, numIterations = -1):
        """
        Reset the progress tracker
        """
        self.started = False
        self.elapsed = 0
        self.startTime = 0
        self.iteration = 0
        if numIterations != -1:
            self.numIterations = numIterations