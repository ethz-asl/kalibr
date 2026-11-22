import sm

import numpy as np
import sys
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import time
import copy
import cv2

# Sentinel pushed onto the bounded task queue -- one per worker -- so a
# worker can tell "no more real work is ever coming" apart from "the queue
# is momentarily empty because the producer hasn't caught up yet". See the
# big comment inside extractCornersFromDataset() for why this is needed.
_WORKER_STOP_SENTINEL = '__kalibr_worker_stop__'

# Safety net only: how long a worker will block on an empty queue before
# giving up and exiting on its own. Normal shutdown is always via
# _WORKER_STOP_SENTINEL; this just guarantees a worker can never hang
# forever if the producer process dies without feeding a sentinel.
_WORKER_IDLE_TIMEOUT_SEC = 120


def _putWithLivenessCheck(taskq, item, workers, poll_timeout=1.0):
    """Blocking put() that also notices a dead worker pool.

    A plain blocking taskq.put() on a full queue would hang forever if
    every worker process has already exited (e.g. all of them crashed on a
    bad frame) -- polling with a short timeout lets the producer notice
    that and raise instead of deadlocking the whole extraction.
    """
    while True:
        try:
            taskq.put(item, timeout=poll_timeout)
            return
        except queue.Full:
            if all(not p.is_alive() for p in workers):
                raise RuntimeError(
                    "All multithreaded-extraction worker processes exited "
                    "while the task queue was still full; aborting instead "
                    "of deadlocking.")


def multicoreExtractionWrapper(detector, taskq, resultq, progressq, clearImages, noTransformation):
    while True:
        try:
            task = taskq.get(timeout=_WORKER_IDLE_TIMEOUT_SEC)
        except queue.Empty:
            # The producer should always send a sentinel before exiting.
            # If we time out here, the producer died unexpectedly -- exit
            # rather than block forever.
            return

        if task == _WORKER_STOP_SENTINEL:
            return

        idx = task[0]
        stamp = task[1]
        image = task[2]

        if noTransformation:
            success, obs = detector.findTargetNoTransformation(stamp, np.array(image))
        else:
            success, obs = detector.findTarget(stamp, np.array(image))

        if clearImages:
            obs.clearImage()
        if success:
            resultq.put( (obs, idx) )

        # Report one completion (success or failure) so the parent can
        # derive progress without taskq.qsize(), which is meaningless now
        # that the queue is bounded and continuously refilled.
        progressq.put(1)

def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    print("Extracting calibration target corners")
    targetObservations = []
    numImages = dataset.numImages()

    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()

    if multithreading:
        # ------------------------------------------------------------------
        # BOUNDED-QUEUE REWORK -- read this before touching anything below.
        #
        # Original failure mode (upstream kalibr):
        #   taskq = multiprocessing.Manager().Queue()        # UNBOUNDED
        #   for idx, (timestamp, image) in enumerate(dataset.readDataset()):
        #       taskq.put((idx, timestamp, image))            # ALL images, eagerly
        #   # ... only *after* this loop finishes are worker Processes
        #   # created and started.
        #
        #   Every decoded frame is pushed into the task queue before a
        #   single worker exists to drain it, so peak memory is
        #   numImages * frame_size, not numProcesses * frame_size. Worse,
        #   that queue is backed by a multiprocessing.Manager -- a
        #   separate server process that has to hold all of that queued
        #   image data in its own address space.
        #
        #   Measured on real data (5320x3032 mono8 frames, 15.4 MiB each):
        #     -   31 frames -> ~0.5 GiB   : fine
        #     - 2490 frames -> ~37.5 GiB  : the Manager process serving
        #       taskq gets OOM-killed by the kernel once it outgrows the
        #       container's memory (19.6 GiB available here). The parent
        #       then loses its pipe to the manager and sees an EOFError
        #       out of multiprocessing/connection.py's _recv_bytes, which
        #       the original bare `except Exception` turns into:
        #         RuntimeError: Exception during multithreaded extraction: ...
        #   That caps usable dataset size well below the 4000+ frames this
        #   pipeline needs to process.
        #
        # Fix:
        #   1. taskq is now a plain, bounded multiprocessing.Queue (NOT a
        #      Manager queue) with maxsize = a small multiple of
        #      numProcesses. A native multiprocessing.Queue streams data
        #      directly between producer and consumer processes over a
        #      pipe with a bounded buffer, instead of funnelling
        #      everything through a separate manager process that has to
        #      hold it all -- which is exactly the thing that got
        #      OOM-killed above. (resultq/progressq stay on a Manager,
        #      same as upstream, since they only ever hold small
        #      observation objects / integers, never raw frames.)
        #   2. Worker Processes are started BEFORE any task is fed.
        #   3. The producer (this process) feeds the queue only after the
        #      workers are already running and draining it; taskq.put()
        #      blocks once the queue is full, and that blocking *is* the
        #      backpressure -- it caps frames resident at once at roughly
        #      (maxsize + numProcesses), not numImages.
        #   4. Because the queue is now routinely empty while more work is
        #      still coming (bounded + fed gradually), the old
        #      termination signal -- queue.Empty on get_nowait() -- no
        #      longer means "no more work", only "producer hasn't caught
        #      up yet". Workers now stop only on an explicit per-worker
        #      sentinel (_WORKER_STOP_SENTINEL) pushed by the producer
        #      after every real task has been queued, with a bounded idle
        #      timeout (_WORKER_IDLE_TIMEOUT_SEC) as a safety net so a
        #      worker can never hang forever if the producer dies first
        #      without sending one.
        #   5. Progress reporting no longer reads taskq.qsize() (which
        #      would now just bounce around inside [0, maxsize] and tell
        #      you nothing about overall completion); instead each worker
        #      reports one completion -- success or failure -- per task
        #      via progressq, and the parent sums those.
        #
        # Semantics preserved exactly: the return value is still a
        # sequence of observations ordered by original dataset index,
        # built by putting (obs, idx) on resultq only for successful
        # detections and sorting by idx at the end. clearImages and
        # noTransformation are still honoured per task. The function
        # signature is unchanged, and the single-threaded else: branch
        # below is untouched.
        # ------------------------------------------------------------------
        if not numProcesses:
            numProcesses = max(1,multiprocessing.cpu_count()-1)
        try:
            manager = multiprocessing.Manager()
            resultq = manager.Queue()
            progressq = manager.Queue()

            # Bounded, native (non-Manager) task queue: at most a small,
            # fixed multiple of numProcesses frames are ever resident in
            # it at once, regardless of numImages.
            taskq_maxsize = max(1, numProcesses * 4)
            taskq = multiprocessing.Queue(maxsize=taskq_maxsize)

            # Start workers FIRST: they must already be draining taskq by
            # the time the producer loop below starts feeding it.
            plist=list()
            for pidx in range(0, numProcesses):
                detector_copy = copy.copy(detector)
                p = multiprocessing.Process(target=multicoreExtractionWrapper, args=(detector_copy, taskq, resultq, progressq, clearImages, noTransformation, ))
                p.start()
                plist.append(p)

            # Feed the queue now that workers are running. put() blocks
            # once taskq is full -- that blocking is the backpressure
            # that bounds memory to a handful of frames per worker.
            for idx, (timestamp, image) in enumerate(dataset.readDataset()):
                _putWithLivenessCheck(taskq, (idx, timestamp, image), plist)

            # One stop sentinel per worker so each exits exactly once,
            # strictly after it has drained every real task queued ahead
            # of its sentinel.
            for _ in range(numProcesses):
                _putWithLivenessCheck(taskq, _WORKER_STOP_SENTINEL, plist)

            #wait for results, deriving progress from worker completions
            #(progressq) instead of the now-meaningless taskq size
            done=0
            last_done=0
            while 1:
                try:
                    while True:
                        progressq.get_nowait()
                        done += 1
                except queue.Empty:
                    pass
                if (done-last_done) > 0:
                    iProgress.sample(done-last_done)
                    last_done = done
                sys.stdout.flush()
                if all([not p.is_alive() for p in plist]):
                    time.sleep(0.1)
                    break
                time.sleep(0.5)

            # Drain any last completion markers that landed between the
            # final progressq poll above and the workers actually exiting.
            try:
                while True:
                    progressq.get_nowait()
                    done += 1
            except queue.Empty:
                pass
            if (done-last_done) > 0:
                iProgress.sample(done-last_done)
                last_done = done

            for p in plist:
                p.join()

            resultq.put('STOP')
        except Exception as e:
            raise RuntimeError("Exception during multithreaded extraction: {0}".format(e))

        #get result sorted by time (=idx)
        if resultq.qsize() > 1:
            targetObservations = [[]]*(resultq.qsize()-1)
            for lidx, data in enumerate(iter(resultq.get, 'STOP')):
                obs=data[0]; time_idx = data[1]
                targetObservations[lidx] = (time_idx, obs)
            targetObservations = list(zip(*sorted(targetObservations, key=lambda tup: tup[0])))[1]
        else:
            targetObservations=[]

    #single threaded implementation
    else:
        for timestamp, image in dataset.readDataset():
            if noTransformation:
                success, observation = detector.findTargetNoTransformation(timestamp, np.array(image))
            else:
                success, observation = detector.findTarget(timestamp, np.array(image))
            if clearImages:
                observation.clearImage()
            if success == 1:
                targetObservations.append(observation)
            iProgress.sample()

    if len(targetObservations) == 0:
        print("\r")
        sm.logFatal("No corners could be extracted for camera {0}! Check the calibration target configuration and dataset.".format(dataset.topic))
    else:
        print("\r  Extracted corners for %d images (of %d images)                              " % (len(targetObservations), numImages))

    #close all opencv windows that might be open
    cv2.destroyAllWindows()

    return targetObservations
