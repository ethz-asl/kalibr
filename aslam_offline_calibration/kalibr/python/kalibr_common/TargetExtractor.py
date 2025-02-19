import sm
import os
import numpy as np
import multiprocessing
try:
   import queue
except ImportError:
   import Queue as queue # python 2.x
import time
import copy
import cv2

def multicoreExtractionWrapper(args):
    # Unpack arguments
    detector, timestamp, image, clearImages, noTransformation = args

    # Perform detection
    np_image = np.array(image)
    if noTransformation:
        success, obs = detector.findTargetNoTransformation(timestamp, np_image)
    else:
        success, obs = detector.findTarget(timestamp, np_image)

    if clearImages:
        obs.clearImage()

    return obs if success else None

def extractCornersFromDataset(dataset, detector, multithreading=False, numProcesses=None, clearImages=True, noTransformation=False):
    print("Extracting calibration target corners")
    targetObservations = []
    numImages = dataset.numImages()

    # prepare progess bar
    iProgress = sm.Progress2(numImages)
    iProgress.sample()

    if multithreading:
        if not numProcesses:
            # Get available CPU count. Prefer `os.sched_getaffinity` if it's available as that
            # works when the CPUs avaialble to the process are restricted:
            # https://stackoverflow.com/a/55423170
            if "sched_getaffinity" in dir(os):
                numProcesses = len(os.sched_getaffinity(0))
            else:
                numProcesses = multiprocessing.cpu_count()

            # Leave one CPU core free
            numProcesses = max(1, numProcesses - 1)

        # Create a pool of worker processes
        with multiprocessing.Pool(processes=numProcesses) as pool:
            # Build a generator of tasks to avoid loading everything in memory at once
            tasks = (
                (copy.copy(detector), ts, img, clearImages, noTransformation)
                for ts, img in dataset.readDataset()
            )

            # Use imap to lazily (one by one) exccute the tasks in the process pool
            results_iter = pool.imap(multicoreExtractionWrapper, tasks)

            # Get results as they finish. imap returns results in the order the tasks
            # are submitted. That's the same order as the timestamp
            targetObservations = []
            done_count = 0
            for obs in results_iter:
                if obs is not None:
                    targetObservations.append(obs)
                done_count += 1
                iProgress.sample()

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
