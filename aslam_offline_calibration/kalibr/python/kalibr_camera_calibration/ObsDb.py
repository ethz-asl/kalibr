import sm
import aslam_backend as aopt

import numpy as np
import collections
import igraph
import itertools
import sys

#simple data structure that stores all observations for a multi-cam system
#and can approx. sync observations
#
#target view table in the form of:
#-----------------------------------------------------------------
#|           |         cam0        |  ...  |         camN        |
#| timestamp |---------------------|-------|---------------------|
#|           | obs_id | corner_ids |       | obs_id | corner_ids |
#-----------------------------------------------------------------
#|   float   |  int   | list(int)  |       |  int   | list(int)  |
#-----------------------------------------------------------------
#
class ObservationDatabase(object):
    def __init__(self, max_delta_approxsync=0.0):
        #approximate sync 
        self.max_delta_approxsync = max_delta_approxsync
        #storage for raw observations 
        self.observations = dict()
        #table that connects observations to time (approx sync. work in here...)
        self.targetViews = collections.OrderedDict()
        
    def addObservation(self, cam_id, obs):        
        #create camera list (initialization)
        if cam_id not in self.observations:
            self.observations[cam_id] = list()
            
        #add to archive
        self.observations[cam_id].append(obs)
        obs_idx = len(self.observations[cam_id])-1
    
        #find the nearest timestamp in the table
        timestamps_table = self.targetViews.keys()
        timestamp_obs = obs.time().toSec()
        
        #check if the table is still empty (initialization)
        if not timestamps_table:
            #force a new entry (by putting the "nearest" timestamp more than max_delta_approxsync away)
            nearest_timestamp = timestamp_obs + 5*(self.max_delta_approxsync+1)
        else:
            nearest_timestamp = min(timestamps_table, key=lambda x: abs(x-timestamp_obs))    
            
        #if +-max approx. sync add to this time instant otherwise create a new timestamp)
        if abs(nearest_timestamp-timestamp_obs) <= self.max_delta_approxsync:
            #add to existing timestamp
            timestamp = nearest_timestamp
        else:
            #add new timestamp
            timestamp = timestamp_obs
            self.targetViews[ timestamp ] = dict()
        
        #fill in observation data
        if cam_id not in self.targetViews[timestamp]:
            #create entry if it doesnt exists           
            self.targetViews[timestamp][cam_id] = dict()
            self.targetViews[timestamp][cam_id]['obs_id'] = obs_idx
            self.targetViews[timestamp][cam_id]['observed_corners'] = set(obs.getCornersIdx())
        else:
            #we already have a view from this camera on this timestamp --> STH IS WRONG
            sm.logError("[TargetViewTable]: Tried to add second view to a given cameraId & " 
                        "timestamp. Maybe try to reduce the approximate syncing tolerance..")


#############################################################
## data queries
#############################################################    
    #returns a list of tuples (cam_id, obs) for all observations at a given time
    def getAllObsAtTimestamp(self, timestamp):
        observations_at_time=[]
        for cam_id in self.getCamIdsAtTimestamp(timestamp):
            obs_id = self.targetViews[timestamp][cam_id]['obs_id']
            obs = self.observations[cam_id][obs_id]
            observations_at_time.append( (cam_id, obs) )
        return observations_at_time

    #get the number of cameras that see at least one target
    def numCameras(self):
        return len(self.observations.keys())

    #get a list of all timestamps at which we see a target
    def getAllViewTimestamps(self):
        return self.targetViews.keys()
    
    #get a list of all cam_ids that see the target at the given time
    def getCamIdsAtTimestamp(self, timestamp):
        return self.targetViews[timestamp].keys()
    
    #get the observation for given cam_id and timestamp
    def getObservationAtTime(self, timestamp, cam_id):
        obs_id = self.getObsIdForCamAtTime(timestamp, cam_id)
        return self.observations[cam_id][obs_id]
    
    #get the obs_id for a camera at a given time
    def getObsIdForCamAtTime(self, timestamp, cam_id):
        return self.targetViews[timestamp][cam_id]['obs_id']
    
    #get a list of the IDs of all observed corners for a camera at a given time
    def getCornerIdsAtTime(self, timestamp, cam_id):
        return self.targetViews[timestamp][cam_id]['observed_corners']
    
    #return a list of tuples for all observations of a camera pair
    #       list(tuple) = [ (obsA, obsB), (None, obsB), ...]
    #        None if there is no target for a cam in the view
    def getAllObsTwoCams(self, cam_id_A, cam_id_B):
        tuples = list()
        for timestamp in self.getAllViewTimestamps():
            try:
                obsA = self.getObservationAtTime(timestamp, cam_id_A)
            except KeyError:
                obsA = None
            try:
                obsB = self.getObservationAtTime(timestamp, cam_id_B)
            except KeyError:
                obsB = None
                
            if (obsA is not None) or (obsB is not None):
                tuples.append( (obsA, obsB) )
        return tuples

    #return a list of all observations of a pair
    def getAllObsCam(self, cam_id):
        observations = list()
        for timestamp in self.getAllViewTimestamps():
            try:
                obs = self.getObservationAtTime(timestamp, cam_id)
            except KeyError:
                obs = None

            if obs is not None:
                observations.append(obs)
        return observations
    
#############################################################
## data queries
#############################################################    
    def printTable(self):
        #header
        print "timestamp \t",        
        for cam_id in range(0, self.numCameras()):
            print "cam{0} \t".format(cam_id),
        print
        
        #sort for time
        times_sorted = np.sort(self.targetViews.keys())
        
        #data lines
        for time in times_sorted:
            print time,
            for cam_id in range(0, self.numCameras()):
                try:
                    numCorners = len(self.targetViews[time][cam_id]['observed_corners'])
                except KeyError:
                    numCorners = "-"
                print "\t", numCorners,
            print

