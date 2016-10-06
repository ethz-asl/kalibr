import rosbag
import os
import sm
import numpy as np
import pylab as pl
import aslam_cv as acv

class BagScanDatasetReaderIterator(object):
    def __init__(self,dataset,indices=None):
        self.dataset = dataset
        if indices is None:
            self.indices = np.arange(dataset.numMessages())
        else:
            self.indices = indices
        self.iter = self.indices.__iter__()

        self.ranges_iter = None
        #self.intensities_iter = None
        self.angle = 0.0
        self.angle_increment = 0.0
        self.timestamp = acv.Time(0.0)
        self.time_increment = acv.Duration(0.0)

    def __iter__(self):
        return self

    def next(self):

        try:
            distance = self.ranges_iter.next()
            self.angle = self.angle + self.angle_increment
            self.timestamp = self.timestamp + self.time_increment
        except:
            idx = self.iter.next()
            (self.timestamp, self.time_increment, self.angle, self.angle_increment, ranges, intensities) = self.dataset.getMessage(idx)
            self.ranges_iter = ranges.__iter__()
            distance = self.ranges_iter.next()
        return (self.timestamp, self.angle, distance)

class BagScanDatasetReader(object):
    def __init__(self, bagfile, scantopic, bag_from_to=None):
        self.bagfile = bagfile
        self.topic = scantopic
        self.bag = rosbag.Bag(bagfile)
        self.uncompress = None
        if scantopic is None:
            raise RuntimeError("Please pass in a topic name referring to the scan stream in the bag file\n{0}".format(self.bag));

        # Get the message indices
        conx = self.bag._get_connections(topics=scantopic)
        indices = self.bag._get_indexes(conx)
        
        try:
            self.index = indices.next()
        except:
            raise RuntimeError("Could not find topic {0} in {1}.".format(scantopic, self.bagfile))
        
        self.indices = np.arange(len(self.index))
        
        #sort the indices by header.stamp
        self.indices = self.sortByTime(self.indices)
        
        #go through the bag and remove the indices outside the timespan [bag_start_time, bag_end_time]
        if bag_from_to:
            self.indices = self.truncateIndicesFromTime(self.indices, bag_from_to)
                        
    #sort the ros messages by the header time not message time
    def sortByTime(self, indices):
        timestamps=list()
        for idx in self.indices:
            topic, data, stamp = self.bag._read_message(self.index[idx].position)
            timestamp = data.header.stamp.secs*1e9 + data.header.stamp.nsecs
            timestamps.append(timestamp)
        
        sorted_tuples = sorted(zip(timestamps, indices))
        sorted_indices = [tuple_value[1] for tuple_value in sorted_tuples]
        return sorted_indices
    
    def truncateIndicesFromTime(self, indices, bag_from_to):
        #get the timestamps
        timestamps=list()
        for idx in self.indices:
            topic, data, stamp = self.bag._read_message(self.index[idx].position)
            timestamp = data.header.stamp.secs + data.header.stamp.nsecs/1.0e9
            timestamps.append(timestamp)

        bagstart = min(timestamps)
        baglength = max(timestamps)-bagstart
        print "bagstart",bagstart 
        print "baglength",baglength
        #some value checking
        if bag_from_to[0]>=bag_from_to[1]:
            raise RuntimeError("Bag start time must be bigger than end time.".format(bag_from_to[0]))
        if bag_from_to[0]<0.0:
            sm.logWarn("Bag start time of {0} s is smaller 0".format(bag_from_to[0]) )
        if bag_from_to[1]>baglength:
            sm.logWarn("Bag end time of {0} s is bigger than the total length of {1} s".format(bag_from_to[1], baglength) )

        #find the valid timestamps
        valid_indices = []
        for idx, timestamp in enumerate(timestamps):
             if timestamp>=(bagstart+bag_from_to[0]) and timestamp<=(bagstart+bag_from_to[1]):
                 valid_indices.append(idx)  
        sm.logWarn("BagScanDatasetReader: truncated {0} / {1} messages.".format(len(indices)-len(valid_indices), len(indices)))
        
        return valid_indices
    
    def __iter__(self):
        # Reset the bag reading
        return self.readDataset()
    
    def readDataset(self):
        return BagScanDatasetReaderIterator(self, self.indices)

    def readDatasetShuffle(self):
        indices = self.indices
        np.random.shuffle(indices)
        return BagScanDatasetReaderIterator(self,indices)

    def numMessages(self):
        return len(self.indices)
    
    def getMessage(self,idx):
        topic, data, stamp = self.bag._read_message(self.index[idx].position)

        ts = acv.Time(stamp.to_sec())
        return (ts, acv.Duration(data.time_increment), data.angle_min, data.angle_increment, data.ranges, data.intensities)


class BagTriggeredScanDatasetReader(BagScanDatasetReader):
    def __init__(self, bagfile, scantopic, triggertopic, triggerid, indexoffset = 0, bag_from_to=None):
        super(self.__class__, self).__init__(bagfile, scantopic, bag_from_to)
        
        #associate trigger and scan with an offset from closest in time
        self.index_offset = indexoffset
        
        #Read trigger timestamps for subsequent synchronization with scan messages.
        triggerTimestampCorrector = sm.DoubleTimestampCorrector()
        remote_timestamps = []
        for (topic, data, stamp) in rosbag.Bag(bagfile).read_messages(topics={triggertopic}):
            if data.trigger_id == triggerid or triggerid == -1:
                triggerTimestampCorrector.correctTimestamp(data.header.stamp.to_sec(), stamp.to_sec())
                remote_timestamps.append(acv.Time(data.header.stamp.secs, data.header.stamp.nsecs))
        
        self.trigger_timestamps = [(acv.Time(triggerTimestampCorrector.getLocalTime(r.toSec())), r) \
                                   for r in remote_timestamps]

        self.rangeTimestampCorrector = sm.DoubleTimestampCorrector()
        for (topic, data, stamp) in rosbag.Bag(bagfile).read_messages(topics=scantopic):
            self.rangeTimestampCorrector.correctTimestamp(data.header.stamp.to_sec(), stamp.to_sec())

    def findClosestTimestamps(self, timestamps, query_timestamps):
        indices = np.abs(np.subtract.outer(timestamps, query_timestamps)).argmin(0)
        return indices
        
    def getMessage(self,idx):
        topic, data, stamp = self.bag._read_message(self.index[idx].position)
        
        host_timestamps, device_timestamps = zip(*self.trigger_timestamps)
        #TODO(jrn) read number of measurements instead of hardcoding it!
        measurements_per_scan = 1440.0
        
        time_increment = np.diff(device_timestamps) / measurements_per_scan
        
        #pad the time increments for same length as scans.
        time_increment = np.append(time_increment, time_increment[-1])
        
        index = self.findClosestTimestamps([t.toSec() for t in host_timestamps], \
                                           self.rangeTimestampCorrector.getLocalTime(data.header.stamp.to_sec()))
        
        #TODO(jrn) if the offset != 0, this will return at least one incorrect timestamp. Fix this! 
        if index + self.index_offset > 0 and index + self.index_offset < len(device_timestamps):
            index += self.index_offset
        
        return (device_timestamps[index], time_increment[index], \
                data.angle_min, data.angle_increment, data.ranges, data.intensities)