import roslib
roslib.load_manifest('aslam_cv_python')
import aslam_cv as acv
import sm
import cv2
import pylab as pl
import numpy as np

# This function builds a big image with all images in imlist side-by-side
def buildSuperImage( imlist ):
    if type(imlist) == acv.MultiFrame:
        imlist = [ imlist.getFrame(i).image() for i in range(0,imlist.numCameras()) ]
    height = max( [I.shape[0] for I in imlist ] )
    width = sum( [I.shape[1] for I in imlist ] )
    SI = np.zeros([height,width])
    woff = 0
    for I in imlist:
        SI[0:I.shape[0],woff:(woff + I.shape[1])] = I;
        woff = woff + I.shape[1];
    return SI

# this function plots two image lists right above one another
def plotImages( imlist, MF, fno, name):
    raw = buildSuperImage(imlist)
    processed = buildSuperImage(MF)
    f = pl.figure(fno)
    f.clf()
    pl.subplot(211)
    pl.imshow(raw,cmap=pl.cm.gray)
    pl.title('Raw images')
    pl.axis('off')

    pl.subplot(212)
    pl.imshow(processed,cmap=pl.cm.gray)
    pl.title('Processed images {0}'.format(name))
    pl.axis('off')

    pl.tight_layout()
    f.show()


##################################################
# Here is where the real processing starts

# This is the config file we will load
configFile = "pipeline.info"

# Create a property tree object
config = sm.BoostPropertyTree()
# and parse the file
config.loadInfo(configFile);

# For comparison, we will create two undistortion pipelines
# This one does not undistort
nullProcessor = acv.NCameraPipeline( sm.PropertyTree(config,'NullPipeline') )
# this one does
undistortingProcessor = acv.NCameraPipeline( sm.PropertyTree( config, 'UndistortingPipeline') )

# load some sample images
left  = cv2.imread('im_left.jpg',  cv2.CV_LOAD_IMAGE_GRAYSCALE)
front = cv2.imread('im_front.jpg', cv2.CV_LOAD_IMAGE_GRAYSCALE)
right = cv2.imread('im_right.jpg', cv2.CV_LOAD_IMAGE_GRAYSCALE)
rear  = cv2.imread('im_rear.jpg',  cv2.CV_LOAD_IMAGE_GRAYSCALE)

IM = [front,left,rear,right]

# get a fake timestamp.
stamp = acv.Time.now()

# process the images. A real multiframe should only be returned after all images are added.
MF1 = nullProcessor.addImage( stamp, 0, front )
if not MF1 is None:
    print "Expected none!"
MF1 = nullProcessor.addImage( stamp, 1, left )
if not MF1 is None:
    print "Expected none!"
MF1 = nullProcessor.addImage( stamp, 2, rear )
if not MF1 is None:
    print "Expected none!"
MF1 = nullProcessor.addImage( stamp, 3, right )
if MF1 is None:
    print "Expected a completed multiframe!"


# Do the same for the undistorting processor
MF2 = undistortingProcessor.addImage( stamp, 0, front )
if not MF2 is None:
    print "Expected none!"
MF2 = undistortingProcessor.addImage( stamp, 1, left )
if not MF2 is None:
    print "Expected none!"
MF2 = undistortingProcessor.addImage( stamp, 2, rear )
if not MF2 is None:
    print "Expected none!"
MF2 = undistortingProcessor.addImage( stamp, 3, right )
if MF2 is None:
    print "Expected a completed multiframe!"


# Now we have a multiframe from each processor.
# Plot the images vs. the raw images to see what they look like.
# Figure 1 is from the Null undistorters
plotImages( IM, MF1, 1, "Null Undistorter" )
# Figure 2 is from the undistorters.
plotImages( IM, MF2, 2, "Omni Undistorter")

# Note that the underlying camera geometry objects are different and
# they have different numbers of features
print "\nFrom the null undistorter:"
for i in range(0,MF1.numCameras()):
    print "MF1, camera {0} has geometry {1} and {2} features".format(i, type(MF1.getFrame(i).geometryBase()), MF1.getFrame(i).numKeypoints())
print "\nFrom the undistorter:"
for i in range(0,MF2.numCameras()):
    print "MF2, camera {0} has geometry {1} and {2} features".format(i, type(MF2.getFrame(i).geometryBase()), MF2.getFrame(i).numKeypoints())
